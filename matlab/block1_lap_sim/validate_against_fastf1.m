function res = validate_against_fastf1(params, ref_lap_s, fig_path)
%VALIDATE_AGAINST_FASTF1 방법론 검증 (W2 게이트): 시나리오 A 프로파일을 I0 실측과 비교한다.
%   res = VALIDATE_AGAINST_FASTF1(params, ref_lap_s, fig_path)
%   params: F1급이면 f1_params() (기본). ref_lap_s: 기준 랩타임 [s], 기본 114.4 = 2022 Spa 정상 랩 613개 평균
%   (최속 109.354 s, 전처리-FastF1-I0.md). fig_path: 비교 그림 저장 경로(생략하면 저장 안 함).
%   비교: 랩타임, 속도 RMSE·상관, 코너 정점 속도·위치, 제동 시작점.
%   I0는 정상 랩 613개 평균이라 정점 속도가 평균으로 무뎌져 있고, 레이스 페이스(연료·타이어·트래픽)라 한계 주행인
%   시뮬이 몇 % 빠른 것이 정상이다 — 기준을 정상 랩 평균과 최속 랩 사이로 본다. I0 brake는 0–1.

here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..', '..');
if nargin < 1 || isempty(params), params = f1_params(); end
if nargin < 2 || isempty(ref_lap_s), ref_lap_s = 114.4; end
best_lap = 109.354;

trk = load_raceline(fullfile(root, 'data', 'tracks', 'tumftm', 'racelines', 'Spa.csv'), 1);
d = readmatrix(fullfile(root, 'data', 'raw', 'fastf1_spa.csv'));          % s,x,y,z,v,throttle,brake
[~, theta, kv, info] = align_elevation(trk, fullfile(root, 'data', 'raw', 'fastf1_spa.csv'));
kappa = compute_curvature(trk.x, trk.y, [], trk.ds, 10);
v = quasi_steady_speed_profile(kappa, theta, kv, trk.ds, params);

% I0 → 트랙 s 격자 (둘레 비율 + 곡률 상호상관 오프셋)
N = numel(trk.s);  u = (0:N-1)' / N;  L0 = d(end,1) + (d(2,1) - d(1,1));
uf = mod(u - info.shift_m / trk.L, 1);
at = @(c) interp1([d(:,1) / L0; 1], [d(:,c); d(1,c)], uf, 'linear');
vref = at(5);  bref = at(7);

ds = trk.ds;  g = params.g;
lap = sum(ds ./ v);
res.lap_sim = lap;  res.lap_ref = ref_lap_s;  res.lap_best = best_lap;
res.lap_err = lap / ref_lap_s - 1;
e = v - vref;
res.rmse_kmh = 3.6 * sqrt(mean(e.^2));
res.rmse_rel = sqrt(mean(e.^2)) / mean(vref);
cc = corrcoef(v, vref);  res.corr = cc(1,2);

% 코너 정점: 평활한 실측의 지역 최소 (prominence 15 km/h 이상)
vs = movmean(vref, 21);
ia = find(islocalmin(vs, 'MinProminence', 15/3.6));
ax = (circshift(v, -1).^2 - circshift(v, 1).^2) / (4*ds) / g;     % 진행방향 가속도 [g]
w = 60;                                                           % 정점 탐색 반경 [m]
for k = 1:numel(ia)
    j = mod(ia(k) + (-w:w) - 1, N) + 1;
    [vm, jm] = min(v(j));
    res.apex_s(k,1) = trk.s(ia(k));
    res.apex_dv_kmh(k,1) = 3.6 * (vm - vref(ia(k)));
    res.apex_ds_m(k,1) = (jm - w - 1) * ds;
    % 제동 시작점: 정점에서 뒤로 가며 연속 제동 구간(실측 brake>0.5, 즉 절반 넘는 랩이 제동 / 시뮬 ax<-1 g)의 시작
    res.brk_ref(k,1) = onset(bref > 0.5, ia(k), N, ds);
    res.brk_sim(k,1) = onset(ax < -1, mod(ia(k) + jm - w - 2, N) + 1, N, ds);
end
res.brk_err_m = res.brk_sim - res.brk_ref;                        % 양수 = 시뮬이 더 늦게 제동 시작

fprintf('== 방법론 검증 (Spa, 시나리오 A) ==\n');
fprintf('랩타임: 시뮬 %.2f s | 정상 랩 평균 %.2f s (%+.1f%%) | 최속 랩 %.2f s (%+.1f%%)\n', ...
    lap, ref_lap_s, 100*res.lap_err, best_lap, 100*(lap/best_lap - 1));
fprintf('속도 RMSE %.1f km/h (%.1f%%), 상관 %.3f\n', res.rmse_kmh, 100*res.rmse_rel, res.corr);
fprintf('코너 정점 %d개: 속도 오차 중앙값 %+.1f km/h (|.| 평균 %.1f), 위치 오차 |.| 중앙값 %.0f m\n', numel(ia), ...
    median(res.apex_dv_kmh), mean(abs(res.apex_dv_kmh)), median(abs(res.apex_ds_m)));
ok = ~isnan(res.brk_err_m);
fprintf('제동 시작점(%d개 비교): 오차 중앙값 %+.0f m (양수 = 시뮬이 늦음), |.| 중앙값 %.0f m\n', ...
    sum(ok), median(res.brk_err_m(ok)), median(abs(res.brk_err_m(ok))));
tbl = table(res.apex_s, 3.6*vref(ia), 3.6*(vref(ia) + res.apex_dv_kmh/3.6), res.apex_dv_kmh, res.apex_ds_m, res.brk_err_m, ...
    'VariableNames', {'s_m', 'v_ref_kmh', 'v_sim_kmh', 'dv_kmh', 'ds_m', 'brk_err_m'});
disp(round(tbl, 0));
res.table = tbl;

% 게이트 판정 — 기준 숫자는 문서에 없어 첫 결과(2026-11-03)를 본 뒤 정했다 (사용자 승인). 한계 주행 시뮬 대 레이스
% 페이스 평균이라 여유를 둔다. 정점 속도는 I0가 랩 평균이라 정점이 무뎌져 시뮬에 불리한 편향이 있어 참고 지표로만 둔다.
gate = {'랩타임 오차 ±5% (정상 랩 평균 대비)',       abs(res.lap_err) <= 0.05;
        '속도 RMSE ≤ 8%',                             res.rmse_rel <= 0.08;
        '속도 상관 ≥ 0.95',                           res.corr >= 0.95;
        '정점 위치 오차 |중앙값| ≤ 30 m',             median(abs(res.apex_ds_m)) <= 30;
        '제동 시작점 오차 |중앙값| ≤ 50 m',           median(abs(res.brk_err_m(ok))) <= 50};
res.gate = cell2table(gate, 'VariableNames', {'criterion', 'pass'});
res.pass = all([gate{:,2}]);
for k = 1:size(gate, 1), fprintf('  [%s] %s\n', ternary(gate{k,2}, 'PASS', 'FAIL'), gate{k,1}); end
fprintf('  [참고] 정점 속도 오차 중앙값 %+.1f km/h (판정 제외)\n', median(res.apex_dv_kmh));
fprintf('방법론 검증: %s\n', ternary(res.pass, 'PASS', 'FAIL'));

if nargin >= 3 && ~isempty(fig_path)
    f = figure('Visible', 'off', 'Position', [100 100 1400 650]);
    subplot(2,1,1);  plot(trk.s, 3.6*vref, 'k', trk.s, 3.6*v, 'r', 'LineWidth', 1);  hold on;
    plot(trk.s(ia), 3.6*vref(ia), 'bv');  ylabel('v [km/h]');  grid on;
    legend('I0 실측(정상 랩 평균)', '시뮬 A', '정점');  title(sprintf('Spa: 시뮬 %.1f s / 정상랩 평균 %.1f s / 최속랩 %.1f s', lap, ref_lap_s, best_lap));
    subplot(2,1,2);  plot(trk.s, 100*bref, 'k', trk.s, min(max(-ax, 0), 6) * 100/6, 'r');  ylabel('brake % / 시뮬 감속 (6 g = 100)');
    xlabel('s [m]');  grid on;
    exportgraphics(f, fig_path, 'Resolution', 110);  close(f);
end
end

function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end

function s0 = onset(flag, i_apex, N, ds)
%ONSET i_apex 에서 뒤로 가며 flag 가 연속으로 참인 구간의 시작까지의 거리 [m] (정점 기준 음수가 아니라 정점 앞 거리). 없으면 NaN.
s0 = NaN;
n = 0;
while n < 600 && ~flag(mod(i_apex - n - 1, N) + 1), n = n + 1; end     % 정점 직전의 제동 구간까지 거슬러 감
if n >= 600, return; end
while n < 900 && flag(mod(i_apex - n - 1, N) + 1), n = n + 1; end
s0 = -n * ds;                                                           % 정점 대비 상대 위치 [m]
end
