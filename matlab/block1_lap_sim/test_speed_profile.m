%TEST_SPEED_PROFILE quasi_steady_speed_profile / lap_sim 시나리오 A 자체 검사 (실패하면 assert 에러).
here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..', '..');
p = f82_params();
N = 3000; ds = 1;

% 1) 직선·평지: 종단속도 = 속도 제한
v = quasi_steady_speed_profile(zeros(N,1), zeros(N,1), zeros(N,1), ds, p);
assert(all(abs(v - p.vmax) < 1e-6), '직선 속도가 vmax가 아님');

% 2) 정상 원선회 R=100: v = sqrt(mu g R) (항력 때문에 한계보다 아주 조금 낮아질 수 있음)
R = 100; N2 = round(2*pi*R / ds); ds2 = 2*pi*R / N2;
[v, vl] = quasi_steady_speed_profile(ones(N2,1)/R, zeros(N2,1), zeros(N2,1), ds2, p);
assert(all(abs(vl - sqrt(p.mu*p.g*R)) < 1e-9), 'v_lim 식 오류');
assert(all(v <= vl + 1e-9) && min(v) > 0.99 * sqrt(p.mu*p.g*R), '원선회 속도 이상');

% 3) 직선+급커브 루프: 감속 거리가 맞는지 (커브 앞에서 v가 v_lim까지 내려오고 위반 없음)
kap = zeros(N,1);  kap(1500:1520) = 1/30;      % R=30 m 코너 하나, 부호는 한 방향 → 루프 의미는 없지만 속도식은 유효
[v, vl] = quasi_steady_speed_profile(kap, zeros(N,1), zeros(N,1), ds, p);
assert(all(v <= vl + 1e-9), 'v > v_lim');
assert(min(v) < 0.99*p.vmax && v(1500) > 0.99*vl(1500) - 0.5, '코너 속도');
[~, ip] = min(v);
a_dec = (v(ip-100)^2 - v(ip-1)^2) / (2*99*ds);  % 코너 직전 평균 감속도 [m/s^2], 마찰한계(~mu*g) 이하여야 함
assert(a_dec > 0 && a_dec < 1.2 * p.mu * p.g, '제동 감속도가 마찰한계 초과');

% 4) Spa 시나리오 A (데이터 있을 때)
rl = fullfile(root, 'data', 'tracks', 'tumftm', 'racelines', 'Spa.csv');
i0 = fullfile(root, 'data', 'raw', 'fastf1_spa.csv');
if isfile(rl) && isfile(i0)
    [t, v, ax, ay, s] = lap_sim(rl, i0, p, 'A');
    lap = t(end) + 2*(s(2)-s(1)) / (v(end) + v(1));
    fprintf('Spa A: 랩타임 %.1f s (%d:%04.1f), v %.0f–%.0f km/h, |ax|max %.2f g, |ay|max %.2f g, 평균 %.0f km/h\n', ...
        lap, floor(lap/60), mod(lap,60), 3.6*min(v), 3.6*max(v), max(abs(ax))/p.g, max(abs(ay))/p.g, 3.6*(s(end)+1)/lap);
    assert(all(isfinite(t)) && issorted(t), 't 오류');
    assert(max(abs(ay)) < 1.25 * p.mu * p.g, '횡가속도가 마찰한계를 크게 초과');
    % 종가속도 상한 = μ·g_n,max (제동) + 공기저항 + 경사(<~0.25g) — 1.84g가 나왔던 압축 구간 과대평가 회귀 방지
    assert(max(abs(ax)) < (p.mu * p.gn_max + 0.25) * p.g, '종가속도가 타이어 한계를 크게 초과');
    assert(lap > 120 && lap < 200, '랩타임이 비현실적');
end
% 5) 시나리오 B: 합성 검사 + Spa 중심선 (상한 준수가 W2 검증 항목)
kap = zeros(N,1);  kap(1500:1560) = 1/60;
[v, vl] = capped_speed_profile(kap, zeros(N,1), ds, p);
assert(all(v <= vl + 1e-9) && abs(min(v) - sqrt(p.ay_cap*60)) < 1e-6, 'B 코너 속도');
assert(abs(v(1500)^2 - v(1500 - 120)^2) / (2*120*ds) <= p.ax_cap + 1e-6, 'B 감속 상한');
cl = fullfile(root, 'data', 'tracks', 'tumftm', 'tracks', 'Spa.csv');
if isfile(cl) && isfile(i0)
    [t, v, ax, ay, s] = lap_sim(cl, i0, p, 'B');
    lapB = t(end) + 2*(s(2)-s(1)) / (v(end) + v(1));
    fprintf('Spa B: 랩타임 %.1f s (%d:%04.1f), v %.0f–%.0f km/h, |ax|max %.3f g, |ay|max %.3f g, 평균 %.0f km/h\n', ...
        lapB, floor(lapB/60), mod(lapB,60), 3.6*min(v), 3.6*max(v), max(abs(ax))/p.g, max(abs(ay))/p.g, 3.6*(s(end)+1)/lapB);
    assert(max(abs(ay)) <= p.ay_cap * 1.001, 'B: |ay| 상한 초과');
    assert(max(abs(ax)) <= p.ax_cap * 1.02, 'B: |ax| 상한 초과');   % 중심차분이라 2% 여유
    assert(lapB > lap, 'B가 A보다 빠름');
end
disp('test_speed_profile OK');
