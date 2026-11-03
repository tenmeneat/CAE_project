function res = validate_class_laptime(target_s, mu_range)
%VALIDATE_CLASS_LAPTIME 차급 검증 (W2 게이트, 03-결정사항 "차급 검증 기준"): Lapmeta 2:43.2를 내는 μ를 역산해
%   R컴파운드 범위 안인지 판정한다. μ를 맞춰 넣는 게 아니라, 순정 M4 파라미터로 그 랩타임에 필요한 μ가 그럴듯한지 본다.
%   res = VALIDATE_CLASS_LAPTIME(target_s, mu_range)
%   target_s: 기본 163.2 s (F82 M4 + Hoosier R7, https://lapmeta.com/en/vi/3145)
%   mu_range: 통과 범위, 기본 [1.2 1.5] — [가정] R컴파운드 μ 공개값이 없어 출처 없는 추정 (썸머 타이어 μ 1.10에
%             비공식 비교 "+0.1–0.2 g"를 더한 수준). 승인·문헌 보강 필요 (D3).
%   주 판정은 순정 파라미터(f82_params). 기록 차량이 밝힌 456 hp·1,515 kg(+드라이버 75 kg 가정) 값은 민감도로만 보고.
%   역산 중에는 마찰원(μx = μ)을 유지한다.

if nargin < 1 || isempty(target_s), target_s = 163.2; end
if nargin < 2 || isempty(mu_range), mu_range = [1.2 1.5]; end
here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..', '..');
trk = load_raceline(fullfile(root, 'data', 'tracks', 'tumftm', 'racelines', 'Spa.csv'), 1);
[~, theta, kv] = align_elevation(trk, fullfile(root, 'data', 'raw', 'fastf1_spa.csv'));
kappa = compute_curvature(trk.x, trk.y, [], trk.ds, 10);

p = f82_params();
res.mu = solve_mu(p);
q = p;  q.P = 456 * 745.7;  q.m = 1515 + 75;        % 기록 차량 신고값 (hp → W), 드라이버 75 kg는 가정
res.mu_record_car = solve_mu(q);
res.target = target_s;  res.range = mu_range;
res.pass = res.mu >= mu_range(1) && res.mu <= mu_range(2);

fprintf('== 차급 검증 (Spa, Lapmeta %.1f s) ==\n', target_s);
fprintf('순정 M4 파라미터(317 kW, 1642 kg): 필요한 μ = %.3f → %s (R컴파운드 범위 [%.2f %.2f], 가정)\n', ...
    res.mu, ternary(res.pass, 'PASS', 'FAIL'), mu_range);
fprintf('민감도 — 기록 차량 신고값(340 kW, 1590 kg): 필요한 μ = %.3f\n', res.mu_record_car);
fprintf('참고: 순정 썸머 타이어 μ %.2f (시나리오 A 입력)\n', p.mu);

    function mu = solve_mu(pp)
        % 랩타임은 μ에 대해 단조 감소 → 이분법
        lo = 0.8;  hi = 2.5;
        for it = 1:40
            mu = (lo + hi) / 2;
            pp.mu = mu;  pp.mu_x = mu;
            if sum(trk.ds ./ quasi_steady_speed_profile(kappa, theta, kv, trk.ds, pp)) > target_s
                lo = mu;
            else
                hi = mu;
            end
            if hi - lo < 1e-4, break; end
        end
    end
end

function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end
