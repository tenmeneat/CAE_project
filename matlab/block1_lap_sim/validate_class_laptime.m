function res = validate_class_laptime(target_s, mu_range)
%VALIDATE_CLASS_LAPTIME 차급 검증 (W2 게이트, 03-결정사항 "차급 검증 기준"): Lapmeta 2:43.2를 내는 μ를 역산해
%   R컴파운드 범위 안인지 판정한다. μ를 맞춰 넣는 게 아니라, 순정 M4 파라미터로 그 랩타임에 필요한 μ가 그럴듯한지 본다.
%   res = VALIDATE_CLASS_LAPTIME(target_s, mu_range)
%   target_s: 기본 163.2 s (F82 M4 + Hoosier R7, https://lapmeta.com/en/vi/3145)
%   mu_range: 통과 범위, 기본 [1.11 1.25] = 순정 썸머 μ 1.10 × (1.01–1.14). 같은 차 타이어 비교 두 건에서 유도
%             (블록1-랩시뮬레이션.md "차급 검증 판정 범위의 근거"):
%             - Tire Rack 2025 Test 4 (BMW M2, 275/35R18, 3.2 km 서킷): R컴파운드(A055·A7)가 익스트림 썸머보다
%               평균 랩 0.5–3.1% 빠름 → 이 모델의 랩타임-μ 민감도 0.35로 나누면 μ +1.4–9.0%
%             - Car and Driver 2012 (BMW 328i): 익스트림 썸머끼리 마른 스키드패드 0.87–0.91 g (+0–4.6%) →
%               순정급 썸머와 익스트림 썸머 차이의 상한으로 사용
%             범위는 넓어 약한 검증이고, μ 환산에 모델 자신의 민감도를 빌렸다. R7 자체 데이터는 없다.
%   주 판정은 순정 파라미터(f82_params). 기록 차량이 밝힌 456 hp·1,515 kg(+드라이버 75 kg 가정) 값은 민감도로만 보고.
%   역산 중에는 마찰원(μx = μ)을 유지한다.

if nargin < 1 || isempty(target_s), target_s = 163.2; end
if nargin < 2 || isempty(mu_range), mu_range = [1.11 1.25]; end
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
fprintf('순정 M4 파라미터(317 kW, 1642 kg): 필요한 μ = %.3f → %s (R컴파운드 범위 [%.2f %.2f])\n', ...
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
