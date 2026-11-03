function [v, vlim] = quasi_steady_speed_profile(kappa, theta, kv, ds, p)
%QUASI_STEADY_SPEED_PROFILE 시나리오 A: 마찰원 한계 준정상 속도 프로파일 v(s) (블록 1, 4.1절).
%   [v, vlim] = QUASI_STEADY_SPEED_PROFILE(kappa, theta, kv, ds, p)
%   kappa, theta, kv: 닫힌 루프 Δs=ds 격자 (compute_curvature 출력). p: f82_params().
%   점질량 + RWD 트랙션 한계(종하중이동) + 공기저항 + 경사 + 수직 곡률에 의한 수직항력 변화.
%     법선 가속도  g_n(v) = clamp(g·cosθ + v²·(κv + ρ·CzA/2m), 0.05g, gn_max·g)   (상한은 하중 민감도 대용, f82_params)
%     코너 한계    v_lim² = μ·g_n / |κ|,  상한 전에는 μ·g·cosθ / (|κ| − μ·κv)  (분모는 vmax 바닥 처리)
%     마찰 타원    종방향 한계(μx·g_n 기반) × sqrt(1 − (ay/ay_max)²),  ay = v²|κ|, ay_max = μ·g_n
%                  μ = 횡방향, μx = 종방향(p.mu_x, 없으면 μ와 같음 → 마찰원)
%   전진(파워·트랙션) + 후진(제동) 적분 후 min. 루프라서 v_lim 최소점에서 출발해 한 바퀴 돈다.

N = numel(kappa);
mux = p.mu;  if isfield(p, 'mu_x'), mux = p.mu_x; end
kv = kv + 0.5 * p.rho * p.CzA / p.m;            % 다운포스도 v²에 비례하는 법선 하중 → κv와 같은 항으로 합친다 (축 배분 무시)
ak = abs(kappa);
gn0 = p.g * cos(theta);
gmax = p.gn_max * p.g;
den = max(ak - p.mu * kv, p.mu * gn0 / p.vmax^2);
va = sqrt(p.mu * gmax ./ max(ak, p.mu * gmax / p.vmax^2));    % 법선 가속도가 상한에 걸린 경우의 한계
vlim = min(sqrt(p.mu * gn0 ./ den), va);

[~, i0] = min(vlim);
idx = mod(i0 - 1 + (0:N-1)', N) + 1;            % v_lim 최소점이 1번이 되게 회전
k = ak(idx);  th = theta(idx);  kvv = kv(idx);  gn = gn0(idx);  vl = vlim(idx);

drag  = @(v) 0.5 * p.rho * p.CdA * v^2 / p.m;   % 공기저항 가속도 [m/s^2]
gnv   = @(v, i) min(max(gn(i) + v^2 * kvv(i), 0.05 * p.g), gmax);   % 법선 가속도
circ  = @(v, i) sqrt(max(1 - (v^2 * k(i) / (p.mu * gnv(v, i)))^2, 0));   % 마찰 타원 종방향 잔여 비율
a_tr  = @(v, i) mux * gnv(v, i) * p.w_rear / (1 - mux * p.h_cg / p.L);   % RWD 트랙션 한계 [m/s^2] (종하중이동 포함)
a_fwd = @(v, i) min(p.P * p.eta / (max(v, 1) * p.m), a_tr(v, i) * circ(v, i)) - drag(v) - p.g * sin(th(i));
a_brk = @(v, i) mux * gnv(v, i) * circ(v, i) + drag(v) + p.g * sin(th(i));

% 전진: i → i+1 (Heun 보정: 끝점 속도로 한 번 더 평가)
vf = vl;
for i = 1:N-1
    v = vf(i);  a1 = a_fwd(v, i);
    a2 = a_fwd(sqrt(max(v^2 + 2*ds*a1, 0)), i);
    vf(i+1) = min(vl(i+1), sqrt(max(v^2 + ds*(a1 + a2), 0)));
end
% 후진: i+1 → i (끝점 N+1 은 루프를 닫는 1번)
vb = vl;
for i = N:-1:2
    v = vb(mod(i, N) + 1);  a1 = a_brk(v, i);
    a2 = a_brk(sqrt(v^2 + 2*ds*a1), i);
    vb(i) = min(vl(i), sqrt(v^2 + ds*(a1 + a2)));
end

v = zeros(N, 1);
v(idx) = min(vf, vb);
end
