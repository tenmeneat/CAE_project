function [v, vlim] = capped_speed_profile(kappa, theta, ds, p)
%CAPPED_SPEED_PROFILE 시나리오 B: 가속도 상한 캡 속도 프로파일 v(s) (블록 1, D17).
%   [v, vlim] = CAPPED_SPEED_PROFILE(kappa, theta, ds, p)
%   |ay| = v²|κ| ≤ p.ay_cap,  |dv/dt| ≤ p.ax_cap  (각각 독립 상한 — 마찰원이 아니다. 한계 주행이 아니라 승차감 조건).
%   코너 한계 v_lim = min(vmax, sqrt(ay_cap/|κ|)) → 후진 적분(감속 ax_cap) + 전진 적분(가속 ≤ min(ax_cap, 파워·항력·경사 여유)).
%   마찰 한계(1.1 g)는 상한(0.4/0.3 g)보다 훨씬 커서 쓰지 않는다. 가속은 고속에서 파워가 모자라면 상한보다 작아진다.

N = numel(kappa);
vlim = min(p.vmax, sqrt(p.ay_cap ./ max(abs(kappa), p.ay_cap / p.vmax^2)));

[~, i0] = min(vlim);
idx = mod(i0 - 1 + (0:N-1)', N) + 1;            % v_lim 최소점이 1번이 되게 회전
th = theta(idx);  vl = vlim(idx);

% 전진: 가속 = min(상한, 파워 여유 − 항력 − 경사)
vf = vl;
for i = 1:N-1
    v = vf(i);
    a = min(p.ax_cap, p.P * p.eta / (max(v, 1) * p.m) - 0.5 * p.rho * p.CdA * v^2 / p.m - p.g * sin(th(i)));
    vf(i+1) = min(vl(i+1), sqrt(max(v^2 + 2 * ds * a, 0)));
end
% 후진: 감속 상한 ax_cap (루프를 닫는 1번에서 출발)
vb = vl;
for i = N:-1:2
    vb(i) = min(vl(i), sqrt(vb(mod(i, N) + 1)^2 + 2 * ds * p.ax_cap));
end

v = zeros(N, 1);
v(idx) = min(vf, vb);
end
