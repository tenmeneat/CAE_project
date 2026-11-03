function [kappa, theta, kv] = compute_curvature(x, y, z, ds, win)
%COMPUTE_CURVATURE 닫힌 루프의 수평 곡률 κ(s), 경사각 θ(s), 수직 곡률 κv(s) (블록 1).
%   kappa = COMPUTE_CURVATURE(x, y, [], ds, win)
%   [kappa, theta, kv] = COMPUTE_CURVATURE(x, y, z, ds, win)
%   x,y,z: Δs=ds 균일 격자 (주기). win: 곡률 평활 창 [m], 기본 10 (0이면 평활 안 함).
%   부호: κ>0 좌회전 (ISO 8855, y 좌측). κv>0 은 노면이 위로 볼록한 쪽이 아니라 d²z/ds² 부호 그대로
%   (오목=+, 크레스트=−). 평활은 위치가 아니라 곡률에 적용 (위치 평활은 코너를 깎는다).

if nargin < 5, win = 10; end
d1 = @(v) (circshift(v, -1) - circshift(v, 1)) / (2*ds);
d2 = @(v) (circshift(v, -1) - 2*v + circshift(v, 1)) / ds^2;
sm = @(v) circ_movmean(v, round(win / ds));

kappa = sm((d1(x).*d2(y) - d1(y).*d2(x)) ./ (d1(x).^2 + d1(y).^2).^1.5);
if nargin > 2 && ~isempty(z)
    theta = atan(d1(z));                     % 경사각 [rad], 상향 +
    kv = sm(d2(z));
end
end

function v = circ_movmean(v, w)
if w < 2, return; end
p = ceil(w / 2);
v = movmean([v(end-p+1:end); v; v(1:p)], w);
v = v(p+1:end-p);
end
