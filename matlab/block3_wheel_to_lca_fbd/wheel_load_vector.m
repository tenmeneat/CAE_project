function [Fcp, Fwc] = wheel_load_vector(wf, i, vp)
%WHEEL_LOAD_VECTOR 코너 i의 외력을 작용점별로 나눔 (블록 3). 좌측 등가 좌표, N×3 [N].
%   [Fcp, Fwc] = WHEEL_LOAD_VECTOR(wf, i, vp)
%   Fcp (접지점): Fy, Fz, 제동 Fx < 0 — 캘리퍼가 너클에 달려 제동 토크를 너클이 받음
%   Fwc (휠센터): 구동 Fx > 0 — 구동 토크는 하프샤프트가 받음 (F82는 후륜구동)
%                 −m_u·(g + z̈_u) — 비현가 질량 자중 + 달랑베르 관성
%   우측 코너(FR, RR)는 Fy → −Fy.

Fx = wf.Fx(:, i);  Fy = wf.Fy(:, i);  Fz = wf.Fz(:, i);
if mod(i, 2) == 0, Fy = -Fy; end
if isfield(wf, 'zdd_u')
    zdd = wf.zdd_u(:, i);
else
    zdd = zeros(size(Fz));
end
z = zeros(size(Fz));
Fcp = [min(Fx, 0), Fy, Fz];
Fwc = [max(Fx, 0), z, -vp.m_u(i) * (vp.g + zdd)];
end
