function MR = motion_ratio_kin(hc, geo)
%MOTION_RATIO_KIN 기구학 모션비 MR = dL_strut / dz_cp (블록 3, 테스트용).
%   MR = MOTION_RATIO_KIN(hc, geo)
%   속도 q = [v_WC(3); ω(3); θ̇_U; θ̇_L] (너클 6 + 상·하부암 회전 2), 구속 7개
%   (UBJ 3, LBJ 3, 타이로드 길이 1, 조향 입력 없음) → 자유도 1. 구속 야코비안의 영공간이 범프 모드.
%   dL_strut는 스트럿 압축 속도 (u_s 방향 마운트 속도).

rL  = hc.LBJ  - hc.WC;
rU  = hc.UBJ  - hc.WC;
rTO = hc.TR_O - hc.WC;
rCP = hc.CP   - hc.WC;
I = eye(3);  z = zeros(3, 1);

% 너클 위 점 r의 속도 = v + ω×r = v − [r]×·ω
J = [I, -skew(rU), -cross(geo.e_U, hc.UBJ - hc.UCA_F), z
     I, -skew(rL), z, -cross(geo.e_L, hc.LBJ - hc.LCA_F)
     geo.u_T', -geo.u_T' * skew(rTO), 0, 0];
q = null(J);
assert(size(q, 2) == 1, 'motion_ratio_kin:Dof', '기구 자유도가 1이 아님 (%d)', size(q, 2));

v_cp = q(1:3) - skew(rCP) * q(4:6);
v_M  = q(8) * cross(geo.e_L, geo.r_M - hc.LCA_F);
MR = dot(geo.u_s, v_M) / v_cp(3);
end

function S = skew(r)
S = [0 -r(3) r(2); r(3) 0 -r(1); -r(2) r(1) 0];
end
