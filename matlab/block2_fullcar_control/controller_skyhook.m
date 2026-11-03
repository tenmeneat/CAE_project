function Kfb = controller_skyhook(p, zeta_sky)
%CONTROLLER_SKYHOOK Skyhook 상태 피드백 (블록 2 기준선 제어기).  F_act,i = −c_sky,i · ż_b,i
%   Kfb = CONTROLLER_SKYHOOK(p, zeta_sky)   → run_fullcar의 Kfb (4×22, 입력 w 쪽은 0)
%   c_sky,i = 2·ζ_sky·√(k_i·m_c,i) — Passive 댐퍼와 같은 식이라 ζ_sky는 "하늘에 건 댐퍼의 감쇠비". Passive 댐퍼는
%   그대로 두고 그 위에 더한다. ż_b,i는 노면 추세 기준 차체 코너 속도 (build_state_space) — 실차의 하이패스한
%   가속도계 적분에 해당. 언덕을 따라가는 절대 속도를 쓰면 Spa에서 10 kN 넘는 힘이 나온다.

[~, ~, veh] = build_state_space(p);
c_sky = 2 * zeta_sky * sqrt(veh.k .* veh.mc);
Kfb = diag(c_sky) * [zeros(4, 7), veh.T, zeros(4, 4), zeros(4, 8)];
end
