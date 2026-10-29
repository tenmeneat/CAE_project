function [A, B, C, D] = build_state_space(hardpoints, vehicle_params, mount_position_p)
%BUILD_STATE_SPACE 풀카 7DOF 상태공간 행렬 구성 (블록 2). 스켈레톤 — 미구현.
%   [A, B, C, D] = BUILD_STATE_SPACE(hardpoints, vehicle_params, mount_position_p)
%   상태 14개(변위+속도), 액추에이터 1차 지연 포함 시 +4.
%   입력 4개: 액추에이터 힘(FL,FR,RL,RR) + 노면 변위 4개.
%
%   mount_position_p: 프런트 액추에이터 마운트 위치 p (3수준, D16). 모선비 MR(p)에 따라
%   스프링 강성(∝1/MR^2)과 액추에이터 포화·스트로크·속도 한계(휠 기준 환산)가 달라지므로
%   p마다 이 함수를 다시 호출해야 한다. 리어는 고정 MR_rear(D18) 사용.
%
%   전제: D1(서스펜션 형식=더블위시본) 확정됨. D3(하드포인트 등 세부 파라미터) 미결.
%   같은 A,B를 lqr()에 그대로 사용 (Simulink State-Space 블록과 공유, 대수루프 회피).

error('build_state_space:NotImplemented', 'TODO: 블록 2 구현 — D16(마운트 위치) 확정 후 MR(p) 기반 포화/강성 환산');
end
