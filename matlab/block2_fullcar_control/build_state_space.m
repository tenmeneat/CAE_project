function [A, B, C, D] = build_state_space(hardpoints, vehicle_params)
%BUILD_STATE_SPACE 풀카 7DOF 상태공간 행렬 구성 (블록 2). 스켈레톤 — 미구현.
%   [A, B, C, D] = BUILD_STATE_SPACE(hardpoints, vehicle_params)
%   상태 14개(변위+속도), 액추에이터 1차 지연 포함 시 +4.
%   입력 4개: 액추에이터 힘(FL,FR,RL,RR) + 노면 변위 4개.
%
%   전제: D1(서스펜션 형식) 확정 필요 — docs/decisions.md 참고.
%   같은 A,B를 lqr()에 그대로 사용 (Simulink State-Space 블록과 공유, 대수루프 회피).

error('build_state_space:NotImplemented', 'TODO: 블록 2 구현 — D1(서스펜션 형식) 확정 후 하드포인트 기구학부터');
end
