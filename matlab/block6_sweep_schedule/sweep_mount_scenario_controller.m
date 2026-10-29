function sweep_results = sweep_mount_scenario_controller(mount_positions_p, scenarios, controller_configs, track_name)
%SWEEP_MOUNT_SCENARIO_CONTROLLER p x 시나리오 x 제어 스윕 (블록 6). 스켈레톤 — 미구현.
%   sweep_results = SWEEP_MOUNT_SCENARIO_CONTROLLER(mount_positions_p, scenarios, controller_configs, track_name)
%   mount_positions_p   - 프런트 마운트 위치 3수준 (D16)
%   scenarios           - {'A','B'}
%   controller_configs  - Passive/Skyhook/LQR(Q/R 격자)/스케줄 LQR 설정 목록 (D14 격자)
%
%   각 조합마다 블록 2 -> 3 -> 5(res = lca_damage(...))를 자동 실행하고, I5(act_thermal)로
%   F_rms <= Fth를 판정, I2 결과에서 R = D/D_passive 를 계산한다 (Passive 조합을 기준으로).
%   약 300-480개 조합 (v0.7). 결과는 I4(sweep_results_<track>.mat)로 저장 -> select_setpoint.m, 뷰어가 소비.
%
%   위치 스케줄 제어는 schedule_gain_blend.m에서 게인을 거리 s 기준으로 블렌딩해서 넘길 것
%   (게인 급전환 금지 — 과도응답 자체가 하중 스파이크가 됨).

error('sweep_mount_scenario_controller:NotImplemented', 'TODO: 블록 6 구현 — 블록 2/3/5, I5 열 한계 판정이 먼저 동작해야 스윕 가능');
end
