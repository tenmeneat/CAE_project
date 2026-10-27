function results = sweep_qr(q_values, r_values, track_name, controller_configs)
%SWEEP_QR LQR Q/R 가중치 스윕 -> 승차감 vs 피로손상 파레토 (블록 6). 스켈레톤 — 미구현.
%   results = SWEEP_QR(q_values, r_values, track_name, controller_configs)
%   각 Q/R 조합마다 블록 2 -> 3 -> 5를 자동 실행 (COMSOL 재해석 불필요, 응력계수 표는 고정 재사용).
%   결과: 차체 가속도 RMS vs 랩당 손상 D_lap 파레토.
%
%   위치 스케줄 제어는 schedule_gain_blend.m에서 게인을 거리 s 기준으로 블렌딩해서 넘길 것
%   (게인 급전환 금지 — 과도응답 자체가 하중 스파이크가 됨).

error('sweep_qr:NotImplemented', 'TODO: 블록 6 구현 — 블록 2/3/5가 먼저 동작해야 스윕 가능');
end
