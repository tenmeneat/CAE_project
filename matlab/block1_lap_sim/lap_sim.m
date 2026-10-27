function [t, v, ax, ay, s] = lap_sim(raceline_csv, fastf1_csv, params)
%LAP_SIM 준정상 랩 시뮬레이션 (블록 1). 스켈레톤 — 미구현.
%   [t, v, ax, ay, s] = LAP_SIM(raceline_csv, fastf1_csv, params)
%   입력:
%     raceline_csv - TUMFTM 레이스라인 CSV 경로
%     fastf1_csv   - 인터페이스 I0 (fastf1_<track>.csv) 경로
%     params       - 차량 파라미터 struct (D3 확정 후)
%   출력: 블록 2(Simulink)로 넘길 시간 이력. ISO 8608 노면은 별도 중첩.
%
%   처리 순서는 README.md 참고 (Δs=1m 재샘플 → κ(s) → v_lim(s) → 전진/후진 적분 → s→t 변환).

error('lap_sim:NotImplemented', 'TODO: 블록 1 구현 (README.md 처리 순서 참고)');
end
