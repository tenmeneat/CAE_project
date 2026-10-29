function [t, v, ax, ay, s] = lap_sim(raceline_csv, fastf1_csv, params, scenario)
%LAP_SIM 준정상 랩 시뮬레이션 (블록 1). 스켈레톤 — 미구현.
%   [t, v, ax, ay, s] = LAP_SIM(raceline_csv, fastf1_csv, params, scenario)
%   입력:
%     raceline_csv - TUMFTM 레이스라인/중심선 CSV 경로 (scenario에 따라 다름)
%     fastf1_csv   - 인터페이스 I0 (fastf1_<track>.csv) 경로
%     params       - 차량 파라미터 struct (D3, BMW F82 M4급 기준)
%     scenario     - 'A' (스포츠, 레이스라인+마찰원 한계) | 'B' (컴포트, 중심선+가속도 상한 |ay|<=0.4g,|ax|<=0.3g)
%   출력: 블록 2(Simulink)로 넘길 시간 이력. ISO 8608 노면은 별도 중첩.
%
%   처리 순서는 README.md 참고. 시나리오 B는 가속도 상한으로 직접 캡(마찰원 적분 아님).

error('lap_sim:NotImplemented', 'TODO: 블록 1 구현 — scenario A/B 분기 (README.md 참고)');
end
