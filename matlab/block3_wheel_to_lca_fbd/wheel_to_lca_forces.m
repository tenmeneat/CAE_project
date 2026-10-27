function F_LCA = wheel_to_lca_forces(wf, hardpoints)
%WHEEL_TO_LCA_FORCES 휠력 -> 로어암 하중 변환 (블록 3, 준정적 FBD). 스켈레톤 — 미구현.
%   F_LCA = WHEEL_TO_LCA_FORCES(wf, hardpoints)
%   입력:
%     wf         - 인터페이스 I1 구조체 (wheel_forces_<track>_<ctrl>.mat)
%     hardpoints - 볼조인트/부싱 2개/마운트 좌표 (D1 서스펜션 형식 확정 후 정의)
%   출력:
%     F_LCA - 시간별 로어암 하중 벡터 (볼조인트 Fx,Fy,Fz + 액추에이터 마운트 힘)
%
%   가정: 링크 질량 무시, 준정적, 미소 변위(하드포인트 고정). 너클 FBD -> 선형 연립방정식.

error('wheel_to_lca_forces:NotImplemented', 'TODO: 블록 3 구현 — D1 확정 후 너클 FBD 세부 설계');
end
