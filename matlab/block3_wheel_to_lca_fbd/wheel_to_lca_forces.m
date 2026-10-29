function F_LCA = wheel_to_lca_forces(wf, hardpoints, mount_position_p)
%WHEEL_TO_LCA_FORCES 휠력 -> 로어암 하중 변환 (블록 3, 준정적 FBD). 스켈레톤 — 미구현.
%   F_LCA = WHEEL_TO_LCA_FORCES(wf, hardpoints, mount_position_p)
%   입력:
%     wf                - 인터페이스 I1 구조체 (wheel_forces_<track>_<scn>_<ctrl>.mat)
%     hardpoints         - 볼조인트/부싱 2점/마운트 좌표 (D3). 프런트는 mount_position_p별 좌표 세트
%     mount_position_p   - 프런트 액추에이터 마운트 위치 p (D16, 3수준 중 하나). 리어는 고정 MR_rear(D18) 사용
%   출력:
%     F_LCA - 시간별 로어암 하중 벡터 (볼조인트 Fx,Fy,Fz + 스프링·액추에이터 마운트 힘)
%
%   가정: 링크 질량 무시, 준정적, 미소 변위(하드포인트 고정). 너클 FBD -> 선형 연립방정식.
%   모선비: F_act = F_wheel / mount_position_p (힌지축 모멘트 평형 단순화, README.md 참고).

error('wheel_to_lca_forces:NotImplemented', 'TODO: 블록 3 구현 — D3/D16 확정 후 너클 FBD + MR(p) 환산');
end
