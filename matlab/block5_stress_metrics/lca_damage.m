function res = lca_damage(wf, stress_coeffs, sn_params, mount_position_p)
%LCA_DAMAGE 응력 중첩 -> 응력 지표 (블록 5, 인터페이스 I2). 스켈레톤 — 미구현.
%   res = LCA_DAMAGE(wf, stress_coeffs, sn_params, mount_position_p)
%   입력:
%     wf                - 인터페이스 I1 구조체 (블록 3을 거친 F_LCA(t) 포함)
%     stress_coeffs      - 블록 4a COMSOL 단위하중 응력계수 표 C, mount_position_p별로 다름
%     sn_params           - S-N 곡선 파라미터 (sigma_f', b, sigma_u) — D4(재질) 확정 후
%     mount_position_p   - 프런트 액추에이터 마운트 위치 p (D16) — 어떤 C 표를 쓸지 결정
%   출력 res (struct):
%     res.D          - 랩당 손상 (Miner, 내부용 — 보고서 주 지표 아님)
%     res.DEL        - 등가응력진폭 (v0.4)
%     res.sigma_peak - 핫스팟 피크 응력 (v0.4, 정적 안전계수 판정용)
%     res.Dsec       - 구간(코너)별 손상
%     res.sig        - 핫스팟 응력 시계열 (등가응력)
%     res.parts      - 응력 기여 분해: 준정적 Fx/Fy, 수직 진동, 액추에이터 성분 (v0.7)
%
%   절차: 텐서 성분 중첩(sigma(t) = sum_k C_k*F_k(t)) -> 등가응력 계산 -> rainflow() ->
%         Goodman 평균응력 보정 -> Basquin S-N + Miner.
%   주의: von Mises를 직접 더하면 안 됨 — 반드시 텐서 성분을 먼저 중첩할 것.
%   R = res.D / D_passive 는 이 함수가 아니라 블록 6(A)의 스윕 루프에서 계산한다.

error('lca_damage:NotImplemented', 'TODO: 블록 5 구현 — D4(재질) 확정 후 S-N 파라미터, res.parts 분해');
end
