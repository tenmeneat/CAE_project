function [D, Dsec, sig] = lca_damage(wf, stress_coeffs, sn_params)
%LCA_DAMAGE 응력 중첩 -> 피로손상 (블록 5, 인터페이스 I2). 스켈레톤 — 미구현.
%   [D, Dsec, sig] = LCA_DAMAGE(wf, stress_coeffs, sn_params)
%   입력:
%     wf            - 인터페이스 I1 구조체 (블록 3을 거친 F_LCA(t) 포함)
%     stress_coeffs - 블록 4 COMSOL 단위하중 응력계수 표 C (핫스팟 응력텐서 6성분/단위하중)
%     sn_params     - S-N 곡선 파라미터 (sigma_f', b, sigma_u) — D4(재질) 확정 후
%   출력:
%     D    - 랩당 손상 (Miner)
%     Dsec - 구간(코너)별 손상
%     sig  - 핫스팟 응력 시계열 (등가응력, 부호 있는 von Mises 또는 최대주응력)
%
%   절차: 텐서 성분 중첩(sigma(t) = sum_k C_k*F_k(t)) -> 등가응력 계산 -> rainflow() ->
%         Goodman 평균응력 보정 -> Basquin S-N + Miner.
%   주의: von Mises를 직접 더하면 안 됨 — 반드시 텐서 성분을 먼저 중첩할 것.

error('lca_damage:NotImplemented', 'TODO: 블록 5 구현 — D4(로어암 재질) 확정 후 S-N 파라미터 채우기');
end
