function MR = motion_ratio(hc, geo)
%MOTION_RATIO 정역학 모션비 (블록 3). 접지점에 수직 단위하중만 걸 때 MR = Fz / F_s.
%   MR = MOTION_RATIO(hc, geo)
%   이 블록이 계산해 블록 2에 넘긴다 (포화·스트로크 환산, 스프링 강성 ∝ 1/MR²). 기구학 정의와 같아야 함
%   (motion_ratio_kin, 가상일).

[A, G] = build_fbd_matrix(hc, geo);
x = A \ G(:, 3);
MR = 1 / x(8);
end
