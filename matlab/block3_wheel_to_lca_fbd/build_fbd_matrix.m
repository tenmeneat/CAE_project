function [A, G] = build_fbd_matrix(hc, geo)
%BUILD_FBD_MATRIX 너클 FBD 8×8 선형 문제 (블록 3). A·x = G·f
%   x = [F_L(3); F_U(3); f_T; F_s]  F_L, F_U: 하부·상부암이 너클에 주는 힘, f_T: 타이로드 축력(인장 +),
%                                   F_s: 스트럿 축력(압축 +, 로어암에 −F_s·u_s)
%   f = [F_cp(3); F_wc(3)]          접지점·휠센터에 걸리는 외력
%   행: 너클+휠 힘 평형 3, 휠센터 기준 모멘트 평형 3, 상부암 힌지축 모멘트 1, 하부암 힌지축 모멘트 1.
%   A는 하드포인트와 p로만 정해진다(미소 변위).

rL  = hc.LBJ  - hc.WC;
rU  = hc.UBJ  - hc.WC;
rTO = hc.TR_O - hc.WC;
rCP = hc.CP   - hc.WC;
aL = hc.LBJ - hc.LCA_F;
aU = hc.UBJ - hc.UCA_F;
aM = geo.r_M - hc.LCA_F;
I = eye(3);  Z = zeros(3);  z = zeros(3, 1);

% e·(a×(−F)) = −(e×a)·F
A = [I,        I,        geo.u_T,             z
     skew(rL), skew(rU), skew(rTO) * geo.u_T, z
     z',       -cross(geo.e_U, aU)', 0,       0
     -cross(geo.e_L, aL)', z', 0,             -dot(geo.e_L, cross(aM, geo.u_s))];
G = -[I,         I
      skew(rCP), Z
      zeros(2, 6)];

c = cond(A);
if c > 1e3
    warning('build_fbd_matrix:IllConditioned', 'cond(A) = %.3g > 1e3 — 하드포인트 확인', c);
end
end

function S = skew(r)
S = [0 -r(3) r(2); r(3) 0 -r(1); -r(2) r(1) 0];
end
