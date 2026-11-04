%TEST_STRESS_COEFFS 블록 4a 응력계수 표 (빔 해석해) 자체 검사. 실패하면 assert 에러.
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'block1_lap_sim'), fullfile(here, '..', 'block3_wheel_to_lca_fbd'));
sec = lca_section();
p = 0.65;
hc = hardpoints_f82(p).c(1);
C = stress_coeffs_beam(hc, p, sec);
L = C.meta.L;
bi = sec.b - 2*sec.tw;  hi = sec.h - 2*sec.tw;
Am = sec.b*sec.h - bi*hi;  I1m = (sec.b*sec.h^3 - bi*hi^3)/12;
hot = @(n) find(strcmp(C.hot.name, n));

% 1) 마운트 단면 위 섬유, LBJ 수직 1 N: σ = −(1−p)L·(h/2)/(2·I1m) (위가 압축)
assert(abs(C.coef(hot('M_FT'), 2, 3) - (-(1-p)*L*(sec.h/2)/(2*I1m))) < 1e-9, '마운트 단면 수직 굽힘');
assert(abs(C.coef(hot('M_FT'), 2, 3) + C.coef(hot('M_FU'), 2, 3)) < 1e-9, '위·아래 대칭');
% 2) 뿌리(힌지)는 수직 굽힘 0 → 위·아래 응력이 같음 (축력만)
assert(abs(C.coef(hot('R_FT'), 2, 3) - C.coef(hot('R_FU'), 2, 3)) < 1e-9, '힌지에서 수직 굽힘이 0이 아님');
% 3) 면내 1 N: 뿌리 응력 ≈ 트러스 부재 축력 L/(w0·Am). 핫스팟은 부재 바깥 모서리(중심 + b/2)라
%    1 ~ (w0 + b)/w0 배 사이여야 함
truss = L / (C.meta.w0 * Am);
r = abs(C.coef(hot('R_FT'), 2, 1)) / truss;
assert(r >= 1 - 1e-9 && r <= (C.meta.w0 + sec.b) / C.meta.w0, '면내 하중이 트러스 근사와 다름');
% 4) 스트럿 반력 = 블록 3 F_M (같은 힌지축 모멘트 평형): F_M = F_BJz,loc / (p·u3)
wf = make_dummy_i1('A', struct('lap', struct('t', [0; 1], 'v', [30; 30], 'ax', [-6; -6], 'ay', [8; 8], 's', [0; 30])));
out = wheel_to_lca_forces(wf, hardpoints_f82(p), block3_params());
assert(max(abs(out.F_M(:, 1) - out.F_BJ(:, 3, 1) * C.meta.R_per_Fz)) < 1e-6 * max(out.F_M(:, 1)), '스트럿 반력 ≠ 블록 3 F_M');
% 5) CSV 왕복
f = [tempname '.csv'];
C2 = stress_coeffs_beam(hc, 0.8, sec);
write_stress_coeffs_csv(f, [C, C2]);
R = read_stress_coeffs_csv(f, 0.8, 'beam_surrogate');
delete(f);
assert(max(abs(R.coef - C2.coef), [], 'all') < 1e-12 * max(abs(C2.coef), [], 'all') && isequal(R.hot.name, C2.hot.name), 'CSV 왕복');
fprintf('p=%.2f L=%.3f m, w0=%.3f m, 마운트 위 섬유 %.1f MPa/kN(수직), 뿌리 %.1f MPa/kN(면내)\n', p, L, C.meta.w0, ...
    C.coef(hot('M_FT'), 2, 3)*1e3/1e6, C.coef(hot('R_FT'), 2, 1)*1e3/1e6);
disp('test_stress_coeffs OK');
