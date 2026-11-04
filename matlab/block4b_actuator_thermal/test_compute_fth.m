%TEST_COMPUTE_FTH 블록 4b MATLAB 측 자체 검사. 실패하면 assert 에러.
ap = actuator_params();
[Rth, parts] = thermal_rth_1d(ap);
% 1) 1D 열저항 손계산: 대류만 있으면 1/(h·A)
assert(abs(parts.conv - 1/(ap.h*2*pi*(ap.r_c + ap.t_pot + ap.t_h)*ap.L_c)) < 1e-12, '대류 저항');
% 2) Fth에서 코일 온도 = Tlim (공식의 역관계)
[Fth, T] = compute_fth(ap.Kf, ap.Rcoil, Rth, ap.Tamb, ap.Tlim, [0 1 2] * 0);
[~, Tth] = compute_fth(ap.Kf, ap.Rcoil, Rth, ap.Tamb, ap.Tlim, Fth);
assert(abs(Tth - ap.Tlim) < 1e-9 && all(T == ap.Tamb), 'Fth에서 Tcoil ≠ Tlim');
% 3) 정현파 힘: RMS = 진폭/√2 → I5 판정
t = (0:1e-3:10)';
I5 = actuator_thermal(Fth * sqrt(2) * [0.99*sin(2*pi*t), 0.5*sin(2*pi*t), 0*t, 1.2*sin(2*pi*t)], ap);
assert(abs(I5.F_rms(1) / Fth - 0.99) < 1e-3 && isequal(I5.ok, [true true true false]), 'I5 판정');
fprintf('Rth = %.3f K/W (포팅 %.3f, 하우징 %.4f, 대류 %.3f), Fth = %.0f N\n', Rth, parts.pot, parts.hous, parts.conv, Fth);
disp('test_compute_fth OK');
