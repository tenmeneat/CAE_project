%TEST_BUILD_STATE_SPACE 풀카 7DOF Passive 행렬 자체 검사 (실패하면 assert 에러).
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'block1_lap_sim'));
p = f82_params();
[A, B, info] = build_state_space(p);

% 1) 모드: 안정, ζ≈0.3, 휠 홉 4모드 10–16 Hz.
%    DI = 1이면 피치·히브가 앞/뒤 코너 모드로 분리 → 비감쇠 차체 모드 중 2개가 f_ride 앞/뒤와 일치해야 함
lam = eig(A);
lam = lam(imag(lam) > 0);
fn = abs(lam) / (2*pi);  zt = -real(lam) ./ abs(lam);
[fn, i] = sort(fn);  zt = zt(i);
fprintf('모드 %d: fn = %5.2f Hz, ζ = %.2f\n', [1:numel(fn); fn'; zt']);
assert(all(real(eig(A)) < 0), '불안정 모드');
assert(numel(fn) == 7, '진동 모드 7개가 아님');
f0 = sort(sqrt(eig(-A(8:14, 1:7)))) / (2*pi);       % 비감쇠 고유진동수
fprintf('비감쇠 차체 모드 %s Hz (목표 앞 %.1f / 뒤 %.1f)\n', mat2str(f0(1:3)', 3), p.f_ride);
for fr = p.f_ride
    assert(min(abs(f0(1:3) - fr)) / fr < 0.03, '승차 주파수 역산 불일치 (%.1f Hz)', fr);
end
assert(all(zt(1:3) > 0.2 & zt(1:3) < 0.4), '차체 모드 감쇠비 이상');
assert(all(fn(4:7) > 10 & fn(4:7) < 16), '휠 홉 모드 범위 밖');

% 2) 정상상태 (x = −A\B u). 타이어 동하중 ΔFz = kt(z_r − z_u)
steady = @(u) -A \ (B * u);
dFz = @(x, zr) info.kt * (zr - x(4:7));

%   노면 4개 동시 +1 cm → 전체가 1 cm 상승, 하중 변화 없음
x = steady([zeros(4,1); 0.01*ones(4,1); 0; 0]);
assert(max(abs(x([1 4:7]) - 0.01)) < 1e-9 && max(abs(x(2:3))) < 1e-12, '노면 균일 상승 DC 이득 이상');

%   좌회전 1 g (ay > 0): 좌측 상승(φ > 0), 우측 하중 증가, ΣΔFz = 0, Σ y·ΔFz = −M_φ
Mphi = info.ms * p.g * p.h_cg;
x = steady([zeros(8,1); 0; p.g]);  d = dFz(x, 0);
fprintf('롤 구배 %.2f deg/g, 앞 롤강성 비 %.2f, ΔFz [FL FR RL RR] = %s N\n', rad2deg(x(3)), ...
    (d(2) - d(1)) * p.tf / ((d(2) - d(1)) * p.tf + (d(4) - d(3)) * p.tr), mat2str(round(d')));
assert(x(3) > 0 && d(2) > 0 && d(1) < 0, '롤 부호 이상');
assert(abs(sum(d)) < 1e-6 && abs(info.yi' * d + Mphi) < 1e-6, '롤 모멘트 평형 이상');
lltd = (d(2) - d(1)) * p.tf / 2 / Mphi;
assert(abs(rad2deg(x(3)) - p.roll_grad) < 1e-6 && abs(lltd - p.lltd_f) < 1e-6, '스태빌라이저 역산 불일치');
fprintf('스태빌라이저 k_arb 앞/뒤 = %s N/mm (휠 기준)\n', mat2str(info.k_arb' / 1e3, 3));

%   제동 1 g (ax < 0): 노즈 다운(θ > 0), 앞 하중 증가, Σ x·ΔFz = M_θ
Mth = -info.ms * (-p.g) * p.h_cg;
x = steady([zeros(8,1); -p.g; 0]);  d = dFz(x, 0);
fprintf('피치 구배 %.2f deg/g (제동), 앞축 ΔFz = %.0f N\n', rad2deg(x(2)), d(1) + d(2));
assert(x(2) > 0 && d(1) > 0 && d(3) < 0, '피치 부호 이상');
assert(abs(sum(d)) < 1e-6 && abs(info.xi' * d - Mth) < 1e-6, '피치 모멘트 평형 이상');

%   액추에이터: FL만 +1 kN → 차체 FL 상승, 휠 하강(타이어 하중 증가), 내력이라 ΣΔFz = 0
x = steady([1000; zeros(9,1)]);  d = dFz(x, 0);
zb = info.T * x(1:3);
assert(zb(1) > 0 && x(4) < 0 && abs(sum(d)) < 1e-6, '액추에이터 부호 이상');
disp('test_build_state_space OK');
