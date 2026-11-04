%TEST_FBD_HANDCALC 블록 3 손계산 대조 (설계 9절 테스트 1–3 + 프로토타입 회귀값). 실패하면 assert 에러.
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'block1_lap_sim'));
vp = block3_params();
Fz0 = 1642 * 0.523 * 9.81 / 2;     % 앞 코너 정적 하중 4,212 N

% 프로토타입(Python, 같은 임시 하드포인트) 기대값: p, cond(A), MR
ref = [0.50 19.0 0.4940; 0.65 18.5 0.6423; 0.80 18.5 0.7905];
for k = 1:size(ref, 1)
    p = ref(k, 1);
    hc = hardpoints_f82(p).c(1);
    geo = lca_geometry(hc, p);
    [A, G] = build_fbd_matrix(hc, geo);
    MR = motion_ratio(hc, geo);
    MRk = motion_ratio_kin(hc, geo);
    fprintf('p=%.2f  cond(A)=%.2f  MR 정역학 %.4f  기구학 %.4f  p·cosα %.4f\n', p, cond(A), MR, MRk, p*cos(hc.alpha));
    assert(abs(cond(A) - ref(k, 2)) < 0.05, 'cond(A)가 프로토타입과 다름');
    assert(abs(MR - ref(k, 3)) < 5e-5, 'MR이 프로토타입과 다름');

    % 3) MR 정역학 = 기구학 (가상일)
    assert(abs(MR - MRk) < 1e-6 * MR, 'MR 정역학 ≠ 기구학');

    % 2) 2D 지렛대 손계산 F_s ≈ Fz/(p·cosα), 5% 이내
    x = A \ (G(:, 3) * Fz0);
    assert(abs(x(8) / (Fz0 / (p*cos(hc.alpha))) - 1) < 0.05, '2D 지렛대 손계산과 5% 넘게 다름');

    % 1) 정적 수직하중 (m_u 포함): F_s = (Fz − m_u·g)/MR
    Z = zeros(1, 4);
    wf = struct('t', 0, 's', 0, 'F_wc', cat(3, Z, Z, Fz0*ones(1,4)), 'a_u', Z, 'F_spring', Z, 'F_damper', Z, 'F_act', Z);
    out = wheel_to_lca_forces(wf, hardpoints_f82(p), vp);
    assert(all(abs(out.F_M ./ ((Fz0 - vp.m_u*vp.g) ./ out.MR) - 1) < 1e-6), '정적 F_s ≠ (Fz − m_u·g)/MR');
end

% 프로토타입 손계산 3케이스 (p = 0.65, m_u = 0): 로어암이 받는 LBJ 힘(차량 좌표), F_s
hc = hardpoints_f82(0.65).c(1);
geo = lca_geometry(hc, 0.65);
[A, G] = build_fbd_matrix(hc, geo);
T = A \ G;
cases = {'정적',            [0 0 1]*Fz0,           [0 805 4131],      6558
         '제동 1 g',        [-1.6 0 1.6]*Fz0,      [-9053 3724 6591], 10358
         '우선회 1 g (FL)', [0 -1.5 1.5]*Fz0,      [0 -7218 6409],    9314};
for k = 1:size(cases, 1)
    x = T(:, 1:3) * cases{k, 2}';
    F = -x(1:3)';
    fprintf('%-14s LBJ (%6.0f %6.0f %6.0f) N  F_s %6.0f N  Fz/MR %6.0f N  차이 %+.1f%%\n', cases{k, 1}, F, x(8), ...
        cases{k, 2}(3)*T(8, 3), 100*(x(8)/(cases{k, 2}(3)*T(8, 3)) - 1));
    assert(all(abs(F - cases{k, 3}) < 1.5) && abs(x(8) - cases{k, 4}) < 1.5, '손계산 케이스가 프로토타입과 다름');
end
disp('test_fbd_handcalc OK');
