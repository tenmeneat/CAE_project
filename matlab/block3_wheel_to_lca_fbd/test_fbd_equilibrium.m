%TEST_FBD_EQUILIBRIUM 블록 3 평형·대칭·더미 I1 검사 (설계 9절 테스트 4–6). 실패하면 assert 에러.
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'block1_lap_sim'));
vp = block3_params();
hp = hardpoints_f82(0.65);
Fz0 = 1642 * 0.523 * 9.81 / 2;

% 4) 너클 평형 잔차, 조건수 (네 코너, 임의 하중)
rng(1);
for i = 1:4
    hc = hp.c(i);
    [A, G] = build_fbd_matrix(hc, lca_geometry(hc, hc.p));
    f = [randn(3, 1); randn(3, 1)] * Fz0;
    b = G * f;
    x = A \ b;
    assert(norm(A*x - b) < 1e-9 * norm(b), '너클 평형 잔차');
    assert(cond(A) < 1e3, 'cond(A) ≥ 1e3');
end

% 5) 좌우 대칭: 같은 크기 좌선회·우선회에서 FL(좌) = FR(우) (좌측 등가 출력). 제동·구동 섞음
lap = struct('t', [0; 1], 'v', [30; 30], 'ax', [-5; 3], 's', [0; 30]);
L = make_dummy_i1('A', lap_opts(lap,  9));
R = make_dummy_i1('A', lap_opts(lap, -9));
oL = wheel_to_lca_forces(L, hp, vp);
oR = wheel_to_lca_forces(R, hp, vp);
for pr = [1 2; 3 4]'
    d = max(abs(oL.F_BJ(:, :, pr(1)) - oR.F_BJ(:, :, pr(2))), [], 'all');
    assert(d < 1e-6 && max(abs(oL.F_M(:, pr(1)) - oR.F_M(:, pr(2)))) < 1e-6, '좌우 대칭 깨짐');
end

% 6) 정지 상태 더미 I1: ΣFz = m·g, resid_Fs = 0, wf.MR = 블록 3 MR (경고 없음)
cp = f82_params();
S = make_dummy_i1('A', lap_opts(struct('t', [0; 1], 'v', [0; 0], 'ax', [0; 0], 's', [0; 0]), 0));
oS = wheel_to_lca_forces(S, hp, vp);
assert(all(abs(sum(S.Fz, 2) - cp.m*cp.g) < 1e-6), 'ΣFz ≠ m·g');
assert(max(abs(oS.resid_Fs), [], 'all') < 1e-6, '정지 상태 resid_Fs ≠ 0');
S.F_arb = 300 * ones(size(S.Fz));  S.F_spring = S.F_spring - S.F_arb;   % 스태빌라이저가 지지력 일부를 맡아도 잔차 0
assert(max(abs(wheel_to_lca_forces(S, hp, vp).resid_Fs), [], 'all') < 1e-6, 'F_arb가 resid_Fs에 안 들어감');
fprintf('정지: F_M = [%s] N, MR = [%s]\n', num2str(oS.F_M(1, :), '%.0f '), num2str(oS.MR, '%.4f '));
disp('test_fbd_equilibrium OK');

function o = lap_opts(lap, ay)
lap.ay = [ay; ay];
o.lap = lap;
end
