%TEST_LCA_DAMAGE I2 lca_damage 자체 검사 (블록 5). 실패하면 assert 에러.
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here, '..', 'block1_lap_sim'), fullfile(here, '..', 'block3_wheel_to_lca_fbd'), ...
    fullfile(here, '..', 'block4a_lca_structure'));
p = 0.65;
hp = hardpoints_f82(p);
vp = block3_params();

% 1) 텐서 중첩: 하중 x만 / z만 따로 중첩한 텐서의 합 = 함께 중첩한 텐서 (von Mises는 합이 안 맞음)
C = stress_coeffs_beam(hp.c(1), p, lca_section());
Fl = [5000 0 0; 0 0 4000];
[ea, sa] = superpose_stress(Fl(1, :), C);
[eb, sb] = superpose_stress(Fl(2, :), C);
[e, sab] = superpose_stress(sum(Fl, 1), C);
assert(max(abs(sab - sa - sb), [], 'all') < 1e-9, '텐서 중첩');
assert(all(abs(e) <= abs(ea) + abs(eb) + 1e-9), 'von Mises 삼각부등식');

% 2) 더미 I1 정현 범프: 일정 진폭 수직 하중 → 지배 핫스팟은 마운트 단면, 기여 분해는 vert_dyn
t = (0:1e-3:20)';
lap = struct('t', t, 'v', 20 + 0*t, 'ax', 0*t, 'ay', 0*t, 's', 20*t);
wf = make_dummy_i1('A', struct('lap', lap, 'dt', 1e-3));
wf.F_wc(:, :, 3) = wf.F_wc(:, :, 3) .* (1 + 0.3*sin(2*pi*2*t));
wf.F_spring = wf.F_wc(:, :, 3) - vp.m_u * vp.g;
res = lca_damage(wf, p);
assert(all(startsWith(res.hot, 'M_')), '수직 하중인데 지배 핫스팟이 마운트 단면이 아님');
% vert_dyn은 평균 0이라 Goodman 보정(정적 인장 평균)을 받는 전체 DEL보다 조금 작다
pd = res.parts;
assert(pd.vert_dyn.DEL(1) > 0.8 * res.DEL(1) && pd.vert_qs.DEL(1) < 0.2 * res.DEL(1) && pd.Fx.DEL(1) < 1e-6, '기여 분해');
assert(abs(sum(res.Dsec(:, 1)) / res.D(1) - 1) < 1e-9, 'Dsec 합 ≠ D');

% 3) 진폭 2배 → DEL 약 2배 (평균 하중이 같아 Goodman 보정 차이만큼 벗어남)
wf2 = wf;
Fz = wf.F_wc(:, :, 3);
wf2.F_wc(:, :, 3) = Fz + (Fz - mean(Fz));
wf2.F_spring = wf2.F_wc(:, :, 3) - vp.m_u * vp.g;
r2 = lca_damage(wf2, p, struct('parts', false));
assert(r2.DEL(1) / res.DEL(1) > 1.9 && r2.DEL(1) / res.DEL(1) < 2.3, '진폭 배율');
fprintf('정현 범프: FL 지배 %s, DEL %.1f MPa, 피크 %.1f MPa, vert_dyn DEL %.1f MPa\n', ...
    res.hot{1}, res.DEL(1), res.sigma_peak(1), res.parts.vert_dyn.DEL(1));
disp('test_lca_damage OK');
