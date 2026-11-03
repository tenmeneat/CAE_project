%TEST_EXPORT_I1 I1 파일 자체 검사 (실패하면 assert 에러). 임시 폴더에 쓰고 다시 읽어 힘 평형을 본다.
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here, '..', 'block1_lap_sim'));
p = f82_params();
[~, ~, veh] = build_state_space(p);
tmp = tempname;
cases = {'A', 'lqr', controller_lqr(p, struct('comfort', 1, 'grip', 4)); 'B', 'passive', zeros(4, 22)};
for i = 1:size(cases, 1)
    export_i1(cases{i, :}, p, tmp);
    s = load(fullfile(tmp, sprintf('wheel_forces_spa_%s_%s.mat', cases{i, 1:2})));
    wf = s.wf;  N = numel(wf.t);
    assert(isequal(size(wf.F_wc), [N 4 3]) && isequal(size(wf.F_act), [N 4]) && isequal(size(wf.a_body), [N 3]), '필드 크기 이상');
    [w, lap] = make_spa_inputs(p, cases{i, 1});
    Fx = wf.F_wc(:,:,1);  Fy = wf.F_wc(:,:,2);  Fz = wf.F_wc(:,:,3);

    % 1) 수평 합력: ΣFy = m·ay, ΣFx = m·ax + 항력 + 경사 중력, 구동 중 앞바퀴 Fx = 0
    Fx_tot = p.m * lap.ax + 0.5 * p.rho * p.CdA * lap.v.^2 + p.m * p.g * lap.grade ./ sqrt(1 + lap.grade.^2);
    assert(max(abs(sum(Fy, 2) - p.m * lap.ay)) < 1e-6 * p.m * p.g && max(abs(sum(Fx, 2) - Fx_tot)) < 1e-6 * p.m * p.g, '수평 합력 이상');
    assert(all(Fx(Fx_tot > 0, 1:2) == 0, 'all'), '구동력이 앞바퀴에 들어감');
    % 2) 차체 수직 평형: Σ(서스펜션 힘) − m_s·g = m_s·z̈s (절대)
    Fs = wf.F_spring + wf.F_damper + wf.F_act + wf.F_arb;
    e_body = sum(Fs, 2) - veh.ms * p.g - veh.ms * wf.a_body(:,1);
    % 3) 너클 수직 평형 잔차 (블록 3 W1 점검 (b)): Fz − F_s − m_u·g − m_u·a_u = 0  (a_u는 Simulink 내부 값)
    e_knuckle = Fz - Fs - veh.mu' * p.g - veh.mu' .* wf.a_u;
    fprintf('%s %s: N %d, 차체 평형 잔차 %.1e N, 너클 잔차 최대 %.1e N, 마찰 사용률 > 1.32 비율 %s %%\n', ...
        cases{i, 1:2}, N, max(abs(e_body)), max(abs(e_knuckle), [], 'all'), mat2str(100 * mean(wf.mu_use > 1.32), 2));
    assert(max(abs(e_body)) < 1e-6 * p.m * p.g, '차체 수직 평형 잔차');
    assert(max(abs(e_knuckle), [], 'all') < 1e-6 * p.m * p.g, '너클 수직 평형 잔차 (export 식과 Simulink 내부가 다름)');
    assert(all(isnan(wf.mu_use(Fz < 1))) && ~any(isnan(wf.mu_use(Fz >= 1))), 'mu_use NaN 처리 이상');
    assert(abs(mean(sum(Fz, 2)) / (p.m * p.g) - 1) < 0.01 && all(Fz >= 0, 'all'), 'Fz 이상');
end
rmdir(tmp, 's');
disp('test_export_i1 OK');
