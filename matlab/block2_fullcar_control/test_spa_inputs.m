%TEST_SPA_INPUTS make_spa_inputs + Passive Spa 1랩 자체 검사 (실패하면 assert 에러). 매끈한 노면(고저만)으로 본다.
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here, '..', 'block1_lap_sim'));
p = f82_params();
[~, ~, info] = build_state_space(p);

for scn = 'AB'
    [w, lap] = make_spa_inputs(p, scn, 1e-3, 1e-15);
    % 1) 후륜 노면 = 앞바퀴가 L 전에 지난 노면 (거리 지연)
    k = lap.s - info.b > info.a + 1;
    zr_delay = interp1(lap.s + info.a, w(:,2), lap.s(k) - info.b);
    assert(max(abs(zr_delay - w(k,4))) < 1e-3, '후륜 거리 지연 이상');
    % 2) Passive 1랩: 타이어 하중 합의 랩 평균 = m·g (닫힌 루프라 평균 수직 가속 ≈ 0), 노면 기준 피치 < 3°
    [t, x, ~, Fz] = run_fullcar(p, zeros(4,14), w, 1e-3);
    th_rel = x(:,2) + (w(:,2) - w(:,4)) / p.L;
    fprintf('%s: 랩 %.1f s, ΣFz 평균/mg = %.4f, Fz min %s N, 노면 기준 피치 max %.2f deg\n', ...
        scn, t(end), mean(sum(Fz, 2)) / (p.m * p.g), mat2str(round(min(Fz))), rad2deg(max(abs(th_rel))));
    assert(abs(mean(sum(Fz, 2)) / (p.m * p.g) - 1) < 0.01, '수직 하중 평형 이상');
    assert(rad2deg(max(abs(th_rel))) < 3, '노면 기준 피치 과대');
end
disp('test_spa_inputs OK');
