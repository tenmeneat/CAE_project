function T = compare_controllers(p, ctrl)
%COMPARE_CONTROLLERS Spa 시나리오 A/B에서 제어기 3축 지표 비교 (블록 2). Simulink 모델(단방향 타이어) 사용.
%   T = COMPARE_CONTROLLERS(p, ctrl)   ctrl: {이름, Kfb; ...} (기본: Passive, Skyhook, LQR 후보)
%   T: 시나리오 × 제어기 table. 접지·스트로크·힘은 네 코너 중 최댓값, 비율은 같은 시나리오 Passive 대비.
%   포화·지연 없음 (W4) — 피크 힘이 포화 3.7 kN(휠 기준, MR 미반영)을 넘는지는 F_peak로 본다.

if nargin < 1, p = f82_params(); end
if nargin < 2
    ctrl = {'Passive', zeros(4, 22)};
    for z = [0.2 0.3 0.4], ctrl(end+1, :) = {sprintf('Skyhook %.1f', z), controller_skyhook(p, z)}; end
    for r = [1 1; 1 2; 1 4; 0.25 1]'
        ctrl(end+1, :) = {sprintf('LQR c%g g%g', r), controller_lqr(p, struct('comfort', r(1), 'grip', r(2)))};
    end
end

rows = {};
for scn = 'AB'
    w = make_spa_inputs(p, scn);
    for i = 1:size(ctrl, 1)
        [t, x, F, Fz] = run_fullcar(p, ctrl{i, 2}, w, 1e-3);
        m = fullcar_metrics(p, t, x, F, Fz, w);
        if i == 1, m0 = m; end
        rows(end+1, :) = {scn, ctrl{i, 1}, m.wk, m.wk / m0.wk, max(m.Fdyn), max(m.Fdyn ./ m0.Fdyn), 100 * max(m.lift), ...
            1e3 * min(m.d_min), 1e3 * max(m.d_max), max(m.F_rms), max(m.F_peak), m.P_mean}; %#ok<AGROW>
    end
end
T = cell2table(rows, 'VariableNames', {'scn', 'ctrl', 'wk', 'wk_ratio', 'Fdyn_max', 'Fdyn_ratio_worst', ...
    'lift_pct', 'd_min_mm', 'd_max_mm', 'F_rms', 'F_peak', 'P_mean'});
end
