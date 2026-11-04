%RUN_LCA_DAMAGE_I1 export_i1 결과(outputs/i1) 전체로 블록 3 → 4a → 4b → 5를 돌려 요약 (블록 5).
%   먼저 블록 2에서 export_i1()을 실행해 outputs/i1/wheel_forces_spa_<A|B>_<ctrl>.mat 6개를 만든다.
%   출력: 화면 요약, outputs/b5/summary_p<p>.csv (git 제외).
%   R = D/D_passive와 재질 민감도(알루미늄·강, m이 다름)는 확인용 — 공식 R 계산은 블록 6(A).
here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..', '..');
addpath(here, fullfile(here, '..', 'block1_lap_sim'), fullfile(here, '..', 'block3_wheel_to_lca_fbd'), ...
    fullfile(here, '..', 'block4a_lca_structure'), fullfile(here, '..', 'block4b_actuator_thermal'));
p = 0.65;                                            % D16 임시값
vp = block3_params();
hp = hardpoints_f82(p);
ap = actuator_params();
mats = {'al_forged', 'steel'};
files = dir(fullfile(root, 'outputs', 'i1', 'wheel_forces_spa_*.mat'));
assert(~isempty(files), 'outputs/i1이 비어 있음 — 블록 2 export_i1()을 먼저 실행');
cn = {'FL', 'FR', 'RL', 'RR'};

rows = {};
for f = files'
    wf = load(fullfile(f.folder, f.name)).wf;
    wf = validate_i1(wf, vp);                        % s 복원은 여기서 한 번만
    F = wheel_to_lca_forces(wf, hp, vp);
    I5 = actuator_thermal(F.F_act_act, ap);
    for m = 1:numel(mats)
        res = lca_damage(wf, p, struct('sn', sn_params(mats{m}), 'parts', m == 1));
        c = res.crit;
        [~, ks] = max(res.Dsec(:, c));
        r = {wf.meta.scenario, wf.meta.ctrl, mats{m}, cn{c}, res.hot{c}, res.D(c), res.DEL(c), max(res.sigma_peak), ...
            res.sec_edges(ks), res.Dsec(ks, c) / res.D(c), max(prctile(abs(F.resid_Fs(:, 1:2)), 99)), ...
            max(I5.F_rms(1:2)), I5.Fth, all(I5.ok), max(I5.F_peak(1:2)), any(I5.sat), max(I5.Tcoil(1:2))};
        if m == 1
            r = [r, {res.parts.Fx.DEL(c), res.parts.Fy.DEL(c), res.parts.vert_qs.DEL(c), res.parts.vert_dyn.DEL(c)}];
        else
            r = [r, {NaN, NaN, NaN, NaN}];
        end
        rows(end+1, :) = r; %#ok<SAGROW>
    end
end
T = cell2table(rows, 'VariableNames', {'scn', 'ctrl', 'material', 'crit_corner', 'hotspot', 'D', 'DEL_MPa', 'peak_MPa', ...
    'worst_sec_s0', 'worst_sec_share', 'resid_front_p99_N', 'Frms_act_front_N', 'Fth_N', 'thermal_ok', ...
    'Fpeak_act_front_N', 'saturated', 'Tcoil_front_C', 'DEL_Fx', 'DEL_Fy', 'DEL_vert_qs', 'DEL_vert_dyn'});
% 시나리오·재질별 Passive 대비 R, DEL 비
T.R = NaN(height(T), 1);  T.DEL_ratio = NaN(height(T), 1);
for k = 1:height(T)
    b = strcmp(T.scn, T.scn{k}) & strcmp(T.material, T.material{k}) & strcmp(T.ctrl, 'passive');
    T.R(k) = T.D(k) / T.D(b);
    T.DEL_ratio(k) = T.DEL_MPa(k) / T.DEL_MPa(b);
end
T = sortrows(T, {'material', 'scn', 'ctrl'});
disp(T(:, {'scn', 'ctrl', 'material', 'crit_corner', 'hotspot', 'DEL_MPa', 'peak_MPa', 'R', 'DEL_ratio', ...
    'worst_sec_s0', 'worst_sec_share'}));
disp(T(strcmp(T.material, 'al_forged'), {'scn', 'ctrl', 'DEL_Fx', 'DEL_Fy', 'DEL_vert_qs', 'DEL_vert_dyn', ...
    'resid_front_p99_N', 'Frms_act_front_N', 'Fth_N', 'thermal_ok', 'Fpeak_act_front_N', 'saturated', 'Tcoil_front_C'}));
% 재질(m)을 바꿔도 시나리오 안에서 제어기 순위가 유지되는지
for s = {'A', 'B'}
    a = T(strcmp(T.scn, s{1}) & strcmp(T.material, 'al_forged'), :);
    st = T(strcmp(T.scn, s{1}) & strcmp(T.material, 'steel'), :);
    [~, ia] = sort(a.R);  [~, is] = sort(st.R);
    fprintf('시나리오 %s 제어기 순위 (R 오름차순): 알루미늄 %s | 강 %s → %s\n', s{1}, strjoin(a.ctrl(ia)', ' < '), ...
        strjoin(st.ctrl(is)', ' < '), string(isequal(a.ctrl(ia), st.ctrl(is))));
end
out = fullfile(root, 'outputs', 'b5');
if ~isfolder(out), mkdir(out); end
writetable(T, fullfile(out, sprintf('summary_p%03d.csv', round(100*p))));
