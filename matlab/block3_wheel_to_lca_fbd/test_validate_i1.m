%TEST_VALIDATE_I1 I1 진입부 검사 (블록 3). 실패하면 assert 에러.
%   1) 필수 필드·크기 에러, 2) 선택 필드 기본값, 3) s 복원 = export_i1의 s, 4) 실제 export_i1 파일 (있으면)
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'block1_lap_sim'));
vp = block3_params();
hp = hardpoints_f82(0.65);
N = 5;  Z = zeros(N, 4);
base = struct('t', (0:N-1)'*1e-3, 's', zeros(N, 1), 'F_wc', cat(3, Z, Z, 4000 + Z), 'F_spring', Z, 'F_damper', Z, 'F_act', Z, 'a_u', Z);

% 1) 필수 필드 없음 / 크기 틀림 → 에러
bad = rmfield(base, 'F_wc');
try, validate_i1(bad, vp); error('x:noerr', ''); catch e, assert(strcmp(e.identifier, 'validate_i1:Missing'), '필드 누락 미검출'); end
bad = base;  bad.F_act = zeros(N, 3);
try, validate_i1(bad, vp); error('x:noerr', ''); catch e, assert(strcmp(e.identifier, 'validate_i1:Size'), '크기 오류 미검출'); end

% 2) F_arb, meta 기본값
w = validate_i1(base, vp);
assert(isequal(w.F_arb, Z) && isfield(w.meta, 'scenario') && isfield(w.meta, 'ctrl'), '기본값 이상');

% 3) s 복원: make_spa_inputs의 lap.s와 같아야 함 (블록 2 경로가 있으면)
b2 = fullfile(here, '..', 'block2_fullcar_control');
if isfile(fullfile(b2, 'make_spa_inputs.m'))
    addpath(b2);
    [~, lap] = make_spa_inputs(f82_params(), 'B');
    k = round(linspace(1, numel(lap.t), 50))';
    w = rmfield(base, 's');  w.t = lap.t(k);
    w.F_wc = repmat(base.F_wc(1, :, :), 50, 1, 1);
    for f = {'F_spring', 'F_damper', 'F_act', 'a_u'}, w.(f{1}) = zeros(50, 4); end
    w.meta = struct('track', 'spa', 'scenario', 'B');
    w = validate_i1(w, vp);
    assert(max(abs(w.s - lap.s(k))) < 1e-9, 's 복원이 export_i1과 다름');
end

% 4) 실제 export_i1 파일이 있으면 블록 3 변환까지 (outputs/i1, git 제외)
f = fullfile(here, '..', '..', 'outputs', 'i1', 'wheel_forces_spa_A_passive.mat');
if isfile(f)
    wf = load(f).wf;
    out = wheel_to_lca_forces(wf, hp, vp);
    q = prctile(abs(out.resid_Fs), 99);
    fprintf('I1 %s: N %d, s %.0f–%.0f m, |resid_Fs| 99%% [%s] N\n', wf.meta.ctrl, numel(out.t), out.s(1), out.s(end), num2str(q, '%.0f '));
    assert(all(isfinite(out.s)) && all(diff(out.s) >= 0), 's 이상');
    assert(all(q(1:2) < 0.15 * max(out.F_M(:, 1:2))), '앞 resid_Fs가 비정상적으로 큼');
end
disp('test_validate_i1 OK');
