function wf = validate_i1(wf, vp)
%VALIDATE_I1 인터페이스 I1 구조체 검사·보완 (블록 3 진입부). 형식은 블록 2 export_i1.m 도움말이 원본.
%   wf = VALIDATE_I1(wf, vp)
%   필수: t (N×1), F_wc (N×4×3, 접지점 [Fx Fy Fz]), F_spring, F_damper, F_act (N×4). 없거나 크기가 다르면 에러.
%   선택 (없으면 채움):
%     F_arb  N×4 → 0
%     a_u    N×4 → 0 + 경고 (준정적 근사)
%     s      N×1 → meta.scenario로 블록 1 lap_sim을 다시 돌려 s(t)를 복원 (export_i1의 make_spa_inputs와 같은 식).
%                  [동기화] export_i1에 s 추가를 요청함 — 추가되면 이 경로는 안 탐
%     meta   scenario, ctrl, track → ''
%   meta.m_u가 vp.m_u와 다르면 경고 (비현가 질량은 두 블록이 f82_params를 공유해야 함).

N = numel(wf.t);
need = {'t', 'F_wc', 'F_spring', 'F_damper', 'F_act'};
miss = need(~isfield(wf, need));
if ~isempty(miss)
    error('validate_i1:Missing', 'I1 필수 필드 없음: %s', strjoin(miss, ', '));
end
wf.t = wf.t(:);
if ~isequal(size(wf.F_wc), [N 4 3])
    error('validate_i1:Size', 'F_wc 크기 %s ≠ [%d 4 3]', mat2str(size(wf.F_wc)), N);
end
for f = {'F_spring', 'F_damper', 'F_act', 'F_arb', 'a_u'}
    if isfield(wf, f{1}) && ~isequal(size(wf.(f{1})), [N 4])
        error('validate_i1:Size', '%s 크기 %s ≠ [%d 4]', f{1}, mat2str(size(wf.(f{1}))), N);
    end
end

if ~isfield(wf, 'meta'), wf.meta = struct(); end
for f = {'scenario', 'ctrl', 'track'}
    if ~isfield(wf.meta, f{1}), wf.meta.(f{1}) = ''; end
end
if ~isfield(wf, 'F_arb'), wf.F_arb = zeros(N, 4); end
if ~isfield(wf, 'a_u')
    warning('validate_i1:NoAu', 'wf.a_u 없음 → 0으로 둠 (준정적 근사, 휠홉 대역 오차)');
    wf.a_u = zeros(N, 4);
end
if isfield(wf.meta, 'm_u') && any(abs(wf.meta.m_u(:)' - vp.m_u) > 1e-9)
    warning('validate_i1:MuMismatch', 'I1 m_u [%s] ≠ 블록 3 m_u [%s]', num2str(wf.meta.m_u(:)'), num2str(vp.m_u));
end
if ~isfield(wf, 's')
    wf.s = lap_distance(wf.t, wf.meta);
end
wf.s = wf.s(:);
end

function s = lap_distance(t, meta)
% export_i1 → make_spa_inputs와 같은 s(t) = interp1(t_lap, s_lap, t). Spa만 지원.
if ~strcmpi(meta.track, 'spa') || ~any(strcmp(meta.scenario, {'A', 'B'}))
    error('validate_i1:NoS', 'wf.s 없음, 복원은 track=spa·scenario A/B만 가능 (track=%s, scenario=%s)', meta.track, meta.scenario);
end
root = fullfile(fileparts(mfilename('fullpath')), '..', '..');
csv = fullfile(root, 'data', 'tracks', 'tumftm', struct('A', 'racelines', 'B', 'tracks').(meta.scenario), 'Spa.csv');
[t1, ~, ~, ~, s1] = lap_sim(csv, fullfile(root, 'data', 'raw', 'fastf1_spa.csv'), f82_params(), meta.scenario);
s = interp1(t1, s1, t);
end
