function F_LCA = wheel_to_lca_forces(wf, hp, vp)
%WHEEL_TO_LCA_FORCES 휠력 → 로어암 하중 변환 (블록 3, 준정적 FBD).
%   F_LCA = WHEEL_TO_LCA_FORCES(wf, hp, vp)
%   입력:
%     wf - 인터페이스 I1 구조체 (블록3-휠력-로어암하중.md 4절, 블록 2 병합 전 가정 스키마 [동기화])
%     hp - hardpoints_f82(p_front). 프런트 p는 hp.p, 리어는 hp.p_rear (D18)
%     vp - block3_params()
%   출력 (좌측 등가 좌표, 코너 순서 FL FR RL RR):
%     t, s       - wf에서 그대로
%     F_BJ       - N×3×4 LBJ에서 로어암이 받는 힘 −F_L, 로어암 로컬 좌표 [N]
%     F_M        - N×4 마운트에서 로어암이 받는 스트럿 힘 크기 (압축 +). 로어암에는 −F_M·u_s_loc
%     u_s_loc    - 3×4 스트럿 단위벡터 (마운트 → 차체), 로컬 좌표
%     F_act_act  - N×4 액추에이터 실제 힘 = F_act / MR
%     resid_Fs   - N×4 F_M − (F_spring + F_damper + F_act)/MR  (anti-dive·롤센터 재킹 등)
%     MR         - 1×4 정역학 모션비
%     F_UBJ, f_T - N×3×4 UBJ에서 상부암이 받는 힘 −F_U (차량 좌표), N×4 타이로드 축력 (인장 +). 진단용
%     T          - 8×6×4 영향 행렬 x = T·[F_cp; F_wc]
%     Rloc, meta - 3×3×4 로어암 로컬 축 [e1 e2 e3], struct (p, p_rear, scn, ctrl, hp_version, geo)

if isfield(wf, 'p_front') && abs(wf.p_front - hp.p) > 1e-9
    error('wheel_to_lca_forces:PMismatch', 'wf.p_front = %g 이지만 hp.p = %g', wf.p_front, hp.p);
end
if ~isfield(wf, 'zdd_u')
    warning('wheel_to_lca_forces:NoZddU', 'wf.zdd_u 없음 → 0으로 둠 (준정적 근사, 휠홉 대역 오차)');
end
if any(wf.Fx(:, 1:2) > 0, 'all')
    warning('wheel_to_lca_forces:FrontDrive', '프런트에 구동 Fx > 0 — F82는 후륜구동. 휠센터 작용으로 처리');
end

N = numel(wf.t);
F_LCA.t = wf.t(:);
F_LCA.s = wf.s(:);
F_LCA.F_BJ = zeros(N, 3, 4);
F_LCA.F_UBJ = zeros(N, 3, 4);
F_LCA.F_M = zeros(N, 4);
F_LCA.f_T = zeros(N, 4);
F_LCA.u_s_loc = zeros(3, 4);
F_LCA.MR = zeros(1, 4);
F_LCA.T = zeros(8, 6, 4);
F_LCA.Rloc = zeros(3, 3, 4);

for i = 1:4
    hc = hp.c(i);
    geo = lca_geometry(hc, hc.p);
    [A, G] = build_fbd_matrix(hc, geo);
    T = A \ G;
    [Fcp, Fwc] = wheel_load_vector(wf, i, vp);
    X = [Fcp, Fwc] * T.';                         % N×8, 시간축 전체를 한 번에

    F_LCA.F_BJ(:, :, i) = -X(:, 1:3) * geo.Rloc;  % 행벡터 기준 로컬 변환 = (Rloc'·F)'
    F_LCA.F_UBJ(:, :, i) = -X(:, 4:6);
    F_LCA.f_T(:, i) = X(:, 7);
    F_LCA.F_M(:, i) = X(:, 8);
    F_LCA.u_s_loc(:, i) = geo.Rloc' * geo.u_s;
    F_LCA.MR(i) = 1 / T(8, 3);                    % = motion_ratio(hc, geo)
    F_LCA.T(:, :, i) = T;
    F_LCA.Rloc(:, :, i) = geo.Rloc;
    meta_geo(i) = geo; %#ok<AGROW>
end

F_LCA.F_act_act = wf.F_act ./ F_LCA.MR;
F_LCA.resid_Fs = F_LCA.F_M - (wf.F_spring + wf.F_damper + wf.F_act) ./ F_LCA.MR;

if isfield(wf, 'MR') && any(abs(wf.MR(:)' - F_LCA.MR) > 1e-3)
    warning('wheel_to_lca_forces:MRMismatch', 'wf.MR [%s] ≠ 블록 3 MR [%s]', ...
        num2str(wf.MR(:)', '%.4f '), num2str(F_LCA.MR, '%.4f '));
end

F_LCA.meta = struct('p', hp.p, 'p_rear', hp.p_rear, 'scn', getf(wf, 'scn'), 'ctrl', getf(wf, 'ctrl'), ...
    'hp_version', hp.version, 'geo', meta_geo);
end

function v = getf(s, f)
if isfield(s, f), v = s.(f); else, v = ''; end
end
