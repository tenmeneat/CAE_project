function wf = make_dummy_i1(scn, opts)
%MAKE_DUMMY_I1 Simulink 없이 블록 3·5를 빠르게 돌리는 더미 I1 (준정적 하중이동, Passive). 형식은 export_i1과 같음.
%   wf = MAKE_DUMMY_I1(scn, opts)
%   scn  - 'A' | 'B' (블록 1 lap_sim 시나리오)
%   opts - (선택) .p_front  프런트 p (기본 0.65, D16 임시값)
%                 .dt       리샘플 간격 [s] (기본 0.01)
%                 .lap      struct(t, v, ax, ay, s) — 주면 lap_sim 대신 사용 (테스트용)
%                 .road     ISO 8608 성분 (미구현 — 쿼터카 파라미터가 D3에서 정해지면 추가)
%
%   Fz_i = 정적 + 종방향 이동(m·ax·h/L, 앞뒤 ±½) + 횡방향 이동(λ·m·ay·h/t_f 앞, (1−λ)·m·ay·h/t_r 뒤),
%          λ = f82_params().lltd_f (블록 2와 같은 앞 비율)
%   Fy_i = m·ay·Fz_i/ΣFz.  Fx 합 = m·ax + 항력 — 제동은 Fz 비례 네 바퀴, 구동은 후륜 둘에만.
%   F_spring = Fz − m_u·g (휠 기준), F_damper = F_act = F_arb = 0, a_u = 0. s는 포함 (export_i1은 아직 없음).
%   meta.mount_p = p_front, meta.MR = 블록 3 MR.

if nargin < 2, opts = struct(); end
p_front = getopt(opts, 'p_front', 0.65);
dt = getopt(opts, 'dt', 0.01);
if getopt(opts, 'road', false)
    error('make_dummy_i1:RoadNotImplemented', 'opts.road는 아직 미구현 (쿼터카 파라미터 D3 미정)');
end

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'block1_lap_sim'));
cp = f82_params();
vp = block3_params();
hp = hardpoints_f82(p_front);

if isfield(opts, 'lap')
    lap = opts.lap;
else
    root = fullfile(here, '..', '..');
    tum = fullfile(root, 'data', 'tracks', 'tumftm');
    csv = struct('A', fullfile(tum, 'racelines', 'Spa.csv'), 'B', fullfile(tum, 'tracks', 'Spa.csv'));
    [lap.t, lap.v, lap.ax, lap.ay, lap.s] = lap_sim(csv.(scn), fullfile(root, 'data', 'raw', 'fastf1_spa.csv'), cp, scn);
end

t = (0:dt:lap.t(end))';
rs = @(y) interp1(lap.t, y, t);
v = rs(lap.v);  ax = rs(lap.ax);  ay = rs(lap.ay);

m = cp.m;  h = cp.h_cg;  g = cp.g;
Fz0 = m * g / 2 * [1 - cp.w_rear, 1 - cp.w_rear, cp.w_rear, cp.w_rear];
dlong = m * ax * h / cp.L / 2;
dlat_f = vp.lltd_f * m * ay * h / vp.tf;
dlat_r = (1 - vp.lltd_f) * m * ay * h / vp.tr;
Fz = Fz0 + [-dlong - dlat_f, -dlong + dlat_f, dlong - dlat_r, dlong + dlat_r];   % ay > 0 (좌회전) → 우측 증가
if any(Fz < 0, 'all')
    warning('make_dummy_i1:WheelLift', '바퀴 들림 %d 샘플 → Fz = 0으로 자름', nnz(any(Fz < 0, 2)));
    Fz = max(Fz, 0);
end
wz = Fz ./ sum(Fz, 2);

Fy = m * ay .* wz;
Fx_tot = m * ax + 0.5 * cp.rho * cp.CdA * v.^2;
Fx = zeros(size(Fz));
br = Fx_tot < 0;
Fx(br, :) = Fx_tot(br) .* wz(br, :);
Fx(~br, 3:4) = Fx_tot(~br) .* Fz(~br, 3:4) ./ sum(Fz(~br, 3:4), 2);

MR = zeros(1, 4);
for i = 1:4
    MR(i) = motion_ratio(hp.c(i), lca_geometry(hp.c(i), hp.c(i).p));
end
Z = zeros(size(Fz));
wf = struct('t', t, 's', rs(lap.s), 'F_wc', cat(3, Fx, Fy, Fz), 'F_spring', Fz - vp.m_u * g, ...
    'F_damper', Z, 'F_act', Z, 'F_arb', Z, 'a_u', Z, ...
    'meta', struct('track', 'spa', 'scenario', scn, 'ctrl', 'dummy_qs', 'm_u', vp.m_u, 'mount_p', p_front, 'MR', MR));
end

function v = getopt(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end
