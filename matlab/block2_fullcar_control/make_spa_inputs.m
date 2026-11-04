function [w, lap] = make_spa_inputs(p, scenario, dt, Gd0)
%MAKE_SPA_INPUTS 블록 1 출력 → 블록 2 입력 w (Spa 1랩, 시나리오 A/B).
%   [w, lap] = MAKE_SPA_INPUTS(p, scenario, dt, Gd0)
%   p: f82_params(), scenario: 'A' | 'B', dt: 균일 시간 간격 [s] (기본 1e-3), Gd0: ISO 8608 (기본 B-C급 128e-6).
%   w: [t zr_FL zr_FR zr_RL zr_RR ax ay] (run_fullcar 입력 형식, build_state_space 입력 5–10과 같은 순서).
%   lap: t, s(CG), v, ax, ay (균일 격자), L, scenario.
%
%   노면 zr = 고저 z(s) + ISO 8608 요철, 좌/우 다른 시드(1/2). 바퀴 위치는 s(t)에서 앞 +a, 뒤 −b로 놓아 후륜
%   지연이 τ = L/v가 아니라 거리로 정확히 들어간다 (가감속 중에도). 좌우 바퀴는 같은 s (코너 내·외측 호 길이 차 무시).
%   고저는 랩 평균을 빼서 넣는다 (4바퀴 공통 상수는 하중에 영향 없음, 수치만 작게).
%   경사의 중력 성분(m·g·sinθ)과 횡단 경사는 무시.

if nargin < 3 || isempty(dt), dt = 1e-3; end
if nargin < 4 || isempty(Gd0), Gd0 = 128e-6; end
root = fullfile(fileparts(mfilename('fullpath')), '..', '..');
i0 = fullfile(root, 'data', 'raw', 'fastf1_spa.csv');
csv = fullfile(root, 'data', 'tracks', 'tumftm', struct('A', 'racelines', 'B', 'tracks').(scenario), 'Spa.csv');

[t1, v1, ax1, ay1, s1] = lap_sim(csv, i0, p, scenario);
trk = load_raceline(csv, 1);
z = align_elevation(trk, i0);
[s_f, zl] = apply_iso8608_roughness(trk, z, Gd0, 1);
[~, zrr] = apply_iso8608_roughness(trk, z, Gd0, 2);
z0 = mean(z);
road = @(zz, sw) interp1([s_f; trk.L], [zz; zz(1)] - z0, mod(sw, trk.L));

t = (0:dt:t1(end))';
s = interp1(t1, s1, t);
[~, ~, info] = build_state_space(p);
sf = s + info.a;  sr = s - info.b;
lap = struct('t', t, 's', s, 'v', interp1(t1, v1, t), 'ax', interp1(t1, ax1, t), ...
    'ay', interp1(t1, ay1, t), 'L', trk.L, 'scenario', scenario);
w = [t, road(zl, sf), road(zrr, sf), road(zl, sr), road(zrr, sr), ...
    lap.ax, lap.ay];
end
