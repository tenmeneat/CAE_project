function [w, lap] = make_spa_inputs(p, scenario, dt, Gd0)
%MAKE_SPA_INPUTS 블록 1 출력 → 블록 2 입력 w (Spa 1랩, 시나리오 A/B).
%   [w, lap] = MAKE_SPA_INPUTS(p, scenario, dt, Gd0)
%   p: f82_params(), scenario: 'A' | 'B', dt: 균일 시간 간격 [s] (기본 1e-3), Gd0: ISO 8608 (기본 B-C급 128e-6).
%   w: [t r_FL r_FR r_RL r_RR ax ay z̈_ref,f z̈_ref,r] (run_fullcar 입력 형식, build_state_space 입력 5–12 순서).
%   lap: t, s(CG), v, ax, ay, grade (CG 위치 tanθ) (균일 격자), L, scenario.
%
%   노면 추세 좌표계 (build_state_space 참고): 평활 고저 z(s)는 변위로 넣지 않고 바퀴 위치의 수직 가속도
%   z̈_ref = v²·z''(s) + ax·z'(s) 로만 넣는다 (z'' = κv, 25 m 평활 — 블록 1 align_elevation). 변위 입력 r은
%   ISO 8608 요철만, 좌/우 다른 시드(1/2). 바퀴 위치는 s(t)에서 앞 +a, 뒤 −b로 놓아 후륜 지연이 τ = L/v가 아니라
%   거리로 정확히 들어간다 (가감속 중에도). 좌우 바퀴는 같은 s (코너 내·외측 호 길이 차 무시).
%   경사의 중력 성분(m·g·sinθ)과 횡단 경사는 무시.

if nargin < 3 || isempty(dt), dt = 1e-3; end
if nargin < 4 || isempty(Gd0), Gd0 = 128e-6; end
root = fullfile(fileparts(mfilename('fullpath')), '..', '..');
i0 = fullfile(root, 'data', 'raw', 'fastf1_spa.csv');
csv = fullfile(root, 'data', 'tracks', 'tumftm', struct('A', 'racelines', 'B', 'tracks').(scenario), 'Spa.csv');

[t1, v1, ax1, ay1, s1] = lap_sim(csv, i0, p, scenario);
trk = load_raceline(csv, 1);
[z, theta, kv] = align_elevation(trk, i0);
[s_f, ~, rl] = apply_iso8608_roughness(trk, z, Gd0, 1);
[~, ~, rr] = apply_iso8608_roughness(trk, z, Gd0, 2);
at = @(s, v, sw) interp1([s; trk.L], [v; v(1)], mod(sw, trk.L));     % 닫힌 루프 보간

t = (0:dt:t1(end))';
s = interp1(t1, s1, t);
[~, ~, info] = build_state_space(p);
sf = s + info.a;  sr = s - info.b;
lap = struct('t', t, 's', s, 'v', interp1(t1, v1, t), 'ax', interp1(t1, ax1, t), ...
    'ay', interp1(t1, ay1, t), 'grade', tan(at(trk.s, theta, s)), 'L', trk.L, 'scenario', scenario);
zdd = @(sw) lap.v.^2 .* at(trk.s, kv, sw) + lap.ax .* tan(at(trk.s, theta, sw));
w = [t, at(s_f, rl, sf), at(s_f, rr, sf), at(s_f, rl, sr), at(s_f, rr, sr), ...
    lap.ax, lap.ay, zdd(sf), zdd(sr)];
end
