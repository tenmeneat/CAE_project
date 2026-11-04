function wf = export_i1(scn, tag, Kfb, p, out_dir)
%EXPORT_I1 인터페이스 I1 쓰기: wheel_forces_spa_<scn>_<tag>.mat (블록 2 → 블록 3). 변수 이름은 wf.
%   wf = EXPORT_I1(scn, tag, Kfb, p, out_dir)   한 조합. scn 'A'|'B', tag 파일명용 제어기 이름, Kfb run_fullcar 이득
%   EXPORT_I1()                                  기본 세트 6개: 시나리오 A/B × passive / skyhook(ζ 0.3) / lqr
%                                                (lqr = 모드별 대표값 A: c1 g4, B: c1 g2)
%   out_dir 기본 <repo>/outputs/i1 (.mat은 git 제외 — B는 이 함수를 돌려 만든다).
%
%   wf 필드 (SI, ISO 8855, 코너 순서 FL FR RL RR, 1 ms 균일, N 샘플):
%     t          N×1
%     F_wc       N×4×3  타이어가 바퀴에 주는 힘 [Fx Fy Fz] (접지점 힘, 휠센터로 옮길 때 생기는 Fx·r 모멘트는 미포함)
%                Fz: Simulink 단방향 접촉 (전체 하중, 정적 포함). Fx, Fy: 총 힘을 Fz 비례로 나눔 —
%                  ΣFy = m·ay,  ΣFx = m·ax + ½ρCdA·v² + m·g·sinθ;  ΣFx > 0(구동)이면 후륜만 (F82 후륜구동)
%     F_act      N×4    액추에이터 힘 (휠 기준, + = 서스펜션을 늘림 = 차체를 밀어 올림). 액추에이터 기준은 ÷MR(p) — 블록 3
%     F_spring   N×4    스프링 힘 (휠 기준, + 같음, 정적 하중 m_c·g 포함)
%     F_damper   N×4    댐퍼 힘 (휠 기준)
%     F_arb      N×4    스태빌라이저 힘 (휠 기준)
%     d, d_dot   N×4    서스펜션 변위·속도 (z_b − z_u, − = 압축)
%     a_u        N×4    비현가 질량 절대 수직 가속도 (Simulink 내부 z̈u + 추세 가속도) — 너클 평형 잔차용:
%                       Fz − (F_spring + F_damper + F_act + F_arb) − m_u·g = m_u·a_u
%     a_body     N×3    차체 절대 가속도 [z̈s θ̈ φ̈]
%     mu_use     N×4    마찰 사용률 √(Fx² + Fy²)/Fz (Fz < 1 N이면 NaN). 블록 1은 μ·g_n 한계(최대 1.1 × 1.2)로 달리는데
%                       동적 하중 감소 순간엔 이보다 커진다 — 자르지 않는다 (자르면 ΣF = m·a가 깨짐). 판단은 03-결정사항
%     metrics    fullcar_metrics 결과 (3축 지표)
%     meta       track, scenario, ctrl, Kfb, m_u, m_c, mount_p (NaN: 아직 p 미반영 — A, B가 p와 무관하고 포화 없음),
%                created, note

if nargin == 0
    p = f82_params();
    for scn = 'AB'
        rho = struct('A', struct('comfort', 1, 'grip', 4), 'B', struct('comfort', 1, 'grip', 2)).(scn);
        ctrl = {'passive', zeros(4, 22); 'skyhook', controller_skyhook(p, 0.3); 'lqr', controller_lqr(p, rho)};
        for i = 1:3, export_i1(scn, ctrl{i, 1}, ctrl{i, 2}, p); end
    end
    wf = [];
    return
end
if nargin < 4 || isempty(p), p = f82_params(); end
if nargin < 5 || isempty(out_dir), out_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'outputs', 'i1'); end

[w, lap] = make_spa_inputs(p, scn);
[t, x, F_act, Fz, zudd] = run_fullcar(p, Kfb, w, 1e-3);
[A, B, veh] = build_state_space(p);
g = p.g;

d = (veh.G * x(:, 1:7)')';
d_dot = (veh.G * x(:, 8:14)')';
F_spring = veh.mc' * g - d .* veh.k';
F_damper = -d_dot .* veh.c';
F_arb = -(veh.S' * diag(veh.k_arb) * veh.S * d')';
zdd_ref = w(:, [8 8 9 9]);
a_u = zudd + zdd_ref;
a_body = (A(8:10, :) * x' + B(8:10, :) * [F_act, w(:, 2:9)]')' + (veh.Jref(1:3, :) * w(:, 8:9)')';

Fx_tot = p.m * lap.ax + 0.5 * p.rho * p.CdA * lap.v.^2 + p.m * g * lap.grade ./ sqrt(1 + lap.grade.^2);
Fy_tot = p.m * lap.ay;
share = @(F) F ./ max(sum(F, 2), eps);
drive = Fx_tot > 0;
wx = share(Fz);  wx(drive, :) = share(Fz(drive, :) .* [0 0 1 1]);
F_wc = cat(3, Fx_tot .* wx, Fy_tot .* share(Fz), Fz);
mu_use = hypot(F_wc(:,:,1), F_wc(:,:,2)) ./ Fz;
mu_use(Fz < 1) = NaN;

wf = struct('t', t, 'F_wc', F_wc, 'F_act', F_act, 'F_spring', F_spring, 'F_damper', F_damper, 'F_arb', F_arb, ...
    'd', d, 'd_dot', d_dot, 'a_u', a_u, 'a_body', a_body, 'mu_use', mu_use, 'metrics', fullcar_metrics(p, t, x, F_act, Fz, w), ...
    'meta', struct('track', 'spa', 'scenario', scn, 'ctrl', tag, 'Kfb', Kfb, 'm_u', veh.mu', 'm_c', veh.mc', ...
        'mount_p', NaN, 'created', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm')), ...
        'note', 'SI, ISO 8855, FL FR RL RR. 휠 기준 힘. 포화·지연 없음 (W4). 필드 설명은 export_i1.m 도움말'));
if ~isfolder(out_dir), mkdir(out_dir); end
f = fullfile(out_dir, sprintf('wheel_forces_spa_%s_%s.mat', scn, tag));
save(f, 'wf', '-v7');
fprintf('I1 %s: %d 샘플, %.1f MB\n', f, numel(t), dir(f).bytes / 1e6);
end
