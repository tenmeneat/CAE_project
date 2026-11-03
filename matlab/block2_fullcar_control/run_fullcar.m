function [t, x, F_act, Fz, zudd] = run_fullcar(p, Kfb, w, dt)
%RUN_FULLCAR fullcar_7dof.slx 실행 (블록 2). 모델이 없으면 build_fullcar_model로 만든다.
%   [t, x, F_act, Fz, zudd] = RUN_FULLCAR(p, Kfb, w, dt)
%   p: f82_params(). Kfb: 4×22, F_act = −Kfb·[x; w(2:9)] (Passive = zeros(4,22), controller_skyhook, controller_lqr).
%   w: [t r_FL r_FR r_RL r_RR ax ay z̈_ref,f z̈_ref,r] (N×9, build_state_space 입력 5–12 순서, make_spa_inputs 출력).
%   dt: 고정 스텝 [s] (ode4). 초기 상태는 첫 입력 w(1,:)에 대한 선형 정상상태 (랩 도중 시작해도 가짜 계단 없음).
%   출력: t (M×1), x (M×14, [zs θ φ zu(4) 속도 7], 노면 추세 기준), F_act (M×4, 휠 기준), Fz (M×4, 전체 타이어 하중, 단방향 접촉),
%         zudd (M×4, 비현가 질량 가속도, 노면 추세 기준 — Simulink 내부 값).

mdl = 'fullcar_7dof';
if ~bdIsLoaded(mdl)
    if isfile(fullfile(fileparts(mfilename('fullpath')), [mdl '.slx'])), load_system(mdl); else, build_fullcar_model(); end
end
[A, B, veh] = build_state_space(p);
Bu = B(:, 1:4);
x0 = -(A - Bu * Kfb(:, 1:14)) \ ((B(:, 5:12) - Bu * Kfb(:, 15:22)) * w(1, 2:9)');
in = Simulink.SimulationInput(mdl);
vars = {'veh', veh; 'Kfb', Kfb; 'x0', x0; 'w_in', w; 'dt', dt; 't_end', w(end, 1)};
for i = 1:size(vars, 1), in = in.setVariable(vars{i, :}); end
out = sim(in);
t = out.tout;  x = out.x;  F_act = out.F_act;  Fz = out.Fz;  zudd = out.zudd;
end
