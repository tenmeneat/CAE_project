function [t, x, F_act, Fz] = run_fullcar(p, Kfb, w, dt)
%RUN_FULLCAR fullcar_7dof.slx 실행 (블록 2). 모델이 없으면 build_fullcar_model로 만든다.
%   [t, x, F_act, Fz] = RUN_FULLCAR(p, Kfb, w, dt)
%   p: f82_params(). Kfb: 4×14 상태 피드백 (Passive = zeros(4,14)).
%   w: [t zr_FL zr_FR zr_RL zr_RR ax ay] (N×7, build_state_space 입력 5–10과 같은 순서).
%   dt: 고정 스텝 [s] (ode4). 초기 상태는 첫 입력 w(1,:)에 대한 선형 정상상태 (랩 도중 시작해도 가짜 계단 없음).
%   출력: t (M×1), x (M×14, [zs θ φ zu(4) 속도 7]), F_act (M×4, 휠 기준), Fz (M×4, 전체 타이어 하중, 단방향 접촉).

mdl = 'fullcar_7dof';
if ~bdIsLoaded(mdl)
    if isfile(fullfile(fileparts(mfilename('fullpath')), [mdl '.slx'])), load_system(mdl); else, build_fullcar_model(); end
end
[A, B, veh] = build_state_space(p);
x0 = -(A - B(:, 1:4) * Kfb) \ (B(:, 5:10) * w(1, 2:7)');
in = Simulink.SimulationInput(mdl);
vars = {'veh', veh; 'Kfb', Kfb; 'x0', x0; 'w_in', w; 'dt', dt; 't_end', w(end, 1)};
for i = 1:size(vars, 1), in = in.setVariable(vars{i, :}); end
out = sim(in);
t = out.tout;  x = out.x;  F_act = out.F_act;  Fz = out.Fz;
end
