%TEST_FULLCAR_MODEL 물리 서브시스템 Simulink 모델 자체 검사 (실패하면 assert 에러).
%   1) linmod 선형화 = build_state_space (주파수응답·고유값) — 블록 다이어그램과 손 유도 행렬의 독립 교차 검증
%   2) 접촉 유지되는 합성 입력에서 Simulink = lsim (Passive, Skyhook 꼴 피드백)
%   3) 큰 범프: 단방향 접촉이면 Fz ≥ 0이고 이탈 구간에서 0, 선형 모델은 같은 곳에서 음수
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here, '..', 'block1_lap_sim'));
p = f82_params();
[A, B, veh] = build_state_space(p);
build_fullcar_model();                                   % 스크립트가 원본이므로 매번 새로 생성
Bu = B(:,1:4);
Ks = {zeros(4,22), controller_skyhook(p, 0.4), controller_lqr(p)};   % Passive, Skyhook, LQR (w 피드포워드 포함)
Acl = @(K) A - Bu * K(:,1:14);  Bcl = @(K) B(:,5:12) - Bu * K(:,15:22);   % F_act = −K·[x; w]

% 1) linmod
x0 = zeros(14,1);  dt = 1e-3;  t_end = 1;  w_in = [0 zeros(1,8)];   % linmod이 모델 컴파일에 쓰는 변수
%    출력 = 상태 x 전체라 Cl이 linmod 내부 상태 순서 → x 순서 치환. x 좌표로 옮겨 행렬을 직접 비교
for K = Ks
    Kfb = K{1};
    [Al, Bl, Cl, Dl] = linmod('fullcar_7dof');
    eA = norm(Cl * Al / Cl - Acl(Kfb)) / norm(A);
    eB = norm(Cl * Bl - Bcl(Kfb)) / norm(B(:,5:12));
    fprintf('linmod (Kfb max %.0f): A 상대오차 %.1e, B 상대오차 %.1e\n', max(abs(Kfb(:))), eA, eB);
    assert(eA < 1e-10 && eB < 1e-10 && ~any(Dl(:)), 'Simulink 선형화가 build_state_space와 다름');
end

% 2) 합성 입력 (2 cm 범프 + 0.8 g 선회 + 0.5 g 제동): 접촉 유지 → lsim과 일치
t = (0:dt:3)';  v = 20;
bump = @(tt, h) h * sin(pi * min(max(tt * v / 0.5, 0), 1)).^2 .* (tt >= 0 & tt * v <= 0.5);
ramp = min(max((t - 1) / 0.5, 0), 1);
w = [t, bump(t - 0.5, 0.02), 0*t, bump(t - 0.5 - p.L / v, 0.02), 0*t, -0.5 * p.g * ramp, 0.8 * p.g * ramp, ...
     3 * ramp, 2 * ramp];   % 추세 관성 (오목 구간) 포함
for K = Ks
    [ts, xs, Fs, Fz] = run_fullcar(p, K{1}, w, dt);
    xl = lsim(ss(Acl(K{1}), Bcl(K{1}), eye(14), 0), w(:,2:9), t, 'foh');
    xs = interp1(ts, xs, t);
    err = max(abs(xs - xl)) ./ max(max(abs(xl)), 1e-9);
    fprintf('lsim (Kfb max %.0f): 최대 상대오차 %.2e, Fz min %.0f N, 롤 정상값 %.2f deg\n', ...
        max(abs(K{1}(:))), max(err), min(Fz(:)), rad2deg(xs(end, 3)));
    assert(min(Fz(:)) > 0, '이 입력에서는 이탈이 없어야 함');
    assert(max(err) < 1e-3, 'Simulink와 lsim 불일치');
    assert(max(abs(Fs - (-K{1} * [xs, w(:,2:9)]')'), [], 'all') < 1e-6 * max(1, max(abs(Fs(:)))), 'F_act 출력 불일치');
end

% 3) 큰 범프 (FL 8 cm, 30 m/s): 이탈
v = 30;  w = [t, bump(t - 0.5, 0.08), zeros(numel(t), 7)];
[ts, xs, ~, Fz] = run_fullcar(p, zeros(4,22), w, dt);
xl = lsim(ss(A, B(:,5:12), eye(14), 0), w(:,2:9), t, 'foh');
Fz_lin = veh.Fz0(1) + veh.kt * (w(:,2) - xl(:,4));
fprintf('큰 범프: 단방향 Fz_FL min %.0f N (이탈 %.0f ms), 선형 Fz_FL min %.0f N\n', ...
    min(Fz(:,1)), 1e3 * dt * sum(Fz(:,1) == 0), min(Fz_lin));
assert(min(Fz(:)) >= 0 && any(Fz(:,1) == 0) && min(Fz_lin) < 0, '단방향 접촉 동작 이상');
disp('test_fullcar_model OK');
