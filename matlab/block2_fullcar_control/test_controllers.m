%TEST_CONTROLLERS Wk 가중 필터, Skyhook·LQR 이득, Spa B 성능 자체 검사 (실패하면 assert 에러).
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here, '..', 'block1_lap_sim'));
p = f82_params();
[A, B, veh] = build_state_space(p);

% 1) ISO 2631-1 Wk: 표준 표 값 (1/3 옥타브, ×1000)과 1% 이내
f = [0.5 1 2 4 6.3 8 16];  tbl = [418 482 531 967 1054 1036 768] / 1000;
g = squeeze(abs(freqresp(iso2631_wk(), 2*pi*f)))';
fprintf('Wk %s vs 표 %s\n', mat2str(round(1000*g)), mat2str(round(1000*tbl)));
assert(max(abs(g ./ tbl - 1)) < 0.01, 'Wk 필터가 ISO 2631-1 표와 다름');

% 2) Skyhook: F_i = −c_sky·ż_b,i, w 쪽 이득 0
K = controller_skyhook(p, 0.3);
x = randn(14, 1);  c = 2 * 0.3 * sqrt(veh.k .* veh.mc);
assert(norm(-K(:,1:14) * x + c .* (veh.T * x(8:10))) < 1e-9 && ~any(K(:,15:22), 'all'), 'Skyhook 이득 구조 이상');

% 3) LQR: 4×22, 폐루프 안정, 좌우 대칭 (FL↔FR 이득이 롤 부호만 다름)
K = controller_lqr(p, struct('comfort', 1, 'grip', 2));
assert(isequal(size(K), [4 22]) && max(real(eig(A - B(:,1:4) * K(:,1:14)))) < 0, 'LQR 크기·폐루프 안정성 이상');
ay_gain = K(:, 15 + 5);                                    % ay 피드포워드: 좌우 반대 부호
assert(sign(ay_gain(1)) == -sign(ay_gain(2)) && abs(ay_gain(1) + ay_gain(2)) < 1e-6 * max(abs(ay_gain)), 'LQR 좌우 대칭 이상');

% 4) Spa B: LQR(c1 g2)이 Passive보다 승차감 10% 이상, 접지 개선, 스트로크 ±60 mm 이내
w = make_spa_inputs(p, 'B');
m = cell(1, 2);
for i = 1:2
    Kfb = {zeros(4,22), K};
    [t, x, F, Fz] = run_fullcar(p, Kfb{i}, w, 1e-3);
    m{i} = fullcar_metrics(p, t, x, F, Fz, w);
end
fprintf('Spa B: Wk %.3f → %.3f, Fdyn 최대 %.0f → %.0f N, 스트로크 [%.0f %.0f] mm, F 피크 %.0f N\n', m{1}.wk, m{2}.wk, ...
    max(m{1}.Fdyn), max(m{2}.Fdyn), 1e3*min(m{2}.d_min), 1e3*max(m{2}.d_max), max(m{2}.F_peak));
assert(m{2}.wk < 0.9 * m{1}.wk && all(m{2}.Fdyn < m{1}.Fdyn), 'LQR이 Passive보다 낫지 않음');
assert(max(abs([m{2}.d_min m{2}.d_max])) < 0.06, 'LQR 스트로크 과대');
disp('test_controllers OK');
