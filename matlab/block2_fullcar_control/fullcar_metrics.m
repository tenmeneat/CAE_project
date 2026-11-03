function m = fullcar_metrics(p, t, x, F_act, Fz, w)
%FULLCAR_METRICS 블록 2 3축 지표 (01-아키텍처 평가 지표: 접지·승차감·액추에이터 부담).
%   m = FULLCAR_METRICS(p, t, x, F_act, Fz, w)   입력은 run_fullcar 출력과 그 입력 w.
%   승차감  wk      운전석(p.seat) 절대 수직 가속도의 ISO 2631-1 Wk 가중 RMS [m/s^2]. 절대 = 상대 + 노면 추세(q̈_ref).
%           a_rms   같은 가속도의 무가중 RMS
%   접지    Fdyn    코너별 타이어 동하중 RMS [N] = Fz − 준정적 하중. 준정적 하중은 Passive가 ax, ay, z̈_ref에 대해
%                   갖는 정상상태 분배 (제어기가 정적 분배를 바꾸면 그것도 동하중으로 잡힌다)
%           lift    코너별 이탈(Fz = 0) 시간 비율
%   스트로크 d_min, d_max, d_rms  코너별 서스펜션 변위 d = z_b − z_u [m] (− = 압축)
%   액추에이터 F_rms, F_peak [N] (휠 기준), P_mean  평균 양의 기계 출력 Σ max(F·ḋ, 0) [W] (회생 없음)
%   지표는 모두 1×4 (FL FR RL RR) 또는 스칼라.

[A, B, veh] = build_state_space(p);
u = [F_act, w(:, 2:9)];
qdd = (A(8:10, :) * x' + B(8:10, :) * u')' + (veh.Jref(1:3, :) * w(:, 8:9)')';
a_seat = qdd(:, 1) - p.seat(1) * qdd(:, 2) + p.seat(2) * qdd(:, 3);
aw = lsim(iso2631_wk(), a_seat, t);

x_qs = -A \ (B(:, 9:12) * w(:, 6:9)');                       % Passive 준정적 상태
Fz_qs = veh.Fz0' - veh.kt * x_qs(4:7, :)';
d = (veh.G * x(:, 1:7)')';
dd = (veh.G * x(:, 8:14)')';

m = struct('wk', rms(aw), 'a_rms', rms(a_seat), 'Fdyn', rms(Fz - Fz_qs), 'lift', mean(Fz == 0), ...
    'd_min', min(d), 'd_max', max(d), 'd_rms', rms(d), ...
    'F_rms', rms(F_act), 'F_peak', max(abs(F_act)), 'P_mean', mean(sum(max(F_act .* dd, 0), 2)));
end
