function [A, B, info] = build_state_space(p)
%BUILD_STATE_SPACE 풀카 7DOF Passive 상태공간 행렬 (블록 2). SI, ISO 8855 (x 전방, y 좌측, z 상방).
%   [A, B, info] = BUILD_STATE_SPACE(p)   p: f82_params()
%
%   일반화 좌표 q = [zs θ φ zu_FL zu_FR zu_RL zu_RR] (정적 평형 기준 변위, 중력 제외), 상태 x = [q; q̇] (14).
%   θ: y축 피치 (+ = 노즈 다운), φ: x축 롤 (+ = 좌측 상승). 코너 i 차체 변위 z_b,i = zs − x_i·θ + y_i·φ.
%   입력 u = [F_act(4); z_r(4); ax; ay] (10). ax, ay [m/s^2]는 블록 1 출력 그대로 (ay 좌회전 +).
%     F_act: 휠 기준 액추에이터 힘 (+ = 차체를 밀어 올림). Passive는 0. 액추에이터 기준 환산(÷MR)은 블록 3.
%     차체에 M_θ = −m_s·ax·h_cg, M_φ = m_s·ay·h_cg로 들어감 (함정 1, 롤센터·anti-dive 무시 → 모멘트 전부 스프링이 받음).
%       블록 노트의 M_θ = m_s·ax·h_cg 와 부호가 다른 건 θ 정의(+ = 노즈 다운) 때문. 제동 ax<0 → 노즈 다운.
%
%   스프링·댐퍼는 휠 기준(wheel rate). 승차 주파수 f는 스프링·타이어 직렬(ride rate) 기준으로 역산:
%     k_ride = m_c(2πf)², k = k_ride·kt/(kt − k_ride), c = 2ζ√(k·m_c), m_c = 코너 현가 질량.
%   휠 기준 강성을 p와 무관하게 고정하므로 A, B는 마운트 위치 p(D16)에 의존하지 않는다 — p는 포화·스트로크
%   한계(F_sat,wheel = MR·F_sat,act)로만 들어온다 (W4).
%   스태빌라이저: 축별 휠 기준 강성 k_arb, 에너지 ½·k_arb·(d_L − d_R)² (히브엔 영향 없음). 롤 구배 p.roll_grad와
%   앞 횡하중 이동 비율 p.lltd_f를 정확히 맞추도록 역산 (축 롤 강성 = 서스펜션·타이어 직렬, M_φ = m_s·g·h_cg 기준).
%
%   info: 코너 좌표 x_i, y_i, T (4×3, [zs θ φ] → z_b), 스프링·댐퍼 k, c, 스태빌라이저 k_arb(앞/뒤)·S, 질량·관성, Fz0.
%         Simulink 모델(build_fullcar_model)의 파라미터 veh로 그대로 쓴다.

g = p.g;
a = p.L * p.w_rear;                         % CG → 앞축
b = p.L - a;                                % CG → 뒤축
xi = [a; a; -b; -b];
yi = [p.tf; -p.tf; p.tr; -p.tr] / 2;        % FL FR RL RR
mu = [p.m_uf; p.m_uf; p.m_ur; p.m_ur];
ms = p.m - sum(mu);

Fz0 = p.m * g * [b; b; a; a] / (2 * p.L);   % 정적 타이어 하중 (현가 CG = 차량 CG 가정)
mc = Fz0 / g - mu;                          % 코너 현가 질량
f = p.f_ride([1 1 2 2])';
kride = mc .* (2*pi*f).^2;
k = kride * p.kt ./ (p.kt - kride);
c = 2 * p.zeta * sqrt(k .* mc);

t = [p.tf; p.tr];
Kphi = ms * g * p.h_cg / deg2rad(p.roll_grad) * [p.lltd_f; 1 - p.lltd_f];   % 축 롤 강성 목표 (타이어 포함)
Kphi_t = p.kt * t.^2 / 2;
Kphi_s = Kphi .* Kphi_t ./ (Kphi_t - Kphi);                                 % 서스펜션 몫
k_arb = (Kphi_s - k([1 3]) .* t.^2 / 2) ./ t.^2;
assert(all(k_arb >= 0), 'build_state_space:ARB', '스프링만으로 롤 강성 초과 (k_arb = %s N/m) — roll_grad/lltd_f 확인', mat2str(k_arb', 3));
S = [1 -1 0 0; 0 0 1 -1];                  % 좌우 상대변위 d_L − d_R

T = [ones(4,1), -xi, yi];
G = [T, -eye(4)];                           % 서스펜션 상대변위 d = z_b − z_u = G q
M = diag([ms, p.Iyy, p.Ixx, mu']);
Kt = blkdiag(zeros(3), p.kt * eye(4));
K = G' * (diag(k) + S' * diag(k_arb) * S) * G + Kt;
C = G' * diag(c) * G;

Bq = [G', [zeros(3,4); p.kt * eye(4)], [0 0; -ms*p.h_cg 0; 0 ms*p.h_cg; zeros(4,2)]];
A = [zeros(7), eye(7); -M\K, -M\C];
B = [zeros(7,10); M\Bq];

info = struct('xi', xi, 'yi', yi, 'T', T, 'G', G, 'k', k, 'c', c, 'k_arb', k_arb, 'kt', p.kt, ...
    'S', S, 'ms', ms, 'Iyy', p.Iyy, 'Ixx', p.Ixx, 'h_cg', p.h_cg, 'mu', mu, 'mc', mc, 'Fz0', Fz0, 'a', a, 'b', b);
end
