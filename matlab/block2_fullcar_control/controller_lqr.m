function [Kfb, info] = controller_lqr(p, rho, v_design, f_load)
%CONTROLLER_LQR LQR 제어기 (블록 2).  F_act = −Kfb·[x; w]   (Kfb 4×22: 상태 14 + 입력 w 8 = 요철 4·ax·ay·z̈_ref 2)
%   [Kfb, info] = CONTROLLER_LQR(p, rho, v_design, f_load)
%   rho: 가중 struct .comfort, .grip, .stroke (기본 1, 1, 1).  v_design: 노면 모델 설계 속도 [m/s] (기본 40)
%   f_load: 하중 외란 모델 대역 [Hz] (기본 0.5)
%
%   비용  J = ∫ ρc·Σ(a_b/1 m/s²)² + ρg·Σ(ΔFz/1 kN)² + ρd·Σ(d/50 mm)² + Σ(F_act/3.7 kN)² dt
%     a_b = [z̈s, θ̈·L/2, φ̈·t_f/2]  차체 가속도 (노면 추세 기준, 길이 환산) — 운전석 하나보다 차체 전체를 본다
%     ΔFz = kt·(r − zu) − ΔFz_qs(l)  타이어 동하중 (Passive 준정적 분배를 뺀 것 — fullcar_metrics와 같은 정의)
%     d = z_b − z_u  스트로크,  F_act 기준 3.7 kN = Audi A8 포화 (D15)
%   정규화 기준(Bryson)은 고정하고 ρ만 바꾼다 — 블록 6 스윕의 격자 변수.
%
%   노면 모델 증강: 타이어 동하중은 노면 r에 의존하므로 r을 상태로 넣는다. ISO 8608 PSD ∝ n⁻²는 적분된 백색잡음이고
%   대역 하한 n_lo = 0.011 cycle/m를 1차 필터로 두면  ṙ = −2π·n_lo·v·r + 잡음  (각 바퀴 독립, 후륜 지연 무시).
%   r 모드는 제어 불가지만 안정이라 LQR이 풀린다. r을 0으로 두고 zu만 벌하면 바퀴가 노면을 따라가는 것까지
%   벌해서 수십 kN·수백 mm가 나온다 (2026-11-03 확인). r 피드백은 실차의 타이어 변형(휠 가속도·하중) 추정에 해당.
%   하중 외란 증강: l = [ax ay z̈_ref,f z̈_ref,r]도  l̇ = −2π·f_load·l + 잡음  상태로 넣는다. 없으면 LQR이 노면만 보고
%   서스펜션을 무르게 만들어 제동·선회·Eau Rouge 압축에서 스트로크가 수백 mm가 된다. 결과로 ax·ay·z̈_ref
%   피드포워드 이득이 생기는데, 양산 액티브 서스펜션의 가속도 피드포워드(롤·피치 보상)에 해당한다.
%   가속도가 u에 직접 의존하므로 교차항 N: z = Cz·xa + Dz·u, Q = Cz'WCz, R = Dz'WDz + Wu, N = Cz'WDz.
%   info: Q, R, N, 폐루프 고유값 eig_cl (증강계), rho, v_design, f_load.

if nargin < 2, rho = struct(); end
if nargin < 3 || isempty(v_design), v_design = 40; end
if nargin < 4 || isempty(f_load), f_load = 0.5; end
r = struct('comfort', 1, 'grip', 1, 'stroke', 1);
for f = fieldnames(rho)', r.(f{1}) = rho.(f{1}); end

[A, B, veh] = build_state_space(p);
Bu = B(:, 1:4);  Bw = B(:, 5:12);
Aa = [A, Bw; zeros(8, 14), -2*pi*diag([0.011*v_design*ones(1,4), f_load*ones(1,4)])];
Ba = [Bu; zeros(8, 4)];
Fqs = veh.kt * [zeros(4,3), eye(4), zeros(4,7)] * (A \ B(:, 9:12));   % Passive 준정적 타이어 하중 / l
Sb = diag([1, p.L/2, p.tf/2]);
Cz = [Sb * A(8:10, :), Sb * Bw(8:10, :);
      zeros(4, 3), -veh.kt * eye(4), zeros(4, 7), veh.kt * eye(4), -Fqs;
      veh.G, zeros(4, 7), zeros(4, 8)];
Dz = [Sb * Bu(8:10, :); zeros(8, 4)];
W = blkdiag(r.comfort * eye(3), r.grip / 1e3^2 * eye(4), r.stroke / 0.05^2 * eye(4));
Wu = eye(4) / 3.7e3^2;

Q = Cz' * W * Cz;  R = Dz' * W * Dz + Wu;  N = Cz' * W * Dz;
Kfb = lqr(Aa, Ba, Q, R, N);
info = struct('Q', Q, 'R', R, 'N', N, 'eig_cl', eig(Aa - Ba * Kfb), 'rho', r, 'v_design', v_design, 'f_load', f_load);
end
