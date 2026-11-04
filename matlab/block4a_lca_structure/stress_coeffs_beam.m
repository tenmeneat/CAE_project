function C = stress_coeffs_beam(hc, p, sec)
%STRESS_COEFFS_BEAM 응력계수 표 C — A형 암 등가 빔 해석해 (블록 4a 대용·검증용).
%   C = STRESS_COEFFS_BEAM(hc, p, sec)   hc = hardpoints_f82(p).c(i), sec = lca_section()
%   COMSOL 표(read_stress_coeffs_csv)와 같은 형식. COMSOL 결과가 나오면 교체하고, 이 값은 4a의 "곡선빔 해석해"
%   3중 비교(솔리드·빔·해석해)의 해석해로 쓴다.
%
%   C.coef  H×6×3 [Pa/N] 핫스팟 h의 응력텐서 [s11 s22 s33 s12 s23 s13] (로어암 로컬), LBJ 로컬 단위하중 k = x, y, z
%   C.hot   table: name, s (힌지축 수선의 발에서 LBJ 쪽 거리), x (e1), z (e3)
%
%   모델: 축 = r_foot → LBJ (e2), 길이 L. 앞·뒤 부재는 같은 사각 중공 단면, 부재 간격 w(s) = w0·(1 − s/L).
%     지지: 힌지축(e1) 둘레 회전 자유 → 수직 굽힘 M1(0) = 0. 면내 굽힘·비틀림은 부싱 2점이 받음 (s = 0 고정).
%           마운트 s = pL은 스트럿 축 u_s 방향만 지지, 반력 R = F_z/(p·u_s3) — 블록 3 F_M과 같음.
%     응력: σ22 = N/A − M1·z/I1 + M3·x/I3, I3 = 2(I3m + Am·(w/2)²) (면내는 두 부재의 짝힘). 모서리 섬유라 전단 0.
%     LBJ와 마운트가 e2 축 위에 있어 비틀림 없음.
%   핫스팟 8개: {마운트 단면 M (s = pL, 바깥쪽), 뿌리 R (s → 0)} × {앞 F, 뒤 B 부재 바깥 모서리} × {위 T, 아래 U}.

geo = lca_geometry(hc, p);
L = norm(hc.LBJ - geo.r_foot);
w0 = abs(dot(hc.LCA_F - hc.LCA_R, geo.e_L));
u = geo.Rloc' * geo.u_s;

bi = sec.b - 2*sec.tw;  hi = sec.h - 2*sec.tw;
Am = sec.b*sec.h - bi*hi;
I1m = (sec.b*sec.h^3 - bi*hi^3) / 12;
I3m = (sec.h*sec.b^3 - hi*bi^3) / 12;

[S, X, Zf] = ndgrid([p*L, 0], [1, -1], [1, -1]);      % 단면 × 앞/뒤 × 위/아래
S = S(:);  sgx = X(:);  Zf = Zf(:) * sec.h/2;
tag = {'M', 'R'};  fb = 'FB';  tu = 'TU';
H = numel(S);
names = cell(H, 1);  xh = zeros(H, 1);
coef = zeros(H, 6, 3);
for h = 1:H
    names{h} = sprintf('%s_%s%s', tag{1 + (S(h) == 0)}, fb(1 + (sgx(h) < 0)), tu(1 + (Zf(h) < 0)));
    w = w0 * (1 - S(h)/L);
    xh(h) = sgx(h) * (w + sec.b) / 2;
    I3 = 2 * (I3m + Am * (w/2)^2);
    for k = 1:3
        F = zeros(3, 1);  F(k) = 1;
        loads = {L, F};                               % {위치 s, 힘(로컬)}
        if S(h) < p*L
            loads(end+1, :) = {p*L, -F(3) / (p*u(3)) * u};   % 스트럿 반력 −R·u_s
        end
        N = 0;  M = zeros(3, 1);
        for j = 1:size(loads, 1)
            N = N + loads{j, 2}(2);
            M = M + cross([0; loads{j, 1} - S(h); 0], loads{j, 2});
        end
        coef(h, 2, k) = N/(2*Am) - M(1)*Zf(h)/(2*I1m) + M(3)*xh(h)/I3;
    end
end

C.coef = coef;
C.comp = {'s11', 's22', 's33', 's12', 's23', 's13'};
C.cases = {'BJ_x', 'BJ_y', 'BJ_z'};
C.hot = table(names, S, xh, Zf, 'VariableNames', {'name', 's', 'x', 'z'});
C.p = p;
C.source = 'beam_surrogate';
C.meta = struct('sec', sec, 'L', L, 'w0', w0, 'R_per_Fz', 1/(p*u(3)));
end
