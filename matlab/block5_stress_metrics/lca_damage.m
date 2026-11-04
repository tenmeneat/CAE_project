function res = lca_damage(wf, p_front, opts)
%LCA_DAMAGE 응력 중첩 → 응력 지표 (블록 5, 인터페이스 I2).
%   res = LCA_DAMAGE(wf, p_front, opts)
%   입력:
%     wf      - 인터페이스 I1 구조체 (블록 2 export_i1). 블록 3을 안에서 호출해 로어암 하중으로 바꾼다
%     p_front - 프런트 마운트 위치 p (D16). 리어는 hardpoints_f82의 고정값 (D18)
%     opts    - (선택) .C_front, .C_rear  응력계수 표 (기본: 블록 4a 빔 해석해 stress_coeffs_beam, lca_section)
%                      .sn      S-N 파라미터 (기본 sn_params('al_forged'), D4 미정)
%                      .Neq     DEL 등가 사이클 수 (기본 1e6 [가정], 팀이 정할 값)
%                      .sec_len 구간 손상 Dsec 구간 길이 [m] (기본 250)
%                      .parts   응력 기여 분해 계산 여부 (기본 true)
%   출력 res (코너 순서 FL FR RL RR, 각 코너는 DEL이 가장 큰 핫스팟 = 지배 핫스팟 기준):
%     D, DEL, sigma_peak   1×4  랩당 손상, 등가응력진폭 [MPa], 피크 |σ_eq| [MPa] (sigma_peak는 전 핫스팟 최대)
%     hot                  1×4  지배 핫스팟 이름,  crit = DEL 최대 코너 (설계 기준 코너)
%     D_hot, DEL_hot, peak_hot  4×H 핫스팟별 값, hot_names
%     sig                  N×4  지배 핫스팟의 부호 있는 von Mises 시계열 [MPa]
%     Dsec, sec_edges      K×4 구간별 손상 (사이클 시작점의 s 기준), 구간 경계 [m]
%     parts.(이름)         .DEL, .peak (1×4) — 하중 채널별로 텐서를 따로 중첩한 지배 핫스팟 지표.
%                          Fx (제동·구동), Fy (횡력), vert_qs (수직 1 s 이동평균: 하중이동·공력·경사),
%                          vert_dyn (수직 나머지: 노면 진동과 제어 효과). 텐서는 합하면 전체와 같고 DEL은 비선형이라
%                          합이 안 맞는다. 액추에이터는 수직 하중을 바꾸는 경로로만 들어온다 (스트럿 축 지지, (나)안)
%     t, s, meta
%   절대 D는 재료 상수에 민감하므로 주 지표가 아니다. R = D/D_passive는 블록 6(A)이 계산한다.

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'block1_lap_sim'), fullfile(here, '..', 'block3_wheel_to_lca_fbd'), ...
    fullfile(here, '..', 'block4a_lca_structure'));
if nargin < 3, opts = struct(); end
vp = block3_params();
hp = hardpoints_f82(p_front);
sec = lca_section();
o = struct('C_front', [], 'C_rear', [], 'sn', sn_params(), 'Neq', 1e6, 'sec_len', 250, 'parts', true);
for f = fieldnames(opts)'
    o.(f{1}) = opts.(f{1});
end
if isempty(o.C_front), o.C_front = stress_coeffs_beam(hp.c(1), hp.p, sec); end
if isempty(o.C_rear),  o.C_rear  = stress_coeffs_beam(hp.c(3), hp.p_rear, sec); end
if abs(o.C_front.p - p_front) > 1e-9
    error('lca_damage:PMismatch', '프런트 응력계수 표 p = %g ≠ p_front = %g', o.C_front.p, p_front);
end

wf = validate_i1(wf, vp);
F = wheel_to_lca_forces(wf, hp, vp);
N = numel(F.t);
H = size(o.C_front.coef, 1);
edges = 0:o.sec_len:(max(F.s) + o.sec_len);
res.D = zeros(1, 4);  res.DEL = zeros(1, 4);  res.sigma_peak = zeros(1, 4);  res.hot = cell(1, 4);
res.D_hot = zeros(4, H);  res.DEL_hot = zeros(4, H);  res.peak_hot = zeros(4, H);
res.sig = zeros(N, 4);
res.Dsec = zeros(numel(edges) - 1, 4);
names = {'Fx', 'Fy', 'vert_qs', 'vert_dyn'};
for n = names
    res.parts.(n{1}) = struct('DEL', zeros(1, 4), 'peak', zeros(1, 4));
end
win = max(1, round(1 / median(diff(F.t))));

for i = 1:4
    if i <= 2, C = o.C_front; else, C = o.C_rear; end
    seq = superpose_stress(F.F_BJ(:, :, i), C);
    mh = cell(1, H);
    for h = 1:H
        mh{h} = fatigue_metrics(seq(:, h), o.sn, o.Neq);
        res.D_hot(i, h) = mh{h}.D;  res.DEL_hot(i, h) = mh{h}.DEL;  res.peak_hot(i, h) = mh{h}.peak;
    end
    [res.DEL(i), g] = max(res.DEL_hot(i, :));
    res.D(i) = res.D_hot(i, g);
    res.sigma_peak(i) = max(res.peak_hot(i, :));
    res.hot{i} = C.hot.name{g};
    res.sig(:, i) = seq(:, g);
    k = discretize(F.s(mh{g}.cyc(:, 4)), edges);
    res.Dsec(:, i) = accumarray(k, mh{g}.d, [numel(edges) - 1, 1]);

    if o.parts
        [Fcp, Fwc] = wheel_load_vector(wf, i, vp);
        f = [Fcp, Fwc];
        vq = movmean(f(:, [3 6]), win);
        for c = 1:4
            fc = zeros(size(f));
            switch names{c}
                case 'Fx',       fc(:, [1 4]) = f(:, [1 4]);
                case 'Fy',       fc(:, 2) = f(:, 2);
                case 'vert_qs',  fc(:, [3 6]) = vq;
                case 'vert_dyn', fc(:, [3 6]) = f(:, [3 6]) - vq;
            end
            Floc = -(fc * F.T(1:3, :, i).') * F.Rloc(:, :, i);
            sp = superpose_stress(Floc, C);
            mp = fatigue_metrics(sp(:, g), o.sn, o.Neq);
            res.parts.(names{c}).DEL(i) = mp.DEL;
            res.parts.(names{c}).peak(i) = mp.peak;
        end
    end
end
[~, res.crit] = max(res.DEL);
res.hot_names = o.C_front.hot.name';
res.sec_edges = edges;
res.t = F.t;  res.s = F.s;
res.meta = struct('p', p_front, 'p_rear', hp.p_rear, 'sn', o.sn, 'Neq', o.Neq, 'C_source', o.C_front.source, ...
    'track', wf.meta.track, 'scenario', wf.meta.scenario, 'ctrl', wf.meta.ctrl);
end
