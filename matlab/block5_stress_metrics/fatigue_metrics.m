function m = fatigue_metrics(sig, sn, Neq)
%FATIGUE_METRICS 등가응력 시계열 → 응력 지표 (블록 5). sig [MPa], sn = sn_params(), Neq 등가 사이클 수.
%   m = FATIGUE_METRICS(sig, sn, Neq)
%   m.D (랩당 손상), m.DEL (등가응력진폭 [MPa], Neq 사이클 기준), m.peak (max|σ| [MPa]),
%   m.cyc (rainflow_counts 출력), m.d (사이클별 손상, 구간 배분용)
%   Goodman: σar = σa/(1 − σm/σu) (σm > 0), 압축 평균은 보정 없음(σar = σa, 보수적). σm ≥ σu면 Inf.
%   Basquin + Miner: D = Σ n·2·(σar/σ'f)^m,  DEL = (Σ n·σar^m / Neq)^(1/m).

cyc = rainflow_counts(sig);
sa = cyc(:, 1) / 2;  sm = cyc(:, 2);  n = cyc(:, 3);
sar = sa;
t = sm > 0;
sar(t) = sa(t) ./ (1 - sm(t) / sn.su);
sar(t & sm >= sn.su) = Inf;
m.d = n .* 2 .* (sar / sn.sf).^sn.m;
m.D = sum(m.d);
m.DEL = (sum(n .* sar.^sn.m) / Neq)^(1/sn.m);
m.peak = max(abs(sig));
m.cyc = cyc;
end
