%TEST_RAINFLOW 레인플로·피로 지표 손계산 검사 (블록 5). 실패하면 assert 에러.
sn = sn_params('al_forged');

% 1) ASTM E1049-85 Fig. 6 예제: -2 1 -3 5 -1 3 -4 4 -2
%    범위별 횟수: 3 → 0.5, 4 → 1.5, 6 → 0.5, 8 → 1.0, 9 → 0.5 (총 4.0)
cyc = rainflow_counts([-2 1 -3 5 -1 3 -4 4 -2]);
got = accumarray(cyc(:, 1), cyc(:, 3), [10 1]);
want = zeros(10, 1);  want([3 4 6 8 9]) = [0.5 1.5 0.5 1.0 0.5];
assert(isequal(got, want), 'ASTM 예제 사이클 수가 다름');

% 2) 정현파 n 사이클 (평균 0): D = n·2·(σa/σ'f)^m, DEL(Neq = n) = σa
sa = 80;  n = 200;
x = sa * sin(2*pi*(0:1/40:n)');   % 1/40 간격 = 피크를 정확히 지남
m = fatigue_metrics(x, sn, n);
assert(abs(m.D / (n * 2 * (sa/sn.sf)^sn.m) - 1) < 0.01, '정현파 손상');
assert(abs(m.DEL / sa - 1) < 0.01 && abs(m.peak - sa) < 1e-9, '정현파 DEL·피크');

% 3) Goodman: 평균 σm > 0이면 σar = σa/(1 − σm/σu)
sm = 100;
m2 = fatigue_metrics(x + sm, sn, n);
assert(abs(m2.DEL / (sa / (1 - sm/sn.su)) - 1) < 0.01, 'Goodman 보정');

% 4) σ'f를 바꿔도 손상비 R은 그대로 (같은 m)
sn2 = sn;  sn2.sf = 2 * sn.sf;
y = 0.7 * x;
R1 = fatigue_metrics(y, sn, n).D / m.D;
R2 = fatigue_metrics(y, sn2, n).D / fatigue_metrics(x, sn2, n).D;
assert(abs(R1/R2 - 1) < 1e-12 && abs(R1 - 0.7^sn.m) < 1e-3 * R1, 'R이 σf에 의존');
disp('test_rainflow OK');
