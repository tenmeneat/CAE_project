%TEST_ISO8608 apply_iso8608_roughness 자체 검사 (실패하면 assert 에러).
here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..', '..');
trk = load_raceline(fullfile(root, 'data', 'tracks', 'tumftm', 'racelines', 'Spa.csv'), 1);
Gd0 = 128e-6;
[s_f, zr, r] = apply_iso8608_roughness(trk, zeros(size(trk.s)), Gd0, 1, 0.1);
N = numel(r);  ds_f = s_f(2) - s_f(1);

% 1) 분산 = ∫Gd dn (유효 대역 0.011–2.83 cycle/m): Gd0·n0²·(1/n_lo − 1/n_hi)
sig_th = sqrt(Gd0 * 0.1^2 * (1/0.011 - 1/2.83));
fprintf('요철 RMS %.2f mm (이론 %.2f mm)\n', 1e3*std(r, 1), 1e3*sig_th);
assert(abs(std(r, 1) / sig_th - 1) < 0.02, 'RMS가 ISO 8608 PSD 적분과 안 맞음');

% 2) 주기도 PSD: n0=0.1 cycle/m 주변에서 Gd(n0) 복원, 기울기 −2
X = fft(r);  k = (1:floor(N/2)-1)';  n = k / trk.L;
G = 2 * ds_f * abs(X(k+1)).^2 / N;                       % 단측 PSD [m^3]
m = n > 0.07 & n < 0.14;
assert(abs(mean(G(m) .* (n(m)/0.1).^2) / Gd0 - 1) < 0.05, 'Gd(n0) 복원 실패');
b = n > 0.02 & n < 2;  pf = polyfit(log(n(b)), log(G(b)), 1);
assert(abs(pf(1) + 2) < 0.1, sprintf('PSD 기울기 %.2f (기대 −2)', pf(1)));
assert(all(G(n < 0.0105) < 1e-20) && all(G(n > 2.84) < 1e-20), '유효 대역 밖에 에너지');   % 반올림 오차 여유

% 3) 재현성, 시드 독립, 전역 rng 불변, 이음매 연속
rng(7);  a = rand;  rng(7);
[~, ~, r1] = apply_iso8608_roughness(trk, zeros(size(trk.s)), Gd0, 1, 0.1);
[~, ~, r2] = apply_iso8608_roughness(trk, zeros(size(trk.s)), Gd0, 1, 0.1);
[~, ~, r3] = apply_iso8608_roughness(trk, zeros(size(trk.s)), Gd0, 2, 0.1);
cc = corrcoef(r1, r3);
assert(isequal(r1, r2) && abs(cc(1,2)) < 0.05, '시드 동작 이상');
assert(rand == a, '전역 rng를 건드림');
assert(abs(r(1) - r(end)) < 5 * max(abs(diff(r))), '이음매 불연속');

% 4) 고저와 합쳐지는지: 평균 보존
z = 100 + 10 * sin(2*pi*trk.s / trk.L);
[~, zr, r] = apply_iso8608_roughness(trk, z, Gd0, 1, 0.1);
assert(abs(mean(zr - r) - 100) < 0.01 && abs(mean(r)) < 1e-9, '고저 합성 이상');
disp('test_iso8608 OK');
