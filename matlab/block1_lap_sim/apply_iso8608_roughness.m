function [s_f, zr, r] = apply_iso8608_roughness(trk, z, Gd0, seed, ds_f)
%APPLY_ISO8608_ROUGHNESS 고저 z(s) 위에 ISO 8608 랜덤 노면 요철을 중첩 (블록 1, R6).
%   [s_f, zr, r] = APPLY_ISO8608_ROUGHNESS(trk, z, Gd0, seed, ds_f)
%   trk: load_raceline/load_centerline 결과, z: trk.s 격자의 고저 (align_elevation 출력).
%   Gd0: n0=0.1 cycle/m 에서의 변위 PSD [m^3], 기본 128e-6 (B/C 경계 = B급 상한·C급 하한의 경계, "B-C급").
%        (A<32e-6, B 32–128e-6, C 128–512e-6, D 512–2048e-6)
%   seed: 난수 시드 (전역 rng는 건드리지 않음), 기본 1. ds_f: 노면 공간 간격 [m], 기본 0.1.
%   출력: s_f (0:ds_f:L), zr = 고저 + 요철, r = 요철만 [m].
%
%   PSD Gd(n) = Gd0·(n/n0)^-2, ISO 8608 유효 대역 0.011–2.83 cycle/m (파장 90–0.35 m)을 랜덤 위상 조화합으로 합성.
%   1 m 격자(트랙 격자)로는 2 m 미만 파장이 안 담기므로 노면은 별도 미세 격자에 만든다 — 시간축 변환은
%   interp1(s_f, r, s(t)) 로 블록 2에서 한다 (저속 코너는 같은 파장이 낮은 주파수라 문제 없음).
%   닫힌 루프라 L 주기 → 랩 이음매에서 연속. 좌/우 트랙은 다른 시드로 두 번 호출 (좌우 상관은 무시 — 롤 가진이
%   과대평가될 수 있음, ponytail: 필요하면 좌우 코히런스 추가).

if nargin < 3 || isempty(Gd0), Gd0 = 128e-6; end
if nargin < 4 || isempty(seed), seed = 1; end
if nargin < 5 || isempty(ds_f), ds_f = 0.1; end

N = round(trk.L / ds_f);
ds_f = trk.L / N;
s_f = (0:N-1)' * ds_f;
n = (1:floor(N/2) - 1)' / trk.L;                         % 공간 주파수 [cycle/m], k/L
band = n >= 0.011 & n <= 2.83;
A = zeros(size(n));
A(band) = sqrt(2 * Gd0 * (n(band) / 0.1).^-2 / trk.L);   % 진폭, Δn = 1/L
phi = 2 * pi * rand(RandStream('twister', 'Seed', seed), size(n));

C = zeros(N, 1);
C(2:numel(n)+1) = N * A .* exp(1i * phi);
r = real(ifft(C));                                       % Σ A_k cos(2π k s/L + φ_k)

z_f = interp1([trk.s; trk.L], [z; z(1)], s_f, 'linear');
zr = z_f + r;
end
