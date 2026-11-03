function [z, theta, kv, info] = align_elevation(trk, i0_csv, win)
%ALIGN_ELEVATION I0(fastf1_<track>.csv)의 z(s)를 트랙(레이스라인/중심선) s 격자로 옮긴다 (블록 1, 4.1절).
%   [z, theta, kv, info] = ALIGN_ELEVATION(trk, i0_csv, win)
%   trk: load_raceline/load_centerline 결과. win: θ·κv 평활 창 [m], 기본 25.
%   XY 좌표계가 달라 XY 정합 대신 곡률 κ(s) 순환 상호상관으로 출발선 오프셋을 구하고, 둘레가 다르므로
%   (레이스라인 ≠ 주행선 길이) 거리가 아니라 둘레 비율 u = s/L 로 맞춘다.
%   info: .shift_m (트랙 s = I0 s + shift), .corr (최대 상관), .corr2 (다음 피크), .closure (폐곡선 z 오차, 제거됨)

if nargin < 3, win = 25; end
d = readmatrix(i0_csv);                        % s, x, y, z, v, throttle, brake
N = numel(trk.s);
u = (0:N-1)' / N;
L0 = d(end,1) + (d(2,1) - d(1,1));             % I0 둘레 (1 m 격자)
atu = @(v, L, s) interp1([s; L], [v; v(1)], u * L, 'linear');   % 비율 격자로 재샘플 (주기)

kT = atu(compute_curvature(trk.x, trk.y, [], trk.ds, 10), trk.L, trk.s);
kF = atu(compute_curvature(d(:,2), d(:,3), [], d(2,1) - d(1,1), 25), L0, d(:,1));  % 랩 평균이라 넓게 평활
a = kT - mean(kT);  b = kF - mean(kF);
xc = real(ifft(fft(a) .* conj(fft(b)))) / (norm(a) * norm(b));    % xc(k+1): 트랙을 k점 뒤로 밀었을 때 일치
[info.corr, i] = max(xc);
lag = i - 1;  if lag > N/2, lag = lag - N; end
kd = mod((1:N)' - i, N);
info.corr2 = max(xc(min(kd, N - kd) > 50));     % 다음 피크: 주 피크에서 순환 거리 50점 넘게 떨어진 곳 (지연 0 근처에서도 맞게)
info.shift_m = lag / N * trk.L;
assert(info.corr > 0.8 && info.corr - info.corr2 > 0.1, ...
    'align_elevation:ambiguous', '곡률 정렬이 불분명 (상관 %.2f, 다음 피크 %.2f)', info.corr, info.corr2);

% 트랙 점 u 에 대응하는 I0 위치는 u - lag/N (주기 보간)
uf = mod(u - lag / N, 1);
info.closure = d(end,4) - d(1,4);
zf = d(:,4) - info.closure * d(:,1) / L0;      % 폐곡선 불일치를 I0 둘레에 선형 분배 (이음매가 I0 s=0이라 보간 전에 제거)
z = interp1([d(:,1) / L0; 1], [zf; zf(1)], uf, 'linear');

if nargout > 1
    [~, theta, kv] = compute_curvature(trk.x, trk.y, z, trk.ds, win);
end
end
