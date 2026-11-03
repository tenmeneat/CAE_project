function trk = load_raceline(csv, ds)
%LOAD_RACELINE TUMFTM 트랙 CSV 로더 + 닫힌 루프를 Δs 균일 격자로 재샘플 (블록 1).
%   trk = LOAD_RACELINE(csv, ds)  ds 기본 1 m
%   csv: 레이스라인(x_m,y_m) 또는 중심선(x_m,y_m,w_tr_right_m,w_tr_left_m). '#' 헤더 1줄.
%   trk: s (N×1, 0부터), x, y, L (둘레), ds (실제 간격 = L/N), 중심선이면 w_r, w_l도.
%   TUMFTM CSV는 시작점을 끝에 중복하지 않는 열린 점열이라 마지막→처음 구간을 닫아서 둘레에 포함한다.

if nargin < 2, ds = 1; end
d = readmatrix(csv, 'NumHeaderLines', 1);
d = [d; d(1,:)];                                   % 루프 닫기
c = [0; cumsum(hypot(diff(d(:,1)), diff(d(:,2))))];
L = c(end);
N = round(L / ds);
trk.s = (0:N-1)' * (L / N);
trk.L = L;
trk.ds = L / N;

% 주기 스플라인: 앞뒤로 점 5개씩 감아 붙여 보간한 뒤 경계 효과를 버린다.
k = 5; n = numel(c) - 1;
cp = [c(n-k+1:n) - L; c(1:n); c(1:k) + L];
names = {'x', 'y', 'w_r', 'w_l'};
for i = 1:size(d, 2)
    v = d(1:n, i);
    trk.(names{i}) = interp1(cp, [v(n-k+1:n); v; v(1:k)], trk.s, 'spline');
end
end
