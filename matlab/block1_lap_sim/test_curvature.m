%TEST_CURVATURE load_raceline / compute_curvature 자체 검사 (실패하면 assert 에러).
here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..', '..');

% 1) 반지름 100 m 원 (반시계 = 좌회전): κ = +0.01. 수직: z = ½·c·s², c = 0.002 → κv = c, θ(s) = atan(c·s)
%    (z는 비주기라 경계 두 점은 틀리므로 중간 지점만 본다)
R = 100; N = 2000; ds = 2*pi*R / N; a = (0:N-1)' / N * 2*pi; s = R*a; c = 0.002;
[k, th, kv] = compute_curvature(R*cos(a), R*sin(a), 0.5*c*s.^2, ds, 0);
assert(max(abs(k - 1/R)) < 1e-4, '원 곡률 오차');
assert(abs(kv(N/2) - c) < 1e-6, '수직 곡률 오차');
assert(abs(th(N/2) - atan(c*s(N/2))) < 1e-4, '경사각 오차');

% 2) Spa 레이스라인·중심선: 닫힌 루프 총 회전각 ∫κ ds = ±2π
for f = {'racelines', 'tracks'}
    trk = load_raceline(fullfile(root, 'data', 'tracks', 'tumftm', f{1}, 'Spa.csv'), 1);
    kap = compute_curvature(trk.x, trk.y, [], trk.ds, 10);
    turn = sum(kap) * trk.ds;
    fprintf('%-10s L=%.1f m  ∫κds=%.3f rad (2π=%.3f)  max|κ|=%.4f 1/m (R_min=%.1f m)\n', ...
        f{1}, trk.L, turn, 2*pi, max(abs(kap)), 1/max(abs(kap)));
    assert(abs(abs(turn) - 2*pi) < 0.1, '루프 총 회전각이 2π가 아님');
end
% 3) 고저 정렬 (I0 CSV는 python/fastf1_preprocessing/fastf1_to_csv.py 로 생성, git 제외)
i0 = fullfile(root, 'data', 'raw', 'fastf1_spa.csv');
if isfile(i0)
    trk = load_raceline(fullfile(root, 'data', 'tracks', 'tumftm', 'racelines', 'Spa.csv'), 1);
    [z, theta, kv, info] = align_elevation(trk, i0);
    [~, ip] = max(theta);  [~, im] = min(theta);
    fprintf('고저 정렬: 오프셋 %.1f m (상관 %.3f / 다음 %.3f), z 범위 %.1f m, 최대 오르막 %.1f%% @ s=%.0f m, 최대 내리막 %.1f%% @ s=%.0f m, max|κv|=%.4f\n', ...
        info.shift_m, info.corr, info.corr2, max(z) - min(z), 100*tan(theta(ip)), trk.s(ip), 100*tan(theta(im)), trk.s(im), max(abs(kv)));
    assert(abs(z(1) - z(end)) < 0.5, 'z가 닫힌 루프가 아님');
    assert(max(z) - min(z) > 80 && max(z) - min(z) < 120, 'z 범위 이상');
    assert(abs(info.shift_m) < 100, '오프셋이 비정상적으로 큼');
end
disp('test_curvature OK');
