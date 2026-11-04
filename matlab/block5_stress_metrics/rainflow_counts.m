function cyc = rainflow_counts(x)
%RAINFLOW_COUNTS 레인플로 사이클 계수 (ASTM E1049-85 5.4.4, 3점법). Signal Processing Toolbox 없이.
%   cyc = RAINFLOW_COUNTS(x)   x: 시계열 (벡터)
%   cyc: K×4 [범위, 평균, 횟수(0.5 또는 1), 시작 인덱스]. 시작 인덱스는 사이클을 이루는 앞 극값의 x 인덱스
%        (구간별 손상 배분용).

x = x(:);
% 극값 추출: 같은 값 연속 제거 후 기울기 부호가 바뀌는 점 + 양 끝
k = [true; diff(x) ~= 0];
idx = find(k);
v = x(idx);
if numel(v) < 2
    cyc = zeros(0, 4);
    return
end
d = sign(diff(v));
tp = [1; find(d(1:end-1) ~= d(2:end)) + 1; numel(v)];
v = v(tp);  idx = idx(tp);

n = numel(v);
cyc = zeros(n, 4);  nc = 0;
sv = zeros(n, 1);  si = zeros(n, 1);  ns = 0;     % 스택 (값, 인덱스)
for j = 1:n
    ns = ns + 1;  sv(ns) = v(j);  si(ns) = idx(j);
    while ns >= 3
        X = abs(sv(ns) - sv(ns-1));
        Y = abs(sv(ns-1) - sv(ns-2));
        if X < Y, break; end
        nc = nc + 1;
        if ns == 3                                   % Y가 시작점을 포함 → 반 사이클, 시작점 버림
            cyc(nc, :) = [Y, (sv(1) + sv(2))/2, 0.5, si(1)];
            sv(1:2) = sv(2:3);  si(1:2) = si(2:3);  ns = 2;
        else                                         % 온 사이클, Y의 두 점 버림
            cyc(nc, :) = [Y, (sv(ns-1) + sv(ns-2))/2, 1, si(ns-2)];
            sv(ns-2) = sv(ns);  si(ns-2) = si(ns);  ns = ns - 2;
        end
    end
end
for j = 1:ns-1                                       % 남은 범위는 반 사이클
    nc = nc + 1;
    cyc(nc, :) = [abs(sv(j+1) - sv(j)), (sv(j+1) + sv(j))/2, 0.5, si(j)];
end
cyc = cyc(1:nc, :);
end
