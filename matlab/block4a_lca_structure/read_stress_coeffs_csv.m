function C = read_stress_coeffs_csv(file, p, source)
%READ_STRESS_COEFFS_CSV 응력계수 표 CSV 읽기 (블록 4a → 5). 형식은 write_stress_coeffs_csv 참고 (열: p, load_case, hotspot, s11…s13).
%   C = READ_STRESS_COEFFS_CSV(file, p, source)   p 한 수준만 꺼냄 (|Δp| < 1e-9), source 기본 'comsol'.
%   케이스는 BJ_x, BJ_y, BJ_z가 모두 있어야 하고 핫스팟 집합이 케이스마다 같아야 한다.
%   핫스팟 위치(s, x, z)는 CSV에 없으므로 C.hot은 이름만 채운다.

if nargin < 3, source = 'comsol'; end
T = readtable(file, 'TextType', 'char');
T = T(abs(T.p - p) < 1e-9, :);
if isempty(T), error('read_stress_coeffs_csv:NoP', '%s에 p = %g 없음', file, p); end
cases = {'BJ_x', 'BJ_y', 'BJ_z'};
names = unique(T.hotspot, 'stable');
coef = NaN(numel(names), 6, 3);
comp = {'s11', 's22', 's33', 's12', 's23', 's13'};
for r = 1:height(T)
    k = find(strcmp(cases, T.load_case{r}));
    h = find(strcmp(names, T.hotspot{r}));
    if isempty(k), error('read_stress_coeffs_csv:Case', '알 수 없는 케이스 %s', T.load_case{r}); end
    coef(h, :, k) = T{r, comp};
end
if any(isnan(coef), 'all'), error('read_stress_coeffs_csv:Incomplete', '케이스 × 핫스팟 조합이 빠짐'); end
C = struct('coef', coef, 'comp', {comp}, 'cases', {cases}, ...
    'hot', table(names, NaN(size(names)), NaN(size(names)), NaN(size(names)), 'VariableNames', {'name', 's', 'x', 'z'}), ...
    'p', p, 'source', source, 'meta', struct('file', file));
end
