function [p, MR] = mr_curve(p_grid)
%MR_CURVE D16용 프런트 모션비 곡선 MR(p) (블록 3).
%   [p, MR] = MR_CURVE(p_grid)   기본 p_grid = 0.30:0.05:0.95
%   p_min은 MR(p_min) ≥ F_wheel,max / F_sat,act 조건으로 정한다 (설계안 4.3절).

if nargin < 1, p_grid = 0.30:0.05:0.95; end
p = p_grid(:);
MR = zeros(size(p));
for k = 1:numel(p)
    hc = hardpoints_f82(p(k)).c(1);
    MR(k) = motion_ratio(hc, lca_geometry(hc, p(k)));
end
end
