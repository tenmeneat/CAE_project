function spec = lca_unit_load_spec(hc, p)
%LCA_UNIT_LOAD_SPEC COMSOL 단위하중 해석 입력 (블록 4a). 로어암 로컬 좌표, 원점 LCA_F [m].
%   spec = LCA_UNIT_LOAD_SPEC(hc, p)   hc = hardpoints_f82(p).c(1)
%   spec.pts    table: 점 이름, x y z (로컬 e1 전방·e2 바깥·e3 위)
%   spec.u_s    스트럿 단위벡터 (마운트 → 차체), 로컬
%   spec.cases  하중 케이스 (블록 3 노트 11절 (나)안): LBJ에 로컬 단위하중 1 N 3개.
%               지지: 부싱 2점 Spring Foundation, 마운트점은 u_s 방향만 구속(스트럿 축 지지).
%               마운트 반력 = 블록 3 F_M (힌지축 모멘트 평형이 같아서 자동으로 일치)

geo = lca_geometry(hc, p);
R = geo.Rloc;
loc = @(r) (R' * (r - hc.LCA_F))';
names = {'LCA_F'; 'LCA_R'; 'LBJ'; 'r_foot'; 'r_M'};
pos = [loc(hc.LCA_F); loc(hc.LCA_R); loc(hc.LBJ); loc(geo.r_foot); loc(geo.r_M)];
spec.pts = table(names, pos(:, 1), pos(:, 2), pos(:, 3), 'VariableNames', {'point', 'x', 'y', 'z'});
spec.u_s = R' * geo.u_s;
spec.cases = {'BJ_x', 'BJ_y', 'BJ_z'};
spec.p = p;
end
