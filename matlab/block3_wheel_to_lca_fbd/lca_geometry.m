function geo = lca_geometry(hc, p)
%LCA_GEOMETRY 코너 1개의 파생 기하 (블록 3). 좌측 등가 좌표.
%   geo = LCA_GEOMETRY(hc, p)   hc = hardpoints_f82(...).c(i), p = 마운트 위치
%   geo 필드:
%     e_L, e_U - 하부·상부암 힌지축 단위벡터 (뒤 → 앞 부싱)
%     r_foot   - LBJ에서 하부암 힌지축에 내린 수선의 발
%     r_M      - 스트럿 마운트점 = r_foot + p·(LBJ − r_foot)   (p 정의 [가정, D16])
%     u_s      - 스트럿 단위벡터, 로어암 마운트 → 차체 상부 마운트 (수직에서 안쪽으로 alpha)
%     u_T      - 타이로드 단위벡터, 바깥 → 안쪽 (인장 f_T > 0이면 너클에 f_T·u_T)
%     Rloc     - 로어암 로컬 축 [e1 e2 e3] (열). 로컬 = Rloc'·전역. 원점은 LCA_F

geo.e_L = unit(hc.LCA_F - hc.LCA_R);
geo.e_U = unit(hc.UCA_F - hc.UCA_R);
geo.r_foot = hc.LCA_R + dot(hc.LBJ - hc.LCA_R, geo.e_L) * geo.e_L;
geo.r_M = geo.r_foot + p * (hc.LBJ - geo.r_foot);
geo.u_s = [0; -sin(hc.alpha); cos(hc.alpha)];
geo.u_T = unit(hc.TR_I - hc.TR_O);

e1 = geo.e_L;
e2 = hc.LBJ - geo.r_foot;
e2 = unit(e2 - dot(e2, e1) * e1);
geo.Rloc = [e1, e2, cross(e1, e2)];
geo.p = p;
end

function u = unit(v)
u = v / norm(v);
end
