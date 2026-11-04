function vp = block3_params()
%BLOCK3_PARAMS 블록 3 차량 파라미터. SI 단위. 표기는 f82_params와 같음 ([확정]/[지정]/[가정]).
%   블록 2와 공유하는 값(비현가 질량, 트레드, 앞 횡하중 이동 비율)은 f82_params에서 읽는다 — 값은 한 곳에만 둔다.
%   f82_params는 block1_lap_sim에 있으므로 경로에 있어야 한다.

p = f82_params();
vp.g     = p.g;
vp.m_u   = [p.m_uf p.m_uf p.m_ur p.m_ur];  % 코너별 비현가 질량 [kg] (FL FR RL RR), 블록 2와 공유
vp.tf    = p.tf;                           % 트레드 [m], 하드포인트 휠센터 y = 트레드/2
vp.tr    = p.tr;
vp.lltd_f = p.lltd_f;                      % 앞 횡하중 이동 비율 (make_dummy_i1 전용, 블록 2 스태빌라이저 역산값과 같음)
vp.r_w   = 0.33;                           % [가정] 타이어 하중 반경 [m] (255/40 R18 무부하 0.3306)
end
