function hp = hardpoints_f82(p_front)
%HARDPOINTS_F82 더블위시본 하드포인트 (블록 3, D3). 좌측 등가 좌표 [m], ISO 8855, 원점 = 앞 차축 중심 아래 지면.
%   hp = HARDPOINTS_F82(p_front)
%   p_front - 프런트 액추에이터 마운트 위치 p (D16). 리어는 고정 0.70 (D18)
%   hp.c(i) (i = 1..4: FL FR RL RR) 필드: WC LBJ UBJ LCA_F LCA_R UCA_F UCA_R TR_O TR_I CP (3×1), alpha, p
%
%   [가정] 일반 더블위시본 문헌형을 F82 트레드·타이어에 맞춘 임시값. D3에서 문헌 출처를 찾으면 교체.
%   우측 코너는 좌측 등가 좌표라 좌측과 같은 값. 리어는 프런트 형상을 평행이동, 타이로드 → 토 링크.

vp = block3_params();
f.WC    = [ 0     ; vp.tf/2; vp.r_w];  % 트레드/2 [확정] (1,579 mm), 높이 = 하중 반경 [가정]
f.LBJ   = [ 0     ; 0.715 ; 0.130];   % [가정]
f.UBJ   = [-0.010 ; 0.660 ; 0.500];   % [가정] 킹핀 경사 약 9°
f.LCA_F = [ 0.20  ; 0.330 ; 0.150];   % [가정]
f.LCA_R = [-0.20  ; 0.330 ; 0.160];   % [가정]
f.UCA_F = [ 0.12  ; 0.400 ; 0.520];   % [가정]
f.UCA_R = [-0.12  ; 0.400 ; 0.530];   % [가정]
f.TR_O  = [-0.13  ; 0.690 ; 0.190];   % [가정] 후방 조향
f.TR_I  = [-0.13  ; 0.330 ; 0.210];   % [가정]
f.CP    = f.WC - [0; 0; vp.r_w];      % 접지점
f.alpha = deg2rad(8);                 % [가정] 스트럿 경사 (수직에서 안쪽으로), p와 무관하게 고정
f.p     = p_front;

r = f;
d = [-2.812; (vp.tr - vp.tf)/2; 0];   % 휠베이스 [확정], 트레드 차의 절반 [확정]
for k = {'WC', 'LBJ', 'UBJ', 'LCA_F', 'LCA_R', 'UCA_F', 'UCA_R', 'TR_O', 'TR_I', 'CP'}
    r.(k{1}) = f.(k{1}) + d;
end
r.p = 0.70;                           % [가정] 리어 마운트 고정 (D18)

hp.c = [f f r r];
hp.p = p_front;
hp.p_rear = r.p;
hp.version = 'tmp-v1 (2026-11-03, 문헌형 임시값)';
end
