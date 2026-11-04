function sn = sn_params(material)
%SN_PARAMS S-N(Basquin) 파라미터 (블록 5). D4(로어암 재질) 미정 → 두 후보 모두 [가정].
%   sn = SN_PARAMS(material)   'al_forged' (기본) | 'steel'
%   σ'f, b는 Bäumel–Seeger 균일 재료 법칙(Uniform Material Law, 1990)으로 σu에서 추정:
%     강: σ'f = 1.5·σu, b = −0.087     알루미늄·티타늄: σ'f = 1.67·σu, b = −0.095
%   R = D/D_passive는 σ'f에 무관하고 m = −1/b에만 의존한다 (블록 5 노트). 절대 D는 보고 지표로 쓰지 않는다.

if nargin < 1, material = 'al_forged'; end
switch material
    case 'al_forged'
        sn.su = 310;              % [가정] 단조 6082-T6 인장강도 [MPa]
        sn.sf = 1.67 * sn.su;     % σ'f [MPa]
        sn.b  = -0.095;
    case 'steel'
        sn.su = 440;              % [가정] 열연 고장력강 (SAPH440급) 인장강도 [MPa]
        sn.sf = 1.5 * sn.su;
        sn.b  = -0.087;
    otherwise
        error('sn_params:Material', '재질 %s 없음 (al_forged | steel)', material);
end
sn.m = -1 / sn.b;
sn.name = material;
end
