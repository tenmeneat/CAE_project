function [Rth, parts] = thermal_rth_1d(ap)
%THERMAL_RTH_1D 코일 → 주변 열저항 1D 손계산 (블록 4b). COMSOL Rth의 검증값이자 COMSOL 전까지의 대용값.
%   [Rth, parts] = THERMAL_RTH_1D(ap)   ap = actuator_params()
%   원통 반경 방향 전도(포팅, 하우징) + 외표면 대류, 직렬. 끝면 방열은 무시(보수적).

r1 = ap.r_c;  r2 = r1 + ap.t_pot;  r3 = r2 + ap.t_h;
parts.pot  = log(r2/r1) / (2*pi*ap.k_pot*ap.L_c);
parts.hous = log(r3/r2) / (2*pi*ap.k_h*ap.L_c);
parts.conv = 1 / (ap.h * 2*pi*r3*ap.L_c);
Rth = parts.pot + parts.hous + parts.conv;
end
