function [Fth, Tcoil] = compute_fth(Kf, Rcoil, Rth, Tamb, Tlim, Frms)
%COMPUTE_FTH 열적 연속 힘 한계 Fth 계산 (블록 4b, MATLAB 측).
%   [Fth, Tcoil] = COMPUTE_FTH(Kf, Rcoil, Rth, Tamb, Tlim, Frms)
%   입력:
%     Kf    - 힘 상수 [N/A] (문헌/데이터시트 값, D15)
%     Rcoil - 코일 저항 [Ohm] (최대 허용 온도 기준값, 보수적 고정)
%     Rth   - 열저항 코일->주변 [K/W] (COMSOL 정상상태 해석, 그 전에는 thermal_rth_1d)
%     Tamb  - 주변 온도 [degC]
%     Tlim  - 절연 등급 온도 한계 [degC] (예: F급 155)
%     Frms  - 운전 RMS 힘 [N] (액추에이터 기준 = 블록 3 F_act_act의 RMS). 배열 가능
%   출력:
%     Fth   - 열적 연속 힘 한계 [N]
%     Tcoil - Frms에서의 정상상태 코일 온도 [degC] (Frms와 같은 크기)
%
%   공식: Tcoil = Tamb + Rth*Rcoil*(Frms/Kf)^2
%         Fth   = Kf*sqrt((Tlim-Tamb)/(Rth*Rcoil))
%   I5(actuator_thermal)로 묶어 블록 6의 F_rms <= Fth 판정에 사용.

Fth = Kf * sqrt((Tlim - Tamb) / (Rth * Rcoil));
Tcoil = Tamb + Rth * Rcoil * (Frms / Kf).^2;
end
