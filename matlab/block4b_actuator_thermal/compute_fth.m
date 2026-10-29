function [Fth, Tcoil] = compute_fth(Kf, Rcoil, Rth, Tamb, Tlim, Frms)
%COMPUTE_FTH 열적 연속 힘 한계 Fth 계산 (블록 4b, MATLAB 측). 스켈레톤 — 미구현.
%   [Fth, Tcoil] = COMPUTE_FTH(Kf, Rcoil, Rth, Tamb, Tlim, Frms)
%   입력:
%     Kf    - 힘 상수 [N/A] (문헌/데이터시트 값, D15)
%     Rcoil - 코일 저항 [Ohm] (최대 허용 온도 기준값, 보수적 고정)
%     Rth   - 열저항 코일->주변 [K/W] (comsol/block4b_actuator_thermal의 정상상태 해석 결과)
%     Tamb  - 주변 온도 [degC]
%     Tlim  - 절연 등급 온도 한계 [degC] (예: F급 155)
%     Frms  - 운전 RMS 힘 [N] (블록 2 출력 기반)
%   출력:
%     Fth   - 열적 연속 힘 한계 [N]
%     Tcoil - Frms에서의 정상상태 코일 온도 [degC] (참고용)
%
%   공식: Tcoil = Tamb + Rth*Rcoil*(Frms/Kf)^2
%         Fth   = Kf*sqrt((Tlim-Tamb)/(Rth*Rcoil))
%   I5(act_thermal_<act>.mat)로 내보내 블록 6의 F_rms <= Fth 판정에 사용.

error('compute_fth:NotImplemented', 'TODO: 블록 4b 구현 — COMSOL 정상상태 해석으로 Rth 확보 후 공식 적용');
end
