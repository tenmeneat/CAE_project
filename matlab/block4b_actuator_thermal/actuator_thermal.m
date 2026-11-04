function I5 = actuator_thermal(F_act_act, ap, Rth, source)
%ACTUATOR_THERMAL 액추에이터 열 판정 (블록 4b → 인터페이스 I5).
%   I5 = ACTUATOR_THERMAL(F_act_act, ap, Rth, source)
%   F_act_act - N×4 액추에이터 기준 힘 [N] (블록 3 F_LCA.F_act_act = 휠 기준 F_act / MR)
%   ap        - actuator_params()
%   Rth       - 열저항 [K/W]. 생략하면 thermal_rth_1d(ap) (source = '1d_handcalc')
%   I5 필드: Rth, Fth, F_rms (1×4), Tcoil (1×4, 정상상태), ok (F_rms ≤ Fth), F_peak (1×4), sat (F_peak > F_sat), source
%   정상상태(랩 RMS를 연속으로 가정)라 보수적.

if nargin < 3 || isempty(Rth)
    Rth = thermal_rth_1d(ap);
    source = '1d_handcalc';
elseif nargin < 4
    source = 'comsol';
end
F_rms = sqrt(mean(F_act_act.^2, 1));
[Fth, Tcoil] = compute_fth(ap.Kf, ap.Rcoil, Rth, ap.Tamb, ap.Tlim, F_rms);
F_peak = max(abs(F_act_act), [], 1);
I5 = struct('Rth', Rth, 'Fth', Fth, 'F_rms', F_rms, 'Tcoil', Tcoil, 'ok', F_rms <= Fth, ...
    'F_peak', F_peak, 'sat', F_peak > ap.F_sat, 'source', source);
end
