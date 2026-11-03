function [t, v, ax, ay, s] = lap_sim(raceline_csv, fastf1_csv, params, scenario)
%LAP_SIM 준정상 랩 시뮬레이션 (블록 1). 시나리오 A/B.
%   [t, v, ax, ay, s] = LAP_SIM(raceline_csv, fastf1_csv, params, scenario)
%   입력:
%     raceline_csv - TUMFTM 레이스라인/중심선 CSV 경로 (scenario에 따라 다름)
%     fastf1_csv   - 인터페이스 I0 (fastf1_<track>.csv) 경로
%     params       - 차량 파라미터 struct (f82_params, D3, BMW F82 M4급 기준)
%     scenario     - 'A' (스포츠, 레이스라인+마찰원 한계) | 'B' (컴포트, 중심선+가속도 상한 |ay|<=0.4g,|ax|<=0.3g)
%   출력: 블록 2(Simulink)로 넘길 시간 이력. 격자 점마다 한 샘플 (t는 비등간격). ISO 8608 노면은 별도 중첩.
%     ax: 진행 방향 가속도 dv/dt, ay: 횡가속도 v²κ (좌회전 +, ISO 8855). 랩타임 = t(end) + 2*ds/(v(end)+v(1)).

switch scenario
    case 'A'
        trk = load_raceline(raceline_csv, 1);
        [~, theta, kv] = align_elevation(trk, fastf1_csv);
        kappa = compute_curvature(trk.x, trk.y, [], trk.ds, 10);
        v = quasi_steady_speed_profile(kappa, theta, kv, trk.ds, params);
    case 'B'
        trk = load_centerline(raceline_csv, 1);
        [~, theta] = align_elevation(trk, fastf1_csv);
        kappa = compute_curvature(trk.x, trk.y, [], trk.ds, 20);   % 중심선은 점이 불균일해 레이스라인(10 m)보다 넓게 평활
        v = capped_speed_profile(kappa, theta, trk.ds, params);
    otherwise
        error('lap_sim:BadScenario', 'scenario는 ''A'' 또는 ''B''');
end

ds = trk.ds;
s = trk.s;
dt = 2 * ds ./ (v + circshift(v, -1));          % i → i+1 구간 시간 (사다리꼴)
t = [0; cumsum(dt(1:end-1))];
ax = (circshift(v, -1).^2 - circshift(v, 1).^2) / (4 * ds);
ay = v.^2 .* kappa;
end
