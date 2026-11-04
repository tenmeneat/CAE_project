function mdl = build_fullcar_model()
%BUILD_FULLCAR_MODEL fullcar_7dof.slx 생성 (블록 2). 모델은 이 스크립트가 원본 — .slx를 손으로 고치지 말 것.
%   물리 부품별 서브시스템 (상태 = 적분기 출력, 힘은 상태에서만 계산 → 대수루프 없음):
%     w (Inport: zr 4, ax, ay) ─┬─ Load Transfer : M_θ = −m_s·h·ax, M_φ = m_s·h·ay
%                               └─ zr → Corner FL/FR/RL/RR : 스프링·댐퍼·액추에이터·스태빌라이저 힘, 비현가 질량,
%                                                            단방향 타이어 Fz = max(0, Fz0 + kt(zr − zu))
%     Sprung Body : 히브·피치·롤 (코너 힘 → T'F + 모멘트 → 적분 2회), 코너 위치 z_b = T·q_b
%     Anti-roll Bar : F_arb = −S'·diag(k_arb)·S·d
%     Controller : F_act = −Kfb·x  (Passive/Skyhook/LQR 공통, x = [zs θ φ zu(4) 속도 7])
%   선형 부분은 build_state_space의 A, B와 같아야 한다 (test_fullcar_model이 linmod로 확인).
%   액추에이터 포화·1차 지연(W4)은 Controller 뒤에 넣을 자리.
%   워크스페이스 변수: veh (build_state_space의 info), Kfb (4×14), x0 (14×1). 입력은 루트 Inport 'w' (6).
%
%   레이아웃: 루트는 블록 포트마다 Goto/From 태그로 연결 (긴 배선 없음). 4-벡터 신호는 '분배'(Demux → 코너별
%   태그)와 '수집'(코너별 태그 → Mux) 열에서 바꾼다. 열: 본체 블록 | 분배 | 코너 | 수집.
%   서브시스템 안은 포트 높이를 맞춰(align) 직선 배선.

mdl = 'fullcar_7dof';
here = fileparts(mfilename('fullpath'));
f = fullfile(here, [mdl '.slx']);
if bdIsLoaded(mdl), close_system(mdl, 0); end
if isfile(f), delete(f); end                 % 같은 이름 파일이 경로에 있으면 new_system이 섀도잉 경고
new_system(mdl);
c4 = {'FL', 'FR', 'RL', 'RR'};
tags = @(n) strcat(n, '_', c4);

% ── 열 1: 본체 블록
add_block('built-in/Inport', [mdl '/w'], 'PortDimensions', '6');
place(mdl, 'w', [200 30 230 44], {}, {'w'});
add_block('built-in/Demux', [mdl '/w_split'], 'Outputs', '[4 2]');
place(mdl, 'w_split', [200 80 205 160], {'w'}, {'zr', 'ax_ay'});
build_load_transfer([mdl '/Load Transfer']);
place(mdl, 'Load Transfer', [200 200 320 240], {'ax_ay'}, {'M'});
build_body([mdl '/Sprung Body']);
place(mdl, 'Sprung Body', [200 290 340 450], {'F_s', 'M'}, {'z_b', 'zd_b', 'q_b', 'qd_b'});
build_arb([mdl '/Anti-roll Bar']);
place(mdl, 'Anti-roll Bar', [200 500 320 540], {'d'}, {'F_arb'});
add_block('built-in/Gain', [mdl '/Controller'], 'Gain', '-Kfb', 'Multiplication', 'Matrix(K*u)');
place(mdl, 'Controller', [200 590 320 630], {'x'}, {'F_act'});
add_block('built-in/Mux', [mdl '/x_mux'], 'Inputs', '[3 4 3 4]');
place(mdl, 'x_mux', [200 680 205 840], {'q_b', 'zu', 'qd_b', 'zud'}, {'x'});
add_block('built-in/Outport', [mdl '/x']);
place(mdl, 'x', [200 890 230 904], {'x'}, {});
for n = {'x', 'F_act', 'Fz'}
    add_block('simulink/Sinks/To Workspace', [mdl '/' n{1} '_log'], 'VariableName', n{1}, 'SaveFormat', 'Array');
end
place(mdl, 'x_log', [200 940 290 970], {'x'}, {});
place(mdl, 'F_act_log', [200 1000 290 1030], {'F_act'}, {});
place(mdl, 'Fz_log', [200 1060 290 1090], {'Fz'}, {});

% ── 열 2: 분배 (4-벡터 → 코너별 태그)
for k = 1:5
    n = {'zr', 'z_b', 'zd_b', 'F_act', 'F_arb'};  n = n{k};
    add_block('built-in/Demux', [mdl '/' n '_split'], 'Outputs', '4');
    place(mdl, [n '_split'], [650, 30 + (k-1)*210, 655, 190 + (k-1)*210], {n}, tags(n));
end

% ── 열 3: 코너
for i = 1:4
    nm = ['Corner ' c4{i}];
    build_corner([mdl '/' nm], i);
    ins = cellfun(@(n) [n '_' c4{i}], {'z_b', 'zd_b', 'zr', 'F_act', 'F_arb'}, 'UniformOutput', false);
    outs = cellfun(@(n) [n '_' c4{i}], {'F_s', 'd', 'zu', 'zud', 'Fz'}, 'UniformOutput', false);
    place(mdl, nm, [1000, 30 + (i-1)*260, 1150, 230 + (i-1)*260], ins, outs);
end

% ── 열 4: 수집 (코너별 태그 → 4-벡터)
for k = 1:5
    n = {'F_s', 'd', 'zu', 'zud', 'Fz'};  n = n{k};
    add_block('built-in/Mux', [mdl '/' n '_mux'], 'Inputs', '4');
    place(mdl, [n '_mux'], [1450, 30 + (k-1)*210, 1455, 190 + (k-1)*210], tags(n), {n});
end

set_param(mdl, 'Solver', 'ode4', 'FixedStep', 'dt', 'StopTime', 't_end', 'SaveTime', 'on', ...
    'LoadExternalInput', 'on', 'ExternalInput', 'w_in', 'AlgebraicLoopMsg', 'error', 'ZoomFactor', 'FitSystem');
save_system(mdl, f);
end

function place(sys, name, pos, ins, outs)
% 블록을 pos에 두고, 입력 포트 k에는 From(ins{k}), 출력 포트 k에는 Goto(outs{k})를 포트 높이에 맞춰 붙인다.
b = [sys '/' name];
set_param(b, 'Position', pos);
ph = get_param(b, 'PortHandles');
for k = 1:numel(ins)
    p = get_param(ph.Inport(k), 'Position');
    fb = sprintf('%s in%d', name, k);
    add_block('simulink/Signal Routing/From', [sys '/' fb], 'GotoTag', ins{k}, 'ShowName', 'off', ...
        'AttributesFormatString', '%<GotoTag>', ...   % R2026a(한국어)에서 From 아이콘이 '-T-'로만 보여 태그를 주석으로 표시
        'Position', [p(1)-110, p(2)-10, p(1)-40, p(2)+10]);
    add_line(sys, [fb '/1'], sprintf('%s/%d', name, k));
end
for k = 1:numel(outs)
    p = get_param(ph.Outport(k), 'Position');
    gb = sprintf('%s out%d', name, k);
    add_block('built-in/Goto', [sys '/' gb], 'GotoTag', outs{k}, 'ShowName', 'off', ...
        'Position', [p(1)+40, p(2)-10, p(1)+110, p(2)+10]);
    add_line(sys, sprintf('%s/%d', name, k), [gb '/1']);
end
end

function blk(sys, type, name, pos, varargin)
add_block(['built-in/' type], [sys '/' name], 'Position', pos, varargin{:});
end

function align(sys, src, sp, dst, dp, move)
% src 출력 sp와 dst 입력 dp를 같은 높이로 (직선 배선). move = 'dst'(기본) 또는 'src' — 그 블록만 세로로 옮긴다
ys = get_param(get_param([sys '/' src], 'PortHandles').Outport(sp), 'Position');
yd = get_param(get_param([sys '/' dst], 'PortHandles').Inport(dp), 'Position');
if nargin < 6 || strcmp(move, 'dst'), b = dst; dy = ys(2) - yd(2); else, b = src; dy = yd(2) - ys(2); end
set_param([sys '/' b], 'Position', get_param([sys '/' b], 'Position') + [0 1 0 1] * dy);
end

function line(sys, src, dst)
add_line(sys, src, dst, 'autorouting', 'smart');
end

function build_load_transfer(sys)
add_block('built-in/Subsystem', sys);
blk(sys, 'Inport', 'ax_ay', [40 43 70 57]);
blk(sys, 'Gain', 'm_s h', [130 30 250 70], 'Gain', 'veh.ms*veh.h_cg*[-1 0; 0 1]', 'Multiplication', 'Matrix(K*u)');
blk(sys, 'Outport', 'M_theta_phi', [310 43 340 57]);
line(sys, 'ax_ay/1', 'm_s h/1');
line(sys, 'm_s h/1', 'M_theta_phi/1');
end

function build_arb(sys)
add_block('built-in/Subsystem', sys);
blk(sys, 'Inport', 'd', [40 43 70 57]);
blk(sys, 'Gain', 'k_arb', [130 30 290 70], 'Gain', '-veh.S''*diag(veh.k_arb)*veh.S', 'Multiplication', 'Matrix(K*u)');
blk(sys, 'Outport', 'F_arb', [350 43 380 57]);
line(sys, 'd/1', 'k_arb/1');
line(sys, 'k_arb/1', 'F_arb/1');
end

function build_body(sys)
% 입력: F_s (4, 코너에서 차체에 위로), M (2). 출력: z_b, zd_b (4), q_b, qd_b (3) = [zs θ φ]
add_block('built-in/Subsystem', sys);
blk(sys, 'Inport', 'F_s', [40 63 70 77]);
blk(sys, 'Inport', 'M', [40 143 70 157]);
blk(sys, 'Gain', 'T''', [120 50 190 90], 'Gain', 'veh.T''', 'Multiplication', 'Matrix(K*u)');
blk(sys, 'Gain', 'to pitch,roll', [120 130 190 170], 'Gain', '[0 0; 1 0; 0 1]', 'Multiplication', 'Matrix(K*u)');
blk(sys, 'Sum', 'Sum', [250 30 280 190], 'Inputs', '++', 'IconShape', 'rectangular');
align(sys, 'T''', 1, 'Sum', 1);
blk(sys, 'Gain', 'inv inertia', [330 90 420 130], 'Gain', '1./[veh.ms; veh.Iyy; veh.Ixx]');
blk(sys, 'Integrator', 'qd_b', [470 90 510 130], 'InitialCondition', 'x0(8:10)');
blk(sys, 'Integrator', 'q_b', [570 90 610 130], 'InitialCondition', 'x0(1:3)');
blk(sys, 'Gain', 'T pos', [680 90 740 130], 'Gain', 'veh.T', 'Multiplication', 'Matrix(K*u)');
blk(sys, 'Outport', 'z_b', [800 103 830 117]);
blk(sys, 'Gain', 'T vel', [680 250 740 290], 'Gain', 'veh.T', 'Multiplication', 'Matrix(K*u)');
blk(sys, 'Outport', 'zd_b', [800 263 830 277]);
blk(sys, 'Outport', 'q_b_out', [800 183 830 197]);
blk(sys, 'Outport', 'qd_b_out', [800 343 830 357]);
for b = {'inv inertia', 'qd_b', 'q_b', 'T pos'}, align(sys, 'Sum', 1, b{1}, 1); end
align(sys, 'T pos', 1, 'z_b', 1);
line(sys, 'F_s/1', 'T''/1');
line(sys, 'M/1', 'to pitch,roll/1');
line(sys, 'T''/1', 'Sum/1');
line(sys, 'to pitch,roll/1', 'Sum/2');
line(sys, 'Sum/1', 'inv inertia/1');
line(sys, 'inv inertia/1', 'qd_b/1');
line(sys, 'qd_b/1', 'q_b/1');
line(sys, 'q_b/1', 'T pos/1');
line(sys, 'T pos/1', 'z_b/1');
line(sys, 'q_b/1', 'q_b_out/1');
line(sys, 'qd_b/1', 'T vel/1');
line(sys, 'T vel/1', 'zd_b/1');
line(sys, 'qd_b/1', 'qd_b_out/1');
end

function build_corner(sys, i)
% 입력: z_b, zd_b, zr, F_act, F_arb. 출력: F_s(차체에 위로), d = z_b − zu, zu, zud, Fz(전체 타이어 하중)
% 위: 서스펜션 힘 4행 → F_s 합 (F_s 포트 높이에 맞춰 뒤로 정렬). 아래: 타이어(단방향 접촉) → 비현가 질량 → 적분.
% 블록 위치는 첫 열 x와 대략의 y만 주고, 나머지는 align으로 포트 높이를 맞춘다.
add_block('built-in/Subsystem', sys);
blk(sys, 'Inport', 'z_b', [30 0 60 14]);
blk(sys, 'Inport', 'zd_b', [30 0 60 14]);
blk(sys, 'Inport', 'zr', [30 443 60 457]);
blk(sys, 'Inport', 'F_act', [30 0 60 14]);
blk(sys, 'Inport', 'F_arb', [30 0 60 14]);
blk(sys, 'Sum', 'F_s', [360 60 390 380], 'Inputs', '++++', 'IconShape', 'rectangular');
blk(sys, 'Gain', 'spring', [220 0 290 40], 'Gain', sprintf('-veh.k(%d)', i));
blk(sys, 'Gain', 'damper', [220 0 290 40], 'Gain', sprintf('-veh.c(%d)', i));
blk(sys, 'Sum', 'd', [130 0 160 50], 'Inputs', '+-', 'IconShape', 'rectangular');
blk(sys, 'Sum', 'dv', [130 0 160 50], 'Inputs', '+-', 'IconShape', 'rectangular');
align(sys, 'spring', 1, 'F_s', 1, 'src');  align(sys, 'd', 1, 'spring', 1, 'src');  align(sys, 'z_b', 1, 'd', 1, 'src');
align(sys, 'damper', 1, 'F_s', 2, 'src');  align(sys, 'dv', 1, 'damper', 1, 'src'); align(sys, 'zd_b', 1, 'dv', 1, 'src');
align(sys, 'F_act', 1, 'F_s', 3, 'src');   align(sys, 'F_arb', 1, 'F_s', 4, 'src');
blk(sys, 'Outport', 'F_s_out', [900 0 930 14]);
align(sys, 'F_s', 1, 'F_s_out', 1);

blk(sys, 'Sum', 'tire defl', [130 400 160 450], 'Inputs', '+-', 'IconShape', 'rectangular');
blk(sys, 'Gain', 'kt', [220 400 270 440], 'Gain', 'veh.kt');
blk(sys, 'Sum', 'Fz lin', [320 400 350 500], 'Inputs', '++', 'IconShape', 'rectangular');
blk(sys, 'MinMax', 'contact', [410 400 450 500], 'Function', 'max', 'Inputs', '2');
blk(sys, 'Sum', 'net', [520 380 550 500], 'Inputs', '+--', 'IconShape', 'rectangular');
blk(sys, 'Gain', 'inv m_u', [600 400 660 440], 'Gain', sprintf('1/veh.mu(%d)', i));
blk(sys, 'Integrator', 'zud', [710 400 750 440], 'InitialCondition', sprintf('x0(%d)', 10 + i));
blk(sys, 'Integrator', 'zu', [800 400 840 440], 'InitialCondition', sprintf('x0(%d)', 3 + i));
chain = {'zr', 'tire defl', 'kt', 'Fz lin', 'contact', 'net', 'inv m_u', 'zud', 'zu'};
for k = 2:numel(chain), align(sys, chain{k-1}, 1, chain{k}, 1); end
blk(sys, 'Constant', 'Fz0', [220 0 280 30], 'Value', sprintf('veh.Fz0(%d)', i));
blk(sys, 'Constant', '0', [380 0 395 20], 'Value', '0', 'ShowName', 'off');
align(sys, 'Fz0', 1, 'Fz lin', 2, 'src');  align(sys, '0', 1, 'contact', 2, 'src');
blk(sys, 'Outport', 'd_out', [900 33 930 47]);   % 맨 위 (d 출력이 위로 돌아 나감)
blk(sys, 'Outport', 'zu_out', [900 0 930 14]);
blk(sys, 'Outport', 'zud_out', [900 0 930 14]);
blk(sys, 'Outport', 'Fz', [900 0 930 14]);
align(sys, 'zu', 1, 'zu_out', 1);
for k = 1:2   % 나머지 출력은 zu_out 아래로 60 간격
    o = {'zud_out', 'Fz'};
    set_param([sys '/' o{k}], 'Position', get_param([sys '/zu_out'], 'Position') + [0 60 0 60] * k);
end

line(sys, 'z_b/1', 'd/1');          line(sys, 'zd_b/1', 'dv/1');
line(sys, 'd/1', 'spring/1');       line(sys, 'dv/1', 'damper/1');
line(sys, 'spring/1', 'F_s/1');     line(sys, 'damper/1', 'F_s/2');
line(sys, 'F_act/1', 'F_s/3');      line(sys, 'F_arb/1', 'F_s/4');
line(sys, 'F_s/1', 'F_s_out/1');
for k = 2:numel(chain), line(sys, [chain{k-1} '/1'], [chain{k} '/1']); end
line(sys, 'Fz0/1', 'Fz lin/2');     line(sys, '0/1', 'contact/2');
% m_u·zu'' = (Fz − Fz0) − F_s   (정적 평형 기준, 중력과 Fz0 상쇄)
line(sys, 'Fz0/1', 'net/2');        line(sys, 'F_s/1', 'net/3');
line(sys, 'zu/1', 'd/2');           line(sys, 'zu/1', 'tire defl/2'); line(sys, 'zud/1', 'dv/2');
line(sys, 'zu/1', 'zu_out/1');      line(sys, 'zud/1', 'zud_out/1');
line(sys, 'contact/1', 'Fz/1');     line(sys, 'd/1', 'd_out/1');
end
