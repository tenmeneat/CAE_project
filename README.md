# CAE Project — 서킷 주행 하중 기반 액티브 서스펜션 제어와 로어암(LCA) 응력 트레이드오프 해석

한양대 미래자동차공학과 CAE 팀 프로젝트 (2인 팀). 설계안 원본(최신): [`docs/design/CAE_design_v0.8.pdf`](docs/design/CAE_design_v0.8.pdf).
이전 버전: [`docs/design/CAE_design_v0.2.pdf`](docs/design/CAE_design_v0.2.pdf).

**연구질문**: 하나의 하드웨어(액추에이터 마운트 위치)로 스포츠 주행의 접지와 컴포트 주행의 승차감을 지키면서,
액추에이터 부하와 로어암 응력을 어디까지 줄일 수 있는가?

> v0.2 → v0.8 사이에 주제 축 자체가 바뀌었다 (피로수명 절대값 → 3축 상대 트레이드오프, 시나리오 A/B 이원화,
> 마운트 위치 설계변수 추가, 블록 4 분리, 담당 재배정). 변경 요약은 [`docs/architecture.md`](docs/architecture.md) 상단 표 참고.

이 레포는 **스캐폴딩 단계**다 — 모든 블록이 함수 시그니처 + `README.md` + `TODO`만 있는 상태고 실제 구현은
비어 있다.

## 먼저 읽을 것

1. [`docs/architecture.md`](docs/architecture.md) — 전체 파이프라인(블록 1–7), v0.2→v0.8 변경 요약, 함정 2개, 평가 지표·선정 규칙
2. [`docs/interfaces.md`](docs/interfaces.md) — 블록 간 파일 계약 (I0–I5)
3. [`docs/decisions.md`](docs/decisions.md) — 미결정 사항(D1–D18) 트래커. 확정 18건 / 결정 필요 9건, 막는 순서대로 정리됨
4. [`docs/schedule.md`](docs/schedule.md) — **날짜 없는 게이트 기반 작업 순서** (W1–W7)

## 담당: 내가 A (동역학·제어·전처리·뷰어)

**D8 확정(v0.8)으로 뷰어(블록 7)가 B에서 A로 넘어왔다.** A가 블록 1·2·6·7, 전처리, 뷰어까지 맡는다 — 부하가
크므로 뷰어는 최소판(트랙맵+차량마커+동기그래프)만 필수로 두고 확장 기능은 제일 먼저 버린다
(`docs/schedule.md` 축소 우선순위).

| 담당 | 블록 | 디렉토리 | 툴 |
|---|---|---|---|
| **A (나)** | 1. 랩 시뮬레이션 (시나리오 A/B) | `matlab/block1_lap_sim/` | MATLAB |
| | 2. 풀카 7DOF + 액티브 제어 | `matlab/block2_fullcar_control/` | Simulink |
| | 6. 마운트·시나리오·제어 스윕 + 2단계 선정 | `matlab/block6_sweep_schedule/` | MATLAB |
| | 7. 리플레이 뷰어 (최소판 필수) | `matlab/block7_replay_viewer/` | MATLAB |
| | FastF1 전처리 (I0 생성) | `python/fastf1_preprocessing/` | Python |
| **B** | 3. 휠력 → 로어암 하중 (FBD, 마운트 위치 MR(p) 환산) | `matlab/block3_wheel_to_lca_fbd/` | MATLAB |
| | 4a. 로어암 구조 해석 | `comsol/block4a_lca_structure/` | COMSOL |
| | 4b. 액추에이터 전기-열-구조 연성 (신규, 필수) | `comsol/block4b_actuator_thermal/` + `matlab/block4b_actuator_thermal/` | COMSOL + MATLAB |
| | 5. 응력 중첩 → 응력 지표 | `matlab/block5_stress_metrics/` | MATLAB |

각 디렉토리의 `README.md`에 해당 블록의 목적·절차·전제조건·TODO가 정리돼 있다.

## 지금 막혀 있는 순서

블록 7(뷰어 최소판)과 FastF1 전처리(I0)를 빼면, 아래 순서로 결정이 안 되면 다음 블록을 못 짠다
(`docs/decisions.md` 전체 참고):

1. **D16 (프런트 마운트 위치 p 3수준 값)** — 블록 2·3·4a 전부가 여기 의존. 선행 조건: D3(하드포인트)·D15(액추에이터
   포화 기준). **셋 다 W1 안에 연쇄로 끝내야 함**
2. **D15 (액추에이터 사양 출처)** — A가 W1에 직접 조사하기로 확정. D16의 입력
3. **D4 (로어암 재질)** — 블록 5의 S-N 지수 m, 인장강도 σu 전제
4. **D12 (안전계수/비교 방식)** — 블록 5·6의 제약 판정(정적 안전계수, DEL vs 피로한도) 전제
5. **D14 (스윕 격자·임계값)** — 블록 6을 실제로 돌리기 전 W1에 승인·고정해야 격자 크기(~300–480회)가 보장됨

**지금 바로 시작해도 되는 것**: 블록 1(랩 시뮬, TUMFTM 트랙 로더 + 시나리오 A/B 뼈대), FastF1 전처리 스크립트,
블록 7 뷰어 최소판 더미 데이터 뼈대, 블록 4b(COMSOL 열해석 모델 구성 — 액추에이터 사양과 독립적으로 2D
축대칭 형상은 먼저 잡을 수 있음) — 이 넷은 위 결정사항과 독립적이다.

## 인터페이스 요약

블록은 전부 **파일로만** 통신한다 (I0–I5, 자세한 스키마는 `docs/interfaces.md`):

```
Python(FastF1) --I0--> 블록1(A, 시나리오A/B) --(내부)--> 블록2(A) --I1--> 블록3(B) --(F_LCA)--> 4a(B)/4b(B)
                                                             ^                                      │
                                                             │                                  C / I5(Fth)
                                                        블록6(A) <--I2-- 블록5(B) <───────────────────┘
                                                             │
                                                             ├──I3──> 블록7(A, 뷰어)
                                                             └──I4──> 뷰어·슬라이드
```

Q/R 가중치만 바꿀 때는 블록 2→3→5만 재실행하면 되고, COMSOL(4a) 재해석은 필요 없다. 4a는 형상·두께가, 4b는
액추에이터 사양이 바뀔 때만 다시 돌린다. 마운트 위치 p가 바뀌면 블록 2·3·4a를 전부 다시 실행해야 한다.

## 평가 지표 (3~4축)

접지(A 주 지표) / 승차감(B 주 지표, ISO 2631-1 가중 RMS) / 액추에이터 부담(RMS·피크 힘, 소비전력, 열 여유
F_rms/F_th) / 로어암 응력(핫스팟 피크 응력, 등가응력진폭 DEL, Passive 대비 상대 손상비 R). **절대 피로수명은
예측하지 않는다** — Passive 대비 비율로만 비교. 선정은 2단계 규칙(모드별 ε-제약 → DEL 최소 → 마운트 위치
p\* 결정, `docs/architecture.md` 참고).

## 환경

- MATLAB/Simulink (Control System Toolbox — `lqr()`, Signal Processing Toolbox — `rainflow()`)
- COMSOL — Structural Mechanics 모듈 확인됨(D7). AC/DC 모듈은 없음 → 액추에이터 힘 상수 Kf는 문헌값 사용.
  LiveLink for MATLAB 유무는 W1–W3에 확인
- Python 3.x + `fastf1` (전처리 전용, 나머지 파이프라인은 MATLAB/Simulink)

```
pip install fastf1 numpy pandas scipy
git clone https://github.com/TUMFTM/racetrack-database data/tracks/racetrack-database
```

## 검증 체크리스트

v0.3 기준 항목 + v0.4~v0.8에서 추가된 신규 항목(총 30개+)은 원본 설계안 8장에 전체 목록이 있고, 블록별 항목은
각 블록 `README.md`에 옮겨뒀다. 핵심만 요약하면:

- 랩 시뮬 속도가 FastF1 실측과 제동점/코너 정점에서 맞는지 (+ 시나리오 B가 가속도 상한을 지키는지)
- 풀카 고유치가 차체 1–1.5Hz, 휠홉 10–15Hz 대역인지
- COMSOL: 메쉬 수렴 + 곡선빔·빔 인터페이스 3중 비교, Rigid Connector 실사용 여부
- 로어암 1차 고유진동수가 노면 가진 대역보다 충분히 높은지 (준정적 가정 정당화)
- 액추에이터 열해석: Rth를 1D 손계산과 비교, Kf/h 민감도
- `lca_damage()`를 단순 정현파 하중 손계산과 대조, σf'·m을 흔들어도 제어기 순위가 유지되는지

## 변경 이력

이 README/스캐폴딩은 설계안 **v0.8** (2026-10-29) 기준으로 작성됨. 이전: v0.2 (2026-10-27, 최초 스캐폴딩).
