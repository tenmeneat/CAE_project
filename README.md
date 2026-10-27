# CAE Project — 서킷 주행 하중 기반 액티브 서스펜션 제어와 로어암(LCA) 피로수명 해석

한양대 미래자동차공학과 CAE 팀 프로젝트 (2인 팀). 설계안 원본: [`docs/design/CAE_design_v0.2.pdf`](docs/design/CAE_design_v0.2.pdf) (v0.2).

**연구질문**: 고속 서킷 주행에서 액티브 서스펜션 제어 전략(승차감 우선 / 접지 우선 / 위치 스케줄)이 로어암
피로손상에 어떤 영향을 주는가?

이 레포는 아직 **스캐폴딩 단계**다 — 모든 블록이 함수 시그니처 + `README.md` + `TODO`만 있는 상태고 실제 구현은
비어 있다. 구조부터 먼저 고정해서 2인이 더미 데이터로 병렬 개발할 수 있게 만드는 게 목적.

## 먼저 읽을 것

1. [`docs/architecture.md`](docs/architecture.md) — 전체 파이프라인(블록 1–7), 함정 2개, 핵심 설계 결정
2. [`docs/interfaces.md`](docs/interfaces.md) — 블록 간 파일 계약 (I0–I3). **이거 안 정하면 병렬 개발 불가**
3. [`docs/decisions.md`](docs/decisions.md) — 미결정 사항(D1–D9) 트래커. 뭐부터 정해야 막힌 게 풀리는지 순서대로 정리됨

## 어떤 파트를 진행해야 하는지

| 담당 | 블록 | 디렉토리 | 툴 |
|---|---|---|---|
| **A (동역학·제어)** | 1. 랩 시뮬레이션 | `matlab/block1_lap_sim/` | MATLAB |
| | 2. 풀카 7DOF + 액티브 제어 | `matlab/block2_fullcar_control/` | Simulink |
| | 6. 가중치 스윕 + 스케줄 제어 | `matlab/block6_sweep_schedule/` | MATLAB |
| | FastF1 전처리 (I0 생성) | `python/fastf1_preprocessing/` | Python |
| **B (구조·내구)** | 3. 휠력 → 로어암 하중 (FBD) | `matlab/block3_wheel_to_lca_fbd/` | MATLAB |
| | 4. 로어암 단위하중 해석 | `comsol/block4_lca_unit_load/` | COMSOL |
| | 5. 응력 중첩 → 피로 | `matlab/block5_fatigue/` | MATLAB |
| | 7. 리플레이 뷰어 (주도) | `matlab/block7_replay_viewer/` | MATLAB |

각 디렉토리의 `README.md`에 해당 블록의 목적·절차·전제조건·TODO가 정리돼 있다. **각자 자기 파트 폴더의
README부터 읽고 시작할 것.**

담당자가 아직 안 정해졌으면 `docs/decisions.md`의 D8 참고 — 설계안 기준 역할 분담은 위 표와 동일.

## 지금 막혀 있는 순서 (docs/decisions.md 요약)

블록 7(뷰어)과 FastF1 전처리(I0)를 빼면, 나머지는 아래 순서로 결정이 안 되면 다음 블록을 못 짠다:

1. **D7 (COMSOL 라이선스/모듈 확인)** → 블록 4 시작 가능 여부 자체를 결정
2. **D1 (서스펜션 형식, 추천: 더블위시본)** → 블록 2·3의 하드포인트 기구학 전제
3. **D4 (로어암 재질, S-N 데이터 확보 가능한 쪽)** → 블록 5 피로 계산 전제
4. **D8 (A/B 담당자 배정)** → 병렬 작업 시작 조건

반대로 **지금 바로 시작해도 되는 것**: 블록 1(랩 시뮬, TUMFTM 트랙 로더), FastF1 전처리 스크립트, 블록 7 뷰어
더미 데이터 뼈대 — 이 셋은 위 결정사항과 독립적이다.

## 인터페이스 요약

블록은 전부 **파일로만** 통신한다 (I0–I3, 자세한 스키마는 `docs/interfaces.md`):

```
Python(FastF1) --I0--> 블록1 --(내부)--> 블록2 --I1--> 블록3 --(내부)--> 블록4
                                            ^                              │
                                            │                              C (응력계수)
                                       블록6 <--I2-- 블록5 <-----------------┘
                                            │
                                            └--I3--> 블록7 (리플레이)
```

Q/R 가중치만 바꿀 때는 블록 2→3→5만 재실행하면 되고, COMSOL(블록 4) 재해석은 필요 없다.

## 환경

- MATLAB/Simulink (Control System Toolbox — `lqr()`, Signal Processing Toolbox — `rainflow()`)
- COMSOL (Structural Mechanics 모듈, 가능하면 LiveLink for MATLAB — D7 확인 필요)
- Python 3.x + `fastf1` (전처리 전용, 나머지 파이프라인은 MATLAB/Simulink)

```
pip install fastf1 numpy pandas scipy
git clone https://github.com/TUMFTM/racetrack-database data/tracks/racetrack-database
```

## 검증 체크리스트

구현하면서 빠뜨리기 쉬운 것들 — 원본 설계안 7장에 전체 목록 있음. 핵심만:

- 랩 시뮬 속도가 FastF1 실측과 제동점/코너 정점에서 맞는지
- 풀카 고유치가 차체 1–1.5Hz, 휠홉 10–15Hz 대역인지
- COMSOL 메쉬 수렴 + 곡선빔 해석해 비교
- 로어암 1차 고유진동수가 노면 가진 대역보다 충분히 높은지 (준정적 가정 정당화)
- `lca_damage()`를 단순 정현파 하중 손계산과 대조

## 변경 이력

이 README/스캐폴딩은 설계안 v0.2 (2026-10-27) 기준으로 작성됨.
