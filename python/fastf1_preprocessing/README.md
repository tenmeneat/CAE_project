# FastF1 전처리 → I0 (담당 A)

FastF1 위치(X,Y,Z)·차량(Speed/Throttle/Brake) 스트림을 TUMFTM Spa 레이스라인의 거리축 `s`로 정렬해 인터페이스
**I0** (`data/raw/fastf1_<track>.csv`)로 내보낸다. 이후 파이프라인은 전부 MATLAB/Simulink — 파이썬은 여기서 끝.
**시나리오 A/B 분기는 블록 1(MATLAB)에서 하므로 I0은 시나리오와 무관하게 한 번만 만든다.**

설계안 4.1 처리 순서 중 1–2단계(레이스라인 Δs=1 m 재샘플, FastF1 Z → s 정렬·평균·평활)를 담당한다.
κ(s)·θ(s)·κv(s)와 속도 프로파일은 블록 1(MATLAB).

## 설치 · 실행 (레포 루트에서)

```bash
pip install -r python/fastf1_preprocessing/requirements.txt
```

```bash
git clone https://github.com/TUMFTM/racetrack-database data/tracks/racetrack-database
```

D9 세션 선정용 조사 (텔레메트리 없이 랩 표만 받음 — 빠름):

```bash
python python/fastf1_preprocessing/fastf1_to_csv.py survey --years 2019 2020 2022 2023 2024 2025
```

I0 생성 (연도는 survey 결과 보고 A가 정함):

```bash
python python/fastf1_preprocessing/fastf1_to_csv.py run --year <YEAR>
```

옵션: `--z-wavelength 50` (고저 평활 차단 파장 [m], D6 민감도 파라미터), `--quick 1.03` (속도용 랩 기준),
`--max-laps N` (텔레메트리 추출 랩 수 상한, 드라이버 라운드로빈). 전역 옵션(`--event`, `--session`, `--cache`,
`--quick`)은 서브커맨드 **앞**에 쓴다. FastF1 캐시는 `data/cache/fastf1/` (git 제외).

테스트 (네트워크 불필요):

```bash
python -m pytest python/fastf1_preprocessing/tests -q
```

## 출력 (data/raw/, git 제외)

| 파일 | 내용 |
|---|---|
| `fastf1_<track>.csv` | **I0**: `s, x, y, z, v, throttle, brake` (스키마는 `docs/interfaces.md`) |
| `fastf1_<track>_diag.csv` | `s, z_median_raw, z_iqr, v_iqr, grade` — 평활 전 고저, 랩 간 산포, 경사 |
| `fastf1_<track>_meta.json` | 세션, 랩 선별 결과, 시작점 오프셋, FastF1→TUMFTM 정합(스케일·회전·잔차), 고저차·경사, 경고 |

## 처리 내용

1. **랩 선별** (`fastf1_source.select_laps`, D9 기준)
   - 클린 랩 = 피트 인/아웃 제외, 1랩 제외, TrackStatus에 SC(4)·적기(5)·VSC(6,7) 포함 랩 제외, 삭제 랩 제외
   - 고저 z: 클린 랩 전부 (드라이버 무관 — 노면 고저는 차와 무관)
   - 속도·스로틀·브레이크: 클린 랩 중 슬릭 + 최속 랩 대비 `--quick` 이내
2. **단위 변환**: X,Y,Z 1/10 m → m, Speed km/h → m/s, Throttle % → 0–1, Brake bool → 0/1
3. **s 정렬** (`i0_pipeline.build_i0`): 랩마다 FastF1 적분거리를 진행률 f(0–1)로 정규화(설계안 4.1 "XY 정합 대신
   출발선 기준 거리"). TUMFTM 파일 첫 점이 타이밍 라인이라는 보장이 없어서, 랩 평균 XY와 레이스라인의
   2D 상사변환(Umeyama) 정합 잔차가 최소가 되는 **상수 시작점 오프셋 하나**만 추정한다. 주행 방향·거울상
   좌표계도 자동 판별. 적분거리가 중앙값에서 ±2% 넘게 벗어난 랩은 제외.
4. **고저 z(s)**: 랩별 z를 s 격자로 보간 → 이탈 랩 제외(RMS 편차 > median + 5·MAD) → 랩 중앙값 → 주기 평활.
   평활은 3차 평활 스플라인과 같은 전달함수 H = 1/(1+(λc/λ)⁴)를 FFT로 적용(폐곡선이라 경계 문제 없음).
   강도는 차단 파장 λc [m] 하나로 표현 (파장 λc 성분 진폭 1/2).
5. **속도류**: 속도용 랩의 중앙값(v, throttle), brake는 해당 위치에서 브레이크를 밟은 랩 비율.

## 실행 후 확인할 것 (설계안 8장 체크리스트)

- [ ] 고저차 `meta.z.elevation_range_m`가 공식값 Spa 102.2 m [R3]과 비슷한지 (평활 전 값도 같이 보기)
- [ ] Eau Rouge–Raidillon 구간이 오르막(경사 +)으로 나오는지 — Z 부호 확인. 공식 경사 17% [R4]는 평활 강도에
      따라 깎이므로 `--z-wavelength`를 바꿔 민감도를 본다(D6)
- [ ] `meta.f1_to_tumftm.scale` ≈ 1, `rms_m`가 수 m 이내 — 크면 트랙/단위 문제
- [ ] `meta.laps.lap_distance_over_raceline` ≈ 1 (FastF1 적분거리 vs TUMFTM 레이스라인 길이)
- [ ] `direction = +1`, 경고 목록 확인

## 주의

- F1 데이터는 **서킷 형상(고저 z)과 블록 1 방법론 검증용 속도 비교에만** 쓴다. 우리 차(F82 M4급) 하중 입력으로
  F1 속도를 직접 쓰지 않는다 (차급 불일치, F1은 액티브 서스펜션 금지).
- 4 Hz 속도 미분으로 가속도를 만들지 말 것 (노이즈 큼).
- FastF1 Z는 절대 해발이 아니다 — 상대 고저만 의미. 노면 요철은 안 잡힘 → ISO 8608로 블록 1에서 별도 중첩.
- 시나리오 B는 **중심선** s를 쓴다. I0은 레이스라인 s 기준이므로 블록 1에서 중심선 점 → 최근접 레이스라인 s로
  z를 옮겨야 한다.

## 후속 과제 (부록, 당장 작업 아님)

Spa 파이프라인이 끝나고 여유가 있으면 블록 1만 교체해 뉘르부르크링 노르트슐라이페를 추가한다. I0 계약은
유지된다. 데이터 소스는 OSM/RLP DGM1/TUMFTM 최적화기 조합 — 원본 설계안 부록 참고.
