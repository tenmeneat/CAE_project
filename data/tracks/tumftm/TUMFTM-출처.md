# TUMFTM racetrack-database (Spa만 발췌)

출처: https://github.com/TUMFTM/racetrack-database, 커밋 `e59595d1f3573b30d1ded6a08984935b957688e0` (2021-09-18).
라이선스: LGPL-3.0 (`LICENSE` 원문 동봉). 파일은 수정하지 않고 그대로 복사했다.

| 파일 | 내용 | 용도 |
|---|---|---|
| `racelines/Spa.csv` | `# x_m,y_m` 최소곡률 레이스라인 | I0의 `s` 기준, 블록 1 시나리오 A |
| `tracks/Spa.csv` | `# x_m,y_m,w_tr_right_m,w_tr_left_m` 중심선 + 트랙 폭 | 블록 1 시나리오 B |

다른 서킷이 필요하면 원본 저장소에서 같은 경로(`racelines/<track>.csv`, `tracks/<track>.csv`)로 복사해 넣는다.

## 연결

- 폴더 규칙: [데이터-폴더](../../데이터-폴더.md) · 사용처: [전처리-FastF1-I0](../../../python/fastf1_preprocessing/전처리-FastF1-I0.md), [블록1-랩시뮬레이션](../../../matlab/block1_lap_sim/블록1-랩시뮬레이션.md)
