# data/

생성되는 데이터는 대부분 `.gitignore`로 커밋 제외 (용량 + 재생성 가능).

- `tracks/` — TUMFTM `racetrack-database` 클론 또는 필요한 서킷 CSV만 복사해서 넣는 위치.
  ```
  git clone https://github.com/TUMFTM/racetrack-database data/tracks/racetrack-database
  ```
- FastF1 원본 텔레메트리는 `fastf1` 라이브러리의 자체 캐시(`fastf1.Cache.enable_cache(...)`)를 쓰고, 이 레포에는
  전처리된 `fastf1_<track>.csv` (인터페이스 I0)만 필요 시 `data/raw/`에 생성 — 이것도 git에는 안 올라감.
