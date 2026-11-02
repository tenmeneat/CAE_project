# data/

생성되는 데이터는 대부분 `.gitignore`로 커밋 제외 (용량 + 재생성 가능). 예외는 아래 두 가지 — 작고, 다시 만들려면
외부 네트워크가 필요해서 레포에 올린다.

- `tracks/tumftm/` — TUMFTM `racetrack-database`에서 **Spa 레이스라인·중심선만 발췌** (LGPL-3.0, 출처 커밋은
  `tracks/tumftm/README.md`). 다른 서킷은 같은 경로 구조로 복사해 넣는다. 전체 저장소를
  `tracks/racetrack-database/`에 클론해도 되지만 그 디렉토리는 git에서 제외된다.
- `raw/fastf1_<track>.csv` — 인터페이스 I0. `python/fastf1_preprocessing/fastf1_to_csv.py`로 생성해 **커밋한다**.
  `raw/` 안의 다른 파일은 커밋 제외. FastF1 원본 텔레메트리는 `~/.cache/fastf1` 캐시에만 둔다.
