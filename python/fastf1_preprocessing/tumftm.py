"""TUMFTM racetrack-database 로더 [R1].

tracks/<name>.csv   : x_m, y_m, w_tr_right_m, w_tr_left_m  (중심선 + 좌우 폭)
racelines/<name>.csv: x_m, y_m                             (최소곡률 레이스라인)

주의: 파일의 첫 점이 출발/결승선이라는 명시가 README에 없다 → 시작점 오프셋은 FastF1 정합으로 추정한다.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

DEFAULT_DB = Path(__file__).resolve().parents[2] / "data" / "tracks" / "racetrack-database"


@dataclass
class TumftmTrack:
    name: str
    centerline_xy: np.ndarray   # (N, 2)
    width_right: np.ndarray     # (N,)
    width_left: np.ndarray      # (N,)
    raceline_xy: np.ndarray     # (M, 2)


def load_track(name: str, db_root: Path | str = DEFAULT_DB) -> TumftmTrack:
    db_root = Path(db_root)
    track_csv = db_root / "tracks" / f"{name}.csv"
    race_csv = db_root / "racelines" / f"{name}.csv"
    for p in (track_csv, race_csv):
        if not p.exists():
            raise FileNotFoundError(
                f"{p} 없음. 먼저 클론: git clone https://github.com/TUMFTM/racetrack-database {db_root}"
            )
    t = np.loadtxt(track_csv, delimiter=",", comments="#")
    r = np.loadtxt(race_csv, delimiter=",", comments="#")
    return TumftmTrack(name=name, centerline_xy=t[:, :2], width_right=t[:, 2], width_left=t[:, 3],
                       raceline_xy=r[:, :2])
