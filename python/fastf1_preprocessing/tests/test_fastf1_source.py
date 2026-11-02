"""fastf1_source 랩 선별·단위 변환 검증 (네트워크 없이 FastF1 Laps 객체를 직접 구성)."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

fastf1 = pytest.importorskip("fastf1")
from fastf1.core import Laps  # noqa: E402

import fastf1_source as src  # noqa: E402

NaT = pd.NaT
td = pd.to_timedelta


def make_laps() -> Laps:
    rows = [
        # Driver, LapNumber, LapTime[s], PitIn, PitOut, TrackStatus, Compound, Deleted
        ("VER", 1, 110.0, NaT, NaT, "1", "MEDIUM", False),     # 1랩 → 제외
        ("VER", 2, 107.0, NaT, NaT, "1", "MEDIUM", False),     # 클린 + 퀵
        ("VER", 3, 107.5, NaT, NaT, "12", "MEDIUM", False),    # 옐로 포함 → 클린 + 퀵
        ("VER", 4, 130.0, NaT, NaT, "14", "MEDIUM", False),    # SC → 제외
        ("VER", 5, 125.0, td(1, "s"), NaT, "1", "MEDIUM", False),  # 피트 인 → 제외
        ("HAM", 2, 109.0, NaT, NaT, "1", "HARD", False),       # 클린, 퀵(1.03 이내: 110.21)
        ("HAM", 3, 111.0, NaT, NaT, "1", "HARD", False),       # 클린, 퀵 아님
        ("HAM", 4, 108.0, NaT, NaT, "1", "HARD", True),        # 삭제 랩 → 제외
        ("HAM", 5, 115.0, NaT, NaT, "16", "HARD", False),      # VSC → 제외
        ("NOR", 2, 108.5, NaT, NaT, "1", "INTERMEDIATE", False),  # 클린, 웻 → 속도 제외
    ]
    df = pd.DataFrame(rows, columns=["Driver", "LapNumber", "LapTime", "PitInTime", "PitOutTime",
                                     "TrackStatus", "Compound", "Deleted"])
    df["LapTime"] = td(df["LapTime"], unit="s")
    return Laps(df)


def test_select_laps():
    sel = src.select_laps(make_laps(), quick_threshold=1.03)
    clean = set(zip(sel.clean["Driver"], sel.clean["LapNumber"]))
    speed = set(zip(sel.speed["Driver"], sel.speed["LapNumber"]))
    assert clean == {("VER", 2), ("VER", 3), ("HAM", 2), ("HAM", 3), ("NOR", 2)}
    assert speed == {("VER", 2), ("VER", 3), ("HAM", 2)}
    assert sel.summary["wet_laps_in_clean"] == 1
    assert sel.summary["fastest_lap_s"] == pytest.approx(107.0)


class FakeLap(dict):
    def __init__(self, tel):
        super().__init__(Driver="VER", LapNumber=7, LapTime=td(106.5, unit="s"))
        self._tel = tel

    def get_telemetry(self):
        return self._tel


def test_lap_to_trace_units():
    tel = pd.DataFrame({
        "Distance": [0.0, 50.0, 100.0], "X": [1000.0, 1500.0, 2000.0], "Y": [-10.0, 0.0, 10.0],
        "Z": [1023.0, 1025.0, 1030.0], "Speed": [180.0, 216.0, 252.0], "Throttle": [100.0, 104.0, 0.0],
        "Brake": [False, False, True],
    })
    tr = src.lap_to_trace(FakeLap(tel), use_for_speed=True)
    d = tr.data
    assert np.allclose(d["x"], [100, 150, 200]) and np.allclose(d["z"], [102.3, 102.5, 103.0])
    assert np.allclose(d["v"], [50, 60, 70])
    assert np.allclose(d["throttle"], [1, 1, 0]) and np.allclose(d["brake"], [0, 0, 1])
    assert tr.lap_time_s == pytest.approx(106.5) and tr.driver == "VER" and tr.lap_number == 7
