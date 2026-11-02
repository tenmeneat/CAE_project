"""FastF1 세션 로드 · 랩 선별 · 텔레메트리 추출 [R8, R9].

랩 선별 기준 (D9, v0.8): Spa 레이스, 세이프티카·피트 입출구 랩 제외, 정상 랩 여러 개.
- 고저 z용: 클린 랩 전부 (드라이버 무관 — 노면 고저는 차와 무관하므로 많이 평균할수록 좋음)
- 속도·스로틀·브레이크용: 클린 랩 중 슬릭 타이어 + 최속 랩 대비 quick_threshold 이내
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

from i0_pipeline import LapTrace

# TrackStatus 코드: 1 green, 2 yellow, 4 SC, 5 red, 6 VSC, 7 VSC ending
NEUTRALIZED_STATUS = "4567"
SLICKS = ("SOFT", "MEDIUM", "HARD")


def enable_cache(cache_dir: Path) -> None:
    import fastf1

    cache_dir.mkdir(parents=True, exist_ok=True)
    fastf1.Cache.enable_cache(str(cache_dir))


def load_session(year: int, event: str, session: str, telemetry: bool = True):
    import fastf1

    ses = fastf1.get_session(year, event, session)
    ses.load(laps=True, telemetry=telemetry, weather=False, messages=True)
    if len(ses.drivers) == 0:
        raise RuntimeError(
            f"{year} {event} {session}: FastF1이 드라이버 0명을 반환 — 세션이 없거나 "
            "livetiming.formula1.com 접속 실패(네트워크/프록시 확인)"
        )
    return ses


@dataclass
class LapSelection:
    clean: pd.DataFrame        # FastF1 Laps (z용)
    speed: pd.DataFrame        # FastF1 Laps (v용, clean의 부분집합)
    summary: dict


def select_laps(laps, quick_threshold: float = 1.03) -> LapSelection:
    """FastF1 Laps 객체에서 클린 랩 / 속도용 랩을 고른다."""
    n_total = len(laps)
    sel = laps.pick_wo_box()
    n_wo_box = len(sel)
    sel = sel[sel["LapNumber"] > 1]                                  # 스탠딩 스타트 랩 제외
    sel = sel.pick_track_status(NEUTRALIZED_STATUS, how="none")      # SC/VSC/적기 제외
    n_green = len(sel)
    if "Deleted" in sel.columns:
        sel = sel[~sel["Deleted"].fillna(False).astype(bool)]
    sel = sel[sel["LapTime"].notna()]
    clean = sel

    speed = clean[clean["Compound"].isin(SLICKS)]
    if len(speed):
        speed = speed.pick_quicklaps(quick_threshold)

    summary = {
        "total": n_total,
        "without_pit_in_out": n_wo_box,
        "green_excl_lap1": n_green,
        "clean": len(clean),
        "speed_laps": len(speed),
        "quick_threshold": quick_threshold,
        "drivers_clean": int(clean["Driver"].nunique()) if len(clean) else 0,
        "wet_laps_in_clean": int((~clean["Compound"].isin(SLICKS)).sum()) if len(clean) else 0,
        "fastest_lap_s": float(clean["LapTime"].min().total_seconds()) if len(clean) else float("nan"),
    }
    return LapSelection(clean=clean, speed=speed, summary=summary)


def lap_to_trace(lap, use_for_speed: bool) -> LapTrace:
    """FastF1 Lap → LapTrace (SI 단위 변환).

    X, Y, Z: 1/10 m → m, Speed: km/h → m/s, Throttle: % → 0–1, Brake: bool → 0/1.
    Distance는 FastF1이 속도 적분으로 계산한 랩 시작 기준 거리 [m].
    """
    tel = lap.get_telemetry()
    df = pd.DataFrame({
        "distance": tel["Distance"].to_numpy(float),
        "x": tel["X"].to_numpy(float) / 10.0,
        "y": tel["Y"].to_numpy(float) / 10.0,
        "z": tel["Z"].to_numpy(float) / 10.0,
        "v": tel["Speed"].to_numpy(float) / 3.6,
        "throttle": np.clip(tel["Throttle"].to_numpy(float) / 100.0, 0.0, 1.0),
        "brake": tel["Brake"].astype(float).to_numpy(),
    })
    lt = lap["LapTime"]
    return LapTrace(
        data=df,
        driver=str(lap["Driver"]),
        lap_number=int(lap["LapNumber"]),
        lap_time_s=float(lt.total_seconds()) if pd.notna(lt) else float("nan"),
        use_for_speed=use_for_speed,
    )


def extract_traces(selection: LapSelection, log=print) -> tuple[list[LapTrace], list[str]]:
    speed_idx = set(selection.speed.index)
    traces, errors = [], []
    n = len(selection.clean)
    for k, (idx, lap) in enumerate(selection.clean.iterlaps()):
        try:
            traces.append(lap_to_trace(lap, use_for_speed=idx in speed_idx))
        except Exception as e:  # noqa: BLE001 — 랩 단위 데이터 결손은 건너뛴다
            errors.append(f"{lap['Driver']} L{int(lap['LapNumber'])}: {type(e).__name__}: {e}")
        if (k + 1) % 50 == 0:
            log(f"  텔레메트리 {k + 1}/{n}")
    return traces, errors
