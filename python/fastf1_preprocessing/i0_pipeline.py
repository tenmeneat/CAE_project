"""I0 생성 핵심 로직 (FastF1 의존성 없음).

입력: 랩별 텔레메트리 표(LapTrace 리스트) + TUMFTM 레이스라인
출력: I0 표 (s, x, y, z, v, throttle, brake) + 진단 표 + 메타데이터

설계안 4.1 처리 순서 중 1–2단계(레이스라인 Δs 재샘플, FastF1 Z → s 정렬·평균·평활)를 담당한다.
곡률 κ(s), 경사 θ(s), 수직 곡률 κv(s) 계산과 속도 프로파일은 블록 1(MATLAB)에서 한다.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import pandas as pd

from geometry import (
    ClosedCurve,
    StartOffset,
    estimate_start_offset,
    periodic_gradient,
    resample_closed,
    smooth_periodic,
)

I0_COLUMNS = ["s", "x", "y", "z", "v", "throttle", "brake"]
SPA_OFFICIAL_ELEVATION_RANGE_M = 102.2  # [R3]


@dataclass
class LapTrace:
    """한 랩의 텔레메트리. 단위는 SI로 변환된 상태여야 한다.

    data 컬럼: distance [m], x, y, z [m], v [m/s], throttle [0–1], brake [0/1]
    """

    data: pd.DataFrame
    driver: str = ""
    lap_number: int = -1
    lap_time_s: float = float("nan")
    use_for_speed: bool = True     # 속도·스로틀·브레이크 평균에 쓸지 (퀵랩·슬릭 여부)


@dataclass
class I0Params:
    ds: float = 1.0                      # 출력 격자 간격 [m]
    z_cutoff_wavelength: float = 50.0    # 고저 평활 차단 파장 [m] (D6 민감도 파라미터)
    n_match: int = 700                   # 시작점 오프셋 정합용 점 수 (~10 m 간격)
    lap_length_tol: float = 0.02         # 랩 적분거리가 중앙값에서 ±2% 넘게 벗어나면 제외
    z_outlier_k: float = 5.0             # z 이탈 랩 제외 기준 (median + k·MAD)
    allow_reflection: bool = True        # FastF1 좌표계가 거울상일 가능성 허용


@dataclass
class I0Result:
    i0: pd.DataFrame
    diag: pd.DataFrame
    meta: dict
    warnings: list[str] = field(default_factory=list)


# --------------------------------------------------------------------------- #
def _lap_fraction(d: pd.DataFrame) -> tuple[np.ndarray, float]:
    dist = d["distance"].to_numpy(float)
    d0, d1 = dist[0], dist[-1]
    return (dist - d0) / (d1 - d0), float(d1 - d0)


def _clean_lap(df: pd.DataFrame) -> pd.DataFrame:
    need = ["distance", "x", "y", "z", "v", "throttle", "brake"]
    missing = [c for c in need if c not in df.columns]
    if missing:
        raise ValueError(f"LapTrace 컬럼 누락: {missing}")
    d = df[need].dropna().sort_values("distance")
    # 정지 구간 등으로 거리 중복 → 첫 샘플만
    d = d.loc[~d["distance"].duplicated()]
    return d.reset_index(drop=True)


def build_i0(laps: list[LapTrace], raceline_xy: np.ndarray, params: I0Params | None = None) -> I0Result:
    p = params or I0Params()
    warn: list[str] = []

    curve: ClosedCurve = resample_closed(raceline_xy, ds=p.ds)
    L, ds = curve.length, curve.ds

    # ---- 1. 랩 정리 + 진행률 ------------------------------------------------
    prepared = []
    for lap in laps:
        try:
            d = _clean_lap(lap.data)
        except ValueError as e:
            warn.append(f"{lap.driver} L{lap.lap_number}: {e}")
            continue
        if len(d) < 50:
            warn.append(f"{lap.driver} L{lap.lap_number}: 샘플 {len(d)}개 — 제외")
            continue
        f, lap_len = _lap_fraction(d)
        prepared.append((lap, d, f, lap_len))
    if len(prepared) < 3:
        raise RuntimeError(f"사용 가능한 랩이 {len(prepared)}개뿐 — 최소 3개 필요")

    lens = np.array([x[3] for x in prepared])
    med_len = float(np.median(lens))
    keep = np.abs(lens / med_len - 1) <= p.lap_length_tol
    n_len_rej = int((~keep).sum())
    if n_len_rej:
        warn.append(f"랩 적분거리 이탈(±{p.lap_length_tol:.0%})로 {n_len_rej}랩 제외")
    prepared = [x for x, k in zip(prepared, keep) if k]

    # ---- 2. 시작점 오프셋 (랩 진행률 → 레이스라인 s) ------------------------
    f_match = np.arange(p.n_match) / p.n_match
    xy_stack = np.stack([
        np.column_stack([np.interp(f_match, f, d["x"]), np.interp(f_match, f, d["y"])])
        for _, d, f, _ in prepared
    ])
    f1_xy = np.median(xy_stack, axis=0)
    off: StartOffset = estimate_start_offset(f1_xy, curve, n_match=p.n_match,
                                             allow_reflection=p.allow_reflection)
    if off.direction < 0:
        warn.append("FastF1 주행 방향이 TUMFTM 레이스라인과 반대로 정합됨 — 데이터 확인 필요")
    if abs(off.transform.scale - 1) > 0.03:
        warn.append(f"FastF1→TUMFTM 스케일 {off.transform.scale:.4f} (1에서 3% 초과) — 단위/트랙 확인")

    # ---- 3. 출력 격자로 보간 -----------------------------------------------
    f_out = off.fraction_of_s(curve.s, L)

    def stack(col: str, subset) -> np.ndarray:
        return np.stack([np.interp(f_out, f, d[col].to_numpy(float)) for _, d, f, _ in subset])

    z_all = stack("z", prepared)
    z_med0 = np.median(z_all, axis=0)
    dev = np.sqrt(((z_all - z_med0) ** 2).mean(axis=1))
    mad = np.median(np.abs(dev - np.median(dev))) * 1.4826
    z_keep = dev <= np.median(dev) + p.z_outlier_k * max(mad, 1e-6)
    if (~z_keep).any():
        warn.append(f"z 이탈 랩 {int((~z_keep).sum())}개 제외 (RMS 편차 > median + {p.z_outlier_k}·MAD)")
    z_all = z_all[z_keep]
    z_med = np.median(z_all, axis=0)
    z_smooth = smooth_periodic(z_med, ds, p.z_cutoff_wavelength)

    speed_laps = [x for x in prepared if x[0].use_for_speed]
    if len(speed_laps) < 1:
        warn.append("속도용 랩 없음 — 전체 클린 랩으로 대체")
        speed_laps = prepared
    v_all = stack("v", speed_laps)
    thr_all = stack("throttle", speed_laps)
    brk_all = stack("brake", speed_laps)

    i0 = pd.DataFrame({
        "s": curve.s,
        "x": curve.xy[:, 0],
        "y": curve.xy[:, 1],
        "z": z_smooth,
        "v": np.median(v_all, axis=0),
        "throttle": np.median(thr_all, axis=0),
        "brake": brk_all.mean(axis=0),          # 해당 위치에서 브레이크를 밟은 랩 비율 (0–1)
    })[I0_COLUMNS]

    q = lambda a, lo, hi: np.percentile(a, hi, axis=0) - np.percentile(a, lo, axis=0)  # noqa: E731
    xy_f1_in_tumftm = off.transform.apply(f1_xy)
    diag = pd.DataFrame({
        "s": curve.s,
        "z_median_raw": z_med,
        "z_iqr": q(z_all, 25, 75),
        "v_iqr": q(v_all, 25, 75),
        "grade": periodic_gradient(z_smooth, ds),
    })

    # ---- 4. 검증 지표 -------------------------------------------------------
    grade = diag["grade"].to_numpy()
    elev_range = float(z_smooth.max() - z_smooth.min())
    xy_res = xy_f1_in_tumftm - np.array([
        np.interp(off.s_of_fraction(f_match, L), np.r_[curve.s, L], np.r_[curve.xy[:, i], curve.xy[0, i]])
        for i in (0, 1)
    ]).T
    meta = {
        "track_length_raceline_m": L,
        "ds_m": ds,
        "n_points": len(curve.s),
        "start_offset_m": off.offset,
        "direction": off.direction,
        "f1_to_tumftm": {
            "scale": off.transform.scale,
            "rotation_deg": float(np.degrees(np.arctan2(off.transform.rot[1, 0], off.transform.rot[0, 0]))),
            "translation_m": off.transform.trans.tolist(),
            "reflected": off.transform.reflected,
            "rms_m": off.transform.rms,
            "max_abs_residual_m": float(np.hypot(*xy_res.T).max()),
        },
        "laps": {
            "input": len(laps),
            "after_cleanup": len(prepared),
            "used_for_z": int(z_keep.sum()),
            "used_for_speed": len(speed_laps),
            "lap_distance_median_m": med_len,
            "lap_distance_over_raceline": med_len / L,
        },
        "z": {
            "cutoff_wavelength_m": p.z_cutoff_wavelength,
            "elevation_range_m": elev_range,
            "elevation_range_raw_median_m": float(z_med.max() - z_med.min()),
            "official_range_m_spa": SPA_OFFICIAL_ELEVATION_RANGE_M,
            "grade_max_pct": float(100 * grade.max()),
            "grade_min_pct": float(100 * grade.min()),
            "lap_spread_iqr_median_m": float(np.median(diag["z_iqr"])),
        },
        "units": {"s": "m", "x": "m", "y": "m", "z": "m", "v": "m/s", "throttle": "0-1",
                  "brake": "fraction of laps braking (0-1)"},
        "conventions": {
            "s_origin": "TUMFTM 레이스라인 재샘플 첫 점 (F1 타이밍 라인 = s_of_fraction(0) = start_offset_m)",
            "xy_frame": "TUMFTM 좌표계 (재샘플한 레이스라인 좌표)",
            "z": "FastF1 Z (1/10 m → m), 절대 기준 임의 — 상대 고저만 의미",
        },
    }
    return I0Result(i0=i0, diag=diag, meta=meta, warnings=warn)
