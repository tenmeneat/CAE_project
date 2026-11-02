"""FastF1 → I0 (fastf1_<track>.csv) 전처리 CLI.

사용 예 (레포 루트에서):

    # D9: 후보 연도별 클린 랩 수 조사 (텔레메트리 없이 랩 표만 받음)
    python python/fastf1_preprocessing/fastf1_to_csv.py survey --years 2019 2020 2022 2023 2024 2025

    # I0 생성
    python python/fastf1_preprocessing/fastf1_to_csv.py run --year 2024

출력 (data/raw/, git 제외):
    fastf1_<track>.csv        I0: s, x, y, z, v, throttle, brake
    fastf1_<track>_diag.csv   진단: z_median_raw, z_iqr, v_iqr, grade
    fastf1_<track>_meta.json  세션·랩 수·시작점 오프셋·정합 잔차·고저차 검증값

스키마는 ../../docs/interfaces.md 의 I0 정의를 따른다.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from i0_pipeline import I0Params, I0Result, build_i0  # noqa: E402
from tumftm import DEFAULT_DB, load_track  # noqa: E402

REPO = Path(__file__).resolve().parents[2]


def write_i0(res: I0Result, track_key: str, out_dir: Path, extra_meta: dict | None = None) -> dict[str, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = {
        "i0": out_dir / f"fastf1_{track_key}.csv",
        "diag": out_dir / f"fastf1_{track_key}_diag.csv",
        "meta": out_dir / f"fastf1_{track_key}_meta.json",
    }
    res.i0.to_csv(paths["i0"], index=False, float_format="%.6f")
    res.diag.to_csv(paths["diag"], index=False, float_format="%.6f")
    meta = dict(res.meta)
    meta["warnings"] = res.warnings
    if extra_meta:
        meta.update(extra_meta)
    paths["meta"].write_text(json.dumps(meta, ensure_ascii=False, indent=2, default=float), encoding="utf-8")
    return paths


def print_report(res: I0Result) -> None:
    m = res.meta
    t, z, lp = m["f1_to_tumftm"], m["z"], m["laps"]
    print("── I0 검증 요약 ──")
    print(f"레이스라인 길이 {m['track_length_raceline_m']:.1f} m, 격자 {m['n_points']}점 (Δs={m['ds_m']:.4f} m)")
    print(f"랩: 입력 {lp['input']} → 정리 후 {lp['after_cleanup']}, z용 {lp['used_for_z']}, 속도용 {lp['used_for_speed']}")
    print(f"랩 적분거리 중앙값 {lp['lap_distance_median_m']:.1f} m (레이스라인 대비 {lp['lap_distance_over_raceline']:.4f})")
    print(f"시작점 오프셋 {m['start_offset_m']:.1f} m, 방향 {m['direction']:+d}")
    print(f"FastF1→TUMFTM: scale {t['scale']:.4f}, rot {t['rotation_deg']:.2f}°, reflected={t['reflected']}, "
          f"RMS {t['rms_m']:.2f} m, max {t['max_abs_residual_m']:.2f} m")
    print(f"고저차 {z['elevation_range_m']:.1f} m (평활 전 {z['elevation_range_raw_median_m']:.1f} m, "
          f"Spa 공식 {z['official_range_m_spa']} m), 경사 {z['grade_min_pct']:.1f} ~ {z['grade_max_pct']:.1f} %")
    print(f"랩 간 z IQR 중앙값 {z['lap_spread_iqr_median_m']:.2f} m, 평활 차단 파장 {z['cutoff_wavelength_m']} m")
    for w in res.warnings:
        print(f"[경고] {w}")


def cmd_run(a: argparse.Namespace) -> int:
    import fastf1_source as src

    src.enable_cache(a.cache)
    track = load_track(a.track, a.db)
    print(f"세션 로드: {a.year} {a.event} {a.session}")
    ses = src.load_session(a.year, a.event, a.session, telemetry=True)
    sel = src.select_laps(ses.laps, quick_threshold=a.quick)
    print(f"랩 선별: {sel.summary}")
    if a.max_laps and len(sel.clean) > a.max_laps:
        # 드라이버 편중 없이 줄이기: 드라이버별 라운드로빈
        order = sel.clean.assign(_r=sel.clean.groupby("Driver").cumcount()).sort_values(["_r", "Driver"])
        keep = order.index[: a.max_laps]
        sel.clean = sel.clean.loc[sel.clean.index.isin(keep)]
        sel.speed = sel.speed.loc[sel.speed.index.isin(keep)]
    traces, errors = src.extract_traces(sel)
    for e in errors:
        print(f"[스킵] {e}")
    params = I0Params(ds=a.ds, z_cutoff_wavelength=a.z_wavelength)
    res = build_i0(traces, track.raceline_xy, params)
    res.warnings.extend(f"텔레메트리 추출 실패: {e}" for e in errors)
    extra = {"source": {"year": a.year, "event": a.event, "session": a.session, "track": a.track,
                        "lap_selection": sel.summary,
                        "fastf1_version": __import__("fastf1").__version__}}
    paths = write_i0(res, a.track.lower(), a.out, extra)
    print_report(res)
    for k, p in paths.items():
        print(f"{k}: {p}")
    return 0


def cmd_survey(a: argparse.Namespace) -> int:
    import fastf1_source as src

    src.enable_cache(a.cache)
    rows = []
    for y in a.years:
        try:
            ses = src.load_session(y, a.event, a.session, telemetry=False)
            s = src.select_laps(ses.laps, quick_threshold=a.quick).summary
            rows.append((y, s, ""))
        except Exception as e:  # noqa: BLE001
            rows.append((y, None, f"{type(e).__name__}: {e}"))
    print(f"{'year':>5} {'total':>6} {'clean':>6} {'speed':>6} {'drivers':>8} {'wet':>5} {'fastest[s]':>11}")
    for y, s, err in rows:
        if s is None:
            print(f"{y:>5}  로드 실패 — {err}")
        else:
            print(f"{y:>5} {s['total']:>6} {s['clean']:>6} {s['speed_laps']:>6} {s['drivers_clean']:>8} "
                  f"{s['wet_laps_in_clean']:>5} {s['fastest_lap_s']:>11.3f}")
    print("D9 기준: clean(정상 랩) 많고 wet 0인 시즌 우선.")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--event", default="Belgian Grand Prix")
    ap.add_argument("--session", default="R", help="FastF1 세션 식별자 (D9: 레이스 'R')")
    ap.add_argument("--cache", type=Path, default=REPO / "data" / "cache" / "fastf1")
    ap.add_argument("--quick", type=float, default=1.03, help="속도용 랩: 최속 랩 대비 비율 상한")
    sub = ap.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("run", help="I0 생성")
    r.add_argument("--year", type=int, required=True, help="D9: survey 결과로 A가 선정")
    r.add_argument("--track", default="Spa", help="TUMFTM 트랙 이름")
    r.add_argument("--db", type=Path, default=DEFAULT_DB)
    r.add_argument("--out", type=Path, default=REPO / "data" / "raw")
    r.add_argument("--ds", type=float, default=1.0)
    r.add_argument("--z-wavelength", type=float, default=50.0, help="고저 평활 차단 파장 [m] (D6 민감도)")
    r.add_argument("--max-laps", type=int, default=0, help="텔레메트리 추출 랩 수 상한 (0=전부)")
    r.set_defaults(func=cmd_run)

    s = sub.add_parser("survey", help="D9 세션 후보 연도별 클린 랩 수")
    s.add_argument("--years", type=int, nargs="+", required=True)
    s.set_defaults(func=cmd_survey)

    a = ap.parse_args(argv)
    np.set_printoptions(precision=4, suppress=True)
    return a.func(a)


if __name__ == "__main__":
    raise SystemExit(main())
