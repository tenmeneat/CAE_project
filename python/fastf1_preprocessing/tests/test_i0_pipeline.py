"""I0 파이프라인 오프라인 검증 (FastF1 네트워크 불필요).

실제 TUMFTM Spa 레이스라인 위에 '정답을 아는' 합성 랩을 만들어 파이프라인이 그 정답을 복원하는지 본다.
합성 랩은 FastF1과 같은 성질을 흉내낸다: 다른 XY 좌표계(회전·평행이동), 다른 시작점,
~4.5 Hz 샘플, 랩마다 속도 적분거리 오차, 위치 노이즈.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from geometry import (  # noqa: E402
    curve_at,
    estimate_start_offset,
    resample_closed,
    smooth_periodic,
    umeyama_2d,
)
from i0_pipeline import I0_COLUMNS, I0Params, LapTrace, build_i0  # noqa: E402
from tumftm import DEFAULT_DB, load_track  # noqa: E402

needs_db = pytest.mark.skipif(not (DEFAULT_DB / "racelines" / "Spa.csv").exists(),
                              reason="TUMFTM racetrack-database 미클론")


# --------------------------------------------------------------------------- #
# 단위 테스트
# --------------------------------------------------------------------------- #
def test_resample_circle_length_and_spacing():
    r = 100.0
    th = np.linspace(0, 2 * np.pi, 73)[:-1]
    c = resample_closed(np.column_stack([r * np.cos(th), r * np.sin(th)]), ds=1.0)
    assert c.length == pytest.approx(2 * np.pi * r, rel=1e-4)
    seg = np.hypot(*np.diff(np.vstack([c.xy, c.xy[:1]]), axis=0).T)
    assert seg.max() - seg.min() < 1e-3
    assert np.allclose(np.hypot(*c.xy.T), r, atol=1e-3)


def test_umeyama_recovers_similarity():
    rng = np.random.default_rng(0)
    src = rng.normal(size=(200, 2)) * 100
    th = np.radians(-123.0)
    rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    dst = 1.02 * src @ rot.T + np.array([500.0, -40.0])
    fit = umeyama_2d(src, dst)
    assert fit.scale == pytest.approx(1.02, rel=1e-9)
    assert np.allclose(fit.rot, rot)
    assert fit.rms < 1e-9 and not fit.reflected

    mirrored = dst * np.array([1.0, -1.0])
    assert umeyama_2d(src, mirrored, allow_reflection=True).reflected
    assert umeyama_2d(src, mirrored, allow_reflection=True).rms < 1e-9


def test_smooth_periodic_transfer():
    ds, n = 1.0, 7000
    s = np.arange(n) * ds
    lam_c = 50.0
    for lam, gain in [(lam_c, 0.5), (500.0, 1 / (1 + (lam_c / 500.0) ** 4)), (10.0, 1 / (1 + 5.0 ** 4))]:
        k = round(n / lam)
        y = np.sin(2 * np.pi * k * s / n)
        out = smooth_periodic(y, ds, lam_c)
        lam_eff = n / k
        expected = 1 / (1 + (lam_c / lam_eff) ** 4)
        assert np.allclose(out, expected * y, atol=1e-9)
        assert expected == pytest.approx(gain, rel=0.05)


# --------------------------------------------------------------------------- #
# 합성 FastF1 랩
# --------------------------------------------------------------------------- #
def z_true(s, L):
    """정답 고저 [m]: 장파장 성분만 (평활 영향 최소)."""
    u = 2 * np.pi * s / L
    return 40 * np.sin(u) + 15 * np.cos(2 * u + 0.4) + 6 * np.sin(5 * u + 1.1) + 300.0


def v_true(s, L):
    u = 2 * np.pi * s / L
    return 65 + 15 * np.sin(3 * u) + 5 * np.cos(7 * u)   # m/s


def make_laps(curve, n_laps, offset, rot_deg, trans, rng, mirror=False, xy_noise=0.5, z_noise=0.3):
    L = curve.length
    th = np.radians(rot_deg)
    rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    laps = []
    for i in range(n_laps):
        # FastF1 진행률 f ↔ 레이스라인 s = f·L + offset. 샘플은 시간 등간격(4.5 Hz) → s 비균일
        s_fine = np.linspace(0, L, 20001)
        dt = np.diff(s_fine) / v_true(offset + s_fine[:-1], L)
        t = np.r_[0, np.cumsum(dt)]
        ts = np.arange(rng.uniform(0, 0.22), t[-1], 1 / 4.5)
        ts = np.r_[0.0, ts, t[-1]]                       # FastF1 interpolate_edges 모사
        sf = np.interp(ts, t, s_fine)
        s_rl = np.mod(sf + offset, L)
        xy = curve_at(curve, s_rl)
        lat = rng.normal(0, 1.0)                          # 랩마다 라인 횡오프셋 비슷한 효과
        xy_f1 = (xy - trans) @ rot + rng.normal(0, xy_noise, xy.shape) + lat
        if mirror:
            xy_f1[:, 1] *= -1
        scale_err = 1 + rng.normal(0, 0.004)              # 속도 적분거리 오차
        laps.append(LapTrace(
            data=pd.DataFrame({
                "distance": sf * scale_err,
                "x": xy_f1[:, 0],
                "y": xy_f1[:, 1],
                "z": z_true(s_rl, L) + rng.normal(0, z_noise, len(sf)),
                "v": v_true(s_rl, L) + rng.normal(0, 1.0, len(sf)),
                "throttle": np.clip(rng.uniform(0.9, 1.0, len(sf)), 0, 1),
                "brake": np.zeros(len(sf)),
            }),
            driver=f"D{i % 20:02d}", lap_number=i + 2, use_for_speed=(i % 2 == 0),
        ))
    return laps


@needs_db
@pytest.mark.parametrize("mirror", [False, True])
def test_build_i0_recovers_truth(mirror):
    track = load_track("Spa")
    curve = resample_closed(track.raceline_xy, ds=1.0)
    L = curve.length
    rng = np.random.default_rng(42)
    offset = 1234.5
    laps = make_laps(curve, 40, offset, rot_deg=37.0, trans=np.array([250.0, -800.0]), rng=rng, mirror=mirror)

    res = build_i0(laps, track.raceline_xy, I0Params(ds=1.0, z_cutoff_wavelength=50.0))

    assert list(res.i0.columns) == I0_COLUMNS
    assert len(res.i0) == res.meta["n_points"]
    err_off = (res.meta["start_offset_m"] - offset + L / 2) % L - L / 2
    assert abs(err_off) < 5.0, err_off
    assert res.meta["direction"] == +1
    assert res.meta["f1_to_tumftm"]["reflected"] == mirror
    assert res.meta["f1_to_tumftm"]["scale"] == pytest.approx(1.0, abs=0.01)

    s = res.i0["s"].to_numpy()
    z_err = res.i0["z"].to_numpy() - z_true(s, L)
    v_err = res.i0["v"].to_numpy() - v_true(s, L)
    assert np.sqrt(np.mean(z_err ** 2)) < 0.3
    assert np.sqrt(np.mean(v_err ** 2)) < 1.0
    z_t = z_true(s, L)
    assert res.meta["z"]["elevation_range_m"] == pytest.approx(z_t.max() - z_t.min(), abs=0.5)
    assert res.meta["laps"]["used_for_speed"] == 20


@needs_db
def test_build_i0_rejects_bad_laps():
    track = load_track("Spa")
    curve = resample_closed(track.raceline_xy, ds=1.0)
    rng = np.random.default_rng(7)
    laps = make_laps(curve, 12, 300.0, rot_deg=-80.0, trans=np.array([0.0, 0.0]), rng=rng)
    laps[0].data["distance"] *= 1.10                     # 적분거리 10% 이탈
    laps[1].data["z"] += 25.0                            # z 이탈
    res = build_i0(laps, track.raceline_xy)
    assert res.meta["laps"]["after_cleanup"] == 11
    assert res.meta["laps"]["used_for_z"] == 10
    assert any("적분거리" in w for w in res.warnings)
    assert any("z 이탈" in w for w in res.warnings)


@needs_db
def test_start_offset_direction_reversed():
    track = load_track("Spa")
    curve = resample_closed(track.raceline_xy, ds=1.0)
    L, m = curve.length, 700
    f = np.arange(m) / m
    xy = curve_at(curve, np.mod(-f * L + 2000.0, L))
    off = estimate_start_offset(xy, curve, n_match=m)
    assert off.direction == -1
    assert abs((off.offset - 2000.0 + L / 2) % L - L / 2) < 2.0
