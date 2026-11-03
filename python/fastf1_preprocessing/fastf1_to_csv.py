"""FastF1 -> I0 (fastf1_<track>.csv) 전처리.

스키마는 ../../docs/02-인터페이스.md 의 I0 정의를 따른다.
실행: python python/fastf1_preprocessing/fastf1_to_csv.py
"""

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.interpolate import splev, splrep
from scipy.ndimage import gaussian_filter1d

REPO_ROOT = Path(__file__).resolve().parents[2]
RACELINE_DIR = REPO_ROOT / "data" / "tracks" / "tumftm" / "racelines"  # 레포에 포함 (Spa)
CACHE_DIR = Path.home() / ".cache" / "fastf1"  # OneDrive 밖에 둔다 (세션당 수백 MB)
CHANNELS = ["x", "y", "z", "v", "throttle", "brake"]


def load_session(year: int, grand_prix: str, session: str):
    """FastF1 세션 로드. session 예: 'R' (레이스). D9 기준 세션은 docs/03-결정사항.md 참고."""
    import fastf1  # 여기서만 필요 — 오프라인 테스트는 fastf1 없이 돈다

    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    fastf1.Cache.configure(cache_dir=str(CACHE_DIR))
    # FastF1 3.9는 F1 계정 입력을 기다린다. 2022 Spa는 익명으로 받아지므로 프롬프트를 건너뛴다.
    sys.stdin = open(os.devnull)
    ses = fastf1.get_session(year, grand_prix, session)
    ses.load(laps=True, telemetry=True, weather=False, messages=False)
    return ses


def select_clean_laps(laps):
    """D9: 세이프티카·황색기·피트 입출구·1랩 제외, 107% 이내 정상 랩만."""
    return laps[
        (laps["TrackStatus"] == "1")
        & laps["PitInTime"].isna()
        & laps["PitOutTime"].isna()
        & (laps["LapNumber"] > 1)
        & laps["IsAccurate"]
    ].pick_quicklaps()


def extract_position_and_car_streams(session, lap_selector=None):
    """정상 랩마다 위치·차량 스트림을 시간축으로 보간 병합해 SI 단위 DataFrame 리스트로 반환."""
    laps = (lap_selector or select_clean_laps)(session.laps)
    streams = []
    for _, lap in laps.iterlaps():
        pos = lap.get_pos_data(pad=1, pad_side="both")
        car = lap.get_car_data(pad=1, pad_side="both")
        tel = pos.merge_channels(car).slice_by_lap(lap, interpolate_edges=True).add_distance()
        streams.append(pd.DataFrame({
            "distance": tel["Distance"].to_numpy(),
            "x": tel["X"].to_numpy() / 10.0,  # 1/10 m -> m
            "y": tel["Y"].to_numpy() / 10.0,
            "z": tel["Z"].to_numpy() / 10.0,
            "v": tel["Speed"].to_numpy() / 3.6,  # km/h -> m/s
            "throttle": np.clip(tel["Throttle"].to_numpy(), 0, 100) / 100.0,
            "brake": tel["Brake"].to_numpy().astype(float),
        }))
    return streams


def closed_arc(raceline_xy):
    """닫힌 레이스라인 좌표와 누적 거리 [m]."""
    closed = np.vstack([raceline_xy, raceline_xy[:1]])
    return closed, np.r_[0.0, np.cumsum(np.hypot(*np.diff(closed, axis=0).T))]


def align_to_arc_length(df, raceline_xy):
    """속도 적분 거리를 랩 길이로 정규화해 TUMFTM 레이스라인의 거리 s로 변환."""
    return df.assign(s=df["distance"] / df["distance"].iloc[-1] * closed_arc(raceline_xy)[1][-1])


def curvature(x, y, sigma=15):
    """격자 간격 기준 곡률. 위치 노이즈를 막으려고 sigma칸 주기 가우시안으로 먼저 평활한다."""
    dx = np.gradient(gaussian_filter1d(x, sigma, mode="wrap"))
    dy = np.gradient(gaussian_filter1d(y, sigma, mode="wrap"))
    return (dx * np.gradient(dy) - dy * np.gradient(dx)) / np.hypot(dx, dy) ** 3


def align_start_line(df, raceline_xy):
    """타이밍 라인 원점을 TUMFTM 시작점으로 순환 이동. 이동량(칸)도 반환.

    XY 좌표계가 달라도 곡률은 같으므로 곡률 순환 상호상관으로 위상차를 구한다 (Spa 약 16 m).
    """
    n = len(df)
    closed, arc = closed_arc(raceline_xy)
    grid = np.linspace(0.0, arc[-1], n, endpoint=False)
    k_ref = curvature(np.interp(grid, arc, closed[:, 0]), np.interp(grid, arc, closed[:, 1]))
    k = curvature(df["x"].to_numpy(), df["y"].to_numpy())
    shift = int(np.argmax(np.fft.irfft(np.fft.rfft(k_ref) * np.conj(np.fft.rfft(k)), n)))
    shifted = df.copy()
    shifted[CHANNELS] = np.roll(df[CHANNELS].to_numpy(), shift, axis=0)
    return shifted, shift if shift <= n / 2 else shift - n


def average_and_smooth_laps(aligned_dfs, z_rms=0.1, ds=1.0):
    """여러 랩을 s 격자에서 평균하고 z만 주기 평활 스플라인 적용.

    z_rms [m]: 평활 전후 허용 RMS 차이 (D6 민감도 파라미터). 클수록 더 매끈하다.
    """
    length = aligned_dfs[0]["s"].iloc[-1]
    # 둘레를 정확히 나눈 주기 격자 (간격 약 ds). 출발선 순환 이동 때 이음매에 κv 스파이크가 안 생긴다.
    s = np.linspace(0.0, length, round(length / ds), endpoint=False)
    df = pd.DataFrame({"s": s, **{
        c: np.mean([np.interp(s, d["s"], d[c]) for d in aligned_dfs], axis=0) for c in CHANNELS
    }})

    s_closed = np.append(s, length)
    z_closed = np.append(df["z"], df["z"].iloc[0])
    tck = splrep(s_closed, z_closed, per=True, s=len(s_closed) * z_rms**2)
    df["z"] = splev(s, tck)
    return df


def write_i0_csv(df, track_name: str, out_dir: Path = REPO_ROOT / "data" / "raw"):
    """I0 스키마로 CSV 저장: s[m], x[m], y[m], z[m], v[m/s], throttle[0-1], brake[0-1]."""
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"fastf1_{track_name.lower()}.csv"
    # 6자리: 1 m 격자에서 z를 두 번 미분(κv)해도 반올림 노이즈가 묻히지 않게
    df[["s", *CHANNELS]].to_csv(path, index=False, float_format="%.6f")
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--year", type=int, default=2022)
    ap.add_argument("--gp", default="Belgian Grand Prix")
    ap.add_argument("--session", default="R")
    ap.add_argument("--track", default="Spa", help="TUMFTM racelines/<track>.csv 이름")
    ap.add_argument("--z-rms", type=float, default=0.1, help="z 평활 강도 [m] (D6)")
    args = ap.parse_args()

    raceline_xy = np.loadtxt(RACELINE_DIR / f"{args.track}.csv", delimiter=",")
    streams = extract_position_and_car_streams(load_session(args.year, args.gp, args.session))
    if not streams:
        raise SystemExit("정상 랩이 없음 — 세션 데이터를 확인할 것")
    df = average_and_smooth_laps([align_to_arc_length(d, raceline_xy) for d in streams], args.z_rms)
    df, shift = align_start_line(df, raceline_xy)
    path = write_i0_csv(df, args.track)

    slope = np.gradient(df["z"], df["s"])
    print(f"{path}: {len(df)}행, 랩 {len(streams)}개 평균, 출발선 보정 {shift} m")
    print(f"L={df['s'].iloc[-1]:.1f} m, 고저차={np.ptp(df['z']):.1f} m, 최대경사={slope.max() * 100:.1f}%, "
          f"평균랩타임={np.sum(1.0 / df['v']):.2f} s, vmax={df['v'].max() * 3.6:.1f} km/h")


if __name__ == "__main__":
    main()
