"""합성 랩으로 s 정렬·평균·평활 검증 (FastF1 네트워크 불필요). 실행: python test_fastf1_to_csv.py"""

import numpy as np
import pandas as pd

from fastf1_to_csv import (
    CHANNELS,
    align_start_line,
    align_to_arc_length,
    average_and_smooth_laps,
    closed_arc,
)


def lap(distance_scale, z_offset=0.0):
    """둘레 1000 m 원, z는 20 m 진폭 사인파. distance_scale은 속도 적분 오차를 흉내낸다."""
    frac = np.linspace(0, 1, 250)
    data = {c: np.full_like(frac, 0.5) for c in CHANNELS}
    data["z"] = 20 * np.sin(2 * np.pi * frac) + z_offset
    return pd.DataFrame({"distance": frac * 1000 * distance_scale, **data})


def main():
    theta = np.linspace(0, 2 * np.pi, 2000, endpoint=False)
    raceline = 1000 / (2 * np.pi) * np.column_stack([np.cos(theta), np.sin(theta)])

    # 랩마다 다른 적분 거리가 같은 s축으로 정규화되는지
    aligned = [align_to_arc_length(lap(k), raceline) for k in (0.98, 1.0, 1.02)]
    assert all(abs(d["s"].iloc[-1] - 1000) < 0.01 for d in aligned)

    # 오프셋 랩이 평균에서 상쇄되고 주기 스플라인이 형상을 유지하는지
    df = average_and_smooth_laps([align_to_arc_length(lap(1, o), raceline) for o in (-1, 1)])
    truth = 20 * np.sin(2 * np.pi * df["s"] / 1000)
    assert np.abs(df["z"] - truth).max() < 0.5
    assert abs(df["z"].iloc[0] - df["z"].iloc[-1]) < 0.5  # 시작=끝 닫힘

    # 비대칭 트랙을 회전·이동한 좌표계에서도 알려진 출발선 위상차를 복원하는지
    r = 300 + 60 * np.cos(theta) + 30 * np.cos(3 * theta + 1)
    ref = np.column_stack([r * np.cos(theta), r * np.sin(theta)])
    closed, arc = closed_arc(ref)
    n = int(arc[-1])
    grid = np.linspace(0, arc[-1], n, endpoint=False)
    xy = np.column_stack([np.interp(grid, arc, closed[:, 0]), np.interp(grid, arc, closed[:, 1])])
    measured = pd.DataFrame(0.0, index=range(n), columns=CHANNELS)
    measured[["x", "y"]] = np.roll(xy @ [[0, 1], [-1, 0]] + 500, -17, axis=0)  # 17 m 앞선 원점
    aligned_df, shift = align_start_line(measured, ref)
    assert shift == 17, shift
    assert np.abs(aligned_df[["x", "y"]].to_numpy() - (xy @ [[0, 1], [-1, 0]] + 500)).max() < 1e-9
    print("ok")


if __name__ == "__main__":
    main()
