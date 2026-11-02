"""폐곡선 기하 유틸: 등간격 재샘플, 2D 상사변환(Umeyama), 시작점 오프셋 추정, 주기 평활.

FastF1과 무관한 순수 numpy/scipy 코드라 오프라인 테스트가 가능하다.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.interpolate import CubicSpline


# --------------------------------------------------------------------------- #
# 폐곡선 재샘플
# --------------------------------------------------------------------------- #
@dataclass
class ClosedCurve:
    """등간격(호장 기준)으로 재샘플한 폐곡선. s[0] = 0, 마지막 점 다음이 s = L(= s[0])."""

    s: np.ndarray        # (N,) 호장 [m]
    xy: np.ndarray       # (N, 2) 좌표 [m]
    length: float        # 전체 길이 L [m]

    @property
    def ds(self) -> float:
        return self.length / len(self.s)


def resample_closed(xy: np.ndarray, ds: float = 1.0, oversample: int = 20) -> ClosedCurve:
    """폐곡선 점열을 주기 큐빅 스플라인으로 보간한 뒤 호장 기준 등간격(≈ds)으로 재샘플한다.

    - 입력 점열은 닫혀 있지 않아도 된다(마지막 점 → 첫 점 구간을 자동으로 닫음).
    - 현(chord) 길이가 아니라 스플라인 호장으로 길이를 다시 잰다(oversample 배 세분화).
    - 주기 데이터를 다루기 쉽도록 N = round(L/ds), 간격 = L/N (≈ds)로 맞춘다.
    """
    xy = np.asarray(xy, dtype=float)
    if np.allclose(xy[0], xy[-1]):
        xy = xy[:-1]
    closed = np.vstack([xy, xy[:1]])
    chord = np.r_[0.0, np.cumsum(np.hypot(*np.diff(closed, axis=0).T))]
    spl = CubicSpline(chord, closed, bc_type="periodic", axis=0)

    # 스플라인 호장 재계산
    u_fine = np.linspace(0.0, chord[-1], len(xy) * oversample + 1)
    p_fine = spl(u_fine)
    arc = np.r_[0.0, np.cumsum(np.hypot(*np.diff(p_fine, axis=0).T))]
    length = float(arc[-1])

    n = max(int(round(length / ds)), 3)
    s = np.arange(n) * (length / n)
    u = np.interp(s, arc, u_fine)
    return ClosedCurve(s=s, xy=spl(u), length=length)


def curve_at(curve: ClosedCurve, s_query: np.ndarray) -> np.ndarray:
    """폐곡선 위 임의 호장 s에서의 좌표(선형 보간, 주기 처리)."""
    s_ext = np.r_[curve.s, curve.length]
    xy_ext = np.vstack([curve.xy, curve.xy[:1]])
    sq = np.mod(s_query, curve.length)
    return np.column_stack([np.interp(sq, s_ext, xy_ext[:, 0]), np.interp(sq, s_ext, xy_ext[:, 1])])


# --------------------------------------------------------------------------- #
# 2D 상사변환 (Umeyama 1991)
# --------------------------------------------------------------------------- #
@dataclass
class Similarity2D:
    scale: float
    rot: np.ndarray      # (2, 2)
    trans: np.ndarray    # (2,)
    reflected: bool
    rms: float           # 정합 잔차 RMS [m] (dst 단위)

    def apply(self, p: np.ndarray) -> np.ndarray:
        return self.scale * p @ self.rot.T + self.trans


def umeyama_2d(src: np.ndarray, dst: np.ndarray, allow_reflection: bool = False) -> Similarity2D:
    """dst ≈ c·R·src + t 를 최소제곱으로 푼다. allow_reflection=True면 거울상(det R = -1)도 허용."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    a, b = src - mu_s, dst - mu_d
    var_s = (a ** 2).sum() / len(a)
    cov = b.T @ a / len(a)
    u, d, vt = np.linalg.svd(cov)
    sgn = np.ones(2)
    if np.linalg.det(u) * np.linalg.det(vt) < 0 and not allow_reflection:
        sgn[-1] = -1.0
    rot = u @ np.diag(sgn) @ vt
    scale = float((d * sgn).sum() / var_s)
    trans = mu_d - scale * rot @ mu_s
    res = dst - (scale * src @ rot.T + trans)
    rms = float(np.sqrt((res ** 2).sum(1).mean()))
    return Similarity2D(scale, rot, trans, bool(np.linalg.det(rot) < 0), rms)


# --------------------------------------------------------------------------- #
# 시작점 오프셋 추정
# --------------------------------------------------------------------------- #
@dataclass
class StartOffset:
    """FastF1 랩 진행률 f(0..1) ↔ 레이스라인 호장 s 대응.

    s = (direction · f · L + offset) mod L
    """

    offset: float            # [m]
    direction: int           # +1: 같은 주행 방향, -1: 반대
    transform: Similarity2D  # FastF1 XY → 레이스라인 XY
    rms_curve: np.ndarray    # 후보 오프셋별 정합 RMS (진단용)

    def s_of_fraction(self, f: np.ndarray, length: float) -> np.ndarray:
        return np.mod(self.direction * np.asarray(f) * length + self.offset, length)

    def fraction_of_s(self, s: np.ndarray, length: float) -> np.ndarray:
        return np.mod(self.direction * (np.asarray(s) - self.offset), length) / length


def estimate_start_offset(
    f1_xy: np.ndarray,
    curve: ClosedCurve,
    n_match: int = 700,
    allow_reflection: bool = True,
) -> StartOffset:
    """FastF1 랩 평균 궤적(진행률 등간격 n_match점)과 레이스라인을 정합해 시작점 오프셋을 찾는다.

    설계안 4.1: 두 데이터의 XY 좌표계가 다르므로 XY를 직접 맞추지 않고 출발선 기준 거리 s로 정렬한다.
    다만 TUMFTM 파일의 첫 점이 F1 타이밍 라인이라는 보장이 없어서, '랩 진행률 → s' 사상의 상수 오프셋
    하나만 상사변환 정합 잔차가 최소가 되도록 고른다. 이후 모든 샘플은 진행률로만 s에 대응된다.

    f1_xy: (n_match, 2) — 진행률 f = k/n_match 에서의 FastF1 XY [m]
    """
    f1_xy = np.asarray(f1_xy, dtype=float)
    m = len(f1_xy)
    if m != n_match:
        raise ValueError("f1_xy 길이와 n_match가 다름")
    L = curve.length
    s_base = np.arange(m) * (L / m)

    best = None
    rms_all = {}
    for direction in (+1, -1):
        rms = np.empty(m)
        fits = []
        for j in range(m):
            shift = j * L / m
            dst = curve_at(curve, direction * s_base + shift)
            fit = umeyama_2d(f1_xy, dst, allow_reflection=allow_reflection)
            rms[j] = fit.rms
            fits.append(fit)
        rms_all[direction] = rms
        j0 = int(np.argmin(rms))
        # 포물선 보간으로 서브샘플 오프셋
        r_m, r_0, r_p = rms[(j0 - 1) % m], rms[j0], rms[(j0 + 1) % m]
        denom = r_m - 2 * r_0 + r_p
        frac = 0.5 * (r_m - r_p) / denom if denom > 0 else 0.0
        offset = (j0 + frac) * L / m
        if best is None or r_0 < best[0]:
            best = (r_0, offset, direction, j0)

    _, offset, direction, j0 = best
    dst = curve_at(curve, direction * s_base + offset)
    fit = umeyama_2d(f1_xy, dst, allow_reflection=allow_reflection)
    return StartOffset(offset=float(np.mod(offset, L)), direction=direction, transform=fit,
                       rms_curve=rms_all[direction])


# --------------------------------------------------------------------------- #
# 주기 평활 (평활 스플라인 등가 필터)
# --------------------------------------------------------------------------- #
def smooth_periodic(y: np.ndarray, ds: float, cutoff_wavelength: float) -> np.ndarray:
    """등간격 주기 신호를 3차 평활 스플라인과 같은 전달함수로 평활한다.

    2계 미분 페널티 평활 스플라인(λ∫y''²)의 연속 근사 전달함수는 H(ω) = 1/(1 + λω⁴).
    λ = (λc/2π)⁴ 로 두면 파장 λc 에서 H = 0.5 (반파워 아님, 진폭 1/2).
    → 평활 강도를 '차단 파장 [m]' 하나로 표현한다 (D6: 평활 강도는 민감도 파라미터).
    폐곡선이라 FFT로 주기 경계를 그대로 처리한다.
    """
    y = np.asarray(y, dtype=float)
    if cutoff_wavelength <= 0:
        return y.copy()
    n = len(y)
    k = np.fft.rfftfreq(n, d=ds) * 2 * np.pi          # 각파수 [rad/m]
    wc = 2 * np.pi / cutoff_wavelength
    h = 1.0 / (1.0 + (k / wc) ** 4)
    return np.fft.irfft(np.fft.rfft(y) * h, n=n)


def periodic_gradient(y: np.ndarray, ds: float) -> np.ndarray:
    """주기 경계 중앙차분."""
    return (np.roll(y, -1) - np.roll(y, 1)) / (2 * ds)
