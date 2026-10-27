"""FastF1 -> I0 (fastf1_<track>.csv) 전처리 스켈레톤.

구현 안 됨 — 함수 시그니처와 TODO만 정의. 담당 A가 채울 것.
스키마는 ../../docs/interfaces.md 의 I0 정의를 따른다.
"""

from pathlib import Path


def load_session(year: int, grand_prix: str, session: str):
    """FastF1 세션 로드. session 예: 'Q' (예선) 또는 'R' (레이스).

    D9 (docs/decisions.md): 예선 최속랩 vs 레이스 여러 랩 중 어느 걸 쓸지 아직 미정.
    """
    raise NotImplementedError


def extract_position_and_car_streams(session, lap_selector=None):
    """위치 스트림(X,Y,Z)과 차량 스트림(Speed,Throttle,Brake,Gear,RPM,DRS)을 추출하고
    시간축을 보간 병합한다."""
    raise NotImplementedError


def align_to_arc_length(df, raceline_xy):
    """TUMFTM 레이스라인 기준 출발선 거리 s로 정렬 (XY 좌표계 직접 정합 대신)."""
    raise NotImplementedError


def average_and_smooth_laps(aligned_dfs):
    """여러 랩을 s 기준으로 평균 + 평활 스플라인 (고저 z(s) 추출용)."""
    raise NotImplementedError


def write_i0_csv(df, track_name: str, out_dir: Path = Path("data/raw")):
    """I0 스키마로 CSV 저장: s, x, y, z, v, throttle, brake."""
    raise NotImplementedError


if __name__ == "__main__":
    raise SystemExit("스켈레톤 상태 — CLI 인자 파싱 및 실행 로직 미구현")
