# FastF1 전처리 (담당 A)

역할: FastF1에서 위치(X,Y,Z)·차량 스트림을 받아와 TUMFTM 레이스라인과 같은 거리축 `s`로 정렬한 뒤
인터페이스 **I0** (`fastf1_<track>.csv`)로 내보낸다. 이후 파이프라인은 전부 MATLAB/Simulink — 파이썬은
여기서 끝.

## 설치

```
pip install fastf1 numpy pandas scipy
```

## 해야 할 것 (TODO, 구현 안 됨)

1. `fastf1.Cache.enable_cache(...)` 설정
2. 세션 로드 (D9 확정 필요: 예선 최속랩 vs 레이스 여러 랩 평균)
3. 위치 스트림(X,Y,Z, ~4.5Hz)과 차량 스트림(Speed/Throttle/Brake/Gear/RPM/DRS, ~4.2Hz) 보간 병합
4. TUMFTM 레이스라인과 출발선 기준 거리 `s`로 정렬 (XY 좌표계가 다르므로 XY 정합 대신 s 정렬 — 설계안 4.1절)
5. 여러 랩 평균 + 평활 스플라인으로 고저 z(s) 추출 (노면 요철은 안 잡힘 — ISO 8608로 블록 1에서 별도 중첩)
6. `fastf1_<track>.csv` 출력: `s, x, y, z, v, throttle, brake`

## 참고

- FastF1 문서: https://docs.fastf1.dev
- 여러 랩 겹치면 약 ±10m 오차 있음 (문서 명시) — 평균/평활로 흡수
- 가속도·서스펜션 채널은 없음 — 4Hz 속도 미분은 쓰지 말 것 (노이즈 큼)
