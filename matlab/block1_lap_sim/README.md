# 블록 1 · 랩 시뮬레이션 (담당 A, MATLAB)

준정상(Quasi-Steady-State) 랩 시뮬. 서킷 곡률·고저 → `v(t), ax(t), ay(t)`.

입력: TUMFTM 레이스라인 CSV + 인터페이스 I0 (`fastf1_<track>.csv`, 고저/검증용)
출력: 블록 2(Simulink)로 넘기는 시간축 하중 입력

## 처리 순서 (설계안 4.1절)

1. 레이스라인 Δs=1m 재샘플 → 수평 곡률 κ(s) (중심선이 아니라 레이스라인 곡률 — 속도가 맞으려면 필수)
2. FastF1 Z를 s 기준 정렬·평균·평활 → 고저 z(s), 경사 θ(s), 수직 곡률 κv(s)
3. 코너 한계속도 `v_lim(s) = sqrt(mu*g/|κ(s)|)`
4. 전진 적분(파워·트랙션 한계) + 후진 적분(제동 한계), 마찰원(ax ≤ ax,max·sqrt(1-(ay/ay,max)^2)) 결합
5. s→t 변환 후 v(t), ax(t), ay(t) 출력. 고저 위에 ISO 8608 랜덤 노면(B-C급) 중첩 (R6)

## 검증 (빠뜨리면 안 됨)

- 방법론 검증: F1급 파라미터로 돌린 v(s)를 FastF1 실측과 비교
- 차급 검증: GT 파라미터 랩타임이 실측 ±10% 이내인지

## TODO

- [ ] `load_raceline.m` — TUMFTM CSV 로더 + Δs=1m 재샘플
- [ ] `compute_curvature.m` — κ(s), 수직 곡률 κv(s)
- [ ] `quasi_steady_speed_profile.m` — 전진/후진 적분 + 마찰원 결합
- [ ] `apply_iso8608_roughness.m` — 노면 요철 중첩
- [ ] `validate_against_fastf1.m` — 검증 1·2
