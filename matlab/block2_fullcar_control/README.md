# 블록 2 · 풀카 7자유도 + 액티브 제어 (담당 A, Simulink)

구현 방식: MATLAB에서 `A, B` 행렬 생성 → Simulink **State-Space 블록**으로 구현 (대수루프 원천 차단, cae5가 겪은
문제 회피). 같은 `A, B`를 `lqr()`에 그대로 사용.

- 상태: 7DOF → 14개(변위+속도), 액추에이터 1차 지연 포함 시 +4
- 입력: 액추에이터 힘 4개(FL,FR,RL,RR) + 노면 변위 4개 (후륜은 τ=L/v 지연)
- 수평 하중 (함정 1 대응): pitch 모멘트 `M_θ = m_s*ax*h_cg`, roll 모멘트 `M_φ = m_s*ay*h_cg`를 차체에 인가.
  타이어 Fx,i / Fy,i는 Fz,i 비례 분배. anti-dive·롤센터 효과는 무시(가정으로 명시).
- 제어기 4종: Passive / Skyhook(기준선) / LQR(승차감·접지 가중치) / 위치 스케줄 LQR (→ 블록 6)
- 현실성: 액추에이터 힘 포화 + 1차 지연(대역폭) 반영
- 출력 → **I1**: 코너별 휠센터력 (Fx,Fy,Fz)_i, 액추에이터 힘, 차체 가속도, 서스펜션 스트로크

## TODO

- [ ] `build_state_space.m` — A, B 행렬 구성 (D1 서스펜션 형식 확정 필요)
- [ ] `fullcar_7dof.slx` — Simulink 모델 (State-Space 블록 + 액추에이터 포화/지연)
- [ ] `controller_passive.m` / `controller_skyhook.m` / `controller_lqr.m`
- [ ] `export_i1.m` — I1 `.mat` 쓰기 (docs/interfaces.md 스키마)

## 의존성

D1(서스펜션 형식)이 확정돼야 A,B 행렬의 하드포인트 기반 기구학이 고정됨 — `docs/decisions.md` 참고.
