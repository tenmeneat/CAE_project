# 블록 5 · 응력 중첩 → 피로손상 (담당 B, MATLAB)

인터페이스 **I2**: `[D, Dsec, sig] = lca_damage(wf)` — A가 스윕 루프(블록 6)에서 호출하는 핵심 함수.

## 절차 (설계안 4.5절)

1. 중첩: `σ(t) = Σ_k C_k * F_k(t)` (C_k: 블록 4의 단위하중 k에 대한 핫스팟 응력텐서 6성분)
2. **주의**: von Mises를 직접 더하면 안 됨 — 텐서 성분을 먼저 중첩한 뒤 등가응력(부호 있는 von Mises 또는
   최대주응력) 계산
3. 사이클 계수: MATLAB `rainflow()`
4. 평균응력 보정: Goodman `σar = σa / (1 - σm/σu)`
5. 수명: Basquin S-N `σar = σ'f * (2N)^b` → Miner `D = Σ(ni/Ni)`
6. 출력: 랩당 손상 D_lap, 코너(구간)별 손상, 핫스팟 응력 시계열

## 전제 조건

- 블록 4의 응력계수 표(C) 필요
- D4(로어암 재질) 확정 → S-N 곡선 파라미터(σ'f, b, σu) 필요

## TODO

- [ ] `superpose_stress.m` — 텐서 성분 중첩 → 등가응력
- [ ] `lca_damage.m` — I2 함수 본체 (rainflow + Goodman + Miner)
- [ ] S-N 파라미터 테이블 (재질별, D4 확정 후)
- [ ] 단순 정현파 하중 손계산 검증 케이스 (검증 체크리스트 항목)
