# 블록 4b · 액추에이터 전기-열-구조 연성 해석 (담당 B, COMSOL — v0.5 신규, 필수)

**과제 가점 요건(멀티피직스). 자기장 해석은 불가(D7: AC/DC 모듈 없음)이므로 힘 상수 Kf는 문헌·데이터시트
값을 입력으로 쓰고, 발표에서 그렇게 밝힌다.**

## 목적

액추에이터 RMS 힘이 만드는 코일 발열이 허용 온도를 넘지 않는지 판정해 **열적 연속 힘 한계 Fth**를 구한다.
블록 6(A)이 `F_rms ≤ Fth`를 제약으로 판정하는 입력값.

## 물리 (COMSOL 프리셋 "Joule Heating and Thermal Expansion")

Electric Currents(코일 전류) + Heat Transfer in Solids(코일·하우징·마운트) + Solid Mechanics(열팽창).
물리 3개가 COMSOL 안에서 결합되는 형태라 시스템 간 연계보다 엄격한 정의로 멀티피직스 요건을 충족한다.

## 형상

2D 축대칭 원통형 리니어 액추에이터 단순 모델 (코일, 하우징, 로어암 마운트 접합부). 외형 디테일은 불필요.

## 입력 (D15)

힘 상수 `Kf` [N/A], 코일 저항 `Rcoil`, 하우징 재료 물성, 외부 대류계수 `h` (문헌 범위로 민감도), 주변 온도,
절연 등급 한계 `Tlim` (예: F급 155°C).

## 방법

1. 전류 `I = F/Kf`, 줄열 `P = I²*Rcoil`
2. 정상상태 해석으로 열저항 `Rth`(코일→주변)을 구함 — **COMSOL 정적 해석 1회로 충분, 선형이라 후보마다
   다시 돌릴 필요 없음**
3. MATLAB에서 `Tcoil = Tamb + Rth*Rcoil*(Frms/Kf)^2`
4. `Fth = Kf * sqrt((Tlim - Tamb) / (Rth*Rcoil))`

`Rcoil`은 최대 허용 온도에서의 값으로 고정해 선형을 유지(보수적 가정).

## 가정

정상상태 가정은 보수적. 필요하면 Time Dependent로 랩 전력 프로파일 1회 확인(옵션).

## 출력 (I5)

`Rth, Fth`, 코일 최고 온도 분포, 하우징·마운트 열응력 최대값, 로어암 마운트 접합부 온도 상승(로어암 응력에
열 기여 무시 가능한지 확인). → `act_thermal_<act>.mat`로 블록 5/블록 6에 전달.

## 검증

- `Rth`를 1D 열저항 손계산(코일→하우징 전도 + 외표면 대류)과 비교
- `Kf` ±20%, `h` 문헌 범위로 민감도 확인
- 추천 후보 1건을 COMSOL에서 직접 재해석해 MATLAB의 `Fth` 기반 계산과 일치하는지 확인

## 한계

AC/DC 모듈이 없어 자기장 해석 불가 → `Kf`는 문헌·데이터시트 값. 3축의 소비전력은 기계 일률(F·v)에 줄열(I²R)을
더해 계산.

## TODO

- [ ] COMSOL "Joule Heating and Thermal Expansion" 프리셋으로 2D 축대칭 모델 구성
- [ ] 정상상태 해석 → Rth 추출
- [ ] 1D 열저항 손계산 대조 검증
- [ ] Kf, h 민감도 스윕
- [ ] I5(`act_thermal_<act>.mat`) 내보내기 — MATLAB 쪽 `Fth` 계산은 `matlab/block4b_actuator_thermal/compute_fth.m`
