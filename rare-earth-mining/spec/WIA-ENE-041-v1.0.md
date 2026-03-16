# WIA-ENE-041: 희토류 채굴 표준 💎

> **Version:** 1.0.0
> **발행일:** 2025-12-25
> **상태:** Active
> **弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

---

## 목차

1. [개요](#1-개요)
2. [표준의 목적](#2-표준의-목적)
3. [적용 범위](#3-적용-범위)
4. [용어 정의](#4-용어-정의)
5. [희토류 원소 분류](#5-희토류-원소-분류)
6. [광상 유형](#6-광상-유형)
7. [채굴 방법](#7-채굴-방법)
8. [분리 및 정제 공정](#8-분리-및-정제-공정)
9. [방사성 폐기물 관리](#9-방사성-폐기물-관리)
10. [공급망 보안](#10-공급망-보안)
11. [데이터 형식 표준](#11-데이터-형식-표준)
12. [API 인터페이스](#12-api-인터페이스)
13. [추적 및 보고](#13-추적-및-보고)
14. [인증 및 준수](#14-인증-및-준수)
15. [부록](#15-부록)

---

## 1. 개요

### 1.1 배경

희토류 원소(Rare Earth Elements, REE)는 현대 첨단 기술의 핵심 재료로, 전 세계적으로 **연간 약 30만 톤**이 채굴되고 있으며, 이 중 **중국이 60%**를 생산합니다.

희토류는 다음 분야에 필수적입니다:
- **영구자석**: 네오디뮴(Nd), 디스프로슘(Dy) - 전기차, 풍력 터빈
- **촉매**: 란타넘(La), 세륨(Ce) - 석유 정제, 배기가스 처리
- **형광체**: 유로퓸(Eu), 테르븀(Tb) - LED, 디스플레이
- **배터리**: 란타넘(La) - 니켈-수소 배터리
- **광학**: 이트륨(Y) - 레이저, 광섬유

### 1.2 문제점

현재 희토류 채굴의 주요 문제점:

1. **환경 오염**: 채굴 및 정제 과정에서 심각한 토양, 수질 오염
2. **방사성 폐기물**: 토륨(Th), 우라늄(U) 함유로 인한 방사성 폐기물 발생
3. **공급망 집중**: 중국 의존도 과도 (60% 생산, 85% 정제)
4. **전략적 중요성**: 군사, 첨단 기술 분야의 필수 자원
5. **가격 변동성**: 지정학적 요인으로 인한 극심한 가격 변동
6. **추적 부재**: 원산지 및 공급망 투명성 부족

### 1.3 WIA-ENE-041의 가치

이 표준은 다음을 제공합니다:

- ✅ **지속가능한 채굴**: 환경 영향 최소화 및 복원
- ✅ **안전한 폐기물 관리**: 방사성 폐기물의 체계적 관리
- ✅ **공급망 투명성**: 블록체인 기반 원산지 추적
- ✅ **재활용 촉진**: 도시광산(e-waste)에서 희토류 회수
- ✅ **글로벌 협력**: 국제 표준과의 호환

---

## 2. 표준의 목적

### 2.1 핵심 목표

1. **환경 보호**: 채굴로 인한 환경 파괴 최소화 및 복원
2. **안전 관리**: 방사성 물질의 안전한 처리 및 보관
3. **공급망 보안**: 전략 자원의 안정적 공급 확보
4. **투명성 확보**: 채굴부터 최종 제품까지 완전한 추적성
5. **순환경제**: 재활용을 통한 2차 자원 확보

### 2.2 기대 효과

- **환경**: 채굴 지역 환경 복원율 90% 이상 달성
- **경제**: 글로벌 희토류 시장 $150억 (2030년 예상)
- **자원**: 재활용을 통한 공급량 30% 증대
- **안보**: 공급망 다변화로 전략 자원 안정성 확보

---

## 3. 적용 범위

### 3.1 대상 원소

이 표준은 17개 희토류 원소에 적용됩니다:

#### 경희토류 (Light REE, LREE)
| 원소 | 기호 | 원자번호 | 주요 용도 |
|------|------|----------|-----------|
| 란타넘 | La | 57 | 촉매, 배터리, 광학 유리 |
| 세륨 | Ce | 58 | 촉매, 연마재, 합금 |
| 프라세오디뮴 | Pr | 59 | 자석, 합금, 착색제 |
| 네오디뮴 | Nd | 60 | 영구자석 (NdFeB), 레이저 |
| 프로메튬 | Pm | 61 | 방사성 동위원소 (인공) |
| 사마륨 | Sm | 62 | 자석, 촉매, 핵반응 제어 |
| 유로퓸 | Eu | 63 | 형광체 (LED, TV), 위조 방지 |

#### 중희토류 (Heavy REE, HREE)
| 원소 | 기호 | 원자번호 | 주요 용도 |
|------|------|----------|-----------|
| 가돌리늄 | Gd | 64 | MRI 조영제, 자석 |
| 테르븀 | Tb | 65 | 형광체, 자석 (고온용) |
| 디스프로슘 | Dy | 66 | 영구자석 (고온용), 레이저 |
| 홀뮴 | Ho | 67 | 레이저, 자석 |
| 어븀 | Er | 68 | 광섬유, 레이저, 착색제 |
| 툴륨 | Tm | 69 | 의료용 X선, 레이저 |
| 이터븀 | Yb | 70 | 레이저, 합금, 스테인리스강 |
| 루테튬 | Lu | 71 | 촉매, PET 스캔, 합금 |

#### 준희토류 (Associated)
| 원소 | 기호 | 원자번호 | 주요 용도 |
|------|------|----------|-----------|
| 스칸듐 | Sc | 21 | 항공 합금, LED, 연료전지 |
| 이트륨 | Y | 39 | LED, 레이저, 초전도체 |

### 3.2 대상 광상

- **바스트네사이트(Bastnäsite)**: 탄산염 광물 (세계 주요 광상)
- **모나자이트(Monazite)**: 인산염 광물 (토륨 함유 주의)
- **제노타임(Xenotime)**: 중희토류 주 공급원
- **이온흡착형 점토(Ion-Adsorption Clay)**: 중국 남부 특수 광상

### 3.3 제외 대상

- 군사용 희토류 채굴 (별도 보안 표준 적용)
- 해저 채굴 (기술 개발 중, 향후 표준 예정)

---

## 4. 용어 정의

### 4.1 핵심 용어

| 용어 | 정의 |
|------|------|
| **희토류 원소 (REE)** | 란타넘계 원소 15개 + 스칸듐, 이트륨 |
| **경희토류 (LREE)** | La, Ce, Pr, Nd, Pm, Sm, Eu (원자번호 57-63) |
| **중희토류 (HREE)** | Gd, Tb, Dy, Ho, Er, Tm, Yb, Lu (원자번호 64-71) |
| **REO** | Rare Earth Oxide (희토류 산화물) |
| **TREO** | Total Rare Earth Oxide (총 희토류 산화물) |
| **바스트네사이트** | (Ce,La)CO₃F - 경희토류 주 광상 |
| **모나자이트** | (Ce,La,Nd,Th)PO₄ - 토륨 함유 광물 |
| **이온흡착형** | 점토에 이온 상태로 흡착된 희토류 광상 |

### 4.2 광업 용어

| 용어 | 정의 |
|------|------|
| **품위 (Grade)** | 광석 내 희토류 함량 (% TREO) |
| **정광 (Concentrate)** | 선광 후 농축된 광물 |
| **침출 (Leaching)** | 화학 용액으로 금속 추출 |
| **용매 추출 (SX)** | Solvent Extraction - 희토류 분리 |
| **소성 (Calcination)** | 고온 처리로 산화물 생성 |
| **환원 (Reduction)** | 산화물 → 금속 전환 |

---

## 5. 희토류 원소 분류

### 5.1 전략적 중요도 등급

```
CRITICAL (매우 중요):
  - 네오디뮴 (Nd): 영구자석 - 전기차, 풍력
  - 디스프로슘 (Dy): 고온 자석 - 전기차 모터
  - 테르븀 (Tb): 형광체, 자석
  - 유로퓸 (Eu): LED 형광체
  - 이트륨 (Y): LED, 레이저

HIGH (중요):
  - 프라세오디뮴 (Pr): 자석 합금
  - 사마륨 (Sm): SmCo 자석
  - 가돌리늄 (Gd): MRI, 자석

MODERATE (보통):
  - 란타넘 (La): 촉매, 배터리
  - 세륨 (Ce): 촉매, 연마재
  - 어븀 (Er): 광섬유 증폭기
```

### 5.2 가격 변동성 (2024 기준)

| 원소 | 가격 (USD/kg) | 최근 5년 변동폭 | 주요 공급국 |
|------|---------------|----------------|-------------|
| **네오디뮴 (Nd)** | $80-150 | ±300% | 중국 (85%), 호주, 미얀마 |
| **디스프로슘 (Dy)** | $350-500 | ±400% | 중국 (95%), 베트남 |
| **테르븀 (Tb)** | $1,200-1,800 | ±350% | 중국 (95%) |
| **유로퓸 (Eu)** | $800-1,200 | ±250% | 중국 (90%) |
| **이트륨 (Y)** | $10-20 | ±150% | 중국 (70%), 베트남 |
| **세륨 (Ce)** | $5-10 | ±100% | 중국 (60%), 베트남 |

---

## 6. 광상 유형

### 6.1 바스트네사이트 (Bastnäsite)

#### 특성
```yaml
화학식: (Ce,La)CO₃F
품위: 5-10% TREO
희토류 구성: LREE 주도 (Ce 50%, La 30%, Nd 12%)
토륨 함량: 낮음 (<0.1%)
주요 광산: Mountain Pass (미국), Bayan Obo (중국)
```

#### 장점
- 토륨 함량 낮아 방사성 폐기물 최소
- 경제성 우수
- 대규모 매장량

#### 단점
- 중희토류 함량 낮음
- 대형 노천 채굴 필요 (환경 영향)

### 6.2 모나자이트 (Monazite)

#### 특성
```yaml
화학식: (Ce,La,Nd,Th)PO₄
품위: 3-12% TREO
희토류 구성: LREE (Ce, La, Nd 주)
토륨 함량: 높음 (3-12%)
주요 산지: 인도, 브라질, 호주 (해변 사광상)
```

#### 장점
- 네오디뮴 함량 높음
- 해변 사광상으로 채굴 용이
- 부산물로 토륨 회수 가능

#### 단점
- **방사성 폐기물 대량 발생** (토륨, 우라늄)
- 환경 규제 엄격
- 분리 공정 복잡

### 6.3 이온흡착형 점토 (Ion-Adsorption Clay)

#### 특성
```yaml
형태: 점토 광물에 이온 형태로 흡착
품위: 0.05-0.2% TREO (매우 낮음)
희토류 구성: HREE 주도 (Dy, Tb, Y 풍부)
방사성: 거의 없음
주요 산지: 중국 남부 (광둥, 장시, 푸젠)
```

#### 장점
- **중희토류 주 공급원** (Dy, Tb, Y)
- 원위치 침출(In-Situ Leaching) 가능
- 방사성 폐기물 없음

#### 단점
- 품위 극히 낮음 (대량 처리 필요)
- 암모늄 침출로 심각한 수질 오염
- 산림 파괴 (표토 제거)

---

## 7. 채굴 방법

### 7.1 노천 채굴 (Open-Pit Mining)

#### 적용: 바스트네사이트, 제노타임 광상

```
Process:
1. 표토 제거 (Overburden Removal)
   - 깊이: 10-100m
   - 폐석량: 광석의 5-20배

2. 채광 (Ore Extraction)
   - 발파 → 굴삭 → 운반
   - 광석 품위: 5-10% TREO

3. 일차 선광 (Primary Beneficiation)
   - 파쇄 → 분쇄 → 자력/부유 선광
   - 정광 품위: 50-70% TREO
```

#### 환경 영향
```
- 토지 훼손: 1 톤 REO 생산 → 2,000 m² 토지 파괴
- 폐석 발생: 1 톤 REO → 200-400 톤 폐석
- 복원 필수: 채굴 종료 후 지형 복원, 식생 회복
```

### 7.2 원위치 침출 (In-Situ Leaching, ISL)

#### 적용: 이온흡착형 점토 광상

```
Process:
1. 탐사 시추 (Exploration Drilling)
   - 광체 확인 및 품위 측정
   - 침출 구역 설계

2. 침출액 주입 (Lixiviant Injection)
   - 침출제: 황산암모늄 (NH₄)₂SO₄ 3-5%
   - 주입 속도: 10-20 L/m²/day
   - 침출 시간: 30-90일

3. 모액 회수 (Pregnant Leach Solution, PLS)
   - REE 농도: 200-500 ppm
   - 침전제: 옥살산 (C₂H₂O₄)

4. 희토류 침전
   - 희토류 옥살산염 → 소성 → REO
```

#### 환경 위험
```
⚠️ 심각한 환경 문제:
- 암모늄 오염: 지하수 질산염 농도 급증
- 토양 산성화: pH 4-5로 저하
- 식생 고사: 침출 지역 식물 대량 사멸
- 중금속 유출: Al, Mn, Fe 동반 용출
```

### 7.3 사광 채굴 (Placer Mining)

#### 적용: 모나자이트 해변 사광상

```
Process:
1. 준설 (Dredging)
   - 해변 모래 수집
   - 처리량: 1,000-5,000 톤/일

2. 중력 선광 (Gravity Separation)
   - 스파이럴 농축기
   - 자력 선별
   - 정전 선별

3. 정광 회수
   - 모나자이트 정광: 50-60% TREO
   - 부산물: 일메나이트, 지르콘, 루타일
```

---

## 8. 분리 및 정제 공정

### 8.1 희토류 추출 (Extraction)

#### 산 침출 (Acid Leaching)

```
바스트네사이트 처리:
1. 소성 (Roasting)
   - 온도: 600-900°C
   - (Ce,La)CO₃F → (Ce,La)₂O₃ + CO₂ + HF

2. 황산 침출
   - 농도: H₂SO₄ 70-90%
   - 온도: 80-100°C
   - 시간: 2-4시간
   - REO → RE₂(SO₄)₃ (가용성 황산염)

3. 불순물 제거
   - Fe, Al, Ca, Th 침전 제거
   - pH 조정: 1.5-3.0
```

#### 알칼리 침출 (Alkali Leaching)

```
모나자이트 처리:
1. 수산화나트륨 분해
   - NaOH 70% @ 150°C, 3시간
   - (Ce,La,Th)PO₄ + 3NaOH → (Ce,La,Th)(OH)₃ + Na₃PO₄

2. 염산 용해
   - HCl 6M
   - RE(OH)₃ → RECl₃

3. 토륨 분리 ⚠️
   - pH 5.8: ThO₂ 침전
   - 방사성 폐기물 처리 필수
```

### 8.2 용매 추출 (Solvent Extraction, SX)

희토류 원소들은 화학적 성질이 매우 유사하여 **수십 단계의 용매 추출**이 필요합니다.

#### 추출제 (Extractant)

```yaml
P507 (D2EHPA):
  - 가장 일반적인 추출제
  - 유기상: P507 in kerosene (0.5-1M)
  - pH 의존적: 원소별 최적 pH 차이 이용

P204 (HDEHP):
  - 중희토류 분리에 효과적

Cyanex 272:
  - HREE/LREE 분리
```

#### 분리 순서

```
혼합 희토류 용액 (Mixed RE Solution)
  ↓ SX Stage 1: Ce⁴⁺ 분리 (산화 후 추출)
세륨 제거 용액
  ↓ SX Stage 2-10: La, Pr, Nd 순차 분리
  ↓ 각 원소별 5-15 단계 추출
  ↓ SX Stage 11-20: Sm, Eu, Gd 분리
  ↓ SX Stage 21-35: HREE 분리 (Tb, Dy, Ho, Er, Tm, Yb, Lu)
고순도 개별 희토류 용액 (99.9%+)
```

#### 회수 (Stripping)

```
1. 역추출 (Back Extraction)
   - 강산 사용: HCl 4-6M 또는 H₂SO₄ 1-2M
   - 고농도 희토류 용액 획득

2. 침전 (Precipitation)
   - 옥살산: (NH₄)₂C₂O₄ → RE₂(C₂O₄)₃
   - 또는 탄산염: (NH₄)₂CO₃ → RE₂(CO₃)₃

3. 소성 (Calcination)
   - 900-1,100°C
   - RE₂(C₂O₄)₃ → RE₂O₃ + CO + CO₂
```

### 8.3 금속 환원 (Metal Reduction)

#### 전기분해 (Electrolysis)

```
용융염 전기분해:
- 온도: 800-1,000°C
- 전해질: RECl₃ + NaCl/KCl
- 음극: 희토류 금속 석출
- 양극: 염소 가스 발생
- 순도: 99-99.9%
```

#### 금속 열환원 (Metallothermic Reduction)

```
칼슘 환원:
- RE₂O₃ + 3Ca → 2RE + 3CaO
- 온도: 1,100-1,300°C (진공/아르곤)
- 용도: Nd, Dy 금속 생산

플루오라이드 환원:
- REF₃ + 3Ca → RE + 3CaF₂
- 더 높은 순도 (99.9%+)
```

---

## 9. 방사성 폐기물 관리

### 9.1 방사성 물질 현황

#### 토륨 (Thorium, Th-232)

```yaml
주요 광상: 모나자이트 (3-12% ThO₂)
반감기: 14.05억 년 (⍺ 붕괴)
방사선: 알파선 (낮은 투과력, 높은 이온화)
건강 위험: 흡입 시 폐암, 골암 위험
규제: IAEA Transport Category I-III
```

#### 우라늄 (Uranium, U-238/U-235)

```yaml
함량: 모나자이트 0.1-0.5%
반감기: U-238 44.7억 년, U-235 7.04억 년
방사선: 알파선, 감마선
규제: 핵물질 관리 (IAEA)
```

### 9.2 방사성 폐기물 분류

#### Level 1: 저준위 (Low-Level)

```
특성:
- 방사능: <100 Bq/g
- 반감기: <30년
- 대상: 바스트네사이트 부산물, 제노타임

처리:
- 격리 매립 (Isolated Disposal)
- 차폐: 콘크리트 또는 납 차폐
- 모니터링: 연 2회 이상
```

#### Level 2: 중준위 (Intermediate-Level)

```
특성:
- 방사능: 100-10,000 Bq/g
- 토륨 함량: 0.5-3%
- 대상: 모나자이트 정광, 침전 슬러지

처리:
- 시멘트 고화 (Cementation)
- 스테인리스 드럼 밀봉
- 중간 저장 시설 (30-50년)
- 최종 처분장 격리
```

#### Level 3: 고준위 (High-Level)

```
특성:
- 방사능: >10,000 Bq/g
- 토륨 농축물 (>10% ThO₂)
- 우라늄 농축물

처리:
- 유리화 (Vitrification) 또는 시너지 고화
- 다중 차폐 (스테인리스 + 납 + 콘크리트)
- 깊은 지층 처분 (Deep Geological Repository)
- 1,000년 이상 격리
```

### 9.3 작업자 안전

#### 방사선 안전 기준

```yaml
선량 한도 (IAEA):
  작업자: 20 mSv/년 (평균 5년)
  일반인: 1 mSv/년

개인 보호구 (PPE):
  - 전신 보호복 (Tyvek suit)
  - 호흡 보호구 (PAPR, P100 필터)
  - 이중 장갑
  - 개인선량계 (TLD 또는 OSL)

모니터링:
  - 공기 중 라돈: <400 Bq/m³
  - 표면 오염: <4 Bq/cm² (알파)
  - 개인 선량: 월 단위 측정
```

---

## 10. 공급망 보안

### 10.1 전략적 중요성

#### 글로벌 공급망 현황 (2024)

```
생산 (광산):
  중국:     60% (170,000 톤/년)
  미국:     15% (43,000 톤/년)
  미얀마:    8% (23,000 톤/년)
  호주:     7% (20,000 톤/년)
  베트남:   5% (14,000 톤/년)
  기타:     5%

분리/정제:
  중국:     85% (절대 우위)
  에스토니아: 5%
  말레이시아: 4%
  미국:     3%
  기타:     3%

소비 (최종 제품):
  중국:     40%
  일본:     15%
  미국:     12%
  EU:       10%
  한국:     8%
  기타:     15%
```

#### 공급망 위험

```
⚠️ 주요 위험 요소:
1. 지정학적 긴장 (중국-미국 무역 분쟁)
2. 수출 쿼터 및 제한 (중국 2010-2015 사례)
3. 환경 규제 강화 (채굴 중단)
4. 가격 조작 (시장 지배력 남용)
5. 단일 공급원 의존 (SPOF)
```

### 10.2 공급망 다변화 전략

#### Tier 1: 1차 공급원

```
목표: 자체 광산 및 정제 능력 확보

주요 프로젝트:
- 미국: Mountain Pass (MP Materials)
- 호주: Mt Weld (Lynas Rare Earths)
- 캐나다: Nechalacho (Vital Metals)
- 브라질: Araxá (CBMM)
```

#### Tier 2: 재활용 (Urban Mining)

```
e-waste에서 희토류 회수:
- 하드디스크: Nd 자석 (1-2g/HDD)
- 형광등: Y, Eu 형광체
- 촉매: La, Ce 회수
- 배터리: La-Ni 배터리

경제성:
- 1차 채굴 대비 30-50% 저렴
- CO₂ 배출 70% 감소
- 방사성 폐기물 없음
```

#### Tier 3: 대체 재료 연구

```
자석:
- Ferrite (페라이트) 자석 개발
- Mn-Al 자석 (Dy-free)

형광체:
- 무희토류 LED (InGaN 기반)

촉매:
- Pt, Pd 기반 촉매 대체
```

### 10.3 블록체인 추적

```json
{
  "batchId": "REE-2025-MP-0012345",
  "origin": {
    "mine": "Mountain Pass",
    "country": "USA",
    "gps": {"lat": 35.4887, "lon": -115.5470},
    "extractionDate": "2025-11-15"
  },
  "composition": {
    "Nd2O3": "12.8%",
    "Pr6O11": "3.5%",
    "Dy2O3": "0.12%",
    "TREO": "65.3%"
  },
  "processing": [
    {
      "facility": "MP Materials Processing",
      "step": "SEPARATION",
      "date": "2025-11-20",
      "certification": "ISO14001"
    },
    {
      "facility": "Neo Performance Materials",
      "step": "METAL_REDUCTION",
      "date": "2025-11-25",
      "certification": "ISO9001"
    }
  ],
  "radioactive": {
    "thorium_ppm": 85,
    "uranium_ppm": 12,
    "radiation_Bq_g": 18,
    "safetyLevel": "LOW"
  },
  "blockchain": {
    "txHash": "0xabc123...",
    "network": "WIA-BLOCKCHAIN",
    "timestamp": "2025-11-26T08:00:00Z"
  }
}
```

---

## 11. 데이터 형식 표준

### 11.1 광산 등록

```json
{
  "mineId": "REE-MINE-2025-USA-001",
  "registrationDate": "2025-12-25T00:00:00Z",
  "mine": {
    "name": "Mountain Pass Rare Earth Mine",
    "operator": "MP Materials",
    "country": "USA",
    "state": "California",
    "location": {
      "lat": 35.4887,
      "lon": -115.5470,
      "elevation_m": 1340
    },
    "mineType": "OPEN_PIT",
    "oreType": "BASTNASITE",
    "capacity_tons_per_year": 40000,
    "treoGrade_percent": 7.8
  },
  "composition": {
    "La2O3_percent": 33.2,
    "Ce2O3_percent": 49.1,
    "Pr6O11_percent": 4.3,
    "Nd2O3_percent": 12.0,
    "Sm2O3_percent": 0.8,
    "Eu2O3_percent": 0.1,
    "Gd2O3_percent": 0.2,
    "other_percent": 0.3
  },
  "radioactive": {
    "thorium_ppm": 150,
    "uranium_ppm": 20,
    "classification": "LOW_LEVEL"
  },
  "environmental": {
    "waterUsage_m3_per_ton": 8.5,
    "landDisturbance_hectares": 485,
    "restorationPlan": true,
    "restorationFund_USD": 25000000
  },
  "certifications": ["ISO14001", "ISO9001", "OHSAS18001"]
}
```

### 11.2 생산 배치 기록

```json
{
  "batchId": "REE-BATCH-2025-MP-0012345",
  "productionDate": "2025-12-20",
  "mineId": "REE-MINE-2025-USA-001",
  "processing": {
    "facilityId": "REE-PROC-USA-MP-001",
    "facilityName": "MP Materials Processing Facility",
    "location": {
      "city": "Mountain Pass",
      "state": "CA",
      "country": "USA"
    }
  },
  "stages": [
    {
      "stage": "EXTRACTION",
      "method": "OPEN_PIT_MINING",
      "date": "2025-12-10",
      "rawOre_tons": 1000,
      "treoGrade_percent": 7.8
    },
    {
      "stage": "BENEFICIATION",
      "method": "FLOTATION",
      "date": "2025-12-12",
      "concentrate_tons": 150,
      "treoGrade_percent": 52.0,
      "recovery_percent": 85
    },
    {
      "stage": "ROASTING",
      "temperature_C": 750,
      "date": "2025-12-14",
      "reo_tons": 120
    },
    {
      "stage": "ACID_LEACHING",
      "acid": "H2SO4",
      "concentration_percent": 85,
      "date": "2025-12-15",
      "leach_efficiency_percent": 92
    },
    {
      "stage": "SOLVENT_EXTRACTION",
      "extractant": "P507",
      "stages": 28,
      "date": "2025-12-18",
      "separation": {
        "La2O3_kg": 3500,
        "Ce2O3_kg": 5200,
        "Pr6O11_kg": 450,
        "Nd2O3_kg": 1280,
        "others_kg": 70
      }
    }
  ],
  "products": [
    {
      "element": "Nd",
      "compound": "Nd2O3",
      "quantity_kg": 1280,
      "purity_percent": 99.5,
      "pricePerKg_USD": 95,
      "totalValue_USD": 121600
    },
    {
      "element": "Pr",
      "compound": "Pr6O11",
      "quantity_kg": 450,
      "purity_percent": 99.0,
      "pricePerKg_USD": 85,
      "totalValue_USD": 38250
    }
  ],
  "waste": {
    "tailings_tons": 850,
    "radioactiveWaste_kg": 125,
    "radioactivity_Bq": 2500,
    "wasteWater_m3": 8500,
    "treatment": "NEUTRALIZATION_AND_SETTLING"
  },
  "environmentalImpact": {
    "co2Emissions_tons": 12.5,
    "waterUsage_m3": 8500,
    "energyConsumption_MWh": 85
  }
}
```

---

## 12. API 인터페이스

### 12.1 RESTful API 엔드포인트

#### 광산 등록

```http
POST /api/v1/mines/register
Content-Type: application/json
Authorization: Bearer {api_key}

Request Body:
{
  "mine": {...},
  "composition": {...},
  "radioactive": {...},
  "environmental": {...}
}

Response (201 Created):
{
  "mineId": "REE-MINE-2025-USA-001",
  "registrationNumber": "US-DOI-REE-2025-1234",
  "certificationUrl": "https://cert.wia.org/ree-041/mines/REE-MINE-2025-USA-001.pdf",
  "trackingUrl": "https://track.wia.org/ree-041/REE-MINE-2025-USA-001"
}
```

#### 배치 생산 기록

```http
POST /api/v1/batches/submit
Content-Type: application/json
Authorization: Bearer {api_key}

Request Body:
{
  "batchId": "REE-BATCH-2025-MP-0012345",
  "mineId": "REE-MINE-2025-USA-001",
  "processing": {...},
  "stages": [...],
  "products": [...]
}

Response (200 OK):
{
  "batchId": "REE-BATCH-2025-MP-0012345",
  "blockchainTx": "0xabc123...",
  "certificateUrl": "https://cert.wia.org/ree-041/batches/REE-BATCH-2025-MP-0012345.pdf",
  "qrCode": "https://api.wia.org/ree-041/qr/REE-BATCH-2025-MP-0012345.png"
}
```

#### 공급망 추적

```http
GET /api/v1/batches/{batchId}/supply-chain
Authorization: Bearer {api_key}

Response (200 OK):
{
  "batchId": "REE-BATCH-2025-MP-0012345",
  "origin": {
    "mine": "Mountain Pass",
    "country": "USA",
    "extractionDate": "2025-12-10"
  },
  "supplyChain": [
    {
      "stage": "MINING",
      "facility": "Mountain Pass Mine",
      "date": "2025-12-10",
      "location": "California, USA"
    },
    {
      "stage": "PROCESSING",
      "facility": "MP Materials Processing",
      "date": "2025-12-15",
      "location": "California, USA"
    },
    {
      "stage": "SEPARATION",
      "facility": "MP Materials SX Plant",
      "date": "2025-12-18",
      "location": "California, USA"
    },
    {
      "stage": "SHIPMENT",
      "destination": "Neo Performance Materials",
      "date": "2025-12-20",
      "location": "Colorado, USA"
    }
  ],
  "currentLocation": {
    "facility": "Neo Performance Materials",
    "address": "13777 E. Arapahoe Rd, Centennial, CO",
    "timestamp": "2025-12-22T10:00:00Z"
  },
  "products": [
    {
      "element": "Nd",
      "form": "Nd2O3",
      "quantity_kg": 1280,
      "purity_percent": 99.5
    }
  ],
  "certifications": ["ISO9001", "ISO14001", "IATF16949"],
  "radioactiveSafety": {
    "level": "LOW",
    "radiation_Bq_g": 18,
    "certified": true
  }
}
```

#### 가격 조회

```http
GET /api/v1/prices/current
Authorization: Bearer {api_key}

Response (200 OK):
{
  "timestamp": "2025-12-25T08:00:00Z",
  "currency": "USD",
  "unit": "kg",
  "prices": [
    {"element": "La", "compound": "La2O3", "price": 3.50, "change_7d": "+2.3%"},
    {"element": "Ce", "compound": "CeO2", "price": 2.80, "change_7d": "-1.5%"},
    {"element": "Pr", "compound": "Pr6O11", "price": 85.00, "change_7d": "+5.2%"},
    {"element": "Nd", "compound": "Nd2O3", "price": 95.00, "change_7d": "+8.7%"},
    {"element": "Dy", "compound": "Dy2O3", "price": 420.00, "change_7d": "+12.3%"},
    {"element": "Tb", "compound": "Tb4O7", "price": 1500.00, "change_7d": "+15.8%"},
    {"element": "Y", "compound": "Y2O3", "price": 12.50, "change_7d": "+3.1%"}
  ],
  "indices": {
    "WIA_REE_Index": 1285.4,
    "change_7d": "+7.2%",
    "change_30d": "+18.5%"
  }
}
```

### 12.2 WebSocket 실시간 가격 피드

```javascript
// 실시간 희토류 가격 모니터링
const ws = new WebSocket('wss://api.wia.org/v1/ree-041/prices/stream');

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('실시간 가격:', data);
};

// 실시간 데이터 예시
{
  "timestamp": "2025-12-25T08:15:32Z",
  "element": "Nd",
  "compound": "Nd2O3",
  "price_USD_kg": 96.20,
  "change_percent": "+1.3%",
  "volume_kg": 12500,
  "exchange": "Shanghai Rare Earth Exchange"
}
```

---

## 13. 추적 및 보고

### 13.1 MRV (Monitoring, Reporting, Verification)

#### Tier 1: 실시간 모니터링

```yaml
광산 모니터링:
  - 일일 생산량 (톤/일)
  - TREO 품위 (%)
  - 개별 원소 함량
  - 방사성 폐기물 발생량
  - 용수 사용량
  - 에너지 소비량

업데이트 주기: 실시간 (1분)
접근: https://dashboard.wia.org/ree-041
```

#### Tier 2: 월간 보고서

```yaml
규제 기관 제출용:
  - 생산량 통계
  - 방사성 폐기물 관리 내역
  - 환경 영향 평가
  - 복원 진척도
  - 안전 사고 기록

형식: PDF + JSON + XML
제출 기한: 매월 10일까지
제출처: 광업국, 환경부, 원자력안전위원회
```

#### Tier 3: 연간 감사

```yaml
제3자 검증:
  - 현장 실사 (광산, 정제 시설)
  - 방사성 안전 감사
  - 환경 복원 검증
  - 공급망 투명성 감사
  - ISO 인증 갱신

감사 기관: SGS, Bureau Veritas, TÜV
주기: 연 1회
```

### 13.2 성과 지표 (KPI)

```yaml
생산 효율:
  채광 효율: >85%
  정광 회수율: >80%
  분리 순도: >99.5%
  금속 수율: >95%

환경 성과:
  용수 재활용률: >70%
  폐기물 재활용: >60%
  토지 복원율: >90% (채굴 종료 5년 내)
  CO₂ 배출 감축: >20% (vs. 2020 기준)

안전 지표:
  방사선 피폭: <10 mSv/년 (작업자)
  환경 방사능: <100 Bq/m² (주변 지역)
  안전 사고율: <1.0 per 200,000 work hours
  환경 사고: 0건
```

---

## 14. 인증 및 준수

### 14.1 국제 표준

#### ISO 14001 (환경경영시스템)
```
범위: 채굴 및 정제 환경 관리
요구사항:
  ✓ 환경 정책 수립
  ✓ 환경 영향 평가
  ✓ 법규 준수
  ✓ 지속적 개선
```

#### ISO 9001 (품질경영시스템)
```
범위: 희토류 제품 품질 관리
요구사항:
  ✓ 품질 방침
  ✓ 공정 관리
  ✓ 검사 및 시험
  ✓ 추적성
```

#### IATF 16949 (자동차 산업)
```
범위: 자동차용 희토류 자석 공급
요구사항:
  ✓ 제품 승인 절차 (PPAP)
  ✓ 공정 능력 (Cpk ≥ 1.67)
  ✓ 추적성 및 리콜 시스템
```

### 14.2 방사성 안전 규제

#### IAEA (국제원자력기구)

```
BSS (Basic Safety Standards):
  - 작업자 선량 한도: 20 mSv/년 (평균 5년)
  - 일반인 선량 한도: 1 mSv/년
  - 방사성 폐기물 분류 및 처분

Transport Regulations:
  - UN2912: Radioactive material, low specific activity
  - 포장 요구사항 (Type A, Type B)
  - 라벨 및 표시
```

#### 국가별 규제

```
미국 (NRC):
  - 10 CFR Part 20: 방사선 안전 기준
  - 40 CFR Part 190: 환경 방사선 기준
  - 광미 저장 시설 허가

중국 (MEE):
  - GB 18871-2002: 전리방사선 방호 기준
  - 희토류 산업 오염물 배출 기준

호주 (ARPANSA):
  - Radiation Protection and Nuclear Safety Act
  - 광산 방사선 안전 규정
```

---

## 15. 부록

### 15.1 참조 표준

- **ISO 14001**: 환경경영시스템
- **ISO 9001**: 품질경영시스템
- **IAEA BSS**: 방사선 안전 기본 기준
- **ASTM E1915**: 희토류 원소 분석 방법
- **IEC 62402**: 전자부품 신뢰성 관리
- **IATF 16949**: 자동차 산업 품질 표준

### 15.2 희토류 용도별 응용

| 원소 | 주요 응용 | 시장 규모 (2024) | 성장률 (CAGR) |
|------|-----------|------------------|---------------|
| **Nd** | 영구자석 (NdFeB) | $15B | 12% |
| **Dy** | 고온 자석, 자기 냉각 | $2B | 15% |
| **Tb** | 형광체, 자석 | $1.5B | 10% |
| **Y** | LED, 세라믹, 레이저 | $3B | 8% |
| **La** | 촉매, 배터리, 광학 | $2.5B | 6% |
| **Ce** | 촉매, 연마재, 합금 | $4B | 5% |
| **Eu** | LED 형광체, 위조방지 | $800M | 9% |
| **Pr** | 자석, 세라믹 착색 | $1.2B | 11% |

### 15.3 재활용 가능성

```
e-waste 재활용 잠재량 (2024):
  네오디뮴:   15,000 톤/년 (1차 생산의 25%)
  디스프로슘:  2,500 톤/년 (1차 생산의 40%)
  이트륨:      3,000 톤/년 (1차 생산의 15%)
  유로퓸:        200 톤/년 (1차 생산의 20%)

재활용 경제성:
  비용 절감:  30-50% (vs. 1차 채굴)
  CO₂ 감축:   70-80%
  방사성 폐기물: 0
  공급망 다변화: 국내 순환 가능
```

### 15.4 용어집 (Glossary)

| EN | KO | 약어 |
|----|----|------|
| Rare Earth Elements | 희토류 원소 | REE |
| Light REE | 경희토류 | LREE |
| Heavy REE | 중희토류 | HREE |
| Total Rare Earth Oxide | 총 희토류 산화물 | TREO |
| Bastnäsite | 바스트네사이트 | - |
| Monazite | 모나자이트 | - |
| Solvent Extraction | 용매 추출 | SX |
| In-Situ Leaching | 원위치 침출 | ISL |
| Permanent Magnet | 영구자석 | PM |
| Neodymium-Iron-Boron | 네오디뮴-철-붕소 | NdFeB |

### 15.5 변경 이력

| 버전 | 날짜 | 변경 내용 |
|------|------|-----------|
| 1.0.0 | 2025-12-25 | 초기 릴리스 |

---

## 결론

WIA-ENE-041 희토류 채굴 표준은 **弘益人間 (홍익인간)**의 철학을 바탕으로, 희토류 자원의 지속가능한 채굴, 안전한 방사성 폐기물 관리, 투명한 공급망 구축을 목표로 합니다.

이 표준을 통해:
- 🌍 **환경 보호**: 채굴 지역 환경 복원 및 오염 최소화
- ⚛️ **방사성 안전**: 토륨, 우라늄 폐기물의 안전한 관리
- 🔒 **공급망 보안**: 블록체인 기반 원산지 추적 및 인증
- ♻️ **순환경제**: e-waste 재활용으로 2차 공급원 확보
- 🌏 **글로벌 협력**: 국제 표준과의 상호운용성

**함께 만드는 지속가능한 희토류 공급망**

희토류는 21세기 첨단 기술의 필수 요소입니다. 이 표준을 통해 환경을 보호하고, 안전을 확보하며, 공급망을 투명하게 관리함으로써 지속가능한 미래를 만들어갑니다.

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 (홍익인간) · 널리 인간을 이롭게 하라**
**Benefit All Humanity**

---

**문의**
- Website: https://wia-official.org
- Email: standards@wia-official.org
- GitHub: https://github.com/WIA-Official/wia-standards

**License**: MIT License
