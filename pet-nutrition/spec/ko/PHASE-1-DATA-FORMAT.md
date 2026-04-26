# WIA-PET-009: 1단계 - 데이터 형식 사양

**버전:** 1.0  
**상태:** 최종  
**최종 업데이트:** 2025-12-25  
**범주:** PET  
**주요 색상:** 앰버 #F59E0B

---

## 개요

WIA-PET-009 표준의 1단계는 반려동물 영양 관리를 위한 기본 데이터 구조를 정의합니다. 레벨 1 인증을 받으려면 모든 구현체가 이러한 핵심 데이터 모델을 지원해야 합니다.

### 설계 원칙

- **상호 운용성:** JSON 형식은 크로스 플랫폼 호환성 보장
- **확장성:** 선택적 필드로 변경 없이 미래 개선 가능
- **검증:** JSON 스키마로 자동 유효성 검사
- **가독성:** 명확한 필드명과 구조로 개발자 접근성

### 규정 준수 요구사항

**레벨 1 (기본) 인증 요구사항:**
- 핵심 엔티티의 모든 필수 필드 지원
- 모든 데이터에 대한 JSON 스키마 검증
- 적절한 데이터 타입 사용 (ISO 8601 날짜, 표준화된 단위)
- UTF-8 인코딩

---

## 핵심 데이터 모델

### 1. 반려동물 프로필

반려동물 프로필은 모든 영양 관련 데이터를 연결하는 중심 엔티티입니다.

**필수 필드:**
- `petId` (문자열): 고유 식별자 (UUID v4 권장)
- `species` (열거형): dog, cat, bird, rabbit, reptile, ferret, other
- `birthDate` (문자열): ISO 8601 날짜 (YYYY-MM-DD)
- `sex` (열거형): male, female, neutered_male, spayed_female
- `weight.current` (숫자): 현재 체중 (kg)
- `weight.unit` (문자열): 항상 "kg"

**선택적 필드:**
- `name` (문자열): 반려동물 이름
- `breed` (문자열): 품종 또는 혼합종 설명
- `microchipId` (문자열): ISO 11784/11785 마이크로칩 번호
- `weight.ideal` (숫자): 목표 체중 (kg)
- `weight.history` (배열): 과거 체중 측정치
- `activityLevel` (열거형): sedentary, moderate, active, very_active
- `healthConditions` (배열): 의학적 상태
- `allergies` (배열): 알려진 알레르기 항원
- `medications` (배열): 현재 복용 중인 약물

### 2. 영양 요구사항

반려동물 프로필을 기반으로 계산되며, 일일 영양소 필요량을 명시합니다.

**필수 필드:**
- `petId` (문자열): 반려동물 프로필 링크
- `dailyCaloricNeeds` (숫자): 총 kcal/일
- `macronutrients.protein.minimum` (숫자): 일일 그램
- `macronutrients.fat.minimum` (숫자): 일일 그램

### 3. 사료 제품

반려동물 사료의 제품 정보입니다.

**필수 필드:**
- `productId` (문자열): 고유 제품 식별자
- `name` (문자열): 제품명
- `manufacturer` (문자열): 제조사명
- `productType` (열거형): 제품 범주
- `targetSpecies` (배열): 대상 종
- `guaranteedAnalysis`: 보증 분석 (조단백, 조지방, 조섬유, 수분)

### 4. 식단 계획

반려동물을 위한 처방된 급식 계획입니다.

**필수 필드:**
- `planId` (문자열): 고유 계획 식별자
- `petId` (문자열): 이 계획이 적용되는 반려동물
- `startDate` (문자열): ISO 8601 날짜
- `goal` (열거형): maintenance, weight_loss, weight_gain, therapeutic, growth
- `dailyMeals` (배열): 분량이 포함된 식사 일정

### 5. 급식 로그

실제 사료 소비 기록입니다.

**필수 필드:**
- `logId` (문자열): 고유 로그 식별자
- `petId` (문자열): 반려동물 식별자
- `timestamp` (문자열): ISO 8601 날짜시간
- `foodsConsumed` (배열): 무엇을 얼마나 먹었는지

---

## 데이터 타입 및 검증

### 표준 단위

**체중:** 킬로그램 (kg) - 모든 체중 측정은 kg 사용 필수
**에너지:** 킬로칼로리 (kcal) - 대사 에너지용
**부피:** 밀리리터 (mL) 또는 리터 (L)
**온도:** 섭씨 (°C)

### 날짜/시간 형식

**날짜:** ISO 8601 (YYYY-MM-DD)  
예시: `"2025-12-25"`

**날짜시간:** 시간대가 포함된 ISO 8601 (YYYY-MM-DDTHH:MM:SSZ)  
예시: `"2025-12-25T14:30:00Z"`

### 식별자

**반려동물 ID:** `PET-YYYY-XXXX` (YYYY는 연도, XXXX는 영숫자)  
예시: `"PET-2025-BELLA-7821"`

**보호자 ID:** `OWNER-YYYY-NNNNNN` (NNNNNN은 6-12자리 숫자)  
예시: `"OWNER-2025-567890"`

**제품 ID:** `PROD-XXXX` 형식  
예시: `"PROD-SENIOR-FORMULA-001"`

---

**弘益人間 (홍익인간) · 널리 인간을 이롭게 하라**

© 2025 WIA (세계 인증 산업 협회)  
표준 ID: WIA-PET-009 | 버전: 1.0 | 단계: 1
