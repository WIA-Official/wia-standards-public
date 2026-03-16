# WIA-CITY-015: 출입 통제 시스템 표준 v1.0 🔐

> **弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

## 문서 정보

- **표준 ID**: WIA-CITY-015
- **표준명**: Access Control System Standard (출입 통제 시스템 표준)
- **버전**: 1.0.0
- **발행일**: 2025-12-25
- **상태**: 정식 (Active)
- **카테고리**: CITY (건축/도시)
- **라이선스**: MIT License
- **발행 기관**: WIA (World Certification Industry Association)

---

## 목차

1. [개요](#1-개요)
2. [적용 범위](#2-적용-범위)
3. [용어 정의](#3-용어-정의)
4. [시스템 아키텍처](#4-시스템-아키텍처)
5. [인증 방법](#5-인증-방법)
6. [카드키 시스템](#6-카드키-시스템)
7. [생체인식 시스템](#7-생체인식-시스템)
8. [PIN 및 비밀번호 인증](#8-pin-및-비밀번호-인증)
9. [방문자 관리 시스템](#9-방문자-관리-시스템)
10. [주차장 출입 통제](#10-주차장-출입-통제)
11. [엘리베이터 층별 접근 제어](#11-엘리베이터-층별-접근-제어)
12. [출입 로그 및 감사](#12-출입-로그-및-감사)
13. [비상시 출입 통제](#13-비상시-출입-통제)
14. [데이터 형식](#14-데이터-형식)
15. [API 명세](#15-api-명세)
16. [보안 요구사항](#16-보안-요구사항)
17. [적합성 평가](#17-적합성-평가)

---

## 1. 개요

### 1.1 목적

WIA-CITY-015 출입 통제 시스템 표준은 건물, 시설, 구역의 물리적 접근을 체계적으로 관리하고 제어하기 위한 통합 표준입니다. 본 표준은 다양한 인증 방법, 출입 정책, 로그 관리, 비상 대응 절차를 정의하여 보안성과 편의성을 동시에 실현합니다.

### 1.2 배경

- 스마트 빌딩과 IoT 기술의 발전으로 지능형 출입 통제 시스템 수요 증가
- 생체인식 기술의 고도화로 비접촉 인증 방식 확산
- 코로나19 팬데믹 이후 위생적이고 안전한 출입 관리 필요성 증대
- GDPR, 개인정보보호법 등 데이터 보호 규제 강화
- 모바일 크리덴셜 및 클라우드 기반 출입 관리 플랫폼 등장

### 1.3 적용 효과

- **보안 강화**: 무단 침입 방지 및 실시간 모니터링
- **운영 효율**: 자동화된 출입 관리로 인력 비용 절감
- **사용자 편의**: 다양한 인증 수단 제공 (카드, 모바일, 생체인식)
- **규정 준수**: 출입 로그 기록으로 감사 및 컴플라이언스 충족
- **비상 대응**: 화재, 재난 시 신속한 대피 및 출입 제어

### 1.4 핵심 기능

1. **다중 인증 방법**: 카드키(RFID/NFC), 생체인식(지문/얼굴/홍채), PIN/비밀번호, 모바일 크리덴셜
2. **세밀한 접근 제어**: 사용자별, 시간대별, 구역별 접근 권한 설정
3. **방문자 관리**: 사전 등록, QR 코드 발급, 임시 출입증
4. **주차장 연동**: 차량 번호 인식, 정기권 관리, 요금 자동 정산
5. **엘리베이터 제어**: 층별 접근 권한에 따른 버튼 활성화
6. **실시간 모니터링**: 출입 현황, 재실 인원, 이상 탐지
7. **출입 로그**: 모든 출입 기록 저장 및 검색, 감사 추적
8. **비상 모드**: 화재, 재난 시 자동 개방 또는 폐쇄

---

## 2. 적용 범위

### 2.1 시설 유형

- **오피스 빌딩**: 사무실, 회의실, 전산실, 임원실
- **주거 시설**: 아파트, 오피스텔, 기숙사, 공동 출입구
- **상업 시설**: 쇼핑몰, 백오피스, 창고, 금고
- **공공 시설**: 관공서, 도서관, 박물관, 미술관
- **산업 시설**: 공장, 연구소, 클린룸, 위험 구역
- **의료 시설**: 병원, 약품 보관소, 수술실, 격리 병동
- **교육 시설**: 학교, 대학, 실험실, 기숙사
- **데이터센터**: 서버실, 콜로케이션, 케이지

### 2.2 출입구 유형

- **주 출입구**: 건물 정문, 로비 게이트
- **부 출입구**: 직원 전용 출입구, 후문
- **내부 출입구**: 층간 이동, 구역 경계
- **주차장 출입구**: 차단기, 차량 게이트
- **엘리베이터**: 층별 접근 제어
- **특수 구역**: 전산실, 금고, 연구실

### 2.3 사용자 유형

- **정규 직원**: 상시 출입 권한 보유
- **임시 직원**: 계약 기간 내 출입 권한
- **방문자**: 사전 등록 또는 현장 등록
- **외주 업체**: 특정 구역 제한 출입
- **관리자**: 시스템 설정 및 모니터링 권한
- **비상 대응**: 소방, 경찰, 구조대

---

## 3. 용어 정의

### 3.1 핵심 용어

- **Access Point**: 출입 지점 (도어, 게이트, 턴스타일 등)
- **Credential**: 인증 수단 (카드, 생체정보, PIN 등)
- **Access Zone**: 접근 제어 구역
- **Access Level**: 접근 권한 수준
- **Anti-Passback**: 중복 출입 방지 (tailgating 방지)
- **Interlocking**: 상호 연동 (한 문이 열리면 다른 문 잠김)
- **Mantrap**: 이중 도어 보안 통로
- **First Person In / Last Person Out**: 최초 입실 / 최종 퇴실

### 3.2 인증 방식

- **RFID (Radio Frequency Identification)**: 무선 주파수 식별
- **NFC (Near Field Communication)**: 근거리 무선 통신
- **Biometric Authentication**: 생체 인증
- **PIN (Personal Identification Number)**: 개인 식별 번호
- **MFA (Multi-Factor Authentication)**: 다중 인증
- **Mobile Credential**: 모바일 크리덴셜 (스마트폰 앱)

### 3.3 생체인식 지표

- **FAR (False Acceptance Rate)**: 오수락률 (타인을 본인으로 잘못 인식)
- **FRR (False Rejection Rate)**: 오거부률 (본인을 타인으로 잘못 거부)
- **EER (Equal Error Rate)**: 등가 오류율 (FAR = FRR인 지점)
- **Liveness Detection**: 생체 활성 검출 (위조 방지)

---

## 4. 시스템 아키텍처

### 4.1 계층 구조

```
┌─────────────────────────────────────────────────────────────┐
│              Layer 5: 클라우드 관리 플랫폼                      │
│   (중앙 관리, 빅데이터 분석, 리포팅, 모바일 앱)                  │
└─────────────────────────────────────────────────────────────┘
                           ↕ HTTPS/REST API
┌─────────────────────────────────────────────────────────────┐
│              Layer 4: 출입 관리 서버                           │
│   (사용자 DB, 권한 관리, 로그 저장, 정책 엔진)                   │
└─────────────────────────────────────────────────────────────┘
                           ↕ TCP/IP (Wiegand over IP)
┌─────────────────────────────────────────────────────────────┐
│              Layer 3: 출입 제어기 (Door Controller)           │
│   (인증 처리, 도어 제어, 로컬 DB, 오프라인 모드)                 │
└─────────────────────────────────────────────────────────────┘
                           ↕ Wiegand / RS-485 / TCP/IP
┌─────────────────────────────────────────────────────────────┐
│              Layer 2: 리더기 및 센서                           │
│   (카드 리더, 생체인식 리더, PIR 센서, 도어 접촉 센서)           │
└─────────────────────────────────────────────────────────────┘
                           ↕ Physical Interface
┌─────────────────────────────────────────────────────────────┐
│              Layer 1: 물리적 장치                              │
│   (전기 도어락, 마그네틱 락, 턴스타일, 차단기)                   │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 통신 프로토콜

| 프로토콜 | 용도 | 특징 |
|---------|------|------|
| Wiegand | 리더 ↔ 컨트롤러 | 26비트, 34비트, 표준 인터페이스 |
| OSDP | 리더 ↔ 컨트롤러 | 암호화, 양방향 통신 |
| Modbus | 컨트롤러 ↔ 서버 | RTU/TCP, 산업 표준 |
| BACnet | 빌딩 자동화 연동 | ISO 16484-5 |
| MQTT | IoT 메시징 | 경량, Pub/Sub |
| REST API | 클라우드 연동 | HTTP/HTTPS, JSON |

### 4.3 데이터 흐름

```
사용자 → 리더기 → 컨트롤러 → 서버 → 클라우드
           ↓         ↓
        인증 요청   권한 확인
                     ↓
                   도어락 개방
                     ↓
                  출입 로그 저장
```

---

## 5. 인증 방법

### 5.1 인증 방법 분류

| 인증 방법 | 유형 | 보안 수준 | 편의성 | 비용 |
|----------|------|----------|--------|------|
| RFID 카드 | 소유 기반 | 낮음 | 높음 | 낮음 |
| NFC 카드 | 소유 기반 | 중간 | 높음 | 중간 |
| 모바일 크리덴셜 | 소유 기반 | 중간 | 매우 높음 | 낮음 |
| PIN 코드 | 지식 기반 | 낮음 | 중간 | 매우 낮음 |
| 지문 인식 | 생체 기반 | 높음 | 높음 | 중간 |
| 얼굴 인식 | 생체 기반 | 높음 | 매우 높음 | 높음 |
| 홍채 인식 | 생체 기반 | 매우 높음 | 중간 | 매우 높음 |
| 정맥 인식 | 생체 기반 | 매우 높음 | 높음 | 높음 |

### 5.2 다중 인증 (MFA)

고보안 구역에서는 2개 이상의 인증 방법 조합 권장:

- **2-Factor**: 카드 + PIN
- **2-Factor**: 생체인식 + PIN
- **3-Factor**: 카드 + 생체인식 + PIN

**적용 예**:
- 데이터센터: 카드 + 생체인식
- 금고: 카드 + PIN + 관리자 승인
- 연구실: 생체인식 + PIN

---

## 6. 카드키 시스템

### 6.1 RFID 기술

**저주파 (LF, 125kHz)**:
- 표준: EM4100, HID Prox
- 읽기 거리: 10cm 이내
- 보안성: 낮음 (복제 취약)
- 용도: 기본 출입 통제

**고주파 (HF, 13.56MHz)**:
- 표준: MIFARE Classic, MIFARE DESFire, FeliCa
- 읽기 거리: 10cm 이내
- 보안성: 중간~높음
- 용도: 결제 연동, 안전한 출입 통제

**초고주파 (UHF, 860-960MHz)**:
- 읽기 거리: 최대 10m
- 용도: 주차장 차량 인식, 장거리 출입 통제

### 6.2 NFC (Near Field Communication)

**특징**:
- ISO 14443 Type A/B 호환
- 스마트폰 지원
- 양방향 통신
- 암호화 지원

**활용**:
- 모바일 출입증
- QR 코드와 함께 사용
- 결제 기능 통합

### 6.3 카드 데이터 구조

**Wiegand 26비트**:
```
[1비트 패리티] [8비트 Facility Code] [16비트 Card Number] [1비트 패리티]
```

**MIFARE Classic 1K**:
```
Sector 0:
  Block 0: UID (4바이트) + BCC + Manufacturer Data
  Block 1-2: 사용자 데이터 (카드 번호, 만료일, 권한)
  Block 3: Sector Trailer (Key A, Access Bits, Key B)
```

### 6.4 카드 수명 주기

1. **발급**: 카드 인코딩 + 사용자 등록
2. **활성화**: 권한 할당 + 출입 구역 설정
3. **사용**: 출입 인증
4. **일시 정지**: 분실 신고 시
5. **재활성화**: 카드 회수 후
6. **폐기**: 퇴사 또는 만료 시 비활성화

---

## 7. 생체인식 시스템

### 7.1 지문 인식

**기술 방식**:
- **광학식**: LED + 카메라, 저렴, 건식 지문에 약함
- **정전식**: 전기장 감지, 중간 비용, 습식 지문에 강함
- **초음파식**: 음파 반사, 고비용, 위조 방지 우수

**성능 지표**:
- FAR: ≤ 0.001% (1/100,000)
- FRR: ≤ 1%
- 인식 시간: ≤ 1초
- 템플릿 크기: 256-512 바이트

**보안 기능**:
- Liveness Detection (활성 감지)
- 지문 위조 방지 (실리콘, 젤라틴 감지)
- 템플릿 암호화 (AES-256)

### 7.2 얼굴 인식

**기술 방식**:
- **2D 얼굴 인식**: RGB 카메라, 저렴, 사진 공격 취약
- **3D 얼굴 인식**: Depth 카메라 (ToF, Structured Light), 고비용, 위조 방지
- **열화상 얼굴 인식**: 체온 측정 가능, 마스크 착용 시 유리

**성능 지표**:
- FAR: ≤ 0.01% (1/10,000)
- FRR: ≤ 3%
- 인식 거리: 0.5-2m
- 인식 시간: ≤ 2초
- 다중 얼굴 동시 인식: 최대 10명

**활성 감지**:
- 눈 깜빡임 감지
- 입 움직임 감지
- 3D Depth 분석
- 적외선 (IR) 반사 분석

### 7.3 홍채 인식

**특징**:
- 가장 높은 정확도
- 평생 변하지 않음
- 쌍둥이도 구분 가능

**성능 지표**:
- FAR: ≤ 0.0001% (1/1,000,000)
- FRR: ≤ 0.5%
- 인식 거리: 30-40cm
- 인식 시간: ≤ 2초

**단점**:
- 고비용
- 안경, 렌즈 영향
- 사용자 저항감

### 7.4 정맥 인식

**기술 방식**:
- **손가락 정맥**: 적외선 투과, 내부 혈관 패턴 인식
- **손바닥 정맥**: 넓은 인식 영역, 높은 정확도

**장점**:
- 위조 불가능 (체내 혈관)
- 비접촉 또는 최소 접촉
- 높은 위생성

**성능 지표**:
- FAR: ≤ 0.0001%
- FRR: ≤ 0.01%
- 인식 시간: ≤ 1초

---

## 8. PIN 및 비밀번호 인증

### 8.1 PIN 코드

**구성**:
- 4-8자리 숫자
- 키패드 입력
- 화면 가림 기능

**보안 정책**:
```json
{
  "pinPolicy": {
    "minLength": 4,
    "maxLength": 8,
    "allowSequential": false,
    "allowRepeating": false,
    "expirationDays": 90,
    "failureLimit": 3,
    "lockoutDuration": 300
  }
}
```

**사용 예**:
- 단독 인증: 저보안 구역
- 2차 인증: 카드 + PIN (고보안 구역)

### 8.2 비밀번호

**터치스크린 키패드**:
- 영문 + 숫자 + 특수문자
- 최소 8자리
- 대소문자 구분

**보안 강화**:
- bcrypt 해싱 (cost factor 12)
- Salt 추가
- 평문 저장 금지
- HTTPS 전송

---

## 9. 방문자 관리 시스템

### 9.1 방문자 유형

- **사전 등록 방문자**: 직원이 미리 등록
- **현장 방문자**: 접수 데스크에서 등록
- **정기 방문자**: 외주 업체, 정기 배송
- **VIP 방문자**: 임원 방문객, 우대 처리

### 9.2 방문 프로세스

```
1. 사전 등록 (직원이 방문자 정보 입력)
   ↓
2. 초대장 발송 (이메일/SMS로 QR 코드 발급)
   ↓
3. 체크인 (방문자가 QR 코드 스캔 또는 신분증 제시)
   ↓
4. 임시 출입증 발급 (방문증 인쇄 또는 모바일 크리덴셜)
   ↓
5. 출입 (지정된 구역만 접근 가능)
   ↓
6. 체크아웃 (퇴실 시 방문증 반납)
```

### 9.3 방문자 데이터 형식

```json
{
  "visitorId": "VIS-20251225-001",
  "name": "홍길동",
  "company": "ABC 주식회사",
  "phone": "+82-10-1234-5678",
  "email": "hong@abc.com",
  "purpose": "업무 미팅",
  "host": {
    "employeeId": "EMP-001",
    "name": "김철수",
    "department": "영업부"
  },
  "visitDate": "2025-12-25",
  "timeSlot": {
    "start": "2025-12-25T14:00:00+09:00",
    "end": "2025-12-25T16:00:00+09:00"
  },
  "accessZones": ["LOBBY", "MEETING-ROOM-A"],
  "credential": {
    "type": "qr-code",
    "code": "QR-VIS-20251225-001-A3F8"
  }
}
```

### 9.4 QR 코드 출입증

**QR 코드 생성**:
```
방문자 ID + 유효 시간 + 접근 구역 + HMAC 서명
```

**보안**:
- 일회용 (Single-use)
- 시간 제한 (Time-limited)
- HMAC-SHA256 서명

---

## 10. 주차장 출입 통제

### 10.1 차량 인식 시스템

**LPR (License Plate Recognition)**:
- 카메라: 1080p 이상, 야간 IR
- OCR 엔진: 한글, 영문, 숫자 인식
- 인식률: ≥ 98% (주간), ≥ 95% (야간)
- 처리 시간: ≤ 1초

**RFID 태그**:
- 주파수: 2.45GHz (능동형), 860-960MHz (수동형)
- 읽기 거리: 5-10m
- 차량 부착 위치: 앞유리 상단

### 10.2 주차 관리

**정기권 관리**:
```json
{
  "parkingPassId": "PP-001",
  "vehicleNumber": "12가3456",
  "ownerName": "김철수",
  "passType": "monthly",
  "validFrom": "2025-12-01",
  "validUntil": "2025-12-31",
  "assignedSlot": "B1-A-15",
  "status": "active"
}
```

**요금 자동 정산**:
- 입차 시간 기록
- 출차 시 요금 계산
- 무인 정산기 또는 앱 결제
- 영수증 자동 발행

### 10.3 주차 현황 모니터링

```json
{
  "parkingLotId": "PL-001",
  "totalSlots": 200,
  "occupiedSlots": 135,
  "availableSlots": 65,
  "occupancyRate": 67.5,
  "floors": [
    {
      "floor": "B1",
      "totalSlots": 100,
      "occupiedSlots": 80
    },
    {
      "floor": "B2",
      "totalSlots": 100,
      "occupiedSlots": 55
    }
  ]
}
```

---

## 11. 엘리베이터 층별 접근 제어

### 11.1 시스템 연동

**엘리베이터 제어 방식**:
- 카드 리더 통합형 (엘리베이터 내부 또는 로비)
- 출입 시스템 연동형 (API 연동)
- 목적층 등록 시스템 (DRS, Destination Registration System)

### 11.2 접근 권한 설정

```json
{
  "userId": "EMP-001",
  "name": "김철수",
  "department": "IT팀",
  "allowedFloors": [1, 2, 3, 5, 10, 15],
  "timeRestrictions": {
    "weekdays": {
      "start": "06:00",
      "end": "22:00"
    },
    "weekends": {
      "start": "08:00",
      "end": "18:00"
    }
  }
}
```

### 11.3 엘리베이터 호출 제어

**프로세스**:
1. 사용자가 로비 리더에 카드 태그
2. 시스템이 권한 확인
3. 허용된 층 목록 표시
4. 사용자가 목적층 선택
5. 엘리베이터 자동 호출
6. 엘리베이터 내 목적층 버튼 자동 활성화

---

## 12. 출입 로그 및 감사

### 12.1 출입 로그 형식

```json
{
  "logId": "LOG-20251225-123456",
  "timestamp": "2025-12-25T09:15:32+09:00",
  "accessPointId": "AP-MAIN-ENTRANCE",
  "accessPointName": "본관 정문",
  "userId": "EMP-001",
  "userName": "김철수",
  "credentialType": "rfid-card",
  "credentialId": "CARD-001",
  "action": "entry",
  "result": "granted",
  "reason": null,
  "doorOpenDuration": 3.5,
  "photoCapture": "/logs/photos/20251225-123456.jpg"
}
```

### 12.2 로그 보존 정책

| 로그 유형 | 보존 기간 | 백업 주기 |
|----------|----------|----------|
| 일반 출입 로그 | 3년 | 일일 |
| 거부된 출입 시도 | 5년 | 일일 |
| 비상 출입 로그 | 영구 | 실시간 |
| 감사 로그 (시스템 변경) | 10년 | 실시간 |

### 12.3 감사 리포트

**일일 리포트**:
- 총 출입 횟수
- 거부된 출입 시도
- 시간외 출입
- 미퇴실 인원

**월간 리포트**:
- 출입 패턴 분석
- 구역별 이용 현황
- 이상 징후 탐지
- 컴플라이언스 리포트

### 12.4 실시간 모니터링

```json
{
  "buildingId": "BLD-001",
  "timestamp": "2025-12-25T15:00:00+09:00",
  "currentOccupancy": 450,
  "maxOccupancy": 800,
  "occupancyRate": 56.25,
  "recentEvents": [
    {
      "timestamp": "2025-12-25T14:59:45+09:00",
      "event": "unauthorized-attempt",
      "location": "서버실 출입구",
      "userId": "VIS-001",
      "photoUrl": "/alerts/20251225-145945.jpg"
    }
  ],
  "alerts": [
    {
      "alertId": "ALT-001",
      "type": "tailgating-detected",
      "location": "주차장 출입구",
      "timestamp": "2025-12-25T14:58:12+09:00"
    }
  ]
}
```

---

## 13. 비상시 출입 통제

### 13.1 비상 모드 유형

**화재 비상**:
- 모든 도어 자동 개방 (Free Egress)
- 엘리베이터 1층 강제 이동
- 비상구 LED 점등

**보안 비상 (침입, 테러)**:
- 모든 도어 자동 폐쇄 (Lockdown)
- 외부 출입구 차단
- 경찰 자동 통보

**재난 비상 (지진, 홍수)**:
- 상황에 따라 개방 또는 폐쇄
- 안전 구역 접근 허용
- 대피 경로 안내

### 13.2 비상 모드 활성화

**자동 활성화**:
- 화재 감지기 연동
- 침입 감지 센서 연동
- 재난 방송 시스템 연동

**수동 활성화**:
- 비상 버튼 (Emergency Push Button)
- 관리자 콘솔
- 모바일 앱

### 13.3 비상 대응 시나리오

```json
{
  "emergencyType": "fire",
  "actions": [
    {
      "action": "unlock-all-doors",
      "priority": 1,
      "duration": "until-cleared"
    },
    {
      "action": "disable-elevators",
      "priority": 2,
      "exceptions": ["ELEVATOR-FIRE-SERVICE"]
    },
    {
      "action": "activate-emergency-lighting",
      "priority": 3
    },
    {
      "action": "sound-evacuation-alarm",
      "priority": 4
    },
    {
      "action": "notify-fire-department",
      "priority": 5,
      "autoCall": true
    }
  ]
}
```

---

## 14. 데이터 형식

### 14.1 사용자 정보

```json
{
  "userId": "EMP-001",
  "employeeId": "2025-001",
  "name": "김철수",
  "department": "IT팀",
  "position": "시니어 엔지니어",
  "email": "kim@company.com",
  "phone": "+82-10-1234-5678",
  "credentials": [
    {
      "credentialId": "CARD-001",
      "type": "rfid-card",
      "cardNumber": "1234567890",
      "facilityCode": "100",
      "validFrom": "2025-01-01",
      "validUntil": "2025-12-31",
      "status": "active"
    },
    {
      "credentialId": "BIO-FINGERPRINT-001",
      "type": "fingerprint",
      "templateHash": "SHA256:a3f8b2...",
      "enrolledDate": "2025-01-15",
      "status": "active"
    }
  ],
  "accessLevel": "employee-level-2",
  "accessZones": ["LOBBY", "OFFICE-FLOOR-5", "PARKING-B1"],
  "schedule": {
    "weekdays": {
      "start": "07:00",
      "end": "20:00"
    },
    "weekends": false
  }
}
```

### 14.2 출입구 정보

```json
{
  "accessPointId": "AP-MAIN-ENTRANCE",
  "name": "본관 정문",
  "location": {
    "building": "본관",
    "floor": "1F",
    "zone": "LOBBY"
  },
  "type": "main-entrance",
  "devices": {
    "reader": {
      "readerId": "RDR-001",
      "type": "rfid-nfc-combo",
      "model": "HID iCLASS SE R40",
      "ipAddress": "192.168.1.101",
      "status": "online"
    },
    "lock": {
      "lockId": "LOCK-001",
      "type": "electromagnetic-lock",
      "holdingForce": "600lbs",
      "status": "locked"
    },
    "doorSensor": {
      "sensorId": "SENSOR-001",
      "type": "magnetic-contact",
      "status": "closed"
    },
    "camera": {
      "cameraId": "CAM-001",
      "resolution": "1080p",
      "recordingEnabled": true
    }
  },
  "accessControl": {
    "normallyOpen": false,
    "unlockDuration": 5,
    "antiPassback": true,
    "twoPersonRule": false
  }
}
```

### 14.3 접근 권한 정책

```json
{
  "accessLevelId": "employee-level-2",
  "name": "일반 직원 (Level 2)",
  "description": "일반 직원 접근 권한",
  "allowedZones": [
    "LOBBY",
    "OFFICE-FLOOR-5",
    "MEETING-ROOMS",
    "CAFETERIA",
    "PARKING-B1"
  ],
  "deniedZones": [
    "SERVER-ROOM",
    "EXECUTIVE-FLOOR",
    "R&D-LAB"
  ],
  "timeRestrictions": {
    "weekdays": {
      "start": "06:00",
      "end": "22:00"
    },
    "saturday": {
      "start": "09:00",
      "end": "18:00"
    },
    "sunday": false,
    "holidays": false
  },
  "specialRules": {
    "requireEscort": ["DATA-CENTER"],
    "requireApproval": ["WAREHOUSE"]
  }
}
```

---

## 15. API 명세

### 15.1 인증

```bash
# API Key 발급
POST /api/v1/auth/register
{
  "organizationName": "ABC 주식회사",
  "email": "admin@abc.com",
  "buildingId": "BLD-001"
}

# 헤더
Authorization: Bearer {apiKey}
Content-Type: application/json
X-WIA-Standard: CITY-015
X-WIA-Version: 1.0.0
```

### 15.2 주요 엔드포인트

| 엔드포인트 | 메서드 | 설명 |
|-----------|--------|------|
| `/api/v1/users` | GET | 사용자 목록 조회 |
| `/api/v1/users` | POST | 사용자 등록 |
| `/api/v1/users/{id}` | GET | 사용자 상세 조회 |
| `/api/v1/users/{id}` | PUT | 사용자 정보 수정 |
| `/api/v1/users/{id}` | DELETE | 사용자 삭제 |
| `/api/v1/credentials` | POST | 크리덴셜 발급 |
| `/api/v1/credentials/{id}` | DELETE | 크리덴셜 폐기 |
| `/api/v1/access-points` | GET | 출입구 목록 |
| `/api/v1/access-points/{id}` | GET | 출입구 상세 |
| `/api/v1/access-points/{id}/unlock` | POST | 출입구 원격 개방 |
| `/api/v1/access-logs` | GET | 출입 로그 조회 |
| `/api/v1/visitors` | POST | 방문자 사전 등록 |
| `/api/v1/visitors/{id}` | GET | 방문자 정보 조회 |
| `/api/v1/visitors/{id}/check-in` | POST | 방문자 체크인 |
| `/api/v1/visitors/{id}/check-out` | POST | 방문자 체크아웃 |
| `/api/v1/parking/vehicles` | GET | 차량 목록 |
| `/api/v1/parking/entry` | POST | 입차 기록 |
| `/api/v1/parking/exit` | POST | 출차 기록 |
| `/api/v1/emergency/activate` | POST | 비상 모드 활성화 |
| `/api/v1/emergency/deactivate` | POST | 비상 모드 해제 |

### 15.3 예시: 출입 권한 부여

```bash
POST /api/v1/users/EMP-001/grant-access
{
  "accessZones": ["SERVER-ROOM"],
  "validFrom": "2025-12-25T00:00:00+09:00",
  "validUntil": "2025-12-31T23:59:59+09:00",
  "requireEscort": false,
  "timeRestrictions": {
    "weekdays": {
      "start": "09:00",
      "end": "18:00"
    }
  }
}
```

---

## 16. 보안 요구사항

### 16.1 데이터 보안

- **전송 암호화**: TLS 1.3 이상
- **저장 암호화**: AES-256
- **생체 템플릿**: 원본 저장 금지, 해시/암호화된 템플릿만 저장
- **개인정보**: GDPR, 개인정보보호법 준수

### 16.2 물리적 보안

- **컨트롤러**: 방화/방수 케이스, 잠금 장치
- **배선**: 은폐 배선, 차단 방지
- **리더기**: 변조 방지 센서 (Tamper Switch)

### 16.3 네트워크 보안

- **세그먼테이션**: OT/IT 네트워크 분리
- **방화벽**: 출입 시스템 전용 VLAN
- **침입 탐지**: IDS/IPS
- **접근 제어**: MAC 주소 필터링

### 16.4 감사 및 컴플라이언스

- **로그 무결성**: 로그 변조 방지 (WORM 스토리지)
- **백업**: 오프사이트 백업
- **정기 감사**: 분기별 보안 감사
- **침투 테스트**: 연 1회

---

## 17. 적합성 평가

### 17.1 인증 요구사항

WIA-CITY-015 표준 준수를 위한 필수 요구사항:

**기본 요구사항**:
- [ ] 2가지 이상의 인증 방법 지원
- [ ] 출입 로그 최소 3년 보존
- [ ] 비상 모드 지원
- [ ] API 제공
- [ ] 생체정보 암호화 저장

**고급 요구사항**:
- [ ] 3가지 이상의 인증 방법 지원
- [ ] 다중 인증 (MFA) 지원
- [ ] 방문자 관리 시스템
- [ ] 주차장 연동
- [ ] 엘리베이터 제어
- [ ] 실시간 모니터링 대시보드

### 17.2 성능 기준

| 지표 | 목표 |
|------|------|
| 인증 응답 시간 | ≤ 1초 |
| 도어 개방 시간 | ≤ 2초 (인증 후) |
| 시스템 가용성 | ≥ 99.9% |
| 로그 저장 성공률 | ≥ 99.99% |
| 생체인식 FAR | ≤ 0.01% |
| 생체인식 FRR | ≤ 3% |

### 17.3 인증 절차

1. **서류 심사**: 시스템 사양서 제출
2. **기능 시험**: 실제 환경 테스트
3. **보안 감사**: 취약점 점검
4. **성능 평가**: 벤치마크 테스트
5. **인증서 발급**: WIA-CITY-015 인증 마크 부여

---

## 18. 참조 표준

- ISO/IEC 19794: 생체정보 데이터 교환 형식
- ISO/IEC 24745: 생체정보 템플릿 보호
- ANSI/INCITS 378: 지문 템플릿 형식
- Wiegand Interface Standard
- OSDP (Open Supervised Device Protocol) v2.2
- ONVIF (Open Network Video Interface Forum)
- BACnet (ISO 16484-5): 빌딩 자동화 연동
- GDPR (General Data Protection Regulation)
- 개인정보보호법 (한국)

---

## 19. 용례 (Use Cases)

### 19.1 오피스 빌딩

**규모**: 25층, 2,000명 재직

**시스템 구성**:
- 정문: 얼굴 인식 + 카드 리더
- 주차장: 차량 번호 인식
- 엘리베이터: 층별 접근 제어
- 서버실: 카드 + 생체인식 (2-Factor)

**효과**:
- 무인 보안 체계 구축
- 비인가 출입 99% 차단
- 방문자 관리 자동화

### 19.2 데이터센터

**보안 등급**: Tier 3

**시스템 구성**:
- Mantrap (이중 도어)
- 카드 + 생체인식 + PIN (3-Factor)
- Anti-Passback
- 24/7 CCTV 녹화

**효과**:
- 물리적 보안 ISO 27001 인증 취득
- 출입 감사 완벽 추적

### 19.3 병원

**특수 요구사항**:
- 격리 병동 출입 통제
- 약품 보관소 접근 제어
- 신생아실 보안

**시스템 구성**:
- RFID 팔찌 (환자)
- 카드 + PIN (의료진)
- 비상 개방 (화재 시)

**효과**:
- 환자 안전 향상
- 약품 도난 방지
- 감염 통제

---

**발행 기관**: WIA (World Certification Industry Association)
**라이선스**: MIT License
**문의**: standards@wia.org
**홈페이지**: https://wia.org/standards/city-015

**弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**
