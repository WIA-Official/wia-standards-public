# Phase 3 사전 조사 결과: 우주 통신 프로토콜

## 1. 표준 통신 프로토콜

### CCSDS (Consultative Committee for Space Data Systems)

28개국 우주 기관이 참여하는 국제 표준화 기구로, 1,000개 이상의 우주 미션에서 채택됨.

**주요 표준:**
| 표준 | 설명 | 용도 |
|-----|------|-----|
| **Space Packet Protocol** | 우주 애플리케이션 데이터 전송 | 페이로드 데이터 |
| **USLP (Unified Space Data Link Protocol)** | 통합 데이터 링크 | 상호운용성 |
| **CFDP (CCSDS File Delivery Protocol)** | 자율 파일 전송 | 대용량 데이터 |
| **TM/TC (Telemetry/Telecommand)** | 원격 측정/명령 | 기본 통신 |
| **SLE (Space Link Extension)** | 지상 데이터 전달 | DSN 연동 |

**장점:**
- 국제 표준으로 상호운용성 보장
- 1,000+ 미션에서 검증됨
- NASA, ESA, JAXA 등 모든 주요 우주 기관 지원

**단점:**
- 복잡한 프로토콜 스택
- 학습 곡선이 높음
- 레거시 호환성 유지로 인한 오버헤드

### NASA Deep Space Network (DSN)

**통신 인터페이스:**
- **810-005 Handbook**: 원격통신 링크 설계
- **810-007 Handbook**: 지상 데이터 및 관리 인터페이스
- **820-13 Series**: 외부 인터페이스 사양

**주파수 대역:**
| 대역 | 주파수 | 용도 |
|-----|-------|-----|
| S-band | 2-4 GHz | 레거시, 근거리 |
| X-band | 8-12 GHz | 심우주 표준 |
| Ka-band | 26-40 GHz | 고속 데이터 |

**프로토콜:**
- 지상 네트워크: TCP/IP 기반
- 우주 링크: CCSDS 표준
- 데이터 전달: SLE (Space Link Extension)

### DTN (Delay-Tolerant Networking)

행성간 통신의 지연 및 단절 문제를 해결하기 위한 프로토콜.

**핵심 개념:**
- Store-and-forward 방식
- Bundle Protocol (RFC 9171)
- 대용량 번들 (100KB - 1MB+)

**구현체:**
| 이름 | 특징 |
|-----|------|
| **ION (Interplanetary Overlay Network)** | NASA JPL, C언어, 우주용 |
| **HDTN (High-Rate DTN)** | NASA Glenn, 고속 전송 |

**성과:**
- 2016: ISS에서 DTN 서비스 운영 시작
- 2024: PACE 미션에서 3,400만+ 번들 전송 (성공률 100%)
- LunaNet: 달 네트워크 표준으로 채택

**장점:**
- 행성간 지연 처리
- 링크 단절 복원력
- 자동 재전송

**단점:**
- 실시간 통신 불가
- 복잡한 라우팅
- 지연 시간 예측 필요

---

## 2. 텔레메트리 및 텔레커맨드

### 데이터 스트리밍 방식

**Time-Division Multiplexing (TDM):**
- 고정 길이 프레임
- 사전 정의된 다중화 규칙
- 레거시 표준

**Virtual Channels:**
- 데이터 스트림 분리
- 실시간 채널 우선순위
- 다중 페이로드 지원

### 실시간 텔레메트리

**NASA ISS 사례:**
- Lightstreamer 사용 (2010년~)
- 웹 브라우저/모바일 직접 스트리밍
- 전문 장비 불필요

**주요 표준:**
| 표준 | 용도 |
|-----|-----|
| IRIG 106 Chapter 10 | 비행 데이터 기록 |
| AOS (Advanced Orbiting Systems) | 오디오/비디오 데이터 |
| PCM Telemetry | 펄스 코드 변조 |

---

## 3. 지상국 통신

### 아키텍처

```
┌─────────────┐    Radio    ┌─────────────┐    TCP/IP   ┌─────────────┐
│  Spacecraft │◄──────────►│Ground Station│◄──────────►│Mission Ctrl │
└─────────────┘  SHF/EHF    └─────────────┘    NISN     └─────────────┘
```

### TT&C (Telemetry, Tracking, Command)

- **Telemetry**: 우주선 → 지상
- **Tracking**: 위치/속도 추적
- **Command**: 지상 → 우주선

### Ground Station as a Service (GSaaS)

**제공업체:**
| 업체 | 특징 |
|-----|------|
| AWS Ground Station | 클라우드 통합 |
| Leaf Space | LEO/MEO/GEO 지원 |
| KSAT | 극지 커버리지 |

**프로토콜 지원:**
- CCSDS
- CSP (CubeSat Space Protocol)
- AX-25 (아마추어 무선)

---

## 4. 상용 위성 통신 (SpaceX Starlink)

### 네트워크 아키텍처

**구성요소:**
- User Terminal (사용자 단말)
- Satellite Constellation (위성군)
- Ground Stations (지상국)
- PoP (Point of Presence)

**주파수:**
| 링크 | 주파수 | 다중화 |
|-----|-------|-------|
| 업링크 | Ka-band | SC-TDM |
| 다운링크 | Ku-band | SC-FDMA |
| ISL (위성간) | 레이저 | 광통신 |

### 기술적 특징

**변조:**
- OFDM 기반
- 적응형 위상 진폭 변조
- SNR에 따른 동적 조절

**Inter-Satellite Links (ISL):**
- 레이저 통신
- 200 Gbps
- 위성당 3개 레이저

**프로토콜:**
- IPv6보다 간단한 P2P 프로토콜
- 네이티브 E2E 암호화
- 자가 복구 아키텍처

---

## 5. WIA Space Standard 적용 방안

### 권장 프로토콜 스택

```
┌─────────────────────────────────────┐
│         Application Layer           │
│    (WIA Space Mission Data)         │
├─────────────────────────────────────┤
│         Protocol Layer              │
│    (WIA Space Protocol - WSP)       │
├─────────────────────────────────────┤
│         Transport Layer             │
│  WebSocket | TCP | DTN Bundle       │
├─────────────────────────────────────┤
│         Network Layer               │
│      IP | CCSDS Space Packet        │
└─────────────────────────────────────┘
```

### 메시지 형식 설계 방향

1. **JSON 기반**: 가독성, 디버깅 용이
2. **CCSDS 호환**: 바이너리 변환 가능
3. **Phase 1/2 통합**: 기존 스펙과 연동
4. **DTN 지원**: 지연 허용 네트워킹

### 전송 계층 어댑터

| 환경 | 프로토콜 | 용도 |
|-----|---------|-----|
| 시뮬레이션 | WebSocket | 개발/테스트 |
| 지상 네트워크 | TCP/IP | 미션 관제 |
| 심우주 | DTN Bundle | 행성간 통신 |
| 위성간 | UDP/CCSDS | 저지연 통신 |

### 연결 관리

- **지구 근방 (LEO)**: 30분/일 접촉 시간
- **달 궤도**: 최대 14일 지연
- **화성**: 4-24분 지연
- **외행성**: 수시간 지연

---

## 6. 결론

### 핵심 설계 원칙

1. **다중 전송 지원**: WebSocket, TCP, DTN
2. **지연 허용**: 비동기 메시지 처리
3. **재연결 복원력**: 연결 끊김 자동 복구
4. **표준 호환**: CCSDS 메시지 변환 가능

### Phase 3 구현 목표

| 구성요소 | 구현 내용 |
|---------|----------|
| 메시지 형식 | WIA Space Protocol (WSP) |
| 연결 관리 | 상태 머신, 재연결 로직 |
| 에러 처리 | 에러 코드, 복구 전략 |
| 전송 계층 | WebSocket, TCP 어댑터 |
| 지연 시뮬레이션 | 심우주 지연 모델링 |

---

## 참조 문헌

### 국제 표준
- [CCSDS.org](https://ccsds.org/) - Consultative Committee for Space Data Systems
- [RFC 9171](https://www.rfc-editor.org/rfc/rfc9171) - Bundle Protocol Version 7
- [RFC 4838](https://www.rfc-editor.org/rfc/rfc4838) - Delay-Tolerant Networking Architecture

### NASA 문서
- [Deep Space Network](https://deepspace.jpl.nasa.gov/) - DSN 기술 문서
- [NASA DTN](https://www.nasa.gov/communicating-with-missions/delay-disruption-tolerant-networking/) - 지연 허용 네트워킹

### 상용 시스템
- [Starlink Technology](https://starlink.com/technology) - SpaceX 위성 통신
- [Leaf Space](https://leafspace.com/) - GSaaS 제공업체

---

*작성일: 2025-12-14*
*WIA Space Standard Phase 3 Research*
