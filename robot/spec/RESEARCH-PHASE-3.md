# Phase 3 사전 조사 결과

**WIA Robot Standard - Communication Protocol Research**

---

## 1. 로봇 통신 프로토콜 비교

### ROS2 DDS (Data Distribution Service)

**개요**
- ROS2는 DDS(Data Distribution Service)를 기본 미들웨어로 사용
- OMG(Object Management Group)에서 발행한 산업 표준
- 분산 실시간 데이터 중심 Pub/Sub 통신 제공

**특징**
- **Quality of Service (QoS)**: 신뢰성, 지속성, 데드라인, 라이브니스 설정
- **분산 탐색**: ROS1과 달리 중앙 마스터 노드 불필요
- **실시간 지원**: Real-time capable 통신
- **크로스 플랫폼**: 벤더 간 상호운용성

**DDS 구현체**
| 구현체 | 특징 |
|--------|------|
| Fast DDS (eProsima) | ROS2 기본, 범용적 |
| Cyclone DDS (Eclipse) | 성능 우수 |
| RTI Connext DDS | 상용, 엔터프라이즈 기능 |
| GurumDDS | 경량 구현 |

**WIA Robot 적용 분석**
- ✅ 로봇 간 실시간 통신에 최적
- ✅ QoS 설정으로 안전-critical 메시지 우선순위 지정
- ✅ 기존 ROS2 로봇과 네이티브 호환
- ⚠️ 학습 곡선 존재
- ⚠️ 임베디드 환경에서 상대적으로 무거움

---

### MQTT (Message Queuing Telemetry Transport)

**개요**
- OASIS 표준 IoT 메시징 프로토콜
- 경량 Publish/Subscribe 메시징 전송
- 작은 코드 풋프린트, 최소 네트워크 대역폭

**특징**
- **경량 설계**: 소형 마이크로컨트롤러에서 실행 가능
- **확장성**: 수백만 IoT 디바이스 연결 가능
- **QoS 레벨**: 0 (at most once), 1 (at least once), 2 (exactly once)
- **보안**: TLS 암호화, OAuth 인증 지원

**의료 기기 적용**
- 원격 환자 모니터링 시스템에서 바이탈 신호 전송
- 의료 실험실 장비 연결
- EHR(전자건강기록) 플랫폼과 통합

**WIA Robot 적용 분석**
- ✅ 원격 모니터링 및 텔레메트리에 적합
- ✅ 클라우드 기반 로봇 관리에 이상적
- ✅ 의료 IoT 생태계와 호환
- ⚠️ 브로커 의존성 (중앙 집중)
- ⚠️ 초저지연 제어에는 부적합

**참고**
- [MQTT.org](https://mqtt.org/)
- [EMQ Healthcare IoT](https://www.emqx.com/en/blog/iot-in-healthcare-connecting-medical-lab-devices-with-mqtt)

---

### WebSocket

**개요**
- 단일 TCP 연결을 통한 전이중(Full-duplex) 통신 프로토콜
- 실시간 양방향 데이터 전송에 최적화

**특징**
- **저지연**: HTTP 폴링 대비 10배 이상 성능 향상
- **지속 연결**: 연결 설정 오버헤드 제거
- **양방향**: 서버-클라이언트 동시 통신
- **웹 친화적**: 브라우저 네이티브 지원

**성능 비교**
| 환경 | 왕복 시간 (RTT) |
|------|----------------|
| 로컬 | 43.8ms |
| 클라우드 (AWS) | 87ms |

**로봇 원격 조작 연구**
- HTTP/TCP 기반 원격 조작 프로토콜의 대체제로 연구됨
- 다수 사용자의 실시간 텔레오퍼레이션에 적합
- 1-2바이트의 제어 명령 전송 시 HTTP 대비 현저한 오버헤드 감소

**WIA Robot 적용 분석**
- ✅ 웹 기반 로봇 제어 인터페이스에 최적
- ✅ 실시간 텔레메트리 대시보드
- ✅ 크로스 플랫폼 호환성
- ⚠️ 초고속 실시간 제어(< 10ms)에는 전용 프로토콜 필요
- ⚠️ 네트워크 불안정 시 재연결 로직 필요

**참고**
- [WebSocket Robot Teleoperation Analysis](https://www.sciencedirect.com/science/article/pii/S1474667015343688)
- [Industrial Protocol Latency Analysis](https://ijettjournal.org/Volume-73/Issue-1/IJETT-V73I1P110.pdf)

---

## 2. 의료 기기 통신 표준

### HL7 FHIR (Fast Healthcare Interoperability Resources)

**개요**
- HL7이 개발한 최신 의료 데이터 상호운용성 표준
- 현대 웹 기술과 의료 정보 기술 결합
- RESTful API 기반 아키텍처

**Device Resource**
- 의료/비의료 기기 모두 커버
- 혀 누르개부터 MRI까지 유연한 범위
- 디바이스 유형, 제조사, 상태 정보 표준화

**주요 리소스**
| 리소스 | 설명 |
|--------|------|
| Device | 의료 기기 정보 |
| DeviceComponent | 기기 구성 요소 |
| DeviceMetric | 기기 측정 데이터 |
| Observation | 관측 데이터 |

**로봇 통합**
- HL7 인터페이스를 통한 로봇 시스템과 약물 처방 연동
- 자동화된 약물 조제 시스템과의 데이터 교환
- 병원 정보 시스템(HIS)과의 상호운용성

**WIA Robot 적용 분석**
- ✅ 돌봄 로봇의 바이탈 데이터를 EHR에 전송
- ✅ 재활 로봇의 치료 데이터 기록
- ✅ 의료 기관과의 표준 통합
- ⚠️ 실시간 제어용이 아닌 데이터 교환용
- ⚠️ 의료 컨텍스트에 특화

**참고**
- [HL7 FHIR Device](https://build.fhir.org/device.html)
- [HL7 Devices Work Group](https://www.hl7.org/Special/committees/healthcaredevices/overview.cfm)

---

### IEEE 11073 (Personal Health Device Communication)

**개요**
- 의료/건강/웰니스 기기와 외부 컴퓨터 시스템 간 통신 표준
- 클라이언트 관련 및 바이탈 신호 정보의 자동 전자 데이터 캡처
- ISO, FDA에서 공인된 표준

**특징**
- **실시간 Plug-and-Play**: 다중 기기 데이터를 밀리초 내에 검색, 상관, 표시
- **자동 설정**: 연결만 하면 시스템이 자동으로 감지, 구성, 통신
- **완전한 연결 솔루션**: 물리적 연결부터 추상 정보 표현까지

**표준 구성**
| 표준 번호 | 내용 |
|-----------|------|
| 11073-10101 | 명명법 (Nomenclature) |
| 11073-20601 | 최적화된 교환 프로토콜 |
| 11073-104xx | 디바이스 전문화 시리즈 |

**디바이스 전문화 예시**
- 혈압 모니터 (10407)
- 혈당 모니터 (10417)
- ECG 장치 (10406)
- 연속 혈당 모니터 CGM (10425)
- 체성분 분석기 (10420)
- 심혈관 피트니스 모니터 (10441)

**WIA Robot 적용 분석**
- ✅ 의수/의족의 EMG 센서 데이터 표준화
- ✅ 돌봄 로봇의 바이탈 모니터링 데이터 형식
- ✅ FDA 인증 의료 기기와의 호환성
- ⚠️ 개인 건강 기기에 특화, 산업 로봇에는 추가 확장 필요

**참고**
- [IEEE 11073 Standards Committee](https://sagroups.ieee.org/11073/phd-wg/)
- [ISO/IEEE 11073-20601:2022](https://www.iso.org/standard/84781.html)

---

### ISO 13482 (Personal Care Robot Safety)

**개요**
- 개인 돌봄 로봇의 안전 요구사항 표준
- 인간과 가까이 상호작용하는 로봇의 안전 보장
- 일상 생활 관련 활동 지원 로봇 대상

**적용 범위**
- **이동 보조 로봇** (Mobile servant robots)
- **물리적 보조 로봇** (Physical assistant robots)
- **사람 운반 로봇** (Person carrier robots)

**주요 요구사항**
- 설계 및 제작 요구사항
- 제공 정보 요구사항
- 안전 요구사항 준수 테스트 방법
- 물리적/심리적 해악 방지

**핵심 안전 관심사**
- 안정성 (Stability)
- 충돌 회피 (Collision avoidance)
- 비제어 환경에서의 Fail-safe 동작

**WIA Robot 적용 분석**
- ✅ 외골격, 휠체어 등 개인 돌봄 로봇의 안전 기준
- ✅ 긴급 정지 및 안전 프로토콜 설계 가이드
- ✅ 가정/병원 환경 로봇의 안전 인증 기반
- ⚠️ 통신 프로토콜 자체보다 안전 요구사항에 초점

**참고**
- [ISO 13482:2014](https://www.iso.org/standard/53820.html)
- [ISO/TR 23482-2:2019 Application Guidelines](https://www.iso.org/standard/71627.html)

---

## 3. 결론 및 권장 사항

### 권장 프로토콜 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                    WIA Robot Protocol (WRP)                  │
├─────────────────────────────────────────────────────────────┤
│  Application Layer                                           │
│  ┌─────────────┬─────────────┬─────────────┬─────────────┐  │
│  │ Telemetry   │ Control     │ Safety      │ Config      │  │
│  │ Messages    │ Commands    │ Alerts      │ Updates     │  │
│  └─────────────┴─────────────┴─────────────┴─────────────┘  │
├─────────────────────────────────────────────────────────────┤
│  Transport Abstraction Layer                                 │
│  ┌─────────────┬─────────────┬─────────────┬─────────────┐  │
│  │ WebSocket   │ MQTT        │ ROS2 DDS    │ Custom      │  │
│  └─────────────┴─────────────┴─────────────┴─────────────┘  │
├─────────────────────────────────────────────────────────────┤
│  Healthcare Integration Layer (Optional)                     │
│  ┌─────────────────────────────────────────┐                │
│  │ HL7 FHIR Bridge    │ IEEE 11073 Bridge  │                │
│  └─────────────────────────────────────────┘                │
└─────────────────────────────────────────────────────────────┘
```

### 메시지 형식 설계 방향

1. **단일 메시지 형식**: Phase 1 Data Format을 payload로 포함
2. **우선순위 기반**: Emergency > Critical > High > Normal > Low
3. **안전 필드 필수**: 모든 메시지에 safety 정보 포함
4. **체크섬 검증**: CRC32 또는 SHA256
5. **타임스탬프**: 밀리초 정밀도의 UTC 시간

### 전송 계층 전략

| 사용 사례 | 권장 전송 | 이유 |
|----------|----------|------|
| 실시간 제어 | ROS2 DDS | QoS, 저지연 |
| 웹 대시보드 | WebSocket | 브라우저 호환 |
| 클라우드 텔레메트리 | MQTT | 확장성, IoT 호환 |
| 의료 기록 연동 | HTTP/REST + FHIR | 의료 표준 호환 |

### ROS2 호환 전략

1. **네이티브 ROS2 지원**: ROS2 DDS 전송 계층 제공
2. **메시지 매핑**: WRP 메시지 ↔ ROS2 메시지 변환
3. **QoS 매핑**: WRP 우선순위 → DDS QoS 정책
4. **토픽 명명**: `/wia/robot/{device_type}/{device_id}/{message_type}`

### 안전 프로토콜 요구사항

1. **긴급 정지 (E-Stop)**: 최우선 처리, 브로드캐스트, ACK 필수
2. **하트비트**: 연결 상태 모니터링, 타임아웃 시 안전 모드
3. **안전 레벨**: Normal → Warning → Caution → Critical → Emergency
4. **Watchdog**: 통신 실패 시 자동 안전 상태 전환
5. **메시지 검증**: 체크섬 실패 시 메시지 거부

---

## 참고문헌

1. [ROS2 Documentation](https://docs.ros.org/)
2. [DDS Specification - OMG](https://www.omg.org/spec/DDS/)
3. [MQTT Protocol](https://mqtt.org/)
4. [HL7 FHIR](https://www.hl7.org/fhir/)
5. [IEEE 11073 PHD](https://sagroups.ieee.org/11073/phd-wg/)
6. [ISO 13482:2014](https://www.iso.org/standard/53820.html)
7. [WebSocket Teleoperation Research](https://www.sciencedirect.com/science/article/pii/S1474667015343688)
8. [AWS MQTT Guide](https://aws.amazon.com/what-is/mqtt/)

---

*연구 완료일: 2025-01-15*
*WIA Robot Standard Phase 3*
