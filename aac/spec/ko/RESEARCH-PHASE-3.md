# Phase 3 사전 조사 결과
# Communication Protocol Research

---

## 1. 전송 프로토콜 비교 (Transport Protocol Comparison)

### 1.1 WebSocket

**장점 (Advantages):**
- 실시간 양방향 통신 (Real-time bidirectional)
- 브라우저/Node.js 기본 지원
- 낮은 오버헤드 (2-14 bytes per frame)
- JSON/Binary 모두 지원
- 방화벽 친화적 (HTTP 포트 사용)

**단점 (Disadvantages):**
- 네트워크 연결 필요
- 연결 유지 오버헤드

**AAC 적용 분석:**
- 가장 범용적인 AAC 센서 통신 방식
- Tobii, Emotiv 등 주요 제조사가 WebSocket 지원
- 웹 애플리케이션 연동에 최적

**참조:** [High Performance Browser Networking - WebSocket](https://hpbn.co/websocket/)

---

### 1.2 USB HID (Human Interface Device)

**장점 (Advantages):**
- 드라이버 불필요 (OS 기본 지원)
- 낮은 지연 시간 (Low latency)
- 안정적인 연결
- 전원 공급 가능

**단점 (Disadvantages):**
- 물리적 연결 필요
- 복잡한 리포트 디스크립터
- 플랫폼별 구현 차이

**AAC 적용 분석:**
- 스위치, 마우스 대체 장치에 적합
- 레거시 AAC 장치 지원
- 웹 환경에서 직접 접근 어려움 (WebHID API 필요)

**참조:** [USB HID Specification](https://www.usb.org/hid)

---

### 1.3 Bluetooth Low Energy (BLE) / HOGP

**장점 (Advantages):**
- 무선 연결
- 저전력 (Low power consumption)
- 모바일 장치 친화적
- HID over GATT (HOGP) 표준

**단점 (Disadvantages):**
- 지연 시간 변동
- 연결 범위 제한 (~10m)
- 페어링 과정 필요

**AAC 적용 분석:**
- 무선 스위치, 머리 움직임 센서에 적합
- 이동성이 중요한 사용자에게 필수
- HOGP 표준으로 HID 호환성 확보

**참조:** [Bluetooth HID over GATT Profile](https://www.bluetooth.com/specifications/specs/hid-over-gatt-profile-1-0/)

---

### 1.4 Serial (RS-232 / USB-Serial)

**장점 (Advantages):**
- 단순한 프로토콜
- 레거시 장치 지원
- 안정적인 데이터 전송

**단점 (Disadvantages):**
- 느린 전송 속도
- 케이블 연결 필요
- 현대 장치에서 감소 추세

**AAC 적용 분석:**
- 구형 AAC 장치 지원용
- OpenBCI 등 일부 EEG 장치에서 사용
- 신규 개발 비권장

---

## 2. 기존 AAC 제품 통신 방식 분석

### 2.1 Tobii Eye Tracker

**통신 방식:**
- **Tobii Pro Glasses 3**: HTTP REST API + WebSocket + WebRTC
- **Tobii EyeX**: Proprietary SDK with WebSocket wrapper available
- **데이터 스트림**: 60-120 Hz gaze data

**메시지 형식:**
```json
{
  "gazePoint": {
    "x": 0.45,
    "y": 0.32
  },
  "timestamp": 1702483200000,
  "validity": "valid"
}
```

**특징:**
- WebSocket 서버 포트: 8765 (PyEyetracker), 8887 (EyeX WebSocket Server)
- 명령어: `startGazePoint`, `stopGazePoint`, `startEyePosition`, `stopEyePosition`
- 상태 조회: `state` 명령

**참조:** [GitHub - Tobii EyeX Web Socket Server](https://github.com/rezreal/Tobii-EyeX-Web-Socket-Server)

---

### 2.2 OpenBCI (EEG/EMG)

**통신 방식:**
- **WiFi Shield**: TCP/HTTP with JSON or Raw binary
- **Cyton Board**: Serial (115200 baud) or BLE
- **데이터 스트림**: 250 Hz sample rate

**메시지 형식 (JSON Mode):**
```json
{
  "timestamp": 1702483200.123,
  "data": [12.5, 11.8, 15.2, 10.5, 8.7, 9.2, 14.1, 13.8]
}
```

**WiFi Shield API:**
- POST `/tcp` - 스트리밍 설정
- GET `/stream/start` - 스트림 시작
- GET `/stream/stop` - 스트림 종료

**설정 옵션:**
```json
{
  "port": 4202,
  "ip": "192.168.4.2",
  "output": "json",
  "latency": 200000,
  "delimiter": true,
  "timestamps": true
}
```

**참조:** [OpenBCI WiFi Shield API](https://docs.openbci.com/ThirdParty/WiFiShield/WiFiAPI/)

---

### 2.3 Emotiv (EEG/BCI)

**통신 방식:**
- **Cortex API**: WebSocket + JSON-RPC
- **포트**: 6868 (local WebSocket)
- **인증**: Client ID + Client Secret (OAuth-like)

**메시지 형식:**
```json
{
  "jsonrpc": "2.0",
  "method": "subscribe",
  "params": {
    "cortexToken": "xxx",
    "session": "xxx",
    "streams": ["eeg", "pow", "met"]
  },
  "id": 1
}
```

**데이터 스트림 종류:**
| Stream | 설명 |
|--------|------|
| `eeg` | Raw EEG data |
| `mot` | Motion sensors |
| `dev` | Device info |
| `pow` | Band powers |
| `met` | Performance metrics |
| `com` | Mental commands |
| `fac` | Facial expressions |

**참조:** [Emotiv Cortex API](https://emotiv.gitbook.io/cortex-api)

---

### 2.4 NeuroSky (EEG)

**통신 방식:**
- **ThinkGear Connector**: TCP Socket
- **포트**: 13854
- **형식**: JSON

**메시지 형식:**
```json
{
  "eSense": {
    "attention": 75,
    "meditation": 60
  },
  "eegPower": {
    "delta": 1234567,
    "theta": 234567,
    "lowAlpha": 34567,
    "highAlpha": 4567,
    "lowBeta": 567,
    "highBeta": 67,
    "lowGamma": 7,
    "highGamma": 1
  },
  "poorSignalLevel": 0
}
```

---

## 3. 메시지 형식 비교 (Message Format Comparison)

### 3.1 직렬화 형식 (Serialization Formats)

| 형식 | 크기 | 속도 | 가독성 | 스키마 |
|-----|------|-----|-------|-------|
| **JSON** | 큼 | 중간 | 좋음 | 선택적 |
| **MessagePack** | 작음 | 빠름 | 없음 | 없음 |
| **Protocol Buffers** | 가장 작음 | 가장 빠름 | 없음 | 필수 |
| **Custom Binary** | 가변 | 빠름 | 없음 | 필수 |

### 3.2 권장 형식

**JSON 선택 이유:**
1. AAC 소프트웨어 개발자 친화적
2. 디버깅/로깅 용이
3. Phase 1 Signal Format과 일관성
4. 대부분의 AAC 제품이 JSON 사용
5. 스키마 검증 가능 (JSON Schema)

**참조:** [WebSocket Data Formats](https://aditya-sunjava.medium.com/exploring-data-formats-in-websocket-communications-5c47871b5df5)

---

## 4. 공통 패턴 분석 (Common Patterns)

### 4.1 연결 관리
- 모든 프로토콜이 연결/해제 핸드셰이크 사용
- 세션 ID 또는 토큰 기반 인증
- 하트비트/핑-퐁으로 연결 상태 확인

### 4.2 데이터 스트리밍
- 구독 기반 (subscribe/unsubscribe)
- 스트림 시작/종료 명령
- 타임스탬프 필수

### 4.3 에러 처리
- 에러 코드 + 메시지
- 복구 가능/불가능 구분
- 재연결 로직 내장

---

## 5. 결론 및 설계 방향

### 5.1 권장 통신 방식

**Primary: WebSocket**
- 가장 범용적이고 접근성 높음
- 웹/데스크탑/모바일 모두 지원
- 기존 AAC 제품들과 호환

**Secondary: USB HID**
- 하드웨어 직접 연결 시
- 레거시 장치 지원

**Tertiary: Bluetooth LE**
- 무선 AAC 장치 지원
- 모바일 환경

### 5.2 메시지 프로토콜 설계 방향

1. **JSON 기반**: 가독성과 호환성 우선
2. **Phase 1 Signal을 payload로**: 기존 형식 재사용
3. **JSON-RPC 스타일 영감**: Emotiv 방식 참조
4. **메시지 타입 구분**: connect, signal, command, error 등
5. **시퀀스 번호/메시지 ID**: 순서 보장 및 ACK
6. **버전 관리**: 프로토콜 버전 필드 포함

### 5.3 참조 구현

주요 참조:
- Tobii WebSocket Server: 간단한 명령-응답 패턴
- Emotiv Cortex API: 풍부한 JSON-RPC 기반 프로토콜
- OpenBCI WiFi: TCP/JSON 스트리밍
- WebSocket RFC 6455: 표준 프레이밍

---

## 참조 문서 (References)

- [RFC 6455 - WebSocket Protocol](https://datatracker.ietf.org/doc/html/rfc6455)
- [USB HID Specification 1.11](https://www.usb.org/sites/default/files/documents/hid1_11.pdf)
- [Bluetooth HOGP 1.0](https://www.bluetooth.com/specifications/specs/hid-over-gatt-profile-1-0/)
- [Tobii Developer Zone](https://developer.tobii.com/)
- [OpenBCI Documentation](https://docs.openbci.com/)
- [Emotiv Cortex API](https://emotiv.gitbook.io/cortex-api)
