# WIA BCI Research Report - Phase 3

**Communication Protocol Research**

---

## 1. Overview

Phase 3 사전 조사로서 BCI 통신 프로토콜에 대한 기술 조사를 수행했습니다. 본 문서는 다양한 전송 프로토콜과 기존 BCI 시스템의 통신 방식을 분석합니다.

---

## 2. Streaming Protocol Comparison

### 2.1 Lab Streaming Layer (LSL)

**개요**: LSL은 연구 실험에서 측정 시계열 데이터의 통합 수집을 위한 시스템입니다.

**장점**:
- Sub-millisecond 시간 동기화 (NTP 기반)
- 언어 독립적 (Python, C++, MATLAB, C#, Java 등)
- XDF 파일 형식으로 자동 저장
- Multi-stream 동기화 지원
- 대부분의 BCI 하드웨어/소프트웨어 호환

**단점**:
- 로컬 네트워크 최적화 (WAN 지원 제한)
- 추가 라이브러리 의존성 (liblsl)
- 웹 브라우저 직접 지원 없음

**BCI 적용**:
- 연구 환경에서 de facto 표준
- OpenBCI, Emotiv, g.tec 등 주요 BCI 지원
- MATLAB, Python 생태계와 완벽 통합

### 2.2 WebSocket

**개요**: RFC 6455 기반 실시간 양방향 통신 프로토콜

**장점**:
- 웹 브라우저 네이티브 지원
- 양방향 실시간 통신
- HTTP 호환 (방화벽 친화적)
- JSON 메시지 전송 용이

**단점**:
- TCP 기반 (UDP 대비 지연)
- 브로드캐스트/멀티캐스트 미지원
- 추가 시간 동기화 필요

**BCI 적용**:
- 웹 기반 BCI 애플리케이션
- 원격 모니터링 시스템
- 크로스 플랫폼 애플리케이션

### 2.3 TCP/UDP Sockets

**개요**: 저수준 네트워크 소켓 통신

**TCP 장점**:
- 신뢰성 있는 전송
- 순서 보장
- 흐름 제어

**UDP 장점**:
- 낮은 지연시간
- 브로드캐스트/멀티캐스트 지원
- 오버헤드 최소

**BCI 적용**:
- OpenBCI WiFi Shield (TCP/UDP 선택 가능)
- 고속 데이터 스트리밍
- 로컬 네트워크 환경

### 2.4 Bluetooth LE (BLE)

**개요**: 저전력 무선 통신 프로토콜

**장점**:
- 저전력 소비
- 모바일 장치 호환
- 표준화된 GATT 프로파일

**단점**:
- 대역폭 제한 (~1 Mbps)
- 범위 제한 (~10m)
- 연결 수 제한

**BCI 적용**:
- OpenBCI Ganglion (BLE 4.n)
- Muse 헤드셋
- 웨어러블 EEG 장치

### 2.5 USB/Serial

**개요**: 유선 직접 연결

**장점**:
- 높은 대역폭
- 낮은 지연시간
- 안정적 연결

**단점**:
- 유선 제약
- 드라이버 필요
- 플랫폼별 구현 차이

**BCI 적용**:
- OpenBCI Cyton (Serial)
- 연구용 고밀도 EEG
- 임상 환경

---

## 3. Existing BCI System Protocols

### 3.1 OpenBCI

**통신 방식**:
- Cyton: Serial (115200 baud)
- Ganglion: Bluetooth LE
- WiFi Shield: TCP/UDP (deprecated)

**데이터 형식**:
- 33-byte binary packet
- 24-bit samples
- Scale factor: 0.02235 µV/count

**프로토콜 특징**:
- Start/stop 명령 기반
- 패킷 기반 전송
- LSL 호환 출력 지원

### 3.2 BrainFlow

**개요**: 통합 데이터 수집 라이브러리

**통신 방식**:
- Board-agnostic API
- 15+ 제조사, 33+ 모델 지원
- Multicast streaming 지원

**데이터 형식**:
- Uniform data format across boards
- Ring buffer 기반 저장
- PlotJuggler UDP 스트리밍

**프로토콜 특징**:
- `streaming_board://` URL 스키마
- `file://` 저장 옵션
- 실시간 필터링 지원

### 3.3 Emotiv

**통신 방식**:
- USB Dongle (독점 프로토콜)
- Bluetooth
- Emotiv Cortex API (WebSocket)

**데이터 형식**:
- JSON over WebSocket (Cortex API)
- 암호화된 raw data

**프로토콜 특징**:
- OAuth 2.0 인증
- 라이선스 기반 접근
- LSL 출력 지원

### 3.4 Neuralink

**통신 방식**:
- Fully wireless (Bluetooth)
- 200x+ 데이터 압축
- On-chip signal processing

**특징**:
- 1024 채널
- Real-time compression
- Low power telemetry

### 3.5 Synchron

**통신 방식**:
- Wireless to chest unit
- Chest unit to external device (Bluetooth)

**특징**:
- 16 electrodes
- Endovascular placement
- Lower bandwidth than cortical

---

## 4. Protocol Comparison Matrix

| Protocol | Latency | Bandwidth | Web Support | Mobile | Multi-stream |
|----------|---------|-----------|-------------|--------|--------------|
| LSL | <1ms | High | No | Limited | Yes |
| WebSocket | 1-10ms | Medium | Yes | Yes | Manual |
| TCP | 1-5ms | High | No | Yes | No |
| UDP | <1ms | High | No | Yes | Yes |
| BLE | 10-50ms | Low | Web BLE | Yes | Limited |
| Serial | <1ms | High | No | No | No |

---

## 5. Recommendations for WIA BCI Protocol

### 5.1 Primary Protocol: WebSocket + LSL Bridge

**설계 방향**:
1. WebSocket을 기본 전송 계층으로 사용 (웹 호환성)
2. LSL 브릿지 제공 (연구 환경 호환성)
3. TCP/UDP 어댑터 지원 (성능 최적화)

### 5.2 Message Format

**권장 사항**:
- JSON 기반 메시지 (가독성, 확장성)
- Binary 옵션 (성능 필요 시)
- Phase 1 Signal Format을 payload로 사용

### 5.3 Time Synchronization

**권장 사항**:
- NTP 기반 타임스탬프
- 서버-클라이언트 시간 오프셋 계산
- 샘플 인덱스 기반 동기화

### 5.4 Connection Management

**권장 사항**:
- Heartbeat (ping/pong) 메커니즘
- Auto-reconnect with exponential backoff
- State machine 기반 연결 관리

---

## 6. References

- [Lab Streaming Layer Documentation](https://labstreaminglayer.readthedocs.io/)
- [OpenBCI WiFi Shield API](https://docs.openbci.com/ThirdParty/WiFiShield/WiFiAPI/)
- [BrainFlow Documentation](https://brainflow.readthedocs.io/)
- [Emotiv Cortex API](https://emotiv.gitbook.io/cortex-api/)

---

**Document Version**: 1.0
**Last Updated**: 2025-01-XX
**Author**: WIA BCI Working Group

---

弘益人間 - *Benefit All Humanity*
