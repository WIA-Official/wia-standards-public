# WIA Perception Clarity Standard - Phase 1 Research

## 1. Executive Summary

피지컬 AI 에이전트(자율주행차·서비스 로봇·드론·AMR)는 카메라·LiDAR·레이더·초음파 같은 광학/인식 센서로 세계를 본다. 그러나 이 센서의 **투명 창(렌즈·커버·레이돔)이 비·진흙·먼지·소금기·서리·벌레·역광으로 오염되면 인식 성능이 급격히 떨어지고, 최악의 경우 "보이지 않는데도 보인다고 착각"하는 위험 상태에 빠진다.**

핵심 문제는 오염 메커니즘이 아니라 **표준화의 공백**이다. 센서가 "지금 얼마나 잘 보이는가"를 **측정·상태화·외부 보고**하는 공개 규약이 없다. 각 제조사는 사설 진단 신호를 쓰며, 차량/로봇/관제가 서로의 인식 명료도를 해석할 공용어가 없다. WIA Perception Clarity Standard는 이 공백을 메운다. 세척 하드웨어가 아니라 **PCI(Perception Clarity Index) 지표 · 측정 프로토콜 · 보고 규약**을 표준화한다.

## 2. Problem Landscape

### 2.1 렌즈 오염 = 인식 실패

| 오염 유형 | 영향 | 대표 상황 |
|----------|------|-----------|
| Rain film (빗물막) | 카메라 굴절·번짐, LiDAR 산란 | 강우 주행, 분수대 근처 로봇 |
| Mud / dust (진흙·먼지) | 부분~전면 가림 | 비포장로, 건설현장 AMR |
| Salt spray (염무) | 누적 백화·부식 | 해안 도로, 제설 구간 |
| Insect strike (벌레 충돌) | 국소 가림, 닦기 어려움 | 고속 주행, 여름철 드론 |
| Frost / ice (서리·결빙) | 전면 불투명 | 겨울 새벽 시동 직후 |
| Condensation (응결) | 내부 김서림 | 급격한 온도차 |
| Sun glare (역광) | 센서 포화·블루밍 | 일출/일몰 정면 주행 |
| Scratch / abrasion (스크래치) | 영구 산란·고스트 | 누적 마모, 세척 불량 |

### 2.2 센서 클래스별 취약성

- **RGB 카메라**: 굴절·번짐·블루밍에 가장 민감. 픽셀 단위 가림이 객체 탐지·차선 인식을 직접 무력화.
- **IR·열화상**: 물막·서리가 방사율을 왜곡. 야간 보행자 탐지 신뢰도 저하.
- **LiDAR 윈도우**: 표면 오염이 레이저 산란·반사를 유발해 포인트 클라우드에 노이즈·고스트 점 생성.
- **레이더 레이돔**: 두꺼운 진흙·얼음·물막이 신호를 감쇠. 가시광엔 깨끗해 보여도 RF는 막힘.
- **초음파**: 물방울·얼음이 송수신면을 덮어 근접 측정 무효화.

### 2.3 측정 가능성의 부재

오늘날 대부분의 시스템은 오염을 **이진(정상/고장)** 으로만 다루거나, 사설 휴리스틱(예: "밝기 분산이 임계 이하")으로 내부 판정만 한다. 0–100 연속 척도의 **재현 가능한 명료도 지수**, 표준 **상태 enum**, 그리고 이를 외부에 알리는 **공용 보고 메시지**가 없다. 그 결과:

- 차량 ↔ 관제(fleet) 간에 "센서가 흐리다"를 전달할 공용어가 없다.
- 적합성·안전 감사가 제조사별 사설 로그에 의존한다.
- 인식 저하 시 안전 동작(감속·우회·세척·safe-state) 트리거 기준이 제각각이다.

## 3. Existing Standards & Reference Models

### 3.1 VDA5050 — 본보기

VDA5050은 AGV(무인운반차)와 관제 시스템 간 통신 인터페이스 표준이다. **AGV를 어떻게 만드는지(기구·모터·알고리즘)는 표준화하지 않고, fleet ↔ 차량의 메시지 교환만 표준화**하여 사실상 산업 표준이 되었다. WIA Perception Clarity는 이 철학을 그대로 따른다: **세척 메커니즘은 건드리지 않고, 인식 명료도의 측정·상태·보고만 표준화**한다.

### 3.2 관련 표준·개념

| 출처 | 관련성 |
|------|--------|
| ISO 16505 (카메라 모니터 시스템) | 차량 카메라 영상 품질 요구 |
| SAE J3016 (자율주행 레벨) | ODD·fallback 개념과 연동 |
| ISO 11270 / ISO 21448 (SOTIF) | 의도 기능의 안전, 센서 성능 한계 인지 |
| MTF (Modulation Transfer Function) | 광학 해상력 저하의 정량 지표 — PCI contrast 축의 근거 |
| ISO 12233 | 해상력(SFR/MTF) 측정 방법론 |
| O-RAN / MQTT / DDS | 보고 메시지 전송 계층 후보 |

### 3.3 특허 지형 (비표준화 범위 근거)

센서 **세척 하드웨어**는 이미 조밀한 특허밭이다 — 노즐 분사 세척(Valeo, Continental, Kautex), 압축공기 에어커튼, 초음파 진동 세척, 발열 디포깅, 발수 코팅 등. 이 영역을 표준화하면 기존 특허와 충돌하고 채택을 가로막는다. 따라서 WIA Perception Clarity는 **"무엇을 어떻게 닦는가"는 벤더 자유에 두고, "지금 얼마나 잘 보이며 그것을 어떻게 알리는가"만 표준화**한다.

## 4. Conformance Gap

- **연속 명료도 지표 부재**: 오염을 0–100으로 재현 가능하게 산출하는 공개 정의가 없다.
- **상태 모델 부재**: CLEAR / DEGRADED / OBSTRUCTED / BLIND 같은 공용 상태 어휘가 없다.
- **보고 규약 부재**: 센서 단위 명료도를 외부(관제·다른 에이전트)에 알리는 표준 메시지가 없다.
- **SLA 부재**: 오염이 얼마나 오래 잔존하면 안전 동작을 트리거해야 하는지(dwell-time) 합의가 없다.

## 5. Recommendations for WIA Standard

### 5.1 측정 (PCI)

PCI(Perception Clarity Index) 0–100 정수를 센서 단위로 산출한다. 3축 가중 합산:
1. **Occlusion ratio** — 가려진 시야 비율
2. **Detection-distance degradation** — 유효 인식 거리 저하
3. **Contrast / MTF reduction** — 대비·해상력 저하

가중치는 센서 클래스별로 정의한다(예: 레이더는 occlusion보다 신호 감쇠 가중이 큼).

### 5.2 상태화 (State Enum)

CLEAR(90–100) · DEGRADED(60–89) · OBSTRUCTED(30–59) · BLIND(0–29). 각 상태는 요구 동작(정상/모니터링/안전동작/센서중단)과 연동한다.

### 5.3 보고 (Reporting)

VDA5050식 JSON 메시지로 header + sensors[]를 송출한다. 각 센서는 sensorId · sensorClass · pci · state · contaminants[] · lastCleanedAt · dwellSeconds · confidence를 담는다.

### 5.4 적합성 레벨

- **L-A**: PCI 산출 + 상태 enum 노출
- **L-B**: A + 표준 보고 메시지 송출
- **L-C**: B + dwell-time SLA + 안전 동작 트리거 연동

### 5.5 상호운용

robot · auto · drone · lidar-sensor · vision-ai · ai-sensor-fusion 등 기존 WIA 표준과 횡단 연동한다. agentType enum: vehicle / robot / drone / amr / other.

## 6. Sources

- [VDA5050 — Interface for the communication between AGVs and a master control](https://www.vda.de/en/news/publications/publication/vda-5050-v-2-0-0)
- [SAE J3016 — Levels of Driving Automation](https://www.sae.org/standards/content/j3016_202104/)
- [ISO 21448 — Safety of the Intended Functionality (SOTIF)](https://www.iso.org/standard/77490.html)
- [ISO 16505 — Camera monitor systems](https://www.iso.org/standard/78970.html)
- [ISO 12233 — Resolution and spatial frequency responses](https://www.iso.org/standard/71696.html)
- [Modulation Transfer Function — optical resolution metric](https://www.imatest.com/docs/sharpness/)
- [Sensor cleaning challenges in autonomous driving — Valeo / Continental whitepapers](https://www.valeo.com/en/sensor-cleaning/)

---

*Research completed: June 2025*
*WIA Perception Clarity Standard - Phase 1*
*弘益人間 · Benefit All Humanity*
