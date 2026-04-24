# WIA BCI Standard

**Brain-Computer Interface Standards**

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/version-0.1.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/standard-WIA%20BCI-orange.svg)](https://bci.wia.live)

---

<div align="center">

🧠 **Part of WIA Standards Ecosystem**

[WIA Standards Hub](https://wia.live/standards) • [API Portal](https://api.wia.live)

---

**홍익인간 (弘益人間)** - *Benefit All Humanity*

</div>

---

## 🌍 Overview

WIA BCI is an open standard for brain-computer interface standards.

This standard aims to:
- Unify data formats across the industry
- Provide standard APIs for developers  
- Enable interoperability between devices and systems
- Accelerate innovation through open collaboration

---

## 📋 Specification Phases

| Phase | Title | Description | Status |
|:-----:|-------|-------------|:------:|
| **1** | Data Format | Standard data format | ✅ Complete |
| **2** | API Interface | SDK for developers | ✅ Complete |
| **3** | Communication Protocol | Device protocols | ✅ Complete |
| **4** | Ecosystem Integration | WIA integration | ✅ Complete |

---

## 📖 Phase 1: Data Format Standard

WIA BCI Data Format은 뇌-컴퓨터 인터페이스 데이터의 저장, 전송, 교환을 위한 통합 표준입니다.

### Key Documents

| Document | Description |
|----------|-------------|
| [Research Report](spec/RESEARCH-PHASE-1.md) | BCI 기술 조사 보고서 |
| [Data Format Spec](spec/PHASE-1-DATA-FORMAT.md) | 데이터 형식 명세서 |

### JSON Schemas

| Schema | Description |
|--------|-------------|
| [recording.schema.json](spec/schemas/recording.schema.json) | Recording metadata |
| [device.schema.json](spec/schemas/device.schema.json) | Device information |
| [subject.schema.json](spec/schemas/subject.schema.json) | Subject/participant info |
| [channels.schema.json](spec/schemas/channels.schema.json) | Channel definitions |
| [events.schema.json](spec/schemas/events.schema.json) | Event markers |
| [eeg-data.schema.json](spec/schemas/eeg-data.schema.json) | EEG data metadata |

### Supported Formats

- **EEG**: Electroencephalography (비침습적)
- **EMG**: Electromyography
- **EOG**: Electrooculography
- **Compatible with**: EDF+, BIDS, XDF

---

## 📖 Phase 2: API Interface Standard

WIA BCI API는 뇌-컴퓨터 인터페이스 기기와 상호작용하기 위한 표준 프로그래밍 인터페이스입니다.

### Key Documents

| Document | Description |
|----------|-------------|
| [API Spec](spec/PHASE-2-API-INTERFACE.md) | API 인터페이스 명세서 |

### SDK Packages

| Package | Language | Description |
|---------|----------|-------------|
| [wia-bci](api/typescript/) | TypeScript | TypeScript/JavaScript SDK |
| [wia-bci](api/python/) | Python | Python SDK |

### Quick Example (TypeScript)

```typescript
import { WiaBci } from 'wia-bci';

const bci = new WiaBci();
await bci.connect({ type: 'eeg_headset' });

bci.on('signal', (event) => {
  console.log('Signal:', event.data);
});

await bci.startStream();
```

### Quick Example (Python)

```python
from wia_bci import WiaBci, DeviceConfig

bci = WiaBci()
await bci.connect(DeviceConfig(type='eeg_headset'))

@bci.on('signal')
def on_signal(event):
    print('Signal:', event.data)

await bci.start_stream()
```

### Examples

- [TypeScript Basic Usage](examples/api-usage/typescript/basic-usage.ts)
- [TypeScript Motor Imagery](examples/api-usage/typescript/motor-imagery.ts)
- [Python Basic Usage](examples/api-usage/python/basic_usage.py)
- [Python Motor Imagery](examples/api-usage/python/motor_imagery.py)

---

## 📖 Phase 3: Communication Protocol

WIA BCI Communication Protocol은 BCI 기기와 소프트웨어 간의 실시간 데이터 전송을 위한 통신 프로토콜입니다.

### Key Documents

| Document | Description |
|----------|-------------|
| [Research Report](spec/RESEARCH-PHASE-3.md) | 통신 프로토콜 조사 보고서 |
| [Protocol Spec](spec/PHASE-3-PROTOCOL.md) | 프로토콜 명세서 |

### Features

- **Transport Agnostic**: WebSocket, TCP, LSL 등 다양한 전송 계층 지원
- **Real-time Streaming**: 저지연 실시간 신호 스트리밍
- **Connection Management**: 재연결, 하트비트, 에러 처리
- **Binary Mode**: 고성능 바이너리 메시지 형식 지원

### Protocol Example (TypeScript)

```typescript
import { MessageBuilder, WebSocketTransport } from 'wia-bci';

const transport = new WebSocketTransport();
await transport.connect('ws://localhost:9876/wia-bci');

const builder = new MessageBuilder();
await transport.send(builder.connect({
  clientId: 'my-app',
  clientName: 'My BCI App',
}));

transport.onMessage((message) => {
  if (message.type === 'signal') {
    console.log('Signal:', message.payload);
  }
});
```

### Protocol Example (Python)

```python
from wia_bci import MessageBuilder, MockTransport

transport = MockTransport()
await transport.connect('mock://localhost')

builder = MessageBuilder()
await transport.send(builder.connect(ConnectPayload(
    client_id='my-app',
    client_name='My BCI App',
)))

@transport.on_message
def on_message(message):
    if message.type == MessageType.SIGNAL:
        print('Signal:', message.payload)
```

---

## 📖 Phase 4: Ecosystem Integration

WIA BCI Ecosystem Integration은 BCI 출력을 WIA 생태계 (TTS, ISP, Braille)와 연동합니다.

### Key Documents

| Document | Description |
|----------|-------------|
| [Research Report](spec/RESEARCH-PHASE-4.md) | 생태계 통합 조사 보고서 |
| [Integration Spec](spec/PHASE-4-INTEGRATION.md) | 통합 명세서 |

### Features

- **TTS Output**: 텍스트 → 음성 변환 (Web Speech API)
- **Sign Language**: 텍스트 → 수어 아바타 (ISP/WIA Talk)
- **Braille Output**: 텍스트 → 점자 디스플레이
- **Neurofeedback**: 실시간 뇌파 시각화

### Integration Example (TypeScript)

```typescript
import { WiaBci, OutputManager, SignalProcessor } from 'wia-bci';

const bci = new WiaBci();
const output = new OutputManager();

await bci.connect({ type: 'simulator' });
await output.initialize();

// 분류 결과를 음성으로 출력
bci.on('classification', async (event) => {
  await output.output({
    type: 'text',
    text: event.className,
  });
});

// 밴드파워를 실시간 시각화
bci.on('signal', async (event) => {
  const powers = SignalProcessor.allBandPowers(event.data, 250);
  await output.output({
    type: 'signal',
    signal: { bandPowers: powers, channels: [] },
  });
});

await bci.startStream();
```

### Integration Example (Python)

```python
from wia_bci import WiaBci, OutputManager, OutputContent

bci = WiaBci()
output = OutputManager()

await bci.connect(type='simulator')
await output.initialize()

@bci.on('classification')
async def on_classification(event):
    await output.output(OutputContent(
        type='text',
        text=event.class_name,
    ))

await bci.start_stream()
```

---

## 🚀 Quick Start

### Data Structure

```
wia-bci-recording/
├── recording.json     # Main recording metadata
├── device.json        # Device information
├── subject.json       # Subject info (optional)
├── channels.json      # Channel definitions
├── events.json        # Event markers
└── data/
    └── eeg.bin        # Raw EEG data (binary)
```

### Example Usage

```python
import json

# Load recording metadata
with open("recording.json") as f:
    recording = json.load(f)

print(f"Recording ID: {recording['recording_id']}")
print(f"Duration: {recording['recording_info']['duration_seconds']}s")
```

---

## 📁 Structure

```
bci/
├── spec/                    # Specifications
├── api/
│   ├── typescript/          # TypeScript SDK
│   └── python/              # Python SDK
├── examples/
├── prompts/                 # Claude Code prompts
└── docs/
```

---

## 🔗 Links

| Resource | URL |
|----------|-----|
| **Website** | https://bci.wia.live |
| **Standards Hub** | https://wia.live/standards |
| **GitHub** | https://github.com/WIA-Official/wia-standards/tree/main/bci |

---

## 📜 License

MIT License - This standard belongs to humanity.

---

<div align="center">

🤟 **홍익인간 (弘益人間)** - Benefit All Humanity

© 2025 SmileStory Inc. / WIA

</div>
