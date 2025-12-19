# WIA Emotion AI Standard

**Affective Computing / Emotion Recognition Standards**

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/standard-WIA%20Emotion%20AI-EC4899.svg)](https://emotion-ai.wia.live)

---

<div align="center">

💗 **Part of WIA Standards Ecosystem**

[WIA Standards Hub](https://wia.live/standards) • [API Portal](https://api.wia.live)

---

**弘益人間** - *Benefit All Humanity*

</div>

---

## 🌍 Overview

WIA Emotion AI is an open standard for affective computing and emotion recognition systems.

This standard aims to:
- Unify emotion data formats across the industry (Ekman, FACS, Valence-Arousal)
- Provide standard APIs for developers
- Enable interoperability between emotion AI systems
- Support multimodal fusion (facial, voice, text, biosignal)
- Ensure ethical and privacy-compliant implementations

---

## 📋 Specification Phases

| Phase | Title | Description | Status |
|:-----:|-------|-------------|:------:|
| **1** | Data Format | Emotion data JSON schema (Ekman, FACS AU, V-A-D) | ✅ Complete |
| **2** | API Interface | REST API for emotion analysis | ✅ Complete |
| **3** | Streaming Protocol | WebSocket real-time streaming | ✅ Complete |
| **4** | Integration | Domain integrations (Healthcare, Education, Automotive) | ✅ Complete |

---

## 📖 Phase 1: Data Format Standard

WIA Emotion AI Data Format은 감정 데이터의 저장, 전송, 교환을 위한 통합 표준입니다.

### Core Components

- **Ekman's 6 Basic Emotions**: happiness, sadness, anger, fear, disgust, surprise
- **Extended Emotions**: neutral, contempt, confusion, interest, boredom, frustration, excitement, anxiety
- **FACS Action Units**: AU1-AU26 encoding with intensity (0-1)
- **Dimensional Model**: Valence (-1 to +1), Arousal (0-1), Dominance (0-1)

### Example

```json
{
    "event_id": "550e8400-e29b-41d4-a716-446655440000",
    "timestamp": "2025-01-15T14:30:00.000Z",
    "version": "1.0.0",
    "emotions": [
        {
            "category": "happiness",
            "intensity": 0.85,
            "confidence": 0.92
        }
    ],
    "action_units": [
        {"au": "AU6", "intensity": 0.75, "confidence": 0.91},
        {"au": "AU12", "intensity": 0.82, "confidence": 0.94}
    ],
    "dimensional": {
        "valence": 0.78,
        "arousal": 0.62,
        "dominance": 0.71
    }
}
```

---

## 🔌 Phase 2: API Interface

REST API for emotion analysis services.

### Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/wia/emotion/v1/analyze/facial/image` | Analyze facial expression in image |
| POST | `/wia/emotion/v1/analyze/facial/video` | Analyze video (async) |
| POST | `/wia/emotion/v1/analyze/voice/audio` | Analyze voice emotion |
| POST | `/wia/emotion/v1/analyze/text` | Analyze text sentiment |
| POST | `/wia/emotion/v1/analyze/biosignal` | Analyze biosignal data |
| POST | `/wia/emotion/v1/analyze/multimodal` | Multimodal fusion analysis |

---

## 📡 Phase 3: Streaming Protocol

WebSocket protocol for real-time emotion streaming.

```
wss://stream.{provider}.com/wia/emotion/v1/stream
```

### Message Types

- `session.start` / `session.started`
- `data.frame` (video/audio frames)
- `emotion.event` (analysis results)
- `emotion.aggregate` (periodic summaries)
- `heartbeat.ping` / `heartbeat.pong`

---

## 🔗 Phase 4: Domain Integration

Industry-specific integration guidelines:

| Domain | Use Cases |
|--------|-----------|
| **Healthcare** | Depression monitoring, anxiety detection, therapy effectiveness |
| **Education** | Engagement tracking, frustration detection, adaptive learning |
| **Marketing** | Ad testing, UX research, customer feedback analysis |
| **Automotive** | Drowsiness detection, distraction monitoring, road rage prevention |

---

## 📦 Installation

### Python

```bash
pip install wia-emotion-ai
```

### TypeScript/JavaScript

```bash
npm install @wia/emotion-ai
```

---

## 🚀 Quick Start

### Python

```python
from wia_emotion_ai import EmotionClient

client = EmotionClient(api_key="your-api-key")

# Analyze image
result = client.analyze_facial_image("photo.jpg")
print(result.emotions[0].category)  # "happiness"

# Real-time streaming
stream = client.create_stream(modalities=["facial", "voice"])
stream.on("emotion", lambda e: print(e.emotions))
stream.start()
```

### TypeScript

```typescript
import { EmotionClient } from '@wia/emotion-ai';

const client = new EmotionClient({ apiKey: 'your-api-key' });

// Analyze image
const result = await client.analyzeFacialImage('photo.jpg');
console.log(result.emotions[0].category); // "happiness"

// Real-time streaming
const stream = client.createStream({ modalities: ['facial', 'voice'] });
stream.on('emotion', (e) => console.log(e.emotions));
stream.start();
```

---

## 📁 Repository Structure

```
emotion-ai/
├── LICENSE
├── README.md
├── api/
│   ├── python/          # Python SDK
│   └── typescript/      # TypeScript SDK
├── examples/            # Usage examples
└── spec/
    ├── PHASE-1-DATA-FORMAT.md
    ├── PHASE-2-API.md
    ├── PHASE-3-PROTOCOL.md
    ├── PHASE-4-INTEGRATION.md
    └── schemas/         # JSON schemas
```

---

## 🤝 Contributing

We welcome contributions! Please see our [Contributing Guide](https://github.com/WIA-Official/wia-standards/blob/main/CONTRIBUTING.md).

---

## 📜 License

MIT License - see [LICENSE](./LICENSE)

---

<div align="center">

**WIA Emotion AI Standard** - Part of **WIA Standards Ecosystem**

💗 Made with love for humanity

**弘益人間** - *Benefit All Humanity*

</div>
