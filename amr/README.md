# WIA-AMR: Autonomous Mobile Robot Interoperability Standard

<div align="center">

🤖 **WIA-AMR Standard** 🤖

*Universal Interoperability for Autonomous Mobile Robots*

[![Version](https://img.shields.io/badge/version-1.0.0-8B5CF6)](https://amr.wiastandards.com)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)
[![VDA 5050](https://img.shields.io/badge/VDA_5050-Compatible-blue)](https://github.com/VDA5050/VDA5050)

**홍익인간 (弘益人間) - Benefit All Humanity**

[Landing Page](https://amr.wiastandards.com) |
[Simulator](https://amr.wiastandards.com/simulator/) |
[Ebook](https://wiabooks.store/tag/wia-amr/) |
[Documentation](./spec/)

</div>

---

## 📋 Overview

WIA-AMR is a comprehensive interoperability standard for Autonomous Mobile Robots (AMRs). It defines data formats, APIs, protocols, and integration patterns to enable seamless communication between robots from different manufacturers and enterprise systems.

### Key Features

- 🔄 **Universal Interoperability**: Connect robots from any manufacturer
- 📊 **Standardized Data Formats**: JSON Schema-based data definitions
- 🔌 **RESTful APIs**: Consistent API design with OpenAPI specification
- 📡 **Real-time Protocols**: MQTT and WebSocket for live communication
- 🔗 **VDA 5050 Compatible**: Full compatibility with European automotive standard
- 🏭 **Enterprise Integration**: WMS, ERP, MES integration patterns

---

## 🏗️ 4-Phase Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        WIA-AMR Standard                         │
├─────────────┬─────────────┬─────────────┬─────────────────────┤
│   Phase 1   │   Phase 2   │   Phase 3   │      Phase 4        │
│ Data Format │    API      │  Protocol   │    Integration      │
├─────────────┼─────────────┼─────────────┼─────────────────────┤
│ JSON Schema │ REST API    │ MQTT        │ WMS Integration     │
│ Core Types  │ OAuth 2.0   │ WebSocket   │ ERP Integration     │
│ Validation  │ Error Codes │ VDA 5050    │ Event-Driven        │
└─────────────┴─────────────┴─────────────┴─────────────────────┘
```

---

## 📁 Directory Structure

```
amr/
├── index.html              # Landing page
├── simulator/
│   └── index.html          # Interactive simulator (5 tabs, 99 languages)
├── ebook/
│   ├── en/                 # English ebook (8 chapters)
│   │   ├── index.html
│   │   └── chapter-*.html
│   └── ko/                 # Korean ebook (8 chapters)
│       ├── index.html
│       └── chapter-*.html
├── spec/
│   ├── PHASE-1-DATA-FORMAT.md
│   ├── PHASE-2-API-INTERFACE.md
│   ├── PHASE-3-PROTOCOL.md
│   └── PHASE-4-INTEGRATION.md
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── types.ts    # TypeScript type definitions
│       │   └── index.ts    # SDK implementation
│       └── package.json
└── README.md
```

---

## 🚀 Quick Start

### Installation

```bash
npm install @wia/amr-sdk
```

### Basic Usage

```typescript
import { WiaAmrClient, WiaAmrMqttClient } from '@wia/amr-sdk';

// REST API Client
const client = new WiaAmrClient({
  baseUrl: 'https://api.example.com',
  authToken: 'your-token'
});

// List robots
const robots = await client.listRobots();

// Create a task
const task = await client.createTask({
  taskType: 'NAVIGATE',
  destination: { x: 50.0, y: 30.0, theta: 0 },
  priority: 80
});

// MQTT for real-time updates
const mqtt = new WiaAmrMqttClient({
  brokerUrl: 'mqtt://broker.example.com',
  username: 'user',
  password: 'pass'
});

await mqtt.connect();

mqtt.subscribeAllRobotStates((topic, state) => {
  console.log(`Robot ${state.robotId} at (${state.position.x}, ${state.position.y})`);
});
```

---

## 📊 Core Data Types

### RobotState

```typescript
interface RobotState {
  robotId: string;
  position: Position;
  velocity?: Velocity;
  battery?: BatteryState;
  operatingState: OperatingState;
  safetyState?: SafetyState;
  currentTaskId?: string;
  errors?: RobotError[];
  timestamp: string;
}
```

### Task

```typescript
interface Task {
  taskId: string;
  taskType: 'NAVIGATE' | 'PICK' | 'PLACE' | 'DOCK' | 'CHARGE';
  priority?: number;
  source?: Position;
  destination?: Position;
  actions?: Action[];
  constraints?: TaskConstraints;
  status?: TaskStatus;
}
```

### Position

```typescript
interface Position {
  x: number;      // meters
  y: number;      // meters
  theta: number;  // radians
  mapId?: string;
}
```

---

## 📡 MQTT Topics

```
/wia/amr/v1/
├── robots/{robotId}/
│   ├── state           # Robot state (QoS 1)
│   ├── telemetry       # Sensor data (QoS 0)
│   ├── commands        # Control commands (QoS 2)
│   └── events          # Robot events (QoS 1)
├── fleet/
│   ├── status          # Fleet summary
│   └── alerts          # Fleet-wide alerts
└── traffic/
    └── reservation/    # Zone reservations
```

---

## 🔌 VDA 5050 Compatibility

WIA-AMR is fully compatible with VDA 5050:

```typescript
import { VDA5050Adapter } from '@wia/amr-sdk';

const adapter = new VDA5050Adapter();

// Convert WIA-AMR Task to VDA 5050 Order
const order = adapter.taskToOrder(task, {
  manufacturer: 'WIA',
  serialNumber: 'amr-001'
});

// Convert VDA 5050 State to WIA-AMR RobotState
const robotState = adapter.stateToRobotState(vdaState);
```

---

## 🏢 Enterprise Integration

### WMS Integration

```http
POST /api/v1/integration/wms/tasks
Content-Type: application/json

{
  "warehouseTaskId": "WT-001",
  "taskType": "PICK",
  "sourceLocation": "RACK-A01-02-03",
  "destinationLocation": "PACK-STATION-01"
}
```

### Event-Driven Architecture

```typescript
eventBus.subscribe('task.completed', async (event) => {
  await updateWMS(event.data);
  await recordERPTransaction(event.data);
  await sendNotification(event.data);
});
```

---

## 📜 Specifications

| Document | Description |
|----------|-------------|
| [PHASE-1-DATA-FORMAT.md](./spec/PHASE-1-DATA-FORMAT.md) | JSON Schema, core types, units |
| [PHASE-2-API-INTERFACE.md](./spec/PHASE-2-API-INTERFACE.md) | REST API, authentication, errors |
| [PHASE-3-PROTOCOL.md](./spec/PHASE-3-PROTOCOL.md) | MQTT, WebSocket, VDA 5050 |
| [PHASE-4-INTEGRATION.md](./spec/PHASE-4-INTEGRATION.md) | WMS, ERP, event architecture |

---

## 🎓 Ebook

Comprehensive guides available in:

- **English**: [Ebook EN](./ebook/en/)
- **Korean**: [Ebook KO](./ebook/ko/)

**Chapters:**
1. Introduction to AMR
2. Current Challenges
3. Standard Overview
4. Phase 1: Data Format
5. Phase 2: API Interface
6. Phase 3: Protocol
7. Phase 4: Integration
8. Implementation & Certification

---

## 🏆 WIA Certification

| Level | Requirements |
|-------|--------------|
| 🥉 Bronze | Phase 1 (Data Format) compliance |
| 🥈 Silver | Bronze + Phase 2, 3 compliance |
| 🥇 Gold | Silver + Phase 4 + integration tests |
| 💎 Platinum | Gold + field verification |

---

## 🤝 Contributing

Contributions are welcome! Please read our contributing guidelines and submit pull requests to:

- GitHub: [WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)

---

## 📄 License

MIT License - See [LICENSE](LICENSE) for details.

---

## 🔗 Links

- **Website**: [https://amr.wiastandards.com](https://amr.wiastandards.com)
- **Simulator**: [https://amr.wiastandards.com/simulator/](https://amr.wiastandards.com/simulator/)
- **Ebook Store**: [https://wiabooks.store/tag/wia-amr/](https://wiabooks.store/tag/wia-amr/)
- **GitHub**: [https://github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)

---

<div align="center">

**弘益人間 · 널리 인간을 이롭게 하라 · Benefit All Humanity**

© 2025 WIA Standards | MIT License

</div>

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
