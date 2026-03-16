# WIA-AMR Phase 4: Integration Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01-15

---

## 1. Overview

Phase 4 of the WIA-AMR standard defines integration patterns for connecting AMR systems with enterprise applications such as Warehouse Management Systems (WMS), Enterprise Resource Planning (ERP), and Manufacturing Execution Systems (MES).

### 1.1 Integration Goals

- **Seamless Data Flow**: Real-time synchronization between systems
- **Automated Workflows**: Minimal manual intervention
- **Unified Monitoring**: Single pane of glass for operations
- **Scalable Architecture**: Support for growing fleet sizes

### 1.2 Integration Patterns

| Pattern | Description | Use Case |
|---------|-------------|----------|
| Point-to-Point | Direct system connections | Small deployments |
| Hub-and-Spoke | Central integration hub | Medium deployments |
| Event-Driven | Message broker based | Large, scalable deployments |

---

## 2. Event-Driven Architecture

### 2.1 Event Bus

WIA-AMR recommends an event-driven architecture for enterprise integration:

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│     WMS     │     │   AMR FMS   │     │     ERP     │
└──────┬──────┘     └──────┬──────┘     └──────┬──────┘
       │                   │                   │
       ▼                   ▼                   ▼
┌─────────────────────────────────────────────────────┐
│                    Event Bus                        │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐ │
│  │ Task Events │  │Robot Events │  │ Inv Events  │ │
│  └─────────────┘  └─────────────┘  └─────────────┘ │
└─────────────────────────────────────────────────────┘
```

### 2.2 Event Catalog

| Event Type | Publisher | Subscribers | Description |
|------------|-----------|-------------|-------------|
| task.created | WMS/ERP | FMS | New task created |
| task.assigned | FMS | WMS, Dashboard | Task assigned to robot |
| task.started | FMS | WMS, Dashboard | Task execution started |
| task.completed | FMS | WMS, ERP | Task completed |
| task.failed | FMS | WMS, Dashboard | Task failed |
| task.cancelled | FMS/WMS | All | Task cancelled |
| robot.state.changed | Robot | FMS, Dashboard | Robot state change |
| robot.position.updated | Robot | FMS, Dashboard | Position update |
| robot.error | Robot/FMS | Dashboard, Alerts | Error occurred |
| robot.battery.low | Robot | FMS | Low battery |
| inventory.moved | FMS | WMS, ERP | Physical inventory move |
| inventory.picked | FMS | WMS | Item picked |
| zone.entered | Robot | FMS, Traffic | Robot entered zone |
| zone.exited | Robot | FMS, Traffic | Robot exited zone |

### 2.3 Event Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wiastandards.com/schemas/amr/event.json",
  "title": "WIA-AMR Event",
  "type": "object",
  "properties": {
    "eventId": {
      "type": "string",
      "description": "Unique event identifier (UUID)"
    },
    "eventType": {
      "type": "string",
      "description": "Event type (e.g., task.completed)"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "Event occurrence time"
    },
    "source": {
      "type": "object",
      "properties": {
        "system": { "type": "string" },
        "instance": { "type": "string" },
        "version": { "type": "string" }
      },
      "required": ["system"]
    },
    "correlationId": {
      "type": "string",
      "description": "For tracking related events"
    },
    "causationId": {
      "type": "string",
      "description": "ID of event that caused this event"
    },
    "data": {
      "type": "object",
      "description": "Event-specific payload"
    },
    "metadata": {
      "type": "object",
      "description": "Additional metadata"
    }
  },
  "required": ["eventId", "eventType", "timestamp", "source", "data"]
}
```

### 2.4 Event Examples

#### Task Completed Event

```json
{
  "eventId": "evt-12345-abcd",
  "eventType": "task.completed",
  "timestamp": "2025-01-15T10:35:00Z",
  "source": {
    "system": "WIA-FMS",
    "instance": "fms-prod-01",
    "version": "2.3.1"
  },
  "correlationId": "order-67890",
  "data": {
    "taskId": "task-12345",
    "robotId": "amr-001",
    "taskType": "PICK_AND_TRANSPORT",
    "startedAt": "2025-01-15T10:32:00Z",
    "completedAt": "2025-01-15T10:35:00Z",
    "duration": 180,
    "distanceTraveled": 45.6,
    "source": {
      "locationId": "RACK-A01-02-03",
      "coordinates": { "x": 45.5, "y": 28.3 }
    },
    "destination": {
      "locationId": "PACK-STATION-01",
      "coordinates": { "x": 100.0, "y": 50.0 }
    },
    "payload": {
      "itemId": "SKU-12345",
      "quantity": 3
    }
  },
  "metadata": {
    "priority": 80,
    "externalRef": {
      "system": "SAP_EWM",
      "orderId": "4500012345"
    }
  }
}
```

---

## 3. WMS Integration

### 3.1 Integration Architecture

```
┌─────────────────────────────────────────────────────────┐
│                          WMS                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ Order Mgmt   │  │ Inventory    │  │ Task Mgmt    │  │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  │
└─────────┼────────────────┼────────────────┼────────────┘
          │                │                │
          ▼                ▼                ▼
┌─────────────────────────────────────────────────────────┐
│                   Integration Layer                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ Task Adapter │  │Location Mapper│  │ Inv Sync    │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
└─────────────────────────────────────────────────────────┘
          │                │                │
          ▼                ▼                ▼
┌─────────────────────────────────────────────────────────┐
│                       AMR FMS                           │
└─────────────────────────────────────────────────────────┘
```

### 3.2 Task Flow

1. **Order Receipt**: WMS receives order from ERP/OMS
2. **Task Creation**: WMS creates warehouse task
3. **Task Dispatch**: WMS sends task to FMS via API/Event
4. **Robot Assignment**: FMS selects and assigns robot
5. **Execution**: Robot navigates and performs actions
6. **Status Updates**: FMS sends progress updates to WMS
7. **Completion**: FMS notifies WMS of task completion
8. **Inventory Update**: WMS updates inventory records

### 3.3 Location Mapping

WMS uses logical locations; AMR uses physical coordinates:

```json
{
  "locationMappings": [
    {
      "wmsLocation": "RACK-A01-02-03",
      "amrCoordinates": {
        "x": 45.5,
        "y": 28.3,
        "theta": 0,
        "mapId": "warehouse-01"
      },
      "approachPoints": [
        { "x": 44.5, "y": 28.3, "theta": 0, "direction": "FRONT" },
        { "x": 46.5, "y": 28.3, "theta": 3.14, "direction": "BACK" }
      ],
      "metadata": {
        "zone": "A",
        "aisle": "01",
        "rack": "02",
        "level": "03",
        "locationType": "STORAGE"
      }
    }
  ]
}
```

### 3.4 WMS Integration API

#### Create Task from WMS

```http
POST /api/v1/integration/wms/tasks
Content-Type: application/json
X-Source-System: SAP-EWM
X-Correlation-Id: order-67890

{
  "warehouseTaskId": "WT-001",
  "taskType": "PICK",
  "priority": 80,
  "sourceLocation": "RACK-A01-02-03",
  "destinationLocation": "PACK-STATION-01",
  "items": [
    {
      "itemId": "SKU-12345",
      "quantity": 3,
      "weight": 2.5
    }
  ],
  "constraints": {
    "deadline": "2025-01-15T12:00:00Z"
  }
}
```

#### Task Status Callback

```http
POST https://wms.example.com/api/amr/task-status
Content-Type: application/json

{
  "warehouseTaskId": "WT-001",
  "amrTaskId": "task-12345",
  "status": "COMPLETED",
  "robotId": "amr-001",
  "completedAt": "2025-01-15T10:35:00Z",
  "actualDuration": 180,
  "result": {
    "success": true,
    "itemsProcessed": 3
  }
}
```

---

## 4. ERP Integration

### 4.1 Integration Points

| Function | Direction | Description |
|----------|-----------|-------------|
| Inventory Movement | FMS → ERP | Record physical movements |
| Work Order | ERP → FMS | Production task triggers |
| Cost Center | FMS → ERP | Energy, maintenance costs |
| Asset Management | FMS → ERP | Robot asset data |
| Analytics | FMS → ERP | Performance metrics |

### 4.2 SAP Integration

#### RFC/BAPI Integration

```typescript
// Inventory Movement via BAPI
async function recordSAPMovement(event: TaskCompletedEvent) {
  const sapClient = new SAPClient(config);

  await sapClient.call('BAPI_GOODSMVT_CREATE', {
    GOODSMVT_HEADER: {
      PSTNG_DATE: formatDate(event.data.completedAt),
      DOC_DATE: formatDate(event.data.completedAt),
      PR_UNAME: 'AMR_SYSTEM'
    },
    GOODSMVT_CODE: {
      GM_CODE: '04'
    },
    GOODSMVT_ITEM: [{
      MATERIAL: event.data.payload.itemId,
      PLANT: '1000',
      STGE_LOC: 'WH01',
      MOVE_TYPE: '311',
      ENTRY_QNT: event.data.payload.quantity,
      ENTRY_UOM: 'EA'
    }]
  });
}
```

#### IDoc Integration

```xml
<IDOC BEGIN="1">
  <EDI_DC40 SEGMENT="1">
    <IDOCTYP>WMMBID02</IDOCTYP>
    <MESTYP>WMMBID</MESTYP>
  </EDI_DC40>
  <E1MBXIH SEGMENT="1">
    <MBLNR>4900000123</MBLNR>
    <MJAHR>2025</MJAHR>
    <BLDAT>20250115</BLDAT>
  </E1MBXIH>
</IDOC>
```

### 4.3 Event-to-Transaction Mapping

| Event | ERP Transaction | Description |
|-------|-----------------|-------------|
| inventory.picked | Goods Issue | Stock removed |
| inventory.placed | Goods Receipt | Stock added |
| inventory.moved | Transfer Posting | Location change |
| task.completed | Confirmation | Work confirmed |

---

## 5. Fleet Monitoring Dashboard

### 5.1 Dashboard API

#### Fleet Summary

```http
GET /api/v1/dashboard/fleet/summary
```

```json
{
  "timestamp": "2025-01-15T10:30:00Z",
  "fleet": {
    "totalRobots": 50,
    "activeRobots": 45,
    "chargingRobots": 3,
    "errorRobots": 2,
    "idleRobots": 0,
    "utilizationRate": 0.90
  },
  "tasks": {
    "pending": 15,
    "inProgress": 42,
    "completedToday": 523,
    "failedToday": 5,
    "averageCompletionTime": 185,
    "throughputPerHour": 65
  },
  "performance": {
    "totalDistanceKm": 127.5,
    "energyConsumedKwh": 42.3,
    "averageSpeed": 0.85
  },
  "alerts": {
    "critical": 0,
    "warning": 3,
    "info": 12
  }
}
```

#### Robot Positions (Real-time)

```http
GET /api/v1/dashboard/robots/positions
```

WebSocket alternative:

```javascript
ws.subscribe('dashboard.robots.positions', (data) => {
  data.robots.forEach(robot => {
    updateMapMarker(robot.robotId, robot.position);
  });
});
```

### 5.2 Key Metrics

| Metric | Description | Update Frequency |
|--------|-------------|------------------|
| Utilization Rate | Active robots / Total robots | Real-time |
| Throughput | Tasks completed per hour | 1 minute |
| Average Task Time | Mean task duration | 5 minutes |
| Distance Traveled | Total km traveled | 1 minute |
| Energy Consumption | Total kWh used | 5 minutes |
| Error Rate | Failed tasks / Total tasks | 1 hour |
| Battery Health | Average battery level | 5 minutes |

### 5.3 Alert Configuration

```json
{
  "alerts": [
    {
      "name": "Low Battery Warning",
      "condition": "robot.battery.level < 20",
      "severity": "WARNING",
      "channels": ["dashboard", "email"]
    },
    {
      "name": "Robot Error",
      "condition": "robot.operatingState == 'ERROR'",
      "severity": "CRITICAL",
      "channels": ["dashboard", "sms", "slack"]
    },
    {
      "name": "High Queue",
      "condition": "tasks.pending > 50",
      "severity": "WARNING",
      "channels": ["dashboard", "email"]
    }
  ]
}
```

---

## 6. Data Analytics

### 6.1 Data Warehouse Integration

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   AMR FMS   │────▶│   ETL/CDC   │────▶│ Data Lake   │
└─────────────┘     └─────────────┘     └──────┬──────┘
                                               │
                    ┌──────────────────────────┼──────┐
                    │                          ▼      │
                    │  ┌─────────────┐  ┌───────────┐ │
                    │  │ Analytics   │  │ ML Models │ │
                    │  └─────────────┘  └───────────┘ │
                    │                                 │
                    └─────────────────────────────────┘
```

### 6.2 Analytics Use Cases

| Use Case | Data Sources | Insights |
|----------|--------------|----------|
| Route Optimization | Position history, tasks | Optimal paths, bottlenecks |
| Predictive Maintenance | Sensor data, errors | Failure prediction |
| Demand Forecasting | Task history, seasonality | Capacity planning |
| Energy Optimization | Battery data, charging | Charging schedules |

### 6.3 Standard Reports

| Report | Frequency | Contents |
|--------|-----------|----------|
| Daily Operations | Daily | Throughput, utilization, errors |
| Weekly Performance | Weekly | Trends, comparisons, efficiency |
| Monthly Cost | Monthly | Energy, maintenance, ROI |
| Maintenance Forecast | Weekly | Predicted service needs |

---

## 7. Security Considerations

### 7.1 Network Segmentation

```
┌─────────────────────────────────────────────────────┐
│                 Enterprise Network                  │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐ │
│  │    WMS      │  │    ERP      │  │  Dashboard  │ │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘ │
└─────────┼────────────────┼────────────────┼────────┘
          │                │                │
          ▼                ▼                ▼
┌─────────────────────────────────────────────────────┐
│                    DMZ / API Gateway                │
└─────────────────────────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────────────────────┐
│                   OT Network (AMR)                  │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐ │
│  │    FMS      │  │ MQTT Broker │  │   Robots    │ │
│  └─────────────┘  └─────────────┘  └─────────────┘ │
└─────────────────────────────────────────────────────┘
```

### 7.2 Data Protection

- Encrypt data at rest and in transit
- Implement access logging and audit trails
- Regular security assessments
- Compliance with industry regulations (GDPR, etc.)

---

## 8. Implementation Checklist

### 8.1 Pre-Integration

- [ ] Define integration scope and requirements
- [ ] Select integration pattern (P2P, Hub, Event)
- [ ] Design data models and mappings
- [ ] Plan security architecture
- [ ] Set up development/test environments

### 8.2 Development

- [ ] Implement API adapters
- [ ] Configure event bus/message broker
- [ ] Develop location mapping service
- [ ] Build monitoring dashboard
- [ ] Create data pipelines

### 8.3 Testing

- [ ] Unit tests for adapters
- [ ] Integration tests with mock systems
- [ ] End-to-end workflow tests
- [ ] Performance/load testing
- [ ] Security testing

### 8.4 Deployment

- [ ] Deploy to staging environment
- [ ] Pilot with limited scope
- [ ] Monitor and tune
- [ ] Full production rollout
- [ ] Documentation and training

---

## 9. References

- Enterprise Integration Patterns: https://www.enterpriseintegrationpatterns.com/
- SAP Integration: https://api.sap.com/
- Event-Driven Architecture: https://martinfowler.com/articles/201701-event-driven.html

---

© 2025 WIA Standards | 弘益人間 · Benefit All Humanity
