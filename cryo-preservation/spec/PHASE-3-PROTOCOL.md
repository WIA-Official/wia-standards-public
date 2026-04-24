# Phase 3: Protocol Specification

## WIA-CRYO-PRESERVATION API and Communication Protocols

> Standardized interfaces for cryopreservation facility systems.

---

## 1. API Overview

### 1.1 Base Configuration

```yaml
openapi: 3.0.3
info:
  title: WIA Cryopreservation API
  version: 1.0.0
  description: |
    API for cryopreservation facility management, monitoring,
    and inter-facility communication.

servers:
  - url: https://api.cryo.wia.org/v1
    description: Production
  - url: https://staging.cryo.wia.org/v1
    description: Staging

security:
  - BearerAuth: []
  - FacilityAuth: []
```

### 1.2 Authentication

```yaml
securitySchemes:
  BearerAuth:
    type: http
    scheme: bearer
    bearerFormat: JWT

  FacilityAuth:
    type: apiKey
    in: header
    name: X-Facility-Key
    description: Facility-to-facility authentication

  DIDAuth:
    type: http
    scheme: bearer
    bearerFormat: DID-Auth
```

---

## 2. Subject Management Endpoints

### 2.1 Subject Registration

```yaml
/subjects:
  post:
    summary: Register new subject for preservation
    operationId: registerSubject
    tags: [Subjects]
    requestBody:
      required: true
      content:
        application/json:
          schema:
            $ref: '#/components/schemas/SubjectRegistration'
    responses:
      201:
        description: Subject registered successfully
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/SubjectDocument'
      400:
        $ref: '#/components/responses/ValidationError'
      409:
        description: Subject already exists

  get:
    summary: List subjects at facility
    operationId: listSubjects
    tags: [Subjects]
    parameters:
      - name: status
        in: query
        schema:
          $ref: '#/components/schemas/PreservationStatus'
      - name: type
        in: query
        schema:
          $ref: '#/components/schemas/SubjectType'
      - $ref: '#/components/parameters/Pagination'
    responses:
      200:
        description: List of subjects
        content:
          application/json:
            schema:
              type: object
              properties:
                subjects:
                  type: array
                  items:
                    $ref: '#/components/schemas/SubjectSummary'
                pagination:
                  $ref: '#/components/schemas/PaginationInfo'
```

### 2.2 Subject Details

```yaml
/subjects/{subjectId}:
  get:
    summary: Get subject details
    operationId: getSubject
    tags: [Subjects]
    parameters:
      - $ref: '#/components/parameters/SubjectId'
    responses:
      200:
        description: Subject details
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/SubjectDocument'
      404:
        $ref: '#/components/responses/NotFound'

  patch:
    summary: Update subject information
    operationId: updateSubject
    tags: [Subjects]
    parameters:
      - $ref: '#/components/parameters/SubjectId'
    requestBody:
      required: true
      content:
        application/json:
          schema:
            $ref: '#/components/schemas/SubjectUpdate'
    responses:
      200:
        description: Subject updated
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/SubjectDocument'
```

### 2.3 Subject Events

```yaml
/subjects/{subjectId}/events:
  get:
    summary: Get subject event history
    operationId: getSubjectEvents
    tags: [Subjects, Events]
    parameters:
      - $ref: '#/components/parameters/SubjectId'
      - name: eventType
        in: query
        schema:
          $ref: '#/components/schemas/PreservationEventType'
      - $ref: '#/components/parameters/DateRange'
      - $ref: '#/components/parameters/Pagination'
    responses:
      200:
        description: Event history
        content:
          application/json:
            schema:
              type: object
              properties:
                events:
                  type: array
                  items:
                    $ref: '#/components/schemas/PreservationEvent'

  post:
    summary: Log new event for subject
    operationId: logSubjectEvent
    tags: [Subjects, Events]
    parameters:
      - $ref: '#/components/parameters/SubjectId'
    requestBody:
      required: true
      content:
        application/json:
          schema:
            $ref: '#/components/schemas/EventLog'
    responses:
      201:
        description: Event logged
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/PreservationEvent'
```

---

## 3. Storage Management Endpoints

### 3.1 Containers

```yaml
/containers:
  get:
    summary: List storage containers
    operationId: listContainers
    tags: [Storage]
    parameters:
      - name: status
        in: query
        schema:
          type: string
          enum: [NORMAL, WARNING, CRITICAL]
      - name: type
        in: query
        schema:
          $ref: '#/components/schemas/ContainerType'
    responses:
      200:
        description: Container list
        content:
          application/json:
            schema:
              type: object
              properties:
                containers:
                  type: array
                  items:
                    $ref: '#/components/schemas/StorageContainer'

  post:
    summary: Register new container
    operationId: registerContainer
    tags: [Storage]
    requestBody:
      required: true
      content:
        application/json:
          schema:
            $ref: '#/components/schemas/ContainerRegistration'
    responses:
      201:
        description: Container registered

/containers/{containerId}:
  get:
    summary: Get container details
    operationId: getContainer
    tags: [Storage]
    parameters:
      - $ref: '#/components/parameters/ContainerId'
    responses:
      200:
        description: Container details
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/StorageContainer'

/containers/{containerId}/slots:
  get:
    summary: Get container slot status
    operationId: getContainerSlots
    tags: [Storage]
    parameters:
      - $ref: '#/components/parameters/ContainerId'
    responses:
      200:
        description: Slot information
        content:
          application/json:
            schema:
              type: object
              properties:
                totalSlots: { type: integer }
                occupiedSlots: { type: integer }
                availableSlots: { type: integer }
                slots:
                  type: array
                  items:
                    $ref: '#/components/schemas/StorageSlot'
```

### 3.2 Monitoring

```yaml
/containers/{containerId}/monitoring:
  get:
    summary: Get container monitoring data
    operationId: getContainerMonitoring
    tags: [Storage, Monitoring]
    parameters:
      - $ref: '#/components/parameters/ContainerId'
      - $ref: '#/components/parameters/DateRange'
      - name: resolution
        in: query
        description: Data point resolution
        schema:
          type: string
          enum: [1min, 5min, 15min, 1hour, 1day]
          default: 1hour
    responses:
      200:
        description: Monitoring data
        content:
          application/json:
            schema:
              type: object
              properties:
                containerId: { type: string }
                readings:
                  type: array
                  items:
                    $ref: '#/components/schemas/MonitoringRecord'
                statistics:
                  $ref: '#/components/schemas/MonitoringStatistics'

/containers/{containerId}/monitoring/live:
  get:
    summary: WebSocket endpoint for live monitoring
    operationId: liveMonitoring
    tags: [Storage, Monitoring]
    parameters:
      - $ref: '#/components/parameters/ContainerId'
    responses:
      101:
        description: Switching to WebSocket protocol
```

### 3.3 Alerts

```yaml
/containers/{containerId}/alerts:
  get:
    summary: Get container alerts
    operationId: getContainerAlerts
    tags: [Storage, Alerts]
    parameters:
      - $ref: '#/components/parameters/ContainerId'
      - name: status
        in: query
        schema:
          type: string
          enum: [ACTIVE, ACKNOWLEDGED, RESOLVED]
      - name: level
        in: query
        schema:
          $ref: '#/components/schemas/AlertLevel'
    responses:
      200:
        description: Alert list
        content:
          application/json:
            schema:
              type: array
              items:
                $ref: '#/components/schemas/Alert'

/alerts/{alertId}/acknowledge:
  post:
    summary: Acknowledge an alert
    operationId: acknowledgeAlert
    tags: [Alerts]
    parameters:
      - name: alertId
        in: path
        required: true
        schema: { type: string }
    requestBody:
      content:
        application/json:
          schema:
            type: object
            properties:
              acknowledgedBy: { type: string }
              notes: { type: string }
    responses:
      200:
        description: Alert acknowledged
```

---

## 4. Protocol Management Endpoints

### 4.1 Preservation Protocols

```yaml
/protocols:
  get:
    summary: List available protocols
    operationId: listProtocols
    tags: [Protocols]
    parameters:
      - name: subjectType
        in: query
        schema:
          $ref: '#/components/schemas/SubjectType'
      - name: species
        in: query
        schema:
          $ref: '#/components/schemas/SpeciesCode'
    responses:
      200:
        description: Protocol list
        content:
          application/json:
            schema:
              type: array
              items:
                $ref: '#/components/schemas/ProtocolSummary'

  post:
    summary: Create new protocol
    operationId: createProtocol
    tags: [Protocols]
    requestBody:
      required: true
      content:
        application/json:
          schema:
            $ref: '#/components/schemas/PreservationProtocol'
    responses:
      201:
        description: Protocol created

/protocols/{protocolId}:
  get:
    summary: Get protocol details
    operationId: getProtocol
    tags: [Protocols]
    parameters:
      - name: protocolId
        in: path
        required: true
        schema: { type: string }
    responses:
      200:
        description: Protocol details
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/PreservationProtocol'

/protocols/{protocolId}/validate:
  post:
    summary: Validate protocol against standards
    operationId: validateProtocol
    tags: [Protocols]
    parameters:
      - name: protocolId
        in: path
        required: true
        schema: { type: string }
    responses:
      200:
        description: Validation results
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/ValidationResult'
```

---

## 5. Transfer Endpoints

### 5.1 Transfer Management

```yaml
/transfers:
  get:
    summary: List transfers
    operationId: listTransfers
    tags: [Transfers]
    parameters:
      - name: status
        in: query
        schema:
          type: string
          enum: [PENDING, IN_TRANSIT, COMPLETED, CANCELLED]
      - name: direction
        in: query
        schema:
          type: string
          enum: [INCOMING, OUTGOING]
    responses:
      200:
        description: Transfer list

  post:
    summary: Initiate transfer
    operationId: initiateTransfer
    tags: [Transfers]
    requestBody:
      required: true
      content:
        application/json:
          schema:
            $ref: '#/components/schemas/TransferRequest'
    responses:
      201:
        description: Transfer initiated
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/TransferManifest'
      400:
        description: Transfer not feasible
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/TransferFeasibility'

/transfers/{transferId}:
  get:
    summary: Get transfer details
    operationId: getTransfer
    tags: [Transfers]
    parameters:
      - name: transferId
        in: path
        required: true
        schema: { type: string }
    responses:
      200:
        description: Transfer details
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/TransferManifest'

/transfers/{transferId}/tracking:
  get:
    summary: Get transfer tracking updates
    operationId: getTransferTracking
    tags: [Transfers]
    parameters:
      - name: transferId
        in: path
        required: true
        schema: { type: string }
    responses:
      200:
        description: Tracking history
        content:
          application/json:
            schema:
              type: array
              items:
                $ref: '#/components/schemas/TransferUpdate'

/transfers/{transferId}/confirm-receipt:
  post:
    summary: Confirm transfer receipt at destination
    operationId: confirmTransferReceipt
    tags: [Transfers]
    parameters:
      - name: transferId
        in: path
        required: true
        schema: { type: string }
    requestBody:
      content:
        application/json:
          schema:
            $ref: '#/components/schemas/ReceiptConfirmation'
    responses:
      200:
        description: Receipt confirmed
```

---

## 6. Quality Assessment Endpoints

### 6.1 VQI Assessment

```yaml
/subjects/{subjectId}/quality:
  get:
    summary: Get quality assessments for subject
    operationId: getSubjectQuality
    tags: [Quality]
    parameters:
      - $ref: '#/components/parameters/SubjectId'
    responses:
      200:
        description: Quality assessments
        content:
          application/json:
            schema:
              type: array
              items:
                $ref: '#/components/schemas/VQIAssessment'

  post:
    summary: Submit new quality assessment
    operationId: submitQualityAssessment
    tags: [Quality]
    parameters:
      - $ref: '#/components/parameters/SubjectId'
    requestBody:
      required: true
      content:
        application/json:
          schema:
            $ref: '#/components/schemas/VQIAssessmentInput'
    responses:
      201:
        description: Assessment recorded
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/VQIAssessment'

/subjects/{subjectId}/quality/latest:
  get:
    summary: Get latest VQI score
    operationId: getLatestVQI
    tags: [Quality]
    parameters:
      - $ref: '#/components/parameters/SubjectId'
    responses:
      200:
        description: Latest VQI
        content:
          application/json:
            schema:
              type: object
              properties:
                vqi: { type: number }
                grade: { type: string }
                assessmentDate: { type: string, format: date-time }
```

---

## 7. WebSocket Protocol

### 7.1 Connection

```typescript
// WebSocket endpoint: wss://api.cryo.wia.org/v1/ws

interface WebSocketMessage {
    type: MessageType;
    payload: any;
    timestamp: string;
    correlationId?: string;
}

type MessageType =
    | "AUTH"
    | "AUTH_ACK"
    | "SUBSCRIBE"
    | "UNSUBSCRIBE"
    | "MONITORING_UPDATE"
    | "ALERT"
    | "TRANSFER_UPDATE"
    | "PING"
    | "PONG"
    | "ERROR";
```

### 7.2 Subscription Topics

```typescript
// Subscribe to container monitoring
{
    type: "SUBSCRIBE",
    payload: {
        topic: "container.monitoring",
        containerId: "container-123",
        interval: 60  // seconds
    }
}

// Subscribe to facility alerts
{
    type: "SUBSCRIBE",
    payload: {
        topic: "facility.alerts",
        levels: ["WARNING", "CRITICAL", "EMERGENCY"]
    }
}

// Subscribe to transfer tracking
{
    type: "SUBSCRIBE",
    payload: {
        topic: "transfer.tracking",
        transferId: "transfer-456"
    }
}
```

### 7.3 Real-time Updates

```typescript
// Monitoring update
{
    type: "MONITORING_UPDATE",
    payload: {
        containerId: "container-123",
        timestamp: "2024-01-15T10:30:00Z",
        temperature: -196.2,
        ln2Level: 85.5,
        status: "NORMAL"
    }
}

// Alert notification
{
    type: "ALERT",
    payload: {
        alertId: "alert-789",
        containerId: "container-123",
        level: "WARNING",
        message: "LN2 level below 80%",
        timestamp: "2024-01-15T10:30:00Z"
    }
}
```

---

## 8. Inter-Facility Protocol

### 8.1 Facility Discovery

```yaml
/facilities:
  get:
    summary: List registered facilities
    operationId: listFacilities
    tags: [Facilities]
    parameters:
      - name: status
        in: query
        schema:
          type: string
          enum: [OPERATIONAL, MAINTENANCE, OFFLINE]
      - name: capability
        in: query
        schema:
          type: array
          items:
            $ref: '#/components/schemas/FacilityCapability'
      - name: location
        in: query
        description: Geographic filter (lat,lng,radius_km)
        schema: { type: string }
    responses:
      200:
        description: Facility list
        content:
          application/json:
            schema:
              type: array
              items:
                $ref: '#/components/schemas/FacilityInfo'

/facilities/{facilityId}:
  get:
    summary: Get facility details
    operationId: getFacility
    tags: [Facilities]
    parameters:
      - name: facilityId
        in: path
        required: true
        schema: { type: string }
    responses:
      200:
        description: Facility details
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/FacilityDocument'
```

### 8.2 Facility-to-Facility Communication

```typescript
interface FacilityMessage {
    messageId: string;
    type: FacilityMessageType;
    from: FacilityId;
    to: FacilityId;
    timestamp: string;
    payload: any;
    signature: DigitalSignature;
}

type FacilityMessageType =
    | "TRANSFER_REQUEST"
    | "TRANSFER_ACCEPT"
    | "TRANSFER_REJECT"
    | "TRANSFER_STATUS"
    | "EMERGENCY_ALERT"
    | "CAPACITY_QUERY"
    | "CAPACITY_RESPONSE"
    | "PROTOCOL_SYNC"
    | "HEARTBEAT";
```

---

## 9. Emergency Endpoints

### 9.1 Emergency Declaration

```yaml
/emergency:
  post:
    summary: Declare emergency
    operationId: declareEmergency
    tags: [Emergency]
    requestBody:
      required: true
      content:
        application/json:
          schema:
            type: object
            required: [type, severity, description]
            properties:
              type:
                $ref: '#/components/schemas/EmergencyType'
              severity:
                type: string
                enum: [WARNING, CRITICAL, CATASTROPHIC]
              description: { type: string }
              affectedContainers:
                type: array
                items: { type: string }
              affectedSubjects:
                type: array
                items: { type: string }
    responses:
      201:
        description: Emergency declared
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/EmergencyResponse'

  get:
    summary: Get active emergencies
    operationId: getActiveEmergencies
    tags: [Emergency]
    responses:
      200:
        description: Active emergency list

/emergency/{emergencyId}/resolve:
  post:
    summary: Resolve emergency
    operationId: resolveEmergency
    tags: [Emergency]
    parameters:
      - name: emergencyId
        in: path
        required: true
        schema: { type: string }
    requestBody:
      content:
        application/json:
          schema:
            type: object
            properties:
              resolution: { type: string }
              actions: { type: array, items: { type: string } }
    responses:
      200:
        description: Emergency resolved
```

---

## 10. Error Responses

```yaml
components:
  responses:
    ValidationError:
      description: Validation failed
      content:
        application/json:
          schema:
            type: object
            properties:
              error: { type: string, example: "VALIDATION_ERROR" }
              message: { type: string }
              details:
                type: array
                items:
                  type: object
                  properties:
                    field: { type: string }
                    message: { type: string }

    NotFound:
      description: Resource not found
      content:
        application/json:
          schema:
            type: object
            properties:
              error: { type: string, example: "NOT_FOUND" }
              message: { type: string }

    Unauthorized:
      description: Authentication required
      content:
        application/json:
          schema:
            type: object
            properties:
              error: { type: string, example: "UNAUTHORIZED" }
              message: { type: string }

    Forbidden:
      description: Insufficient permissions
      content:
        application/json:
          schema:
            type: object
            properties:
              error: { type: string, example: "FORBIDDEN" }
              message: { type: string }
```

---

**Phase 3 Protocol Specification**
**WIA-CRYO-PRESERVATION v1.0.0**
