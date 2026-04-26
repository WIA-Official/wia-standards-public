# WIA Digital Twin City Integration Standard
## Phase 4 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #0EA5E9 (Sky)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [System Architecture](#system-architecture)
4. [Third-party Integrations](#third-party-integrations)
5. [IoT Platform Integration](#iot-platform-integration)
6. [GIS System Integration](#gis-system-integration)
7. [Deployment Models](#deployment-models)
8. [Certification Process](#certification-process)
9. [Migration Guide](#migration-guide)
10. [Best Practices](#best-practices)
11. [References](#references)

---

## Overview

### 1.1 Purpose

The WIA Digital Twin City Integration Standard defines the complete ecosystem integration framework for deploying digital twin city platforms. This Phase 4 specification builds upon Phases 1-3, providing comprehensive guidance for integrating with IoT platforms, GIS systems, traffic management, emergency services, and citizen applications.

**Core Objectives**:
- Define integration patterns for smart city ecosystems
- Enable interoperability with existing city systems
- Support multiple deployment models (cloud, edge, on-premise)
- Provide certification process for compliant implementations
- Ensure scalability to city-scale operations

### 1.2 Scope

This standard defines:

| Component | Description |
|-----------|-------------|
| **Architecture** | System design patterns and components |
| **Integrations** | Third-party platform connectors |
| **Deployment** | Cloud, edge, and hybrid deployment models |
| **Certification** | Compliance testing and validation |
| **Migration** | Legacy system integration strategies |
| **Operations** | Monitoring, maintenance, and scaling |

### 1.3 Integration Layers

```
┌─────────────────────────────────────────────────────────────┐
│               Citizen Services Layer                        │
│  (Mobile Apps, Web Portals, Public Dashboards)             │
├─────────────────────────────────────────────────────────────┤
│               Application Layer                             │
│  (Traffic Management, Energy Management, Emergency)         │
├─────────────────────────────────────────────────────────────┤
│               Digital Twin Core Layer                       │
│  (WIA Digital Twin City Platform)                          │
├─────────────────────────────────────────────────────────────┤
│               Integration Layer                             │
│  (IoT Platforms, GIS Systems, External APIs)               │
├─────────────────────────────────────────────────────────────┤
│               Infrastructure Layer                          │
│  (Cloud, Edge Devices, On-premise Servers)                 │
└─────────────────────────────────────────────────────────────┘
```

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Integration Adapter** | Component connecting external systems to digital twin |
| **Edge Gateway** | Local processing node for IoT devices |
| **Federation** | Multi-city digital twin coordination |
| **Data Lake** | Centralized repository for historical data |
| **ETL Pipeline** | Extract-Transform-Load data processing |
| **Service Mesh** | Microservice communication infrastructure |

### 2.2 Integration Patterns

| Pattern | Description | Use Case |
|---------|-------------|----------|
| **Adapter** | Translate external data formats | Legacy system integration |
| **Gateway** | Aggregate multiple data sources | IoT device management |
| **Broker** | Message routing and transformation | Event-driven architecture |
| **Proxy** | Access control and rate limiting | Public API exposure |
| **Cache** | Performance optimization | High-traffic endpoints |

---

## System Architecture

### 3.1 Reference Architecture

```
                         ┌─────────────────────────────────────┐
                         │         Load Balancer               │
                         └────────────┬────────────────────────┘
                                      │
                    ┌─────────────────┼─────────────────┐
                    │                 │                 │
            ┌───────▼──────┐  ┌──────▼──────┐  ┌──────▼──────┐
            │   API        │  │   API       │  │   API       │
            │   Gateway 1  │  │   Gateway 2 │  │   Gateway 3 │
            └───────┬──────┘  └──────┬──────┘  └──────┬──────┘
                    │                │                 │
            ┌───────┴────────────────┴─────────────────┴───────┐
            │              Service Mesh                         │
            └───────┬──────────────────────────────────────────┘
                    │
        ┌───────────┼───────────────────────────────┐
        │           │                               │
┌───────▼──────┐ ┌──▼──────────┐ ┌────────────┐ ┌─▼──────────┐
│   City       │ │   Sensor    │ │ Simulation │ │   Service  │
│   Object     │ │   Data      │ │   Engine   │ │   Manager  │
│   Service    │ │   Service   │ │            │ │            │
└───────┬──────┘ └──┬──────────┘ └─────┬──────┘ └─┬──────────┘
        │           │                   │           │
        └───────────┼───────────────────┼───────────┘
                    │                   │
            ┌───────▼───────────────────▼──────┐
            │        Data Layer                │
            ├──────────────────────────────────┤
            │  Time-series DB  │  Spatial DB   │
            ├──────────────────┼───────────────┤
            │  Object Store    │  Cache Layer  │
            └──────────────────┴───────────────┘
                    │
            ┌───────▼──────────────────┐
            │   Integration Layer      │
            ├──────────────────────────┤
            │ IoT     GIS    Traffic   │
            │ Platform System  System  │
            └──────────────────────────┘
```

### 3.2 Core Components

#### 3.2.1 API Gateway

**Responsibilities**:
- Request routing and load balancing
- Authentication and authorization
- Rate limiting and throttling
- Request/response transformation
- API versioning

**Configuration**:
```yaml
apiGateway:
  instances: 3
  loadBalancer:
    type: round-robin
    healthCheck:
      interval: 10s
      timeout: 5s
  rateLimit:
    global: 10000/minute
    perClient: 1000/minute
  cors:
    enabled: true
    origins: ["*"]
  cache:
    enabled: true
    ttl: 300s
```

#### 3.2.2 Service Mesh

**Responsibilities**:
- Service discovery
- Load balancing
- Circuit breaking
- Distributed tracing
- mTLS encryption

**Technology**: Istio, Linkerd, or Consul

```yaml
serviceMesh:
  type: istio
  mtls:
    mode: STRICT
  loadBalancing:
    algorithm: LEAST_REQUEST
  circuitBreaker:
    consecutiveErrors: 5
    interval: 30s
    baseEjectionTime: 30s
```

#### 3.2.3 Data Layer

**Time-series Database**:
- Technology: InfluxDB, TimescaleDB
- Purpose: Sensor data, metrics
- Retention: 90 days hot, 2 years cold

**Spatial Database**:
- Technology: PostGIS, MongoDB with geospatial
- Purpose: City objects, geometry
- Indexing: R-tree for spatial queries

**Object Storage**:
- Technology: MinIO, S3
- Purpose: 3D models, images, documents
- Redundancy: 3x replication

**Cache Layer**:
- Technology: Redis, Memcached
- Purpose: Frequently accessed data
- TTL: 5-300 seconds

```yaml
dataLayer:
  timeseries:
    type: influxdb
    retention:
      hot: 90d
      cold: 730d
    replication: 3
  spatial:
    type: postgis
    indexes:
      - type: rtree
        fields: [geometry]
      - type: btree
        fields: [cityObjectId, type]
  objectStorage:
    type: minio
    buckets:
      - name: city-models
        versioning: true
      - name: sensor-data
        lifecycle: 30d
  cache:
    type: redis
    cluster: true
    nodes: 6
    ttl:
      default: 60s
      cityObjects: 300s
      sensorData: 5s
```

### 3.3 Microservice Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                    API Gateway Layer                         │
└────────────────────────┬─────────────────────────────────────┘
                         │
        ┌────────────────┼────────────────┐
        │                │                │
┌───────▼──────┐  ┌─────▼──────┐  ┌─────▼──────┐
│   Auth       │  │   City     │  │   Sensor   │
│   Service    │  │   Object   │  │   Service  │
└───────┬──────┘  │   Service  │  └─────┬──────┘
        │         └─────┬──────┘        │
        │               │               │
┌───────▼──────┐  ┌─────▼──────┐  ┌─────▼──────┐
│   User       │  │   Spatial  │  │   Stream   │
│   Service    │  │   Query    │  │   Processor│
└──────────────┘  │   Service  │  └─────┬──────┘
                  └─────┬──────┘        │
                        │               │
                  ┌─────▼──────┐  ┌─────▼──────┐
                  │   GIS      │  │   IoT      │
                  │   Adapter  │  │   Gateway  │
                  └────────────┘  └────────────┘
```

**Service Communication**:
- Synchronous: gRPC for internal services
- Asynchronous: MQTT/Kafka for events
- API: REST/GraphQL for external clients

---

## Third-party Integrations

### 4.1 Integration Framework

```typescript
interface IntegrationAdapter {
  // Metadata
  id: string;
  name: string;
  version: string;
  vendor: string;

  // Lifecycle
  initialize(config: AdapterConfig): Promise<void>;
  connect(): Promise<ConnectionStatus>;
  disconnect(): Promise<void>;
  healthCheck(): Promise<HealthStatus>;

  // Data flow
  ingest(data: any): Promise<void>;
  export(query: ExportQuery): Promise<any>;
  transform(data: any, mapping: DataMapping): any;

  // Event handling
  on(event: string, handler: EventHandler): void;
  emit(event: string, data: any): void;
}
```

### 4.2 Common Integrations

| System | Purpose | Protocol | Adapter |
|--------|---------|----------|---------|
| AWS IoT Core | IoT device management | MQTT | aws-iot-adapter |
| Azure IoT Hub | IoT device management | MQTT, AMQP | azure-iot-adapter |
| Google Cloud IoT | IoT device management | MQTT, HTTP | gcp-iot-adapter |
| Esri ArcGIS | GIS and mapping | REST, WFS | arcgis-adapter |
| OpenStreetMap | Mapping data | Overpass API | osm-adapter |
| HERE Maps | Traffic and routing | REST | here-adapter |
| Siemens MindSphere | Industrial IoT | OPC UA | mindsphere-adapter |
| SAP HANA | Enterprise data | ODBC, REST | sap-adapter |

### 4.3 Adapter Configuration

```yaml
integrations:
  - id: aws-iot-adapter
    name: AWS IoT Core Integration
    version: 1.0.0
    enabled: true
    config:
      region: ap-northeast-2
      endpoint: a3example.iot.ap-northeast-2.amazonaws.com
      credentials:
        accessKeyId: ${AWS_ACCESS_KEY_ID}
        secretAccessKey: ${AWS_SECRET_ACCESS_KEY}
      topics:
        - topic: sensors/+/temperature
          qos: 1
          mapping:
            source: payload.temperature
            target: measurements[0].value
      dataMapping:
        timestampField: timestamp
        sensorIdField: deviceId
        transform:
          - type: unit-conversion
            from: fahrenheit
            to: celsius

  - id: arcgis-adapter
    name: Esri ArcGIS Integration
    version: 1.0.0
    enabled: true
    config:
      url: https://services.arcgis.com/your-org
      authType: oauth
      credentials:
        clientId: ${ARCGIS_CLIENT_ID}
        clientSecret: ${ARCGIS_CLIENT_SECRET}
      layers:
        - id: buildings-layer
          type: FeatureLayer
          url: /buildings/FeatureServer/0
          sync:
            interval: 1h
            incremental: true
      coordinateSystem:
        source: EPSG:3857
        target: EPSG:4326
```

---

## IoT Platform Integration

### 5.1 AWS IoT Core Integration

```typescript
import { IoTClient, DescribeThingCommand } from '@aws-sdk/client-iot';
import { IoTDataPlaneClient, PublishCommand } from '@aws-sdk/client-iot-data-plane';
import { mqtt5, iot } from 'aws-iot-device-sdk-v2';

class AWSIoTAdapter implements IntegrationAdapter {
  private iotClient: IoTClient;
  private iotDataClient: IoTDataPlaneClient;
  private mqttConnection: mqtt5.Mqtt5Client;

  async initialize(config: AWSIoTConfig): Promise<void> {
    // Initialize AWS SDK clients
    this.iotClient = new IoTClient({
      region: config.region,
      credentials: config.credentials
    });

    this.iotDataClient = new IoTDataPlaneClient({
      region: config.region,
      endpoint: config.endpoint,
      credentials: config.credentials
    });

    // Set up MQTT connection
    const mqttConfig = iot.AwsIotMqtt5ClientConfigBuilder
      .newDirectMqttBuilderWithMtlsFromPath(
        config.endpoint,
        config.certPath,
        config.keyPath
      )
      .withConnectProperties({
        keepAliveIntervalSeconds: 60,
        clientId: config.clientId
      })
      .build();

    this.mqttConnection = new mqtt5.Mqtt5Client(mqttConfig);
  }

  async connect(): Promise<ConnectionStatus> {
    await this.mqttConnection.start();

    // Subscribe to sensor topics
    this.mqttConnection.on('messageReceived', (eventData) => {
      const topic = eventData.message.topicName;
      const payload = JSON.parse(
        new TextDecoder().decode(eventData.message.payload)
      );

      this.handleSensorData(topic, payload);
    });

    await this.mqttConnection.subscribe({
      subscriptions: [
        { topicFilter: 'sensors/+/temperature', qos: 1 },
        { topicFilter: 'sensors/+/humidity', qos: 1 }
      ]
    });

    return { connected: true, timestamp: new Date() };
  }

  private handleSensorData(topic: string, payload: any): void {
    // Transform AWS IoT format to WIA format
    const wiaMessage = this.transform(payload, {
      sensorId: payload.deviceId,
      measurements: [
        {
          parameter: extractParameter(topic),
          value: payload.value,
          unit: payload.unit,
          timestamp: payload.timestamp,
          quality: 'good'
        }
      ]
    });

    // Ingest into Digital Twin City
    this.emit('sensor:data', wiaMessage);
  }
}
```

### 5.2 Azure IoT Hub Integration

```python
from azure.iot.hub import IoTHubRegistryManager
from azure.iot.device import IoTHubDeviceClient, Message
import asyncio

class AzureIoTAdapter:
    def __init__(self, config):
        self.connection_string = config['connectionString']
        self.registry_manager = IoTHubRegistryManager(self.connection_string)
        self.device_clients = {}

    async def initialize(self):
        # Get all devices
        devices = self.registry_manager.get_devices()

        for device in devices:
            # Create device client for each device
            device_client = IoTHubDeviceClient.create_from_connection_string(
                device.connection_string
            )
            await device_client.connect()

            # Set message handler
            device_client.on_message_received = self.message_handler

            self.device_clients[device.device_id] = device_client

    async def message_handler(self, message):
        # Parse message
        data = json.loads(message.data.decode('utf-8'))

        # Transform to WIA format
        wia_message = {
            'sensorId': message.custom_properties.get('sensorId'),
            'measurements': [{
                'parameter': data.get('parameter'),
                'value': data.get('value'),
                'unit': data.get('unit'),
                'timestamp': datetime.now().isoformat(),
                'quality': 'good'
            }]
        }

        # Ingest into Digital Twin City
        await self.ingest_sensor_data(wia_message)

    async def send_command(self, device_id, command):
        device_client = self.device_clients.get(device_id)
        if device_client:
            message = Message(json.dumps(command))
            await device_client.send_message(message)
```

### 5.3 Google Cloud IoT Integration

```typescript
import { DeviceManagerClient } from '@google-cloud/iot';
import { PubSub } from '@google-cloud/pubsub';

class GCPIoTAdapter implements IntegrationAdapter {
  private deviceManager: DeviceManagerClient;
  private pubsub: PubSub;
  private subscription: any;

  async initialize(config: GCPIoTConfig): Promise<void> {
    this.deviceManager = new DeviceManagerClient({
      projectId: config.projectId,
      keyFilename: config.keyFilename
    });

    this.pubsub = new PubSub({
      projectId: config.projectId,
      keyFilename: config.keyFilename
    });
  }

  async connect(): Promise<ConnectionStatus> {
    // Subscribe to telemetry topic
    const subscription = this.pubsub
      .topic(config.telemetryTopic)
      .subscription(config.subscriptionId);

    subscription.on('message', (message) => {
      const data = JSON.parse(message.data.toString());
      const deviceId = message.attributes.deviceId;

      this.handleTelemetry(deviceId, data);
      message.ack();
    });

    subscription.on('error', (error) => {
      console.error('Subscription error:', error);
    });

    this.subscription = subscription;

    return { connected: true, timestamp: new Date() };
  }

  private handleTelemetry(deviceId: string, data: any): void {
    const wiaMessage = {
      sensorId: deviceId,
      measurements: data.measurements.map(m => ({
        parameter: m.type,
        value: m.value,
        unit: m.unit,
        timestamp: new Date().toISOString(),
        quality: 'good'
      }))
    };

    this.emit('sensor:data', wiaMessage);
  }
}
```

---

## GIS System Integration

### 6.1 Esri ArcGIS Integration

```typescript
import esri from '@arcgis/core';
import FeatureLayer from '@arcgis/core/layers/FeatureLayer';
import Query from '@arcgis/core/rest/support/Query';

class ArcGISAdapter implements IntegrationAdapter {
  private portal: esri.Portal;
  private featureLayers: Map<string, FeatureLayer>;

  async initialize(config: ArcGISConfig): Promise<void> {
    // Authenticate
    this.portal = new esri.Portal({
      url: config.portalUrl,
      authMode: 'immediate'
    });

    await this.portal.load();

    // Initialize feature layers
    this.featureLayers = new Map();

    for (const layerConfig of config.layers) {
      const layer = new FeatureLayer({
        url: layerConfig.url,
        outFields: ['*']
      });

      await layer.load();
      this.featureLayers.set(layerConfig.id, layer);
    }
  }

  async syncBuildings(): Promise<void> {
    const buildingsLayer = this.featureLayers.get('buildings-layer');

    // Query all buildings
    const query = new Query({
      where: '1=1',
      outFields: ['*'],
      returnGeometry: true
    });

    const result = await buildingsLayer.queryFeatures(query);

    // Transform to WIA format
    for (const feature of result.features) {
      const wiaBuilding = {
        id: `building-${feature.attributes.OBJECTID}`,
        type: 'Building',
        geometry: this.convertGeometry(feature.geometry),
        properties: {
          name: feature.attributes.NAME,
          address: feature.attributes.ADDRESS,
          buildingType: feature.attributes.TYPE,
          floors: {
            above: feature.attributes.FLOORS,
            below: feature.attributes.BASEMENTS
          },
          height: feature.attributes.HEIGHT
        }
      };

      // Ingest into Digital Twin City
      await this.ingestCityObject(wiaBuilding);
    }
  }

  private convertGeometry(arcgisGeometry: any): any {
    // Convert ArcGIS geometry to GeoJSON
    if (arcgisGeometry.type === 'polygon') {
      return {
        type: 'Polygon',
        coordinates: arcgisGeometry.rings
      };
    }
    // ... handle other geometry types
  }
}
```

### 6.2 OpenStreetMap Integration

```python
import overpy
from shapely.geometry import shape, mapping
import json

class OSMAdapter:
    def __init__(self, config):
        self.api = overpy.Overpass()
        self.bbox = config['bbox']

    async def sync_roads(self):
        # Query roads in bounding box
        query = f"""
        [out:json];
        way["highway"]({self.bbox['south']},{self.bbox['west']},
                        {self.bbox['north']},{self.bbox['east']});
        (._;>;);
        out body;
        """

        result = self.api.query(query)

        for way in result.ways:
            # Convert to WIA format
            coordinates = [[node.lon, node.lat] for node in way.nodes]

            wia_road = {
                'id': f'road-osm-{way.id}',
                'type': 'Road',
                'geometry': {
                    'type': 'LineString',
                    'coordinates': coordinates
                },
                'properties': {
                    'name': way.tags.get('name', ''),
                    'roadType': way.tags.get('highway', ''),
                    'lanes': int(way.tags.get('lanes', 1)),
                    'maxSpeed': way.tags.get('maxspeed', ''),
                    'surface': way.tags.get('surface', '')
                },
                'metadata': {
                    'dataSource': 'openstreetmap',
                    'osmId': way.id,
                    'lastUpdated': datetime.now().isoformat()
                }
            }

            # Ingest into Digital Twin City
            await self.ingest_city_object(wia_road)

    async def sync_buildings(self):
        query = f"""
        [out:json];
        way["building"]({self.bbox['south']},{self.bbox['west']},
                        {self.bbox['north']},{self.bbox['east']});
        (._;>;);
        out body;
        """

        result = self.api.query(query)

        for way in result.ways:
            coordinates = [[[node.lon, node.lat] for node in way.nodes]]

            wia_building = {
                'id': f'building-osm-{way.id}',
                'type': 'Building',
                'geometry': {
                    'type': 'Polygon',
                    'coordinates': coordinates
                },
                'properties': {
                    'buildingType': way.tags.get('building', 'yes'),
                    'name': way.tags.get('name', ''),
                    'address': way.tags.get('addr:full', ''),
                    'floors': {
                        'above': int(way.tags.get('building:levels', 1))
                    }
                }
            }

            await self.ingest_city_object(wia_building)
```

---

## Deployment Models

### 7.1 Cloud Deployment

```yaml
# Kubernetes Deployment Configuration
apiVersion: apps/v1
kind: Deployment
metadata:
  name: digital-twin-city
  namespace: smart-city
spec:
  replicas: 3
  selector:
    matchLabels:
      app: digital-twin-city
  template:
    metadata:
      labels:
        app: digital-twin-city
    spec:
      containers:
      - name: api-gateway
        image: wia/digital-twin-city-gateway:1.0.0
        ports:
        - containerPort: 8080
        env:
        - name: CITY_ID
          value: "KR-SEOUL-2025"
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: db-credentials
              key: url
        resources:
          requests:
            cpu: 500m
            memory: 1Gi
          limits:
            cpu: 2000m
            memory: 4Gi
        livenessProbe:
          httpGet:
            path: /health
            port: 8080
          initialDelaySeconds: 30
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /ready
            port: 8080
          initialDelaySeconds: 10
          periodSeconds: 5

---
apiVersion: v1
kind: Service
metadata:
  name: digital-twin-city-service
  namespace: smart-city
spec:
  selector:
    app: digital-twin-city
  ports:
  - protocol: TCP
    port: 80
    targetPort: 8080
  type: LoadBalancer

---
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: digital-twin-city-hpa
  namespace: smart-city
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: digital-twin-city
  minReplicas: 3
  maxReplicas: 20
  metrics:
  - type: Resource
    resource:
      name: cpu
      target:
        type: Utilization
        averageUtilization: 70
  - type: Resource
    resource:
      name: memory
      target:
        type: Utilization
        averageUtilization: 80
```

**Cloud Providers**:
- AWS: EKS, RDS, S3, IoT Core
- Azure: AKS, Cosmos DB, Blob Storage, IoT Hub
- GCP: GKE, Cloud SQL, Cloud Storage, Cloud IoT

### 7.2 Edge Deployment

```yaml
# Edge Gateway Configuration
edge:
  location:
    lat: 37.4979
    lon: 127.0276
    district: gangnam

  hardware:
    cpu: ARM64 8-core
    memory: 16GB
    storage: 512GB SSD
    network: 5G + Ethernet

  software:
    os: Ubuntu 22.04
    runtime: Docker + K3s
    databases:
      - type: sqlite
        purpose: local-cache
      - type: redis
        purpose: real-time-buffer

  sensors:
    - id: env-001
      type: environmental
      protocol: modbus
      interface: /dev/ttyUSB0
    - id: trf-001
      type: traffic
      protocol: mqtt
      broker: local

  connectivity:
    primary:
      type: 5G
      apn: iot.carrier.com
    backup:
      type: ethernet
      failover: auto

  dataFlow:
    # Local processing
    localProcessing:
      enabled: true
      rules:
        - trigger: high-aqi
          threshold: 100
          action: local-alert
    # Cloud sync
    cloudSync:
      enabled: true
      interval: 5m
      batch: 1000
      compression: true
```

### 7.3 Hybrid Deployment

```
┌─────────────────────────────────────────────────────────┐
│                    Cloud Layer                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ City-wide    │  │ Long-term    │  │ ML Training  │  │
│  │ Dashboard    │  │ Storage      │  │ & Analysis   │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
└─────────────────────────┬───────────────────────────────┘
                          │ API / MQTT
                          │
┌─────────────────────────▼───────────────────────────────┐
│                    Regional Edge                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ District     │  │ Medium-term  │  │ Aggregation  │  │
│  │ Coordination │  │ Storage      │  │ & Filtering  │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
└─────────────────────────┬───────────────────────────────┘
                          │ Local Network
                          │
┌─────────────────────────▼───────────────────────────────┐
│                    Local Edge Gateways                  │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ Gateway 1    │  │ Gateway 2    │  │ Gateway 3    │  │
│  │ (Gangnam)    │  │ (Jung-gu)    │  │ (Songpa)     │  │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  │
└─────────┼──────────────────┼──────────────────┼─────────┘
          │                  │                  │
     ┌────▼────┐        ┌────▼────┐        ┌────▼────┐
     │ Sensors │        │ Sensors │        │ Sensors │
     │  (50)   │        │  (50)   │        │  (50)   │
     └─────────┘        └─────────┘        └─────────┘
```

**Data Flow**:
1. Sensors → Local Edge Gateway (immediate processing)
2. Local Edge → Regional Edge (5min aggregation)
3. Regional Edge → Cloud (hourly batch)
4. Cloud → Long-term Storage (daily archival)

### 7.4 On-premise Deployment

```yaml
# Docker Compose for On-premise Deployment
version: '3.8'

services:
  # Load Balancer
  nginx:
    image: nginx:alpine
    ports:
      - "80:80"
      - "443:443"
    volumes:
      - ./nginx.conf:/etc/nginx/nginx.conf
      - ./ssl:/etc/nginx/ssl
    depends_on:
      - api-gateway

  # API Gateway
  api-gateway:
    image: wia/digital-twin-city-gateway:1.0.0
    environment:
      - CITY_ID=KR-SEOUL-2025
      - DATABASE_URL=postgresql://postgres:5432/dtc
      - REDIS_URL=redis://redis:6379
    ports:
      - "8080:8080"
    depends_on:
      - postgres
      - redis
      - mqtt-broker

  # Services
  city-object-service:
    image: wia/dtc-city-object-service:1.0.0
    environment:
      - DATABASE_URL=postgresql://postgres:5432/dtc
    depends_on:
      - postgres

  sensor-service:
    image: wia/dtc-sensor-service:1.0.0
    environment:
      - INFLUXDB_URL=http://influxdb:8086
      - MQTT_BROKER=mqtt://mqtt-broker:1883
    depends_on:
      - influxdb
      - mqtt-broker

  simulation-service:
    image: wia/dtc-simulation-service:1.0.0
    environment:
      - DATABASE_URL=postgresql://postgres:5432/dtc
    depends_on:
      - postgres

  # Databases
  postgres:
    image: postgis/postgis:15-3.3
    environment:
      - POSTGRES_DB=dtc
      - POSTGRES_USER=dtc_user
      - POSTGRES_PASSWORD=${DB_PASSWORD}
    volumes:
      - postgres-data:/var/lib/postgresql/data
    ports:
      - "5432:5432"

  influxdb:
    image: influxdb:2.7
    environment:
      - INFLUXDB_DB=sensor_data
      - INFLUXDB_ADMIN_USER=admin
      - INFLUXDB_ADMIN_PASSWORD=${INFLUXDB_PASSWORD}
    volumes:
      - influxdb-data:/var/lib/influxdb2
    ports:
      - "8086:8086"

  redis:
    image: redis:7-alpine
    volumes:
      - redis-data:/data
    ports:
      - "6379:6379"

  # MQTT Broker
  mqtt-broker:
    image: eclipse-mosquitto:2
    volumes:
      - ./mosquitto.conf:/mosquitto/config/mosquitto.conf
      - mosquitto-data:/mosquitto/data
    ports:
      - "1883:1883"
      - "8883:8883"

  # Monitoring
  prometheus:
    image: prom/prometheus:latest
    volumes:
      - ./prometheus.yml:/etc/prometheus/prometheus.yml
      - prometheus-data:/prometheus
    ports:
      - "9090:9090"

  grafana:
    image: grafana/grafana:latest
    environment:
      - GF_SECURITY_ADMIN_PASSWORD=${GRAFANA_PASSWORD}
    volumes:
      - grafana-data:/var/lib/grafana
    ports:
      - "3000:3000"
    depends_on:
      - prometheus

volumes:
  postgres-data:
  influxdb-data:
  redis-data:
  mosquitto-data:
  prometheus-data:
  grafana-data:
```

---

## Certification Process

### 8.1 Compliance Levels

| Level | Requirements | Testing | Badge |
|-------|-------------|---------|-------|
| **Bronze** | Phase 1 Data Format | Self-certification | 🥉 WIA DTC Bronze |
| **Silver** | Phase 1 + Phase 2 API | Automated testing | 🥈 WIA DTC Silver |
| **Gold** | Phase 1-3 | Third-party audit | 🥇 WIA DTC Gold |
| **Platinum** | Full stack + Performance | Comprehensive audit | 💎 WIA DTC Platinum |

### 8.2 Certification Process

```
1. Registration
   ├─ Submit application
   ├─ Provide system documentation
   └─ Pay certification fee

2. Self-Assessment
   ├─ Complete compliance checklist
   ├─ Run automated tests
   └─ Document results

3. Technical Review
   ├─ Code review
   ├─ Architecture review
   └─ Security assessment

4. Testing
   ├─ Functional testing
   ├─ Integration testing
   ├─ Performance testing
   └─ Security testing

5. Audit (Gold/Platinum only)
   ├─ On-site inspection
   ├─ Live system testing
   └─ Documentation review

6. Certification
   ├─ Issue certificate
   ├─ Grant badge usage rights
   └─ List in registry

7. Maintenance
   ├─ Annual renewal
   ├─ Continuous monitoring
   └─ Incident reporting
```

### 8.3 Test Suite

```bash
# WIA Digital Twin City Compliance Test Suite

# Phase 1: Data Format
npm run test:phase1
  ✓ JSON schema validation
  ✓ Required fields presence
  ✓ Data type correctness
  ✓ Coordinate system validation
  ✓ GeoJSON compliance
  ✓ Timestamp format
  ✓ Value range validation

# Phase 2: API Interface
npm run test:phase2
  ✓ REST endpoint availability
  ✓ Authentication mechanisms
  ✓ Request/response format
  ✓ Error handling
  ✓ Rate limiting
  ✓ Pagination
  ✓ Filtering and sorting

# Phase 3: Protocol
npm run test:phase3
  ✓ WebSocket connection
  ✓ MQTT connectivity
  ✓ Message format
  ✓ QoS levels
  ✓ Heartbeat mechanism
  ✓ Reconnection logic
  ✓ TLS encryption

# Phase 4: Integration
npm run test:phase4
  ✓ IoT platform connectivity
  ✓ GIS system integration
  ✓ Data synchronization
  ✓ Event propagation
  ✓ Performance benchmarks
  ✓ Scalability tests

# Performance Testing
npm run test:performance
  ✓ Throughput: 10,000 req/s
  ✓ Latency: p99 < 100ms
  ✓ Concurrent connections: 100,000+
  ✓ Data ingestion: 50,000 sensors
  ✓ Query response: < 500ms

# Security Testing
npm run test:security
  ✓ Authentication bypass attempts
  ✓ Authorization checks
  ✓ SQL injection resistance
  ✓ XSS protection
  ✓ CSRF protection
  ✓ Rate limit enforcement
  ✓ TLS configuration
```

---

## Migration Guide

### 9.1 Legacy System Migration

```typescript
// Migration Strategy
interface MigrationPlan {
  phases: MigrationPhase[];
  rollbackStrategy: RollbackStrategy;
  dataValidation: ValidationRules[];
}

const seoulMigration: MigrationPlan = {
  phases: [
    {
      name: 'Phase 1: Assessment',
      duration: '2 weeks',
      tasks: [
        'Inventory existing systems',
        'Map data schemas',
        'Identify integration points',
        'Assess data quality'
      ]
    },
    {
      name: 'Phase 2: Pilot',
      duration: '1 month',
      tasks: [
        'Deploy in single district',
        'Migrate 1000 buildings',
        'Integrate 50 sensors',
        'Validate data accuracy'
      ]
    },
    {
      name: 'Phase 3: Incremental Rollout',
      duration: '3 months',
      tasks: [
        'Expand to 5 districts',
        'Migrate 50,000 buildings',
        'Integrate 1,000 sensors',
        'Parallel run with legacy system'
      ]
    },
    {
      name: 'Phase 4: Full Migration',
      duration: '2 months',
      tasks: [
        'Complete city-wide deployment',
        'Migrate all buildings and infrastructure',
        'Integrate all sensors',
        'Decommission legacy system'
      ]
    }
  ],
  rollbackStrategy: {
    triggerConditions: [
      'Data quality < 95%',
      'System downtime > 1 hour',
      'Critical functionality failure'
    ],
    rollbackProcedure: 'Revert to legacy system within 15 minutes'
  },
  dataValidation: [
    {
      rule: 'geometry-integrity',
      threshold: 0.99,
      action: 'flag-for-review'
    },
    {
      rule: 'sensor-data-accuracy',
      threshold: 0.95,
      action: 'recalibrate'
    }
  ]
};
```

### 9.2 Data Migration Script

```python
import asyncio
from typing import List
from wia_digital_twin_city import DigitalTwinCity
from legacy_system import LegacyGISSystem

class DataMigrator:
    def __init__(self, legacy_db, wia_api):
        self.legacy_db = legacy_db
        self.wia_api = wia_api
        self.batch_size = 100

    async def migrate_buildings(self):
        offset = 0
        total_migrated = 0

        while True:
            # Fetch from legacy system
            buildings = await self.legacy_db.fetch_buildings(
                limit=self.batch_size,
                offset=offset
            )

            if not buildings:
                break

            # Transform to WIA format
            wia_buildings = [
                self.transform_building(b) for b in buildings
            ]

            # Validate
            valid_buildings = [
                b for b in wia_buildings
                if self.validate_building(b)
            ]

            # Ingest into WIA Digital Twin City
            results = await asyncio.gather(*[
                self.wia_api.create_city_object(b)
                for b in valid_buildings
            ])

            total_migrated += len(results)
            offset += self.batch_size

            print(f'Migrated {total_migrated} buildings...')

        print(f'Migration complete: {total_migrated} buildings')

    def transform_building(self, legacy_building):
        return {
            'type': 'Building',
            'geometry': {
                'type': 'Polygon',
                'coordinates': self.convert_coordinates(
                    legacy_building['geometry']
                )
            },
            'properties': {
                'name': legacy_building['name'],
                'address': legacy_building['address'],
                'buildingType': self.map_building_type(
                    legacy_building['type']
                ),
                'floors': {
                    'above': legacy_building['floors'],
                    'below': legacy_building.get('basements', 0)
                },
                'height': legacy_building['height']
            },
            'metadata': {
                'dataSource': 'legacy-migration',
                'legacyId': legacy_building['id'],
                'migrationDate': datetime.now().isoformat()
            }
        }

    def validate_building(self, building):
        # Validate geometry
        if not self.validate_geometry(building['geometry']):
            return False

        # Validate required fields
        required = ['name', 'buildingType', 'height']
        if not all(building['properties'].get(f) for f in required):
            return False

        return True
```

---

## Best Practices

### 10.1 Performance Optimization

```typescript
// Caching Strategy
const cacheStrategy = {
  // Hot data: frequently accessed
  hot: {
    layer: 'redis',
    ttl: 60,        // 1 minute
    examples: ['current sensor values', 'service status']
  },

  // Warm data: regularly accessed
  warm: {
    layer: 'application cache',
    ttl: 300,       // 5 minutes
    examples: ['city objects', 'historical aggregates']
  },

  // Cold data: rarely accessed
  cold: {
    layer: 'CDN',
    ttl: 3600,      // 1 hour
    examples: ['3D models', 'static maps']
  }
};

// Query Optimization
class QueryOptimizer {
  optimizeSpatialQuery(bounds: GeoBounds): Query {
    return {
      // Use spatial index
      index: 'rtree',

      // Limit fields
      fields: ['id', 'type', 'geometry'],

      // Batch results
      batchSize: 1000,

      // Use appropriate detail level
      lod: this.selectLOD(bounds)
    };
  }

  private selectLOD(bounds: GeoBounds): number {
    const area = this.calculateArea(bounds);

    if (area > 100) return 0;      // City-wide view
    if (area > 10) return 1;       // District view
    if (area > 1) return 2;        // Neighborhood view
    return 3;                       // Building view
  }
}
```

### 10.2 Security Best Practices

```yaml
security:
  authentication:
    - Use OAuth 2.0 for user authentication
    - Implement API key rotation every 90 days
    - Use mTLS for service-to-service communication
    - Enable multi-factor authentication for admin access

  authorization:
    - Implement role-based access control (RBAC)
    - Use principle of least privilege
    - Audit all access to sensitive data
    - Implement topic-based access control for MQTT

  data:
    - Encrypt data at rest (AES-256)
    - Encrypt data in transit (TLS 1.3)
    - Anonymize personal data
    - Implement data retention policies

  network:
    - Use VPC/private networks
    - Implement network segmentation
    - Enable DDoS protection
    - Use WAF for HTTP traffic

  monitoring:
    - Log all API requests
    - Monitor for anomalous behavior
    - Set up intrusion detection
    - Regular security audits
```

### 10.3 Scalability Guidelines

```typescript
// Horizontal Scaling Strategy
const scalingStrategy = {
  // Stateless services: scale freely
  stateless: {
    services: ['api-gateway', 'city-object-service', 'sensor-service'],
    scaling: {
      metric: 'cpu-utilization',
      threshold: 70,
      minReplicas: 3,
      maxReplicas: 50
    }
  },

  // Stateful services: scale carefully
  stateful: {
    services: ['database', 'message-broker'],
    scaling: {
      method: 'vertical',  // Scale up first
      readReplicas: true,  // Then add read replicas
      sharding: {
        enabled: true,
        key: 'cityId'      // Shard by city
      }
    }
  },

  // Data layer
  data: {
    partitioning: {
      timeseries: 'by-month',
      spatial: 'by-district',
      cityObjects: 'by-type'
    },
    archival: {
      hot: '90-days',
      warm: '1-year',
      cold: '5-years',
      archive: 'indefinite'
    }
  }
};
```

---

## References

### Related Standards

- [WIA Digital Twin City Data Format (Phase 1)](/digital-twin-city/spec/PHASE-1-DATA-FORMAT.md)
- [WIA Digital Twin City API Interface (Phase 2)](/digital-twin-city/spec/PHASE-2-API-INTERFACE.md)
- [WIA Digital Twin City Protocol (Phase 3)](/digital-twin-city/spec/PHASE-3-PROTOCOL.md)

### Integration Standards

- [AWS IoT Core Documentation](https://docs.aws.amazon.com/iot/)
- [Azure IoT Hub Documentation](https://docs.microsoft.com/azure/iot-hub/)
- [Google Cloud IoT Documentation](https://cloud.google.com/iot/docs)
- [Esri ArcGIS API](https://developers.arcgis.com/)
- [OpenStreetMap API](https://wiki.openstreetmap.org/wiki/API)

### Deployment Standards

- [Kubernetes Documentation](https://kubernetes.io/docs/)
- [Docker Documentation](https://docs.docker.com/)
- [Terraform Documentation](https://www.terraform.io/docs)

### Monitoring Standards

- [Prometheus Documentation](https://prometheus.io/docs/)
- [Grafana Documentation](https://grafana.com/docs/)
- [OpenTelemetry](https://opentelemetry.io/docs/)

---

<div align="center">

**WIA Digital Twin City Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-4-INTEGRATION validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
