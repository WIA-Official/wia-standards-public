# WIA-FOOD_SAFETY Specification - PHASE 4: Deployment, Operations & Monitoring

**Version:** 1.0.0
**Last Updated:** 2026-01-11

## Overview

This document defines the deployment architecture, operational procedures, monitoring strategies, and incident response protocols for WIA-FOOD_SAFETY. The system supports cloud-native (SaaS), on-premises, and hybrid deployment models.

## Deployment Architecture

### Cloud-Native Deployment (AWS)

```
┌─────────────────────────────────────────────────────────────┐
│                       Route 53 (DNS)                         │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────────┐
│              CloudFront (CDN) + WAF                          │
│              • DDoS protection                               │
│              • TLS termination                               │
└────────────────────────┬────────────────────────────────────┘
                         │
        ┌────────────────┼────────────────┐
        │                                  │
┌───────▼───────┐               ┌─────────▼─────────┐
│ ALB (App LB)  │               │   S3 + CloudFront  │
│ Multi-AZ      │               │   (Static Assets)  │
└───────┬───────┘               └────────────────────┘
        │
┌───────▼────────────────────────────────────────────┐
│        ECS Fargate (Containers)                     │
│   ┌──────────┐  ┌──────────┐  ┌──────────┐        │
│   │ API      │  │ Worker   │  │ GraphQL  │        │
│   │ Service  │  │ Queue    │  │ Service  │        │
│   │ (3 tasks)│  │ (2 tasks)│  │ (2 tasks)│        │
│   └──────────┘  └──────────┘  └──────────┘        │
└────────────────────┬───────────────────────────────┘
                     │
    ┌────────────────┼────────────────┐
    │                │                │
┌───▼─────┐  ┌──────▼──────┐  ┌─────▼─────┐
│ RDS     │  │ ElastiCache │  │ DynamoDB  │
│ Postgres│  │ Redis       │  │ (Sessions)│
│ Multi-AZ│  │ Cluster     │  │           │
└─────────┘  └─────────────┘  └───────────┘

┌─────────────────────────────────────────────────────┐
│         Blockchain & External Services              │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐         │
│  │ Ethereum │  │ IPFS     │  │ IoT Hub  │         │
│  │ Polygon  │  │ (Pinata) │  │ (AWS IoT)│         │
│  └──────────┘  └──────────┘  └──────────┘         │
└─────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────┐
│      Monitoring & Logging (Observability)           │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐         │
│  │CloudWatch│  │ DataDog  │  │ Sentry   │         │
│  │ Logs     │  │ (Metrics)│  │ (Errors) │         │
│  └──────────┘  └──────────┘  └──────────┘         │
└─────────────────────────────────────────────────────┘
```

### Infrastructure as Code (Terraform)

```hcl
# terraform/main.tf
terraform {
  required_version = ">= 1.0"
  backend "s3" {
    bucket = "wia-food-safety-terraform-state"
    key    = "production/terraform.tfstate"
    region = "us-east-1"
    encrypt = true
    dynamodb_table = "terraform-lock"
  }
}

provider "aws" {
  region = var.aws_region
}

# ECS Cluster
resource "aws_ecs_cluster" "main" {
  name = "wia-food-safety-cluster"

  setting {
    name  = "containerInsights"
    value = "enabled"
  }
}

# ECS Task Definition
resource "aws_ecs_task_definition" "api" {
  family                   = "wia-food-safety-api"
  network_mode             = "awsvpc"
  requires_compatibilities = ["FARGATE"]
  cpu                      = "1024"
  memory                   = "2048"
  execution_role_arn       = aws_iam_role.ecs_execution_role.arn
  task_role_arn            = aws_iam_role.ecs_task_role.arn

  container_definitions = jsonencode([
    {
      name      = "api"
      image     = "${var.ecr_repository_url}:${var.app_version}"
      cpu       = 1024
      memory    = 2048
      essential = true

      portMappings = [
        {
          containerPort = 3000
          protocol      = "tcp"
        }
      ]

      environment = [
        { name = "NODE_ENV", value = "production" },
        { name = "DATABASE_URL", value = "postgresql://..." },
        { name = "REDIS_URL", value = "redis://..." }
      ]

      secrets = [
        { name = "JWT_SECRET", valueFrom = "arn:aws:secretsmanager:..." }
      ]

      logConfiguration = {
        logDriver = "awslogs"
        options = {
          "awslogs-group"         = "/ecs/wia-food-safety-api"
          "awslogs-region"        = var.aws_region
          "awslogs-stream-prefix" = "api"
        }
      }
    }
  ])
}

# RDS PostgreSQL
resource "aws_db_instance" "postgres" {
  identifier              = "wia-food-safety-db"
  engine                  = "postgres"
  engine_version          = "15.4"
  instance_class          = "db.r6g.xlarge"
  allocated_storage       = 100
  storage_type            = "gp3"
  storage_encrypted       = true

  multi_az                = true
  db_subnet_group_name    = aws_db_subnet_group.main.name
  vpc_security_group_ids  = [aws_security_group.rds.id]

  backup_retention_period = 30
  backup_window           = "03:00-04:00"
  maintenance_window      = "mon:04:00-mon:05:00"

  enabled_cloudwatch_logs_exports = ["postgresql", "upgrade"]
  performance_insights_enabled    = true

  username = "postgres"
  password = var.db_password  # Stored in AWS Secrets Manager
}

# ElastiCache Redis
resource "aws_elasticache_replication_group" "redis" {
  replication_group_id       = "wia-food-safety-redis"
  replication_group_description = "Redis cluster for caching"
  engine                     = "redis"
  engine_version             = "7.0"
  node_type                  = "cache.r6g.large"
  num_cache_clusters         = 3
  automatic_failover_enabled = true
  multi_az_enabled           = true
  at_rest_encryption_enabled = true
  transit_encryption_enabled = true
  subnet_group_name          = aws_elasticache_subnet_group.main.name
}

# Auto Scaling
resource "aws_appautoscaling_target" "ecs_target" {
  max_capacity       = 10
  min_capacity       = 3
  resource_id        = "service/${aws_ecs_cluster.main.name}/${aws_ecs_service.api.name}"
  scalable_dimension = "ecs:service:DesiredCount"
  service_namespace  = "ecs"
}

resource "aws_appautoscaling_policy" "cpu_scaling" {
  name               = "cpu-scaling"
  policy_type        = "TargetTrackingScaling"
  resource_id        = aws_appautoscaling_target.ecs_target.resource_id
  scalable_dimension = aws_appautoscaling_target.ecs_target.scalable_dimension
  service_namespace  = aws_appautoscaling_target.ecs_target.service_namespace

  target_tracking_scaling_policy_configuration {
    target_value       = 70.0
    predefined_metric_specification {
      predefined_metric_type = "ECSServiceAverageCPUUtilization"
    }
    scale_in_cooldown  = 300
    scale_out_cooldown = 60
  }
}
```

### On-Premises Deployment (Kubernetes)

```yaml
# kubernetes/deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: wia-food-safety-api
  namespace: production
spec:
  replicas: 3
  selector:
    matchLabels:
      app: wia-food-safety-api
  template:
    metadata:
      labels:
        app: wia-food-safety-api
    spec:
      containers:
      - name: api
        image: registry.example.com/wia-food-safety-api:v1.0.0
        ports:
        - containerPort: 3000
        env:
        - name: NODE_ENV
          value: "production"
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: app-secrets
              key: database-url
        resources:
          requests:
            memory: "1Gi"
            cpu: "500m"
          limits:
            memory: "2Gi"
            cpu: "1000m"
        livenessProbe:
          httpGet:
            path: /health
            port: 3000
          initialDelaySeconds: 30
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /ready
            port: 3000
          initialDelaySeconds: 10
          periodSeconds: 5

---
apiVersion: v1
kind: Service
metadata:
  name: wia-food-safety-api
  namespace: production
spec:
  selector:
    app: wia-food-safety-api
  ports:
  - protocol: TCP
    port: 80
    targetPort: 3000
  type: LoadBalancer

---
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: wia-food-safety-api-hpa
  namespace: production
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: wia-food-safety-api
  minReplicas: 3
  maxReplicas: 10
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

### Blockchain Node Deployment

#### Ethereum Node (Geth)
```bash
# docker-compose.yml
version: '3.8'
services:
  geth:
    image: ethereum/client-go:v1.13.0
    container_name: geth-node
    ports:
      - "8545:8545"  # HTTP-RPC
      - "8546:8546"  # WebSocket
      - "30303:30303"  # P2P
    volumes:
      - ./geth-data:/root/.ethereum
    command:
      - "--http"
      - "--http.addr=0.0.0.0"
      - "--http.port=8545"
      - "--http.api=eth,net,web3,personal,admin"
      - "--ws"
      - "--ws.addr=0.0.0.0"
      - "--ws.port=8546"
      - "--ws.api=eth,net,web3"
      - "--syncmode=snap"
      - "--cache=4096"
      - "--maxpeers=50"
    restart: unless-stopped
```

#### Hyperledger Fabric Network
```yaml
# fabric-network/docker-compose.yaml
version: '3.7'

networks:
  food-safety-network:

services:
  # Orderer (Consensus)
  orderer.wia-food-safety.org:
    image: hyperledger/fabric-orderer:2.5
    environment:
      - ORDERER_GENERAL_LISTENADDRESS=0.0.0.0
      - ORDERER_GENERAL_BOOTSTRAPMETHOD=file
      - ORDERER_GENERAL_LOCALMSPID=OrdererMSP
    volumes:
      - ./orderer.yaml:/etc/hyperledger/fabric/orderer.yaml
      - ./crypto-config/ordererOrganizations:/var/hyperledger/orderer/msp
    networks:
      - food-safety-network

  # Peer nodes for Farmer organization
  peer0.farmer.wia-food-safety.org:
    image: hyperledger/fabric-peer:2.5
    environment:
      - CORE_PEER_ID=peer0.farmer.wia-food-safety.org
      - CORE_PEER_LOCALMSPID=FarmerMSP
      - CORE_PEER_ADDRESS=peer0.farmer.wia-food-safety.org:7051
      - CORE_CHAINCODE_EXECUTETIMEOUT=300s
    volumes:
      - ./crypto-config/peerOrganizations/farmer:/etc/hyperledger/fabric/msp
      - peer0-farmer-data:/var/hyperledger/production
    networks:
      - food-safety-network

  # Peer nodes for Retailer organization
  peer0.retailer.wia-food-safety.org:
    image: hyperledger/fabric-peer:2.5
    environment:
      - CORE_PEER_ID=peer0.retailer.wia-food-safety.org
      - CORE_PEER_LOCALMSPID=RetailerMSP
      - CORE_PEER_ADDRESS=peer0.retailer.wia-food-safety.org:8051
    volumes:
      - ./crypto-config/peerOrganizations/retailer:/etc/hyperledger/fabric/msp
      - peer0-retailer-data:/var/hyperledger/production
    networks:
      - food-safety-network

volumes:
  peer0-farmer-data:
  peer0-retailer-data:
```

## IoT Sensor Network Deployment

### LoRaWAN Gateway Setup

```yaml
# LoRaWAN Gateway Configuration
Gateway Specifications:
  Model: RAK7268 WisGate Edge Lite 2
  Frequency: US915 (North America), EU868 (Europe)
  Channels: 8 channels
  Range: Up to 15 km (rural), 2-5 km (urban)
  Backhaul: Ethernet, Wi-Fi, LTE

Network Server:
  Option 1: ChirpStack (Open Source)
    - Self-hosted
    - PostgreSQL database
    - Redis for device sessions
    - Full control over data

  Option 2: AWS IoT Core for LoRaWAN
    - Managed service
    - Pay-per-use ($0.00099 per message)
    - Integration with AWS services
```

**ChirpStack Deployment:**
```yaml
# docker-compose-chirpstack.yml
version: '3.8'
services:
  postgres:
    image: postgres:15
    environment:
      POSTGRES_DB: chirpstack
      POSTGRES_USER: chirpstack
      POSTGRES_PASSWORD: ${DB_PASSWORD}
    volumes:
      - postgres-data:/var/lib/postgresql/data

  redis:
    image: redis:7-alpine
    volumes:
      - redis-data:/data

  chirpstack:
    image: chirpstack/chirpstack:4
    ports:
      - "8080:8080"  # Web UI
    environment:
      - DATABASE_URL=postgresql://chirpstack:${DB_PASSWORD}@postgres/chirpstack
      - REDIS_URL=redis://redis:6379
    depends_on:
      - postgres
      - redis

  chirpstack-gateway-bridge:
    image: chirpstack/chirpstack-gateway-bridge:4
    ports:
      - "1700:1700/udp"  # Semtech UDP protocol
    environment:
      - INTEGRATION__MQTT__EVENT_TOPIC_TEMPLATE=gateway/+/event/+
      - INTEGRATION__MQTT__COMMAND_TOPIC_TEMPLATE=gateway/+/command/+
```

### Temperature Sensor Deployment

**Sensor Placement Strategy:**
```yaml
Cold Storage Facility:
  Total Area: 1000 m²
  Zones: 10 zones (100 m² each)
  Sensors per Zone: 1
  Total Sensors: 10
  Installation: Ceiling-mounted, 2m height
  Calibration: Annual

Refrigerated Truck:
  Capacity: 40-foot container
  Zones: 3 (front, middle, rear)
  Sensors per Zone: 1
  Total Sensors: 3
  Installation: Interior walls
  Calibration: Semi-annual

Retail Display Case:
  Sensors per Case: 1
  Installation: Top shelf, center position
  Calibration: Annual
```

**Sensor Configuration:**
```json
{
  "deviceProfile": {
    "name": "Temperature Sensor v2",
    "region": "US915",
    "macVersion": "1.0.3",
    "regParamsRevision": "A",
    "supportsOTAA": true,
    "classBTimeout": 60,
    "classCTimeout": 60,
    "payloadCodec": "CUSTOM",
    "payloadDecoderScript": "function Decode(fPort, bytes) { return { temperature: (bytes[0] << 8 | bytes[1]) / 10, humidity: bytes[2], battery: bytes[3] }; }"
  },
  "device": {
    "devEUI": "70B3D57ED0001234",
    "name": "Cold Storage - Zone 1",
    "description": "Warehouse A, Zone 1",
    "applicationID": "wia-food-safety",
    "deviceProfileID": "temp-sensor-v2",
    "skipFCntCheck": false,
    "tags": {
      "location": "warehouse-a-zone-1",
      "batchId": "FARM-2026-001234"
    }
  }
}
```

## Database Migration & Backup

### Database Migrations (Flyway)

```sql
-- migrations/V1__initial_schema.sql
CREATE TABLE products (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    batch_id VARCHAR(50) UNIQUE NOT NULL,
    product_name VARCHAR(255) NOT NULL,
    category VARCHAR(50) NOT NULL,
    company_id VARCHAR(100) NOT NULL,
    harvest_date TIMESTAMP WITH TIME ZONE NOT NULL,
    expiration_date TIMESTAMP WITH TIME ZONE NOT NULL,
    status VARCHAR(20) DEFAULT 'ACTIVE',
    blockchain_tx_hash VARCHAR(66),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX idx_products_batch_id ON products(batch_id);
CREATE INDEX idx_products_company_id ON products(company_id);
CREATE INDEX idx_products_status ON products(status);

-- migrations/V2__temperature_logs.sql
CREATE TABLE temperature_logs (
    id BIGSERIAL PRIMARY KEY,
    sensor_id VARCHAR(50) NOT NULL,
    batch_id VARCHAR(50) NOT NULL,
    temperature NUMERIC(5,2) NOT NULL,
    humidity NUMERIC(5,2),
    timestamp TIMESTAMP WITH TIME ZONE NOT NULL,
    within_spec BOOLEAN DEFAULT true,
    alert_triggered BOOLEAN DEFAULT false,
    blockchain_tx_hash VARCHAR(66),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Convert to TimescaleDB hypertable for time-series optimization
SELECT create_hypertable('temperature_logs', 'timestamp');

CREATE INDEX idx_temp_logs_batch_id ON temperature_logs(batch_id, timestamp DESC);
CREATE INDEX idx_temp_logs_sensor_id ON temperature_logs(sensor_id, timestamp DESC);
```

### Backup Strategy

```yaml
Backup Policy:
  RDS PostgreSQL:
    Automated Backups:
      - Retention: 30 days
      - Backup Window: 03:00-04:00 UTC
      - Incremental: Every 5 minutes (Transaction Logs)
    Manual Snapshots:
      - Frequency: Weekly (before major releases)
      - Retention: 90 days

  Redis ElastiCache:
    Automated Backups:
      - Retention: 7 days
      - Backup Window: 04:00-05:00 UTC
    Manual Snapshots:
      - Frequency: Before maintenance
      - Retention: 14 days

  Blockchain Data:
    Ethereum:
      - Full node data: No backup needed (sync from network)
      - Private keys: AWS Secrets Manager + offline backup
    Hyperledger Fabric:
      - Ledger snapshots: Daily
      - Chaincode: Version controlled (Git)
      - Crypto materials: Hardware Security Module (HSM)

  IPFS Data:
    - Pinning Service: Pinata (redundant pins: 3 locations)
    - Backup: S3 Glacier (monthly full backup)
```

**Backup Script:**
```bash
#!/bin/bash
# backup-database.sh

DATE=$(date +%Y%m%d_%H%M%S)
BACKUP_DIR="/backups/postgres"
S3_BUCKET="s3://wia-food-safety-backups/postgres"

# Create backup
pg_dump -h $DB_HOST -U $DB_USER -d wia_food_safety \
  --format=custom \
  --compress=9 \
  --file=$BACKUP_DIR/backup_$DATE.dump

# Upload to S3
aws s3 cp $BACKUP_DIR/backup_$DATE.dump $S3_BUCKET/

# Encrypt backup
gpg --encrypt --recipient backup@wia-food-safety.org $BACKUP_DIR/backup_$DATE.dump

# Delete local backup older than 7 days
find $BACKUP_DIR -name "backup_*.dump" -mtime +7 -delete

# Verify backup integrity
pg_restore --list $BACKUP_DIR/backup_$DATE.dump > /dev/null

if [ $? -eq 0 ]; then
  echo "Backup successful: backup_$DATE.dump"
else
  echo "Backup verification failed!" >&2
  exit 1
fi
```

## Monitoring & Observability

### Prometheus Metrics

```yaml
# prometheus.yml
global:
  scrape_interval: 15s
  evaluation_interval: 15s

scrape_configs:
  - job_name: 'wia-food-safety-api'
    static_configs:
      - targets: ['api:3000']
    metrics_path: '/metrics'

  - job_name: 'postgres'
    static_configs:
      - targets: ['postgres-exporter:9187']

  - job_name: 'redis'
    static_configs:
      - targets: ['redis-exporter:9121']

  - job_name: 'node'
    static_configs:
      - targets: ['node-exporter:9100']
```

**Application Metrics (Node.js):**
```typescript
import promClient from 'prom-client';

// Custom metrics
const httpRequestDuration = new promClient.Histogram({
  name: 'http_request_duration_seconds',
  help: 'HTTP request duration in seconds',
  labelNames: ['method', 'route', 'status_code'],
  buckets: [0.01, 0.05, 0.1, 0.5, 1, 2, 5]
});

const traceabilityQueryDuration = new promClient.Histogram({
  name: 'traceability_query_duration_seconds',
  help: 'Traceability query duration in seconds',
  buckets: [0.1, 0.5, 1, 2.2, 5, 10]
});

const ccpViolationsTotal = new promClient.Counter({
  name: 'ccp_violations_total',
  help: 'Total number of CCP violations detected',
  labelNames: ['ccp_type', 'severity']
});

const activeRecalls = new promClient.Gauge({
  name: 'active_recalls_total',
  help: 'Number of active product recalls'
});

// Middleware to track request duration
app.use((req, res, next) => {
  const start = Date.now();
  res.on('finish', () => {
    const duration = (Date.now() - start) / 1000;
    httpRequestDuration.observe({
      method: req.method,
      route: req.route?.path || 'unknown',
      status_code: res.statusCode
    }, duration);
  });
  next();
});

// Expose metrics endpoint
app.get('/metrics', async (req, res) => {
  res.set('Content-Type', promClient.register.contentType);
  res.send(await promClient.register.metrics());
});
```

### Grafana Dashboards

**Dashboard: System Overview**
```json
{
  "title": "WIA-FOOD_SAFETY System Overview",
  "panels": [
    {
      "title": "API Request Rate",
      "targets": [
        {
          "expr": "rate(http_request_duration_seconds_count[5m])"
        }
      ],
      "type": "graph"
    },
    {
      "title": "API Response Time (P95)",
      "targets": [
        {
          "expr": "histogram_quantile(0.95, rate(http_request_duration_seconds_bucket[5m]))"
        }
      ],
      "type": "graph"
    },
    {
      "title": "CCP Violations (Last 24h)",
      "targets": [
        {
          "expr": "increase(ccp_violations_total[24h])"
        }
      ],
      "type": "stat"
    },
    {
      "title": "Active Recalls",
      "targets": [
        {
          "expr": "active_recalls_total"
        }
      ],
      "type": "stat"
    }
  ]
}
```

### Logging (ELK Stack)

```yaml
# logstash/pipeline.conf
input {
  beats {
    port => 5044
  }
}

filter {
  # Parse JSON logs from application
  if [type] == "app-logs" {
    json {
      source => "message"
    }

    # Extract traceability query time
    if [event] == "traceability_query" {
      mutate {
        add_field => { "query_duration_ms" => "%{duration}" }
      }
    }
  }

  # Parse CCP violation logs
  if [event] == "ccp_violation" {
    mutate {
      add_tag => ["alert", "ccp_violation"]
    }
  }
}

output {
  elasticsearch {
    hosts => ["elasticsearch:9200"]
    index => "wia-food-safety-logs-%{+YYYY.MM.dd}"
  }
}
```

## Incident Response

### Runbook: API Service Down

```yaml
Incident: API Service Unresponsive
Severity: P1 (Critical)
On-Call: DevOps team

Detection:
  - Prometheus alert: API service down for 3+ minutes
  - PagerDuty notification sent
  - Status page updated automatically

Investigation Steps:
  1. Check service health:
     - kubectl get pods -n production
     - kubectl logs -n production wia-food-safety-api-xxxxx --tail=100

  2. Check dependencies:
     - PostgreSQL: SELECT 1;
     - Redis: redis-cli ping
     - Blockchain node: curl http://geth:8545

  3. Check resource utilization:
     - kubectl top pods -n production
     - Check CloudWatch metrics (CPU, memory, network)

  4. Check recent deployments:
     - kubectl rollout history deployment/wia-food-safety-api
     - Review recent commits in Git

Mitigation:
  - If OOM (Out of Memory): Increase memory limit, restart pod
  - If CrashLoopBackOff: Rollback to previous version
  - If database connection issue: Restart connection pool
  - If blockchain node down: Switch to backup RPC endpoint

Communication:
  - Update status page every 15 minutes
  - Post updates to Slack #incidents channel
  - Notify key stakeholders if downtime > 30 minutes

Post-Incident:
  - Write post-mortem (within 48 hours)
  - Identify root cause
  - Create action items to prevent recurrence
  - Update runbook with lessons learned
```

### Runbook: CCP Violation Alert

```yaml
Incident: Critical Control Point (CCP) Violation Detected
Severity: P2 (High) - Food Safety Issue
On-Call: Food safety team + DevOps team

Detection:
  - IoT sensor reports temperature > 4°C for > 4 minutes
  - Alert triggered in system
  - SMS + Push notification sent to supervisor

Investigation Steps:
  1. Verify sensor accuracy:
     - Check sensor calibration date
     - Cross-reference with nearby sensors
     - Review historical sensor data

  2. Identify affected batches:
     - Query database: SELECT * FROM temperature_logs WHERE batch_id = 'XXX' AND temperature > 4.0
     - Trace supply chain: GET /products/trace/{batchId}

  3. Assess food safety risk:
     - Duration of violation (< 2 hours = low risk, > 2 hours = high risk)
     - Product type (fresh produce vs. packaged goods)
     - Current location (warehouse vs. in-transit)

Corrective Actions:
  - If < 2 hours violation + refrigerated product:
    - Quarantine batch in system
    - Conduct product evaluation (sensory, temperature check)
    - Document corrective action on blockchain

  - If > 2 hours violation OR high-risk product:
    - Quarantine batch immediately
    - Initiate recall if already distributed
    - Notify FDA if Class I recall

Communication:
  - SMS to facility supervisor (immediate)
  - Email to food safety manager (within 15 minutes)
  - Incident report generated (within 1 hour)
  - FDA notification if required (within 24 hours)

Post-Incident:
  - Review HACCP plan
  - Retrain staff on corrective actions
  - Consider additional sensors for redundancy
```

## Disaster Recovery

### Recovery Time Objective (RTO) & Recovery Point Objective (RPO)

```yaml
Service Tier 1 (Critical):
  Components: API service, Database, Blockchain node
  RTO: < 1 hour
  RPO: < 5 minutes (transaction logs)
  Strategy: Multi-AZ deployment, automated failover

Service Tier 2 (Important):
  Components: Background workers, Analytics
  RTO: < 4 hours
  RPO: < 1 hour
  Strategy: Manual failover, backup restoration

Service Tier 3 (Non-Critical):
  Components: Reporting, Historical data export
  RTO: < 24 hours
  RPO: < 24 hours
  Strategy: Rebuild from backups
```

### Disaster Recovery Plan

```yaml
Scenario 1: Database Failure (RDS)
  1. Automated failover to standby replica (Multi-AZ)
     - Automatic DNS update
     - Downtime: < 2 minutes

  2. If standby also fails:
     - Restore from latest automated snapshot
     - Apply transaction logs (Point-in-Time Recovery)
     - Downtime: 15-30 minutes

Scenario 2: Region Outage (AWS)
  1. Activate DR region (us-west-2)
  2. Update Route 53 to point to DR region
  3. Restore database from cross-region snapshot
  4. Verify blockchain node connectivity
  5. Run smoke tests
  6. Update status page
  Total Time: 30-60 minutes

Scenario 3: Blockchain Node Failure
  1. Switch to backup RPC endpoint:
     - Infura: https://mainnet.infura.io/v3/{API_KEY}
     - Alchemy: https://eth-mainnet.alchemyapi.io/v2/{API_KEY}
  2. Spin up new self-hosted node (background)
  3. Sync new node from network (6-12 hours)
  4. Switch back to self-hosted node
  Downtime: None (seamless failover)
```

---

**© 2026 WIA | 弘익人間 (Benefit All Humanity)**
