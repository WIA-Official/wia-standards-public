# WIA-AGRI-034: Phase 4 - Integration Specification

**Standard ID:** WIA-AGRI-034
**Title:** Single Cell Protein Production System Integration
**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-01-15

---

## 1. Overview

This specification defines integration patterns and interfaces for connecting WIA-AGRI-034 compliant SCP production systems with existing industrial control systems, laboratory information management systems, supply chain platforms, and external data sources.

### 1.1 Integration Objectives

- **Seamless Connectivity**: Minimize integration effort with existing infrastructure
- **Data Interoperability**: Enable bidirectional data exchange across platforms
- **Legacy Support**: Support integration with older systems via adapters
- **Cloud-Native**: First-class support for cloud-based architectures
- **Security**: Maintain security boundaries across system integrations

### 1.2 Integration Layers

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                        │
│  (ERP, MES, LIMS, Supply Chain, Analytics Platforms)       │
└─────────────────────────────────────────────────────────────┘
                            ↕
┌─────────────────────────────────────────────────────────────┐
│                  WIA-AGRI-034 API Layer                     │
│         (REST API, WebSocket, MQTT, gRPC)                   │
└─────────────────────────────────────────────────────────────┘
                            ↕
┌─────────────────────────────────────────────────────────────┐
│              Integration Middleware Layer                    │
│    (Message Brokers, ETL, Data Transformation)              │
└─────────────────────────────────────────────────────────────┘
                            ↕
┌─────────────────────────────────────────────────────────────┐
│                   Control System Layer                       │
│         (SCADA, DCS, PLC, Sensors, Actuators)               │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. SCADA/DCS Integration

### 2.1 OPC UA Integration

**Overview:**
OPC UA (Unified Architecture) is the standard protocol for industrial automation, providing secure, reliable data exchange between production systems and SCADA/DCS platforms.

**Connection Configuration:**
```json
{
  "integration_type": "opc_ua",
  "connection": {
    "endpoint": "opc.tcp://scada.facility.local:4840",
    "security_mode": "SignAndEncrypt",
    "security_policy": "Basic256Sha256",
    "authentication": {
      "type": "username_password",
      "username": "wia_integration",
      "password": "${OPC_PASSWORD}"
    },
    "certificate": {
      "client_cert_path": "/certs/wia-client.der",
      "client_key_path": "/certs/wia-client.pem",
      "server_cert_path": "/certs/scada-server.der"
    }
  },
  "session": {
    "timeout_seconds": 300,
    "keepalive_interval_seconds": 30,
    "max_reconnect_attempts": 10
  }
}
```

**Data Mapping:**
```json
{
  "data_mappings": [
    {
      "wia_parameter": "temperature",
      "opc_node": "ns=2;s=Fermenter.PV.Temperature",
      "direction": "bidirectional",
      "scaling": {
        "source_unit": "C",
        "target_unit": "C",
        "conversion": "linear",
        "factor": 1.0,
        "offset": 0.0
      },
      "quality_mapping": {
        "good": 192,
        "uncertain": 64,
        "bad": 0
      }
    },
    {
      "wia_parameter": "pH",
      "opc_node": "ns=2;s=Fermenter.PV.pH",
      "direction": "read",
      "update_rate_ms": 1000
    },
    {
      "wia_parameter": "dissolved_oxygen",
      "opc_node": "ns=2;s=Fermenter.PV.DO",
      "direction": "read",
      "update_rate_ms": 1000
    },
    {
      "wia_parameter": "agitation_speed",
      "opc_node": "ns=2;s=Fermenter.SP.AgitatorRPM",
      "direction": "write",
      "update_rate_ms": 5000
    }
  ]
}
```

**Implementation Example (Python):**
```python
from opcua import Client, ua
import json

class SCPOPCUAIntegration:
    def __init__(self, config):
        self.client = Client(config['connection']['endpoint'])
        self.client.set_security_string(
            f"{config['connection']['security_policy']},{config['connection']['security_mode']}"
        )
        self.mappings = config['data_mappings']

    async def connect(self):
        await self.client.connect()
        print("Connected to OPC UA server")

    async def read_sensor_data(self):
        data = {}
        for mapping in self.mappings:
            if mapping['direction'] in ['read', 'bidirectional']:
                node = self.client.get_node(mapping['opc_node'])
                value = await node.read_value()
                data[mapping['wia_parameter']] = {
                    'value': value,
                    'timestamp': datetime.now().isoformat()
                }
        return data

    async def write_setpoint(self, parameter, value):
        mapping = next(m for m in self.mappings if m['wia_parameter'] == parameter)
        if mapping['direction'] in ['write', 'bidirectional']:
            node = self.client.get_node(mapping['opc_node'])
            await node.write_value(ua.DataValue(value))
```

### 2.2 Modbus Integration

**Configuration:**
```json
{
  "integration_type": "modbus_tcp",
  "connection": {
    "host": "192.168.1.100",
    "port": 502,
    "unit_id": 1,
    "timeout_seconds": 3
  },
  "register_mappings": [
    {
      "wia_parameter": "temperature",
      "register_type": "holding",
      "address": 40001,
      "data_type": "float32",
      "byte_order": "big_endian",
      "scaling_factor": 0.1
    },
    {
      "wia_parameter": "pH",
      "register_type": "holding",
      "address": 40003,
      "data_type": "float32",
      "byte_order": "big_endian",
      "scaling_factor": 0.01
    }
  ]
}
```

---

## 3. LIMS (Laboratory Information Management System) Integration

### 3.1 Bidirectional Data Exchange

**Quality Sample Submission to LIMS:**
```json
{
  "integration": "lims",
  "operation": "submit_sample",
  "payload": {
    "batch_id": "BATCH-20250115-001",
    "sample_id": "SAMPLE-20250117-001",
    "sample_time": "2025-01-17T12:00:00Z",
    "sample_type": "final_product",
    "tests_requested": [
      {
        "test_code": "PROT-001",
        "test_name": "Protein Content - Kjeldahl",
        "priority": "standard"
      },
      {
        "test_code": "AA-PROF-001",
        "test_name": "Amino Acid Profile - HPLC",
        "priority": "standard"
      },
      {
        "test_code": "MICRO-001",
        "test_name": "Microbial Contamination Panel",
        "priority": "urgent"
      },
      {
        "test_code": "HEAVY-001",
        "test_name": "Heavy Metals Analysis",
        "priority": "standard"
      }
    ],
    "sample_metadata": {
      "volume_mL": 100,
      "storage_conditions": "4°C",
      "container_type": "sterile_bottle",
      "collected_by": "OP-001"
    }
  }
}
```

**Receiving Results from LIMS:**
```json
{
  "integration": "lims",
  "operation": "receive_results",
  "payload": {
    "sample_id": "SAMPLE-20250117-001",
    "lims_job_id": "LIMS-JOB-12345",
    "analysis_complete_time": "2025-01-18T10:00:00Z",
    "analyst": "Jane Smith",
    "results": [
      {
        "test_code": "PROT-001",
        "result": 52.3,
        "unit": "percent",
        "method": "Kjeldahl",
        "status": "approved",
        "uncertainty": 0.5,
        "specification_min": 50.0,
        "specification_max": 60.0,
        "pass_fail": "pass"
      },
      {
        "test_code": "AA-PROF-001",
        "result": {
          "lysine_g_100g": 6.8,
          "methionine_g_100g": 2.1,
          "threonine_g_100g": 4.5
        },
        "status": "approved",
        "pass_fail": "pass"
      },
      {
        "test_code": "MICRO-001",
        "result": {
          "total_plate_count_cfu_g": 150,
          "salmonella": "not_detected",
          "e_coli": "not_detected"
        },
        "status": "approved",
        "pass_fail": "pass"
      }
    ],
    "overall_status": "approved",
    "certificate_url": "https://lims.example.com/certs/LIMS-JOB-12345.pdf"
  }
}
```

### 3.2 HL7 FHIR Integration (for regulated environments)

```json
{
  "resourceType": "DiagnosticReport",
  "id": "SAMPLE-20250117-001",
  "status": "final",
  "category": [{
    "coding": [{
      "system": "http://terminology.hl7.org/CodeSystem/v2-0074",
      "code": "LAB",
      "display": "Laboratory"
    }]
  }],
  "code": {
    "coding": [{
      "system": "http://loinc.org",
      "code": "2085-9",
      "display": "Protein [Mass/volume] in Serum or Plasma"
    }]
  },
  "subject": {
    "reference": "Batch/BATCH-20250115-001"
  },
  "effectiveDateTime": "2025-01-18T10:00:00Z",
  "result": [{
    "reference": "Observation/protein-content-001"
  }]
}
```

---

## 4. ERP System Integration

### 4.1 SAP Integration via REST API

**Production Order Creation:**
```http
POST /sap/opu/odata/sap/API_PRODUCTION_ORDER_2_SRV/A_ProductionOrder_2
Authorization: Bearer SAP_TOKEN
Content-Type: application/json

{
  "ManufacturingOrder": "",
  "MfgOrderCreationDate": "2025-01-15",
  "Material": "SCP-PROTEIN-001",
  "ProductionPlant": "FAC-001",
  "MfgOrderPlannedStartDate": "2025-01-15",
  "MfgOrderPlannedEndDate": "2025-01-17",
  "MfgOrderPlannedTotalQty": "1000",
  "ProductionUnit": "KG",
  "YY1_BatchID_ORD": "BATCH-20250115-001",
  "YY1_StrainType_ORD": "bacteria-methylophilus",
  "YY1_SubstrateType_ORD": "methanol"
}
```

**Goods Receipt Posting:**
```http
POST /sap/opu/odata/sap/API_PRODUCTION_ORDER_2_SRV/A_ProductionOrderConfirmation
Authorization: Bearer SAP_TOKEN
Content-Type: application/json

{
  "ManufacturingOrder": "1000012345",
  "ConfirmationText": "Batch BATCH-20250115-001 completed",
  "ConfirmationYieldQuantity": "1120",
  "ConfirmationYieldQuantityUnit": "KG",
  "FinalConfirmationFlag": "X",
  "YY1_ProteinContent_CNF": "52.3",
  "YY1_QualityGrade_CNF": "A",
  "YY1_ProductionCost_CNF": "2950.00"
}
```

### 4.2 Oracle ERP Cloud Integration

**REST API Configuration:**
```json
{
  "integration_type": "oracle_erp_cloud",
  "connection": {
    "base_url": "https://your-instance.oraclecloud.com",
    "auth_type": "oauth2",
    "client_id": "${ORACLE_CLIENT_ID}",
    "client_secret": "${ORACLE_CLIENT_SECRET}",
    "scope": "urn:opc:resource:consumer::all"
  },
  "endpoints": {
    "work_orders": "/fscmRestApi/resources/11.13.18.05/manufacturingWorkOrders",
    "inventory_transactions": "/fscmRestApi/resources/11.13.18.05/inventoryTransactions",
    "quality_results": "/fscmRestApi/resources/11.13.18.05/qualityResults"
  }
}
```

---

## 5. Cloud Platform Integration

### 5.1 AWS Integration

**IoT Core Configuration:**
```json
{
  "integration_type": "aws_iot_core",
  "connection": {
    "endpoint": "a1b2c3d4e5f6g7.iot.us-east-1.amazonaws.com",
    "port": 8883,
    "protocol": "MQTT over TLS",
    "client_id": "wia-agri-034-client",
    "certificate": "${AWS_IOT_CERT}",
    "private_key": "${AWS_IOT_KEY}",
    "root_ca": "${AWS_ROOT_CA}"
  },
  "topics": {
    "sensor_data": "wia/agri-034/FAC-001/sensor-data",
    "batch_events": "wia/agri-034/FAC-001/batch-events",
    "commands": "wia/agri-034/FAC-001/commands"
  },
  "rules": [
    {
      "name": "Store_Sensor_Data",
      "sql": "SELECT * FROM 'wia/agri-034/+/sensor-data'",
      "actions": [
        {
          "type": "timestream",
          "database": "SCPProduction",
          "table": "SensorData"
        }
      ]
    },
    {
      "name": "Alert_High_Temperature",
      "sql": "SELECT * FROM 'wia/agri-034/+/sensor-data' WHERE temperature > 40",
      "actions": [
        {
          "type": "sns",
          "topic_arn": "arn:aws:sns:us-east-1:123456789:SCP-Alerts"
        }
      ]
    }
  ]
}
```

**Lambda Function for Data Processing:**
```python
import json
import boto3
from datetime import datetime

timestream = boto3.client('timestream-write')

def lambda_handler(event, context):
    """Process WIA-AGRI-034 sensor data and store in Timestream"""

    for record in event['Records']:
        payload = json.loads(record['body'])

        records = []
        for param, data in payload['sensor_data'].items():
            records.append({
                'Time': str(int(datetime.fromisoformat(data['timestamp']).timestamp() * 1000)),
                'Dimensions': [
                    {'Name': 'batch_id', 'Value': payload['batch_id']},
                    {'Name': 'facility_id', 'Value': payload['facility_id']},
                    {'Name': 'parameter', 'Value': param}
                ],
                'MeasureName': 'value',
                'MeasureValue': str(data['value']),
                'MeasureValueType': 'DOUBLE'
            })

        timestream.write_records(
            DatabaseName='SCPProduction',
            TableName='SensorData',
            Records=records
        )

    return {'statusCode': 200, 'body': 'Data processed successfully'}
```

### 5.2 Azure Integration

**IoT Hub Configuration:**
```json
{
  "integration_type": "azure_iot_hub",
  "connection": {
    "hostname": "wia-agri-034-hub.azure-devices.net",
    "device_id": "FAC-001-REACTOR-A",
    "shared_access_key": "${AZURE_SAS_KEY}",
    "protocol": "MQTT"
  },
  "device_twin": {
    "reported": {
      "batch_id": "BATCH-20250115-001",
      "status": "active",
      "firmware_version": "1.0.0",
      "last_calibration": "2025-01-10T10:00:00Z"
    },
    "desired": {
      "telemetry_interval_seconds": 5,
      "alert_thresholds": {
        "temperature_max": 40,
        "pH_min": 6.0,
        "pH_max": 7.0,
        "do_min": 30
      }
    }
  },
  "routes": [
    {
      "name": "SensorDataToTimeSeriesInsights",
      "source": "DeviceMessages",
      "condition": "true",
      "endpoint": "TimeSeriesInsights"
    },
    {
      "name": "AlertsToEventHub",
      "source": "DeviceMessages",
      "condition": "$body.severity = 'critical'",
      "endpoint": "AlertEventHub"
    }
  ]
}
```

### 5.3 Google Cloud Platform Integration

**Pub/Sub Configuration:**
```json
{
  "integration_type": "gcp_pubsub",
  "project_id": "wia-agri-034-production",
  "topics": {
    "sensor_data": "projects/wia-agri-034-production/topics/sensor-data",
    "batch_events": "projects/wia-agri-034-production/topics/batch-events"
  },
  "subscriptions": {
    "sensor_data_bigquery": {
      "topic": "sensor-data",
      "push_endpoint": "https://bigquery.googleapis.com/bigquery/v2/projects/...",
      "ack_deadline_seconds": 60
    },
    "alerts_cloud_functions": {
      "topic": "batch-events",
      "push_endpoint": "https://us-central1-wia-agri-034-production.cloudfunctions.net/process-alerts",
      "ack_deadline_seconds": 30
    }
  }
}
```

---

## 6. Supply Chain & Traceability Integration

### 6.1 Blockchain Integration (Hyperledger Fabric)

**Batch Registration on Blockchain:**
```json
{
  "integration_type": "hyperledger_fabric",
  "chaincode": "wia-agri-034-chaincode",
  "operation": "registerBatch",
  "payload": {
    "batch_id": "BATCH-20250115-001",
    "facility": {
      "id": "FAC-001",
      "name": "SCP Production Center Alpha",
      "certifications": ["ISO22000", "FSSC22000", "Non-GMO"]
    },
    "strain": {
      "id": "STRAIN-001",
      "name": "Methylophilus methylotrophus WIA-M1",
      "genetic_status": "non_gmo"
    },
    "production": {
      "start_time": "2025-01-15T08:00:00Z",
      "end_time": "2025-01-17T12:00:00Z",
      "substrate": "methanol",
      "quantity_kg": 1120
    },
    "quality": {
      "protein_content_percent": 52.3,
      "grade": "A",
      "certifications": ["organic", "vegan", "non_gmo"],
      "analysis_id": "QA-20250117-001"
    },
    "timestamp": "2025-01-17T12:00:00Z",
    "submitter": "OP-001"
  }
}
```

**Smart Contract Example (Go):**
```go
package main

import (
    "encoding/json"
    "github.com/hyperledger/fabric-contract-api-go/contractapi"
)

type SCPBatchContract struct {
    contractapi.Contract
}

type Batch struct {
    BatchID     string    `json:"batch_id"`
    Facility    Facility  `json:"facility"`
    Strain      Strain    `json:"strain"`
    Production  Production `json:"production"`
    Quality     Quality   `json:"quality"`
    Timestamp   string    `json:"timestamp"`
    Status      string    `json:"status"`
}

func (c *SCPBatchContract) RegisterBatch(ctx contractapi.TransactionContextInterface, batchJSON string) error {
    var batch Batch
    err := json.Unmarshal([]byte(batchJSON), &batch)
    if err != nil {
        return err
    }

    batch.Status = "registered"

    batchBytes, err := json.Marshal(batch)
    if err != nil {
        return err
    }

    return ctx.GetStub().PutState(batch.BatchID, batchBytes)
}

func (c *SCPBatchContract) QueryBatch(ctx contractapi.TransactionContextInterface, batchID string) (*Batch, error) {
    batchBytes, err := ctx.GetStub().GetState(batchID)
    if err != nil {
        return nil, err
    }

    var batch Batch
    err = json.Unmarshal(batchBytes, &batch)
    if err != nil {
        return nil, err
    }

    return &batch, nil
}
```

### 6.2 GS1 Standards Integration

**Electronic Product Code Information Services (EPCIS):**
```xml
<?xml version="1.0" encoding="UTF-8"?>
<epcis:EPCISDocument xmlns:epcis="urn:epcglobal:epcis:xsd:1">
  <EPCISBody>
    <EventList>
      <ObjectEvent>
        <eventTime>2025-01-17T12:00:00Z</eventTime>
        <eventTimeZoneOffset>+00:00</eventTimeZoneOffset>
        <epcList>
          <epc>urn:epc:id:sgtin:0614141.107346.BATCH-20250115-001</epc>
        </epcList>
        <action>ADD</action>
        <bizStep>urn:epcglobal:cbv:bizstep:commissioning</bizStep>
        <disposition>urn:epcglobal:cbv:disp:active</disposition>
        <readPoint>
          <id>urn:epc:id:sgln:0614141.00001.0</id>
        </readPoint>
        <bizLocation>
          <id>urn:epc:id:sgln:0614141.00001.0</id>
        </bizLocation>
        <extension>
          <quantityList>
            <quantityElement>
              <epcClass>urn:epc:class:lgtin:0614141.107346.BATCH-20250115-001</epcClass>
              <quantity>1120</quantity>
              <uom>KGM</uom>
            </quantityElement>
          </quantityList>
          <ilmd>
            <wia:proteinContent xmlns:wia="http://wia.org/agri-034">52.3</wia:proteinContent>
            <wia:qualityGrade xmlns:wia="http://wia.org/agri-034">A</wia:qualityGrade>
            <wia:strainType xmlns:wia="http://wia.org/agri-034">bacteria-methylophilus</wia:strainType>
          </ilmd>
        </extension>
      </ObjectEvent>
    </EventList>
  </EPCISBody>
</epcis:EPCISDocument>
```

---

## 7. Analytics Platform Integration

### 7.1 Time-Series Database (InfluxDB)

**Configuration:**
```json
{
  "integration_type": "influxdb",
  "connection": {
    "url": "http://influxdb.example.com:8086",
    "token": "${INFLUX_TOKEN}",
    "org": "wia-agri-034",
    "bucket": "scp-production"
  },
  "write_options": {
    "batch_size": 1000,
    "flush_interval_ms": 1000,
    "retry_interval_ms": 5000,
    "max_retries": 3
  }
}
```

**Data Writing Example (Python):**
```python
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

client = InfluxDBClient(url="http://influxdb.example.com:8086", token=token, org=org)
write_api = client.write_api(write_options=SYNCHRONOUS)

# Write sensor data
point = Point("sensor_data") \
    .tag("batch_id", "BATCH-20250115-001") \
    .tag("facility_id", "FAC-001") \
    .tag("parameter", "temperature") \
    .field("value", 37.2) \
    .field("setpoint", 37.0) \
    .field("deviation", 0.2) \
    .time(datetime.utcnow(), WritePrecision.NS)

write_api.write(bucket="scp-production", org="wia-agri-034", record=point)
```

### 7.2 Apache Kafka Integration

**Producer Configuration:**
```json
{
  "integration_type": "kafka",
  "bootstrap_servers": ["kafka1.example.com:9092", "kafka2.example.com:9092"],
  "security_protocol": "SASL_SSL",
  "sasl_mechanism": "PLAIN",
  "sasl_username": "${KAFKA_USERNAME}",
  "sasl_password": "${KAFKA_PASSWORD}",
  "topics": {
    "sensor_data": "wia.agri-034.sensor-data",
    "batch_events": "wia.agri-034.batch-events",
    "quality_results": "wia.agri-034.quality-results"
  },
  "producer_config": {
    "acks": "all",
    "compression_type": "snappy",
    "batch_size": 16384,
    "linger_ms": 10,
    "retries": 3
  }
}
```

---

## 8. Integration Testing

### 8.1 End-to-End Integration Test

```yaml
integration_test:
  name: "Complete SCP Production Workflow"
  steps:
    - name: "Create batch in WIA-AGRI-034"
      action: "POST /api/v1/batches"
      validate:
        - status_code: 201
        - response.batch_id: not_null

    - name: "Sync to ERP system"
      action: "Verify SAP production order created"
      validate:
        - production_order_exists: true
        - material_number: "SCP-PROTEIN-001"

    - name: "Start sensor streaming"
      action: "WebSocket subscribe"
      validate:
        - connection_established: true
        - data_rate_min_per_second: 1

    - name: "Write to SCADA via OPC UA"
      action: "Update temperature setpoint"
      validate:
        - opc_write_success: true
        - scada_value_updated: true

    - name: "Submit quality sample to LIMS"
      action: "POST quality sample"
      validate:
        - lims_job_created: true
        - sample_id_mapped: true

    - name: "Complete batch and register on blockchain"
      action: "POST /api/v1/batches/{id}/complete"
      validate:
        - batch_status: "completed"
        - blockchain_tx_hash: not_null
        - epcis_event_created: true

    - name: "Generate analytics report"
      action: "GET /api/v1/analytics/reports/{batch_id}"
      validate:
        - report_generated: true
        - influxdb_data_stored: true
```

---

## 9. Error Handling & Resilience

### 9.1 Circuit Breaker Pattern

```python
from circuitbreaker import circuit

@circuit(failure_threshold=5, recovery_timeout=60)
def call_erp_api(data):
    """Call ERP system with circuit breaker protection"""
    response = requests.post(
        f"{ERP_URL}/api/production-orders",
        json=data,
        timeout=10
    )
    response.raise_for_status()
    return response.json()

# Usage
try:
    result = call_erp_api(batch_data)
except CircuitBreakerError:
    logger.error("ERP system circuit breaker open - queueing for retry")
    queue_for_retry(batch_data)
```

### 9.2 Message Queue for Reliable Delivery

```json
{
  "integration_resilience": {
    "message_queue": {
      "type": "rabbitmq",
      "connection": "amqp://rabbitmq.example.com",
      "queues": {
        "erp_integration": {
          "durable": true,
          "auto_delete": false,
          "message_ttl_ms": 86400000,
          "max_retries": 5,
          "retry_delay_ms": 60000
        },
        "lims_integration": {
          "durable": true,
          "auto_delete": false,
          "message_ttl_ms": 86400000
        }
      },
      "dead_letter_exchange": "integration-failures",
      "monitoring": {
        "alert_on_queue_depth": 1000,
        "alert_on_dlq_messages": 10
      }
    }
  }
}
```

---

## 10. Monitoring & Observability

### 10.1 OpenTelemetry Integration

```python
from opentelemetry import trace
from opentelemetry.exporter.otlp.proto.grpc.trace_exporter import OTLPSpanExporter
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.sdk.trace.export import BatchSpanProcessor

# Configure tracing
trace.set_tracer_provider(TracerProvider())
tracer = trace.get_tracer(__name__)

otlp_exporter = OTLPSpanExporter(endpoint="otel-collector:4317")
span_processor = BatchSpanProcessor(otlp_exporter)
trace.get_tracer_provider().add_span_processor(span_processor)

# Trace integration operations
@tracer.start_as_current_span("create_batch")
def create_batch(batch_data):
    with tracer.start_as_current_span("validate_data"):
        validate(batch_data)

    with tracer.start_as_current_span("call_wia_api"):
        batch = wia_client.batches.create(batch_data)

    with tracer.start_as_current_span("sync_to_erp"):
        erp_client.create_production_order(batch)

    return batch
```

---

**弘益人間 (홍익인간) - Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
