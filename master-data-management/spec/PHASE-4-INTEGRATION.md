# WIA-DATA-009: Master Data Management Standard
## PHASE 4: Integration Specification

**Version:** 1.0.0  
**Status:** Draft  
**Last Updated:** 2025-12-26

---

## Overview

This phase defines integration patterns, reference implementations, and best practices for connecting MDM systems with enterprise applications, data platforms, and external services.

## 1. Integration Architecture Patterns

### 1.1 MDM Architectural Styles

| Style | Description | Best For |
|-------|-------------|----------|
| **Registry** | Central index with references to source systems | Cross-referencing, lightweight MDM |
| **Consolidation** | Central repository stores golden records | Analytics, reporting, data warehousing |
| **Coexistence** | Master data managed centrally and synchronized to operational systems | Hybrid environments, gradual adoption |
| **Transaction** | MDM hub drives operational processes | Real-time applications, e-commerce |

### 1.2 Hybrid Architecture

```
┌────────────────────────────────────────────────────────────┐
│                    MDM Hub (Registry)                      │
│  ┌──────────────────────────────────────────────────┐     │
│  │          Golden Record Repository                │     │
│  │        (Consolidation for Analytics)             │     │
│  └──────────────────────────────────────────────────┘     │
│                          │                                 │
│  ┌───────────────────────┴─────────────────────────┐      │
│  │         Synchronization Services                 │      │
│  │    (Coexistence for Operational Systems)         │      │
│  └──────────────────────────────────────────────────┘     │
└────────────────────────────────────────────────────────────┘
           │                    │                    │
    ┌──────▼──────┐      ┌────▼────┐         ┌────▼────┐
    │     CRM     │      │   ERP   │         │Analytics│
    └─────────────┘      └─────────┘         └─────────┘
```

## 2. Source System Integration

### 2.1 CRM Integration (Salesforce, Microsoft Dynamics)

**Integration Method**: Bidirectional API sync

**Customer Master Data Flow**:
```
CRM → MDM Hub → Enrich/Match/Merge → Golden Record → CRM
```

**Implementation**:
```javascript
// Salesforce Apex Trigger
trigger AccountMDMSync on Account (after insert, after update) {
    List<Account> accounts = Trigger.new;
    MDMService.syncAccounts(accounts);
}

// MDM Sync Service
public class MDMService {
    @future(callout=true)
    public static void syncAccounts(List<Account> accounts) {
        HttpRequest req = new HttpRequest();
        req.setEndpoint('https://mdm.api.example.com/v1/customers');
        req.setMethod('POST');
        req.setHeader('Authorization', 'Bearer ' + getAccessToken());
        req.setHeader('Content-Type', 'application/json');
        req.setBody(JSON.serialize(accounts));
        
        HttpResponse res = new Http().send(req);
        // Handle response...
    }
}
```

### 2.2 ERP Integration (SAP, Oracle)

**Integration Method**: Batch ETL + Real-time events for critical data

**Vendor Master Data Flow**:
```
ERP → Extract (Nightly) → Transform → MDM Hub → Publish Events → Target Systems
```

**SAP IDoc Integration**:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<IDOC>
  <EDI_DC40>
    <IDOCTYP>DEBMAS</IDOCTYP>
    <MESTYP>DEBMAS</MESTYP>
  </EDI_DC40>
  <E1KNA1M>
    <KUNNR>0000001234</KUNNR>
    <NAME1>ACME Corporation</NAME1>
    <STRAS>123 Main Street</STRAS>
    <!-- Customer master data fields -->
  </E1KNA1M>
</IDOC>
```

### 2.3 E-Commerce Integration (Shopify, Magento)

**Integration Method**: Webhook + REST API

**Product Master Data Flow**:
```
MDM Hub → REST API → E-Commerce Platform
E-Commerce → Webhook → MDM Hub (for inventory/orders)
```

**Shopify Webhook Handler**:
```javascript
app.post('/webhooks/shopify/products', async (req, res) => {
    const product = req.body;
    
    // Map Shopify product to MDM format
    const mdmProduct = {
        entityType: 'product',
        sourceSystem: 'Shopify',
        sourceSystemId: product.id,
        attributes: {
            productIdentifiers: {
                sku: product.variants[0].sku,
                upc: product.variants[0].barcode
            },
            productInfo: {
                name: product.title,
                description: product.body_html
            }
        }
    };
    
    // Send to MDM
    await mdmClient.createOrUpdate('product', mdmProduct);
    res.status(200).send('OK');
});
```

## 3. Data Platform Integration

### 3.1 Data Warehouse Integration

**Pattern**: Dimension table synchronization

**Customer Dimension Table**:
```sql
CREATE TABLE dim_customer (
    customer_key INTEGER PRIMARY KEY,
    customer_id VARCHAR(50) UNIQUE,  -- MDM Golden Record ID
    customer_name VARCHAR(200),
    customer_type VARCHAR(50),
    customer_segment VARCHAR(50),
    effective_date DATE,
    expiration_date DATE,
    is_current BOOLEAN,
    created_at TIMESTAMP,
    updated_at TIMESTAMP
);

-- Type 2 SCD update process
INSERT INTO dim_customer
SELECT 
    nextval('customer_key_seq'),
    mdm.golden_record_id,
    mdm.customer_name,
    mdm.customer_type,
    mdm.customer_segment,
    CURRENT_DATE,
    '9999-12-31',
    TRUE,
    CURRENT_TIMESTAMP,
    CURRENT_TIMESTAMP
FROM mdm.golden_records mdm
WHERE NOT EXISTS (
    SELECT 1 FROM dim_customer dc 
    WHERE dc.customer_id = mdm.golden_record_id 
    AND dc.is_current = TRUE
);
```

### 3.2 Data Lake Integration

**Pattern**: Event streaming to data lake

**Kafka → S3 Data Lake**:
```python
from kafka import KafkaConsumer
import boto3
import json
from datetime import datetime

consumer = KafkaConsumer(
    'mdm.customer.*',
    bootstrap_servers=['kafka:9092'],
    value_deserializer=lambda m: json.loads(m.decode('utf-8'))
)

s3_client = boto3.client('s3')
bucket = 'mdm-data-lake'

for message in consumer:
    event = message.value
    entity_type = event['entityType']
    date = datetime.now().strftime('%Y/%m/%d')
    
    # Write to S3 in partitioned structure
    key = f'raw/mdm/{entity_type}/year={date.split("/")[0]}/month={date.split("/")[1]}/day={date.split("/")[2]}/{event["eventId"]}.json'
    
    s3_client.put_object(
        Bucket=bucket,
        Key=key,
        Body=json.dumps(event)
    )
```

### 3.3 Analytics Platform Integration (Tableau, Power BI)

**Pattern**: Direct query via ODBC/JDBC or cached extracts

**Power BI DirectQuery**:
```m
let
    Source = Odbc.DataSource("dsn=MDM_ODBC", [
        HierarchicalNavigation=true
    ]),
    Database = Source{[Name="mdm_db",Kind="Database"]}[Data],
    Schema = Database{[Name="public",Kind="Schema"]}[Data],
    Table = Schema{[Name="vw_customer_360",Kind="View"]}[Data]
in
    Table
```

## 4. Third-Party Service Integration

### 4.1 Address Validation (Google Maps, USPS)

**Integration Pattern**: Enrichment service

```javascript
async function enrichAddress(customer) {
    const address = `${customer.street}, ${customer.city}, ${customer.state} ${customer.postalCode}`;
    
    const response = await fetch(
        `https://maps.googleapis.com/maps/api/geocode/json?address=${encodeURIComponent(address)}&key=${API_KEY}`
    );
    
    const data = await response.json();
    if (data.results[0]) {
        return {
            formattedAddress: data.results[0].formatted_address,
            latitude: data.results[0].geometry.location.lat,
            longitude: data.results[0].geometry.location.lng,
            placeId: data.results[0].place_id
        };
    }
}
```

### 4.2 Data Enrichment (Clearbit, ZoomInfo)

**Integration Pattern**: Asynchronous enrichment

```python
import clearbit
clearbit.key = 'YOUR_API_KEY'

def enrich_company(company_domain):
    company = clearbit.Company.find(domain=company_domain, stream=True)
    
    return {
        'name': company['name'],
        'domain': company['domain'],
        'industry': company['category']['industry'],
        'employees': company['metrics']['employees'],
        'revenue': company['metrics']['estimatedAnnualRevenue'],
        'description': company['description']
    }
```

### 4.3 Identity Verification (Jumio, Onfido)

**Integration Pattern**: Webhook callback

```javascript
app.post('/webhooks/jumio/verification', (req, res) => {
    const verification = req.body;
    
    // Update customer record with verification status
    mdmClient.update('customer', verification.customerId, {
        'metadata.identityVerified': verification.verificationStatus === 'APPROVED',
        'metadata.identityVerifiedAt': new Date().toISOString(),
        'metadata.identityVerificationProvider': 'Jumio'
    });
    
    res.status(200).send('OK');
});
```

## 5. Cloud Platform Integration

### 5.1 AWS Integration

**Services**:
- **S3**: Data lake storage
- **Lambda**: Serverless data transformations
- **DynamoDB**: NoSQL golden record storage
- **RDS**: Relational MDM database
- **SQS**: Message queuing
- **EventBridge**: Event routing

**Lambda Function for MDM Event Processing**:
```python
import boto3
import json

dynamodb = boto3.resource('dynamodb')
table = dynamodb.Table('mdm-golden-records')

def lambda_handler(event, context):
    for record in event['Records']:
        message = json.loads(record['body'])
        
        if message['eventType'] == 'entity.updated':
            table.update_item(
                Key={'entityId': message['entityId']},
                UpdateExpression='SET #attrs = :attrs, version = version + :inc',
                ExpressionAttributeNames={'#attrs': 'attributes'},
                ExpressionAttributeValues={
                    ':attrs': message['payload']['attributes'],
                    ':inc': 1
                }
            )
    
    return {'statusCode': 200}
```

### 5.2 Azure Integration

**Services**:
- **Azure SQL**: MDM database
- **Cosmos DB**: Globally distributed golden records
- **Service Bus**: Enterprise messaging
- **Functions**: Serverless processing
- **Data Factory**: ETL orchestration

**Azure Function for MDM Sync**:
```csharp
[FunctionName("MDMSync")]
public static async Task Run(
    [ServiceBusTrigger("mdm-events", Connection = "ServiceBusConnection")] string message,
    [CosmosDB(
        databaseName: "mdm",
        collectionName: "golden-records",
        ConnectionStringSetting = "CosmosDBConnection")] IAsyncCollector<dynamic> records,
    ILogger log)
{
    var mdmEvent = JsonConvert.DeserializeObject<MDMEvent>(message);
    
    if (mdmEvent.EventType == "entity.updated")
    {
        await records.AddAsync(mdmEvent.Payload);
    }
}
```

### 5.3 Google Cloud Integration

**Services**:
- **BigQuery**: Analytics and data warehouse
- **Cloud SQL**: Relational database
- **Pub/Sub**: Message broker
- **Cloud Functions**: Serverless
- **Dataflow**: Stream/batch processing

## 6. API Management Integration

### 6.1 API Gateway (Kong, Apigee)

**Kong Configuration**:
```yaml
services:
- name: mdm-service
  url: http://mdm-api:8080
  routes:
  - name: mdm-customers
    paths:
    - /v1/customers
    methods:
    - GET
    - POST
    - PUT
    - DELETE
  plugins:
  - name: rate-limiting
    config:
      minute: 100
      policy: local
  - name: jwt
  - name: correlation-id
```

## 7. Monitoring and Observability Integration

### 7.1 Metrics (Prometheus, Datadog)

**Prometheus Metrics**:
```python
from prometheus_client import Counter, Histogram

mdm_requests = Counter('mdm_requests_total', 'Total MDM API requests', ['method', 'endpoint'])
mdm_duration = Histogram('mdm_request_duration_seconds', 'MDM request duration')

@mdm_duration.time()
def process_customer_update(customer_id, data):
    mdm_requests.labels('PUT', '/customers').inc()
    # Process update...
```

### 7.2 Logging (ELK Stack, Splunk)

**Structured Logging**:
```json
{
  "timestamp": "2025-12-26T10:30:00Z",
  "level": "INFO",
  "service": "mdm-hub",
  "operation": "entity.update",
  "entityType": "customer",
  "entityId": "a1b2c3d4...",
  "duration_ms": 45,
  "userId": "user@example.com",
  "correlationId": "req-123abc"
}
```

### 7.3 Tracing (Jaeger, Zipkin)

**OpenTelemetry Tracing**:
```javascript
const { trace } = require('@opentelemetry/api');

async function updateCustomer(customerId, data) {
    const tracer = trace.getTracer('mdm-service');
    const span = tracer.startSpan('update_customer');
    span.setAttribute('customer.id', customerId);
    
    try {
        await validateCustomer(data);
        await enrichCustomer(data);
        await saveCustomer(customerId, data);
        span.setStatus({ code: SpanStatusCode.OK });
    } catch (error) {
        span.setStatus({ code: SpanStatusCode.ERROR, message: error.message });
        throw error;
    } finally {
        span.end();
    }
}
```

## 8. DevOps Integration

### 8.1 CI/CD (Jenkins, GitLab CI, GitHub Actions)

**GitHub Actions Workflow**:
```yaml
name: MDM Deployment

on:
  push:
    branches: [ main ]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v2
    
    - name: Run Tests
      run: npm test
    
    - name: Build Docker Image
      run: docker build -t mdm-hub:${{ github.sha }} .
    
    - name: Push to Registry
      run: docker push mdm-hub:${{ github.sha }}
    
    - name: Deploy to Kubernetes
      run: |
        kubectl set image deployment/mdm-hub mdm-hub=mdm-hub:${{ github.sha }}
```

### 8.2 Infrastructure as Code (Terraform)

**Terraform Configuration**:
```hcl
resource "aws_rds_instance" "mdm_db" {
  identifier           = "mdm-database"
  engine              = "postgres"
  engine_version      = "14.7"
  instance_class      = "db.r5.xlarge"
  allocated_storage   = 500
  storage_encrypted   = true
  multi_az            = true
  
  tags = {
    Application = "MDM"
    Environment = "Production"
  }
}

resource "aws_elasticache_cluster" "mdm_cache" {
  cluster_id           = "mdm-cache"
  engine              = "redis"
  node_type           = "cache.r5.large"
  num_cache_nodes     = 3
  parameter_group_name = "default.redis6.x"
}
```

## 9. Disaster Recovery and Business Continuity

### 9.1 Backup Strategy

- **Database**: Point-in-time recovery enabled, 30-day retention
- **Files**: S3 versioning and cross-region replication
- **Events**: Kafka topic retention (7 days)

### 9.2 Failover Architecture

```
Primary Region (US-East)          Secondary Region (EU-West)
┌────────────────────┐            ┌────────────────────┐
│   MDM Hub          │◄──Sync────▶│  MDM Hub (Standby) │
│   Active-Active    │            │  Read Replica      │
└────────────────────┘            └────────────────────┘
```

## 10. Compliance and Audit

### 10.1 GDPR Compliance

**Right to Access**:
```sql
SELECT * FROM mdm.customer_360_view 
WHERE email = 'customer@example.com';
```

**Right to be Forgotten**:
```sql
UPDATE mdm.customers 
SET status = 'deleted',
    gdpr_deleted_at = CURRENT_TIMESTAMP
WHERE customer_id = 'C12345';

-- Anonymize historical records
UPDATE mdm.customer_history
SET email = 'anonymized@deleted.com',
    phone = 'REDACTED',
    name = 'DELETED USER'
WHERE customer_id = 'C12345';
```

### 10.2 Audit Trail

**Comprehensive Audit Log**:
```json
{
  "auditId": "uuid",
  "timestamp": "2025-12-26T10:30:00Z",
  "action": "UPDATE",
  "entityType": "customer",
  "entityId": "a1b2c3d4...",
  "userId": "steward@example.com",
  "ipAddress": "192.168.1.100",
  "changes": [
    {"field": "email", "oldValue": "old@example.com", "newValue": "new@example.com"}
  ],
  "reason": "Customer request",
  "approvedBy": "manager@example.com"
}
```

## Summary

This integration specification provides comprehensive guidance for connecting MDM systems with enterprise applications, cloud platforms, and external services, ensuring seamless data flow and interoperability across the organization.
