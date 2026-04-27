# WIA-DATA-014: PHASE 4 - Integration Specification

**Version:** 1.0.0
**Status:** Complete
**Date:** 2025-12-26

---

## 1. Overview

PHASE 4 defines integration patterns, connectors, and interoperability guidelines for time-series systems.

---

## 2. Database Integrations

### 2.1 InfluxDB Integration

**Connection:**
```javascript
const { InfluxDB } = require('@influxdata/influxdb-client');

const client = new InfluxDB({
  url: 'http://localhost:8086',
  token: 'your-token'
});

const writeApi = client.getWriteApi('org', 'bucket');
writeApi.writePoint(point);
await writeApi.close();
```

**Data Mapping:**
| WIA-DATA-014 | InfluxDB |
|--------------|----------|
| measurement | measurement |
| tags | tags |
| fields | fields |
| timestamp | time |

### 2.2 TimescaleDB Integration

```sql
-- Create hypertable
CREATE TABLE sensor_data (
    time TIMESTAMPTZ NOT NULL,
    measurement TEXT NOT NULL,
    tags JSONB,
    fields JSONB
);

SELECT create_hypertable('sensor_data', 'time');
```

### 2.3 Prometheus Integration

**Exposition Format:**
```
# HELP cpu_usage CPU usage percentage
# TYPE cpu_usage gauge
cpu_usage{host="server1",region="us-east"} 75.2 1735208400000
```

**WIA to Prometheus Mapping:**
```
{measurement}_{field}{tags} value timestamp
```

---

## 3. Data Collection Tools

### 3.1 Telegraf Integration

**Config:**
```toml
[[outputs.wia_timeseries]]
  url = "https://api.example.com/v1/write"
  token = "$API_TOKEN"
  database = "metrics"
  
[[inputs.cpu]]
  percpu = true
  totalcpu = true
```

### 3.2 Fluentd Integration

```ruby
<match timeseries.**>
  @type wia_timeseries
  endpoint https://api.example.com/v1/write
  api_key ${API_KEY}
  database metrics
</match>
```

### 3.3 Logstash Integration

```ruby
output {
  wia_timeseries {
    endpoint => "https://api.example.com/v1/write"
    api_key => "${API_KEY}"
    database => "logs"
  }
}
```

---

## 4. Visualization Tools

### 4.1 Grafana Integration

**Data Source Plugin:**
```json
{
  "type": "wia-timeseries-datasource",
  "name": "WIA Time-Series",
  "url": "https://api.example.com/v1",
  "jsonData": {
    "database": "metrics"
  },
  "secureJsonData": {
    "apiKey": "YOUR_API_KEY"
  }
}
```

**Query Builder:**
```javascript
{
  measurement: 'cpu',
  tags: {host: '$host'},
  field: 'usage',
  aggregation: 'mean',
  interval: '5m'
}
```

### 4.2 Tableau Integration

**WIA ODBC Driver:**
```ini
[WIA Time-Series]
Driver=WIA Time-Series ODBC Driver
Server=api.example.com
Port=443
Database=metrics
APIKey=YOUR_API_KEY
```

---

## 5. Programming Language SDKs

### 5.1 Python SDK

```python
from wia_timeseries import Client

client = Client(
    url='https://api.example.com/v1',
    api_key='your-api-key'
)

# Write data
client.write(
    database='metrics',
    measurement='temperature',
    tags={'location': 'office'},
    fields={'value': 23.5}
)

# Query data
results = client.query(
    database='metrics',
    measurement='temperature',
    start='2025-12-26T00:00:00Z',
    end='2025-12-26T23:59:59Z'
)
```

### 5.2 JavaScript/Node.js SDK

```javascript
const WIATimeSeries = require('wia-timeseries-client');

const client = new WIATimeSeries({
  url: 'https://api.example.com/v1',
  apiKey: 'your-api-key'
});

await client.write({
  database: 'metrics',
  measurement: 'cpu',
  tags: {host: 'server1'},
  fields: {usage: 75.2}
});
```

### 5.3 Go SDK

```go
import "github.com/wia-official/timeseries-go"

client := timeseries.NewClient(
    "https://api.example.com/v1",
    "your-api-key",
)

point := &timeseries.Point{
    Measurement: "temperature",
    Tags:        map[string]string{"location": "office"},
    Fields:      map[string]interface{}{"value": 23.5},
    Timestamp:   time.Now(),
}

client.Write("metrics", point)
```

---

## 6. Cloud Platform Integrations

### 6.1 AWS Integration

**CloudWatch Metrics:**
```python
import boto3
from wia_timeseries import CloudWatchAdapter

adapter = CloudWatchAdapter(
    wia_client=client,
    namespace='MyApp'
)

adapter.sync_metrics()
```

**Lambda Function:**
```javascript
exports.handler = async (event) => {
  const client = new WIATimeSeries({/*...*/});
  await client.write({/*...*/});
};
```

### 6.2 Azure Integration

**Azure Monitor:**
```csharp
var adapter = new AzureMonitorAdapter(
    wiaClient: client,
    workspace: "workspace-id"
);

await adapter.SyncMetrics();
```

### 6.3 GCP Integration

**Cloud Monitoring:**
```go
adapter := monitoring.NewAdapter(
    wiaClient: client,
    projectID: "my-project",
)

adapter.SyncMetrics(ctx)
```

---

## 7. Stream Processing

### 7.1 Apache Kafka

```java
Properties props = new Properties();
props.put("bootstrap.servers", "localhost:9092");

KafkaProducer<String, TimeSeriesPoint> producer = 
    new KafkaProducer<>(props);

producer.send(new ProducerRecord<>(
    "timeseries-topic", 
    timeSeriesPoint
));
```

### 7.2 Apache Flink

```java
DataStream<TimeSeriesPoint> stream = env
    .addSource(new WIATimeSeriesSource())
    .keyBy(point -> point.getMeasurement())
    .window(TumblingEventTimeWindows.of(Time.minutes(5)))
    .aggregate(new AverageAggregator());
```

### 7.3 Apache Spark

```python
df = spark.read \
    .format("wia-timeseries") \
    .option("url", "https://api.example.com/v1") \
    .option("database", "metrics") \
    .load()

df.groupBy(window("timestamp", "5 minutes")) \
    .agg(avg("value")) \
    .write \
    .format("wia-timeseries") \
    .save()
```

---

## 8. Migration Tools

### 8.1 Data Import

```bash
wia-import \
  --source influxdb://localhost:8086/mydb \
  --target wia://api.example.com/v1/metrics \
  --api-key YOUR_API_KEY \
  --batch-size 10000
```

### 8.2 Data Export

```bash
wia-export \
  --source wia://api.example.com/v1/metrics \
  --target influxdb://localhost:8086/mydb \
  --start 2025-01-01T00:00:00Z \
  --end 2025-12-31T23:59:59Z
```

---

## 9. Testing and Validation

### 9.1 Integration Tests

```python
def test_wia_integration():
    client = WIAClient(url=TEST_URL, api_key=TEST_KEY)
    
    # Write test data
    client.write(
        database='test',
        measurement='test_metric',
        fields={'value': 42}
    )
    
    # Query and verify
    results = client.query(
        database='test',
        measurement='test_metric',
        start='-1h'
    )
    
    assert len(results) == 1
    assert results[0]['value'] == 42
```

### 9.2 Compatibility Tests

```bash
# Run compatibility test suite
wia-test-suite \
  --endpoint https://api.example.com/v1 \
  --phase 1,2,3,4 \
  --report compatibility-report.html
```

---

## 10. Best Practices

### 10.1 Connection Management

- Use connection pooling
- Implement retry logic
- Handle backpressure
- Monitor connection health

### 10.2 Data Consistency

- Use idempotency keys
- Implement deduplication
- Handle clock skew
- Validate timestamps

### 10.3 Performance Optimization

- Batch write operations
- Use appropriate compression
- Cache metadata
- Optimize query patterns

---

## 11. Compliance Checklist

WIA-DATA-014 PHASE 4 compliant integrations MUST:

- [ ] Support standard data format (PHASE 1)
- [ ] Use standard API (PHASE 2)
- [ ] Implement secure protocols (PHASE 3)
- [ ] Provide SDK or connector
- [ ] Document integration process
- [ ] Include error handling
- [ ] Support authentication
- [ ] Validate data before writing

Integrations SHOULD:

- [ ] Support batch operations
- [ ] Implement retry logic
- [ ] Provide configuration examples
- [ ] Include integration tests
- [ ] Monitor integration health
- [ ] Support multiple environments

---

## Appendix — Integration Implementation Notes

### Conformance test suite

A black-box test suite is published at
`https://github.com/WIA-Official/wia-time-series-data-conformance` and
walks through the public surface of this Phase plus the cross-Phase
integration scenarios. Hosts publishing `bridge_profile=Full` SHOULD
additionally pass the suite's extension tests for at least one
supported third-party time-series database (InfluxDB, Prometheus,
TimescaleDB, Apache IoTDB, VictoriaMetrics).

### Reference container

The `wia/time-series-data-host:1.0.0` container image implements every
endpoint specified in this Phase with mock data; integrators exercise
their bridge against it before going to production. The container ships
with a SQLite-backed store suitable for small-to-medium scale (up to
10 million points per series); production deployments typically swap in
a purpose-built time-series database via the WIA-supplied schema.

### Companion CLI

The `cli/time-series-data.sh` script ships sample envelope generators
(validate, series, point, query, rollup, info subcommands) so an
implementer can produce conformant payloads without hand-rolling JSON.
The CLI has no dependency beyond `jq` and POSIX shell, so it runs in
any CI environment without additional tooling installation.

### Operational considerations

Time-series data infrastructure has three operational considerations
that integrators consistently underestimate. First, retention policy
scales aggressively with cardinality; the standard recommends explicit
retention windows per series in the Phase 1 series envelope. Second,
rollup lag (the delay between a raw point arriving and being included
in a downsampled aggregate) is a UX issue for live dashboards; the
standard recommends documenting the rollup latency budget in the host’s
discovery document. Third, late-arriving points (out-of-order timestamps
from mobile devices on flaky networks) are common; hosts MUST accept
late points up to a configurable bound (default 24 hours) and MUST
trigger a re-rollup of the affected window.

### Backwards-compatibility promise

Within the 1.x line every endpoint and envelope listed in this Phase
MUST remain reachable and MUST continue to honour the documented
status codes and content shapes. Hosts MAY add optional query parameters,
response fields, new endpoints, or media types. Hosts MUST NOT remove
or repurpose existing ones. Breaking changes ride a major version bump
and MUST be preceded by a 12-month deprecation window per IETF
RFC 8594 and RFC 9745.

## Appendix — Worked Bridge Configuration

A representative scenario for a mid-scale industrial deployment:
the operator runs 200 production lines, each line has 50 sensors,
each sensor emits one measurement per second. Total ingest rate is
10 000 points per second.

```
series_id : ts_line_47_temp_sensor_3
labels    : { line=47, sensor=3, region=plant-A, env=prod }
unit      : Cel
cadence   : 1 Hz raw, 60 s avg downsample after 1h, 1h avg after 7d
retention : 90 days raw, 1y rollup, 5y archive
```

The 200 lines × 50 sensors = 10 000 distinct series; per series
cardinality (label combinations) is bounded at 1 since the sensor is
fixed to a line. At 1 Hz × 10 000 series × 86 400 seconds/day = 864 M
points/day, which a SQLite-backed reference container can serve
directly until raw retention boundary at 90 days. Production deployments
typically swap to a purpose-built time-series database (InfluxDB,
TimescaleDB, VictoriaMetrics, Apache IoTDB) once daily ingest exceeds
100 M points.

The rollup policy (1 Hz → 60 s avg after 1h, then 1h avg after 7d)
reduces storage by ~98% over the first 7 days while preserving the
operationally meaningful aggregate. Late-arriving points (the standard
accepts them up to 24 hours) trigger a re-rollup of the affected
60 s window; the re-rollup MUST emit a `rollup_updated` notice to
subscribers so live dashboards can refresh.

## Appendix — Conformance and Verification

Every host claiming conformance MUST pass the open conformance test
suite at `https://github.com/WIA-Official/wia-time-series-data-conformance`.
The suite runs against the host’s public Phase 2 endpoints and
verifies envelope round-trips, idempotency-key handling, replay-defence
bounds, audience-control enforcement, and the late-arriving-points
re-rollup contract.

Hosts publishing `bridge_profile=Full` SHOULD additionally pass the
suite’s extension tests for at least one supported third-party
time-series database. Bridge tests verify that the WIA wire format
maps deterministically to the target database’s native ingest format
without information loss.

---

**License:** MIT
**Copyright:** © 2025 WIA Standards
**弘益人間 (Benefit All Humanity)**
