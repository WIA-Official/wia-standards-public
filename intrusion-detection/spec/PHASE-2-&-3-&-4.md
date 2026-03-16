# WIA-SEC-016: Intrusion Detection Standard
## PHASES 2, 3, & 4 - ADVANCED FEATURES

**Standard ID:** WIA-SEC-016
**Title:** Intrusion Detection and Prevention Systems (Advanced)
**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2025-12-25

---

# PHASE 2: ADVANCED DETECTION

## 1. Machine Learning-Based Detection

### 1.1 Supervised Learning
**Objective:** Train models on labeled attack datasets to classify traffic.

**Algorithms:**
- **Random Forest**: Ensemble decision trees for classification
  - Features: Packet size, inter-arrival time, protocol, flags
  - Training: 80/20 train-test split
  - Accuracy target: >92%

- **Support Vector Machines (SVM)**: Binary classification (benign/malicious)
  - Kernel: RBF (Radial Basis Function)
  - Feature normalization: Min-max scaling
  - Cross-validation: 5-fold

- **Neural Networks**: Deep learning for complex pattern recognition
  - Architecture: Input (50 features) → Dense(128) → Dense(64) → Dense(32) → Output(2)
  - Activation: ReLU hidden layers, Softmax output
  - Loss: Categorical crossentropy
  - Training epochs: 100 with early stopping

**Training Data Requirements:**
- Minimum 1 million labeled samples
- Balanced classes (50% benign, 50% malicious)
- Diverse attack types (DDoS, SQL injection, malware, port scans)
- Real-world traffic scenarios

### 1.2 Unsupervised Learning
**Objective:** Detect unknown attacks without labeled data.

**Techniques:**
- **K-Means Clustering**: Group similar traffic patterns
  - K selection: Elbow method or silhouette analysis
  - Distance metric: Euclidean or Mahalanobis
  - Outlier threshold: 3 standard deviations from cluster centroid

- **Isolation Forest**: Identify anomalous data points
  - Tree depth: log₂(sample size)
  - Contamination factor: 0.01-0.05
  - Use case: Zero-day attack detection

- **Autoencoder**: Neural network for anomaly detection
  - Architecture: Encoder-Bottleneck-Decoder
  - Reconstruction error threshold: 95th percentile of training error
  - Input: Traffic features normalized to [0,1]

**Performance Metrics:**
- Anomaly detection rate: >80%
- False positive rate: <10%
- Processing time: <50ms per sample

### 1.3 Feature Engineering
**Network Traffic Features (50+ total):**

**Packet-Level:**
- Packet size (bytes)
- Time-to-live (TTL)
- Protocol type (TCP/UDP/ICMP)
- TCP flags (SYN, ACK, FIN, RST, PSH, URG)
- Payload entropy (Shannon entropy)

**Flow-Level:**
- Flow duration
- Total packets (forward/backward)
- Total bytes (forward/backward)
- Packets per second
- Bytes per second
- Mean/min/max/std packet size
- Mean/min/max/std inter-arrival time

**Connection-Level:**
- Number of connections per IP (last 5 minutes)
- Unique destination IPs per source
- Port diversity (number of unique ports accessed)
- Protocol distribution

**Application-Level:**
- HTTP request method (GET/POST/PUT/DELETE)
- HTTP status code
- DNS query length
- TLS cipher suite
- SSL certificate validity

---

## 2. Behavioral Analysis

### 2.1 User and Entity Behavior Analytics (UEBA)
**Goal:** Detect insider threats and compromised accounts.

**Baseline Establishment:**
- Monitor user activity for 30-90 days
- Profile normal behaviors:
  - Login times (time of day, day of week)
  - Access patterns (resources accessed, frequency)
  - Geographic locations (IP geolocation)
  - Device fingerprints (user agent, OS)

**Anomaly Indicators:**
- Login from unusual location (geo-velocity analysis)
- Access to sensitive resources never accessed before
- Unusual data exfiltration volume
- Privilege escalation attempts
- Off-hours activity (statistically significant deviation)

**Scoring System:**
- Risk score: 0-100 (weighted sum of anomaly indicators)
- Threshold for alert: >70
- Threshold for automatic response: >90

### 2.2 Network Behavior Analysis
**Traffic Pattern Analysis:**
- Identify deviations in traffic volume, protocol distribution, connection patterns
- Detect lateral movement (east-west traffic anomalies)
- Identify command-and-control (C2) beaconing

**C2 Beaconing Detection:**
```python
def detect_beaconing(flows):
    # Analyze connection regularity
    intervals = calculate_intervals(flows)

    # Statistical tests
    mean_interval = np.mean(intervals)
    std_interval = np.std(intervals)
    coefficient_of_variation = std_interval / mean_interval

    # Beaconing indicators
    if coefficient_of_variation < 0.3:  # Regular intervals
        if len(flows) > 10:  # Persistent connections
            if mean_interval < 3600:  # Less than 1 hour
                return True, "High confidence C2 beaconing"

    return False, "Normal traffic"
```

---

# PHASE 3: INTEGRATION & DEPLOYMENT

## 3. SIEM Integration

### 3.1 Supported Protocols
**Syslog (RFC 5424):**
```
<Priority>Version Timestamp Hostname AppName ProcID MsgID [StructuredData] Message

Example:
<134>1 2025-12-25T10:30:15.123Z ids-01 wia-ids 1234 ID001 [ids alert_id="a1b2c3" severity="HIGH"] SQL Injection detected
```

**Common Event Format (CEF):**
```
CEF:Version|Device Vendor|Device Product|Device Version|Signature ID|Name|Severity|Extension

Example:
CEF:0|WIA|IDS|1.0|2100498|SQL Injection|8|src=203.0.113.45 dst=192.168.1.100 spt=54321 dpt=80
```

**Log Event Extended Format (LEEF):**
```
LEEF:Version|Vendor|Product|Version|EventID|Delimiter|Key-Value Pairs

Example:
LEEF:2.0|WIA|IDS|1.0|2100498|^|src=203.0.113.45^dst=192.168.1.100^severity=8
```

### 3.2 API Integration
**REST API Endpoints:**

**POST /api/v1/alerts** - Submit alert
```json
{
  "timestamp": "2025-12-25T10:30:15Z",
  "signature_id": 2100498,
  "severity": "HIGH",
  "source_ip": "203.0.113.45",
  "dest_ip": "192.168.1.100"
}
```

**GET /api/v1/alerts?start_time={ts}&severity={level}** - Query alerts

**GET /api/v1/sensors** - List active sensors

**POST /api/v1/rules** - Upload detection rules

**Authentication:**
- API Key in header: `X-API-Key: <key>`
- OAuth 2.0 client credentials flow
- JWT tokens with 1-hour expiration

### 3.3 Platform-Specific Integrations

#### Splunk Integration
**HTTP Event Collector (HEC):**
```bash
curl -k https://splunk-server:8088/services/collector \
  -H "Authorization: Splunk <HEC_TOKEN>" \
  -d '{
    "time": 1735123815,
    "source": "wia-ids",
    "sourcetype": "ids:alert",
    "event": {
      "signature_id": 2100498,
      "severity": "HIGH",
      "src_ip": "203.0.113.45"
    }
  }'
```

**Splunk Universal Forwarder:**
- Monitor IDS log files: `/var/log/wia-ids/alerts.log`
- Index: `security_ids`
- Source type: `ids:alert`

#### Elastic Stack (ELK)
**Filebeat Configuration:**
```yaml
filebeat.inputs:
- type: log
  enabled: true
  paths:
    - /var/log/wia-ids/alerts.json
  json.keys_under_root: true

output.elasticsearch:
  hosts: ["elasticsearch:9200"]
  index: "ids-alerts-%{+yyyy.MM.dd}"

processors:
  - add_host_metadata: ~
  - add_cloud_metadata: ~
```

**Elasticsearch Index Mapping:**
```json
{
  "mappings": {
    "properties": {
      "timestamp": { "type": "date" },
      "signature_id": { "type": "integer" },
      "severity": { "type": "keyword" },
      "source.ip": { "type": "ip" },
      "destination.ip": { "type": "ip" },
      "signature": { "type": "text" }
    }
  }
}
```

---

## 4. Deployment Models

### 4.1 Network Tap Deployment
**Passive Monitoring (IDS Mode):**
- Physical network tap or optical splitter
- No impact on network traffic
- Zero risk of network disruption
- Cannot block attacks (alert-only)

**Recommended for:**
- High-availability production networks
- Compliance monitoring
- Forensic analysis

**Architecture:**
```
Internet → Firewall → [Network Tap] → Switch → Internal Network
                            |
                        IDS Sensor
```

### 4.2 Inline Deployment
**Active Prevention (IPS Mode):**
- Installed in network path
- Inspects and can block traffic
- Single point of failure (requires HA setup)
- Minimal latency (<5ms)

**Recommended for:**
- Perimeter defense
- Critical asset protection
- Zero-trust networks

**Architecture:**
```
Internet → Firewall → IPS Sensor → Switch → Internal Network
                      (inline)
```

**High Availability (HA):**
- Active-Passive or Active-Active clustering
- Heartbeat monitoring (VRRP or proprietary)
- Synchronized rule sets and state tables
- Automatic failover <2 seconds

### 4.3 Hybrid Deployment
**Combination of IDS and IPS:**
- IPS at network perimeter (inline)
- IDS at internal segments (passive)
- Host-based IDS on critical servers

**Benefits:**
- Defense in depth
- Minimal performance impact on internal networks
- Comprehensive visibility

### 4.4 Cloud Deployment
**AWS Integration:**
- VPC Traffic Mirroring to IDS instance
- AWS Transit Gateway for centralized inspection
- CloudWatch Logs for SIEM integration

**Azure Integration:**
- Azure Network Watcher packet capture
- Virtual Network TAP
- Azure Sentinel integration

**GCP Integration:**
- VPC Packet Mirroring
- Cloud Logging export to IDS
- Chronicle SIEM integration

---

# PHASE 4: ADVANCED FEATURES

## 5. Threat Intelligence Integration

### 5.1 Threat Feed Ingestion
**Supported Formats:**
- STIX 2.1 (Structured Threat Information Expression)
- TAXII 2.1 (Trusted Automated eXchange of Indicator Information)
- OpenIOC (Open Indicators of Compromise)
- MISP (Malware Information Sharing Platform)

**STIX Indicator Example:**
```json
{
  "type": "indicator",
  "spec_version": "2.1",
  "id": "indicator--8e2e2d2b-17d4-4cbf-938f-98ee46b3cd3f",
  "created": "2025-12-25T10:00:00.000Z",
  "modified": "2025-12-25T10:00:00.000Z",
  "name": "Malicious IP - Emotet C2",
  "pattern": "[ipv4-addr:value = '203.0.113.45']",
  "pattern_type": "stix",
  "valid_from": "2025-12-25T10:00:00.000Z",
  "indicator_types": ["malicious-activity"]
}
```

### 5.2 IP Reputation
**Reputation Sources:**
- Commercial feeds (Talos, Proofpoint ET Intelligence)
- Open-source feeds (Abuse.ch, AlienVault OTX)
- Internal reputation (learned from past incidents)

**Scoring:**
- Score range: 0 (benign) - 100 (malicious)
- Thresholds:
  - 0-30: Low risk (log only)
  - 31-70: Medium risk (alert)
  - 71-100: High risk (block in IPS mode)

**Enrichment:**
```json
{
  "ip": "203.0.113.45",
  "reputation_score": 85,
  "categories": ["malware-c2", "phishing"],
  "first_seen": "2025-11-20T00:00:00Z",
  "last_seen": "2025-12-25T09:00:00Z",
  "geo": {
    "country": "RU",
    "asn": 12345,
    "org": "Suspicious Hosting Inc."
  }
}
```

### 5.3 MITRE ATT&CK Mapping
**Map Alerts to Tactics and Techniques:**
```json
{
  "alert_id": "a1b2c3d4",
  "signature": "SQL Injection Attempt",
  "mitre_attack": {
    "tactic": "TA0001 - Initial Access",
    "technique": "T1190 - Exploit Public-Facing Application",
    "sub_technique": "T1190.001 - SQL Injection"
  }
}
```

**Benefits:**
- Understand attacker TTPs (Tactics, Techniques, Procedures)
- Correlate alerts across kill chain
- Prioritize defensive measures

---

## 6. Incident Response Integration

### 6.1 Automated Playbooks
**SOAR (Security Orchestration, Automation, and Response):**

**Playbook Example - High Severity Alert:**
```yaml
trigger:
  event: "alert_created"
  condition: "severity == 'HIGH'"

actions:
  - step: 1
    action: "enrich_alert"
    params:
      - geo_lookup: true
      - threat_intel_check: true
      - whois_lookup: true

  - step: 2
    action: "notify_soc"
    params:
      channels: ["email", "slack", "pagerduty"]

  - step: 3
    action: "quarantine_host"
    condition: "threat_score > 90"
    params:
      method: "firewall_block"
      duration: "1h"

  - step: 4
    action: "create_ticket"
    params:
      system: "ServiceNow"
      priority: "P1"
      assignment_group: "Security Incident Response"
```

### 6.2 Notification Channels
**Email:**
- SMTP/SMTPS support
- HTML formatted alerts with embedded details
- Attachments: PCAP file, alert JSON

**SMS:**
- Twilio API integration
- Maximum 160 characters summary
- Critical alerts only (severity >= HIGH)

**Webhook:**
- HTTP POST to custom endpoint
- JSON payload
- Retry logic with exponential backoff

**SNMP Trap:**
- SNMPv3 for security
- Custom MIB for WIA-IDS alerts
- Integration with network management systems (NMS)

**Slack/Teams:**
- Rich message formatting
- Interactive buttons (acknowledge, escalate, dismiss)
- Channel routing based on severity

---

## 7. Compliance Reporting

### 7.1 PCI DSS Reports
**Requirement 11.4 - Intrusion Detection:**
- Alert summary by severity
- Top triggered signatures
- Response times (MTTD, MTTA, MTTR)
- Evidence of daily signature updates
- Quarterly review documentation

### 7.2 HIPAA Reports
**164.312(b) - Audit Controls:**
- Access logs to ePHI systems
- Failed authentication attempts
- Malware detection events
- Network anomalies near ePHI data stores

### 7.3 ISO 27001 Reports
**Control 8.16 - Monitoring Activities:**
- Security event trends
- Incident response metrics
- System availability and performance
- Policy violations

**Report Format:**
```json
{
  "report_id": "rpt-2025-12",
  "period": {
    "start": "2025-12-01T00:00:00Z",
    "end": "2025-12-31T23:59:59Z"
  },
  "summary": {
    "total_alerts": 1247,
    "critical": 12,
    "high": 89,
    "medium": 456,
    "low": 690
  },
  "top_signatures": [
    {"id": 2100498, "name": "SQL Injection", "count": 234},
    {"id": 2100499, "name": "XSS Attempt", "count": 189}
  ],
  "response_times": {
    "mean_time_to_detect": "45s",
    "mean_time_to_alert": "8s",
    "mean_time_to_respond": "12m"
  },
  "compliance_status": "PASS"
}
```

---

## 8. Performance Optimization

### 8.1 Hardware Acceleration
**FPGA (Field-Programmable Gate Array):**
- Offload signature matching to FPGA
- 10-100x performance improvement
- Supports 100+ Gbps throughput

**GPU Acceleration:**
- Use CUDA for ML inference
- Real-time anomaly detection on GPU
- TensorRT optimization for neural networks

**DPDK (Data Plane Development Kit):**
- Bypass kernel for packet processing
- Zero-copy packet forwarding
- <1 microsecond latency

### 8.2 Multi-Threading
**Packet Processing Pipeline:**
1. Capture thread (dedicated CPU core)
2. Decoder threads (2-4 cores)
3. Detection threads (8-16 cores)
4. Output threads (2 cores)

**Thread Affinity:**
- Pin threads to specific CPU cores
- NUMA-aware memory allocation
- Minimize cache misses

### 8.3 Rule Optimization
**Signature Pruning:**
- Disable irrelevant signatures (e.g., Windows rules on Linux networks)
- Group rules by protocol
- Use fast pattern matching (Boyer-Moore)

**Threshold Tuning:**
- Suppress noisy signatures
- Aggregate similar alerts (event_filter)
- Rate-limit alerts per source IP

---

## Appendix: Advanced Configuration Examples

### Example 1: Machine Learning Model Deployment
```python
import joblib
import numpy as np

# Load pre-trained model
model = joblib.load('ids_random_forest.pkl')

# Extract features from packet
def extract_features(packet):
    return np.array([
        len(packet),
        packet.ttl,
        packet.protocol,
        packet.tcp_flags,
        calculate_entropy(packet.payload)
    ]).reshape(1, -1)

# Predict
features = extract_features(packet)
prediction = model.predict(features)
probability = model.predict_proba(features)

if prediction == 1 and probability[0][1] > 0.85:
    generate_alert("ML Model: Malicious traffic detected", confidence=probability[0][1])
```

### Example 2: SIEM Correlation Rule
```python
# Splunk SPL (Search Processing Language)
index="ids_alerts" severity="HIGH"
| stats count by source_ip
| where count > 5
| join source_ip [
    search index="firewall" action="allowed"
    | stats count by src_ip
    | rename src_ip as source_ip
]
| eval risk_score = count * 10
| where risk_score > 50
| alert
```

---

**Document Control:**
- Author: WIA Security Standards Committee
- Effective Date: 2025-12-25
- Review Cycle: Annual
- Next Review: 2026-12-25

**弘益人間 · Benefit All Humanity**

© 2025 World Certification Industry Association (WIA)
