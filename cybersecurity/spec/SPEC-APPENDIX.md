# WIA-SEC-015 Cybersecurity Standard
## Appendix: Reference Materials & Examples

---

**Version**: 1.0.0
**Standard ID**: WIA-SEC-015
**Date**: 2025-12-25
**License**: MIT

---

## 목차 (Table of Contents)

1. [JSON 스키마 (JSON Schemas)](#json-스키마)
2. [코드 예제 (Code Examples)](#코드-예제)
3. [배포 가이드 (Deployment Guides)](#배포-가이드)
4. [테스트 케이스 (Test Cases)](#테스트-케이스)
5. [보안 체크리스트 (Security Checklists)](#보안-체크리스트)
6. [FAQ](#faq)

---

## JSON 스키마

### A.1 Security Event Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.org/schemas/security-event-v1.0.json",
  "title": "WIA-SEC-015 Security Event",
  "type": "object",
  "required": ["version", "standard", "event_id", "timestamp", "event_type", "severity"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$",
      "description": "Schema version"
    },
    "standard": {
      "type": "string",
      "const": "WIA-SEC-015",
      "description": "Standard identifier"
    },
    "event_id": {
      "type": "string",
      "pattern": "^EVT-\\d{4}-[A-Z0-9]+$",
      "description": "Unique event identifier"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 timestamp"
    },
    "event_type": {
      "type": "string",
      "enum": [
        "intrusion_attempt",
        "malware_detection",
        "ddos_attack",
        "data_breach",
        "unauthorized_access",
        "policy_violation",
        "vulnerability_exploit",
        "insider_threat"
      ]
    },
    "severity": {
      "type": "string",
      "enum": ["critical", "high", "medium", "low", "info"]
    },
    "source": {
      "type": "object",
      "required": ["ip_address"],
      "properties": {
        "ip_address": {
          "type": "string",
          "format": "ipv4"
        },
        "port": {
          "type": "integer",
          "minimum": 1,
          "maximum": 65535
        },
        "hostname": {
          "type": "string"
        },
        "geolocation": {
          "type": "object",
          "properties": {
            "country": {"type": "string"},
            "city": {"type": "string"},
            "latitude": {"type": "number"},
            "longitude": {"type": "number"}
          }
        }
      }
    },
    "destination": {
      "type": "object",
      "required": ["ip_address"],
      "properties": {
        "ip_address": {"type": "string", "format": "ipv4"},
        "port": {"type": "integer"},
        "hostname": {"type": "string"},
        "asset_id": {"type": "string"},
        "criticality": {
          "type": "string",
          "enum": ["critical", "high", "medium", "low"]
        }
      }
    },
    "detection": {
      "type": "object",
      "properties": {
        "method": {
          "type": "string",
          "enum": ["signature", "anomaly_detection", "behavioral_analysis", "threat_intelligence"]
        },
        "rule_id": {"type": "string"},
        "confidence_score": {
          "type": "number",
          "minimum": 0,
          "maximum": 1
        }
      }
    }
  }
}
```

### A.2 Threat Analysis Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.org/schemas/threat-analysis-v1.0.json",
  "title": "WIA-SEC-015 Threat Analysis",
  "type": "object",
  "required": ["analysis_id", "timestamp", "threat_detected", "severity"],
  "properties": {
    "analysis_id": {
      "type": "string",
      "pattern": "^ANALYSIS-[A-Z0-9]+$"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "threat_detected": {
      "type": "boolean"
    },
    "severity": {
      "type": "string",
      "enum": ["critical", "high", "medium", "low", "none"]
    },
    "confidence": {
      "type": "number",
      "minimum": 0,
      "maximum": 1
    },
    "mitre_tactics": {
      "type": "array",
      "items": {
        "type": "string",
        "pattern": "^TA\\d{4}$"
      }
    },
    "mitre_techniques": {
      "type": "array",
      "items": {
        "type": "string",
        "pattern": "^T\\d{4}(\\.\\d{3})?$"
      }
    },
    "indicators_of_compromise": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "type": {
            "type": "string",
            "enum": ["ip", "domain", "hash", "url", "email"]
          },
          "value": {"type": "string"},
          "confidence": {"type": "number"}
        }
      }
    },
    "recommended_actions": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "action": {"type": "string"},
          "priority": {
            "type": "string",
            "enum": ["immediate", "high", "medium", "low"]
          },
          "automated": {"type": "boolean"}
        }
      }
    }
  }
}
```

---

## 코드 예제

### B.1 Complete Security Monitoring Example

```typescript
// Complete example of WIA-SEC-015 implementation
import WiaSecuritySDK from 'wia-security-sdk';
import { SecurityEvent, ThreatAnalysis } from './types';

class SecurityMonitor {
  private sdk: WiaSecuritySDK;
  private eventQueue: SecurityEvent[] = [];
  private isRunning: boolean = false;

  constructor(apiKey: string) {
    this.sdk = new WiaSecuritySDK({
      apiKey,
      endpoint: 'https://api.wia.security'
    });
  }

  async start(): Promise<void> {
    this.isRunning = true;
    console.log('Security Monitor started');

    // Start monitoring threads
    await Promise.all([
      this.monitorNetworkTraffic(),
      this.monitorFileSystem(),
      this.monitorUserBehavior(),
      this.processEventQueue()
    ]);
  }

  private async monitorNetworkTraffic(): Promise<void> {
    while (this.isRunning) {
      const packets = await this.captureNetworkPackets();

      for (const packet of packets) {
        const analysis = await this.sdk.analyzeThreat({
          type: 'network_traffic',
          data: packet
        });

        if (analysis.threat_detected) {
          await this.handleThreat(packet, analysis);
        }
      }

      await this.sleep(100);
    }
  }

  private async handleThreat(
    data: any,
    analysis: ThreatAnalysis
  ): Promise<void> {
    const event: SecurityEvent = {
      version: '1.0.0',
      standard: 'WIA-SEC-015',
      event_id: this.generateEventId(),
      timestamp: new Date().toISOString(),
      event_type: this.mapToEventType(analysis),
      severity: analysis.severity,
      source: data.source,
      destination: data.destination,
      detection: {
        method: 'anomaly_detection',
        confidence_score: analysis.confidence,
        models_triggered: analysis.models_triggered
      }
    };

    // Report to SIEM
    await this.sdk.reportSecurityEvent(event);

    // Auto-respond if critical
    if (analysis.severity === 'critical') {
      await this.executeAutoResponse(event, analysis);
    }
  }

  private async executeAutoResponse(
    event: SecurityEvent,
    analysis: ThreatAnalysis
  ): Promise<void> {
    console.log(`Auto-responding to ${event.event_type}`);

    for (const action of analysis.recommended_actions) {
      if (action.automated && action.priority === 'immediate') {
        switch (action.action) {
          case 'block_ip':
            await this.blockIP(event.source.ip_address);
            break;
          case 'quarantine_device':
            await this.quarantineDevice(event.destination.asset_id);
            break;
          case 'kill_process':
            await this.killSuspiciousProcess(event.destination.asset_id);
            break;
        }
      }
    }
  }

  private async blockIP(ip: string): Promise<void> {
    // Integrate with firewall
    console.log(`Blocking IP: ${ip}`);
    // Implementation depends on firewall type
  }

  private async quarantineDevice(assetId: string): Promise<void> {
    // Isolate device from network
    console.log(`Quarantining device: ${assetId}`);
    // Implementation depends on EDR/MDM
  }
}

// Usage
const monitor = new SecurityMonitor('your-api-key');
monitor.start();
```

### B.2 Python SIEM Integration Example

```python
#!/usr/bin/env python3
"""
WIA-SEC-015 SIEM Integration Example
Demonstrates complete SIEM integration with automated response
"""

import asyncio
import json
from datetime import datetime
from typing import Dict, List
from wia_security import WiaSecuritySDK

class SIEMIntegration:
    def __init__(self, api_key: str, siem_config: Dict):
        self.sdk = WiaSecuritySDK(api_key)
        self.siem_config = siem_config
        self.event_buffer = []
        self.max_buffer_size = 1000

    async def start(self):
        """Start SIEM integration"""
        print("Starting SIEM Integration...")

        tasks = [
            self.collect_events(),
            self.process_events(),
            self.flush_buffer_periodically()
        ]

        await asyncio.gather(*tasks)

    async def collect_events(self):
        """Collect security events from various sources"""
        while True:
            # Collect from network sensors
            network_events = await self.collect_network_events()

            # Collect from endpoint agents
            endpoint_events = await self.collect_endpoint_events()

            # Collect from application logs
            app_events = await self.collect_application_events()

            all_events = network_events + endpoint_events + app_events

            for event in all_events:
                enriched = await self.enrich_event(event)
                self.event_buffer.append(enriched)

            await asyncio.sleep(1)

    async def enrich_event(self, event: Dict) -> Dict:
        """Enrich event with threat intelligence"""
        # Add threat intelligence
        if 'source' in event and 'ip_address' in event['source']:
            threat_intel = await self.sdk.check_threat_intelligence(
                event['source']['ip_address']
            )
            event['threat_intelligence'] = threat_intel

        # Add geolocation
        event['geolocation'] = await self.get_geolocation(
            event['source']['ip_address']
        )

        # Add MITRE ATT&CK mapping
        event['mitre_mapping'] = await self.map_to_mitre(event)

        return event

    async def process_events(self):
        """Process and analyze events"""
        while True:
            if not self.event_buffer:
                await asyncio.sleep(0.1)
                continue

            event = self.event_buffer.pop(0)

            # Analyze threat
            analysis = await self.sdk.analyze_threat(event)

            if analysis['threat_detected']:
                # High severity - immediate action
                if analysis['severity'] in ['critical', 'high']:
                    await self.handle_high_severity_threat(event, analysis)
                else:
                    # Lower severity - queue for analyst
                    await self.queue_for_review(event, analysis)

            # Send to SIEM
            await self.send_to_siem(event, analysis)

    async def handle_high_severity_threat(
        self,
        event: Dict,
        analysis: Dict
    ):
        """Handle high severity threats"""
        print(f"🚨 HIGH SEVERITY THREAT DETECTED: {event['event_type']}")

        # Create incident
        incident = await self.sdk.create_incident({
            'severity': analysis['severity'],
            'event': event,
            'analysis': analysis
        })

        # Auto-respond
        for action in analysis.get('recommended_actions', []):
            if action.get('automated'):
                await self.execute_action(action, event)

        # Notify SOC
        await self.notify_soc(incident)

    async def execute_action(self, action: Dict, event: Dict):
        """Execute automated response action"""
        action_type = action['action']

        if action_type == 'block_ip':
            await self.block_ip(event['source']['ip_address'])
        elif action_type == 'quarantine_endpoint':
            await self.quarantine_endpoint(event['destination']['asset_id'])
        elif action_type == 'disable_account':
            await self.disable_user_account(event['user']['id'])

    async def send_to_siem(self, event: Dict, analysis: Dict):
        """Send event to SIEM platform"""
        siem_event = {
            **event,
            'analysis': analysis,
            'timestamp': datetime.utcnow().isoformat()
        }

        if self.siem_config['type'] == 'splunk':
            await self.send_to_splunk(siem_event)
        elif self.siem_config['type'] == 'qradar':
            await self.send_to_qradar(siem_event)
        elif self.siem_config['type'] == 'sentinel':
            await self.send_to_sentinel(siem_event)

# Run
if __name__ == '__main__':
    config = {
        'type': 'splunk',
        'url': 'https://splunk.example.com:8088',
        'token': 'your-hec-token'
    }

    integration = SIEMIntegration('your-api-key', config)
    asyncio.run(integration.start())
```

---

## 배포 가이드

### C.1 Docker Deployment

```dockerfile
# Dockerfile for WIA-SEC-015 Security Platform
FROM ubuntu:22.04

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3.11 \
    python3-pip \
    nodejs \
    npm \
    redis-server \
    postgresql-client \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
COPY requirements.txt /app/
RUN pip3 install -r /app/requirements.txt

# Install Node packages
COPY package.json /app/
WORKDIR /app
RUN npm install

# Copy application
COPY . /app/

# Configure
ENV WIA_SEC_API_KEY=""
ENV WIA_SEC_ENDPOINT="https://api.wia.security"
ENV REDIS_URL="redis://localhost:6379"
ENV POSTGRES_URL="postgresql://user:pass@localhost/security"

# Expose ports
EXPOSE 8080 9090

# Health check
HEALTHCHECK --interval=30s --timeout=10s --retries=3 \
    CMD curl -f http://localhost:8080/health || exit 1

# Start
CMD ["python3", "/app/main.py"]
```

**docker-compose.yml**:

```yaml
version: '3.8'

services:
  security-platform:
    build: .
    ports:
      - "8080:8080"
      - "9090:9090"
    environment:
      - WIA_SEC_API_KEY=${API_KEY}
      - REDIS_URL=redis://redis:6379
      - POSTGRES_URL=postgresql://postgres:password@db:5432/security
    depends_on:
      - redis
      - db
    restart: unless-stopped

  redis:
    image: redis:7-alpine
    ports:
      - "6379:6379"
    volumes:
      - redis-data:/data

  db:
    image: postgres:15
    environment:
      - POSTGRES_DB=security
      - POSTGRES_USER=postgres
      - POSTGRES_PASSWORD=password
    volumes:
      - postgres-data:/var/lib/postgresql/data
    ports:
      - "5432:5432"

  threat-engine:
    build: ./threat-engine
    depends_on:
      - redis
      - db
    environment:
      - MODEL_PATH=/models
    volumes:
      - ./models:/models

volumes:
  redis-data:
  postgres-data:
```

### C.2 Kubernetes Deployment

```yaml
# kubernetes-deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: wia-security-platform
  namespace: security
spec:
  replicas: 3
  selector:
    matchLabels:
      app: wia-security
  template:
    metadata:
      labels:
        app: wia-security
    spec:
      containers:
      - name: security-platform
        image: wia/security-platform:1.0.0
        ports:
        - containerPort: 8080
          name: api
        - containerPort: 9090
          name: metrics
        env:
        - name: WIA_SEC_API_KEY
          valueFrom:
            secretKeyRef:
              name: wia-secrets
              key: api-key
        resources:
          requests:
            memory: "2Gi"
            cpu: "1000m"
          limits:
            memory: "4Gi"
            cpu: "2000m"
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
  name: wia-security-service
  namespace: security
spec:
  selector:
    app: wia-security
  ports:
  - name: api
    port: 80
    targetPort: 8080
  - name: metrics
    port: 9090
    targetPort: 9090
  type: LoadBalancer

---
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: wia-security-hpa
  namespace: security
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: wia-security-platform
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

---

## 테스트 케이스

### D.1 Threat Detection Tests

```python
import unittest
from wia_security import WiaSecuritySDK

class TestThreatDetection(unittest.TestCase):
    def setUp(self):
        self.sdk = WiaSecuritySDK(api_key='test-key')

    def test_malware_detection(self):
        """Test malware detection"""
        malware_sample = {
            'type': 'file',
            'hash': '5d41402abc4b2a76b9719d911017c592',
            'behavior': 'file_encryption',
            'network_activity': 'c2_communication'
        }

        result = self.sdk.analyze_threat(malware_sample)

        self.assertTrue(result['threat_detected'])
        self.assertEqual(result['severity'], 'critical')
        self.assertGreater(result['confidence'], 0.9)
        self.assertIn('T1486', result['mitre_techniques'])  # Ransomware

    def test_ddos_detection(self):
        """Test DDoS attack detection"""
        traffic_data = {
            'type': 'network',
            'requests_per_second': 50000,
            'unique_ips': 10000,
            'geographic_distribution': 'global',
            'pattern': 'syn_flood'
        }

        result = self.sdk.analyze_threat(traffic_data)

        self.assertTrue(result['threat_detected'])
        self.assertEqual(result['severity'], 'high')

    def test_false_positive_rate(self):
        """Ensure false positive rate is acceptable"""
        benign_samples = self.load_benign_samples(1000)
        false_positives = 0

        for sample in benign_samples:
            result = self.sdk.analyze_threat(sample)
            if result['threat_detected']:
                false_positives += 1

        fp_rate = false_positives / len(benign_samples)
        self.assertLess(fp_rate, 0.05)  # < 5% FP rate

if __name__ == '__main__':
    unittest.main()
```

---

## 보안 체크리스트

### E.1 Pre-Deployment Security Checklist

```markdown
## WIA-SEC-015 Pre-Deployment Checklist

### Authentication & Access Control
- [ ] Multi-factor authentication (MFA) enabled for all users
- [ ] Least privilege access model implemented
- [ ] Service accounts use strong, unique passwords
- [ ] API keys rotated regularly (90 days)
- [ ] SSH keys using Ed25519 or RSA-4096
- [ ] Password policy enforced (min 14 chars, complexity)

### Network Security
- [ ] Firewall rules configured and documented
- [ ] Network segmentation implemented
- [ ] VPN required for remote access
- [ ] DDoS protection enabled
- [ ] TLS 1.3 enforced for all connections
- [ ] Certificate pinning implemented

### Data Protection
- [ ] Data encrypted at rest (AES-256)
- [ ] Data encrypted in transit (TLS 1.3)
- [ ] Database encryption enabled
- [ ] Backup encryption enabled
- [ ] PII/sensitive data identified and protected
- [ ] Data Loss Prevention (DLP) configured

### Logging & Monitoring
- [ ] Centralized logging configured
- [ ] Log retention policy defined (min 1 year)
- [ ] SIEM integration tested
- [ ] Alert rules configured
- [ ] Log integrity protection enabled
- [ ] Audit logs immutable

### Incident Response
- [ ] Incident response plan documented
- [ ] SOC team trained and staffed
- [ ] Automated response playbooks configured
- [ ] Communication plan established
- [ ] Backup and recovery tested
- [ ] Forensics tools prepared

### Compliance
- [ ] Required compliance frameworks identified
- [ ] Controls mapped and implemented
- [ ] Compliance evidence collected
- [ ] Regular audits scheduled
- [ ] Compliance reports automated
- [ ] Remediation process defined

### Application Security
- [ ] Security testing completed (SAST/DAST)
- [ ] Vulnerability scan passed
- [ ] Penetration testing completed
- [ ] Dependencies scanned for vulnerabilities
- [ ] Security headers configured
- [ ] Input validation implemented

### Infrastructure
- [ ] Systems hardened per CIS benchmarks
- [ ] Unnecessary services disabled
- [ ] Patch management process established
- [ ] Configuration management implemented
- [ ] Secrets management solution deployed
- [ ] High availability configured
```

---

## FAQ

### F.1 General Questions

**Q: Is WIA-SEC-015 compatible with existing security tools?**

A: Yes, WIA-SEC-015 is designed to integrate with existing security infrastructure through standard APIs and protocols. We support major SIEM platforms (Splunk, QRadar, Sentinel), EDR solutions, firewalls, and IAM systems.

**Q: What are the minimum system requirements?**

A:
- CPU: 4 cores minimum, 8 cores recommended
- RAM: 8GB minimum, 16GB recommended
- Storage: 100GB minimum for logs
- Network: 1Gbps minimum
- OS: Linux (Ubuntu 22.04, RHEL 8+), Windows Server 2019+

**Q: How long does implementation take?**

A: Typical timeline:
- Basic deployment: 1-2 weeks
- Full integration: 4-8 weeks
- Production hardening: 2-4 weeks
Total: 2-3 months for complete implementation

**Q: What is the licensing model?**

A: WIA-SEC-015 core standard and reference implementations are open source (MIT License). Commercial support and enterprise features available through WIA partners.

---

**弘益人間 (Hongik Ingan) - Benefit All Humanity**

© 2025 WIA - World Certification Industry Association
Licensed under MIT License
