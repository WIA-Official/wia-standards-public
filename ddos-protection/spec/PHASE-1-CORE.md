# WIA-SEC-021: DDoS Protection - Phase 1: Core

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01
**Primary Color:** #8B5CF6 (Purple - Security)

---

## 1. Overview

This specification defines the foundational standards for Distributed Denial of Service (DDoS) attack protection, covering detection, mitigation, and prevention mechanisms for volumetric, protocol, and application-layer attacks. It establishes comprehensive protection strategies in accordance with RFC 4732 (DDoS Mitigation), NIST SP 800-61 (Incident Handling), and industry best practices.

### 1.1 Scope

- **Attack Types**: Volumetric, Protocol, Application-layer attacks
- **Protection Layers**: Network (L3), Transport (L4), Application (L7)
- **Mitigation Strategies**: Traffic scrubbing, rate limiting, CDN integration
- **Compliance**: NIST SP 800-61, RFC 4732, ISO 27001

### 1.2 Normative References

- **RFC 4732**: Internet Denial-of-Service Considerations
- **RFC 5635**: Remote Triggered Black Hole Filtering
- **NIST SP 800-61 Rev. 2**: Computer Security Incident Handling Guide
- **ISO/IEC 27001**: Information Security Management
- **OWASP DDoS Prevention Cheat Sheet**
- **IETF BCP 38**: Network Ingress Filtering

---

## 2. DDoS Attack Types

### 2.1 Volumetric Attacks

Volumetric attacks aim to consume all available bandwidth between the target and the Internet.

#### 2.1.1 UDP Flood

**Description**: Overwhelming the target with UDP packets, typically targeting random ports.

**Characteristics**:
- **Traffic Volume**: 10-100+ Gbps
- **Packet Rate**: 1M-100M packets per second
- **Source IPs**: Often spoofed from 10K-1M+ addresses
- **Target Ports**: Random or specific services (DNS, NTP, etc.)

**Detection Criteria**:
```json
{
  "attackType": "UDP_FLOOD",
  "detectionThresholds": {
    "bandwidthUtilization": ">= 80%",
    "udpPacketRate": "> 500000 pps",
    "sourceIPDiversity": "> 10000 unique IPs",
    "portDistribution": "random or concentrated"
  },
  "indicators": {
    "asymmetricTraffic": true,
    "spoofedSourceIPs": true,
    "fragmentedPackets": "common",
    "tcpHandshakeCompletion": "N/A"
  }
}
```

#### 2.1.2 ICMP Flood

**Description**: Overwhelming the target with ICMP Echo Request (ping) packets.

**Characteristics**:
- **Traffic Volume**: 1-50 Gbps
- **Packet Size**: Typically small (64-1500 bytes)
- **Response Impact**: Server resources consumed generating replies
- **Amplification**: None (1:1 ratio)

**Mitigation Strategy**:
```json
{
  "mitigationType": "ICMP_FLOOD_MITIGATION",
  "strategies": [
    {
      "method": "RATE_LIMITING",
      "parameters": {
        "maxICMPRate": "100 pps per source IP",
        "globalICMPLimit": "10000 pps",
        "burstAllowance": 500
      }
    },
    {
      "method": "SELECTIVE_DROP",
      "parameters": {
        "dropThreshold": "when bandwidth > 70%",
        "prioritizeLegitimate": true
      }
    }
  ]
}
```

#### 2.1.3 DNS Amplification

**Description**: Reflection attack using open DNS resolvers to amplify traffic (up to 70x amplification).

**Attack Mechanics**:
```
1. Attacker sends DNS query with spoofed source IP (victim's IP)
2. DNS resolver sends large response to victim
3. Amplification factor: 28-54 bytes query → 3000+ bytes response
4. Thousands of open resolvers exploited simultaneously
```

**Data Format**:
```json
{
  "attackType": "DNS_AMPLIFICATION",
  "amplificationFactor": 70,
  "queryType": "ANY",
  "targetDomain": "example.com",
  "openResolvers": 15000,
  "metrics": {
    "querySizeaverage": 60,
    "responseSizeAverage": 4200,
    "totalBandwidth": "85 Gbps",
    "sourceIPsVisible": "victim IP (spoofed target)"
  },
  "mitigation": {
    "detectReflectorIPs": true,
    "blockDNSANYQueries": true,
    "rateLimitDNSTraffic": true,
    "sourceDomainValidation": true
  }
}
```

### 2.2 Protocol Attacks

Protocol attacks exploit weaknesses in Layer 3 and Layer 4 protocols to exhaust server resources or intermediate network equipment.

#### 2.2.1 SYN Flood

**Description**: TCP connection state exhaustion by sending SYN packets without completing the handshake.

**Attack Mechanism**:
```
Normal TCP Handshake:
  Client → Server: SYN
  Server → Client: SYN-ACK
  Client → Server: ACK (connection established)

SYN Flood Attack:
  Attacker → Server: SYN (with spoofed source IP)
  Server → Spoofed IP: SYN-ACK (never receives ACK)
  Server: Maintains half-open connection (resource leak)
  Result: Connection table full, legitimate connections denied
```

**Detection and Mitigation**:
```json
{
  "attackType": "SYN_FLOOD",
  "detection": {
    "synRateThreshold": "> 10000 SYN/sec",
    "halfOpenConnections": "> 50% of max connections",
    "synAckRatio": "< 0.3 (abnormal)",
    "sourceIPEntropy": "high (spoofed IPs)"
  },
  "mitigation": {
    "synCookies": {
      "enabled": true,
      "description": "Stateless TCP handshake validation",
      "implementation": "kernel-level or firewall"
    },
    "connectionTimeout": {
      "synReceivedTimeout": "3 seconds",
      "normalTimeout": "30 seconds"
    },
    "rateLimiting": {
      "perSourceIP": "100 SYN/sec",
      "global": "50000 SYN/sec"
    },
    "sourceValidation": {
      "reversePath": true,
      "geolocation": "block unexpected countries"
    }
  }
}
```

#### 2.2.2 ACK Flood

**Description**: Flooding the target with TCP ACK packets, consuming connection tracking resources.

**Characteristics**:
- **Bypass Mechanism**: Can bypass simple SYN flood protections
- **Target**: Stateful firewalls and load balancers
- **Packet Characteristics**: Valid-looking ACK packets for non-existent connections
- **Resource Impact**: Connection state table exhaustion

**Detection**:
```json
{
  "attackType": "ACK_FLOOD",
  "detectionSignatures": {
    "ackPacketsWithoutEstablishedConnection": true,
    "highACKRateWithLowDataTransfer": true,
    "invalidSequenceNumbers": "common",
    "spoofedSourceIPs": "often present"
  },
  "mitigationTechniques": {
    "strictConnectionTracking": true,
    "ackValidation": "verify sequence numbers",
    "statefulFiltering": "drop ACKs for non-existent connections",
    "connectionTimeout": "aggressive timeout for idle connections"
  }
}
```

### 2.3 Application Layer Attacks

Application layer (L7) attacks target the application itself with seemingly legitimate requests.

#### 2.3.1 HTTP Flood

**Description**: Overwhelming web servers with HTTP GET or POST requests that appear legitimate.

**Attack Variants**:
```json
{
  "attackType": "HTTP_FLOOD",
  "variants": [
    {
      "name": "GET_FLOOD",
      "description": "High volume of GET requests to resource-intensive pages",
      "targetPages": ["/search", "/api/query", "/report/generate"],
      "requestRate": "1000-100000 requests/sec",
      "characteristics": {
        "validHTTPHeaders": true,
        "completeTCPHandshake": true,
        "difficultToDistinguish": "from legitimate traffic"
      }
    },
    {
      "name": "POST_FLOOD",
      "description": "Expensive POST requests with large payloads",
      "targetEndpoints": ["/api/upload", "/search", "/login"],
      "payloadSize": "large or crafted to maximize processing",
      "characteristics": {
        "serverResourceConsumption": "database queries, file I/O",
        "amplificationEffect": "small request → large processing"
      }
    },
    {
      "name": "CACHE_BYPASS",
      "description": "Requests designed to bypass CDN cache",
      "techniques": [
        "Random query strings: /page?random=12345",
        "Cache-busting headers: Cache-Control: no-cache",
        "Unique user agents",
        "Session-specific requests"
      ]
    }
  ],
  "detectionChallenges": {
    "legitimateHTTP": true,
    "completeTCPConnections": true,
    "validUserAgents": "often rotated",
    "distributedSources": "botnet, not single IP"
  }
}
```

**Mitigation Strategies**:
```json
{
  "mitigationType": "HTTP_FLOOD_DEFENSE",
  "strategies": {
    "challengeResponse": {
      "javascriptChallenge": {
        "description": "Serve JavaScript that must be executed to prove browser",
        "effectiveness": "blocks simple HTTP bots",
        "falsePositiveRate": "low (browsers execute JS)"
      },
      "captcha": {
        "triggerCondition": "suspicious behavior detected",
        "effectiveness": "blocks automated bots",
        "userImpact": "moderate (user must solve CAPTCHA)"
      }
    },
    "rateLimiting": {
      "perIPAddress": "10 requests/sec",
      "perSession": "20 requests/sec",
      "perEndpoint": "1000 requests/sec global",
      "adaptive": "increase limits for verified legitimate users"
    },
    "behavioralAnalysis": {
      "mouseMovement": "track human-like behavior",
      "keystrokePattern": "detect automation",
      "navigationPattern": "analyze page flow",
      "sessionDuration": "flag unrealistic rapid navigation"
    },
    "connectionFingerprinting": {
      "tlsFingerprint": "detect bot TLS stacks",
      "httpHeaderOrder": "identify automated tools",
      "userAgentValidation": "verify UA matches TLS fingerprint"
    }
  }
}
```

#### 2.3.2 Slowloris

**Description**: Low-bandwidth attack that holds HTTP connections open indefinitely by slowly sending partial requests.

**Attack Mechanism**:
```
1. Attacker opens connection to web server
2. Sends partial HTTP request headers
3. Periodically sends additional headers to keep connection alive
4. Never completes the request
5. Repeats for hundreds/thousands of connections
6. Server's connection pool exhausted
7. Legitimate users cannot connect
```

**Data Structure**:
```json
{
  "attackType": "SLOWLORIS",
  "characteristics": {
    "bandwidth": "very low (few kbps per connection)",
    "connectionsPerAttacker": "100-1000",
    "requestCompletionTime": "never or extremely slow",
    "headerSendingInterval": "every 10-15 seconds"
  },
  "detection": {
    "indicators": [
      "High number of connections in HTTP header reading state",
      "Connections open for unusually long time",
      "Slow trickle of data on established connections",
      "Low data transfer despite connection count"
    ],
    "metrics": {
      "avgConnectionDuration": "> 60 seconds",
      "incompleteRequestRatio": "> 70%",
      "dataRatePerConnection": "< 100 bytes/sec"
    }
  },
  "mitigation": {
    "connectionTimeout": {
      "headerReadTimeout": "10 seconds",
      "requestBodyReadTimeout": "30 seconds",
      "keepAliveTimeout": "5 seconds"
    },
    "connectionLimits": {
      "perIPAddress": "10-20 concurrent connections",
      "global": "monitor and alert on threshold"
    },
    "reverseProxy": {
      "description": "Use reverse proxy to buffer slow clients",
      "implementation": "nginx, HAProxy, or CDN"
    },
    "minDataRate": {
      "enforceMinimumBytesPerSecond": true,
      "threshold": "150 bytes/sec",
      "action": "terminate slow connections"
    }
  }
}
```

---

## 3. Detection Mechanisms

### 3.1 Baseline Traffic Analysis

Establish normal traffic patterns to identify anomalies.

```json
{
  "baselineMetrics": {
    "collectionPeriod": "30 days",
    "dataPoints": {
      "avgRequestsPerSecond": 5000,
      "peakRequestsPerSecond": 15000,
      "avgBandwidth": "2 Gbps",
      "peakBandwidth": "8 Gbps",
      "topSourceCountries": ["US", "GB", "DE", "JP"],
      "protocolDistribution": {
        "http": "60%",
        "https": "35%",
        "other": "5%"
      },
      "avgConnectionDuration": "5 seconds",
      "avgPacketSize": 750
    },
    "anomalyThresholds": {
      "requestRateMultiplier": 3,
      "bandwidthMultiplier": 2.5,
      "newSourceCountryThreshold": "5% of total traffic"
    }
  }
}
```

### 3.2 Entropy-Based Detection

Measure randomness in traffic patterns to detect spoofed or botnet traffic.

```python
# Source IP Entropy Calculation
def calculate_source_ip_entropy(ip_addresses):
    """
    High entropy: Diverse source IPs (normal or distributed attack)
    Low entropy: Few source IPs (targeted attack or legitimate)
    Very high entropy with high volume: Likely spoofed IPs
    """
    from collections import Counter
    import math

    ip_counts = Counter(ip_addresses)
    total = len(ip_addresses)

    entropy = -sum((count/total) * math.log2(count/total)
                   for count in ip_counts.values())

    return {
        "entropy": entropy,
        "uniqueIPs": len(ip_counts),
        "totalPackets": total,
        "interpretation": classify_entropy(entropy, total)
    }

def classify_entropy(entropy, total):
    if entropy > 15 and total > 100000:
        return "SUSPECTED_SPOOFED_IPS"
    elif entropy > 12:
        return "HIGH_DIVERSITY_NORMAL_OR_BOTNET"
    elif entropy < 5:
        return "LOW_DIVERSITY_TARGETED"
    else:
        return "NORMAL_RANGE"
```

### 3.3 Machine Learning Detection

Advanced anomaly detection using machine learning models.

```json
{
  "mlDetectionModel": {
    "algorithm": "Isolation Forest + LSTM",
    "features": [
      "packets_per_second",
      "bytes_per_second",
      "source_ip_entropy",
      "destination_port_distribution",
      "packet_size_variance",
      "tcp_flags_distribution",
      "connection_duration_avg",
      "syn_ack_ratio",
      "http_request_rate",
      "geographic_distribution",
      "temporal_pattern"
    ],
    "trainingData": {
      "normalTrafficDays": 60,
      "knownAttackSamples": "labeled dataset",
      "updateFrequency": "weekly retraining"
    },
    "output": {
      "anomalyScore": "0.0 - 1.0",
      "attackProbability": "percentage",
      "attackTypeClassification": "UDP_FLOOD | SYN_FLOOD | HTTP_FLOOD | etc.",
      "confidence": "percentage"
    },
    "thresholds": {
      "lowRisk": "< 0.3",
      "mediumRisk": "0.3 - 0.7",
      "highRisk": "> 0.7",
      "autoMitigate": "> 0.85"
    }
  }
}
```

---

## 4. Core Mitigation Techniques

### 4.1 Traffic Scrubbing

Redirect traffic through scrubbing centers to filter malicious packets.

```json
{
  "trafficScrubbing": {
    "architecture": {
      "normalFlow": "Client → Origin Server",
      "underAttack": "Client → Scrub Center → Origin Server"
    },
    "scrubCenter": {
      "capacity": "100+ Gbps",
      "locations": ["us-east", "us-west", "eu-west", "ap-southeast"],
      "filteringTechniques": [
        "IP reputation filtering",
        "Protocol validation",
        "Rate limiting",
        "Behavioral analysis",
        "Signature-based detection"
      ],
      "latencyImpact": "5-20ms additional",
      "legitimateTrafficPreservation": "> 98%"
    },
    "activationTrigger": {
      "automatic": "when anomaly score > 0.8",
      "manual": "security team override",
      "bgpAnnouncement": {
        "description": "Redirect traffic via BGP route announcement",
        "implementationTime": "< 60 seconds"
      }
    },
    "flowDiagram": {
      "detection": "Monitoring system detects attack",
      "diversion": "BGP announcement redirects traffic to scrub center",
      "analysis": "Deep packet inspection and filtering",
      "forwarding": "Clean traffic forwarded to origin via GRE tunnel",
      "deactivation": "Return to normal routing when attack subsides"
    }
  }
}
```

### 4.2 Rate Limiting

Dynamically limit request rates based on source, destination, and behavior.

```json
{
  "rateLimiting": {
    "algorithms": {
      "tokenBucket": {
        "description": "Allow burst traffic within limits",
        "parameters": {
          "bucketSize": 100,
          "refillRate": "10 tokens/second",
          "action": "drop or queue when bucket empty"
        }
      },
      "leakyBucket": {
        "description": "Smooth traffic to constant rate",
        "parameters": {
          "bucketSize": 1000,
          "outflowRate": "100 packets/second",
          "action": "queue or drop overflow"
        }
      },
      "slidingWindow": {
        "description": "Track requests in time window",
        "parameters": {
          "windowSize": "60 seconds",
          "maxRequests": 600,
          "action": "reject requests beyond limit"
        }
      }
    },
    "limitingDimensions": {
      "perSourceIP": {
        "http": "10 req/sec",
        "tcp": "100 SYN/sec",
        "udp": "1000 pkt/sec",
        "icmp": "10 pkt/sec"
      },
      "perDestinationPort": {
        "port80": "100000 req/sec global",
        "port443": "100000 req/sec global",
        "dns": "50000 queries/sec global"
      },
      "perGeolocation": {
        "expectedCountries": "normal limits",
        "unexpectedCountries": "strict limits or block"
      },
      "perASN": {
        "reputationBased": "trusted ASNs get higher limits"
      }
    },
    "adaptiveRateLimiting": {
      "description": "Dynamically adjust limits based on attack severity",
      "implementation": {
        "normalConditions": "baseline limits",
        "underAttack": "reduce limits by 50-90%",
        "postAttack": "gradual increase over 10 minutes"
      }
    }
  }
}
```

### 4.3 Blackhole Routing

Discard traffic destined for targeted IP addresses.

```json
{
  "blackholeRouting": {
    "description": "Null route attack traffic at network edge",
    "useCase": "Last resort when infrastructure at risk",
    "types": {
      "destinationBased": {
        "description": "Drop all traffic to specific IP/prefix",
        "command": "ip route add blackhole 203.0.113.0/24",
        "impact": "Complete service disruption for target"
      },
      "sourceBased": {
        "description": "Drop traffic from specific sources",
        "command": "ip route add blackhole 198.51.100.0/24 src",
        "impact": "Block malicious source networks"
      },
      "rtbh": {
        "name": "Remotely Triggered Black Hole",
        "description": "Coordinate blackhole with upstream ISP",
        "rfc": "RFC 5635",
        "advantage": "Drops traffic before reaching your network"
      }
    },
    "implementation": {
      "trigger": "automated when bandwidth > 95%",
      "scope": "single IP or /32, avoid large prefixes",
      "duration": "5-60 minutes",
      "monitoring": "continuous evaluation for deactivation"
    },
    "tradeoff": {
      "advantage": "Immediate relief for infrastructure",
      "disadvantage": "Legitimate traffic also dropped"
    }
  }
}
```

---

## 5. Prevention Best Practices

### 5.1 Network Architecture

```json
{
  "preventionArchitecture": {
    "cdn": {
      "description": "Content Delivery Network absorbs traffic",
      "benefits": [
        "Distributed points of presence",
        "Built-in DDoS mitigation",
        "Caching reduces origin load",
        "Anycast routing"
      ],
      "providers": ["Cloudflare", "Akamai", "AWS CloudFront", "Fastly"]
    },
    "loadBalancing": {
      "description": "Distribute traffic across multiple servers",
      "techniques": [
        "Round-robin DNS",
        "Global server load balancing (GSLB)",
        "Anycast IP",
        "Geographic load balancing"
      ]
    },
    "redundancy": {
      "multipleDataCenters": "Geographic distribution",
      "multipleISPs": "Diverse network paths",
      "overprovisioningBandwidth": "2-10x normal capacity"
    },
    "firewall": {
      "statefulInspection": true,
      "connectionTracking": true,
      "synFloodProtection": "SYN cookies enabled",
      "rateLimiting": "per-IP and global"
    }
  }
}
```

### 5.2 Application Hardening

```json
{
  "applicationSecurity": {
    "inputValidation": {
      "sanitizeAllInputs": true,
      "limitRequestSize": "prevent large payload attacks",
      "timeoutExpensiveOperations": "prevent resource exhaustion"
    },
    "connectionManagement": {
      "maxConcurrentConnections": "per IP and global",
      "connectionTimeout": "aggressive for slow clients",
      "keepAliveTimeout": "short duration"
    },
    "resourceLimits": {
      "cpuThrottling": "per-request CPU limits",
      "memoryLimits": "prevent memory exhaustion",
      "databaseQueryTimeout": "kill long-running queries"
    },
    "caching": {
      "aggressiveCaching": "reduce origin server load",
      "cdn": "serve static content from edge",
      "apiResponseCaching": "cache frequent queries"
    }
  }
}
```

---

## 6. Monitoring and Alerting

### 6.1 Key Metrics

```json
{
  "monitoringMetrics": {
    "networkLayer": {
      "bandwidth": {
        "inbound": "Mbps/Gbps",
        "outbound": "Mbps/Gbps",
        "alertThreshold": "> 80% capacity"
      },
      "packetRate": {
        "pps": "packets per second",
        "alertThreshold": "> 1M pps"
      },
      "protocolDistribution": {
        "tcp": "percentage",
        "udp": "percentage",
        "icmp": "percentage",
        "alertCondition": "abnormal distribution"
      }
    },
    "applicationLayer": {
      "httpRequestRate": {
        "requestsPerSecond": "integer",
        "alertThreshold": "> 3x baseline"
      },
      "errorRate": {
        "http5xx": "percentage",
        "http4xx": "percentage",
        "alertThreshold": "> 5% error rate"
      },
      "responseTime": {
        "p50": "milliseconds",
        "p95": "milliseconds",
        "p99": "milliseconds",
        "alertThreshold": "p95 > 2x baseline"
      }
    },
    "serverResources": {
      "cpuUtilization": "percentage",
      "memoryUtilization": "percentage",
      "connectionCount": "integer",
      "alertThreshold": "> 90% utilization"
    }
  }
}
```

### 6.2 Alerting Workflow

```json
{
  "alerting": {
    "severityLevels": {
      "info": "Anomaly detected, within tolerance",
      "warning": "Approaching threshold, monitor closely",
      "critical": "Threshold exceeded, mitigation recommended",
      "emergency": "Service degradation, immediate action required"
    },
    "notificationChannels": {
      "email": "security-team@example.com",
      "sms": "on-call engineer",
      "slack": "#security-incidents",
      "pagerDuty": "escalation policy",
      "webhook": "SIEM integration"
    },
    "alertConditions": {
      "bandwidthSpike": "bandwidth > 2x baseline for 60 seconds",
      "errorRateIncrease": "5xx errors > 10% for 2 minutes",
      "cpuExhaustion": "CPU > 95% for 5 minutes",
      "connectionFlood": "connections > 10000 and increasing"
    },
    "autoMitigation": {
      "enabledForCritical": true,
      "actions": [
        "Activate rate limiting",
        "Enable traffic scrubbing",
        "Notify security team",
        "Log all decisions for audit"
      ]
    }
  }
}
```

---

© 2025 World Certification Industry Association (WIA)
**弘益人間 · Benefit All Humanity**


## Annex E — Implementation Notes for PHASE-1-CORE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-CORE.

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
evidence for PHASE-1-CORE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-core/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-1-CORE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-CORE does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-CORE.
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
for PHASE-1-CORE. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P1-CORE-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
