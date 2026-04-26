# WIA-SEC-021: DDoS Protection - Phases 2, 3 & 4

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01
**Primary Color:** #8B5CF6 (Purple - Security)

---

## PHASE 2: Advanced Mitigation

### 2.1 CDN-Based Protection

Content Delivery Networks provide distributed DDoS mitigation at global scale.

#### 2.1.1 Architecture

```json
{
  "cdnProtection": {
    "architecture": {
      "edgeNodes": {
        "globalDistribution": "200+ points of presence",
        "anycastRouting": "single IP, multiple locations",
        "localFiltering": "detect and mitigate at edge",
        "capacity": "10+ Tbps aggregate"
      },
      "trafficFlow": {
        "normal": "Client → CDN Edge → Origin (cache hit) OR Origin (cache miss)",
        "underAttack": "Client → CDN Edge (filters attack) → Origin (clean traffic only)"
      }
    },
    "protectionLayers": {
      "layer3_4": {
        "volumetricAttacks": "absorbed by massive bandwidth",
        "synFlood": "SYN cookies at edge",
        "udpFlood": "rate limiting and validation"
      },
      "layer7": {
        "httpFlood": "challenge-response mechanisms",
        "botDetection": "JavaScript challenge, CAPTCHA",
        "rateLimiting": "per-IP and per-session"
      }
    },
    "cachingStrategy": {
      "staticContent": "served from edge cache",
      "dynamicContent": "shielded origin with cache headers",
      "api": "cache frequent responses with short TTL"
    },
    "originShielding": {
      "description": "Hide origin IP, only allow CDN connections",
      "implementation": [
        "Origin firewall whitelist CDN IP ranges",
        "Use CDN-specific origin hostnames",
        "TLS origin certificate validation"
      ]
    }
  }
}
```

#### 2.1.2 Configuration Example

```nginx
# Nginx configuration with CDN protection
server {
    listen 443 ssl http2;
    server_name example.com;

    # Only allow Cloudflare IPs (example)
    set_real_ip_from 103.21.244.0/22;
    set_real_ip_from 103.22.200.0/22;
    # ... (add all Cloudflare ranges)
    real_ip_header CF-Connecting-IP;

    # Rate limiting
    limit_req_zone $binary_remote_addr zone=general:10m rate=10r/s;
    limit_req zone=general burst=20 nodelay;

    # Connection limiting
    limit_conn_zone $binary_remote_addr zone=addr:10m;
    limit_conn addr 10;

    # Timeouts
    client_body_timeout 10s;
    client_header_timeout 10s;
    keepalive_timeout 5s;

    # Cache control
    location /static/ {
        expires 1y;
        add_header Cache-Control "public, immutable";
    }

    location /api/ {
        expires 60s;
        add_header Cache-Control "public, max-age=60";
        limit_req zone=api burst=5 nodelay;
    }
}
```

### 2.2 WAF Integration

Web Application Firewall provides Layer 7 protection against application-specific attacks.

```json
{
  "wafProtection": {
    "ruleCategories": {
      "httpFloodProtection": {
        "challengeUnknownVisitors": true,
        "captchaThreshold": "high request rate",
        "jsChallenge": "verify browser capability"
      },
      "botManagement": {
        "goodBots": "allow search engines, monitoring",
        "badBots": "block scrapers, attack tools",
        "fingerprinting": "TLS, HTTP/2, browser characteristics"
      },
      "owaspTop10": {
        "sqlInjection": "block malicious SQL patterns",
        "xss": "filter script injection attempts",
        "rfi": "prevent remote file inclusion"
      },
      "rateLimiting": {
        "perURL": "limit requests to expensive endpoints",
        "perAction": "limit login attempts, form submissions",
        "adaptive": "tighten limits during attack"
      }
    },
    "customRules": {
      "geoBlocking": "block traffic from unexpected countries",
      "ipReputation": "block known malicious IPs",
      "userAgentValidation": "detect suspicious user agents",
      "httpHeaderAnomalies": "detect malformed or suspicious headers"
    },
    "loggingAndAnalytics": {
      "logAllBlocked": true,
      "logSampledAllowed": "10% sampling",
      "realTimeDashboard": "view attacks in progress",
      "historicalAnalysis": "identify attack patterns"
    }
  }
}
```

### 2.3 Behavioral Analysis

Advanced threat detection using machine learning and behavioral patterns.

```python
# Behavioral Analysis System
class BehavioralAnalyzer:
    def __init__(self):
        self.models = {
            'request_pattern': IsolationForest(),
            'temporal_pattern': LSTM(),
            'navigation_pattern': MarkovChain()
        }

    def analyze_session(self, session_data):
        """
        Analyze user session for bot-like behavior
        """
        features = self.extract_features(session_data)

        scores = {
            'request_pattern_score': self.models['request_pattern'].predict(features['requests']),
            'temporal_score': self.models['temporal_pattern'].predict(features['timings']),
            'navigation_score': self.models['navigation_pattern'].predict(features['pages'])
        }

        composite_score = self.calculate_composite_score(scores)

        return {
            'is_suspicious': composite_score > 0.7,
            'confidence': composite_score,
            'reasoning': self.explain_score(scores),
            'recommended_action': self.recommend_action(composite_score)
        }

    def extract_features(self, session_data):
        return {
            'requests': {
                'rate': len(session_data['requests']) / session_data['duration'],
                'variance': np.var([r['timestamp'] for r in session_data['requests']]),
                'url_diversity': len(set(r['url'] for r in session_data['requests']))
            },
            'timings': {
                'think_time': self.calculate_think_time(session_data),
                'consistency': self.timing_consistency(session_data),
                'human_like': self.human_pattern_score(session_data)
            },
            'pages': {
                'sequence': [r['url'] for r in session_data['requests']],
                'depth': self.calculate_navigation_depth(session_data),
                'backtracking': self.detect_backtracking(session_data)
            }
        }

    def recommend_action(self, score):
        if score > 0.9:
            return 'BLOCK'
        elif score > 0.7:
            return 'CHALLENGE'
        elif score > 0.5:
            return 'RATE_LIMIT'
        else:
            return 'ALLOW'
```

---

## PHASE 3: Protocol & Integration

### 3.1 BGP Flowspec

BGP Flow Specification (RFC 5575) enables dynamic traffic filtering at network edges.

```json
{
  "bgpFlowspec": {
    "description": "Disseminate traffic filtering rules via BGP",
    "useCase": "Coordinate DDoS mitigation across ISP infrastructure",
    "flowSpecification": {
      "matchCriteria": {
        "destinationPrefix": "203.0.113.0/24",
        "sourcePrefix": "198.51.100.0/24",
        "ipProtocol": "UDP",
        "sourcePort": "53",
        "destinationPort": "0-1023",
        "packetLength": ">= 1400",
        "tcpFlags": "SYN && !ACK"
      },
      "actions": {
        "trafficRate": "0 (drop)",
        "redirect": "VRF or next-hop",
        "trafficMarking": "DSCP value",
        "trafficAction": "sample or log"
      }
    },
    "example": {
      "scenario": "Mitigate UDP flood to 203.0.113.10",
      "rule": {
        "match": {
          "destination": "203.0.113.10/32",
          "protocol": "UDP",
          "packetLength": "> 1000 bytes"
        },
        "action": "rate-limit to 1 Mbps"
      },
      "deployment": "BGP announces rule to all edge routers",
      "effect": "Automatic filtering at network edge within seconds"
    }
  }
}
```

### 3.2 SIEM Integration

Security Information and Event Management systems provide centralized logging and correlation.

```json
{
  "siemIntegration": {
    "dataSources": {
      "firewall": "connection logs, blocked connections",
      "ids_ips": "intrusion detection alerts",
      "webServer": "access logs, error logs",
      "loadBalancer": "connection metrics, health checks",
      "cdn": "edge logs, mitigation events",
      "waf": "blocked requests, attack signatures"
    },
    "eventCorrelation": {
      "crossSourceAnalysis": "correlate events from multiple systems",
      "timelineReconstruction": "build attack timeline",
      "impactAssessment": "quantify attack impact",
      "attribution": "identify attack source and type"
    },
    "alerting": {
      "realTime": "immediate notification on critical events",
      "aggregation": "group similar events to reduce noise",
      "prioritization": "rank alerts by severity and confidence",
      "escalation": "auto-escalate prolonged attacks"
    },
    "compliance": {
      "logRetention": "retain logs for 90+ days",
      "auditTrail": "tamper-proof log storage",
      "reporting": "automated compliance reports",
      "forensics": "support post-incident investigation"
    }
  }
}
```

### 3.3 API for Automation

RESTful API for programmatic DDoS protection management.

```javascript
// DDoS Protection API Specification

/**
 * Get current protection status
 */
GET /api/v1/ddos/status
Response: {
  "status": "normal" | "under_attack" | "mitigation_active",
  "currentThreatLevel": 0-100,
  "activeAttacks": [
    {
      "attackID": "ddos-20250125-001",
      "type": "UDP_FLOOD",
      "startTime": "2025-01-25T14:30:00Z",
      "volume": "85 Gbps",
      "mitigationStatus": "active"
    }
  ],
  "metrics": {
    "bandwidthIn": "12 Gbps",
    "bandwidthOut": "3 Gbps",
    "requestsPerSecond": 50000,
    "blockedRequestsPerSecond": 45000
  }
}

/**
 * Configure protection rules
 */
POST /api/v1/ddos/rules
Request: {
  "ruleName": "Block UDP from malicious ASN",
  "priority": 100,
  "match": {
    "protocol": "UDP",
    "sourceASN": "AS64512",
    "destinationPort": 53
  },
  "action": "DROP",
  "expiresAt": "2025-01-26T14:30:00Z"
}
Response: {
  "ruleID": "rule-12345",
  "status": "active",
  "appliedAt": "2025-01-25T14:35:00Z"
}

/**
 * Activate emergency mitigation
 */
POST /api/v1/ddos/mitigate
Request: {
  "mode": "aggressive",
  "duration": 3600,
  "strategies": ["SCRUBBING", "RATE_LIMIT", "CHALLENGE_RESPONSE"]
}
Response: {
  "mitigationID": "mitigation-67890",
  "status": "activating",
  "estimatedActivationTime": "45 seconds"
}

/**
 * Get attack analytics
 */
GET /api/v1/ddos/analytics?from=2025-01-20&to=2025-01-25
Response: {
  "period": {
    "start": "2025-01-20T00:00:00Z",
    "end": "2025-01-25T23:59:59Z"
  },
  "summary": {
    "totalAttacks": 12,
    "totalVolumeBlocked": "1.2 Tbps",
    "averageMitigationTime": "18 seconds",
    "peakAttackVolume": "150 Gbps"
  },
  "attackTypes": {
    "UDP_FLOOD": 5,
    "SYN_FLOOD": 3,
    "HTTP_FLOOD": 4
  },
  "topSourceCountries": ["CN", "RU", "US"],
  "topTargets": ["203.0.113.10", "203.0.113.20"]
}

/**
 * Whitelist trusted IPs
 */
POST /api/v1/ddos/whitelist
Request: {
  "ipAddress": "198.51.100.50",
  "description": "Corporate VPN",
  "expiresAt": null
}
Response: {
  "whitelistID": "wl-111",
  "status": "active"
}
```

---

## PHASE 4: Advanced Topics

### 4.1 Anycast Routing

Anycast provides geographic load distribution and automatic failover.

```json
{
  "anycastArchitecture": {
    "concept": {
      "description": "Single IP announced from multiple locations",
      "routing": "Traffic routed to nearest node via BGP",
      "benefit": "Automatic geographic distribution and failover"
    },
    "implementation": {
      "ipAddress": "203.0.113.1 (anycast)",
      "locations": [
        { "city": "New York", "asn": "AS64501", "capacity": "20 Gbps" },
        { "city": "London", "asn": "AS64502", "capacity": "20 Gbps" },
        { "city": "Singapore", "asn": "AS64503", "capacity": "20 Gbps" },
        { "city": "Sydney", "asn": "AS64504", "capacity": "20 Gbps" }
      ],
      "totalCapacity": "80 Gbps aggregate"
    },
    "ddosProtection": {
      "attackAbsorption": "Distributed across all locations",
      "localizedAttacks": "Affect only nearby region",
      "globalAttacks": "Absorbed proportionally by all nodes",
      "failover": "Traffic automatically reroutes if node overwhelmed"
    },
    "bgpConfiguration": {
      "announcement": "Each location announces same prefix",
      "asPath": "Keep AS path short for optimal routing",
      "community": "Use BGP communities for traffic engineering",
      "healthCheck": "Withdraw announcement if node unhealthy"
    }
  }
}
```

### 4.2 DPDK-Based Filtering

Data Plane Development Kit for high-performance packet processing.

```c
// DPDK-based DDoS filtering (simplified example)
#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_mbuf.h>

#define BURST_SIZE 32

// Packet processing function
static void process_packets(struct rte_mbuf **pkts, uint16_t nb_pkts) {
    for (uint16_t i = 0; i < nb_pkts; i++) {
        struct rte_mbuf *pkt = pkts[i];

        // Parse packet headers
        struct rte_ether_hdr *eth = rte_pktmbuf_mtod(pkt, struct rte_ether_hdr *);
        struct rte_ipv4_hdr *ip = (struct rte_ipv4_hdr *)(eth + 1);

        // DDoS filtering logic
        if (is_ddos_packet(ip)) {
            // Drop packet
            rte_pktmbuf_free(pkt);
            continue;
        }

        // Rate limiting check
        uint32_t src_ip = rte_be_to_cpu_32(ip->src_addr);
        if (!check_rate_limit(src_ip)) {
            rte_pktmbuf_free(pkt);
            continue;
        }

        // Forward clean packet
        forward_packet(pkt);
    }
}

// Main packet processing loop
static void packet_processing_loop(void) {
    struct rte_mbuf *pkts_burst[BURST_SIZE];
    uint16_t nb_rx;

    while (1) {
        // Receive burst of packets
        nb_rx = rte_eth_rx_burst(0, 0, pkts_burst, BURST_SIZE);

        if (nb_rx == 0)
            continue;

        // Process packets (millions of packets per second)
        process_packets(pkts_burst, nb_rx);
    }
}

// Performance: 10+ million packets per second on modern hardware
```

### 4.3 Quantum-Resistant Cryptography

Future-proof DDoS mitigation signatures against quantum computing threats.

```json
{
  "quantumResistantDDoS": {
    "rationale": {
      "currentThreat": "Classical crypto sufficient for DDoS today",
      "futureThreat": "Quantum computers may forge mitigation messages",
      "timeline": "Prepare now for post-quantum era"
    },
    "applications": {
      "bgpFlowspec": {
        "current": "RSA/ECDSA signed BGP messages",
        "quantum": "CRYSTALS-Dilithium signatures",
        "benefit": "Prevent forged mitigation rules"
      },
      "apiAuthentication": {
        "current": "TLS 1.3 with ECDHE",
        "quantum": "Kyber KEM for key exchange",
        "benefit": "Secure API even against quantum attacks"
      },
      "logIntegrity": {
        "current": "SHA-256 hashes",
        "quantum": "SHA-3 or SPHINCS+ signatures",
        "benefit": "Tamper-proof audit logs"
      }
    },
    "implementation": {
      "library": "liboqs (Open Quantum Safe)",
      "algorithms": {
        "keyExchange": "Kyber-768",
        "signatures": "Dilithium3",
        "hashing": "SHA3-256"
      },
      "migration": {
        "hybrid": "Use both classical and quantum-resistant crypto",
        "gradual": "Deploy over 2-5 years",
        "testing": "Validate performance impact"
      }
    }
  }
}
```

### 4.4 AI-Driven Prediction

Predictive analytics to anticipate DDoS attacks before they occur.

```python
# AI-Driven DDoS Prediction System
import numpy as np
import tensorflow as tf
from sklearn.ensemble import RandomForest

class DDoSPredictor:
    def __init__(self):
        self.lstm_model = self.build_lstm_model()
        self.rf_classifier = RandomForest(n_estimators=100)

    def build_lstm_model(self):
        """
        LSTM model for time-series traffic prediction
        """
        model = tf.keras.Sequential([
            tf.keras.layers.LSTM(128, return_sequences=True, input_shape=(24, 10)),
            tf.keras.layers.Dropout(0.2),
            tf.keras.layers.LSTM(64, return_sequences=False),
            tf.keras.layers.Dropout(0.2),
            tf.keras.layers.Dense(32, activation='relu'),
            tf.keras.layers.Dense(1, activation='sigmoid')
        ])
        model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])
        return model

    def predict_attack_probability(self, historical_metrics):
        """
        Predict likelihood of DDoS attack in next hour

        Args:
            historical_metrics: 24 hours of traffic data

        Returns:
            {
                'probability': 0.0-1.0,
                'confidence': 0.0-1.0,
                'expectedTime': ISO timestamp,
                'expectedType': attack type,
                'recommendation': action to take
            }
        """
        # Time-series prediction
        X_lstm = self.prepare_lstm_input(historical_metrics)
        attack_probability = self.lstm_model.predict(X_lstm)[0][0]

        # Feature-based classification
        X_rf = self.extract_features(historical_metrics)
        attack_type = self.rf_classifier.predict(X_rf)

        # Calculate confidence
        lstm_confidence = 1 - abs(attack_probability - 0.5) * 2
        rf_confidence = max(self.rf_classifier.predict_proba(X_rf)[0])
        confidence = (lstm_confidence + rf_confidence) / 2

        return {
            'probability': float(attack_probability),
            'confidence': float(confidence),
            'expectedTime': self.estimate_attack_time(historical_metrics),
            'expectedType': attack_type[0] if attack_probability > 0.5 else None,
            'recommendation': self.recommend_action(attack_probability, confidence)
        }

    def extract_features(self, metrics):
        """
        Extract predictive features from traffic metrics
        """
        return {
            'traffic_trend': self.calculate_trend(metrics['bandwidth']),
            'source_ip_growth': self.calculate_growth_rate(metrics['unique_ips']),
            'port_scan_activity': self.detect_port_scans(metrics['connections']),
            'geographic_anomaly': self.geographic_entropy(metrics['geo_distribution']),
            'protocol_distribution_change': self.protocol_shift(metrics['protocols']),
            'time_of_day': metrics['timestamp'].hour,
            'day_of_week': metrics['timestamp'].weekday(),
            'historical_attack_correlation': self.correlate_with_past_attacks(metrics)
        }

    def recommend_action(self, probability, confidence):
        """
        Recommend preemptive action based on prediction
        """
        if probability > 0.8 and confidence > 0.7:
            return {
                'action': 'PREEMPTIVE_MITIGATION',
                'steps': [
                    'Enable traffic scrubbing',
                    'Tighten rate limits',
                    'Alert security team',
                    'Prepare incident response'
                ]
            }
        elif probability > 0.6:
            return {
                'action': 'HEIGHTENED_MONITORING',
                'steps': [
                    'Increase monitoring frequency',
                    'Standby mitigation systems',
                    'Notify on-call engineer'
                ]
            }
        else:
            return {
                'action': 'NORMAL_OPERATIONS',
                'steps': ['Continue standard monitoring']
            }
```

### 4.5 Blockchain-Based Attack Attribution

Decentralized system for tracking and attributing DDoS attacks.

```json
{
  "blockchainAttribution": {
    "concept": {
      "description": "Immutable ledger of DDoS attack metadata",
      "purpose": "Share threat intelligence across organizations",
      "benefit": "Coordinated defense and attribution"
    },
    "dataStructure": {
      "attackRecord": {
        "attackID": "hash of attack metadata",
        "timestamp": "ISO 8601",
        "victimDID": "did:wia:victim-org",
        "reporterDID": "did:wia:reporter-org",
        "attackType": "UDP_FLOOD | SYN_FLOOD | HTTP_FLOOD",
        "metadata": {
          "volume": "150 Gbps",
          "duration": "45 minutes",
          "sourceASNs": ["AS64512", "AS64513"],
          "topSourceIPs": "hashed for privacy",
          "attackSignature": "hash of packet patterns"
        },
        "proof": {
          "type": "Ed25519Signature2020",
          "proofValue": "digital signature",
          "verificationMethod": "did:wia:reporter-org#key-1"
        }
      }
    },
    "smartContract": {
      "functions": {
        "submitAttackReport": "Add new attack record",
        "queryAttackHistory": "Search for similar attacks",
        "reputationScore": "Calculate IP/ASN reputation",
        "coordinatedMitigation": "Trigger multi-org response"
      }
    },
    "privacy": {
      "victimAnonymity": "Optional anonymous reporting",
      "dataMinimization": "Only share necessary metadata",
      "aggregation": "Statistical summaries, not raw data"
    },
    "useCase": {
      "scenario": "ISP detects large DDoS attack",
      "action": "Records attack metadata on blockchain",
      "benefit": "Other ISPs see pattern, preemptively block sources",
      "outcome": "Reduced global attack effectiveness"
    }
  }
}
```

### 4.6 5G Network Slicing for DDoS Isolation

Utilize 5G network slicing to isolate attack traffic.

```json
{
  "5gNetworkSlicing": {
    "concept": {
      "description": "Dedicated virtual networks for different traffic types",
      "advantage": "Isolate attack traffic from critical services",
      "technology": "5G core network (5GC) slicing"
    },
    "architecture": {
      "slices": [
        {
          "name": "critical-services",
          "sla": {
            "latency": "< 10ms",
            "reliability": "99.999%",
            "bandwidth": "dedicated 10 Gbps"
          },
          "ddosProtection": "isolated from other slices"
        },
        {
          "name": "public-internet",
          "sla": {
            "latency": "< 50ms",
            "reliability": "99.9%",
            "bandwidth": "shared"
          },
          "ddosProtection": "absorb attacks without affecting critical slice"
        },
        {
          "name": "iot-devices",
          "sla": {
            "latency": "< 100ms",
            "reliability": "99.5%",
            "bandwidth": "low, many connections"
          },
          "ddosProtection": "rate limiting per device"
        }
      ]
    },
    "ddosScenario": {
      "attack": "Massive IoT botnet DDoS attack",
      "withoutSlicing": "All services degraded",
      "withSlicing": "IoT slice saturated, critical services unaffected",
      "benefit": "Business continuity for critical functions"
    },
    "implementation": {
      "3gppStandards": "TS 23.501, TS 23.502",
      "orchestration": "Network Slice Selection Function (NSSF)",
      "isolation": "QoS enforcement at UPF (User Plane Function)"
    }
  }
}
```

---

## Summary

This specification has covered:

**Phase 2 - Advanced Mitigation**:
- CDN-based protection
- WAF integration
- Behavioral analysis

**Phase 3 - Protocol & Integration**:
- BGP Flowspec
- SIEM integration
- Automation APIs

**Phase 4 - Advanced Topics**:
- Anycast routing
- DPDK high-performance filtering
- Quantum-resistant cryptography
- AI-driven prediction
- Blockchain attribution
- 5G network slicing

---

© 2025 World Certification Industry Association (WIA)
**弘益人間 · Benefit All Humanity**


## Annex E — Implementation Notes for PHASE-2-&-3-&-4

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-&-3-&-4.

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
evidence for PHASE-2-&-3-&-4. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-&-3-&-4/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-2-&-3-&-4 with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-&-3-&-4 does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-&-3-&-4.
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
for PHASE-2-&-3-&-4. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P2-&-3-&-4-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
