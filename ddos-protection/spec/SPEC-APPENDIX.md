# WIA-SEC-021: DDoS Protection - Appendix

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01
**Primary Color:** #8B5CF6 (Purple - Security)

---

## Appendix A: Real-World Case Studies

### A.1 GitHub 2018 Memcached DDoS Attack

**Date**: February 28, 2018
**Target**: GitHub.com
**Attack Type**: Memcached amplification (UDP reflection)
**Peak Volume**: 1.35 Tbps

**Attack Details**:
```json
{
  "attackID": "github-2018-memcached",
  "timeline": {
    "00:00": "Attack begins with 40 Gbps",
    "00:10": "Ramps to 1.35 Tbps (peak)",
    "00:20": "Traffic subsides to 400 Gbps",
    "00:30": "Attack ends"
  },
  "attackVector": {
    "type": "MEMCACHED_AMPLIFICATION",
    "exploitedServers": "~50,000 memcached servers",
    "amplificationFactor": "51,000x (15 byte request → 750 KB response)",
    "protocol": "UDP port 11211"
  },
  "victimResponse": {
    "mitigationProvider": "Akamai Prolexic",
    "activationTime": "< 10 minutes",
    "strategy": "Traffic scrubbing at edge",
    "outcome": "Service restored, minimal downtime"
  },
  "lessonsLearned": {
    "vulnerability": "Exposed memcached servers without authentication",
    "fix": "Disable UDP or firewall memcached ports",
    "industryImpact": "Mass patching of vulnerable servers",
    "preventionGuidance": "Never expose caching servers to public Internet"
  }
}
```

**Mitigation Breakdown**:
1. **Detection** (< 1 minute): Monitoring systems detected abnormal traffic spike
2. **Analysis** (2 minutes): Identified memcached amplification signature
3. **Activation** (3 minutes): Activated Akamai scrubbing centers globally
4. **Scrubbing** (ongoing): Filtered malicious UDP traffic, passed legitimate HTTPS
5. **Resolution** (20 minutes): Attack traffic subsided, normal operations resumed

**Technical Insights**:
```python
# Memcached amplification attack structure
attack_packet = {
    "source_ip": "203.0.113.10",  # Spoofed to victim's IP
    "destination_ip": "198.51.100.50",  # Open memcached server
    "protocol": "UDP",
    "destination_port": 11211,
    "payload": b"\x00\x00\x00\x00\x00\x01\x00\x00stats\r\n",  # 15 bytes
    "response_size": "750 KB"  # 51,000x amplification
}

# Prevention measure
iptables_rule = "iptables -A INPUT -p udp --dport 11211 -j DROP"
```

---

### A.2 Dyn DNS 2016 Mirai Botnet Attack

**Date**: October 21, 2016
**Target**: Dyn DNS infrastructure
**Attack Type**: IoT botnet (Mirai)
**Impact**: Major websites (Twitter, Netflix, Reddit) inaccessible

**Attack Profile**:
```json
{
  "attackID": "dyn-2016-mirai",
  "timeline": {
    "07:00 EDT": "First wave begins",
    "09:20 EDT": "Service partially restored",
    "11:52 EDT": "Second wave begins",
    "13:00 EDT": "Partial mitigation",
    "15:52 EDT": "Third wave begins",
    "17:00 EDT": "Full restoration"
  },
  "attackSource": {
    "botnet": "Mirai",
    "compromisedDevices": "~100,000 IoT devices",
    "deviceTypes": [
      "IP cameras",
      "DVRs",
      "Home routers",
      "Baby monitors",
      "Smart thermostats"
    ],
    "exploitMethod": "Default credentials (admin/admin, root/root)"
  },
  "attackVector": {
    "primaryType": "DNS_QUERY_FLOOD",
    "secondaryType": "TCP_SYN_FLOOD",
    "targetedDomains": "Dyn's managed DNS servers",
    "queryRate": "Tens of millions of queries per second"
  },
  "cascadingImpact": {
    "affectedServices": [
      "Twitter",
      "Netflix",
      "Reddit",
      "GitHub",
      "Spotify",
      "PayPal",
      "PlayStation Network"
    ],
    "reason": "These services used Dyn for DNS resolution",
    "userExperience": "Unable to resolve domain names → service unavailable"
  },
  "lessonsLearned": {
    "singlePointOfFailure": "Reliance on single DNS provider risky",
    "iotSecurity": "Default credentials on IoT devices are critical vulnerability",
    "industryChanges": [
      "Multi-provider DNS strategies",
      "IoT security standards",
      "Better default security on consumer devices"
    ]
  }
}
```

---

## Appendix B: DDoS Protection Checklist

### B.1 Network Layer Protection

- [ ] **Bandwidth over-provisioning**: Provision 2-10x normal traffic capacity
- [ ] **Multiple ISP connections**: Diverse network paths
- [ ] **BGP configuration**:
  - [ ] Configure ROA (Route Origin Authorization) for prefix validation
  - [ ] Implement BGP Flowspec for dynamic filtering (RFC 5575)
  - [ ] Enable RTBH (Remotely Triggered Black Hole) filtering
- [ ] **DDoS mitigation service**: Contract with Cloudflare, Akamai, or AWS Shield
- [ ] **Anycast implementation**: Deploy services across multiple geographic locations
- [ ] **Network monitoring**:
  - [ ] NetFlow/sFlow collection
  - [ ] Real-time bandwidth monitoring
  - [ ] Anomaly detection alerts

### B.2 Transport Layer Protection

- [ ] **SYN flood protection**:
  - [ ] Enable SYN cookies on all servers
  - [ ] Configure aggressive TCP timeouts
  - [ ] Limit half-open connections
- [ ] **Connection rate limiting**:
  - [ ] Per-IP connection limits
  - [ ] Global connection pool limits
  - [ ] Connection timeout policies
- [ ] **Firewall rules**:
  - [ ] Drop fragmented packets (unless required)
  - [ ] Block spoofed source IPs (BCP 38)
  - [ ] Geo-blocking for unexpected countries

### B.3 Application Layer Protection

- [ ] **Web Application Firewall (WAF)**:
  - [ ] Deploy WAF (ModSecurity, Cloudflare WAF, AWS WAF)
  - [ ] Configure OWASP Core Rule Set
  - [ ] Custom rules for application-specific threats
- [ ] **Rate limiting**:
  - [ ] Per-IP rate limits on API endpoints
  - [ ] Per-session rate limits
  - [ ] Progressive rate limiting (stricter during attacks)
- [ ] **Challenge-response mechanisms**:
  - [ ] JavaScript challenge for suspected bots
  - [ ] CAPTCHA for high-risk requests
  - [ ] Cookie validation
- [ ] **CDN configuration**:
  - [ ] Aggressive caching of static content
  - [ ] Origin IP hiding (only allow CDN IPs)
  - [ ] DDoS protection features enabled
- [ ] **Application hardening**:
  - [ ] Optimize database queries
  - [ ] Implement request timeouts
  - [ ] Resource usage limits per request
  - [ ] Graceful degradation under load

### B.4 Monitoring & Response

- [ ] **Real-time monitoring**:
  - [ ] Bandwidth and packet rate dashboards
  - [ ] Error rate monitoring (4xx, 5xx)
  - [ ] Latency monitoring (p50, p95, p99)
  - [ ] Connection count and state tracking
- [ ] **Alerting**:
  - [ ] Configure alerts for anomaly thresholds
  - [ ] Multiple notification channels (email, SMS, Slack)
  - [ ] Escalation policies for critical incidents
- [ ] **Incident response plan**:
  - [ ] Document mitigation procedures
  - [ ] Define roles and responsibilities
  - [ ] Establish communication channels
  - [ ] Maintain vendor contact information
- [ ] **Post-incident analysis**:
  - [ ] Log collection and retention
  - [ ] Attack pattern analysis
  - [ ] Mitigation effectiveness review
  - [ ] Update defenses based on learnings

### B.5 Organizational Preparedness

- [ ] **DDoS mitigation contract**: Ensure service agreement is in place
- [ ] **Runbook**: Document step-by-step response procedures
- [ ] **Contact list**: Maintain updated list of key stakeholders
- [ ] **Regular drills**: Conduct quarterly DDoS response simulations
- [ ] **Insurance**: Consider cyber insurance covering DDoS incidents
- [ ] **Legal preparedness**: Understand law enforcement reporting requirements

---

## Appendix C: Command Reference

### C.1 Linux Server Hardening

```bash
# Enable SYN cookies
sysctl -w net.ipv4.tcp_syncookies=1
echo "net.ipv4.tcp_syncookies=1" >> /etc/sysctl.conf

# Increase SYN backlog
sysctl -w net.ipv4.tcp_max_syn_backlog=4096
echo "net.ipv4.tcp_max_syn_backlog=4096" >> /etc/sysctl.conf

# Reduce SYN-ACK retries
sysctl -w net.ipv4.tcp_synack_retries=2
echo "net.ipv4.tcp_synack_retries=2" >> /etc/sysctl.conf

# Enable reverse path filtering (anti-spoofing)
sysctl -w net.ipv4.conf.all.rp_filter=1
echo "net.ipv4.conf.all.rp_filter=1" >> /etc/sysctl.conf

# Ignore ICMP echo requests (optional, blocks ping)
sysctl -w net.ipv4.icmp_echo_ignore_all=1

# Limit ICMP rate
iptables -A INPUT -p icmp --icmp-type echo-request -m limit --limit 1/s --limit-burst 5 -j ACCEPT
iptables -A INPUT -p icmp --icmp-type echo-request -j DROP

# Connection tracking limits
sysctl -w net.netfilter.nf_conntrack_max=1000000
echo "net.netfilter.nf_conntrack_max=1000000" >> /etc/sysctl.conf

# Apply all sysctl changes
sysctl -p
```

### C.2 Iptables DDoS Protection Rules

```bash
# Create new chain for DDoS protection
iptables -N DDOS_PROTECT

# SYN flood protection
iptables -A DDOS_PROTECT -p tcp --syn -m limit --limit 10/s --limit-burst 20 -j RETURN
iptables -A DDOS_PROTECT -p tcp --syn -j DROP

# ICMP flood protection
iptables -A DDOS_PROTECT -p icmp -m limit --limit 1/s --limit-burst 5 -j RETURN
iptables -A DDOS_PROTECT -p icmp -j DROP

# UDP flood protection
iptables -A DDOS_PROTECT -p udp -m limit --limit 50/s --limit-burst 100 -j RETURN
iptables -A DDOS_PROTECT -p udp -j DROP

# Apply DDoS protection chain
iptables -A INPUT -j DDOS_PROTECT

# Per-IP connection limiting (max 50 concurrent connections)
iptables -A INPUT -p tcp --syn -m connlimit --connlimit-above 50 -j REJECT --reject-with tcp-reset

# Block invalid packets
iptables -A INPUT -m state --state INVALID -j DROP

# Save rules
iptables-save > /etc/iptables/rules.v4
```

### C.3 Nginx Rate Limiting

```nginx
# Define rate limit zones
http {
    # IP-based rate limiting
    limit_req_zone $binary_remote_addr zone=general:10m rate=10r/s;
    limit_req_zone $binary_remote_addr zone=api:10m rate=5r/s;
    limit_req_zone $binary_remote_addr zone=login:10m rate=1r/s;

    # Connection limiting
    limit_conn_zone $binary_remote_addr zone=addr:10m;

    server {
        listen 80;
        server_name example.com;

        # Apply connection limit (max 10 concurrent connections per IP)
        limit_conn addr 10;

        # General rate limiting
        location / {
            limit_req zone=general burst=20 nodelay;
            proxy_pass http://backend;
        }

        # API endpoint with stricter limits
        location /api/ {
            limit_req zone=api burst=10 nodelay;
            proxy_pass http://api_backend;
        }

        # Login endpoint with very strict limits
        location /login {
            limit_req zone=login burst=3 nodelay;
            proxy_pass http://auth_backend;
        }

        # Timeout settings
        client_body_timeout 10s;
        client_header_timeout 10s;
        keepalive_timeout 5s;
        send_timeout 10s;

        # Buffer size limits
        client_body_buffer_size 1K;
        client_header_buffer_size 1k;
        client_max_body_size 1k;
        large_client_header_buffers 2 1k;
    }
}
```

### C.4 Apache mod_evasive Configuration

```apache
# Install: apt-get install libapache2-mod-evasive
# Configuration: /etc/apache2/mods-available/evasive.conf

<IfModule mod_evasive20.c>
    # Max requests per page per second
    DOSPageCount 5

    # Max requests per site per second
    DOSSiteCount 100

    # Interval for page count threshold (seconds)
    DOSPageInterval 1

    # Interval for site count threshold (seconds)
    DOSSiteInterval 1

    # Blocking period (seconds)
    DOSBlockingPeriod 60

    # Email alert
    DOSEmailNotify security@example.com

    # Log directory
    DOSLogDir /var/log/mod_evasive

    # Whitelist trusted IPs
    DOSWhitelist 127.0.0.1
    DOSWhitelist 192.168.1.0/24
</IfModule>
```

---

## Appendix D: Vendor Comparison Matrix

### D.1 DDoS Mitigation Providers

| Provider | Type | Max Capacity | L3/L4 | L7 | Anycast | Price Range | Best For |
|----------|------|--------------|-------|----|---------| ------------|----------|
| **Cloudflare** | CDN + DDoS | Unlimited | ✓ | ✓ | ✓ | Free - $5K+/mo | Web apps, global reach |
| **Akamai** | CDN + DDoS | 15+ Tbps | ✓ | ✓ | ✓ | $$$$ | Enterprise, high stakes |
| **AWS Shield** | Cloud-native | 100+ Tbps | ✓ | ✓ (Advanced) | ✓ | Free - $3K/mo | AWS workloads |
| **Azure DDoS** | Cloud-native | 60+ Tbps | ✓ | ✓ | ✓ | $3K/mo | Azure workloads |
| **Google Cloud Armor** | Cloud-native | 1+ Tbps | ✓ | ✓ | ✓ | Usage-based | GCP workloads |
| **Imperva** | WAF + DDoS | 3+ Tbps | ✓ | ✓ | ✓ | $$$ | Application security |
| **Radware** | On-prem + Cloud | 1+ Tbps | ✓ | ✓ | ✓ | $$$$ | Hybrid deployments |
| **Arbor Networks** | ISP-grade | 10+ Tbps | ✓ | Limited | ✓ | $$$$ | Telecom, ISPs |

### D.2 Feature Comparison

```json
{
  "features": {
    "cloudflare": {
      "protection": {
        "layer3_4": "Unlimited capacity, anycast network",
        "layer7": "WAF, rate limiting, bot management, CAPTCHA",
        "apiProtection": "API Shield with schema validation",
        "botManagement": "Advanced bot detection & mitigation"
      },
      "deployment": "DNS change, instant activation",
      "reporting": "Real-time dashboard, analytics",
      "sla": "100% uptime SLA (Enterprise)",
      "globalPoPs": "300+",
      "pricing": {
        "free": "Unlimited L3/L4, basic L7",
        "pro": "$20/mo, enhanced L7",
        "business": "$200/mo, advanced features",
        "enterprise": "Custom, dedicated support"
      }
    },
    "awsShield": {
      "protection": {
        "standard": "Free L3/L4 for all AWS customers",
        "advanced": "L3/L4/L7, cost protection, DRT support",
        "integration": "CloudFront, Route 53, ALB, ELB"
      },
      "deployment": "Automatic for AWS resources",
      "reporting": "CloudWatch metrics, Shield console",
      "sla": "99.99% (Shield Advanced)",
      "costProtection": "Refund for DDoS-related scaling costs",
      "pricing": {
        "standard": "Free",
        "advanced": "$3,000/mo + data transfer"
      }
    }
  }
}
```

---

## Appendix E: Testing & Validation

### E.1 DDoS Simulation Tools

**WARNING**: Only test against your own infrastructure with proper authorization.

```bash
# hping3 - SYN flood simulation
hping3 -S --flood -V -p 80 TARGET_IP

# slowhttptest - Slowloris simulation
slowhttptest -c 1000 -H -g -o slowloris_report -i 10 -r 200 -t GET -u http://TARGET -x 24 -p 3

# LOIC (Low Orbit Ion Cannon) - HTTP flood
# GUI-based tool, use responsibly in controlled environments only

# Apache Bench - Legitimate load testing
ab -n 100000 -c 1000 http://TARGET/

# wrk - Modern HTTP benchmarking
wrk -t12 -c400 -d30s http://TARGET/
```

### E.2 Mitigation Validation

```bash
# Test rate limiting
for i in {1..100}; do
  curl -I http://TARGET/
done
# Expected: HTTP 429 Too Many Requests after threshold

# Test SYN cookies
hping3 -S --flood -p 80 TARGET_IP
# Expected: Server remains responsive, connections accepted

# Test connection limits
for i in {1..100}; do
  curl http://TARGET/ &
done
wait
# Expected: Some connections rejected if limit exceeded

# Verify firewall rules
iptables -L -v -n | grep DROP
# Expected: Drop rules present and showing packet counts

# Check current connections
netstat -an | grep :80 | wc -l
ss -s  # Summary of socket statistics
```

### E.3 Monitoring Validation

```bash
# Generate test traffic
ab -n 10000 -c 100 http://TARGET/

# Verify monitoring captures the traffic
# Check your monitoring dashboard for:
# - Traffic spike in bandwidth graph
# - Increased request rate
# - Connection count increase

# Verify alerting
# Trigger alert threshold
# Confirm alert notification received via configured channels
```

---

## Appendix F: Compliance & Legal

### F.1 Regulatory Considerations

```json
{
  "compliance": {
    "pciDss": {
      "requirement": "PCI DSS 6.6 - Web application firewall",
      "ddosRelevance": "WAF protects payment applications from L7 DDoS",
      "implementation": "Deploy WAF with DDoS mitigation capability"
    },
    "gdpr": {
      "relevance": "Service availability = data protection",
      "requirement": "Article 32 - Security of processing",
      "ddosConnection": "DDoS protection ensures data availability",
      "implementation": "Document DDoS mitigation as security measure"
    },
    "iso27001": {
      "control": "A.17.2 - Redundancies",
      "requirement": "Ensure availability of information processing facilities",
      "ddosRelevance": "DDoS mitigation = availability control",
      "implementation": "Include DDoS in ISMS risk assessment"
    },
    "soc2": {
      "trustPrinciple": "Availability",
      "requirement": "System available for operation and use",
      "ddosRelevance": "DDoS protection demonstrates availability commitment",
      "evidence": "DDoS mitigation logs, incident response procedures"
    }
  }
}
```

### F.2 Legal Response to Attacks

```json
{
  "legalConsiderations": {
    "lawEnforcement": {
      "when": "Major attacks, suspected nation-state actors",
      "agencies": ["FBI (IC3)", "Secret Service", "Local cyber crime unit"],
      "evidence": "Preserve logs, packet captures, attack metadata",
      "timeline": "Report within 24-72 hours of detection"
    },
    "isp": {
      "notification": "Inform ISP of attack for upstream mitigation",
      "cooperation": "Work with ISP NOC for traffic diversion",
      "documentation": "Maintain communication records"
    },
    "customers": {
      "transparency": "Notify customers of service disruption",
      "statusPage": "Update public status page",
      "sla": "Honor SLA credits if applicable"
    },
    "attribution": {
      "difficulty": "Spoofed IPs make attribution challenging",
      "forensics": "Engage cyber forensics firm for major incidents",
      "legal action": "Rarely successful due to jurisdiction issues"
    }
  }
}
```

---

© 2025 World Certification Industry Association (WIA)
**弘益人間 · Benefit All Humanity**
