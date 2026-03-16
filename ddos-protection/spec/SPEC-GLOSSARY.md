# WIA-SEC-021: DDoS Protection - Glossary

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01
**Primary Color:** #8B5CF6 (Purple - Security)

---

## A

**ACK (Acknowledge)**
- TCP flag indicating receipt of data
- Used in TCP three-way handshake and data transfer
- ACK flood: Attack using malicious ACK packets

**Amplification Attack**
- DDoS technique exploiting services that send large responses to small requests
- Common vectors: DNS, NTP, memcached, SSDP
- Amplification factor: Ratio of response size to request size (e.g., 70x for DNS)

**Anycast**
- Routing method where single IP address is announced from multiple locations
- Traffic routed to nearest node based on BGP routing
- Provides geographic distribution and automatic failover

**API (Application Programming Interface)**
- Interface for programmatic access to services
- DDoS mitigation APIs allow automation of protection measures
- Often protected with rate limiting and authentication

**ASN (Autonomous System Number)**
- Unique identifier for network operators in BGP routing
- Used for tracking attack sources by network origin
- Example: AS15169 (Google), AS16509 (Amazon)

**Asymmetric Traffic**
- Traffic pattern where inbound and outbound volumes differ significantly
- Common in DDoS attacks (high inbound, low outbound)
- Normal for some services (video streaming: high outbound)

---

## B

**Bandwidth**
- Maximum data transfer rate of network connection
- Measured in bits per second (bps), often Gbps or Tbps
- Volumetric attacks aim to saturate bandwidth

**BCP 38 (Best Current Practice 38)**
- RFC 2827: Network Ingress Filtering
- Prevents IP address spoofing by validating source addresses
- Should be implemented by all ISPs

**BGP (Border Gateway Protocol)**
- Routing protocol for Internet backbone
- Used for traffic diversion to scrubbing centers
- BGP Flowspec (RFC 5575): Distribute filtering rules via BGP

**Black Hole Routing**
- Routing technique that drops traffic to null interface
- Used as last resort when infrastructure at risk
- RTBH (Remotely Triggered Black Hole): Coordinated with upstream ISPs

**Botnet**
- Network of compromised devices controlled by attacker
- Used to generate distributed attack traffic
- Examples: Mirai (IoT devices), Emotet, TrickBot

**Burst**
- Short-term traffic spike above sustained rate
- Rate limiting often allows controlled bursts
- Example: Token bucket algorithm with burst capacity

---

## C

**CAPTCHA (Completely Automated Public Turing test to tell Computers and Humans Apart)**
- Challenge-response test to distinguish humans from bots
- Used during suspected DDoS attacks to verify legitimate users
- Modern variants: reCAPTCHA, hCaptcha

**CDN (Content Delivery Network)**
- Distributed network of servers delivering content from edge locations
- Provides DDoS protection through geographic distribution
- Examples: Cloudflare, Akamai, Fastly, AWS CloudFront

**Challenge-Response**
- Authentication mechanism requiring client to solve puzzle
- JavaScript challenge: Verify browser can execute JavaScript
- Computational challenge: Proof-of-work to slow down bots

**Connection Timeout**
- Maximum time allowed for idle or incomplete connections
- Aggressive timeouts mitigate Slowloris and slow HTTP attacks
- Balance: Too short breaks legitimate slow clients

**Connection Tracking**
- Firewall feature monitoring state of TCP connections
- Enables stateful packet filtering
- Can be exhausted by SYN floods (half-open connections)

---

## D

**DDoS (Distributed Denial of Service)**
- Cyber attack making service unavailable by overwhelming with traffic
- "Distributed": Attack originates from many sources simultaneously
- Differs from DoS (Denial of Service) which uses single source

**DNS (Domain Name System)**
- System translating domain names to IP addresses
- DNS amplification: Exploits open resolvers for traffic amplification
- DNS flood: Overwhelming DNS servers with queries

**DPDK (Data Plane Development Kit)**
- Framework for high-performance packet processing
- Bypasses kernel for faster packet handling
- Capable of processing millions of packets per second

**DSCP (Differentiated Services Code Point)**
- Field in IP header for packet classification
- Used for Quality of Service (QoS)
- Can be used to mark attack traffic for filtering

---

## E

**Edge Server**
- Server at network edge, close to end users
- CDN edge servers cache content and filter traffic
- First line of defense in DDoS attacks

**Entropy**
- Measure of randomness or diversity in dataset
- High source IP entropy may indicate spoofed addresses
- Used in anomaly detection algorithms

**Error Rate**
- Percentage of failed requests (HTTP 4xx, 5xx errors)
- Spike in error rate may indicate DDoS attack
- Monitor p95, p99 percentiles for early detection

---

## F

**Failover**
- Automatic switching to backup system when primary fails
- Critical for maintaining availability during DDoS
- Anycast provides automatic geographic failover

**False Positive**
- Legitimate traffic incorrectly identified as attack
- Challenge in DDoS mitigation: Block attacks without blocking users
- Behavioral analysis reduces false positives

**FAR (False Accept Rate)**
- Rate at which attack traffic passes through defenses
- Lower is better for security
- Trade-off with FRR (False Reject Rate)

**Firewall**
- Network security system filtering traffic based on rules
- Stateful firewall: Tracks connection state
- Application firewall (WAF): Protects web applications

**Flowspec (Flow Specification)**
- BGP extension for distributing traffic filtering rules (RFC 5575)
- Enables dynamic, network-wide DDoS mitigation
- Supports complex match criteria and actions

**FRR (False Reject Rate)**
- Rate at which legitimate traffic is blocked
- Lower is better for user experience
- Trade-off with FAR (False Accept Rate)

---

## G

**Gbps (Gigabits per second)**
- Measure of data transfer rate
- 1 Gbps = 1,000,000,000 bits per second
- Modern DDoS attacks often exceed 100 Gbps

**Geo-blocking**
- Filtering traffic based on geographic origin
- Used when attacks originate from specific countries
- Implement via IP geolocation databases

**GSLB (Global Server Load Balancing)**
- DNS-based load balancing across multiple data centers
- Provides geographic distribution and failover
- Helps absorb DDoS attacks across locations

**GRE (Generic Routing Encapsulation)**
- Tunneling protocol for encapsulating packets
- Used to forward clean traffic from scrubbing centers to origin
- Alternative: IP-in-IP tunneling

---

## H

**Half-Open Connection**
- TCP connection in SYN-RECEIVED state (SYN sent, no ACK received)
- SYN flood attacks create many half-open connections
- Exhausts server connection table

**Health Check**
- Automated monitoring of server/service availability
- Used by load balancers to detect failures
- Should continue during DDoS to identify degradation

**HTTP Flood**
- Layer 7 DDoS attack using legitimate-looking HTTP requests
- Difficult to distinguish from normal traffic
- Targets resource-intensive pages (search, API, database queries)

---

## I

**ICMP (Internet Control Message Protocol)**
- Protocol for network diagnostics and error reporting
- ICMP Echo Request/Reply: Ping
- ICMP flood: DDoS using ping packets

**IDS/IPS (Intrusion Detection/Prevention System)**
- Security system monitoring network traffic for threats
- IDS: Detects and alerts
- IPS: Detects and blocks
- Can detect DDoS attack signatures

**IP Reputation**
- Score indicating trustworthiness of IP address
- Based on historical behavior (spam, attacks, etc.)
- Used to block traffic from known bad IPs

**ISP (Internet Service Provider)**
- Organization providing Internet access
- Can assist with upstream DDoS mitigation
- Should implement BCP 38 to prevent spoofing

---

## J

**JavaScript Challenge**
- Web-based challenge requiring JavaScript execution
- Verifies client is a real browser, not simple HTTP bot
- Transparent to users with JavaScript enabled

---

## K

**Kernel Bypass**
- Technique to process packets without kernel involvement
- Improves performance for high-volume packet processing
- Implemented by DPDK, Snort, Suricata

**KPS (Kilo Packets per Second)**
- Measure of packet processing rate
- 1 KPS = 1,000 packets per second
- Modern DDoS attacks often exceed 1,000 KPS (1 MPS)

---

## L

**Latency**
- Time delay between request and response
- Increased latency may indicate resource exhaustion
- DDoS mitigation (scrubbing) adds 5-20ms latency

**Layer 3 Attack**
- Network layer DDoS (IP protocol)
- Examples: IP floods, ICMP floods
- Mitigation: Bandwidth over-provisioning, scrubbing

**Layer 4 Attack**
- Transport layer DDoS (TCP/UDP)
- Examples: SYN flood, UDP flood
- Mitigation: SYN cookies, connection limits

**Layer 7 Attack**
- Application layer DDoS (HTTP, DNS application protocols)
- Examples: HTTP flood, DNS query flood
- Mitigation: WAF, rate limiting, challenge-response

**Leaky Bucket**
- Rate limiting algorithm smoothing traffic to constant rate
- Packets arriving faster than rate are queued or dropped
- Alternative: Token bucket (allows bursts)

**Load Balancer**
- Distributes traffic across multiple servers
- Provides horizontal scaling to absorb attacks
- Can implement health checks and failover

---

## M

**Memcached**
- Distributed caching system
- Vulnerable to amplification attacks when UDP port exposed
- 51,000x amplification factor (small request → large response)

**Mitigation**
- Actions taken to reduce impact of DDoS attack
- Techniques: Scrubbing, rate limiting, black holing
- Automatic vs. manual mitigation

**MPS (Mega Packets per Second)**
- Measure of packet processing rate
- 1 MPS = 1,000,000 packets per second
- High-volume attacks often exceed 10 MPS

---

## N

**NetFlow**
- Cisco protocol for collecting IP traffic information
- Used for traffic analysis and anomaly detection
- Alternative: sFlow, IPFIX

**NTP (Network Time Protocol)**
- Protocol for clock synchronization
- Vulnerable to amplification attacks (NTP monlist command)
- Up to 556x amplification factor

**Null Route**
- Route directing traffic to null interface (discard)
- Used in black hole routing
- Command: `ip route add blackhole <prefix>`

---

## O

**Origin Server**
- Backend server hosting actual content/application
- Protected by CDN/scrubbing center during DDoS
- Origin IP should be hidden to prevent direct attacks

**Over-provisioning**
- Deploying more capacity than normal requirements
- Provides buffer to absorb DDoS attacks
- Typical: 2-10x normal traffic capacity

---

## P

**Packet**
- Unit of data transmitted over network
- DDoS attacks generate millions/billions of packets
- Packet rate (pps) more relevant than bandwidth for some attacks

**PoP (Point of Presence)**
- Geographic location where network infrastructure deployed
- CDNs have hundreds of PoPs globally
- More PoPs = better DDoS absorption

**PPS (Packets per Second)**
- Measure of packet rate
- Critical metric for protocol attacks (SYN flood)
- Router/firewall capacity often limited by pps, not bandwidth

**Protocol Attack**
- DDoS exploiting weaknesses in network protocols
- Examples: SYN flood (TCP), smurf attack (ICMP)
- Mitigation: Protocol-specific defenses (SYN cookies)

---

## Q

**QoS (Quality of Service)**
- Network management prioritizing certain traffic
- Can deprioritize attack traffic during DDoS
- Implements via DSCP marking, traffic shaping

**Query Flood**
- Overwhelming server with large number of queries
- Common: DNS query flood, database query flood
- Mitigation: Rate limiting, caching, query optimization

---

## R

**Rate Limiting**
- Restricting number of requests per time period
- Dimensions: Per-IP, per-session, per-endpoint, global
- Algorithms: Token bucket, leaky bucket, sliding window

**Reflection Attack**
- DDoS using third-party servers to reflect traffic to victim
- Attacker spoofs victim's IP as source
- Server responds to victim, amplifying traffic

**Reverse Path Filtering**
- Anti-spoofing technique validating source IP against routing table
- Enabled via: `sysctl -w net.ipv4.conf.all.rp_filter=1`
- Prevents attackers from using spoofed source addresses

**ROA (Route Origin Authorization)**
- Cryptographic attestation of route origin in BGP
- Part of RPKI (Resource Public Key Infrastructure)
- Prevents BGP hijacking

**RTBH (Remotely Triggered Black Hole)**
- Technique to trigger black hole routing at upstream ISP
- RFC 5635
- Filters attack traffic before reaching your network

---

## S

**Scrubbing Center**
- Facility specializing in filtering DDoS traffic
- Traffic diverted through scrubbing center during attacks
- Clean traffic forwarded to origin

**sFlow**
- Network monitoring protocol (sampling-based)
- Alternative to NetFlow
- Used for traffic analysis and anomaly detection

**SIEM (Security Information and Event Management)**
- Centralized logging and security analytics platform
- Correlates events from multiple sources
- Examples: Splunk, ELK Stack, QRadar

**Signature-based Detection**
- Identifying attacks by matching known patterns
- Effective for known attack types
- Complemented by anomaly-based detection for novel attacks

**SLA (Service Level Agreement)**
- Contract defining expected service levels (uptime, performance)
- DDoS mitigation providers offer uptime SLAs
- Credits issued if SLA breached

**Slowloris**
- Low-bandwidth Layer 7 DDoS attack
- Holds connections open by slowly sending headers
- Exhausts server connection pool

**Smurf Attack**
- ICMP-based DDoS using broadcast addresses
- Largely obsolete (routers now block directed broadcasts)
- Historical significance in DDoS evolution

**Spoofing**
- Forging source IP address in packets
- Used in reflection attacks to hide attacker and target victim
- Prevented by BCP 38 and reverse path filtering

**SYN (Synchronize)**
- TCP flag initiating connection (first step of three-way handshake)
- SYN flood: Attack sending SYN packets without completing handshake
- Mitigation: SYN cookies

**SYN Cookies**
- Technique to prevent SYN flood without maintaining connection state
- Server encodes connection info in sequence number
- Enabled: `sysctl -w net.ipv4.tcp_syncookies=1`

---

## T

**Tbps (Terabits per second)**
- Measure of data transfer rate
- 1 Tbps = 1,000 Gbps
- Largest DDoS attacks approach or exceed 1 Tbps

**Three-Way Handshake**
- TCP connection establishment process
- Steps: SYN → SYN-ACK → ACK
- Exploited by SYN flood attacks

**Threshold**
- Value triggering alert or mitigation action
- Examples: Bandwidth > 10 Gbps, request rate > 10,000/sec
- Should be adaptive based on baseline traffic

**Token Bucket**
- Rate limiting algorithm allowing controlled bursts
- Tokens added at fixed rate, consumed by requests
- More flexible than leaky bucket

**Traffic Shaping**
- Controlling rate and volume of traffic
- Used to enforce QoS policies
- Can limit attack traffic during DDoS

**TTL (Time to Live)**
- IP header field limiting packet lifetime
- Decremented at each hop, packet dropped when 0
- Abnormal TTL values may indicate spoofed packets

---

## U

**UDP (User Datagram Protocol)**
- Connectionless transport protocol
- No handshake = easier to spoof source IP
- Common in volumetric attacks (UDP flood, DNS amplification)

**Uptime**
- Percentage of time service is available
- Target: 99.9% (8.76 hours downtime/year) to 99.999% (5.26 minutes/year)
- DDoS attacks threaten uptime SLAs

---

## V

**Volumetric Attack**
- DDoS overwhelming bandwidth with traffic volume
- Examples: UDP flood, ICMP flood, DNS amplification
- Measured in Gbps or Tbps

**VPN (Virtual Private Network)**
- Encrypted tunnel for secure communication
- Can be used to access services during DDoS (if VPN IP not targeted)
- VPN servers themselves can be DDoS targets

---

## W

**WAF (Web Application Firewall)**
- Firewall protecting web applications at Layer 7
- Blocks malicious HTTP traffic (SQL injection, XSS, DDoS)
- Examples: ModSecurity, Cloudflare WAF, AWS WAF

**Whitelist**
- List of trusted IPs/entities always allowed
- Ensures critical services maintained during DDoS
- Examples: Monitoring services, corporate VPN IPs

---

## X

**XFF (X-Forwarded-For)**
- HTTP header containing original client IP
- Added by proxies/load balancers/CDNs
- Used for rate limiting real client IP, not CDN IP

---

## Z

**Zero-Day Attack**
- Attack exploiting previously unknown vulnerability
- Difficult to defend with signature-based detection
- Requires behavioral analysis and anomaly detection

---

## Acronyms Quick Reference

| Acronym | Full Name |
|---------|-----------|
| ACK | Acknowledge |
| ASN | Autonomous System Number |
| BCP | Best Current Practice |
| BGP | Border Gateway Protocol |
| CAPTCHA | Completely Automated Public Turing test to tell Computers and Humans Apart |
| CDN | Content Delivery Network |
| DDoS | Distributed Denial of Service |
| DNS | Domain Name System |
| DoS | Denial of Service |
| DPDK | Data Plane Development Kit |
| DSCP | Differentiated Services Code Point |
| FAR | False Accept Rate |
| FRR | False Reject Rate |
| Gbps | Gigabits per second |
| GRE | Generic Routing Encapsulation |
| GSLB | Global Server Load Balancing |
| HTTP | Hypertext Transfer Protocol |
| ICMP | Internet Control Message Protocol |
| IDS | Intrusion Detection System |
| IP | Internet Protocol |
| IPS | Intrusion Prevention System |
| ISP | Internet Service Provider |
| KPS | Kilo Packets per Second |
| MPS | Mega Packets per Second |
| NTP | Network Time Protocol |
| PoP | Point of Presence |
| PPS | Packets per Second |
| QoS | Quality of Service |
| ROA | Route Origin Authorization |
| RPKI | Resource Public Key Infrastructure |
| RTBH | Remotely Triggered Black Hole |
| SIEM | Security Information and Event Management |
| SLA | Service Level Agreement |
| SYN | Synchronize |
| Tbps | Terabits per second |
| TCP | Transmission Control Protocol |
| TTL | Time to Live |
| UDP | User Datagram Protocol |
| VPN | Virtual Private Network |
| WAF | Web Application Firewall |
| XFF | X-Forwarded-For |

---

© 2025 World Certification Industry Association (WIA)
**弘益人間 · Benefit All Humanity**
