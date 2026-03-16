# WIA-SEC-015 Cybersecurity Standard
## Glossary of Terms

---

**Version**: 1.0.0
**Standard ID**: WIA-SEC-015
**Date**: 2025-12-25
**License**: MIT

---

## 목차 (Table of Contents)

1. [A-C](#a-c)
2. [D-F](#d-f)
3. [G-I](#g-i)
4. [J-M](#j-m)
5. [N-R](#n-r)
6. [S-Z](#s-z)
7. [Acronyms](#acronyms)

---

## A-C

### Advanced Persistent Threat (APT)
**고급 지속 위협**

장기간에 걸쳐 특정 목표를 대상으로 하는 정교한 사이버 공격. 일반적으로 국가 지원 또는 조직화된 범죄 그룹에 의해 수행됨.

Example: APT29 (Cozy Bear)는 러시아 정부와 연계된 것으로 알려진 APT 그룹.

---

### Anomaly Detection
**이상 탐지**

정상 행동 패턴에서 벗어나는 활동을 식별하는 기술. 머신러닝 알고리즘을 사용하여 알려지지 않은 위협을 탐지.

Example:
```python
# Anomaly detection using Isolation Forest
from sklearn.ensemble import IsolationForest

model = IsolationForest(contamination=0.1)
predictions = model.fit_predict(network_traffic_data)
anomalies = network_traffic_data[predictions == -1]
```

---

### Authentication
**인증**

사용자나 시스템의 신원을 확인하는 프로세스.

**Types**:
- **Something you know**: Password, PIN
- **Something you have**: Token, Smart card
- **Something you are**: Biometrics (fingerprint, face)
- **Somewhere you are**: Geolocation
- **Something you do**: Behavioral patterns

---

### Authorization
**권한 부여**

인증된 사용자가 특정 리소스에 접근할 수 있는 권한을 결정하는 프로세스.

**Models**:
- **RBAC** (Role-Based Access Control): 역할 기반
- **ABAC** (Attribute-Based Access Control): 속성 기반
- **MAC** (Mandatory Access Control): 강제 접근 제어
- **DAC** (Discretionary Access Control): 임의 접근 제어

---

### Backdoor
**백도어**

정상적인 인증 절차를 우회하여 시스템에 접근할 수 있도록 하는 숨겨진 방법.

Example: 악성코드가 설치한 원격 접근 트로이목마(RAT).

---

### Blockchain Security
**블록체인 보안**

분산 원장 기술을 사용하여 데이터 무결성과 변조 방지를 보장하는 보안 메커니즘.

Use cases in WIA-SEC-015:
- Tamper-proof audit logs
- Decentralized identity management
- Smart contract-based access control

---

### Brute Force Attack
**무차별 대입 공격**

가능한 모든 조합을 시도하여 패스워드나 암호화 키를 찾는 공격 방법.

Mitigation:
```yaml
brute_force_protection:
  max_attempts: 5
  lockout_duration: 15_minutes
  progressive_delay: true
  captcha_after: 3_attempts
```

---

### Certificate Authority (CA)
**인증 기관**

디지털 인증서를 발급하고 관리하는 신뢰할 수 있는 기관.

Example: Let's Encrypt, DigiCert, Verisign

---

### CIA Triad
**CIA 3원칙**

정보 보안의 세 가지 핵심 원칙:

1. **Confidentiality (기밀성)**: 권한 없는 접근 방지
2. **Integrity (무결성)**: 데이터 변조 방지
3. **Availability (가용성)**: 필요 시 데이터 접근 보장

---

### Common Vulnerabilities and Exposures (CVE)
**공통 취약점 식별자**

공개적으로 알려진 소프트웨어 취약점을 고유하게 식별하는 시스템.

Format: CVE-YYYY-NNNNN
Example: CVE-2021-44228 (Log4Shell)

---

### Cryptography
**암호학**

정보를 보호하기 위해 수학적 알고리즘을 사용하는 기술.

**Types**:
- **Symmetric**: Same key for encryption/decryption (AES)
- **Asymmetric**: Public/private key pairs (RSA, ECC)
- **Hash Functions**: One-way functions (SHA-256)

---

## D-F

### Data Loss Prevention (DLP)
**데이터 유출 방지**

민감한 데이터가 조직 외부로 무단 전송되는 것을 탐지하고 방지하는 기술.

Example DLP Rule:
```json
{
  "rule_name": "SSN Detection",
  "pattern": "\\d{3}-\\d{2}-\\d{4}",
  "action": "block",
  "notify": ["security_team", "compliance_officer"],
  "log": true
}
```

---

### DDoS (Distributed Denial of Service)
**분산 서비스 거부 공격**

다수의 시스템을 사용하여 대상 시스템을 과부하 상태로 만들어 정상적인 서비스를 불가능하게 하는 공격.

**Types**:
- **Volumetric**: Bandwidth saturation
- **Protocol**: Protocol stack exploitation
- **Application Layer**: Application resource exhaustion

---

### Defense in Depth
**심층 방어**

여러 계층의 보안 제어를 구현하여 단일 장애점을 방지하는 전략.

```
Layer 7: User Education
Layer 6: Application Security (WAF)
Layer 5: Endpoint Protection (EDR)
Layer 4: Network Security (Firewall, IPS)
Layer 3: Access Control (IAM, MFA)
Layer 2: Data Security (Encryption)
Layer 1: Physical Security
```

---

### Digital Signature
**디지털 서명**

공개키 암호화를 사용하여 문서의 진위성과 무결성을 보장하는 기술.

Example:
```python
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import padding

# Sign
signature = private_key.sign(
    message,
    padding.PSS(
        mgf=padding.MGF1(hashes.SHA256()),
        salt_length=padding.PSS.MAX_LENGTH
    ),
    hashes.SHA256()
)

# Verify
public_key.verify(
    signature,
    message,
    padding.PSS(
        mgf=padding.MGF1(hashes.SHA256()),
        salt_length=padding.PSS.MAX_LENGTH
    ),
    hashes.SHA256()
)
```

---

### Endpoint Detection and Response (EDR)
**엔드포인트 탐지 및 대응**

엔드포인트에서 위협을 탐지하고 조사하며 대응하는 사이버 보안 기술.

Capabilities:
- Real-time monitoring
- Behavioral analysis
- Threat hunting
- Incident response
- Forensic investigation

Popular EDR solutions: CrowdStrike Falcon, SentinelOne, Microsoft Defender

---

### Encryption
**암호화**

데이터를 읽을 수 없는 형태로 변환하여 권한 없는 접근으로부터 보호하는 프로세스.

**WIA-SEC-015 Approved Algorithms**:
```json
{
  "symmetric": [
    {"algorithm": "AES-256-GCM", "status": "recommended"},
    {"algorithm": "ChaCha20-Poly1305", "status": "approved"}
  ],
  "asymmetric": [
    {"algorithm": "RSA-4096", "status": "recommended"},
    {"algorithm": "ECDSA-P384", "status": "recommended"},
    {"algorithm": "Ed25519", "status": "approved"}
  ],
  "post_quantum": [
    {"algorithm": "CRYSTALS-Kyber", "status": "emerging"},
    {"algorithm": "CRYSTALS-Dilithium", "status": "emerging"}
  ]
}
```

---

### Exploit
**익스플로잇**

시스템의 취약점을 악용하는 코드나 기술.

Example: Buffer overflow exploit to gain shell access.

---

### Firewall
**방화벽**

네트워크 트래픽을 모니터링하고 보안 규칙에 따라 허용 또는 차단하는 보안 시스템.

**Types**:
- **Packet Filtering**: Layer 3/4
- **Stateful Inspection**: Connection state tracking
- **Application Layer**: Layer 7 inspection
- **Next-Generation (NGFW)**: Deep packet inspection, IPS, application awareness

---

## G-I

### GDPR (General Data Protection Regulation)
**일반 데이터 보호 규정**

EU의 데이터 보호 및 개인정보 보호 법률. 2018년 5월 시행.

Key requirements:
- Data minimization
- Right to be forgotten
- Data portability
- Breach notification (72 hours)
- Privacy by design

---

### Hash Function
**해시 함수**

임의 크기의 데이터를 고정 크기의 값(해시)으로 변환하는 일방향 함수.

Properties:
- Deterministic
- Quick computation
- Collision resistant
- Preimage resistant
- Avalanche effect

**WIA-SEC-015 Approved**:
- SHA-256
- SHA-384
- SHA-512
- SHA3-256
- BLAKE3

---

### Honeypot
**허니팟**

공격자를 유인하여 공격 기법을 연구하고 실제 시스템을 보호하기 위한 가짜 시스템.

Types:
- **Low-interaction**: Simulated services
- **High-interaction**: Full operating systems
- **Research honeypots**: Threat intelligence gathering
- **Production honeypots**: Early warning system

---

### Incident Response
**사고 대응**

사이버 보안 사고를 관리하고 해결하는 체계적인 접근 방법.

**NIST Incident Response Lifecycle**:
```
1. Preparation
   └─ Policies, procedures, tools

2. Detection & Analysis
   └─ Identify and validate incidents

3. Containment, Eradication & Recovery
   └─ Limit damage, remove threat, restore

4. Post-Incident Activity
   └─ Lessons learned, improve defenses
```

---

### Indicators of Compromise (IOC)
**침해 지표**

시스템이 손상되었음을 나타내는 증거.

Types:
```json
{
  "ioc_types": [
    {
      "type": "IP Address",
      "example": "203.0.113.45",
      "description": "Malicious IP communicating with C2"
    },
    {
      "type": "Domain",
      "example": "evil.example.com",
      "description": "Command & Control domain"
    },
    {
      "type": "File Hash",
      "example": "5d41402abc4b2a76b9719d911017c592",
      "description": "MD5 hash of malware"
    },
    {
      "type": "Registry Key",
      "example": "HKLM\\Software\\Malware",
      "description": "Persistence mechanism"
    },
    {
      "type": "URL",
      "example": "http://malware.com/payload.exe",
      "description": "Malware download location"
    }
  ]
}
```

---

### Intrusion Detection System (IDS)
**침입 탐지 시스템**

네트워크나 시스템에서 악의적인 활동을 모니터링하고 탐지하는 시스템.

**Types**:
- **NIDS** (Network-based): Network traffic monitoring
- **HIDS** (Host-based): Host activity monitoring
- **Signature-based**: Known attack patterns
- **Anomaly-based**: Deviation from baseline

Popular: Snort, Suricata, Zeek

---

### Intrusion Prevention System (IPS)
**침입 방지 시스템**

IDS와 유사하지만 탐지된 위협을 자동으로 차단하는 능력 포함.

Actions:
- Drop malicious packets
- Reset connections
- Block IP addresses
- Trigger alerts

---

## J-M

### JSON Web Token (JWT)
**JSON 웹 토큰**

당사자 간에 정보를 안전하게 전송하기 위한 컴팩트한 URL-safe 토큰.

Structure:
```
header.payload.signature

Example:
eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.
eyJzdWIiOiIxMjM0NTY3ODkwIiwibmFtZSI6IkpvaG4gRG9lIiwiaWF0IjoxNTE2MjM5MDIyfQ.
SflKxwRJSMeKKF2QT4fwpMeJf36POk6yJV_adQssw5c
```

---

### Kerberos
**커버로스**

네트워크 인증 프로토콜. 티켓 기반 인증을 사용하여 안전하지 않은 네트워크에서 안전한 인증 제공.

Components:
- **KDC** (Key Distribution Center)
- **AS** (Authentication Server)
- **TGS** (Ticket Granting Server)

---

### Key Management
**키 관리**

암호화 키의 생성, 저장, 배포, 순환, 폐기를 관리하는 프로세스.

Best practices:
```yaml
key_management:
  generation:
    - Use cryptographically secure RNG
    - Minimum 256-bit entropy
    - FIPS 140-2 compliant

  storage:
    - Hardware Security Module (HSM)
    - Encrypted key vault
    - Multi-party authorization

  rotation:
    - Automated rotation
    - 90-day symmetric keys
    - 1-year asymmetric keys

  destruction:
    - Cryptographic erasure
    - Physical destruction for HSM
    - Audit trail maintained
```

---

### Least Privilege
**최소 권한 원칙**

사용자나 시스템에게 작업 수행에 필요한 최소한의 권한만 부여하는 보안 원칙.

Example:
```json
{
  "user": "developer",
  "permissions": [
    "code:read",
    "code:write",
    "deploy:development",
    "logs:read"
  ],
  "denied": [
    "production:access",
    "security:modify",
    "users:admin"
  ]
}
```

---

### Malware
**악성코드**

컴퓨터 시스템에 해를 끼치거나 무단 접근을 획득하기 위해 설계된 소프트웨어.

**Types**:
- **Virus**: Self-replicating, requires host file
- **Worm**: Self-replicating, standalone
- **Trojan**: Disguised as legitimate software
- **Ransomware**: Encrypts files, demands payment
- **Spyware**: Collects information covertly
- **Rootkit**: Conceals malware presence
- **Adware**: Displays unwanted advertisements

---

### MITRE ATT&CK
**MITRE 공격 프레임워크**

실제 관찰된 공격자의 전술과 기술을 체계적으로 분류한 지식 베이스.

Structure:
```
Tactics (The "Why")
└─ Techniques (The "How")
   └─ Sub-techniques (Specific variants)
      └─ Procedures (Real-world examples)

Example:
TA0001: Initial Access
├─ T1190: Exploit Public-Facing Application
│  └─ T1190.001: Exploit SQL Injection
└─ T1566: Phishing
   ├─ T1566.001: Spearphishing Attachment
   └─ T1566.002: Spearphishing Link
```

---

### Multi-Factor Authentication (MFA)
**다중 인증**

사용자 신원을 확인하기 위해 두 개 이상의 검증 요소를 요구하는 인증 방법.

Methods:
```typescript
enum MFAMethod {
  TOTP = 'Time-based One-Time Password',  // Google Authenticator
  SMS = 'SMS Code',
  Email = 'Email Code',
  WebAuthn = 'Hardware Security Key',     // YubiKey
  Push = 'Push Notification',              // Duo Push
  Biometric = 'Fingerprint/Face ID'
}
```

---

## N-R

### Network Segmentation
**네트워크 세그멘테이션**

네트워크를 논리적 또는 물리적 세그먼트로 분할하여 공격 표면을 줄이고 측면 이동을 제한.

Example:
```
DMZ (Demilitarized Zone)
├─ Public Web Servers
└─ Mail Servers

Internal Network
├─ User Workstations (VLAN 10)
├─ Database Servers (VLAN 20)
├─ Application Servers (VLAN 30)
└─ Management Network (VLAN 99)

Security Zone
└─ Security Tools (VLAN 100)
```

---

### NIST Cybersecurity Framework
**NIST 사이버보안 프레임워크**

조직의 사이버보안 위험을 관리하기 위한 자발적 프레임워크.

Five Core Functions:
1. **Identify**: Asset management, risk assessment
2. **Protect**: Access control, data security
3. **Detect**: Continuous monitoring, anomaly detection
4. **Respond**: Incident response, mitigation
5. **Recover**: Recovery planning, improvements

---

### OAuth 2.0
**OAuth 2.0**

권한 부여를 위한 개방형 표준 프로토콜.

Grant Types:
- Authorization Code
- Implicit
- Resource Owner Password Credentials
- Client Credentials

---

### OWASP Top 10
**OWASP 상위 10대 위협**

웹 애플리케이션 보안의 가장 중요한 10가지 위험.

**2021 Edition**:
1. Broken Access Control
2. Cryptographic Failures
3. Injection
4. Insecure Design
5. Security Misconfiguration
6. Vulnerable and Outdated Components
7. Identification and Authentication Failures
8. Software and Data Integrity Failures
9. Security Logging and Monitoring Failures
10. Server-Side Request Forgery (SSRF)

---

### Penetration Testing
**모의 침투 테스트**

시스템의 보안 취약점을 발견하기 위해 승인된 공격을 시뮬레이션하는 테스트.

Phases:
```
1. Reconnaissance
   └─ Information gathering

2. Scanning
   └─ Vulnerability identification

3. Exploitation
   └─ Attempt to exploit vulnerabilities

4. Post-Exploitation
   └─ Maintain access, pivot

5. Reporting
   └─ Document findings, recommendations
```

---

### Phishing
**피싱**

사용자를 속여 민감한 정보를 공개하도록 하는 소셜 엔지니어링 공격.

Types:
- **Spear Phishing**: Targeted at specific individuals
- **Whaling**: Targeted at high-profile executives
- **Vishing**: Voice phishing via phone
- **Smishing**: SMS phishing
- **Clone Phishing**: Duplicating legitimate emails

---

### Post-Quantum Cryptography
**양자 내성 암호**

양자 컴퓨터의 공격에도 안전하도록 설계된 암호 알고리즘.

**NIST Selected Algorithms** (2022):
```json
{
  "key_encapsulation": [
    "CRYSTALS-Kyber"
  ],
  "digital_signatures": [
    "CRYSTALS-Dilithium",
    "FALCON",
    "SPHINCS+"
  ]
}
```

---

### Public Key Infrastructure (PKI)
**공개 키 기반 구조**

디지털 인증서를 생성, 관리, 배포, 사용, 저장 및 폐기하는 정책, 프로세스 및 기술의 집합.

Components:
- **Certificate Authority (CA)**
- **Registration Authority (RA)**
- **Certificate Database**
- **Certificate Store**
- **Certificate Revocation List (CRL)**

---

### Ransomware
**랜섬웨어**

파일을 암호화하고 복호화를 위해 금전을 요구하는 악성코드.

Notable examples:
- WannaCry (2017)
- NotPetya (2017)
- Ryuk (2018-)
- REvil (2019-2021)

---

### Risk Assessment
**위험 평가**

시스템과 데이터에 대한 위협을 식별하고 우선순위를 정하는 프로세스.

Formula:
```
Risk = Likelihood × Impact

Risk Score = (Threat × Vulnerability × Asset Value) / Countermeasures
```

---

## S-Z

### Security Information and Event Management (SIEM)
**보안 정보 및 이벤트 관리**

실시간 보안 경보 분석을 제공하는 보안 관리 시스템.

Capabilities:
- Log aggregation
- Correlation
- Alerting
- Dashboards
- Compliance reporting
- Incident response

Popular SIEM: Splunk, IBM QRadar, Azure Sentinel, Elastic Security

---

### Security Operations Center (SOC)
**보안 운영 센터**

조직의 보안 상태를 모니터링하고 개선하는 중앙 집중식 팀.

**SOC Tiers**:
```
Tier 1: Analysts
└─ Alert triage, initial investigation

Tier 2: Incident Responders
└─ Deep analysis, containment

Tier 3: Threat Hunters
└─ Proactive threat hunting, advanced forensics

SOC Manager
└─ Team management, metrics, process improvement
```

---

### Security Orchestration, Automation and Response (SOAR)
**보안 오케스트레이션, 자동화 및 대응**

보안 운영을 자동화하고 조율하는 플랫폼.

Functions:
- Playbook automation
- Case management
- Threat intelligence integration
- Response orchestration

---

### SQL Injection
**SQL 인젝션**

악의적인 SQL 코드를 주입하여 데이터베이스를 조작하는 공격.

Example:
```sql
-- Vulnerable code
query = "SELECT * FROM users WHERE username = '" + username + "'"

-- Attack input
username = "admin' OR '1'='1"

-- Resulting query
SELECT * FROM users WHERE username = 'admin' OR '1'='1'
```

Prevention:
- Parameterized queries
- ORM usage
- Input validation
- Least privilege database accounts

---

### SSL/TLS
**보안 소켓 계층/전송 계층 보안**

인터넷 통신을 암호화하는 프로토콜.

**Current Recommendation**:
- TLS 1.3 (recommended)
- TLS 1.2 (acceptable)
- SSL 2.0, SSL 3.0, TLS 1.0, TLS 1.1 (deprecated)

---

### Threat Intelligence
**위협 인텔리전스**

조직에 대한 위협에 대한 증거 기반 지식.

Types:
- **Strategic**: High-level trends
- **Tactical**: TTPs (Tactics, Techniques, Procedures)
- **Operational**: Specific campaigns
- **Technical**: IOCs (Indicators of Compromise)

---

### Two-Factor Authentication (2FA)
**2단계 인증**

→ See Multi-Factor Authentication (MFA)

---

### Vulnerability
**취약점**

공격자가 악용할 수 있는 시스템의 약점.

CVSS (Common Vulnerability Scoring System):
```
Score Range | Severity
0.0         | None
0.1 - 3.9   | Low
4.0 - 6.9   | Medium
7.0 - 8.9   | High
9.0 - 10.0  | Critical
```

---

### Web Application Firewall (WAF)
**웹 애플리케이션 방화벽**

HTTP/HTTPS 트래픽을 필터링하고 모니터링하여 웹 애플리케이션을 보호하는 방화벽.

Protection against:
- SQL Injection
- Cross-Site Scripting (XSS)
- Cross-Site Request Forgery (CSRF)
- File inclusion
- DDoS

Popular WAF: ModSecurity, Cloudflare, AWS WAF, Imperva

---

### Zero-Day
**제로데이**

공개되지 않았거나 패치가 없는 취약점.

Timeline:
```
Day 0: Vulnerability discovered by attacker
Day 0: Exploit developed
Day 0-N: Attacks in the wild
Day N: Vendor becomes aware
Day N+X: Patch released
```

---

### Zero Trust
**제로 트러스트**

"절대 신뢰하지 말고, 항상 검증하라"는 보안 모델.

Core Principles:
1. Verify explicitly
2. Use least privilege access
3. Assume breach
4. Inspect and log all traffic
5. Continuous monitoring
6. Micro-segmentation

Implementation:
```yaml
zero_trust_architecture:
  identity_verification:
    - Multi-factor authentication
    - Device compliance check
    - Continuous authentication

  network:
    - Software-defined perimeter
    - Micro-segmentation
    - Encrypted tunnels

  data:
    - Data classification
    - Encryption everywhere
    - DLP policies

  applications:
    - Least privilege
    - API security
    - Runtime protection
```

---

## Acronyms

| Acronym | Full Form | Korean |
|---------|-----------|--------|
| **AES** | Advanced Encryption Standard | 고급 암호화 표준 |
| **API** | Application Programming Interface | 응용 프로그램 인터페이스 |
| **APT** | Advanced Persistent Threat | 고급 지속 위협 |
| **BCP** | Business Continuity Plan | 사업 연속성 계획 |
| **C2** | Command and Control | 명령 및 제어 |
| **CA** | Certificate Authority | 인증 기관 |
| **CASB** | Cloud Access Security Broker | 클라우드 접근 보안 브로커 |
| **CIS** | Center for Internet Security | 인터넷 보안 센터 |
| **CISO** | Chief Information Security Officer | 최고 정보 보안 책임자 |
| **CSIRT** | Computer Security Incident Response Team | 컴퓨터 보안 사고 대응팀 |
| **CVE** | Common Vulnerabilities and Exposures | 공통 취약점 식별자 |
| **CVSS** | Common Vulnerability Scoring System | 공통 취약점 평가 시스템 |
| **DDoS** | Distributed Denial of Service | 분산 서비스 거부 |
| **DLP** | Data Loss Prevention | 데이터 유출 방지 |
| **DMZ** | Demilitarized Zone | 비무장지대 |
| **DNS** | Domain Name System | 도메인 이름 시스템 |
| **DRP** | Disaster Recovery Plan | 재해 복구 계획 |
| **EDR** | Endpoint Detection and Response | 엔드포인트 탐지 및 대응 |
| **GDPR** | General Data Protection Regulation | 일반 데이터 보호 규정 |
| **HIDS** | Host-based Intrusion Detection System | 호스트 기반 침입 탐지 시스템 |
| **HSM** | Hardware Security Module | 하드웨어 보안 모듈 |
| **IAM** | Identity and Access Management | 신원 및 접근 관리 |
| **IDS** | Intrusion Detection System | 침입 탐지 시스템 |
| **IOC** | Indicator of Compromise | 침해 지표 |
| **IPS** | Intrusion Prevention System | 침입 방지 시스템 |
| **IR** | Incident Response | 사고 대응 |
| **ISMS** | Information Security Management System | 정보 보안 관리 시스템 |
| **MFA** | Multi-Factor Authentication | 다중 인증 |
| **MITM** | Man-in-the-Middle | 중간자 공격 |
| **NIDS** | Network-based Intrusion Detection System | 네트워크 기반 침입 탐지 시스템 |
| **NIST** | National Institute of Standards and Technology | 미국 표준 기술 연구소 |
| **OWASP** | Open Web Application Security Project | 오픈 웹 애플리케이션 보안 프로젝트 |
| **PAM** | Privileged Access Management | 특권 접근 관리 |
| **PCI DSS** | Payment Card Industry Data Security Standard | 지불 카드 산업 데이터 보안 표준 |
| **PII** | Personally Identifiable Information | 개인 식별 정보 |
| **PKI** | Public Key Infrastructure | 공개 키 기반 구조 |
| **PQC** | Post-Quantum Cryptography | 양자 후 암호학 |
| **RAT** | Remote Access Trojan | 원격 접근 트로이목마 |
| **RBAC** | Role-Based Access Control | 역할 기반 접근 제어 |
| **RTO** | Recovery Time Objective | 복구 시간 목표 |
| **RPO** | Recovery Point Objective | 복구 시점 목표 |
| **SAML** | Security Assertion Markup Language | 보안 주장 마크업 언어 |
| **SIEM** | Security Information and Event Management | 보안 정보 및 이벤트 관리 |
| **SOC** | Security Operations Center | 보안 운영 센터 |
| **SOAR** | Security Orchestration, Automation and Response | 보안 오케스트레이션, 자동화 및 대응 |
| **SOC2** | Service Organization Control 2 | 서비스 조직 통제 2 |
| **SQL** | Structured Query Language | 구조화된 쿼리 언어 |
| **SSL** | Secure Sockets Layer | 보안 소켓 계층 |
| **SSO** | Single Sign-On | 단일 로그인 |
| **TLS** | Transport Layer Security | 전송 계층 보안 |
| **TTP** | Tactics, Techniques, and Procedures | 전술, 기법 및 절차 |
| **UEBA** | User and Entity Behavior Analytics | 사용자 및 개체 행동 분석 |
| **VPN** | Virtual Private Network | 가상 사설망 |
| **WAF** | Web Application Firewall | 웹 애플리케이션 방화벽 |
| **XSS** | Cross-Site Scripting | 크로스 사이트 스크립팅 |

---

**弘益人間 (Hongik Ingan) - Benefit All Humanity**

© 2025 WIA - World Certification Industry Association
Licensed under MIT License
