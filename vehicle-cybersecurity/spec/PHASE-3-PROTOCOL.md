# WIA-AUTO-023 PHASE 3: Security Protocol Specification

**Version:** 1.0
**Status:** Final
**Date:** 2025-12-27
**Category:** Automotive Cybersecurity

---

## 1. Introduction

Phase 3 of the WIA-AUTO-023 Vehicle Cybersecurity Standard defines security protocols governing execution of security operations across automotive systems. These protocols specify procedures, sequences, and behaviors for secure boot, intrusion detection, incident response, and over-the-air updates.

---

## 2. Secure Boot Protocol

### 2.1 Boot Chain of Trust

The secure boot protocol establishes a chain of trust from hardware through all software layers:

**Stage 0 - Boot ROM:**
- Immutable hardware root of trust
- Verifies Stage 1 bootloader signature using public key in hardware fuses
- Hash: SHA-256 minimum
- Signature: RSA-2048 or ECDSA-P256 minimum
- Failure action: Halt boot, enter recovery mode

**Stage 1 - Bootloader:**
- Verifies OS kernel signature and critical libraries
- Maintains rollback counter in tamper-resistant storage
- Signature: RSA-3072 or ECDSA-P384 recommended
- Failure action: Attempt fallback bootloader or previous version

**Stage 2 - OS Kernel:**
- Verifies security services and critical applications
- Establishes security policies and initializes crypto subsystems
- Loads signed kernel modules only
- Failure action: Boot into safe mode with reduced functionality

**Stage 3 - Applications:**
- Verifies application signatures before loading
- Applications run with least privilege based on verified permissions
- Failure action: Skip application, continue boot

### 2.2 Rollback Protection

- Each firmware version has security patch level counter
- Bootloader verifies firmware patch level >= stored counter
- Successful boot increments counter if firmware has higher patch level
- Counter stored in write-once or authenticated storage
- Emergency rollback requires authorized override with strong authentication

---

## 3. CAN Bus Security Protocol

### 3.1 Message Authentication

**Truncated MAC Approach:**
- Compute HMAC-SHA256 over (Message ID || Data || Counter)
- Transmit 32-48 bits of MAC in CAN payload
- Remaining payload carries application data
- Receiver verifies MAC before processing message
- Counter prevents replay attacks (incrementing, 16-bit minimum)

**Group Key Management:**
- ECUs share group keys for specific message categories
- Keys derived from master key using HKDF
- Key rotation every 24 hours or 1,000,000 messages
- Smooth key transition: accept previous key for overlap period

### 3.2 Intrusion Detection Protocol

**Frequency Monitoring:**
- Monitor message rate for each CAN ID
- Establish baseline during learning period (1000km or 30 days)
- Alert if rate deviates > 3 standard deviations from baseline
- Adaptive thresholds adjust for different driving modes

**Sequence Validation:**
- Define expected message sequences for safety-critical functions
- Verify state machine progression matches expected transitions
- Alert on unexpected sequences or missing expected messages
- Example: Brake activation must follow brake pedal depression

**Content Validation:**
- Check payloads against physically possible ranges
- Cross-validate related sensor data (e.g., wheel speeds vs vehicle speed)
- Alert on impossible values or inconsistent data
- Use vehicle dynamics model for advanced validation

---

## 4. Incident Response Protocol

### 4.1 Response Phases

**1. Detection:**
- IDS alerts, authentication failures, integrity check failures
- External threat intelligence notifications
- Automated correlation of multiple low-severity events

**2. Classification:**
- Severity: CRITICAL, HIGH, MEDIUM, LOW, INFORMATIONAL
- Type: Attack category (injection, DoS, unauthorized access, etc.)
- Scope: Affected systems and potential impact
- Automated classification using threat signatures and ML models

**3. Containment:**
- CRITICAL: Immediate isolation of affected systems
- HIGH: Enhanced monitoring, rate limiting, access restrictions
- MEDIUM: Log for analysis, continue monitoring
- Actions: Network isolation, credential revocation, fail-safe modes

**4. Eradication:**
- Remove threat from system
- May require: firmware updates, key rotation, system reconfiguration
- Verify removal through forensic analysis
- Document root cause and attack vector

**5. Recovery:**
- Restore normal operation while maintaining security
- Verify system integrity before full operation
- Monitor closely for recurrence (enhanced monitoring period)
- Gradual restoration of functionality

**6. Post-Incident:**
- Root cause analysis
- Update threat signatures and detection rules
- Implement preventive measures
- Share anonymized threat intelligence with industry

### 4.2 Response Time Requirements

| Severity | Detection | Classification | Containment | Eradication |
|----------|-----------|----------------|-------------|-------------|
| CRITICAL | < 1 min | < 5 min | < 15 min | < 24 hours |
| HIGH | < 5 min | < 15 min | < 1 hour | < 7 days |
| MEDIUM | < 30 min | < 2 hours | < 24 hours | < 30 days |
| LOW | < 24 hours | < 1 week | As scheduled | < 90 days |

---

## 5. Over-the-Air Update Protocol

### 5.1 Update Distribution Flow

**1. Update Notification:**
- Backend notifies vehicle of available update
- Includes: version, size, priority, prerequisites
- Vehicle checks: storage space, battery level, network connectivity

**2. Pre-Update Verification:**
- Verify prerequisites (vehicle state, resources)
- Check update applicability (hardware version, current firmware)
- User consent if required (non-critical updates)

**3. Secure Download:**
- Establish encrypted channel (TLS 1.3 minimum)
- Download with resume capability
- Compute rolling hash during download
- Verify complete package hash before installation

**4. Signature Verification:**
- Verify package signature using manufacturer public key
- Check certificate validity and revocation status
- Verify signing key authorized for this vehicle/ECU
- Reject if any verification fails

**5. Installation (A/B Partitioning):**
- Install to inactive partition while current version runs
- Maintain current version as fallback
- Update bootloader last (if required)

**6. Post-Install Verification:**
- Boot new version in test mode
- Run integrity checks and basic functionality tests
- Monitor for errors during initial operation

**7. Commit or Rollback:**
- Success: Mark new version as active, update rollback counter
- Failure: Automatic rollback to previous version, report failure
- Partial success: System-specific decision based on criticality

**8. Reporting:**
- Report status to backend (success/failure/partial)
- Include logs and diagnostics for failures
- Anonymized telemetry for update analytics

### 5.2 Update Priority Levels

| Priority | Description | Download | Installation | User Action |
|----------|-------------|----------|--------------|-------------|
| EMERGENCY | Critical safety/security fix | Immediate | Immediate (with notification) | None required |
| HIGH | Important security update | Within 24 hours | Next ignition cycle | Notification only |
| NORMAL | Regular updates, new features | Within 1 week | User scheduled or automatic | User can defer |
| LOW | Minor improvements | Within 1 month | User scheduled only | User must approve |

---

## 6. Network Segmentation Protocol

### 6.1 Security Zones

**Zone 1 - Safety Critical:**
- Systems: Brake, steering, powertrain ECUs
- Trust level: Highest
- External access: Denied
- Inter-zone: Filtered through security gateway only

**Zone 2 - Body Control:**
- Systems: Lighting, windows, seats, HVAC
- Trust level: High
- External access: Restricted, authenticated only
- Inter-zone: Allowed with gateway inspection

**Zone 3 - Infotainment:**
- Systems: Media, navigation, apps
- Trust level: Medium
- External access: Allowed with firewall filtering
- Inter-zone: Limited, no direct access to Zone 1

**Zone 4 - Telematics:**
- Systems: Cellular, GPS, V2X
- Trust level: Low (highest exposure)
- External access: Full (monitored)
- Inter-zone: Gateway with deep packet inspection

**Zone 5 - Diagnostic:**
- Systems: OBD-II, service ports
- Trust level: Variable (depends on authentication)
- External access: Authenticated only, time-limited
- Inter-zone: Full access if properly authenticated

### 6.2 Gateway Firewall Rules

- Default deny: Block all traffic not explicitly allowed
- Stateful inspection: Track connection state
- Rate limiting: Prevent flooding attacks
- Protocol validation: Verify message format compliance
- Content filtering: Block known attack patterns
- Logging: All denied connections logged for analysis

---

## 7. Authentication Protocols

### 7.1 Challenge-Response Protocol

**Mutual Authentication Flow:**

1. **Client → Server:** Authentication request with identity claim
2. **Server → Client:** Random challenge (nonce, minimum 128 bits)
3. **Client → Server:** Response = Sign(challenge || timestamp || clientData, privateKey)
4. **Server:** Verify signature, check timestamp freshness (< 5 minutes)
5. **Server → Client:** Session token (time-limited, 15-60 minutes)
6. **Client:** Include token in subsequent requests
7. **Server:** Validate token on each request

**Token Format (JWT):**
```json
{
  "header": {
    "alg": "ES256",
    "typ": "JWT"
  },
  "payload": {
    "sub": "vehicle-id",
    "iss": "wia-auth-service",
    "exp": timestamp,
    "iat": timestamp,
    "scopes": ["read", "write", "admin"]
  },
  "signature": "base64-encoded-signature"
}
```

---

## 8. Anomaly Detection Protocol

### 8.1 Baseline Establishment

**Learning Phase (1000km or 30 days):**
- Record normal operation patterns
- Message frequencies, timing, value ranges
- System state transitions
- Build statistical models (mean, std dev, min/max)
- Train machine learning models on normal behavior

**Baseline Updates:**
- Continuous refinement after initial learning
- Gradual adaptation to changed usage patterns
- Major changes (software update, hardware change) may require re-learning

### 8.2 Anomaly Scoring

**Score Calculation (0-100):**
- Deviation from statistical baseline
- ML model confidence of abnormality
- Historical pattern (recurring vs new)
- Multiple correlated anomalies increase score

**Score Thresholds:**
- 90-100: Very high confidence attack, immediate response
- 70-89: High confidence, enhanced monitoring + notification
- 50-69: Medium confidence, log and analyze
- 30-49: Low confidence, background logging only
- 0-29: Noise, ignore

**Score Decay:**
- Scores decay over time if anomaly doesn't recur
- Prevents false positive accumulation
- Decay rate: 10% per hour for scores < 70, no decay for scores >= 70

---

## 9. Key Rotation Protocol

### 9.1 Rotation Schedule

**Root Keys (Manufacturer Master Keys):**
- Rotation: Every 2-5 years or upon suspected compromise
- Coordination: Industry-wide notification, coordinated rollout
- Process: Dual-key period with overlap for transition

**Intermediate Keys (Fleet/Region Keys):**
- Rotation: Every 6-12 months
- Coordination: Fleet-level rollout over 1-2 months
- Process: Gradual migration, monitor for issues

**Session Keys:**
- Rotation: Every session or 24 hours
- Coordination: Automatic, no user impact
- Process: Transparent key exchange

**Message Keys (CAN MAC Keys):**
- Rotation: Every 10,000-100,000 messages or 24 hours
- Coordination: Synchronized across ECUs
- Process: Smooth transition accepting both old and new

### 9.2 Key Transition Protocol

1. **Pre-Rotation:** Distribute new key (encrypted with current key)
2. **Transition Start:** Accept messages with old OR new key
3. **Switchover:** Begin sending with new key, continue accepting old
4. **Transition End:** Stop accepting old key after grace period
5. **Post-Rotation:** Old key securely destroyed

---

## 10. Testing and Validation

### 10.1 Protocol Conformance Testing

- **Positive Tests:** Verify correct protocol execution
- **Negative Tests:** Verify proper handling of invalid inputs
- **Timing Tests:** Verify timing requirements met
- **Fault Injection:** Test protocol robustness under failures
- **Interoperability:** Verify protocol compatibility across implementations

### 10.2 Security Testing

- **Penetration Testing:** Attempt to bypass protocol security
- **Fuzzing:** Test protocol parsers with malformed inputs
- **Replay Attacks:** Verify replay protection effective
- **Man-in-the-Middle:** Test resistance to MITM attacks
- **Downgrade Attacks:** Verify cannot force weaker cryptography

---

**弘益人間 (홍익인간) - Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
