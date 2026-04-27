# PHASE 4 — Advanced Protection Mechanisms


### 4.1 Application Whitelisting

#### 4.1.1 Allow-List Enforcement
Only permit execution of approved applications:

**Windows AppLocker Policy:**
```xml
<AppLockerPolicy Version="1">
  <RuleCollection Type="Exe" EnforcementMode="Enabled">
    <!-- Allow Windows system binaries -->
    <FilePathRule Id="windows-system"
      Action="Allow"
      Path="%WINDIR%\*" />

    <!-- Allow Program Files -->
    <FilePathRule Id="program-files"
      Action="Allow"
      Path="%PROGRAMFILES%\*" />

    <!-- Deny user writable locations -->
    <FilePathRule Id="block-temp"
      Action="Deny"
      Path="%TEMP%\*.exe" />
    <FilePathRule Id="block-appdata"
      Action="Deny"
      Path="%APPDATA%\*.exe" />
  </RuleCollection>
</AppLockerPolicy>
```

**Benefits:**
- Prevents execution of ransomware from common drop locations
- Reduces attack surface significantly
- Low performance overhead

---

### 4.2 Network Segmentation

#### 4.2.1 Zero Trust Architecture
Implement micro-segmentation to limit ransomware spread:

**Network Zones:**
```
┌─────────────────────────────────────────┐
│ ZONE 1: Critical Infrastructure         │
│ - Domain Controllers                     │
│ - Backup Servers (air-gapped access)     │
│ - Database Servers                       │
│ Access: Strictly controlled, MFA required│
└─────────────────────────────────────────┘
         ↕ (Firewall + IPS)
┌─────────────────────────────────────────┐
│ ZONE 2: Application Servers              │
│ - Web servers                            │
│ - Application servers                    │
│ Access: Service accounts only            │
└─────────────────────────────────────────┘
         ↕ (Firewall + IPS)
┌─────────────────────────────────────────┐
│ ZONE 3: User Workstations                │
│ - Employee endpoints                     │
│ - VDI sessions                           │
│ Access: User authentication              │
└─────────────────────────────────────────┘
```

#### 4.2.2 Firewall Rules
Prevent lateral movement with restrictive firewall policies:

```bash
# Block workstation-to-workstation SMB
iptables -A FORWARD -s 192.168.1.0/24 -d 192.168.1.0/24 -p tcp --dport 445 -j DROP

# Block RDP between workstations
iptables -A FORWARD -s 192.168.1.0/24 -d 192.168.1.0/24 -p tcp --dport 3389 -j DROP

# Allow only to designated RDP gateways
iptables -A FORWARD -d 192.168.10.100 -p tcp --dport 3389 -j ACCEPT
```

---

### 4.3 Credential Protection

#### 4.3.1 Credential Guard
Enable Windows Credential Guard to protect credentials:

```powershell
# Enable Credential Guard
Set-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\DeviceGuard" -Name "EnableVirtualizationBasedSecurity" -Value 1
Set-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\Lsa" -Name "LsaCfgFlags" -Value 1
```

#### 4.3.2 Least Privilege Access
Minimize privileges to limit ransomware impact:

**Principles:**
- Users run with standard (non-admin) accounts
- Admin rights granted via Just-In-Time (JIT) access
- Service accounts have minimal necessary permissions
- Privileged Access Workstations (PAWs) for admin tasks

---

### 4.4 Secure Boot & TPM

#### 4.4.1 UEFI Secure Boot
Prevent bootkit and rootkit infections:

**Requirements:**
- UEFI firmware with Secure Boot
- Signed bootloader and kernel
- TPM 2.0 chip for measured boot

**Configuration:**
```bash
# Verify Secure Boot status
mokutil --sb-state
# Expected: SecureBoot enabled

# Check TPM status
cat /sys/class/tpm/tpm0/device/enabled
# Expected: 1
```

#### 4.4.2 BitLocker / LUKS Encryption
Full disk encryption prevents offline attacks:

**Windows BitLocker:**
```powershell
Enable-BitLocker -MountPoint "C:" -EncryptionMethod XtsAes256 -TpmProtector
```

**Linux LUKS:**
```bash
cryptsetup luksFormat /dev/sda2 --type luks2 --cipher aes-xts-plain64 --key-size 512
```

---

### 4.5 Email & Web Protection

#### 4.5.1 Advanced Email Filtering
Multi-layer email security to prevent phishing:

**Layers:**
1. **SPF/DKIM/DMARC**: Verify sender authenticity
2. **Sandbox Analysis**: Detonate attachments in sandbox
3. **URL Rewriting**: Proxy all links through security gateway
4. **AI-Based Detection**: Identify phishing patterns

**Email Gateway Config:**
```json
{
  "email_security": {
    "sandbox": {
      "enabled": true,
      "timeout": "60s",
      "file_types": [".exe", ".zip", ".pdf", ".doc", ".xls"]
    },
    "url_protection": {
      "rewrite": true,
      "time_of_click": true,
      "categories_blocked": ["malware", "phishing", "newly_registered"]
    },
    "attachment_blocking": {
      "extensions": [".exe", ".scr", ".bat", ".cmd", ".ps1"]
    }
  }
}
```

#### 4.5.2 Web Content Filtering
Block access to malicious and risky sites:

**Categories to Block:**
- Malware distribution sites
- Phishing sites
- Newly registered domains (<30 days)
- Tor exit nodes
- File sharing sites (reduce risk)

---

### 4.6 Continuous Monitoring & Improvement

#### 4.6.1 Security Metrics Dashboard
Real-time visibility into ransomware protection posture:

**Key Metrics:**
- Detection rate (should be >99.5%)
- False positive rate (should be <0.1%)
- Average response time (target: <100ms)
- Backup success rate (should be 100%)
- Mean time to recovery (MTTR)
- Employee training completion rate

#### 4.6.2 Continuous Improvement
Regular reviews and updates:

**Monthly:**
- Review detection logs and false positives
- Update detection rules and signatures
- Patch management review

**Quarterly:**
- DR drill execution
- Security training for employees
- Threat landscape assessment

**Annually:**
- Full security audit
- Penetration testing
- Policy and procedure review

---

## Summary

This comprehensive ransomware protection standard provides:

✅ **Prevention**: Multiple layers prevent ransomware from executing
✅ **Detection**: Advanced AI and behavioral analysis detect threats <100ms
✅ **Response**: Automated isolation and containment within milliseconds
✅ **Recovery**: Immutable backups ensure rapid recovery (RTO <15 min)
✅ **Resilience**: Business continuity planning maintains operations
✅ **Improvement**: Continuous monitoring and testing strengthen defenses

---

**Related Documents:**
- [PHASE 1 - Core Specification](./PHASE-1-CORE.md)
- [Appendix - Implementation Guides](./SPEC-APPENDIX.md)
- [Glossary - Terms & Definitions](./SPEC-GLOSSARY.md)

---

*© 2025 WIA - World Certification Industry Association*
*弘益人間 · Benefit All Humanity*

## Implementer notes — protection envelope

Phase 4 covers proactive protection mechanisms: deception, segmentation,
identity hardening, and supply-chain integrity. The envelopes below are
intended to plug into existing zero-trust and EDR/XDR stacks without
requiring product-specific schema bridges.

### `deception_asset`

A `deception_asset` is any honeytoken, honeyfile, honeyaccount, or
honey-network deployed to detect adversaries earlier in the kill chain.

```
{
  "asset_id": "ULID",
  "tenant_id": "did:wia:tenant:...",
  "kind": "honeyfile" | "honeytoken" | "honeyaccount" | "honeyhost"
        | "honeykubelet" | "honeybucket",
  "deployed_at": "RFC 3339",
  "location_ref": "opaque (path/identifier in tenant context)",
  "fidelity": "low" | "medium" | "high",
  "alert_routing": "did:wia:soc:...",
  "auto_isolate_on_touch": true,
  "signature": "Ed25519 JCS"
}
```

`fidelity` is a deliberate signal to detection engineers: low-fidelity
canaries are noisy (e.g., a fake admin account that triggers on simple
LDAP enumeration); high-fidelity canaries trigger only on meaningful
adversary action (e.g., honeyfile opened from outside the segment).

### `segmentation_policy`

A `segmentation_policy` envelope describes a network/identity boundary
in declarative form, independent of vendor (Illumio, Cisco ACI, AWS
SCP, Kubernetes NetworkPolicy, Azure NSG, etc.). Implementations
translate into the underlying enforcement plane.

```
{
  "policy_id": "ULID",
  "scope": "workload" | "namespace" | "vpc" | "tenant",
  "selector": { ... vendor-neutral label selector ... },
  "ingress": [ { "from": "...", "ports": [...] } ],
  "egress":  [ { "to":   "...", "ports": [...] } ],
  "default_action": "deny",
  "exception_log_required": true,
  "signature": "Ed25519 JCS"
}
```

`exception_log_required: true` is the default; any allow-list entry
emits a `policy_exception_log` envelope on first hit, so that
exceptions can be audited rather than silently expanding the blast
radius.

### `identity_hardening_attestation`

This envelope is published per-account on a rolling basis (≤24h):

- `mfa_factor_count` and `mfa_factor_kinds` (e.g., FIDO2, TOTP, SMS).
- `phishing_resistant_only` boolean.
- `last_credential_rotation_at`.
- `dormant_since` (null if active).
- `privilege_class` (read, write, admin, root).
- `breakglass` boolean (and accompanying `breakglass_review_at`).

The intent is that an organisation can answer "show me every privileged
account that does not have a phishing-resistant MFA factor" with a
single query against a stream of these attestations, without inventing
a new control framework.

### Supply-chain integrity (Phase 4 cross-reference to SLSA/SBOM)

The standard does not redefine SBOM (CycloneDX 1.6 / SPDX 3.0) or
SLSA provenance (v1.0); instead, `supply_chain_attestation` envelopes
carry references to those external artefacts so that an incident
responder can pivot from a detected ransomware binary to its
attested build provenance and known dependency graph in two hops.

## Backwards compatibility

Existing zero-trust deployments based on Kubernetes NetworkPolicy,
Istio AuthorizationPolicy, AWS SCP, or Cisco Secure Workload MAY
continue to author policy in their native form and emit
`segmentation_policy` envelopes as a *parallel* attestation generated
by a translator. This avoids forcing a single rewrite of policy
catalogues that span thousands of rules; the translator emits one
WIA envelope per native rule for cross-vendor visibility.

## Operational considerations

Honeyaccount and honeyfile fidelity ratings SHOULD be reviewed
quarterly. Adversary tradecraft evolves; assets that were high-fidelity
canaries two years ago may have become noise as benign tooling now
trips them. The standard provides a `fidelity_review` envelope to
record review outcomes and any fidelity reclassifications.

Segmentation policies MUST be tested under failure conditions
(controller outage, agent crash) at least annually; the resulting
`fail_closed_test` envelope is referenced by the next
`segmentation_posture` report.

Implementations SHOULD also record the fail-open vs fail-closed default
of each enforcement plane in the `segmentation_posture` envelope so that
auditors can distinguish a control that defaults to allow-on-failure
from one that defaults to deny-on-failure without re-reading vendor
documentation.
