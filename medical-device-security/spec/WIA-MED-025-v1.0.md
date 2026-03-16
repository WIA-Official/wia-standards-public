# WIA-MED-025: Medical Device Security Standard
## Version 1.0.0

**Status:** Published  
**Date:** 2025-01-26  
**Organization:** World Certification Industry Association (WIA)  
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Executive Summary

This standard provides comprehensive cybersecurity requirements for medical devices throughout their entire lifecycle, from design to decommissioning. It addresses FDA guidance, device-specific vulnerabilities, network protection, firmware management, testing, incident response, and legacy device handling.

### 1.1 Scope

This standard applies to:
- **Medical Device Manufacturers** designing, developing, and maintaining connected medical devices
- **Healthcare Delivery Organizations (HDOs)** operating medical device networks
- **Biomedical Engineering Departments** managing device security
- **IT/Security Teams** protecting healthcare infrastructure

### 1.2 Key Objectives

- Protect patient safety from cyber threats
- Ensure device integrity and availability
- Maintain confidentiality of patient health information
- Comply with FDA, HIPAA, and international regulations
- Establish incident response capabilities

---

## 2. FDA Cybersecurity Guidance Compliance

### 2.1 Premarket Requirements

**SBOM (Software Bill of Materials):**
- Comprehensive inventory of all software components
- Version numbers and known vulnerabilities
- License information
- Dependency mapping

**Threat Modeling:**
- STRIDE methodology application
- Attack surface analysis
- Risk assessment per ISO 14971
- Mitigation strategies

**Cybersecurity Risk Management:**
- Identification of cybersecurity risks
- Risk analysis (likelihood × impact)
- Risk controls implementation
- Residual risk acceptance

### 2.2 Postmarket Management

- Continuous vulnerability monitoring
- Coordinated Vulnerability Disclosure (CVD) program
- Patch management within defined timelines
- Incident reporting to FDA (MedWatch)

---

## 3. Device-Specific Security

### 3.1 Implantable Devices

**Pacemakers & ICDs:**
- Body-coupled communication (BCC) for anti-eavesdropping
- Zero-power authentication
- Physical proximity requirements for programming
- Emergency access protocols

**Insulin Pumps & CGM:**
- Bluetooth LE Secure Connections
- Multi-factor authentication for bolus delivery
- Anomaly detection (dosing patterns)
- Safety limits enforcement

### 3.2 Networked Devices

**Imaging Equipment (MRI, CT, X-ray):**
- Network segmentation (VLAN isolation)
- DICOM TLS encryption
- Access control lists (ACL)
- Regular vulnerability scanning

**Patient Monitors:**
- Read-only remote monitoring
- Tamper-evident logging
- Alarm integrity protection

---

## 4. Network Segmentation Architecture

### 4.1 VLAN Design

```
VLAN 20: Critical Medical Devices (life support)
  ├─ VLAN 21: Ventilators, Dialysis
  ├─ VLAN 22: Imaging (MRI, CT)
  └─ VLAN 23: Patient Monitoring

VLAN 30: Clinical Systems (EMR, PACS)
VLAN 40: Business Network
VLAN 50: Guest/Patient WiFi (isolated)
```

### 4.2 Firewall Policies

- Default deny all traffic
- Explicit allow rules (whitelist)
- Medical device → EMR: HTTPS (443), HL7 (2575)
- Medical device → Internet: DENY
- Inter-device communication: DENY (unless required)

### 4.3 Zero Trust Principles

- Never trust, always verify
- Least privilege access
- Assume breach
- Micro-segmentation
- Continuous monitoring

---

## 5. Firmware Update Security

### 5.1 Secure Boot

- Hardware root of trust (ROM bootloader)
- Chain of trust verification
- Digital signature validation
- Rollback prevention

### 5.2 Code Signing

**Minimum Requirements:**
- RSA-4096 or ECDSA-384
- Hardware Security Module (HSM) key storage
- Multi-party approval for signing
- Timestamp inclusion

### 5.3 OTA Update Process

1. Availability notification
2. User/admin approval
3. TLS 1.3 encrypted download
4. Signature verification
5. Battery level check (>50%)
6. A/B partition installation
7. Verification and rollback capability

---

## 6. Penetration Testing Requirements

### 6.1 Testing Frequency

- Annual full penetration test
- After major firmware updates
- Before FDA submission
- Following security incidents

### 6.2 Methodology

- OWASP Medical Device Testing Guide
- Network-based attacks
- Wireless protocol analysis
- Firmware reverse engineering
- Hardware hacking (with permission)

### 6.3 Ethical Guidelines

- Written authorization required
- Patient safety paramount
- Isolated test environment
- Responsible disclosure (90-day window)
- No actual patient data access

---

## 7. Incident Response Plan

### 7.1 CSIRT Composition

Required roles:
- Incident Response Manager
- Security Analyst
- Biomedical Engineer
- Network Engineer
- Clinical Representative
- Legal/Regulatory
- Communications

### 7.2 Response Phases

**Preparation:**
- Team training
- Playbook development
- Tool provisioning

**Detection & Analysis:**
- SIEM monitoring
- Severity classification (P1-P4)
- Patient safety impact assessment

**Containment, Eradication & Recovery:**
- Network isolation
- Malware removal
- System restoration
- Validation

**Post-Incident:**
- Lessons learned
- Process improvement
- Regulatory reporting

### 7.3 Regulatory Reporting

- HIPAA: 60 days for 500+ affected individuals
- FDA: Immediate for death/serious injury
- State AG: 30-90 days (varies by state)

---

## 8. Legacy Device Protection

### 8.1 Risk Assessment

**Risk Score = (Patient Impact × Breach Likelihood) + Data Sensitivity**

### 8.2 Compensating Controls

- Extreme VLAN restriction
- Virtual patching (IPS signatures)
- Application whitelisting
- Behavioral monitoring
- Read-only mode implementation

### 8.3 Replacement Strategy

**Prioritization:**
- Replacement Priority = (Risk Score × 2) + Business Impact - Replacement Difficulty

**Phases:**
- Phase 1 (Year 1): Critical (40+ points)
- Phase 2 (Year 2-3): High (30-39)
- Phase 3 (Year 4-5): Medium (20-29)
- Phase 4 (Year 6+): Low or continued compensating controls

---

## 9. Compliance Checklist

### 9.1 Manufacturers

☐ SBOM generated and maintained  
☐ Threat model documented  
☐ Secure boot implemented  
☐ Code signing with HSM  
☐ CVD program operational  
☐ Patch management process  
☐ FDA premarket submission complete  

### 9.2 Healthcare Organizations

☐ Medical device inventory  
☐ Network segmentation  
☐ Firewall policies reviewed  
☐ Penetration testing annually  
☐ CSIRT established  
☐ Incident response plan  
☐ Legacy device risk assessment  

---

## 10. References

- FDA (2023). Cybersecurity in Medical Devices: Quality System Considerations and Content of Premarket Submissions
- IEC 62443: Industrial Control Systems Security
- IEC 62304: Medical Device Software Lifecycle
- ISO 14971: Medical Device Risk Management
- AAMI TIR57: Medical Device Security Principles
- NIST Cybersecurity Framework
- OWASP Medical Device Security Testing Guide

---

## Appendix A: Glossary

**CIED:** Cardiac Implantable Electronic Device  
**CGM:** Continuous Glucose Monitoring  
**CSIRT:** Computer Security Incident Response Team  
**CVD:** Coordinated Vulnerability Disclosure  
**DICOM:** Digital Imaging and Communications in Medicine  
**HDO:** Healthcare Delivery Organization  
**HL7:** Health Level 7 (interoperability standard)  
**NAC:** Network Access Control  
**OTA:** Over-the-Air (firmware update)  
**PACS:** Picture Archiving and Communication System  
**PHI:** Protected Health Information  
**SBOM:** Software Bill of Materials  

---

## Appendix B: Contact Information

**WIA Standards Committee:**  
Email: standards@wiastandards.com  
Web: https://wiastandards.com  
GitHub: https://github.com/WIA-Official/wia-standards  

**Emergency Security Hotline:**  
+1-XXX-XXX-XXXX (24/7)

---

**Document Version History:**

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01-26 | Initial publication |

---

© 2025 World Certification Industry Association (WIA)  
Licensed under MIT License  
弘益人間 · Benefit All Humanity
