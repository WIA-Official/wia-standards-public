# WIA BCI Consent Protocol
## Phase 3: Protocol Workflows Specification

**Version:** 1.0.0  
**Status:** Release  
**Last Updated:** 2025-12-25

---

## 1. Introduction

This specification defines end-to-end workflows for consent management in BCI systems.

## 2. Initial Consent Workflow

### 2.1 Workflow Steps

1. **Capacity Assessment** - Evaluate ability to consent
2. **Information Provision** - Present risks, benefits, alternatives
3. **Understanding Verification** - Confirm comprehension
4. **Voluntary Agreement** - Ensure free choice
5. **Digital Signature** - Capture cryptographic signature
6. **Record Creation** - Generate and store consent record
7. **Confirmation** - Provide copy to subject

### 2.2 State Machine

```
[Initial] → [Capacity Assessment] → [Information Provided] 
→ [Understanding Verified] → [Agreement Obtained] 
→ [Signed] → [Active]
```

## 3. Ongoing Consent Workflow

### 3.1 Pre-Operation Verification

Before each BCI operation:
1. Verify consent exists and is active
2. Check required permissions granted
3. Confirm not expired
4. Log verification attempt

### 3.2 Periodic Renewal

```
30 days before expiration:
  - Send renewal reminder to subject
  - Provide updated information
  - Request renewed consent
  
Upon renewal:
  - Create new consent version
  - Link to previous version
  - Update expiration date
```

## 4. Modification Workflow

1. **Change Request** - Subject requests modification
2. **Impact Analysis** - Assess implications
3. **Additional Disclosure** - Explain changes
4. **Understanding Confirmation** - Verify comprehension
5. **Signature** - Capture signature on modifications
6. **Version Creation** - Create new version with tracking

## 5. Revocation Workflow

1. **Revocation Request** - Subject initiates
2. **Confirmation** - Verify intentional
3. **Data Processing Cessation** - Stop all processing immediately
4. **Data Deletion Timeline** - Schedule data deletion
5. **Notification** - Inform all parties
6. **Documentation** - Record revocation

### 5.1 Data Deletion Schedule

| Data Type | Timeline |
|-----------|----------|
| Real-time data | Immediate |
| Cached data | 24 hours |
| Analytical data | 30 days |
| Backup data | 90 days |
| Anonymized aggregates | Retained |
| Regulatory submissions | Per requirements |

## 6. Emergency Override Protocol

### 6.1 Conditions

- Immediate threat to life or serious harm
- Consent impossible to obtain
- Delay would increase risk
- Within standard of care

### 6.2 Process

1. **Emergency Determination** - Physician declares emergency
2. **Authorization** - Licensed physician authorizes
3. **Documentation** - Record justification
4. **Time Limit** - 24-48 hours maximum
5. **Ethics Review** - Submit within 48 hours
6. **Transition** - Obtain proper consent when able

---

**弘益人間 (Hongik Ingan)** - *Benefit All Humanity*

© 2025 SmileStory Inc. / WIA
