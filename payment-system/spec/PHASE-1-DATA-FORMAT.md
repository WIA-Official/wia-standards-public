# WIA-FIN-012 Phase 1: Data Format Specification

**Version:** 1.0.0  
**Status:** Final  
**Date:** 2025-01-15  
**Category:** Finance - Payment Systems

---

## 1. Introduction

This specification defines the data formats, structures, and validation rules for payment card data in the WIA-FIN-012 Payment System Standard. It covers card data representation, encryption standards, tokenization formats, and PCI-DSS compliance requirements.

### 1.1 Scope

This phase covers:
- Primary Account Number (PAN) format and validation
- Card metadata (expiry, CVV, cardholder name)
- Magnetic stripe data structures
- EMV chip data formats
- ISO 8583 message structure
- Data encryption and tokenization
- PCI-DSS data classification

### 1.2 Normative References

- ISO/IEC 7812: Identification cards - Identification of issuers
- ISO/IEC 7813: Identification cards - Financial transaction cards
- ISO 8583: Financial transaction card originated messages
- EMV 4.3: Integrated Circuit Card Specifications for Payment Systems
- PCI-DSS 4.0: Payment Card Industry Data Security Standard

---

## 2. Primary Account Number (PAN)

### 2.1 Structure

```
PAN Structure (16 digits typical):
┌─────────┬──────────────────┬────────┐
│  1-6    │      7-15        │   16   │
│  BIN    │  Account Number  │ Check  │
│  (IIN)  │                  │ Digit  │
└─────────┴──────────────────┴────────┘

Detailed Breakdown:
Position 1:      Major Industry Identifier (MII)
Positions 1-6:   Bank Identification Number (BIN) / Issuer Identification Number (IIN)
Positions 7-15:  Individual Account Identifier
Position 16:     Check Digit (Luhn algorithm)
```

### 2.2 Length Specifications

| Card Network | PAN Length | Format |
|-------------|-----------|---------|
| Visa | 13, 16, 19 | 4xxx xxxx xxxx xxx(x)(x) |
| Mastercard | 16 | 5xxx xxxx xxxx xxxx |
| American Express | 15 | 3xxx xxxxxx xxxxx |
| Discover | 16 | 6xxx xxxx xxxx xxxx |
| Diners Club | 14 | 3xxx xxxx xxxx xx |
| JCB | 16 | 35xx xxxx xxxx xxxx |
| UnionPay | 16-19 | 62xx xxxx xxxx xxxx (xxx) |

### 2.3 Luhn Algorithm

**Purpose:** Detect accidental errors in PAN entry

**Algorithm:**
```python
def luhn_check(pan: str) -> bool:
    """
    Validates PAN using Luhn algorithm (modulus 10)
    
    Steps:
    1. Remove all non-digit characters
    2. Reverse the digit string
    3. Double every second digit (starting from index 1)
    4. If doubled digit > 9, subtract 9
    5. Sum all digits
    6. If sum % 10 == 0, valid
    """
    digits = [int(d) for d in pan if d.isdigit()][::-1]
    checksum = sum(
        d if i % 2 == 0 else (d * 2 - 9 if d * 2 > 9 else d * 2)
        for i, d in enumerate(digits)
    )
    return checksum % 10 == 0

# Example:
# PAN: 4532 1234 5678 9010
# Reversed: 0109 8765 4321 2354
# Process: 0+1+0+18+8+14+6+10+4+6+2+2+2+6+5+8 = 92
# 92 % 10 = 2 (if this were 0, it would be valid)
```

**Test Card Numbers (Always Valid):**
```
Visa:              4532 1234 5678 9010
Visa:              4916 3385 0643 3430
Mastercard:        5425 2334 3010 9903
American Express:  3782 822463 10005
Discover:          6011 1111 1111 1117
```

---

## 3. Card Metadata

### 3.1 Expiration Date

**Format:** `YYMM` or `MM/YY`

```
Storage Format:   YYMM (4 digits)
Display Format:   MM/YY
Example:          2512 (December 2025)
Display:          12/25

Validation Rules:
- MM must be 01-12
- YY must be current year or future
- Card invalid if current date > last day of expiry month
```

### 3.2 Card Verification Value (CVV/CVC/CID)

| Card Network | Name | Location | Length |
|-------------|------|----------|---------|
| Visa | CVV2 | Back | 3 digits |
| Mastercard | CVC2 | Back | 3 digits |
| American Express | CID | Front | 4 digits |
| Discover | CID | Back | 3 digits |

**Security Rules:**
```
✅ ALLOWED:
- Collect for authorization
- Transmit encrypted for processing

❌ NEVER:
- Store after authorization (even encrypted)
- Log in plain text
- Display to unauthorized parties
- Include in receipts or statements

PENALTY: Up to $500,000 per incident + decertification
```

### 3.3 Cardholder Name

**Format:**
```
Pattern: [A-Z ]{2,26}
Max Length: 26 characters (embossing limitation)
Case: UPPERCASE (traditional embossing)
Separator: Space or slash (/)

Examples:
JOHN DOE
MARY JANE SMITH
O'BRIEN/PATRICK

Validation:
- Minimum 2 characters
- Maximum 26 characters
- Letters and spaces only (some systems allow apostrophe, hyphen)
- No numbers or special characters
```

---

## 4. Magnetic Stripe Data

### 4.1 Track 1 Specification

**Format Code:** ALPHA (alphanumeric)  
**Density:** 210 bits per inch (bpi)  
**Capacity:** 79 characters maximum

```
Structure:
%B{PAN}^{Name}^{Expiry}{ServiceCode}{DiscretionaryData}?{LRC}

Example:
%B4532123456789010^DOE/JOHN^25121011234567890123456789?

Field Breakdown:
%               Start Sentinel
B               Format Code (B = ALPHA)
4532...9010     Primary Account Number
^               Field Separator
DOE/JOHN        Cardholder Name (surname/firstname)
^               Field Separator
2512            Expiration Date (YYMM)
101             Service Code (3 digits)
1234...789      Discretionary Data (optional, max 28 chars)
?               End Sentinel
LRC             Longitudinal Redundancy Check
```

### 4.2 Track 2 Specification

**Format Code:** NUMERIC  
**Density:** 75 bits per inch (bpi)  
**Capacity:** 40 characters maximum

```
Structure:
;{PAN}={Expiry}{ServiceCode}{DiscretionaryData}?{LRC}

Example:
;4532123456789010=25121011234?

Field Breakdown:
;               Start Sentinel
4532...9010     Primary Account Number
=               Field Separator
2512            Expiration Date (YYMM)
101             Service Code
1234            Discretionary Data (max 13 chars)
?               End Sentinel
LRC             Longitudinal Redundancy Check

Note: Track 2 is most commonly used for payment transactions
```

### 4.3 Service Code

**3-Digit Code Structure:**

**First Digit (Interchange & Technology):**
```
1 = International, ICC (chip)
2 = International, magnetic stripe
5 = National, ICC
6 = National, magnetic stripe
7 = Private use
```

**Second Digit (Authorization Processing):**
```
0 = Normal authorization, no restrictions
2 = Online authorization required
3 = Offline PIN required (if supported)
5 = Offline PIN required
6 = Prompt for PIN
```

**Third Digit (Allowed Services):**
```
0 = No restrictions, PIN required if indicated
1 = Goods and services only (no cash)
2 = ATM only
3 = Goods and services, authorized ATM
4 = Cash only
5 = Goods and services (no cash), PIN required
6 = No restrictions, PIN required
7 = Goods and services, PIN required
```

**Common Service Codes:**
```
101 = International, no restrictions, no PIN (most credit cards)
121 = International, no restrictions, online PIN (debit cards)
201 = International, mag stripe, no restrictions
221 = International, mag stripe, PIN required
```

---

## 5. EMV Chip Data

### 5.1 TLV (Tag-Length-Value) Structure

```
TLV Encoding:
┌──────┬────────┬───────────────┐
│ Tag  │ Length │     Value     │
│ 1-2  │  1-3   │   Variable    │
│ bytes│ bytes  │     bytes     │
└──────┴────────┴───────────────┘

Example:
5A 08 4532123456789010
│  │  └─ Value (8 bytes): PAN
│  └─── Length (1 byte): 8
└────── Tag (1 byte): Application PAN (0x5A)
```

### 5.2 Critical EMV Tags

```
Card Data Tags:
├─ 5A         Application PAN
├─ 5F20       Cardholder Name
├─ 5F24       Application Expiration Date (YYMMDD)
├─ 5F25       Application Effective Date
├─ 5F28       Issuer Country Code (ISO 3166-1 numeric)
├─ 5F34       PAN Sequence Number
└─ 57         Track 2 Equivalent Data

Application Tags:
├─ 4F         Application Identifier (AID)
├─ 50         Application Label
├─ 82         Application Interchange Profile (AIP)
├─ 84         Dedicated File (DF) Name
├─ 87         Application Priority Indicator
├─ 8C         Card Risk Management Data Object List 1 (CDOL1)
├─ 8D         Card Risk Management Data Object List 2 (CDOL2)
└─ 8E         Cardholder Verification Method (CVM) List

Transaction Tags:
├─ 9A         Transaction Date (YYMMDD)
├─ 9C         Transaction Type
├─ 9F02       Amount, Authorized (Numeric)
├─ 9F03       Amount, Other (Numeric)
├─ 9F1A       Terminal Country Code
└─ 9F21       Transaction Time (HHMMSS)

Cryptographic Tags:
├─ 9F26       Application Cryptogram (ARQC/TC/AAC)
├─ 9F27       Cryptogram Information Data
├─ 9F34       CVM Results
├─ 9F36       Application Transaction Counter (ATC)
└─ 9F37       Unpredictable Number (Terminal Random)
```

### 5.3 Application Identifiers (AID)

```
Registered Application Identifiers (RID):

Visa:
A0000000031010    Visa Credit/Debit
A0000000032010    Visa Electron  
A0000000033010    Visa Interlink (V PAY)
A0000000038010    Visa Plus

Mastercard:
A0000000041010    Mastercard Credit/Debit
A0000000042010    Maestro
A0000000043010    Cirrus

American Express:
A00000002501      American Express

Discover:
A0000001523010    Discover

JCB:
A0000000651010    JCB

UnionPay:
A000000333010101  UnionPay Credit
A000000333010102  UnionPay Debit
A000000333010103  UnionPay Electronic Cash
```

---

## 6. ISO 8583 Data Elements

### 6.1 Message Type Indicator (MTI)

```
4-Digit MTI Structure:
┌────────┬────────┬────────┬────────┐
│Version │ Class  │Function│ Origin │
│  0-9   │  0-9   │  0-9   │  0-9   │
└────────┴────────┴────────┴────────┘

Version:
0 = ISO 8583-1:1987
1 = ISO 8583-1:1993
2 = ISO 8583-1:2003
9 = Private use

Class:
0 = Reserved
1 = Authorization
2 = Financial
3 = File actions
4 = Reversal/Chargeback
5 = Reconciliation
6 = Administrative
7 = Fee collection
8 = Network management
9 = Reserved

Function:
0 = Request
1 = Request response
2 = Advice
3 = Advice response
4 = Notification
5-9 = Reserved

Origin:
0 = Acquirer
1 = Acquirer repeat
2 = Issuer
3 = Issuer repeat
4 = Other
5 = Other repeat
6-9 = Reserved

Common MTIs:
0100 = Authorization Request (Acquirer → Issuer)
0110 = Authorization Response (Issuer → Acquirer)
0200 = Financial Transaction Request
0210 = Financial Transaction Response
0220 = Financial Advice
0400 = Reversal Request
0420 = Reversal Advice
0800 = Network Management Request
0810 = Network Management Response
```

### 6.2 Common Data Elements

```
Field | Name | Type | Length | Description
------|------|------|--------|-------------
1     | Bitmap | b | 8/16 | Secondary bitmap indicator
2     | PAN | n | ≤19 | Primary Account Number
3     | Processing Code | n | 6 | Transaction type
4     | Amount, Transaction | n | 12 | Transaction amount
7     | Transmission Date/Time | n | 10 | MMDDhhmmss
11    | STAN | n | 6 | Systems Trace Audit Number
12    | Time, Local Transaction | n | 6 | hhmmss
13    | Date, Local Transaction | n | 4 | MMDD
14    | Expiration Date | n | 4 | YYMM
18    | Merchant Type | n | 4 | MCC Code
22    | POS Entry Mode | n | 3 | Card data entry method
23    | Card Sequence Number | n | 3 | For chip cards
25    | POS Condition Code | n | 2 | Transaction environment
32    | Acquiring Institution ID | n | ≤11 | Acquirer BIN
35    | Track 2 Data | z | ≤37 | Track 2 equivalent
37    | Retrieval Reference | an | 12 | Unique reference
38    | Authorization Code | an | 6 | Approval code
39    | Response Code | an | 2 | Transaction result
41    | Terminal ID | ans | 8 | Terminal identifier
42    | Merchant ID | ans | 15 | Card acceptor ID
43    | Merchant Name/Location | ans | 40 | Merchant details
49    | Currency Code | n | 3 | ISO 4217 numeric
52    | PIN Data | b | 8 | Encrypted PIN block
54    | Additional Amounts | an | ≤120 | Cashback, etc.
55    | ICC Data | b | ≤255 | EMV chip data
90    | Original Data Elements | n | 42 | For reversals
95    | Replacement Amounts | an | 42 | Amended amounts

Type Codes:
n   = Numeric only
an  = Alphanumeric
ans = Alphanumeric + special
b   = Binary
z   = Track 2/3 (numeric + = ?)
```

---

## 7. Data Encryption

### 7.1 Encryption Standards

**At Rest:**
```
Required:
- AES-256-GCM (recommended)
- AES-128-GCM (minimum)
- RSA-2048 (for key encryption)

Prohibited:
- DES
- 3DES/TDEA (deprecated 2023)
- RSA-1024 or lower
- RC4
- MD5
- SHA-1
```

**In Transit:**
```
TLS Requirements:
✓ TLS 1.3 (recommended)
✓ TLS 1.2 (minimum)
✗ TLS 1.1 (prohibited)
✗ TLS 1.0 (prohibited)
✗ SSL 3.0 (prohibited)
✗ SSL 2.0 (prohibited)

Cipher Suites (Strong):
- TLS_AES_256_GCM_SHA384
- TLS_CHACHA20_POLY1305_SHA256
- TLS_AES_128_GCM_SHA256
- ECDHE-RSA-AES256-GCM-SHA384
- ECDHE-RSA-AES128-GCM-SHA256
```

### 7.2 PAN Encryption

```javascript
// Example: AES-256-GCM Encryption
const crypto = require('crypto');

function encryptPAN(pan, key) {
  const algorithm = 'aes-256-gcm';
  const iv = crypto.randomBytes(12); // 96-bit IV for GCM
  const cipher = crypto.createCipheriv(algorithm, key, iv);
  
  let encrypted = cipher.update(pan, 'utf8', 'hex');
  encrypted += cipher.final('hex');
  
  const authTag = cipher.getAuthTag();
  
  return {
    encrypted: encrypted,
    iv: iv.toString('hex'),
    authTag: authTag.toString('hex')
  };
}

function decryptPAN(encryptedData, key) {
  const algorithm = 'aes-256-gcm';
  const decipher = crypto.createDecipheriv(
    algorithm,
    key,
    Buffer.from(encryptedData.iv, 'hex')
  );
  
  decipher.setAuthTag(Buffer.from(encryptedData.authTag, 'hex'));
  
  let decrypted = decipher.update(encryptedData.encrypted, 'hex', 'utf8');
  decrypted += decipher.final('utf8');
  
  return decrypted;
}
```

---

## 8. Tokenization

### 8.1 Token Format

**Format-Preserving Tokenization:**
```
Real PAN:  4532 1234 5678 9010
Token:     4532 9876 5432 1098
           ↑↑↑↑               ↑
           │││└─ Preserved for BIN routing
           │││
           └──────────────── Maintains Luhn validity

Characteristics:
✓ Same length as PAN
✓ Passes Luhn check
✓ First 6 (BIN) preserved for routing
✓ Last 4 may be preserved for display
✓ Cannot be reverse-engineered
```

**Non-Format-Preserving:**
```
Real PAN:  4532 1234 5678 9010
Token:     tok_7h3k2m9p4x1q8n5w

Characteristics:
✓ Different format from PAN
✓ Clear distinction from PAN
✓ Flexible structure
✓ Cannot be mistaken for real PAN
```

### 8.2 Token Lifecycle

```
1. Token Generation:
   PAN → Token Vault → Token
   - Store PAN securely encrypted
   - Generate cryptographically random token
   - Create bidirectional mapping
   
2. Token Usage:
   Merchant → Token → Payment Gateway
   - Merchant stores/transmits token only
   - Token vault detokenizes for processing
   - Real PAN sent to card network
   
3. Token Expiration:
   - Default: Tied to card expiry
   - Custom: 30/60/90 days
   - Revocation: Immediate on request
   
4. Token Security:
   - Tokens are single-domain (cannot use elsewhere)
   - Rate limiting on detokenization
   - Audit logging of all token operations
   - Automatic token rotation
```

---

## 9. PCI-DSS Data Classification

### 9.1 Cardholder Data (CHD)

**Primary Account Number (PAN):**
```
Status: SENSITIVE - Highest Protection Level

Storage Rules:
✓ Encrypted with AES-256 if stored
✓ Tokenized preferred over storage
✓ Masked when displayed (show last 4 only)
✗ Never in plain text logs
✗ Never in emails/SMS
✗ Minimize retention period

Masking Format:
Full PAN:     4532 1234 5678 9010
Displayed:    •••• •••• •••• 9010
Logs:         PAN ending 9010
Receipts:     ************9010
```

**Cardholder Name:**
```
Status: Moderately Sensitive

Rules:
✓ May store if business need
✓ Encrypt if stored with PAN
✓ Access controls required
✓ Include in data retention policy
```

**Expiration Date:**
```
Status: Moderately Sensitive

Rules:
✓ Required for recurring billing
✓ Encrypt if stored with PAN
✓ Include in security policies
```

**Service Code:**
```
Status: Moderately Sensitive

Rules:
✓ May store for fraud checks
✓ Encrypt if stored with PAN
✓ Not required post-authorization
```

### 9.2 Sensitive Authentication Data (SAD)

**NEVER STORE AFTER AUTHORIZATION:**

**Full Magnetic Stripe:**
```
Status: FORBIDDEN TO STORE
❌ NEVER store (not even encrypted)
❌ Purge immediately after authorization
✓ Use for authorization only
Penalty: Up to $500,000 + decertification
```

**CVV/CVC/CVV2/CID:**
```
Status: FORBIDDEN TO STORE
❌ NEVER store (not even encrypted)
❌ No logging allowed
✓ Collect for authorization only
✓ Transmitted encrypted
Penalty: Immediate PCI non-compliance
```

**PIN/PIN Block:**
```
Status: FORBIDDEN TO STORE
❌ NEVER store in any form
✓ Process only in HSM
✓ Triple-DES encryption minimum
✓ No clear-text PIN ever
Penalty: Criminal liability possible
```

---

## 10. Validation Rules

### 10.1 PAN Validation

```typescript
interface PANValidation {
  isLuhnValid: boolean;
  cardNetwork: string;
  isLengthValid: boolean;
  isFormatValid: boolean;
}

function validatePAN(pan: string): PANValidation {
  // Remove spaces and non-digits
  const cleanPAN = pan.replace(/\D/g, '');
  
  // Luhn check
  const luhnValid = luhnCheck(cleanPAN);
  
  // Detect network
  const network = detectCardNetwork(cleanPAN);
  
  // Length validation
  const expectedLengths = {
    'visa': [13, 16, 19],
    'mastercard': [16],
    'amex': [15],
    'discover': [16],
    'diners': [14],
    'jcb': [16]
  };
  
  const lengthValid = network && 
    expectedLengths[network]?.includes(cleanPAN.length);
  
  // Format validation
  const formatValid = /^\d+$/.test(cleanPAN);
  
  return {
    isLuhnValid: luhnValid,
    cardNetwork: network,
    isLengthValid: lengthValid,
    isFormatValid: formatValid
  };
}
```

### 10.2 Expiry Validation

```typescript
function validateExpiry(expiry: string): boolean {
  // Format: YYMM or MM/YY
  const match = expiry.match(/^(\d{2})[\/\-]?(\d{2})$/);
  if (!match) return false;
  
  const [_, mmOrYY, yyOrMM] = match;
  
  // Determine format
  let month, year;
  if (parseInt(mmOrYY) > 12) {
    year = parseInt(mmOrYY);
    month = parseInt(yyOrMM);
  } else {
    month = parseInt(mmOrYY);
    year = parseInt(yyOrMM);
  }
  
  // Validate month
  if (month < 1 || month > 12) return false;
  
  // Convert 2-digit year to 4-digit
  const fullYear = year < 100 ? 2000 + year : year;
  
  // Check not expired
  const now = new Date();
  const expiryDate = new Date(fullYear, month, 0); // Last day of month
  
  return expiryDate >= now;
}
```

---

## 11. Implementation Requirements

### 11.1 Mandatory

- Luhn algorithm validation for all PAN inputs
- PAN masking in all displays and logs
- TLS 1.2+ for all data transmission
- AES-256 for PAN encryption if stored
- CVV/CVC never stored after authorization
- Magnetic stripe purged after authorization
- PIN processing only in HSM

### 11.2 Recommended

- Tokenization for recurring payments
- Format-preserving encryption
- Network tokenization integration
- Real-time PAN validation
- Automated key rotation
- Comprehensive audit logging

### 11.3 Optional

- Homomorphic encryption for analytics
- Blockchain-based token registry
- AI-powered fraud detection
- Biometric cardholder verification

---

## 12. Compliance Checklist

- [ ] PAN encryption at rest (AES-256)
- [ ] PAN transmission over TLS 1.2+
- [ ] PAN masking in all interfaces
- [ ] Luhn validation implemented
- [ ] CVV never stored
- [ ] Magnetic stripe never stored
- [ ] PIN processing in HSM only
- [ ] Tokenization system operational
- [ ] Key management procedures
- [ ] Data retention policies
- [ ] Audit logging enabled
- [ ] Regular security testing
- [ ] PCI-DSS SAQ completed
- [ ] Incident response plan
- [ ] Staff security training

---

**弘益人間 (Hongik Ingan) - Benefit All Humanity**

*Secure data formats are the foundation of trustworthy payment systems. By properly handling sensitive payment data, we protect consumers and enable global commerce.*

---

**Document Control:**
- Version: 1.0.0
- Last Updated: 2025-01-15
- Next Review: 2026-01-15
- Maintained by: WIA Payment Standards Committee
- Contact: standards@wia.org

