# WIA-IND-019 — Phase 3: Protocol

> Ticketing-system canonical Phase 3: protocols (fraud + transfer + access control + refund).

# WIA-IND-019: Ticketing System Standard - Complete Specification

> **Standard ID:** WIA-IND-019
> **Version:** 1.0.0
> **Status:** Active
> **Category:** IND (Industry)
> **Last Updated:** 2025-12-27
> **Authors:** WIA Industry Standards Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Scope](#2-scope)
3. [Normative References](#3-normative-references)
4. [Terms and Definitions](#4-terms-and-definitions)
5. [Ticket Data Format](#5-ticket-data-format)
6. [QR Code and Barcode Standards](#6-qr-code-and-barcode-standards)
7. [Seat Reservation System](#7-seat-reservation-system)
8. [Dynamic Pricing Algorithms](#8-dynamic-pricing-algorithms)
9. [Fraud Prevention](#9-fraud-prevention)
10. [Ticket Transfer and Resale](#10-ticket-transfer-and-resale)
11. [Multi-Venue Support](#11-multi-venue-support)
12. [Season Pass Management](#12-season-pass-management)
13. [Access Control Integration](#13-access-control-integration)
14. [API Specification](#14-api-specification)
15. [Security Requirements](#15-security-requirements)
16. [Privacy and Data Protection](#16-privacy-and-data-protection)
17. [Interoperability](#17-interoperability)
18. [Testing and Certification](#18-testing-and-certification)
19. [Appendices](#19-appendices)

---


## 9. Fraud Prevention

### 9.1 Multi-Layer Security

#### 9.1.1 Security Layers

1. **QR Code Encryption**: AES-256-GCM
2. **Digital Signature**: ECDSA (secp256k1)
3. **Blockchain Verification**: Ethereum/Polygon
4. **Biometric Binding**: Face ID / Fingerprint
5. **Geofencing**: GPS location validation
6. **TOTP Rotation**: 30-second code refresh
7. **Device Fingerprinting**: Browser/app identification
8. **AI Anomaly Detection**: Behavioral analysis

#### 9.1.2 Duplicate Detection

Real-time duplicate scan prevention:

```typescript
async function validateTicket(validation: TicketValidation): Promise<ValidationResult> {
  const { ticketId, qrCode, location, timestamp } = validation;

  // Check if already used
  const checkInRecord = await getCheckInHistory(ticketId);
  if (checkInRecord.status === 'checked-in') {
    return {
      isValid: false,
      error: 'DUPLICATE_SCAN',
      message: 'Ticket already used',
      originalCheckIn: checkInRecord.timestamp
    };
  }

  // Verify QR signature
  const qrValid = await verifyQRSignature(qrCode);
  if (!qrValid) {
    return {
      isValid: false,
      error: 'INVALID_QR',
      message: 'QR code verification failed'
    };
  }

  // Check geofence
  const withinVenue = await checkGeofence(location, validation.venueId);
  if (!withinVenue) {
    return {
      isValid: false,
      error: 'LOCATION_MISMATCH',
      message: 'Not within venue geofence'
    };
  }

  // Verify TOTP
  const totpValid = verifyTOTP(validation.totp, timestamp);
  if (!totpValid) {
    return {
      isValid: false,
      error: 'TOTP_EXPIRED',
      message: 'Time-based code expired'
    };
  }

  // Blockchain verification
  const blockchainValid = await verifyBlockchain(ticketId);
  if (!blockchainValid) {
    return {
      isValid: false,
      error: 'BLOCKCHAIN_INVALID',
      message: 'Blockchain ownership verification failed'
    };
  }

  // Mark as used
  await recordCheckIn(ticketId, location, timestamp);

  return {
    isValid: true,
    message: 'Ticket validated successfully',
    holder: await getTicketHolder(ticketId)
  };
}
```

### 9.2 Anti-Scalping Measures

#### 9.2.1 Purchase Limits

```typescript
interface PurchaseLimits {
  maxPerUser: number;          // e.g., 6 tickets
  maxPerTransaction: number;   // e.g., 10 tickets
  maxPerIP: number;            // e.g., 20 tickets
  maxPerCard: number;          // e.g., 12 tickets
  cooldownPeriod: number;      // seconds between purchases
}
```

#### 9.2.2 Bot Detection

```typescript
async function detectBot(request: PurchaseRequest): Promise<BotScore> {
  const signals = {
    // Request characteristics
    userAgent: analyzeUserAgent(request.userAgent),
    ipReputation: await checkIPReputation(request.ip),
    browserFingerprint: request.fingerprint,

    // Behavioral signals
    mouseMovements: request.mouseData?.entropy || 0,
    keystrokeDynamics: request.keystrokes?.variance || 0,
    pageNavigation: request.navigationPattern,

    // Time-based signals
    requestSpeed: calculateRequestSpeed(request),
    formFillTime: request.formCompletionTime
  };

  // ML model prediction
  const botProbability = await mlModel.predict(signals);

  return {
    probability: botProbability,
    signals,
    action: botProbability > 0.7 ? 'block' :
            botProbability > 0.4 ? 'challenge' :
            'allow'
  };
}
```

#### 9.2.3 CAPTCHA Integration

```typescript
if (botScore.action === 'challenge') {
  const captchaResult = await verifyCaptcha({
    token: request.captchaToken,
    action: 'ticket_purchase',
    expectedHostname: config.hostname
  });

  if (captchaResult.score < 0.5) {
    throw new Error('CAPTCHA verification failed');
  }
}
```

### 9.3 Blockchain Integration

#### 9.3.1 NFT Ticket Minting

```solidity
// Smart contract (Solidity)
pragma solidity ^0.8.0;

import "@openzeppelin/contracts/token/ERC721/ERC721.sol";

contract WIATicketNFT is ERC721 {
    struct Ticket {
        string ticketId;
        string eventId;
        uint256 expiryTime;
        bool used;
    }

    mapping(uint256 => Ticket) public tickets;
    uint256 private _tokenIdCounter;

    function mintTicket(
        address holder,
        string memory ticketId,
        string memory eventId,
        uint256 expiryTime
    ) public returns (uint256) {
        uint256 tokenId = _tokenIdCounter++;
        _safeMint(holder, tokenId);

        tickets[tokenId] = Ticket({
            ticketId: ticketId,
            eventId: eventId,
            expiryTime: expiryTime,
            used: false
        });

        return tokenId;
    }

    function validateTicket(uint256 tokenId) public returns (bool) {
        require(_exists(tokenId), "Ticket does not exist");
        require(block.timestamp < tickets[tokenId].expiryTime, "Ticket expired");
        require(!tickets[tokenId].used, "Ticket already used");

        tickets[tokenId].used = true;
        return true;
    }
}
```

#### 9.3.2 Blockchain Verification

```typescript
async function verifyBlockchain(ticketId: string): Promise<boolean> {
  const ticket = await getTicket(ticketId);
  const { hash, network, contractAddress, tokenId } = ticket.security.blockchain;

  // Connect to blockchain
  const provider = getProvider(network);
  const contract = new ethers.Contract(contractAddress, ABI, provider);

  // Verify ownership
  const owner = await contract.ownerOf(tokenId);
  const expectedOwner = await getHolderWalletAddress(ticket.holder.email);

  if (owner.toLowerCase() !== expectedOwner.toLowerCase()) {
    return false;
  }

  // Verify not used
  const ticketData = await contract.tickets(tokenId);
  if (ticketData.used) {
    return false;
  }

  return true;
}
```

---



## 10. Ticket Transfer and Resale

### 10.1 Transfer Mechanism

#### 10.1.1 Initiate Transfer

```typescript
async function transferTicket(transfer: TicketTransfer): Promise<TransferResult> {
  const {
    ticketId,
    fromEmail,
    toEmail,
    requireApproval,
    message
  } = transfer;

  // Verify ownership
  const ticket = await getTicket(ticketId);
  if (ticket.holder.email !== fromEmail) {
    throw new Error('Not ticket owner');
  }

  // Check if transferable
  if (!ticket.transferable) {
    throw new Error('Ticket is non-transferable');
  }

  // Create transfer request
  const transferRequest = await createTransferRequest({
    ticketId,
    from: fromEmail,
    to: toEmail,
    status: requireApproval ? 'pending' : 'auto-approved',
    createdAt: new Date(),
    message
  });

  // Send notifications
  await sendTransferNotification(toEmail, transferRequest);

  if (!requireApproval) {
    await executeTransfer(transferRequest.id);
  }

  return {
    success: true,
    transferId: transferRequest.id,
    status: transferRequest.status
  };
}
```

#### 10.1.2 Accept Transfer

```typescript
async function acceptTransfer(transferId: string, recipientEmail: string): Promise<void> {
  const transfer = await getTransferRequest(transferId);

  if (transfer.to !== recipientEmail) {
    throw new Error('Not the intended recipient');
  }

  if (transfer.status !== 'pending') {
    throw new Error('Transfer not pending');
  }

  await executeTransfer(transferId);
}

async function executeTransfer(transferId: string): Promise<void> {
  const transfer = await getTransferRequest(transferId);
  const ticket = await getTicket(transfer.ticketId);

  // Update ticket holder
  ticket.holder.email = transfer.to;
  ticket.holder.name = transfer.toName;
  ticket.holder.verified = false; // Require re-verification
  ticket.status = 'transferred';

  // Update blockchain ownership (if NFT)
  if (ticket.security.blockchain) {
    await transferNFT(
      ticket.security.blockchain.tokenId,
      transfer.from,
      transfer.to
    );
  }

  // Regenerate QR code
  ticket.security.qrCode = await generateNewQRCode(ticket);

  await updateTicket(ticket);
  await updateTransferStatus(transferId, 'completed');

  // Notify both parties
  await sendTransferCompletedNotification(transfer);
}
```

### 10.2 Resale Marketplace

#### 10.2.1 List for Resale

```typescript
async function listForResale(listing: ResaleListing): Promise<ListingResult> {
  const {
    ticketId,
    askingPrice,
    sellerId,
    expiryDate
  } = listing;

  const ticket = await getTicket(ticketId);

  // Verify resellable
  if (!ticket.resellable) {
    throw new Error('Ticket is non-resellable');
  }

  // Check price constraints
  const rules = await getResaleRules(ticket.eventId);
  if (askingPrice > ticket.pricing.finalPrice * rules.maxMarkup) {
    throw new Error(`Price exceeds maximum markup of ${rules.maxMarkup * 100}%`);
  }

  if (askingPrice < ticket.pricing.finalPrice * rules.minPriceRatio) {
    throw new Error(`Price below minimum of ${rules.minPriceRatio * 100}%`);
  }

  // Create listing
  const marketplaceListing = await createListing({
    ticketId,
    sellerId,
    askingPrice,
    originalPrice: ticket.pricing.finalPrice,
    listedAt: new Date(),
    expiryDate,
    status: 'active',
    transferFee: rules.transferFee
  });

  // Lock ticket
  await lockTicket(ticketId, 'resale-listing');

  return {
    success: true,
    listingId: marketplaceListing.id,
    url: `https://marketplace.wia.com/listing/${marketplaceListing.id}`
  };
}
```

#### 10.2.2 Purchase from Resale

```typescript
async function purchaseResaleTicket(
  listingId: string,
  buyerId: string
): Promise<PurchaseResult> {
  const listing = await getListing(listingId);

  if (listing.status !== 'active') {
    throw new Error('Listing not active');
  }

  // Process payment
  const payment = await processPayment({
    amount: listing.askingPrice + listing.transferFee,
    buyerId,
    sellerId: listing.sellerId,
    description: `Resale ticket ${listing.ticketId}`
  });

  if (!payment.success) {
    throw new Error('Payment failed');
  }

  // Transfer ticket
  const transfer = await transferTicket({
    ticketId: listing.ticketId,
    fromEmail: listing.sellerEmail,
    toEmail: listing.buyerEmail,
    requireApproval: false
  });

  // Update listing
  await updateListing(listingId, {
    status: 'sold',
    soldAt: new Date(),
    buyerId,
    finalPrice: listing.askingPrice
  });

  // Distribute payment
  await distributePayment({
    seller: listing.askingPrice,
    platform: listing.transferFee,
    originalIssuer: listing.transferFee * 0.1 // 10% to original issuer
  });

  return {
    success: true,
    ticketId: listing.ticketId,
    transferId: transfer.transferId,
    amountPaid: listing.askingPrice + listing.transferFee
  };
}
```

#### 10.2.3 Anti-Scalping Rules

```typescript
interface ResaleRules {
  enabled: boolean;
  maxMarkup: number;        // e.g., 1.2 (120% of face value)
  minPriceRatio: number;    // e.g., 0.8 (80% of face value)
  transferFee: number;      // e.g., 5.00 USD
  maxListingsPerUser: number; // e.g., 4 tickets
  verificationRequired: boolean;
  cooldownPeriod: number;   // seconds before relist
}
```

---



## 13. Access Control Integration

### 13.1 Supported Access Control Systems

- **Turnstiles**: Automatic gate control
- **Handheld Scanners**: Mobile validation devices
- **NFC Readers**: Contactless validation
- **Biometric Kiosks**: Fingerprint/face recognition
- **Mobile Apps**: Self-service validation
- **QR Scanners**: Camera-based validation

### 13.2 Validation Protocol

```typescript
interface ValidationRequest {
  ticketId: string;
  validationMethod: 'qr' | 'barcode' | 'nfc' | 'biometric';
  validationData: string; // QR payload, NFC UID, etc.
  location: {
    venueId: string;
    gate: string;
    coordinates?: { lat: number; lon: number };
  };
  timestamp: Date;
  deviceId: string;
  validatorId?: string; // Staff member ID
}

interface ValidationResponse {
  isValid: boolean;
  ticketId: string;
  holder: {
    name: string;
    verified: boolean;
  };
  seat?: {
    section: string;
    row: string;
    seat: string;
  };
  warnings: string[];
  errors: string[];
  action: 'allow' | 'deny' | 'manual-check';
  accessLevel: 'standard' | 'vip' | 'staff' | 'accessible';
}
```

### 13.3 Offline Validation

```typescript
// Generate offline validation bundle
const offlineBundle = await generateOfflineBundle({
  eventId: 'EVT-2025-001',
  validUntil: new Date('2025-06-21T02:00:00Z'),
  includeTickets: 'all',
  encryptionKey: 'SECURE_KEY'
});

// Validate offline
const validator = new OfflineValidator(offlineBundle);
const result = validator.validate({
  qrCode: scannedQR,
  timestamp: new Date()
});
```

---




---

## A.1 Fraud-prevention protocol

Fraud signals: device fingerprint, geolocation drift, account-age vs. purchase-volume ratio, bot-pattern detection (request-rate, mouse/touch entropy), payment-token reuse across high-risk accounts, and known-bot user-agent lists. The protocol layer scores each transaction at reservation, purchase, and transfer time; high-risk scores route to a manual-review queue with a configurable SLA (default 24 hours). Reference scorers integrate with existing risk-management providers.

## A.2 Transfer and resale protocol

Tickets carry a transfer policy: untransferable, holder-named transfer only, free transfer, transfer-with-fee, transfer-disallowed-after-T (cooling-off period). Resale policy: not allowed, allowed at face value only, allowed with venue-set ceiling, free resale on the venue's platform. Each transfer event mints a new transfer record signed by the previous holder and the venue's key; the chain of custody is verifiable by anyone with the ticket QR-payload.

## A.3 Access-control protocol

Gate scanners verify the ticket QR-payload by checking: signature validity, ticket validity window, prior-scan record (each ticket is single-use unless explicitly re-entry-allowed), and the seat-vs-attended-zone match. The scanner reports the scan event back to the central system within 10 seconds; offline-tolerant scanners cache scans and replay on reconnect with the same idempotency-key model used elsewhere in the WIA family.

## A.4 Multi-venue and season-pass protocol

Season-pass tickets are a special case of multi-event tickets: the envelope carries an event_id_set instead of a single event_id, and the access-control protocol verifies membership at scan time. Multi-venue passes (ski resorts, museum networks) follow the same model with venue_id_set; per-venue caps (e.g., 4 visits per quarter) are enforced at scan time via a counter that the venue's local cache shares with the central system on each reconnection.

## A.5 Refund and cancellation protocol

Refund policy applies per-ticket; default is `full-refund-up-to-T-hours-before-event, partial-refund-after-T, no-refund-after-event-start`. Cancellation events cascade across transfers (the most-recent holder is refunded; previous holders are notified). Event-cancellation refund is universal; the venue assumes the chargeback risk.

## A.6 Replay defence

The QR-payload's JWT-style signature, plus the central scan-record store, defeats simple replay (scanning a screenshot or photocopy after the original holder has already entered). Single-use enforcement is the primary defence; secondary defence is the per-ticket-per-scanner bloom filter that flags rapid-succession scans of the same payload.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/ticketing-system/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-ticketing-system-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/ticketing-system-host:1.0.0` ships every ticketing-system envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/ticketing-system.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Ticketing-system deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
