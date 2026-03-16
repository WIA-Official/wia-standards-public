# WIA-FIN-015: Digital Wallet Standard v1.0

## Overview

WIA-FIN-015 defines a comprehensive standard for digital wallet implementation, covering data formats, APIs, security protocols, and integration patterns. This specification ensures interoperability, security, and user-friendly experiences across different wallet implementations.

**Version:** 1.0.0
**Status:** Stable
**Last Updated:** 2025-01-15

## Philosophy

**弘익人間 (홍익인간) · Benefit All Humanity**

This standard is built on the principle of creating technology that serves and benefits all of humanity, making financial services accessible, secure, and fair for everyone.

## Scope

This specification covers:
- Wallet data structures and formats
- Multi-currency support (fiat and cryptocurrency)
- Transaction protocols
- Security and encryption standards
- API specifications
- Integration guidelines

## Wallet Types

### 1. Hierarchical Deterministic (HD) Wallet

HD wallets generate a tree of key pairs from a single master seed, following BIP32, BIP39, and BIP44 standards.

#### Wallet Structure

```json
{
  "wallet_id": "wlt_abc123xyz",
  "type": "hd",
  "version": "1.0.0",
  "created_at": "2025-01-15T10:30:00Z",
  "master_fingerprint": "d34db33f",
  "derivation_paths": {
    "bitcoin": "m/44'/0'/0'/0",
    "ethereum": "m/44'/60'/0'/0",
    "fiat": "m/44'/9999'/0'/0"
  },
  "supported_currencies": [
    {
      "code": "USD",
      "type": "fiat",
      "linked_account": {
        "type": "bank_account",
        "account_id": "acc_456def",
        "routing_number": "021000021",
        "account_number_last4": "1234"
      }
    },
    {
      "code": "BTC",
      "type": "cryptocurrency",
      "network": "mainnet",
      "addresses": [
        {
          "index": 0,
          "address": "bc1qxy2kgdygjrsqtzq2n0yrf2493p83kkfjhx0wlh",
          "balance": "0.0234"
        }
      ]
    },
    {
      "code": "ETH",
      "type": "cryptocurrency",
      "network": "mainnet",
      "addresses": [
        {
          "index": 0,
          "address": "0x742d35Cc6634C0532925a3b844E76735f18D8E9C",
          "balance": "1.5"
        }
      ]
    }
  ],
  "security": {
    "encryption": "AES-256-GCM",
    "biometric_enabled": true,
    "two_factor_enabled": true,
    "backup_status": "completed",
    "last_backup": "2025-01-15T09:00:00Z"
  }
}
```

### 2. Simple Wallet

A basic wallet with a single key pair per currency.

```json
{
  "wallet_id": "wlt_simple789",
  "type": "simple",
  "version": "1.0.0",
  "created_at": "2025-01-15T11:00:00Z",
  "currencies": [
    {
      "code": "USD",
      "balance": 1234.56,
      "address": "user@example.com"
    }
  ]
}
```

### 3. Multi-Signature Wallet

Requires multiple signatures for transaction authorization.

```json
{
  "wallet_id": "wlt_multisig456",
  "type": "multisig",
  "version": "1.0.0",
  "required_signatures": 2,
  "total_signers": 3,
  "signers": [
    {
      "id": "signer_1",
      "public_key": "03a34b99f22c790c4e36b2b3c2c35a36db06226e41c692fc82b8b56ac1c540c5bd",
      "role": "owner",
      "weight": 1
    },
    {
      "id": "signer_2",
      "public_key": "02f9308a019258c31049344f85f89d5229b531c845836f99b08601f113bce036f9",
      "role": "co-owner",
      "weight": 1
    },
    {
      "id": "signer_3",
      "public_key": "0279be667ef9dcbbac55a06295ce870b07029bfcdb2dce28d959f2815b16f81798",
      "role": "guardian",
      "weight": 1
    }
  ]
}
```

## Transaction Format

### Standard Transaction Structure

```json
{
  "transaction_id": "tx_20250115_001",
  "wallet_id": "wlt_abc123xyz",
  "timestamp": "2025-01-15T14:23:45Z",
  "type": "send",
  "status": "confirmed",
  "from": {
    "wallet_id": "wlt_abc123xyz",
    "address": "0x742d35Cc6634C0532925a3b844E76735f18D8E9C",
    "name": "John Doe"
  },
  "to": {
    "address": "0x8ba1f109551bd432803012645ac136ddd64dba72",
    "name": "Coffee Shop",
    "merchant_id": "merchant_789"
  },
  "amount": {
    "value": 5.99,
    "currency": "USD"
  },
  "fee": {
    "value": 0.12,
    "currency": "USD",
    "type": "network"
  },
  "method": "nfc",
  "network": {
    "name": "Visa",
    "confirmation_time": "1.2s"
  },
  "confirmations": 12,
  "metadata": {
    "merchant_category": "food_beverage",
    "location": {
      "city": "New York",
      "state": "NY",
      "country": "US",
      "coordinates": {
        "lat": 40.7128,
        "lon": -74.0060
      }
    },
    "device": {
      "type": "mobile",
      "model": "iPhone 15 Pro",
      "os": "iOS 17.2"
    },
    "receipt_url": "https://receipts.example.com/tx_20250115_001"
  },
  "security": {
    "signature": "0x1b2c3d4e5f...",
    "authentication_method": "biometric",
    "fraud_check": "passed",
    "risk_score": 0.05
  }
}
```

### Transaction Types

- `send`: Outgoing payment
- `receive`: Incoming payment
- `exchange`: Currency conversion
- `stake`: Cryptocurrency staking
- `withdraw`: Withdrawal to external account
- `deposit`: Deposit from external source

### Transaction Status

- `pending`: Initiated but not confirmed
- `processing`: Being processed by network
- `confirmed`: Successfully completed
- `failed`: Transaction failed
- `cancelled`: Cancelled by user or system
- `refunded`: Payment returned to sender

## Currency Support

### Fiat Currencies

Wallets MUST support at least these major fiat currencies:

```json
{
  "fiat_currencies": [
    { "code": "USD", "name": "US Dollar", "symbol": "$", "decimals": 2 },
    { "code": "EUR", "name": "Euro", "symbol": "€", "decimals": 2 },
    { "code": "GBP", "name": "British Pound", "symbol": "£", "decimals": 2 },
    { "code": "JPY", "name": "Japanese Yen", "symbol": "¥", "decimals": 0 },
    { "code": "CNY", "name": "Chinese Yuan", "symbol": "¥", "decimals": 2 }
  ]
}
```

### Cryptocurrencies

Support for major cryptocurrencies is RECOMMENDED:

```json
{
  "cryptocurrencies": [
    {
      "code": "BTC",
      "name": "Bitcoin",
      "network": "mainnet",
      "decimals": 8,
      "address_format": "bech32"
    },
    {
      "code": "ETH",
      "name": "Ethereum",
      "network": "mainnet",
      "decimals": 18,
      "address_format": "eip55"
    },
    {
      "code": "USDT",
      "name": "Tether",
      "type": "erc20",
      "contract_address": "0xdac17f958d2ee523a2206206994597c13d831ec7",
      "decimals": 6
    }
  ]
}
```

## Security Requirements

### Encryption

- **At Rest**: AES-256-GCM for all stored data
- **In Transit**: TLS 1.3 for all network communications
- **Keys**: PBKDF2 with minimum 100,000 iterations for key derivation

### Authentication

Wallets MUST implement at least two of the following:

1. **Password/PIN**: Minimum 6 characters, alphanumeric + special characters recommended
2. **Biometric**: Face ID, Touch ID, or fingerprint scanning
3. **Two-Factor Authentication**: TOTP (Time-based One-Time Password)
4. **Hardware Token**: Physical security key (FIDO2/WebAuthn)

### Transaction Signing

All transactions MUST be cryptographically signed:

```json
{
  "signature": {
    "algorithm": "ECDSA",
    "curve": "secp256k1",
    "hash": "SHA256",
    "value": "0x1b2c3d4e5f6789abcdef...",
    "public_key": "03a34b99f22c790c4e36b2b3c2c35a36db06226e41c692fc82b8b56ac1c540c5bd"
  }
}
```

## API Endpoints

### Authentication

```
POST /api/v1/auth/login
POST /api/v1/auth/logout
POST /api/v1/auth/refresh
POST /api/v1/auth/2fa/enable
POST /api/v1/auth/2fa/verify
```

### Wallet Management

```
POST /api/v1/wallets
GET /api/v1/wallets/{wallet_id}
PUT /api/v1/wallets/{wallet_id}
DELETE /api/v1/wallets/{wallet_id}
GET /api/v1/wallets/{wallet_id}/balance
GET /api/v1/wallets/{wallet_id}/addresses
```

### Transactions

```
POST /api/v1/wallets/{wallet_id}/send
POST /api/v1/wallets/{wallet_id}/receive
GET /api/v1/wallets/{wallet_id}/transactions
GET /api/v1/wallets/{wallet_id}/transactions/{tx_id}
POST /api/v1/wallets/{wallet_id}/exchange
```

### Payment Methods

```
POST /api/v1/wallets/{wallet_id}/nfc/enable
POST /api/v1/wallets/{wallet_id}/qr/generate
POST /api/v1/wallets/{wallet_id}/payment-request
```

## Error Codes

Standard HTTP status codes with specific error messages:

```json
{
  "error": {
    "code": "INSUFFICIENT_BALANCE",
    "message": "Insufficient funds to complete transaction",
    "details": {
      "required": 100.00,
      "available": 85.50,
      "currency": "USD"
    },
    "timestamp": "2025-01-15T14:30:00Z"
  }
}
```

### Common Error Codes

- `INVALID_ADDRESS`: Recipient address format invalid
- `INSUFFICIENT_BALANCE`: Not enough funds
- `AUTHENTICATION_FAILED`: Authentication required or failed
- `RATE_LIMIT_EXCEEDED`: Too many requests
- `NETWORK_ERROR`: Network or connectivity issue
- `INVALID_SIGNATURE`: Transaction signature invalid
- `TRANSACTION_EXPIRED`: Transaction timeout
- `CURRENCY_NOT_SUPPORTED`: Currency not available

## Compliance

### KYC (Know Your Customer)

Wallets handling fiat currency MUST implement KYC:

- Identity verification (government ID)
- Address verification (utility bill, bank statement)
- Selfie verification with liveness detection
- PEP (Politically Exposed Person) screening

### AML (Anti-Money Laundering)

- Transaction monitoring and suspicious activity reporting
- Transaction limits based on verification level
- Geographic restrictions for high-risk regions
- Regular compliance audits

## Versioning

This specification uses Semantic Versioning (SemVer):

- **MAJOR**: Incompatible API changes
- **MINOR**: Backwards-compatible functionality additions
- **PATCH**: Backwards-compatible bug fixes

Current version: **1.0.0**

## License

This specification is released under the MIT License.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
