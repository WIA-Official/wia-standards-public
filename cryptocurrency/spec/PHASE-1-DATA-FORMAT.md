# WIA Cryptocurrency Standard
## Phase 1: Data Format Specification

**Version:** 1.0.0
**Standard:** WIA-FIN-003
**Status:** Draft
**Last Updated:** December 2025

---

## Table of Contents

1. [Overview](#overview)
2. [Transaction Schema](#transaction-schema)
3. [Block Structure](#block-structure)
4. [Wallet Formats](#wallet-formats)
5. [UTXO Model](#utxo-model)
6. [Account Model](#account-model)
7. [Signature Schemes](#signature-schemes)
8. [Data Encoding](#data-encoding)
9. [Validation Rules](#validation-rules)
10. [Examples](#examples)

---

## Overview

The WIA Cryptocurrency Standard defines universal data formats for cryptocurrency transactions, blocks, wallets, and cryptographic operations. This specification ensures interoperability across different blockchain architectures including Bitcoin-style UTXO systems and Ethereum-style account-based systems.

### Key Principles

- **Universal Compatibility**: Support for both UTXO and account-based models
- **Cryptographic Security**: Multiple signature schemes (ECDSA, Schnorr)
- **Standardization**: Consistent data structures across cryptocurrencies
- **Extensibility**: Forward-compatible schema design
- **Interoperability**: Cross-chain transaction representation

### Supported Cryptocurrencies

- Bitcoin (BTC) - UTXO model
- Ethereum (ETH) - Account model
- Litecoin (LTC) - UTXO model
- Bitcoin Cash (BCH) - UTXO model
- Ripple (XRP) - Account model
- Cardano (ADA) - Extended UTXO model
- Polkadot (DOT) - Account model
- Solana (SOL) - Account model
- And 100+ other cryptocurrencies

---

## Transaction Schema

### Universal Transaction Format

All cryptocurrency transactions conform to this base format:

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-003",
  "cryptocurrency": "string",
  "network": "mainnet|testnet|regtest",
  "transactionId": "string",
  "transactionType": "transfer|contract|stake|swap",
  "timestamp": "ISO-8601 timestamp",
  "lockTime": "number (optional)",
  "size": "number (bytes)",
  "weight": "number (optional, for SegWit)",
  "fee": {
    "amount": "string",
    "currency": "string",
    "feeRate": "string (satoshi/byte or gwei)"
  },
  "confirmations": "number",
  "blockHeight": "number (optional)",
  "blockHash": "string (optional)",
  "signatures": [],
  "metadata": {}
}
```

### Bitcoin-Style Transaction (UTXO)

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-003",
  "cryptocurrency": "BTC",
  "network": "mainnet",
  "transactionId": "a1b2c3d4e5f6...",
  "transactionType": "transfer",
  "timestamp": "2025-12-25T10:00:00Z",
  "version_number": 2,
  "lockTime": 0,
  "size": 250,
  "weight": 1000,
  "inputs": [
    {
      "previousTxId": "f6e5d4c3b2a1...",
      "outputIndex": 0,
      "scriptSig": "hex string",
      "sequence": 4294967295,
      "witness": ["hex string"],
      "value": "50000000"
    }
  ],
  "outputs": [
    {
      "index": 0,
      "value": "48000000",
      "scriptPubKey": "hex string",
      "address": "1A1zP1eP5QGefi2DMPTfTL5SLmv7DivfNa",
      "type": "P2PKH|P2SH|P2WPKH|P2WSH|P2TR"
    },
    {
      "index": 1,
      "value": "1900000",
      "scriptPubKey": "hex string",
      "address": "bc1qxy2kgdygjrsqtzq2n0yrf2493p83kkfjhx0wlh",
      "type": "P2WPKH"
    }
  ],
  "fee": {
    "amount": "100000",
    "currency": "satoshi",
    "feeRate": "400"
  },
  "confirmations": 6,
  "blockHeight": 820000,
  "blockHash": "0000000000000000000123456789abcdef...",
  "signatures": [
    {
      "scheme": "ECDSA",
      "curve": "secp256k1",
      "signature": "hex string",
      "publicKey": "hex string"
    }
  ]
}
```

### Ethereum-Style Transaction (Account)

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-003",
  "cryptocurrency": "ETH",
  "network": "mainnet",
  "transactionId": "0xabcdef123456...",
  "transactionType": "transfer",
  "timestamp": "2025-12-25T10:00:00Z",
  "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "to": "0x5aAeb6053F3E94C9b9A09f33669435E7Ef1BeAed",
  "value": "1000000000000000000",
  "nonce": 42,
  "gasLimit": "21000",
  "gasPrice": "20000000000",
  "maxFeePerGas": "25000000000",
  "maxPriorityFeePerGas": "2000000000",
  "data": "0x",
  "chainId": 1,
  "type": 2,
  "accessList": [],
  "fee": {
    "amount": "420000000000000",
    "currency": "wei",
    "feeRate": "20"
  },
  "confirmations": 12,
  "blockHeight": 18500000,
  "blockHash": "0x1234567890abcdef...",
  "signatures": [
    {
      "scheme": "ECDSA",
      "curve": "secp256k1",
      "r": "0x1234567890abcdef...",
      "s": "0xfedcba0987654321...",
      "v": 27,
      "recoveryParam": 0
    }
  ]
}
```

---

## Block Structure

### Universal Block Format

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-003",
  "cryptocurrency": "string",
  "network": "mainnet|testnet",
  "blockHeight": "number",
  "blockHash": "string",
  "previousBlockHash": "string",
  "timestamp": "ISO-8601 timestamp",
  "difficulty": "string",
  "nonce": "number",
  "size": "number (bytes)",
  "weight": "number (optional)",
  "transactionCount": "number",
  "transactions": [],
  "merkleRoot": "string",
  "stateRoot": "string (optional, for account-based)",
  "receiptsRoot": "string (optional)",
  "miner": "string (address or identifier)",
  "reward": {
    "block": "string",
    "fees": "string",
    "total": "string"
  },
  "metadata": {}
}
```

### Bitcoin Block Structure

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-003",
  "cryptocurrency": "BTC",
  "network": "mainnet",
  "blockHeight": 820000,
  "blockHash": "0000000000000000000123456789abcdef...",
  "previousBlockHash": "00000000000000000001fedcba987654321...",
  "timestamp": "2025-12-25T10:00:00Z",
  "difficulty": "65512345678901",
  "nonce": 2573587392,
  "bits": "170ba48f",
  "chainwork": "hex string",
  "size": 1500000,
  "weight": 4000000,
  "strippedSize": 1000000,
  "transactionCount": 2500,
  "transactions": ["txid1", "txid2", "..."],
  "merkleRoot": "abc123def456...",
  "miner": "1A1zP1eP5QGefi2DMPTfTL5SLmv7DivfNa",
  "reward": {
    "block": "625000000",
    "fees": "50000000",
    "total": "675000000"
  },
  "version_number": 536870912,
  "medianTime": 1735120000,
  "metadata": {
    "coinbaseText": "BTC.com",
    "pool": "BTC.com"
  }
}
```

### Ethereum Block Structure

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-003",
  "cryptocurrency": "ETH",
  "network": "mainnet",
  "blockHeight": 18500000,
  "blockHash": "0x1234567890abcdef...",
  "previousBlockHash": "0xfedcba0987654321...",
  "timestamp": "2025-12-25T10:00:00Z",
  "difficulty": "17179869184",
  "totalDifficulty": "58750003716598352816469",
  "nonce": "0x1234567890abcdef",
  "size": 150000,
  "gasLimit": "30000000",
  "gasUsed": "29500000",
  "baseFeePerGas": "15000000000",
  "transactionCount": 300,
  "transactions": ["0xtxid1", "0xtxid2", "..."],
  "merkleRoot": "0xabc123def456...",
  "stateRoot": "0x1a2b3c4d5e6f...",
  "receiptsRoot": "0x6f5e4d3c2b1a...",
  "logsBloom": "0x00000000...",
  "miner": "0x829BD824B016326A401d083B33D092293333A830",
  "extraData": "0x476574682f76312e302e302f6c696e75782f676f312e342e32",
  "mixHash": "0x1234567890abcdef...",
  "sha3Uncles": "0x1dcc4de8dec75d7aab85b567b6ccd41ad312451b948a7413f0a142fd40d49347",
  "uncles": [],
  "reward": {
    "block": "2000000000000000000",
    "fees": "500000000000000000",
    "total": "2500000000000000000"
  },
  "metadata": {
    "withdrawals": [],
    "blobGasUsed": "0",
    "excessBlobGas": "0"
  }
}
```

---

## Wallet Formats

### Hierarchical Deterministic (HD) Wallet

Based on BIP32, BIP39, and BIP44 standards:

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-003",
  "walletType": "HD",
  "cryptocurrency": "multi",
  "mnemonic": {
    "phrase": "12 or 24 word mnemonic (encrypted or hashed)",
    "wordCount": 12,
    "language": "english",
    "passphrase": "optional passphrase (encrypted)"
  },
  "seed": "hex string (encrypted)",
  "masterKey": {
    "privateKey": "hex string (encrypted)",
    "publicKey": "hex string",
    "chainCode": "hex string (encrypted)",
    "fingerprint": "hex string"
  },
  "derivationPath": "m/44'/0'/0'/0/0",
  "accounts": [
    {
      "index": 0,
      "cryptocurrency": "BTC",
      "purpose": 44,
      "coinType": 0,
      "account": 0,
      "change": 0,
      "addressIndex": 0,
      "path": "m/44'/0'/0'/0/0",
      "address": "1A1zP1eP5QGefi2DMPTfTL5SLmv7DivfNa",
      "publicKey": "hex string",
      "privateKey": "hex string (encrypted)",
      "balance": "5000000000"
    },
    {
      "index": 1,
      "cryptocurrency": "ETH",
      "purpose": 44,
      "coinType": 60,
      "account": 0,
      "change": 0,
      "addressIndex": 0,
      "path": "m/44'/60'/0'/0/0",
      "address": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
      "publicKey": "hex string",
      "privateKey": "hex string (encrypted)",
      "balance": "10000000000000000000"
    }
  ],
  "encryption": {
    "algorithm": "AES-256-GCM",
    "kdf": "PBKDF2",
    "iterations": 100000,
    "salt": "hex string"
  },
  "metadata": {
    "name": "My Crypto Wallet",
    "createdAt": "2025-12-25T10:00:00Z",
    "lastUsed": "2025-12-25T12:00:00Z"
  }
}
```

### Single-Key Wallet

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-003",
  "walletType": "single",
  "cryptocurrency": "BTC",
  "address": "1A1zP1eP5QGefi2DMPTfTL5SLmv7DivfNa",
  "publicKey": "hex string",
  "privateKey": "hex string (encrypted)",
  "wif": "Wallet Import Format (encrypted)",
  "compressed": true,
  "addressType": "P2PKH|P2SH|P2WPKH|P2WSH|P2TR",
  "balance": "5000000000",
  "encryption": {
    "algorithm": "AES-256-GCM",
    "kdf": "scrypt",
    "n": 16384,
    "r": 8,
    "p": 1,
    "salt": "hex string"
  },
  "metadata": {
    "label": "Savings Wallet",
    "createdAt": "2025-12-25T10:00:00Z"
  }
}
```

### Multi-Signature Wallet

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-003",
  "walletType": "multisig",
  "cryptocurrency": "BTC",
  "scriptType": "P2SH|P2WSH",
  "address": "3J98t1WpEZ73CNmYviecrnyiWrnqRhWNLy",
  "redeemScript": "hex string",
  "witnessScript": "hex string (optional)",
  "requiredSignatures": 2,
  "totalSigners": 3,
  "signers": [
    {
      "index": 0,
      "name": "Alice",
      "publicKey": "hex string",
      "fingerprint": "hex string",
      "derivationPath": "m/48'/0'/0'/2'"
    },
    {
      "index": 1,
      "name": "Bob",
      "publicKey": "hex string",
      "fingerprint": "hex string",
      "derivationPath": "m/48'/0'/0'/2'"
    },
    {
      "index": 2,
      "name": "Carol",
      "publicKey": "hex string",
      "fingerprint": "hex string",
      "derivationPath": "m/48'/0'/0'/2'"
    }
  ],
  "balance": "100000000",
  "metadata": {
    "name": "Corporate Treasury",
    "createdAt": "2025-12-25T10:00:00Z",
    "purpose": "Business funds"
  }
}
```

---

## UTXO Model

### Unspent Transaction Output Format

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-003",
  "cryptocurrency": "BTC",
  "utxoId": "txid:outputIndex",
  "transactionId": "a1b2c3d4e5f6...",
  "outputIndex": 0,
  "value": "50000000",
  "scriptPubKey": "hex string",
  "address": "1A1zP1eP5QGefi2DMPTfTL5SLmv7DivfNa",
  "scriptType": "P2PKH",
  "confirmations": 100,
  "blockHeight": 819900,
  "blockTime": "2025-12-24T10:00:00Z",
  "spendable": true,
  "solvable": true,
  "safe": true,
  "metadata": {
    "coinbase": false,
    "locked": false
  }
}
```

### UTXO Set

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-003",
  "cryptocurrency": "BTC",
  "address": "1A1zP1eP5QGefi2DMPTfTL5SLmv7DivfNa",
  "totalBalance": "150000000",
  "utxoCount": 3,
  "utxos": [
    {
      "utxoId": "tx1:0",
      "value": "50000000",
      "confirmations": 100
    },
    {
      "utxoId": "tx2:1",
      "value": "75000000",
      "confirmations": 50
    },
    {
      "utxoId": "tx3:0",
      "value": "25000000",
      "confirmations": 10
    }
  ],
  "metadata": {
    "lastUpdated": "2025-12-25T10:00:00Z"
  }
}
```

---

## Account Model

### Account State Format

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-003",
  "cryptocurrency": "ETH",
  "address": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "nonce": 42,
  "balance": "10000000000000000000",
  "storageRoot": "0x1a2b3c4d5e6f...",
  "codeHash": "0x6f5e4d3c2b1a...",
  "accountType": "EOA|contract",
  "code": "hex string (for contracts)",
  "storage": {},
  "metadata": {
    "createdAt": "2025-01-01T00:00:00Z",
    "lastActivity": "2025-12-25T10:00:00Z",
    "transactionCount": 42
  }
}
```

### Smart Contract Account

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-003",
  "cryptocurrency": "ETH",
  "address": "0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48",
  "nonce": 1,
  "balance": "0",
  "storageRoot": "0xabc123def456...",
  "codeHash": "0x123456789abc...",
  "accountType": "contract",
  "code": "hex string (bytecode)",
  "contractName": "USD Coin",
  "contractSymbol": "USDC",
  "tokenStandard": "ERC-20",
  "totalSupply": "40000000000000000",
  "creator": "0x95ba4cF87D6723ad9C0Db21737D862bE80e93911",
  "deployedAt": {
    "blockHeight": 6082465,
    "timestamp": "2018-08-07T17:51:00Z"
  },
  "verified": true,
  "audited": true,
  "metadata": {
    "website": "https://www.circle.com/usdc",
    "decimals": 6
  }
}
```

---

## Signature Schemes

### ECDSA (Elliptic Curve Digital Signature Algorithm)

Default signature scheme for Bitcoin and Ethereum:

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-003",
  "signatureScheme": "ECDSA",
  "curve": "secp256k1",
  "hashAlgorithm": "SHA-256",
  "signature": {
    "r": "hex string (32 bytes)",
    "s": "hex string (32 bytes)",
    "v": 27,
    "recoveryParam": 0
  },
  "publicKey": {
    "format": "uncompressed|compressed",
    "x": "hex string (32 bytes)",
    "y": "hex string (32 bytes, optional for compressed)",
    "full": "hex string (65 bytes uncompressed or 33 bytes compressed)"
  },
  "message": "hex string",
  "messageHash": "hex string (32 bytes)",
  "metadata": {
    "signedAt": "2025-12-25T10:00:00Z",
    "purpose": "transaction"
  }
}
```

### Schnorr Signatures

Used in Bitcoin Taproot (BIP340):

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-003",
  "signatureScheme": "Schnorr",
  "curve": "secp256k1",
  "hashAlgorithm": "SHA-256",
  "signature": {
    "r": "hex string (32 bytes)",
    "s": "hex string (32 bytes)",
    "combined": "hex string (64 bytes)"
  },
  "publicKey": {
    "format": "x-only",
    "x": "hex string (32 bytes)",
    "parity": "even|odd"
  },
  "message": "hex string",
  "messageHash": "hex string (32 bytes)",
  "auxRand": "hex string (32 bytes, optional)",
  "metadata": {
    "signedAt": "2025-12-25T10:00:00Z",
    "purpose": "taproot",
    "keyAggregation": false
  }
}
```

### Multi-Signature (Schnorr MuSig2)

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-003",
  "signatureScheme": "Schnorr-MuSig2",
  "curve": "secp256k1",
  "hashAlgorithm": "SHA-256",
  "requiredSigners": 2,
  "totalSigners": 3,
  "publicKeys": [
    {
      "signer": "Alice",
      "publicKey": "hex string (32 bytes)",
      "nonce": "hex string"
    },
    {
      "signer": "Bob",
      "publicKey": "hex string (32 bytes)",
      "nonce": "hex string"
    },
    {
      "signer": "Carol",
      "publicKey": "hex string (32 bytes)",
      "nonce": "hex string"
    }
  ],
  "aggregatedPublicKey": "hex string (32 bytes)",
  "partialSignatures": [
    {
      "signer": "Alice",
      "signature": "hex string (32 bytes)"
    },
    {
      "signer": "Bob",
      "signature": "hex string (32 bytes)"
    }
  ],
  "aggregatedSignature": {
    "r": "hex string (32 bytes)",
    "s": "hex string (32 bytes)"
  },
  "message": "hex string",
  "messageHash": "hex string (32 bytes)",
  "metadata": {
    "signedAt": "2025-12-25T10:00:00Z",
    "purpose": "multisig-transaction"
  }
}
```

---

## Data Encoding

### Base58 Encoding

Used for Bitcoin addresses and keys:

```json
{
  "encoding": "Base58",
  "alphabet": "123456789ABCDEFGHJKLMNPQRSTUVWXYZabcdefghijkmnopqrstuvwxyz",
  "checksum": "double SHA-256",
  "examples": {
    "address": "1A1zP1eP5QGefi2DMPTfTL5SLmv7DivfNa",
    "privateKey": "5HueCGU8rMjxEXxiPuD5BDku4MkFqeZyd4dZ1jvhTVqvbTLvyTJ"
  }
}
```

### Bech32 Encoding

Used for SegWit addresses:

```json
{
  "encoding": "Bech32",
  "hrp": "bc (mainnet) | tb (testnet)",
  "checksum": "BCH code",
  "examples": {
    "P2WPKH": "bc1qw508d6qejxtdg4y5r3zarvary0c5xw7kv8f3t4",
    "P2WSH": "bc1qrp33g0q5c5txsp9arysrx4k6zdkfs4nce4xj0gdcccefvpysxf3qccfmv3",
    "P2TR": "bc1p5d7rjq7g6rdk2yhzks9smlaqtedr4dekq08ge8ztwac72sfr9rusxg3297"
  }
}
```

### Hexadecimal Encoding

Standard for Ethereum and raw data:

```json
{
  "encoding": "Hexadecimal",
  "prefix": "0x",
  "case": "lowercase preferred",
  "examples": {
    "address": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
    "transactionHash": "0xabcdef1234567890abcdef1234567890abcdef1234567890abcdef1234567890",
    "data": "0xa9059cbb00000000000000000000000012345678901234567890123456789012345678900000000000000000000000000000000000000000000000000000000000000064"
  }
}
```

---

## Validation Rules

### Address Validation

#### Bitcoin Address Validation

1. **Base58Check**: Verify checksum for legacy addresses (P2PKH, P2SH)
2. **Bech32**: Verify BCH checksum for SegWit addresses
3. **Length**:
   - P2PKH: 26-35 characters
   - P2SH: 26-35 characters
   - P2WPKH: 42 characters (Bech32)
   - P2WSH: 62 characters (Bech32)
   - P2TR: 62 characters (Bech32m)
4. **Prefix**:
   - Mainnet P2PKH: starts with '1'
   - Mainnet P2SH: starts with '3'
   - Mainnet SegWit: starts with 'bc1'
   - Testnet: starts with 'm', 'n', '2', or 'tb1'

#### Ethereum Address Validation

1. **Format**: 0x + 40 hexadecimal characters
2. **Checksum**: EIP-55 mixed-case checksum validation
3. **Zero Address**: `0x0000000000000000000000000000000000000000` is valid but special

### Transaction Validation

#### UTXO Model Validation

1. **Input Validation**:
   - All referenced UTXOs must exist
   - Sum of input values ≥ sum of output values + fee
   - No double-spending
   - Valid signatures for all inputs
   - Sequence numbers valid

2. **Output Validation**:
   - Output values > 0 (except OP_RETURN)
   - Total output value ≤ total input value
   - ScriptPubKey valid

3. **Fee Validation**:
   - Fee = (input sum - output sum) ≥ 0
   - Fee rate reasonable (not too low, not excessive)

#### Account Model Validation

1. **Nonce Validation**:
   - Nonce must equal current account nonce
   - Prevents replay attacks

2. **Balance Validation**:
   - Account balance ≥ (value + gas cost)
   - Gas limit ≥ 21000 for basic transfer

3. **Signature Validation**:
   - Valid ECDSA signature
   - Recovers to sender address
   - Chain ID matches (EIP-155)

### Signature Validation

#### ECDSA Validation

1. **Format**: r and s must be 32 bytes each
2. **Range**: 0 < r < n and 0 < s < n (where n is curve order)
3. **Malleability**: s must be in lower half of curve order (BIP 62)
4. **Recovery**: Public key recovery must yield expected address

#### Schnorr Validation

1. **Format**: Combined signature must be 64 bytes
2. **Public Key**: X-only public key must be 32 bytes
3. **Verification**: R + hash(R||P||m) * P = s * G
4. **Batch Verification**: Support for batch validation

---

## Examples

### Example 1: Bitcoin Transaction (P2WPKH)

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-003",
  "cryptocurrency": "BTC",
  "network": "mainnet",
  "transactionId": "a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9f0a1b2",
  "transactionType": "transfer",
  "timestamp": "2025-12-25T10:00:00Z",
  "version_number": 2,
  "lockTime": 0,
  "size": 222,
  "weight": 561,
  "inputs": [
    {
      "previousTxId": "f0e9d8c7b6a5f4e3d2c1b0a9f8e7d6c5b4a3f2e1d0c9b8a7f6e5d4c3b2a1f0e9",
      "outputIndex": 0,
      "scriptSig": "",
      "sequence": 4294967295,
      "witness": [
        "304402201234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef02201234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef01",
        "031234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef12"
      ],
      "value": "50000000"
    }
  ],
  "outputs": [
    {
      "index": 0,
      "value": "48000000",
      "scriptPubKey": "0014751e76e8199196d454941c45d1b3a323f1433bd6",
      "address": "bc1qw508d6qejxtdg4y5r3zarvary0c5xw7kv8f3t4",
      "type": "P2WPKH"
    },
    {
      "index": 1,
      "value": "1900000",
      "scriptPubKey": "0014ab68025513c3dbd2f7b92a94e0581f5d50f654e7",
      "address": "bc1q4d5qy4gnchdamdaeh2jwqkqlt4g0v48xqafqhx",
      "type": "P2WPKH"
    }
  ],
  "fee": {
    "amount": "100000",
    "currency": "satoshi",
    "feeRate": "178"
  },
  "confirmations": 6,
  "blockHeight": 820000,
  "blockHash": "00000000000000000001234567890abcdef1234567890abcdef1234567890abc",
  "signatures": [
    {
      "scheme": "ECDSA",
      "curve": "secp256k1",
      "signature": "304402201234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef02201234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef",
      "publicKey": "031234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef12"
    }
  ]
}
```

### Example 2: Ethereum Transaction (EIP-1559)

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-003",
  "cryptocurrency": "ETH",
  "network": "mainnet",
  "transactionId": "0xabcdef1234567890abcdef1234567890abcdef1234567890abcdef1234567890",
  "transactionType": "transfer",
  "timestamp": "2025-12-25T10:30:00Z",
  "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "to": "0x5aAeb6053F3E94C9b9A09f33669435E7Ef1BeAed",
  "value": "1000000000000000000",
  "nonce": 42,
  "gasLimit": "21000",
  "maxFeePerGas": "25000000000",
  "maxPriorityFeePerGas": "2000000000",
  "data": "0x",
  "chainId": 1,
  "type": 2,
  "accessList": [],
  "fee": {
    "amount": "441000000000000",
    "currency": "wei",
    "feeRate": "21"
  },
  "confirmations": 12,
  "blockHeight": 18500000,
  "blockHash": "0x1234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef",
  "signatures": [
    {
      "scheme": "ECDSA",
      "curve": "secp256k1",
      "r": "0x1234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef",
      "s": "0xfedcba0987654321fedcba0987654321fedcba0987654321fedcba0987654321",
      "v": 0,
      "recoveryParam": 0
    }
  ]
}
```

---

## Compliance & Security

### Security Requirements

1. **Private Key Protection**: All private keys must be encrypted at rest
2. **Secure Random Generation**: Use cryptographically secure random number generators
3. **Signature Verification**: Verify all signatures before accepting transactions
4. **Replay Protection**: Implement chain-specific replay protection (EIP-155)
5. **Double-Spend Prevention**: Maintain UTXO set integrity

### Data Privacy

1. **Key Management**: Never expose private keys in logs or APIs
2. **Mnemonic Security**: Encrypt mnemonic phrases with strong passphrases
3. **Address Reuse**: Discourage address reuse for privacy
4. **Transaction Privacy**: Support confidential transactions where applicable

---

## Version History

- **v1.0.0** (December 2025): Initial release
  - Universal transaction format
  - UTXO and account model support
  - ECDSA and Schnorr signature schemes
  - Wallet format specifications
  - Comprehensive validation rules

---

**弘益人間 · Benefit All Humanity**

© 2025 WIA - World Interoperability Alliance
MIT License
