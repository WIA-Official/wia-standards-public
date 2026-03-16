# WIA Blockchain Finance Standard
## Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** December 2025

---

## Table of Contents

1. [Overview](#overview)
2. [Blockchain Data Format](#blockchain-data-format)
3. [JSON Schema for Transactions](#json-schema-for-transactions)
4. [Token Standards](#token-standards)
5. [Validation Rules](#validation-rules)
6. [Examples](#examples)

---

## Overview

The WIA Blockchain Finance Standard defines a universal data format for blockchain-based financial transactions, ensuring interoperability across different blockchain networks, token standards, and DeFi protocols.

### Key Principles

- **Interoperability**: Seamless data exchange across chains
- **ERC Compatibility**: Full support for Ethereum token standards
- **Type Safety**: Strong typing for all data structures
- **Extensibility**: Forward-compatible schema design
- **Security**: Built-in validation and security checks

### Supported Blockchains

- Ethereum (ETH)
- Binance Smart Chain (BSC)
- Polygon (MATIC)
- Arbitrum
- Optimism
- Avalanche
- Solana
- Polkadot
- Cosmos
- And 40+ other chains

---

## Blockchain Data Format

### Transaction Format

All blockchain transactions must conform to this universal format:

```json
{
  "version": "1.0.0",
  "chainId": "number",
  "transactionType": "string",
  "timestamp": "ISO-8601 timestamp",
  "from": "address",
  "to": "address",
  "value": "string (wei)",
  "data": "hex string",
  "nonce": "number",
  "gasLimit": "string",
  "gasPrice": "string",
  "maxFeePerGas": "string (optional, EIP-1559)",
  "maxPriorityFeePerGas": "string (optional, EIP-1559)",
  "signature": {
    "r": "hex string",
    "s": "hex string",
    "v": "number"
  },
  "hash": "transaction hash",
  "metadata": {}
}
```

### Block Format

```json
{
  "version": "1.0.0",
  "chainId": "number",
  "blockNumber": "number",
  "blockHash": "hex string",
  "parentHash": "hex string",
  "timestamp": "ISO-8601 timestamp",
  "miner": "address",
  "difficulty": "string",
  "totalDifficulty": "string",
  "gasLimit": "string",
  "gasUsed": "string",
  "transactions": [],
  "stateRoot": "hex string",
  "receiptsRoot": "hex string",
  "logsBloom": "hex string"
}
```

---

## JSON Schema for Transactions

### Base Transaction Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA Blockchain Transaction",
  "type": "object",
  "required": [
    "version",
    "chainId",
    "transactionType",
    "timestamp",
    "from",
    "to",
    "value",
    "nonce",
    "gasLimit",
    "gasPrice",
    "hash"
  ],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$",
      "description": "WIA standard version"
    },
    "chainId": {
      "type": "number",
      "minimum": 1,
      "description": "EIP-155 chain ID"
    },
    "transactionType": {
      "type": "string",
      "enum": [
        "transfer",
        "contractCall",
        "contractDeployment",
        "tokenTransfer",
        "nftTransfer",
        "swap",
        "stake",
        "unstake",
        "bridge"
      ]
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "from": {
      "type": "string",
      "pattern": "^0x[a-fA-F0-9]{40}$",
      "description": "Sender address"
    },
    "to": {
      "type": "string",
      "pattern": "^0x[a-fA-F0-9]{40}$",
      "description": "Recipient address"
    },
    "value": {
      "type": "string",
      "pattern": "^\\d+$",
      "description": "Value in wei"
    },
    "data": {
      "type": "string",
      "pattern": "^0x[a-fA-F0-9]*$",
      "description": "Contract call data"
    },
    "nonce": {
      "type": "number",
      "minimum": 0
    },
    "gasLimit": {
      "type": "string",
      "pattern": "^\\d+$"
    },
    "gasPrice": {
      "type": "string",
      "pattern": "^\\d+$"
    },
    "maxFeePerGas": {
      "type": "string",
      "pattern": "^\\d+$"
    },
    "maxPriorityFeePerGas": {
      "type": "string",
      "pattern": "^\\d+$"
    },
    "signature": {
      "type": "object",
      "required": ["r", "s", "v"],
      "properties": {
        "r": {
          "type": "string",
          "pattern": "^0x[a-fA-F0-9]{64}$"
        },
        "s": {
          "type": "string",
          "pattern": "^0x[a-fA-F0-9]{64}$"
        },
        "v": {
          "type": "number",
          "minimum": 0
        }
      }
    },
    "hash": {
      "type": "string",
      "pattern": "^0x[a-fA-F0-9]{64}$"
    },
    "metadata": {
      "type": "object",
      "description": "Additional metadata"
    }
  }
}
```

### Token Transfer Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA Token Transfer",
  "allOf": [
    { "$ref": "#/definitions/baseTransaction" }
  ],
  "properties": {
    "tokenData": {
      "type": "object",
      "required": ["standard", "contract", "amount"],
      "properties": {
        "standard": {
          "type": "string",
          "enum": ["ERC-20", "ERC-721", "ERC-1155"]
        },
        "contract": {
          "type": "string",
          "pattern": "^0x[a-fA-F0-9]{40}$"
        },
        "amount": {
          "type": "string",
          "pattern": "^\\d+$"
        },
        "tokenId": {
          "type": "string",
          "description": "Required for ERC-721 and ERC-1155"
        },
        "metadata": {
          "type": "object"
        }
      }
    }
  }
}
```

---

## Token Standards

### ERC-20: Fungible Tokens

ERC-20 is the standard for fungible tokens (cryptocurrencies, stablecoins).

**Required Methods:**
- `totalSupply() → uint256`
- `balanceOf(address account) → uint256`
- `transfer(address to, uint256 amount) → bool`
- `allowance(address owner, address spender) → uint256`
- `approve(address spender, uint256 amount) → bool`
- `transferFrom(address from, address to, uint256 amount) → bool`

**Required Events:**
- `Transfer(address indexed from, address indexed to, uint256 value)`
- `Approval(address indexed owner, address indexed spender, uint256 value)`

**WIA Extension:**
```solidity
interface IWIAERC20 {
    function metadata() external view returns (
        string memory name,
        string memory symbol,
        uint8 decimals,
        string memory standard
    );

    function supportsInterface(bytes4 interfaceId) external view returns (bool);
}
```

### ERC-721: Non-Fungible Tokens (NFTs)

ERC-721 is the standard for unique, non-fungible tokens.

**Required Methods:**
- `balanceOf(address owner) → uint256`
- `ownerOf(uint256 tokenId) → address`
- `safeTransferFrom(address from, address to, uint256 tokenId)`
- `transferFrom(address from, address to, uint256 tokenId)`
- `approve(address to, uint256 tokenId)`
- `setApprovalForAll(address operator, bool approved)`
- `getApproved(uint256 tokenId) → address`
- `isApprovedForAll(address owner, address operator) → bool`

**Required Events:**
- `Transfer(address indexed from, address indexed to, uint256 indexed tokenId)`
- `Approval(address indexed owner, address indexed approved, uint256 indexed tokenId)`
- `ApprovalForAll(address indexed owner, address indexed operator, bool approved)`

**WIA Extension:**
```solidity
interface IWIAERC721 {
    function tokenMetadata(uint256 tokenId) external view returns (
        string memory uri,
        bytes32 metadataHash,
        uint256 createdAt
    );

    function royaltyInfo(uint256 tokenId, uint256 salePrice) external view returns (
        address receiver,
        uint256 royaltyAmount
    );
}
```

### ERC-1155: Multi-Token Standard

ERC-1155 supports both fungible and non-fungible tokens in a single contract.

**Required Methods:**
- `balanceOf(address account, uint256 id) → uint256`
- `balanceOfBatch(address[] accounts, uint256[] ids) → uint256[]`
- `setApprovalForAll(address operator, bool approved)`
- `isApprovedForAll(address account, address operator) → bool`
- `safeTransferFrom(address from, address to, uint256 id, uint256 amount, bytes data)`
- `safeBatchTransferFrom(address from, address to, uint256[] ids, uint256[] amounts, bytes data)`

**Required Events:**
- `TransferSingle(address indexed operator, address indexed from, address indexed to, uint256 id, uint256 value)`
- `TransferBatch(address indexed operator, address indexed from, address indexed to, uint256[] ids, uint256[] values)`
- `ApprovalForAll(address indexed account, address indexed operator, bool approved)`
- `URI(string value, uint256 indexed id)`

**WIA Extension:**
```solidity
interface IWIAERC1155 {
    function tokenType(uint256 id) external view returns (string memory);

    function supply(uint256 id) external view returns (uint256 current, uint256 max);

    function batchMetadata(uint256[] ids) external view returns (string[] memory);
}
```

---

## Validation Rules

### Address Validation

1. **Format**: Must be a valid Ethereum-style address (0x + 40 hex characters)
2. **Checksum**: Should use EIP-55 mixed-case checksum
3. **Zero Address**: `0x0000000000000000000000000000000000000000` is invalid for `from` field

### Amount Validation

1. **Format**: String representation of integer (no decimals)
2. **Range**: Must be >= 0
3. **Max Value**: Must not exceed `2^256 - 1`
4. **Decimals**: Token amounts must account for token decimals (e.g., 1 USDC = 1000000 with 6 decimals)

### Gas Validation

1. **Gas Limit**: Must be >= 21000 for basic transfers
2. **Gas Price**: Must be > 0
3. **Max Fee**: For EIP-1559, `maxFeePerGas >= maxPriorityFeePerGas`

### Signature Validation

1. **Format**: ECDSA signature with r, s, v components
2. **Recovery**: Must recover to the `from` address
3. **Replay Protection**: Must include chain ID in signature (EIP-155)

### Chain ID Validation

Common chain IDs:
- Ethereum Mainnet: `1`
- Goerli Testnet: `5`
- Sepolia Testnet: `11155111`
- BSC Mainnet: `56`
- Polygon Mainnet: `137`
- Arbitrum One: `42161`
- Optimism: `10`

---

## Examples

### Example 1: ETH Transfer

```json
{
  "version": "1.0.0",
  "chainId": 1,
  "transactionType": "transfer",
  "timestamp": "2025-12-25T10:30:00Z",
  "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "to": "0x5aAeb6053F3E94C9b9A09f33669435E7Ef1BeAed",
  "value": "1000000000000000000",
  "data": "0x",
  "nonce": 42,
  "gasLimit": "21000",
  "gasPrice": "20000000000",
  "signature": {
    "r": "0x1234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef",
    "s": "0xfedcba0987654321fedcba0987654321fedcba0987654321fedcba0987654321",
    "v": 27
  },
  "hash": "0xabcdef1234567890abcdef1234567890abcdef1234567890abcdef1234567890",
  "metadata": {
    "description": "Payment for services"
  }
}
```

### Example 2: ERC-20 Token Transfer

```json
{
  "version": "1.0.0",
  "chainId": 1,
  "transactionType": "tokenTransfer",
  "timestamp": "2025-12-25T11:00:00Z",
  "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "to": "0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48",
  "value": "0",
  "data": "0xa9059cbb0000000000000000000000005aaeb6053f3e94c9b9a09f33669435e7ef1beaed00000000000000000000000000000000000000000000000000000000000f4240",
  "nonce": 43,
  "gasLimit": "65000",
  "gasPrice": "25000000000",
  "signature": {
    "r": "0x9876543210fedcba9876543210fedcba9876543210fedcba9876543210fedcba",
    "s": "0x0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
    "v": 28
  },
  "hash": "0x1111111111111111111111111111111111111111111111111111111111111111",
  "tokenData": {
    "standard": "ERC-20",
    "contract": "0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48",
    "amount": "1000000",
    "metadata": {
      "symbol": "USDC",
      "decimals": 6,
      "name": "USD Coin"
    }
  },
  "metadata": {
    "description": "USDC payment"
  }
}
```

### Example 3: NFT Transfer (ERC-721)

```json
{
  "version": "1.0.0",
  "chainId": 1,
  "transactionType": "nftTransfer",
  "timestamp": "2025-12-25T12:00:00Z",
  "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "to": "0xBC4CA0EdA7647A8aB7C2061c2E118A18a936f13D",
  "value": "0",
  "data": "0x42842e0e000000000000000000000000742d35cc6634c0532925a3b844bc9e7595f0beb0000000000000000000000005aaeb6053f3e94c9b9a09f33669435e7ef1beaed0000000000000000000000000000000000000000000000000000000000000539",
  "nonce": 44,
  "gasLimit": "100000",
  "gasPrice": "30000000000",
  "signature": {
    "r": "0xaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
    "s": "0xbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
    "v": 27
  },
  "hash": "0x2222222222222222222222222222222222222222222222222222222222222222",
  "tokenData": {
    "standard": "ERC-721",
    "contract": "0xBC4CA0EdA7647A8aB7C2061c2E118A18a936f13D",
    "amount": "1",
    "tokenId": "1337",
    "metadata": {
      "name": "Bored Ape #1337",
      "collection": "Bored Ape Yacht Club",
      "uri": "ipfs://QmeSjSinHpPnmXmspMjwiXyN6zS4E9zccariGR3jxcaWtq/1337"
    }
  },
  "metadata": {
    "description": "NFT sale"
  }
}
```

### Example 4: Multi-Token Transfer (ERC-1155)

```json
{
  "version": "1.0.0",
  "chainId": 137,
  "transactionType": "tokenTransfer",
  "timestamp": "2025-12-25T13:00:00Z",
  "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "to": "0x2953399124F0cBB46d2CbACD8A89cF0599974963",
  "value": "0",
  "data": "0xf242432a000000000000000000000000742d35cc6634c0532925a3b844bc9e7595f0beb0000000000000000000000005aaeb6053f3e94c9b9a09f33669435e7ef1beaed00000000000000000000000000000000000000000000000000000000000000010000000000000000000000000000000000000000000000000000000000000005",
  "nonce": 15,
  "gasLimit": "150000",
  "gasPrice": "50000000000",
  "signature": {
    "r": "0xcccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
    "s": "0xdddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd",
    "v": 28
  },
  "hash": "0x3333333333333333333333333333333333333333333333333333333333333333",
  "tokenData": {
    "standard": "ERC-1155",
    "contract": "0x2953399124F0cBB46d2CbACD8A89cF0599974963",
    "amount": "5",
    "tokenId": "1",
    "metadata": {
      "name": "Gaming Item #1",
      "type": "fungible",
      "uri": "https://api.example.com/metadata/1"
    }
  },
  "metadata": {
    "description": "In-game item transfer",
    "game": "MetaverseRPG"
  }
}
```

---

## Compliance & Security

### Security Requirements

1. **Signature Verification**: All transactions must have valid ECDSA signatures
2. **Nonce Management**: Prevent replay attacks with proper nonce handling
3. **Gas Limits**: Set reasonable gas limits to prevent out-of-gas errors
4. **Contract Verification**: Verify token contracts before interaction

### Data Privacy

1. **Personal Data**: Do not include personally identifiable information in transaction metadata
2. **Encryption**: Use end-to-end encryption for sensitive metadata
3. **GDPR Compliance**: Ensure compliance with data protection regulations

---

## Version History

- **v1.0.0** (December 2025): Initial release
  - Base transaction format
  - ERC-20, ERC-721, ERC-1155 support
  - JSON schema validation
  - Cross-chain compatibility

---

**弘益人間 · Benefit All Humanity**

© 2025 WIA - World Interoperability Alliance
MIT License
