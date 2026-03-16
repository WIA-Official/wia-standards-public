# WIA-SEC-006: Blockchain Security - PHASE 2 DATA FORMAT

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-25
**Category:** Security (SEC)

---

## 1. Introduction

This document specifies data formats and structures for blockchain security, including block data, transaction formats, wallet structures, security audit reports, and certificate schemas.

---

## 2. Block Data Format

### 2.1 Generic Block Structure

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "BlockchainBlock",
  "type": "object",
  "required": ["index", "timestamp", "transactions", "previousHash", "hash", "nonce"],
  "properties": {
    "index": {
      "type": "integer",
      "minimum": 0,
      "description": "Block number in the chain"
    },
    "timestamp": {
      "type": "integer",
      "description": "Unix timestamp when block was created"
    },
    "transactions": {
      "type": "array",
      "items": {
        "$ref": "#/definitions/Transaction"
      },
      "description": "List of transactions in this block"
    },
    "previousHash": {
      "type": "string",
      "pattern": "^0x[a-fA-F0-9]{64}$",
      "description": "Hash of the previous block"
    },
    "hash": {
      "type": "string",
      "pattern": "^0x[a-fA-F0-9]{64}$",
      "description": "Hash of this block"
    },
    "nonce": {
      "type": "integer",
      "minimum": 0,
      "description": "Proof of Work nonce"
    },
    "difficulty": {
      "type": "integer",
      "minimum": 1,
      "description": "Mining difficulty target"
    },
    "miner": {
      "type": "string",
      "pattern": "^0x[a-fA-F0-9]{40}$",
      "description": "Address of block miner"
    },
    "gasLimit": {
      "type": "integer",
      "description": "Maximum gas allowed in this block"
    },
    "gasUsed": {
      "type": "integer",
      "description": "Actual gas used by all transactions"
    },
    "stateRoot": {
      "type": "string",
      "pattern": "^0x[a-fA-F0-9]{64}$",
      "description": "Merkle root of state tree"
    },
    "transactionsRoot": {
      "type": "string",
      "pattern": "^0x[a-fA-F0-9]{64}$",
      "description": "Merkle root of transactions"
    },
    "receiptsRoot": {
      "type": "string",
      "pattern": "^0x[a-fA-F0-9]{64}$",
      "description": "Merkle root of transaction receipts"
    }
  }
}
```

### 2.2 Example Block

```json
{
  "index": 15537393,
  "timestamp": 1703505600,
  "transactions": [
    {
      "hash": "0x1234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef",
      "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
      "to": "0x8ba1f109551bD432803012645Ac136ddd64DBA72",
      "value": "1000000000000000000",
      "gas": 21000,
      "gasPrice": "20000000000",
      "nonce": 5
    }
  ],
  "previousHash": "0xabcdef1234567890abcdef1234567890abcdef1234567890abcdef1234567890",
  "hash": "0x0000000000000abc1234567890abcdef1234567890abcdef1234567890abcdef",
  "nonce": 123456,
  "difficulty": 17555997533,
  "miner": "0x5A0b54D5dc17e0AadC383d2db43B0a0D3E029c4c",
  "gasLimit": 30000000,
  "gasUsed": 21000,
  "stateRoot": "0xd7f8974fb5ac78d9ac099b9ad5018bedc2ce0a72dad1827a1709da30580f0544",
  "transactionsRoot": "0x56e81f171bcc55a6ff8345e692c0f86e5b48e01b996cadc001622fb5e363b421",
  "receiptsRoot": "0x56e81f171bcc55a6ff8345e692c0f86e5b48e01b996cadc001622fb5e363b421"
}
```

---

## 3. Transaction Data Format

### 3.1 Transaction Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Transaction",
  "type": "object",
  "required": ["hash", "from", "to", "value", "nonce"],
  "properties": {
    "hash": {
      "type": "string",
      "pattern": "^0x[a-fA-F0-9]{64}$",
      "description": "Transaction hash"
    },
    "from": {
      "type": "string",
      "pattern": "^0x[a-fA-F0-9]{40}$",
      "description": "Sender address"
    },
    "to": {
      "type": "string",
      "pattern": "^0x[a-fA-F0-9]{40}$",
      "description": "Recipient address (null for contract creation)"
    },
    "value": {
      "type": "string",
      "description": "Amount transferred in wei (as string to avoid precision loss)"
    },
    "gas": {
      "type": "integer",
      "minimum": 21000,
      "description": "Gas limit for transaction"
    },
    "gasPrice": {
      "type": "string",
      "description": "Gas price in wei"
    },
    "maxFeePerGas": {
      "type": "string",
      "description": "EIP-1559 max fee per gas"
    },
    "maxPriorityFeePerGas": {
      "type": "string",
      "description": "EIP-1559 max priority fee"
    },
    "nonce": {
      "type": "integer",
      "minimum": 0,
      "description": "Transaction nonce"
    },
    "data": {
      "type": "string",
      "pattern": "^0x[a-fA-F0-9]*$",
      "description": "Transaction data (contract call or deployment)"
    },
    "v": {
      "type": "integer",
      "description": "Signature recovery id"
    },
    "r": {
      "type": "string",
      "pattern": "^0x[a-fA-F0-9]{64}$",
      "description": "Signature r value"
    },
    "s": {
      "type": "string",
      "pattern": "^0x[a-fA-F0-9]{64}$",
      "description": "Signature s value"
    },
    "chainId": {
      "type": "integer",
      "description": "Chain ID (EIP-155)"
    },
    "type": {
      "type": "integer",
      "enum": [0, 1, 2],
      "description": "Transaction type: 0=legacy, 1=EIP-2930, 2=EIP-1559"
    }
  }
}
```

---

## 4. Wallet Data Formats

### 4.1 HD Wallet Structure

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "HDWallet",
  "type": "object",
  "required": ["mnemonic", "seed", "masterKey", "accounts"],
  "properties": {
    "mnemonic": {
      "type": "object",
      "required": ["words", "language"],
      "properties": {
        "words": {
          "type": "array",
          "items": {"type": "string"},
          "minItems": 12,
          "maxItems": 24,
          "description": "BIP39 mnemonic words"
        },
        "language": {
          "type": "string",
          "enum": ["english", "chinese_simplified", "chinese_traditional", "french", "italian", "japanese", "korean", "spanish"],
          "default": "english"
        }
      }
    },
    "seed": {
      "type": "string",
      "pattern": "^[a-fA-F0-9]{128}$",
      "description": "512-bit seed derived from mnemonic"
    },
    "masterKey": {
      "type": "object",
      "required": ["privateKey", "chainCode"],
      "properties": {
        "privateKey": {
          "type": "string",
          "pattern": "^[a-fA-F0-9]{64}$"
        },
        "publicKey": {
          "type": "string",
          "pattern": "^[a-fA-F0-9]{66,130}$"
        },
        "chainCode": {
          "type": "string",
          "pattern": "^[a-fA-F0-9]{64}$"
        }
      }
    },
    "accounts": {
      "type": "array",
      "items": {
        "$ref": "#/definitions/HDAccount"
      }
    }
  },
  "definitions": {
    "HDAccount": {
      "type": "object",
      "required": ["path", "address", "publicKey"],
      "properties": {
        "path": {
          "type": "string",
          "pattern": "^m(/[0-9]+'?)+$",
          "description": "BIP32 derivation path (e.g., m/44'/60'/0'/0/0)"
        },
        "address": {
          "type": "string",
          "pattern": "^0x[a-fA-F0-9]{40}$"
        },
        "publicKey": {
          "type": "string",
          "pattern": "^0x[a-fA-F0-9]{66,130}$"
        },
        "index": {
          "type": "integer",
          "minimum": 0
        }
      }
    }
  }
}
```

### 4.2 Multi-Signature Wallet

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "MultiSigWallet",
  "type": "object",
  "required": ["contractAddress", "owners", "required", "threshold"],
  "properties": {
    "contractAddress": {
      "type": "string",
      "pattern": "^0x[a-fA-F0-9]{40}$",
      "description": "Multi-sig contract address"
    },
    "owners": {
      "type": "array",
      "items": {
        "type": "object",
        "required": ["address", "name"],
        "properties": {
          "address": {
            "type": "string",
            "pattern": "^0x[a-fA-F0-9]{40}$"
          },
          "name": {
            "type": "string",
            "description": "Owner identifier"
          },
          "publicKey": {
            "type": "string",
            "pattern": "^0x[a-fA-F0-9]{66,130}$"
          }
        }
      },
      "minItems": 2,
      "description": "List of wallet owners"
    },
    "required": {
      "type": "integer",
      "minimum": 1,
      "description": "Number of signatures required (M in M-of-N)"
    },
    "threshold": {
      "type": "integer",
      "minimum": 2,
      "description": "Total number of owners (N in M-of-N)"
    },
    "pendingTransactions": {
      "type": "array",
      "items": {
        "$ref": "#/definitions/PendingTransaction"
      }
    }
  },
  "definitions": {
    "PendingTransaction": {
      "type": "object",
      "required": ["txId", "to", "value", "data", "signatures"],
      "properties": {
        "txId": {
          "type": "integer",
          "description": "Transaction ID in multi-sig contract"
        },
        "to": {
          "type": "string",
          "pattern": "^0x[a-fA-F0-9]{40}$"
        },
        "value": {
          "type": "string",
          "description": "Amount in wei"
        },
        "data": {
          "type": "string",
          "pattern": "^0x[a-fA-F0-9]*$"
        },
        "signatures": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "signer": {
                "type": "string",
                "pattern": "^0x[a-fA-F0-9]{40}$"
              },
              "signature": {
                "type": "string",
                "pattern": "^0x[a-fA-F0-9]{130}$"
              },
              "timestamp": {
                "type": "integer"
              }
            }
          }
        },
        "executed": {
          "type": "boolean",
          "default": false
        }
      }
    }
  }
}
```

---

## 5. Smart Contract Audit Report Format

### 5.1 Audit Report Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "SmartContractAuditReport",
  "type": "object",
  "required": ["reportId", "contractInfo", "auditor", "auditDate", "findings", "conclusion"],
  "properties": {
    "reportId": {
      "type": "string",
      "pattern": "^WIA-SEC-006-AUDIT-[0-9]{8}$",
      "description": "Unique audit report identifier"
    },
    "contractInfo": {
      "type": "object",
      "required": ["name", "version", "address", "sourceCode"],
      "properties": {
        "name": {
          "type": "string",
          "description": "Contract name"
        },
        "version": {
          "type": "string",
          "description": "Contract version"
        },
        "address": {
          "type": "string",
          "pattern": "^0x[a-fA-F0-9]{40}$",
          "description": "Deployed contract address"
        },
        "sourceCode": {
          "type": "string",
          "description": "Contract source code or hash"
        },
        "compiler": {
          "type": "string",
          "description": "Solidity compiler version"
        },
        "blockchain": {
          "type": "string",
          "enum": ["ethereum", "bsc", "polygon", "arbitrum", "optimism", "avalanche"],
          "description": "Target blockchain"
        }
      }
    },
    "auditor": {
      "type": "object",
      "required": ["company", "auditors", "certification"],
      "properties": {
        "company": {
          "type": "string",
          "description": "Audit firm name"
        },
        "auditors": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "name": {"type": "string"},
              "certification": {"type": "string"}
            }
          }
        },
        "certification": {
          "type": "string",
          "description": "Auditor certification (CertiK, Trail of Bits, etc.)"
        }
      }
    },
    "auditDate": {
      "type": "object",
      "required": ["start", "end"],
      "properties": {
        "start": {
          "type": "string",
          "format": "date-time"
        },
        "end": {
          "type": "string",
          "format": "date-time"
        }
      }
    },
    "scope": {
      "type": "array",
      "items": {"type": "string"},
      "description": "List of files/contracts audited"
    },
    "methodology": {
      "type": "array",
      "items": {"type": "string"},
      "description": "Audit methodology (manual review, automated tools, fuzzing, etc.)"
    },
    "findings": {
      "type": "array",
      "items": {
        "$ref": "#/definitions/Finding"
      }
    },
    "summary": {
      "type": "object",
      "properties": {
        "critical": {"type": "integer"},
        "high": {"type": "integer"},
        "medium": {"type": "integer"},
        "low": {"type": "integer"},
        "informational": {"type": "integer"}
      }
    },
    "conclusion": {
      "type": "object",
      "required": ["riskLevel", "recommendation"],
      "properties": {
        "riskLevel": {
          "type": "string",
          "enum": ["CRITICAL", "HIGH", "MEDIUM", "LOW", "MINIMAL"],
          "description": "Overall risk assessment"
        },
        "recommendation": {
          "type": "string",
          "description": "Auditor's final recommendation"
        },
        "fixedIssues": {
          "type": "integer",
          "description": "Number of issues fixed during audit"
        }
      }
    }
  },
  "definitions": {
    "Finding": {
      "type": "object",
      "required": ["id", "title", "severity", "description", "location", "status"],
      "properties": {
        "id": {
          "type": "string",
          "description": "Finding identifier"
        },
        "title": {
          "type": "string",
          "description": "Short title of finding"
        },
        "severity": {
          "type": "string",
          "enum": ["CRITICAL", "HIGH", "MEDIUM", "LOW", "INFORMATIONAL"]
        },
        "category": {
          "type": "string",
          "enum": ["Reentrancy", "Access Control", "Arithmetic", "Front-Running", "Denial of Service", "Logic Error", "Gas Optimization", "Best Practice"]
        },
        "description": {
          "type": "string",
          "description": "Detailed description of the issue"
        },
        "location": {
          "type": "object",
          "properties": {
            "file": {"type": "string"},
            "line": {"type": "integer"},
            "function": {"type": "string"}
          }
        },
        "impact": {
          "type": "string",
          "description": "Potential impact if exploited"
        },
        "recommendation": {
          "type": "string",
          "description": "How to fix the issue"
        },
        "status": {
          "type": "string",
          "enum": ["Open", "Fixed", "Acknowledged", "Disputed"],
          "description": "Current status of finding"
        },
        "proof_of_concept": {
          "type": "string",
          "description": "PoC code demonstrating the vulnerability"
        }
      }
    }
  }
}
```

### 5.2 Example Audit Report

```json
{
  "reportId": "WIA-SEC-006-AUDIT-20251225",
  "contractInfo": {
    "name": "MyDeFiProtocol",
    "version": "1.0.0",
    "address": "0x1234567890123456789012345678901234567890",
    "sourceCode": "https://github.com/myprotocol/contracts",
    "compiler": "0.8.20",
    "blockchain": "ethereum"
  },
  "auditor": {
    "company": "WIA Security Labs",
    "auditors": [
      {"name": "Alice Smith", "certification": "OSCP, CEH"},
      {"name": "Bob Johnson", "certification": "CISSP"}
    ],
    "certification": "WIA-SEC-006 Certified Auditor"
  },
  "auditDate": {
    "start": "2025-12-01T00:00:00Z",
    "end": "2025-12-15T23:59:59Z"
  },
  "scope": [
    "contracts/MyToken.sol",
    "contracts/Staking.sol",
    "contracts/Governance.sol"
  ],
  "methodology": [
    "Manual code review",
    "Slither static analysis",
    "Mythril symbolic execution",
    "Echidna fuzzing",
    "Formal verification with Certora"
  ],
  "findings": [
    {
      "id": "FINDING-001",
      "title": "Reentrancy vulnerability in withdraw function",
      "severity": "CRITICAL",
      "category": "Reentrancy",
      "description": "The withdraw() function makes an external call before updating user balances, allowing reentrancy attacks.",
      "location": {
        "file": "contracts/Staking.sol",
        "line": 45,
        "function": "withdraw(uint256 amount)"
      },
      "impact": "Attacker can drain all funds from the contract",
      "recommendation": "Implement Checks-Effects-Interactions pattern or use ReentrancyGuard",
      "status": "Fixed",
      "proof_of_concept": "// Attacker contract...\ncontract Attacker {\n  function attack() { victim.withdraw(1 ether); }\n  receive() { victim.withdraw(1 ether); }\n}"
    },
    {
      "id": "FINDING-002",
      "title": "Missing access control on critical function",
      "severity": "HIGH",
      "category": "Access Control",
      "description": "The setRewardRate() function lacks onlyOwner modifier",
      "location": {
        "file": "contracts/Staking.sol",
        "line": 78,
        "function": "setRewardRate(uint256 newRate)"
      },
      "impact": "Anyone can manipulate reward rates",
      "recommendation": "Add onlyOwner modifier",
      "status": "Fixed"
    }
  ],
  "summary": {
    "critical": 1,
    "high": 1,
    "medium": 3,
    "low": 5,
    "informational": 8
  },
  "conclusion": {
    "riskLevel": "LOW",
    "recommendation": "After addressing critical and high severity issues, the contracts are suitable for mainnet deployment. Continue monitoring and implement bug bounty program.",
    "fixedIssues": 2
  }
}
```

---

## 6. Security Certificate Format

### 6.1 Certificate Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "BlockchainSecurityCertificate",
  "type": "object",
  "required": ["certificateId", "issuer", "subject", "certType", "securityLevel", "issuanceDate", "expiryDate", "blockchainRecord"],
  "properties": {
    "certificateId": {
      "type": "string",
      "pattern": "^WIA-SEC-006-CERT-[0-9]{10}$",
      "description": "Unique certificate identifier"
    },
    "issuer": {
      "type": "object",
      "required": ["name", "did"],
      "properties": {
        "name": {
          "type": "string",
          "default": "WIA (World Certification Industry Association)"
        },
        "did": {
          "type": "string",
          "pattern": "^did:wia:[a-z0-9]+$",
          "description": "Decentralized Identifier of issuer"
        },
        "publicKey": {
          "type": "string",
          "description": "Issuer's public key for verification"
        }
      }
    },
    "subject": {
      "type": "object",
      "required": ["entityName", "contractAddress"],
      "properties": {
        "entityName": {
          "type": "string",
          "description": "Name of certified entity"
        },
        "contractAddress": {
          "type": "string",
          "pattern": "^0x[a-fA-F0-9]{40}$",
          "description": "Smart contract address"
        },
        "website": {
          "type": "string",
          "format": "uri"
        }
      }
    },
    "certType": {
      "type": "string",
      "enum": ["SmartContractAudit", "WalletSecurity", "NodeValidation", "Compliance", "FullStack"],
      "description": "Type of certification"
    },
    "securityLevel": {
      "type": "string",
      "enum": ["Basic", "Standard", "Advanced", "Enterprise"],
      "description": "Security certification level"
    },
    "issuanceDate": {
      "type": "string",
      "format": "date-time"
    },
    "expiryDate": {
      "type": "string",
      "format": "date-time"
    },
    "auditReports": {
      "type": "array",
      "items": {
        "type": "string",
        "description": "Reference to audit report IDs"
      }
    },
    "blockchainRecord": {
      "type": "object",
      "required": ["chain", "txHash", "blockNumber"],
      "properties": {
        "chain": {
          "type": "string",
          "enum": ["ethereum", "polygon", "bsc"],
          "description": "Blockchain where certificate is recorded"
        },
        "txHash": {
          "type": "string",
          "pattern": "^0x[a-fA-F0-9]{64}$",
          "description": "Transaction hash of certificate record"
        },
        "blockNumber": {
          "type": "integer",
          "description": "Block number where certificate was recorded"
        },
        "certificateHash": {
          "type": "string",
          "pattern": "^0x[a-fA-F0-9]{64}$",
          "description": "Hash of certificate data"
        }
      }
    },
    "signature": {
      "type": "object",
      "required": ["algorithm", "value"],
      "properties": {
        "algorithm": {
          "type": "string",
          "enum": ["ECDSA-secp256k1", "EdDSA-ed25519"],
          "default": "ECDSA-secp256k1"
        },
        "value": {
          "type": "string",
          "pattern": "^0x[a-fA-F0-9]{130}$",
          "description": "Digital signature of certificate"
        }
      }
    }
  }
}
```

---

## 7. Key Storage Format (Encrypted)

### 7.1 Encrypted Private Key Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "EncryptedPrivateKey",
  "type": "object",
  "required": ["version", "id", "address", "crypto"],
  "properties": {
    "version": {
      "type": "integer",
      "default": 3,
      "description": "Keystore version (Web3 Secret Storage Definition)"
    },
    "id": {
      "type": "string",
      "format": "uuid",
      "description": "Unique identifier for this key"
    },
    "address": {
      "type": "string",
      "pattern": "^[a-fA-F0-9]{40}$",
      "description": "Ethereum address (without 0x prefix)"
    },
    "crypto": {
      "type": "object",
      "required": ["cipher", "ciphertext", "cipherparams", "kdf", "kdfparams", "mac"],
      "properties": {
        "cipher": {
          "type": "string",
          "enum": ["aes-128-ctr", "aes-256-gcm"],
          "default": "aes-128-ctr"
        },
        "ciphertext": {
          "type": "string",
          "pattern": "^[a-fA-F0-9]+$",
          "description": "Encrypted private key"
        },
        "cipherparams": {
          "type": "object",
          "properties": {
            "iv": {
              "type": "string",
              "pattern": "^[a-fA-F0-9]{32}$",
              "description": "Initialization vector"
            }
          }
        },
        "kdf": {
          "type": "string",
          "enum": ["pbkdf2", "scrypt", "argon2id"],
          "description": "Key derivation function"
        },
        "kdfparams": {
          "type": "object",
          "description": "Parameters for KDF"
        },
        "mac": {
          "type": "string",
          "pattern": "^[a-fA-F0-9]{64}$",
          "description": "Message authentication code"
        }
      }
    }
  }
}
```

### 7.2 Example Encrypted Key (Web3 Keystore Format)

```json
{
  "version": 3,
  "id": "3198bc9c-6672-5ab3-d995-4942343ae5b6",
  "address": "008aeeda4d805471df9b2a5b0f38a0c3bcba786b",
  "crypto": {
    "ciphertext": "5318b4d5bcd28de64ee5559e671353e16f075ecae9f99c7a79a38af5f869aa46",
    "cipherparams": {
      "iv": "6087dab2f9fdbbfaddc31a909735c1e6"
    },
    "cipher": "aes-128-ctr",
    "kdf": "scrypt",
    "kdfparams": {
      "dklen": 32,
      "salt": "ae3cd4e7013836a3df6bd7241b12db061dbe2c6785853cce422d148a624ce0bd",
      "n": 262144,
      "r": 8,
      "p": 1
    },
    "mac": "517ead924a9d0dc3124507e3393d175ce3ff7c1e96529c6c555ce9e51205e9b2"
  }
}
```

---

## 8. Consensus Security Metrics

### 8.1 Network Security Metrics Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "ConsensusSecurityMetrics",
  "type": "object",
  "required": ["timestamp", "blockchain", "consensusType", "metrics"],
  "properties": {
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "blockchain": {
      "type": "string",
      "description": "Blockchain network name"
    },
    "consensusType": {
      "type": "string",
      "enum": ["PoW", "PoS", "DPoS", "PoA", "Hybrid"]
    },
    "metrics": {
      "type": "object",
      "properties": {
        "hashRate": {
          "type": "object",
          "properties": {
            "total": {"type": "number", "description": "Total network hash rate (TH/s)"},
            "topMinerPercentage": {"type": "number", "description": "Percentage of largest miner"},
            "herfindahlIndex": {"type": "number", "description": "Mining centralization index"}
          }
        },
        "nakamotoCoefficient": {
          "type": "integer",
          "description": "Minimum entities needed to control 51% of network"
        },
        "attack51Cost": {
          "type": "number",
          "description": "Estimated cost to execute 51% attack (USD)"
        },
        "blockTime": {
          "type": "number",
          "description": "Average block time (seconds)"
        },
        "confirmationTime": {
          "type": "number",
          "description": "Time for transaction finality (seconds)"
        },
        "reorgDepth": {
          "type": "integer",
          "description": "Maximum observed chain reorganization depth"
        }
      }
    }
  }
}
```

---

## 9. Vulnerability Report Format

### 9.1 Vulnerability Disclosure Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "VulnerabilityReport",
  "type": "object",
  "required": ["reportId", "reporter", "targetContract", "vulnerability", "severity", "disclosure"],
  "properties": {
    "reportId": {
      "type": "string",
      "description": "Unique vulnerability report ID"
    },
    "reporter": {
      "type": "object",
      "properties": {
        "name": {"type": "string"},
        "email": {"type": "string", "format": "email"},
        "pgpKey": {"type": "string"}
      }
    },
    "targetContract": {
      "type": "object",
      "properties": {
        "name": {"type": "string"},
        "address": {"type": "string", "pattern": "^0x[a-fA-F0-9]{40}$"},
        "blockchain": {"type": "string"}
      }
    },
    "vulnerability": {
      "type": "object",
      "required": ["title", "description", "type"],
      "properties": {
        "title": {"type": "string"},
        "description": {"type": "string"},
        "type": {"type": "string", "enum": ["Reentrancy", "Access Control", "Arithmetic", "Logic Error", "Other"]},
        "affectedFunctions": {"type": "array", "items": {"type": "string"}},
        "proofOfConcept": {"type": "string"}
      }
    },
    "severity": {
      "type": "string",
      "enum": ["CRITICAL", "HIGH", "MEDIUM", "LOW"]
    },
    "disclosure": {
      "type": "object",
      "properties": {
        "reportDate": {"type": "string", "format": "date-time"},
        "publicDisclosureDate": {"type": "string", "format": "date-time"},
        "status": {"type": "string", "enum": ["Reported", "Acknowledged", "Fixed", "Disclosed"]},
        "bountyAmount": {"type": "number"}
      }
    }
  }
}
```

---

**弘益人間 (Benefit All Humanity)**

© 2025 WIA (World Certification Industry Association)
