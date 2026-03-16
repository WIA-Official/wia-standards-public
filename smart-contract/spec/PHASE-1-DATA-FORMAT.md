# WIA-FIN-007 Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** Final
**Last Updated:** 2025-01-20

## Table of Contents

1. [Overview](#overview)
2. [Contract Metadata Schema](#contract-metadata-schema)
3. [ABI Extensions](#abi-extensions)
4. [Storage Layout Standards](#storage-layout-standards)
5. [Event and Error Standards](#event-and-error-standards)
6. [Bytecode Requirements](#bytecode-requirements)
7. [Compliance Validation](#compliance-validation)

## Overview

Phase 1 of WIA-FIN-007 defines standardized data formats for smart contracts, enabling better tooling, security analysis, and ecosystem integration. All WIA-compliant contracts MUST implement these data format standards.

### Goals

- **Machine-readable metadata** for automated analysis and tooling
- **Extended ABI** with validation and security information
- **Optimized storage layouts** for gas efficiency
- **Standardized events and errors** for consistent logging
- **Bytecode validation** for security and compatibility

## Contract Metadata Schema

### JSON Schema

All WIA-compliant contracts MUST include a metadata JSON file following this schema:

```json
{
  "$schema": "https://wia.org/schemas/contract-metadata/v1.0.0",
  "wiaVersion": "1.0.0",
  "contractInfo": {
    "name": string,
    "type": string,
    "version": string (semver),
    "description": string,
    "author": string,
    "license": string,
    "repository": string (URL),
    "documentation": string (URL)
  },
  "security": {
    "audited": boolean,
    "auditor": string,
    "auditDate": string (ISO 8601),
    "auditReport": string (URL),
    "vulnerabilities": array,
    "securityLevel": "low" | "medium" | "high",
    "reentrancyGuard": boolean,
    "overflowProtection": boolean,
    "accessControl": {
      "type": string,
      "roles": array<string>
    },
    "pausable": boolean,
    "timelocked": boolean
  },
  "deployment": {
    "networks": array<NetworkDeployment>,
    "upgradeable": boolean,
    "upgradePattern": string,
    "proxyAddress": string (optional),
    "implementationAddress": string (optional),
    "adminAddress": string (optional),
    "initializer": string (optional),
    "constructorArgs": array (optional)
  },
  "features": {
    "mintable": boolean,
    "burnable": boolean,
    "pausable": boolean,
    "snapshot": boolean,
    "permit": boolean,
    "flashMint": boolean,
    "voting": boolean
  },
  "gasOptimization": {
    "level": "low" | "medium" | "high",
    "techniques": array<string>,
    "estimatedDeployGas": number,
    "estimatedTransferGas": number,
    "benchmarks": object
  },
  "standards": {
    "implements": array<string>,
    "compatible": array<string>
  },
  "dependencies": object
}
```

### Network Deployment Schema

```typescript
interface NetworkDeployment {
  chainId: number;
  name: string;
  address: string;
  deployedAt: string; // ISO 8601
  blockNumber: number;
  transactionHash: string;
}
```

## ABI Extensions

### Extended Function ABI

WIA extends the standard Ethereum ABI with additional metadata:

```json
{
  "type": "function",
  "name": string,
  "inputs": array<ExtendedParameter>,
  "outputs": array<ExtendedParameter>,
  "stateMutability": "pure" | "view" | "nonpayable" | "payable",
  "wiaExtensions": {
    "accessControl": string,
    "gasEstimate": {
      "min": number,
      "avg": number,
      "max": number
    },
    "reentrancyProtected": boolean,
    "pausable": boolean,
    "emitsEvents": array<string>,
    "modifiers": array<string>,
    "securityChecks": array<string>,
    "errorCodes": object,
    "documentation": {
      "notice": string,
      "dev": string,
      "param": object,
      "return": string
    },
    "examples": array<CodeExample>
  }
}
```

### Extended Parameter Schema

```json
{
  "name": string,
  "type": string,
  "internalType": string,
  "wiaValidation": {
    "notZeroAddress": boolean,
    "notContractAddress": boolean,
    "checkBlacklist": boolean,
    "minValue": string,
    "maxValue": string,
    "checkBalance": boolean
  },
  "description": string
}
```

## Storage Layout Standards

### Slot Allocation

WIA-FIN-007 defines standardized storage slot allocation:

- **Slots 0-9:** Packed configuration and owner data
- **Slots 10-49:** Core contract state
- **Slots 50-99:** Feature-specific state
- **Slots 100-199:** Reserved for future upgrades (`__gap`)
- **Slots 200+:** Extension data

### Packing Rules

1. **Pack small types together** in the same slot
2. **Order by size** (smallest to largest within a slot)
3. **Group related variables** for better code organization
4. **Leave gaps** for upgradeability (100-slot gaps minimum)

### Example

```solidity
// Efficient storage layout
contract WIAStorage {
    // Slot 0: Packed (23 bytes used, 9 bytes padding)
    address private _owner;         // 20 bytes
    bool private _paused;           // 1 byte
    bool private _initialized;      // 1 byte
    uint8 private _decimals;        // 1 byte

    // Slot 1-9: Configuration
    mapping(address => uint256) private _balances;
    mapping(address => mapping(address => uint256)) private _allowances;

    // Slot 10-49: Core state
    uint256 private _totalSupply;

    // Slot 50-99: Features
    mapping(address => bool) private _blacklist;

    // Slot 100-199: Reserved for upgrades
    uint256[100] private __gap;

    // Slot 200+: Extensions
}
```

## Event and Error Standards

### Standard Events

All WIA contracts MUST emit these standard events:

```solidity
event WIAContractDeployed(
    address indexed contractAddress,
    string contractType,
    string version,
    uint256 timestamp
);

event WIAContractUpgraded(
    address indexed proxy,
    address indexed oldImplementation,
    address indexed newImplementation,
    string version,
    uint256 timestamp
);

event WIAOwnershipTransferred(
    address indexed previousOwner,
    address indexed newOwner,
    uint256 timestamp
);

event WIAEmergencyStop(
    address indexed triggeredBy,
    string reason,
    bool paused,
    uint256 timestamp
);

event WIAAccessGranted(
    address indexed user,
    bytes32 indexed role,
    address indexed grantedBy,
    uint256 timestamp
);

event WIAAccessRevoked(
    address indexed user,
    bytes32 indexed role,
    address indexed revokedBy,
    uint256 timestamp
);
```

### Contract-Specific Events

Contract-specific events MUST follow WIA naming convention:

```solidity
event WIATokenTransfer(
    address indexed from,
    address indexed to,
    uint256 value,
    bytes data,
    uint256 timestamp
);

event WIATokenApproval(
    address indexed owner,
    address indexed spender,
    uint256 value,
    uint256 timestamp
);
```

### Standard Errors

Use custom errors for gas efficiency:

```solidity
error WIAUnauthorized(address caller, bytes32 requiredRole);
error WIAInsufficientBalance(address account, uint256 requested, uint256 available);
error WIAInvalidAddress(address provided, string reason);
error WIAContractPaused();
error WIAInvalidAmount(uint256 amount, string reason);
error WIATransferFailed(address from, address to, uint256 amount);
error WIAApprovalFailed(address owner, address spender, uint256 amount);
error WIABlacklisted(address account);
error WIAUpgradeFailed(address implementation, string reason);
error WIAInitializationFailed(string reason);
```

## Bytecode Requirements

### Compiler Settings

Recommended Solidity compiler configuration:

```json
{
  "solidity": {
    "version": "0.8.20",
    "settings": {
      "optimizer": {
        "enabled": true,
        "runs": 200
      },
      "viaIR": true,
      "evmVersion": "shanghai",
      "metadata": {
        "bytecodeHash": "ipfs",
        "appendCBOR": true
      }
    }
  }
}
```

### Size Limits

- Maximum bytecode size: 24,576 bytes (24 KB)
- Contracts exceeding this MUST use diamond pattern or modular architecture

### Immutables

Use `immutable` keyword for deployment-time constants to save gas.

## Compliance Validation

### Automated Validation

Use WIA CLI to validate Phase 1 compliance:

```bash
wia validate phase1 --contract contracts/MyContract.sol
```

### Validation Checklist

- [ ] Metadata JSON present and valid
- [ ] ABI includes WIA extensions
- [ ] Storage layout documented
- [ ] Standard events implemented
- [ ] Custom errors used (not revert strings)
- [ ] Bytecode size within limits
- [ ] NatSpec documentation complete
- [ ] Gas benchmarks provided

### Scoring

Phase 1 compliance is scored 0-100:

- **Metadata (25 points):** Complete, valid JSON schema
- **ABI Extensions (25 points):** All functions documented with WIA extensions
- **Storage Optimization (20 points):** Efficient packing, gap reservations
- **Events/Errors (15 points):** Standard events, custom errors
- **Bytecode (10 points):** Size limits, optimization settings
- **Documentation (5 points):** Complete NatSpec

**Passing Score:** 80/100

## References

- [EIP-165: Standard Interface Detection](https://eips.ethereum.org/EIPS/eip-165)
- [OpenZeppelin Contracts](https://docs.openzeppelin.com/contracts/)
- [Solidity Style Guide](https://docs.soliditylang.org/en/latest/style-guide.html)
- [Ethereum Contract ABI Specification](https://docs.soliditylang.org/en/latest/abi-spec.html)

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
