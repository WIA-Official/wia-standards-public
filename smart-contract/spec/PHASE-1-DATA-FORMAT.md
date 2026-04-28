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

---

## A.1 Worked metadata envelope (canonical)

A canonical contract-metadata document includes the contract identity (chain, address, deployer), the proxy / implementation pair when an upgradeable pattern is used, the SPDX license identifier, the ABI hash of the implementation bytecode (keccak-256 over the deployed bytecode), and the storage-layout descriptor. The metadata is anchored on chain via a `MetadataAnchored` event so that off-chain indexers can rebuild the layout without re-resolving the source code.

```json
{
  "wiaVersion": "1.0.0",
  "contractInfo": {
    "name": "WIAVault",
    "type": "vault",
    "version": "1.4.2",
    "license": "MIT",
    "documentation": "https://docs.example.com/wia-vault"
  },
  "deployment": {
    "chainId": 1,
    "proxyAddress": "0x000000000000000000000000000000000000beef",
    "implementationAddress": "0x000000000000000000000000000000000000feed",
    "upgradePattern": "UUPS"
  },
  "security": {
    "audited": true,
    "auditReport": "ipfs://bafy...",
    "reentrancyGuard": true,
    "pausable": true
  }
}
```

## A.2 Storage-layout descriptor

The storage-layout descriptor enumerates each declared variable, its slot, and its offset in the slot. WIA-conformant implementations MUST publish the descriptor so post-deployment auditors can verify upgrade compatibility and can reason about state migration when an `_authorizeUpgrade` boundary is crossed.

```json
{
  "_owner":          {"slot": 0, "offset": 0,  "size": 20, "type": "address"},
  "_paused":         {"slot": 0, "offset": 20, "size": 1,  "type": "bool"},
  "_initialized":    {"slot": 0, "offset": 21, "size": 1,  "type": "bool"},
  "_decimals":       {"slot": 0, "offset": 22, "size": 1,  "type": "uint8"},
  "_balances":       {"slot": 1, "offset": 0,  "size": 32, "type": "mapping(address => uint256)"},
  "_totalSupply":    {"slot": 10, "offset": 0, "size": 32, "type": "uint256"},
  "__gap":           {"slot": 100, "offset": 0, "size": 32 * 100, "type": "uint256[100]"}
}
```

## A.3 Standard event canonical encoding

WIA standard events are emitted with topic[0] = keccak-256 of the canonical event signature, indexed parameters in topics[1..3], and ABI-encoded non-indexed parameters in `data`. The `WIAContractDeployed` event is emitted exactly once per deployment as the first log of the deployment transaction.

## A.4 Custom-error encoding policy

WIA contracts MUST use custom errors over revert-string returns. The 4-byte selector of each error MUST be globally unique across the contract, and the error MUST emit during a `try/catch` boundary at every external call so off-chain indexers can correlate failure cause with the failing call site. The selector registry is published at `https://wiastandards.com/smart-contract/error-selectors.json`.

## A.4b Bytecode build profile

The recommended Solidity compiler profile pins version 0.8.20+, optimiser runs = 200, viaIR = true, evmVersion = `shanghai` (or chain-appropriate target), `bytecodeHash = "ipfs"`, and `appendCBOR = true`. Hosts that target older EVMs MAY downgrade `evmVersion` but MUST publish the deviation in the metadata block. The compiler settings are themselves part of the metadata so post-deployment auditors can rebuild bit-for-bit identical bytecode.

## A.4c Diamond pattern guidance

Contracts above the EIP-170 24,576-byte limit use the EIP-2535 diamond pattern. The `DiamondCut` event is emitted on every facet change with the function selectors and the new facet address; the facet registry is canonicalised through the metadata block (Phase 1 §A.1) so off-chain auditors can reconstruct the active surface from a single read.

## A.5 ABI extension v1 wire shape

The `wiaExtensions` block on each ABI entry is OPTIONAL but recommended. Tooling that does not understand the block MUST ignore unknown keys; tooling that emits the block MUST stamp a `wiaSchema` field with the URI of the schema version used so consumers can validate.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/smart-contract/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-smart-contract-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/smart-contract-host:1.0.0` ships every smart-contract envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/smart-contract.sh` ships sample envelope generators with no
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
ecosystem. Smart-contract deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
