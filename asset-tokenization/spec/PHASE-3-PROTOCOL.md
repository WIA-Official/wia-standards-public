# WIA-FIN-008 Phase 3: Smart Contract Protocol Specification

**Version:** 1.0.0
**Status:** Final
**Last Updated:** 2025-01-20
**Authors:** WIA Standards Committee

## Table of Contents

1. [Introduction](#introduction)
2. [ERC-1400 Standard](#erc-1400-standard)
3. [Contract Architecture](#contract-architecture)
4. [Compliance Module](#compliance-module)
5. [Transfer Restrictions](#transfer-restrictions)
6. [Dividend Distribution](#dividend-distribution)
7. [Governance & Voting](#governance--voting)
8. [Multi-Chain Deployment](#multi-chain-deployment)
9. [Upgradability](#upgradability)
10. [Security Best Practices](#security-best-practices)

---

## 1. Introduction

Phase 3 of WIA-FIN-008 defines on-chain smart contract protocols for compliant asset tokenization. These protocols implement ERC-1400 security token standards with automated compliance enforcement, dividend distribution, and governance mechanisms.

### Design Principles

- **Compliance First:** Transfer restrictions enforced at contract level
- **Modularity:** Separable compliance, distribution, and governance modules
- **Upgradability:** Proxy patterns for bug fixes and feature additions
- **Gas Efficiency:** Optimized for minimal transaction costs
- **Multi-Chain:** Compatible with Ethereum, Polygon, Avalanche, Arbitrum
- **Security Audited:** All contracts must pass professional security audits

---

## 2. ERC-1400 Standard

### Interface Definition

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

interface IERC1400 {
    // Transfer Validation
    function canTransfer(
        address from,
        address to,
        uint256 value,
        bytes calldata data
    ) external view returns (bool, bytes32, bytes32);

    // Partition-Based Transfers
    function transferByPartition(
        bytes32 partition,
        address to,
        uint256 value,
        bytes calldata data
    ) external returns (bytes32);

    function balanceOfByPartition(
        bytes32 partition,
        address tokenHolder
    ) external view returns (uint256);

    // Document Management
    function setDocument(
        bytes32 name,
        string calldata uri,
        bytes32 documentHash
    ) external;

    function getDocument(bytes32 name) external view returns (
        string memory uri,
        bytes32 documentHash,
        uint256 timestamp
    );

    // Controller Operations
    function controllerTransfer(
        address from,
        address to,
        uint256 value,
        bytes calldata data,
        bytes calldata operatorData
    ) external;

    function controllerRedeem(
        address tokenHolder,
        uint256 value,
        bytes calldata data,
        bytes calldata operatorData
    ) external;

    // Issuance
    function issue(
        address tokenHolder,
        uint256 value,
        bytes calldata data
    ) external;

    function redeem(
        uint256 value,
        bytes calldata data
    ) external;

    // Events
    event TransferByPartition(
        bytes32 indexed fromPartition,
        address operator,
        address indexed from,
        address indexed to,
        uint256 value,
        bytes data,
        bytes operatorData
    );

    event Document(
        bytes32 indexed name,
        string uri,
        bytes32 documentHash
    );
}
```

### Partitions

Partitions enable multiple token classes within single contract:

```solidity
bytes32 constant COMMON_PARTITION = keccak256("COMMON");
bytes32 constant PREFERRED_PARTITION = keccak256("PREFERRED");
bytes32 constant RESTRICTED_PARTITION = keccak256("RESTRICTED");

mapping(address => mapping(bytes32 => uint256)) private partitionBalances;

function transferByPartition(
    bytes32 partition,
    address to,
    uint256 value,
    bytes calldata data
) external returns (bytes32) {
    require(
        partitionBalances[msg.sender][partition] >= value,
        "Insufficient partition balance"
    );

    // Compliance check
    (bool allowed, bytes32 statusCode, ) = canTransfer(
        msg.sender,
        to,
        value,
        data
    );
    require(allowed, "Transfer not allowed");

    partitionBalances[msg.sender][partition] -= value;
    partitionBalances[to][partition] += value;

    emit TransferByPartition(
        partition,
        msg.sender,
        msg.sender,
        to,
        value,
        data,
        ""
    );

    return statusCode;
}
```

---

## 3. Contract Architecture

### Modular Design

```
AssetToken (Main Contract)
├── ERC1400 (Security Token Standard)
├── ComplianceModule (Transfer Restrictions)
├── DividendDistributor (Income Payments)
├── Governance (Token Holder Voting)
└── Pausable (Emergency Controls)
```

### Main Contract

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts-upgradeable/token/ERC20/ERC20Upgradeable.sol";
import "@openzeppelin/contracts-upgradeable/access/AccessControlUpgradeable.sol";
import "@openzeppelin/contracts-upgradeable/security/PausableUpgradeable.sol";
import "@openzeppelin/contracts-upgradeable/proxy/utils/UUPSUpgradeable.sol";

contract AssetToken is
    ERC20Upgradeable,
    AccessControlUpgradeable,
    PausableUpgradeable,
    UUPSUpgradeable
{
    bytes32 public constant ADMIN_ROLE = keccak256("ADMIN_ROLE");
    bytes32 public constant COMPLIANCE_ROLE = keccak256("COMPLIANCE_ROLE");
    bytes32 public constant ISSUER_ROLE = keccak256("ISSUER_ROLE");

    IComplianceModule public complianceModule;
    IDividendDistributor public dividendDistributor;
    IGovernance public governance;

    struct TokenMetadata {
        string name;
        string symbol;
        string assetClass;
        uint256 totalSupply;
        address issuer;
        uint256 issuanceDate;
    }

    TokenMetadata public metadata;

    function initialize(
        string memory name_,
        string memory symbol_,
        uint256 totalSupply_,
        address admin_,
        address complianceModule_
    ) public initializer {
        __ERC20_init(name_, symbol_);
        __AccessControl_init();
        __Pausable_init();
        __UUPSUpgradeable_init();

        _grantRole(DEFAULT_ADMIN_ROLE, admin_);
        _grantRole(ADMIN_ROLE, admin_);
        _grantRole(ISSUER_ROLE, admin_);

        complianceModule = IComplianceModule(complianceModule_);

        metadata = TokenMetadata({
            name: name_,
            symbol: symbol_,
            assetClass: "REAL_ESTATE",
            totalSupply: totalSupply_,
            issuer: admin_,
            issuanceDate: block.timestamp
        });

        _mint(admin_, totalSupply_);
    }

    function _authorizeUpgrade(address newImplementation)
        internal
        override
        onlyRole(ADMIN_ROLE)
    {}

    function transfer(address to, uint256 amount)
        public
        override
        whenNotPaused
        returns (bool)
    {
        // Compliance check
        (bool allowed, string memory reason) = complianceModule.canTransfer(
            msg.sender,
            to,
            amount
        );
        require(allowed, reason);

        return super.transfer(to, amount);
    }

    function transferFrom(
        address from,
        address to,
        uint256 amount
    ) public override whenNotPaused returns (bool) {
        (bool allowed, string memory reason) = complianceModule.canTransfer(
            from,
            to,
            amount
        );
        require(allowed, reason);

        return super.transferFrom(from, to, amount);
    }
}
```

---

## 4. Compliance Module

### Interface

```solidity
interface IComplianceModule {
    function canTransfer(
        address from,
        address to,
        uint256 amount
    ) external view returns (bool allowed, string memory reason);

    function setKYC(address investor, bool verified, uint256 expiry) external;
    function setAccreditation(address investor, bool accredited, uint256 expiry) external;
    function setJurisdiction(address investor, string calldata country) external;
    function setLockup(address investor, uint256 lockupExpiry) external;
}
```

### Implementation

```solidity
contract ComplianceModule is AccessControl {
    bytes32 public constant COMPLIANCE_ROLE = keccak256("COMPLIANCE_ROLE");

    struct InvestorProfile {
        bool kycVerified;
        uint256 kycExpiry;
        bool accredited;
        uint256 accreditationExpiry;
        string jurisdiction;
        uint256 lockupExpiry;
        uint256 lastTransferTime;
    }

    mapping(address => InvestorProfile) public investors;
    mapping(string => bool) public allowedJurisdictions;

    uint256 public maxOwnershipPercent = 500; // 5.00% in basis points
    uint256 public minHoldingPeriod = 30 days;
    address public token;

    constructor(address admin) {
        _grantRole(DEFAULT_ADMIN_ROLE, admin);
        _grantRole(COMPLIANCE_ROLE, admin);

        // Initialize allowed jurisdictions
        allowedJurisdictions["USA"] = true;
        allowedJurisdictions["CAN"] = true;
        allowedJurisdictions["GBR"] = true;
    }

    function canTransfer(
        address from,
        address to,
        uint256 amount
    ) external view returns (bool allowed, string memory reason) {
        InvestorProfile memory sender = investors[from];
        InvestorProfile memory recipient = investors[to];

        // Rule 1: Recipient KYC verification
        if (!recipient.kycVerified) {
            return (false, "Recipient not KYC verified");
        }

        // Rule 2: KYC expiry
        if (block.timestamp > recipient.kycExpiry) {
            return (false, "Recipient KYC expired");
        }

        // Rule 3: Accreditation requirement
        if (!recipient.accredited) {
            return (false, "Recipient not accredited");
        }

        if (block.timestamp > recipient.accreditationExpiry) {
            return (false, "Recipient accreditation expired");
        }

        // Rule 4: Jurisdiction whitelist
        if (!allowedJurisdictions[recipient.jurisdiction]) {
            return (false, "Recipient jurisdiction not allowed");
        }

        // Rule 5: Sender lockup period
        if (block.timestamp < sender.lockupExpiry) {
            return (false, "Sender lockup period active");
        }

        // Rule 6: Maximum ownership limit
        uint256 recipientBalance = IERC20(token).balanceOf(to);
        uint256 totalSupply = IERC20(token).totalSupply();
        uint256 newBalance = recipientBalance + amount;
        uint256 ownershipBasisPoints = (newBalance * 10000) / totalSupply;

        if (ownershipBasisPoints > maxOwnershipPercent) {
            return (false, "Exceeds maximum ownership limit");
        }

        // Rule 7: Minimum holding period
        if (block.timestamp < sender.lastTransferTime + minHoldingPeriod) {
            return (false, "Minimum holding period not met");
        }

        return (true, "Transfer allowed");
    }

    function setKYC(
        address investor,
        bool verified,
        uint256 expiry
    ) external onlyRole(COMPLIANCE_ROLE) {
        investors[investor].kycVerified = verified;
        investors[investor].kycExpiry = expiry;
    }

    function setAccreditation(
        address investor,
        bool accredited,
        uint256 expiry
    ) external onlyRole(COMPLIANCE_ROLE) {
        investors[investor].accredited = accredited;
        investors[investor].accreditationExpiry = expiry;
    }

    function setJurisdiction(
        address investor,
        string calldata country
    ) external onlyRole(COMPLIANCE_ROLE) {
        investors[investor].jurisdiction = country;
    }

    function setLockup(
        address investor,
        uint256 lockupExpiry
    ) external onlyRole(COMPLIANCE_ROLE) {
        investors[investor].lockupExpiry = lockupExpiry;
    }
}
```

---

## 5. Transfer Restrictions

### Common Restriction Types

| Restriction | Purpose | Implementation |
|-------------|---------|----------------|
| **KYC Requirement** | Identity verification | Check `kycVerified` flag |
| **Accreditation** | Reg D compliance | Check `accredited` flag |
| **Lockup Period** | Prevent early selling | Compare `block.timestamp` vs `lockupExpiry` |
| **Jurisdiction Filter** | Geographic restrictions | Whitelist/blacklist countries |
| **Ownership Cap** | Prevent concentration | Calculate ownership percentage |
| **Holding Period** | Prevent wash sales | Track `lastTransferTime` |
| **Trading Hours** | Regulatory requirement | Check `block.timestamp` for market hours |

### Advanced: Time-Based Restrictions

```solidity
// Trading hours: Mon-Fri 9:30 AM - 4:00 PM ET
function isWithinTradingHours() public view returns (bool) {
    uint256 dayOfWeek = (block.timestamp / 1 days + 4) % 7;
    if (dayOfWeek == 0 || dayOfWeek == 6) {
        return false; // Weekend
    }

    uint256 hourOfDay = (block.timestamp / 1 hours) % 24;
    if (hourOfDay < 14 || hourOfDay >= 21) {
        return false; // Outside 9:30 AM - 4:00 PM ET (UTC-5)
    }

    return true;
}
```

---

## 6. Dividend Distribution

### Pull-Based Distribution (Gas Efficient)

```solidity
contract DividendDistributor {
    struct Distribution {
        uint256 totalAmount;
        uint256 amountPerToken;
        uint256 timestamp;
        uint256 claimedAmount;
        address paymentToken; // USDC, USDT, DAI, etc.
    }

    Distribution[] public distributions;
    mapping(address => mapping(uint256 => bool)) public hasClaimed;

    IERC20 public assetToken;

    event DividendDistributed(
        uint256 indexed distributionId,
        uint256 totalAmount,
        uint256 amountPerToken,
        address paymentToken
    );

    event DividendClaimed(
        address indexed investor,
        uint256 indexed distributionId,
        uint256 amount
    );

    constructor(address assetToken_) {
        assetToken = IERC20(assetToken_);
    }

    // Asset manager creates distribution
    function distributeRentalIncome(
        uint256 amount,
        address paymentToken
    ) external {
        require(amount > 0, "Amount must be positive");

        uint256 totalSupply = assetToken.totalSupply();
        uint256 amountPerToken = (amount * 1e18) / totalSupply;

        // Transfer payment tokens to contract
        IERC20(paymentToken).transferFrom(
            msg.sender,
            address(this),
            amount
        );

        distributions.push(Distribution({
            totalAmount: amount,
            amountPerToken: amountPerToken,
            timestamp: block.timestamp,
            claimedAmount: 0,
            paymentToken: paymentToken
        }));

        emit DividendDistributed(
            distributions.length - 1,
            amount,
            amountPerToken,
            paymentToken
        );
    }

    // Token holders claim their dividends
    function claimDividend(uint256 distributionId) external {
        require(distributionId < distributions.length, "Invalid distribution");
        require(
            !hasClaimed[msg.sender][distributionId],
            "Already claimed"
        );

        Distribution storage dist = distributions[distributionId];

        uint256 balance = assetToken.balanceOf(msg.sender);
        require(balance > 0, "No tokens held");

        uint256 amount = (balance * dist.amountPerToken) / 1e18;
        require(amount > 0, "No dividends to claim");

        hasClaimed[msg.sender][distributionId] = true;
        dist.claimedAmount += amount;

        IERC20(dist.paymentToken).transfer(msg.sender, amount);

        emit DividendClaimed(msg.sender, distributionId, amount);
    }

    // Batch claim multiple distributions
    function claimMultiple(uint256[] calldata distributionIds) external {
        for (uint256 i = 0; i < distributionIds.length; i++) {
            uint256 distId = distributionIds[i];
            if (!hasClaimed[msg.sender][distId]) {
                claimDividend(distId);
            }
        }
    }

    // View unclaimed dividends
    function getUnclaimedDividends(address investor)
        external
        view
        returns (uint256[] memory, uint256[] memory)
    {
        uint256 count = 0;
        for (uint256 i = 0; i < distributions.length; i++) {
            if (!hasClaimed[investor][i]) {
                count++;
            }
        }

        uint256[] memory distributionIds = new uint256[](count);
        uint256[] memory amounts = new uint256[](count);

        uint256 balance = assetToken.balanceOf(investor);
        uint256 index = 0;

        for (uint256 i = 0; i < distributions.length; i++) {
            if (!hasClaimed[investor][i]) {
                distributionIds[index] = i;
                amounts[index] = (balance * distributions[i].amountPerToken) / 1e18;
                index++;
            }
        }

        return (distributionIds, amounts);
    }
}
```

---

## 7. Governance & Voting

### Token Holder Governance

```solidity
contract AssetGovernance {
    struct Proposal {
        uint256 id;
        address proposer;
        string description;
        uint256 forVotes;
        uint256 againstVotes;
        uint256 startTime;
        uint256 endTime;
        bool executed;
        ProposalType proposalType;
        bytes executionData;
    }

    enum ProposalType {
        GENERAL,
        ASSET_SALE,
        REFINANCING,
        MANAGER_CHANGE,
        COMPLIANCE_UPDATE
    }

    Proposal[] public proposals;
    mapping(uint256 => mapping(address => bool)) public hasVoted;

    IERC20 public assetToken;
    uint256 public votingPeriod = 7 days;
    uint256 public quorumPercent = 5000; // 50%
    uint256 public approvalThreshold = 6667; // 66.67%
    uint256 public proposalThreshold = 100; // 1%

    event ProposalCreated(
        uint256 indexed proposalId,
        address indexed proposer,
        string description,
        ProposalType proposalType
    );

    event Voted(
        uint256 indexed proposalId,
        address indexed voter,
        bool support,
        uint256 votes
    );

    event ProposalExecuted(uint256 indexed proposalId);

    constructor(address assetToken_) {
        assetToken = IERC20(assetToken_);
    }

    // Create proposal (requires 1% ownership)
    function propose(
        string calldata description,
        ProposalType proposalType,
        bytes calldata executionData
    ) external returns (uint256) {
        uint256 proposerBalance = assetToken.balanceOf(msg.sender);
        uint256 totalSupply = assetToken.totalSupply();

        require(
            (proposerBalance * 10000) / totalSupply >= proposalThreshold,
            "Insufficient tokens to propose"
        );

        proposals.push(Proposal({
            id: proposals.length,
            proposer: msg.sender,
            description: description,
            forVotes: 0,
            againstVotes: 0,
            startTime: block.timestamp,
            endTime: block.timestamp + votingPeriod,
            executed: false,
            proposalType: proposalType,
            executionData: executionData
        }));

        emit ProposalCreated(
            proposals.length - 1,
            msg.sender,
            description,
            proposalType
        );

        return proposals.length - 1;
    }

    // Vote on proposal
    function vote(uint256 proposalId, bool support) external {
        require(proposalId < proposals.length, "Invalid proposal");
        Proposal storage proposal = proposals[proposalId];

        require(block.timestamp <= proposal.endTime, "Voting ended");
        require(!hasVoted[proposalId][msg.sender], "Already voted");

        uint256 votes = assetToken.balanceOf(msg.sender);
        require(votes > 0, "No voting power");

        hasVoted[proposalId][msg.sender] = true;

        if (support) {
            proposal.forVotes += votes;
        } else {
            proposal.againstVotes += votes;
        }

        emit Voted(proposalId, msg.sender, support, votes);
    }

    // Execute approved proposal
    function execute(uint256 proposalId) external {
        require(proposalId < proposals.length, "Invalid proposal");
        Proposal storage proposal = proposals[proposalId];

        require(block.timestamp > proposal.endTime, "Voting ongoing");
        require(!proposal.executed, "Already executed");

        uint256 totalVotes = proposal.forVotes + proposal.againstVotes;
        uint256 totalSupply = assetToken.totalSupply();

        // Check quorum
        require(
            (totalVotes * 10000) / totalSupply >= quorumPercent,
            "Quorum not reached"
        );

        // Check approval
        require(
            (proposal.forVotes * 10000) / totalVotes >= approvalThreshold,
            "Insufficient approval"
        );

        proposal.executed = true;
        _executeProposal(proposalId);

        emit ProposalExecuted(proposalId);
    }

    function _executeProposal(uint256 proposalId) internal {
        Proposal storage proposal = proposals[proposalId];

        if (proposal.proposalType == ProposalType.ASSET_SALE) {
            // Execute asset sale logic
        } else if (proposal.proposalType == ProposalType.MANAGER_CHANGE) {
            // Change asset manager
        }
        // ... other proposal types
    }
}
```

---

## 8. Multi-Chain Deployment

### Deployment Configuration

```javascript
// hardhat.config.js
module.exports = {
  networks: {
    ethereum: {
      url: process.env.ETHEREUM_RPC,
      chainId: 1,
      gasPrice: 30_000_000_000 // 30 gwei
    },
    polygon: {
      url: process.env.POLYGON_RPC,
      chainId: 137,
      gasPrice: 50_000_000_000 // 50 gwei
    },
    avalanche: {
      url: process.env.AVALANCHE_RPC,
      chainId: 43114,
      gasPrice: 25_000_000_000 // 25 nAVAX
    },
    arbitrum: {
      url: process.env.ARBITRUM_RPC,
      chainId: 42161,
      gasPrice: 100_000_000 // 0.1 gwei
    }
  }
};
```

### Cross-Chain Bridge (LayerZero)

```solidity
import "@layerzerolabs/solidity-examples/contracts/lzApp/NonblockingLzApp.sol";

contract AssetTokenBridge is NonblockingLzApp {
    mapping(uint16 => address) public chainIdToToken;

    constructor(address lzEndpoint_) NonblockingLzApp(lzEndpoint_) {}

    function bridgeTokens(
        uint16 dstChainId,
        address recipient,
        uint256 amount
    ) external payable {
        // Burn tokens on source chain
        IERC20(token).burn(msg.sender, amount);

        // Encode payload
        bytes memory payload = abi.encode(recipient, amount);

        // Send cross-chain message
        _lzSend(
            dstChainId,
            payload,
            payable(msg.sender),
            address(0),
            bytes(""),
            msg.value
        );
    }

    function _nonblockingLzReceive(
        uint16 srcChainId,
        bytes memory,
        uint64,
        bytes memory payload
    ) internal override {
        (address recipient, uint256 amount) = abi.decode(
            payload,
            (address, uint256)
        );

        // Mint tokens on destination chain
        IERC20(token).mint(recipient, amount);
    }
}
```

---

## 9. Upgradability

### Transparent Proxy Pattern

```solidity
import "@openzeppelin/contracts/proxy/transparent/TransparentUpgradeableProxy.sol";

// Deployment script
async function deployUpgradeable() {
    const AssetToken = await ethers.getContractFactory("AssetToken");
    const implementation = await AssetToken.deploy();

    const ProxyAdmin = await ethers.getContractFactory("ProxyAdmin");
    const proxyAdmin = await ProxyAdmin.deploy();

    const proxy = await TransparentUpgradeableProxy.deploy(
        implementation.address,
        proxyAdmin.address,
        AssetToken.interface.encodeFunctionData("initialize", [
            "Asset Token",
            "ASSET",
            1000000,
            deployer.address,
            complianceModule.address
        ])
    );

    return { proxy, proxyAdmin, implementation };
}

// Upgrade process
async function upgrade(proxyAddress, newImplementationAddress) {
    const proxyAdmin = await ethers.getContractAt(
        "ProxyAdmin",
        proxyAdminAddress
    );

    await proxyAdmin.upgrade(proxyAddress, newImplementationAddress);
}
```

---

## 10. Security Best Practices

### Pre-Deployment Checklist

- ✅ **Reentrancy Guards:** All state-changing external functions protected
- ✅ **Access Controls:** Role-based permissions for sensitive operations
- ✅ **Integer Overflow:** Use Solidity 0.8+ with built-in overflow checks
- ✅ **Pausability:** Emergency pause mechanism for critical bugs
- ✅ **Input Validation:** Validate all user inputs and external calls
- ✅ **Gas Optimization:** Minimize storage operations, use memory caching
- ✅ **Event Logging:** Comprehensive events for all state changes
- ✅ **Test Coverage:** >95% line coverage, 100% critical path coverage

### Audit Requirements

| Audit Type | When Required | Estimated Cost |
|------------|---------------|----------------|
| **Internal Review** | All deployments | $0 (team) |
| **Third-Party Audit** | Production deployments | $50K-$150K |
| **Formal Verification** | High-value assets (>$10M) | $100K-$300K |
| **Bug Bounty** | Post-deployment | $10K-$100K rewards |

### Recommended Auditors

- OpenZeppelin
- Trail of Bits
- ConsenSys Diligence
- Certora (formal verification)
- Quantstamp

---

© 2025 SmileStory Inc. / WIA
弘益人間 · Benefit All Humanity
