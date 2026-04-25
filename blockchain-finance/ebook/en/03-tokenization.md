# Chapter 3: Tokenization Standards

## Asset Tokenization, Security Tokens, and Real-World Asset Protocols

### The Foundation of Programmable Finance

---

## Overview

Tokenization represents the transformation of real-world assets into digital tokens on blockchain networks. This chapter covers the technical standards, regulatory frameworks, and implementation patterns for creating compliant, interoperable tokenized assets.

---

## Token Standards Foundation

### Ethereum Token Standards

**ERC Standard Evolution:**

| Standard | Purpose | Key Features | Adoption |
|----------|---------|--------------|----------|
| ERC-20 | Fungible tokens | Transfer, approve, balanceOf | Universal |
| ERC-721 | Non-fungible tokens | Unique ownership, metadata | High |
| ERC-1155 | Multi-token | Batch operations, mixed types | Growing |
| ERC-3643 | Security tokens | Identity, compliance modules | Institutional |
| ERC-4626 | Tokenized vaults | Standardized yield-bearing | DeFi standard |
| ERC-6551 | Token bound accounts | NFTs as wallets | Emerging |

### ERC-3643: Security Token Standard

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/token/ERC20/ERC20.sol";
import "./interfaces/IIdentityRegistry.sol";
import "./interfaces/ICompliance.sol";

/**
 * @title WIA Security Token
 * @dev ERC-3643 compliant security token with transfer restrictions
 */
contract WIASecurityToken is ERC20 {
    IIdentityRegistry public identityRegistry;
    ICompliance public compliance;

    address public tokenIssuer;
    bool public tokenPaused;

    // Agent roles for compliance actions
    mapping(address => bool) public agents;

    // Frozen addresses (regulatory action)
    mapping(address => bool) public frozen;

    // Partition support for different share classes
    mapping(bytes32 => mapping(address => uint256)) public partitionBalances;
    bytes32[] public partitions;

    event TokenFrozen(address indexed account, address indexed agent);
    event TokenUnfrozen(address indexed account, address indexed agent);
    event IdentityRegistryUpdated(address indexed newRegistry);
    event ComplianceUpdated(address indexed newCompliance);
    event AgentAdded(address indexed agent);
    event AgentRemoved(address indexed agent);

    modifier onlyAgent() {
        require(agents[msg.sender], "Caller is not an agent");
        _;
    }

    modifier onlyIssuer() {
        require(msg.sender == tokenIssuer, "Caller is not the issuer");
        _;
    }

    modifier whenNotPaused() {
        require(!tokenPaused, "Token is paused");
        _;
    }

    modifier whenNotFrozen(address _account) {
        require(!frozen[_account], "Account is frozen");
        _;
    }

    constructor(
        string memory _name,
        string memory _symbol,
        address _identityRegistry,
        address _compliance
    ) ERC20(_name, _symbol) {
        tokenIssuer = msg.sender;
        identityRegistry = IIdentityRegistry(_identityRegistry);
        compliance = ICompliance(_compliance);
        agents[msg.sender] = true;
    }

    /**
     * @dev Override transfer to include compliance checks
     */
    function transfer(address to, uint256 amount)
        public
        override
        whenNotPaused
        whenNotFrozen(msg.sender)
        whenNotFrozen(to)
        returns (bool)
    {
        require(_canTransfer(msg.sender, to, amount), "Transfer not compliant");
        return super.transfer(to, amount);
    }

    /**
     * @dev Override transferFrom to include compliance checks
     */
    function transferFrom(address from, address to, uint256 amount)
        public
        override
        whenNotPaused
        whenNotFrozen(from)
        whenNotFrozen(to)
        returns (bool)
    {
        require(_canTransfer(from, to, amount), "Transfer not compliant");
        return super.transferFrom(from, to, amount);
    }

    /**
     * @dev Check if transfer is compliant with all rules
     */
    function _canTransfer(
        address from,
        address to,
        uint256 amount
    ) internal view returns (bool) {
        // Check both parties are verified
        require(
            identityRegistry.isVerified(from),
            "Sender not verified"
        );
        require(
            identityRegistry.isVerified(to),
            "Recipient not verified"
        );

        // Check compliance rules
        require(
            compliance.canTransfer(from, to, amount),
            "Compliance check failed"
        );

        return true;
    }

    /**
     * @dev Forced transfer for regulatory compliance
     */
    function forcedTransfer(
        address from,
        address to,
        uint256 amount,
        bytes calldata reason
    ) external onlyAgent returns (bool) {
        require(identityRegistry.isVerified(to), "Recipient not verified");
        _transfer(from, to, amount);
        emit ForcedTransfer(from, to, amount, reason);
        return true;
    }

    /**
     * @dev Freeze account (regulatory action)
     */
    function freeze(address account) external onlyAgent {
        frozen[account] = true;
        emit TokenFrozen(account, msg.sender);
    }

    /**
     * @dev Unfreeze account
     */
    function unfreeze(address account) external onlyAgent {
        frozen[account] = false;
        emit TokenUnfrozen(account, msg.sender);
    }

    /**
     * @dev Mint new tokens (only for issuer)
     */
    function mint(address to, uint256 amount) external onlyIssuer {
        require(identityRegistry.isVerified(to), "Recipient not verified");
        _mint(to, amount);
    }

    /**
     * @dev Burn tokens (redemption)
     */
    function burn(uint256 amount) external {
        _burn(msg.sender, amount);
    }

    /**
     * @dev Recovery function for lost keys
     */
    function recoveryAddress(
        address lostAddress,
        address newAddress,
        bytes calldata investorIdentity
    ) external onlyAgent {
        require(identityRegistry.isVerified(newAddress), "New address not verified");
        uint256 balance = balanceOf(lostAddress);
        _transfer(lostAddress, newAddress, balance);
        emit AddressRecovered(lostAddress, newAddress, investorIdentity);
    }

    event ForcedTransfer(address indexed from, address indexed to, uint256 amount, bytes reason);
    event AddressRecovered(address indexed oldAddress, address indexed newAddress, bytes investorIdentity);
}
```

### Identity Registry

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "./interfaces/IIdentity.sol";
import "./interfaces/IClaimVerifier.sol";

/**
 * @title IdentityRegistry
 * @dev Registry for investor identities and verification status
 */
contract IdentityRegistry {
    // Mapping from wallet address to identity contract
    mapping(address => address) public identity;

    // Mapping from wallet address to country code
    mapping(address => uint16) public investorCountry;

    // Claim topics required for verification
    uint256[] public requiredClaimTopics;

    // Trusted claim issuers per topic
    mapping(uint256 => address[]) public trustedIssuers;

    // Identity registry agents
    mapping(address => bool) public agents;

    event IdentityRegistered(address indexed investor, address indexed identityContract);
    event IdentityRemoved(address indexed investor);
    event CountryUpdated(address indexed investor, uint16 indexed country);

    modifier onlyAgent() {
        require(agents[msg.sender], "Not authorized");
        _;
    }

    /**
     * @dev Register an investor's identity
     */
    function registerIdentity(
        address _investor,
        address _identity,
        uint16 _country
    ) external onlyAgent {
        require(_investor != address(0), "Invalid address");
        require(_identity != address(0), "Invalid identity");
        require(identity[_investor] == address(0), "Already registered");

        identity[_investor] = _identity;
        investorCountry[_investor] = _country;

        emit IdentityRegistered(_investor, _identity);
    }

    /**
     * @dev Check if an investor is verified
     */
    function isVerified(address _investor) public view returns (bool) {
        if (identity[_investor] == address(0)) return false;

        IIdentity investorIdentity = IIdentity(identity[_investor]);

        // Check all required claim topics
        for (uint256 i = 0; i < requiredClaimTopics.length; i++) {
            uint256 topic = requiredClaimTopics[i];

            bool hasValidClaim = false;
            address[] memory issuers = trustedIssuers[topic];

            for (uint256 j = 0; j < issuers.length; j++) {
                if (_hasValidClaim(investorIdentity, topic, issuers[j])) {
                    hasValidClaim = true;
                    break;
                }
            }

            if (!hasValidClaim) return false;
        }

        return true;
    }

    /**
     * @dev Check if identity has valid claim from issuer
     */
    function _hasValidClaim(
        IIdentity _identity,
        uint256 _topic,
        address _issuer
    ) internal view returns (bool) {
        bytes32[] memory claimIds = _identity.getClaimIdsByTopic(_topic);

        for (uint256 i = 0; i < claimIds.length; i++) {
            (
                uint256 topic,
                uint256 scheme,
                address issuer,
                bytes memory signature,
                bytes memory data,
                string memory uri
            ) = _identity.getClaim(claimIds[i]);

            if (issuer == _issuer) {
                // Verify claim is still valid
                IClaimVerifier verifier = IClaimVerifier(_issuer);
                if (verifier.isClaimValid(_identity, topic, signature, data)) {
                    return true;
                }
            }
        }

        return false;
    }

    /**
     * @dev Set required claim topics
     */
    function setClaimTopics(uint256[] calldata _topics) external onlyAgent {
        delete requiredClaimTopics;
        for (uint256 i = 0; i < _topics.length; i++) {
            requiredClaimTopics.push(_topics[i]);
        }
    }

    /**
     * @dev Add trusted claim issuer for topic
     */
    function addTrustedIssuer(
        uint256 _topic,
        address _issuer
    ) external onlyAgent {
        trustedIssuers[_topic].push(_issuer);
    }
}
```

---

## Real-World Asset Tokenization

### RWA Categories

**Tokenizable Asset Classes:**

| Asset Class | Examples | Market Size | Tokenization Rate |
|-------------|----------|-------------|-------------------|
| Treasury Securities | T-Bills, Bonds | $30T+ | $2B+ tokenized |
| Real Estate | Commercial, Residential | $300T+ | $500M+ tokenized |
| Private Credit | Loans, Receivables | $1.5T | $5B+ tokenized |
| Commodities | Gold, Carbon Credits | $5T+ | $200M+ tokenized |
| Funds | ETFs, Mutual Funds | $50T+ | $1B+ tokenized |
| Art & Collectibles | Fine Art, Rare Items | $1T+ | $100M+ tokenized |

### Treasury Tokenization

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/token/ERC20/extensions/ERC4626.sol";
import "@openzeppelin/contracts/access/AccessControl.sol";

/**
 * @title TokenizedTreasury
 * @dev ERC-4626 vault for tokenized treasury securities
 */
contract TokenizedTreasury is ERC4626, AccessControl {
    bytes32 public constant ORACLE_ROLE = keccak256("ORACLE_ROLE");
    bytes32 public constant MANAGER_ROLE = keccak256("MANAGER_ROLE");

    // Treasury details
    struct TreasuryBond {
        string cusip;           // CUSIP number
        uint256 faceValue;      // Par value
        uint256 couponRate;     // Annual coupon (basis points)
        uint256 maturityDate;   // Unix timestamp
        uint256 lastCouponDate; // Last coupon payment
    }

    TreasuryBond public bond;

    // NAV tracking
    uint256 public lastNAV;
    uint256 public lastNAVUpdate;

    // Whitelist for accredited investors
    mapping(address => bool) public whitelist;

    // Minimum investment
    uint256 public minimumInvestment;

    event NAVUpdated(uint256 newNAV, uint256 timestamp);
    event CouponDistributed(uint256 amount, uint256 timestamp);
    event InvestorWhitelisted(address indexed investor);

    modifier onlyWhitelisted() {
        require(whitelist[msg.sender], "Not whitelisted");
        _;
    }

    constructor(
        IERC20 _asset,
        string memory _name,
        string memory _symbol,
        TreasuryBond memory _bond
    ) ERC4626(_asset) ERC20(_name, _symbol) {
        bond = _bond;
        _grantRole(DEFAULT_ADMIN_ROLE, msg.sender);
        _grantRole(MANAGER_ROLE, msg.sender);
    }

    /**
     * @dev Deposit assets and mint shares
     */
    function deposit(uint256 assets, address receiver)
        public
        override
        onlyWhitelisted
        returns (uint256)
    {
        require(assets >= minimumInvestment, "Below minimum investment");
        return super.deposit(assets, receiver);
    }

    /**
     * @dev Update NAV from oracle
     */
    function updateNAV(uint256 newNAV) external onlyRole(ORACLE_ROLE) {
        lastNAV = newNAV;
        lastNAVUpdate = block.timestamp;
        emit NAVUpdated(newNAV, block.timestamp);
    }

    /**
     * @dev Calculate total assets including accrued interest
     */
    function totalAssets() public view override returns (uint256) {
        if (lastNAVUpdate == 0) {
            return super.totalAssets();
        }

        // Calculate accrued interest since last NAV update
        uint256 daysSinceUpdate = (block.timestamp - lastNAVUpdate) / 1 days;
        uint256 dailyRate = bond.couponRate / 365;
        uint256 accruedInterest = (lastNAV * dailyRate * daysSinceUpdate) / 10000;

        return lastNAV + accruedInterest;
    }

    /**
     * @dev Distribute coupon payments
     */
    function distributeCoupon() external onlyRole(MANAGER_ROLE) {
        require(
            block.timestamp >= bond.lastCouponDate + 182 days,
            "Coupon not yet due"
        );

        uint256 couponAmount = (totalAssets() * bond.couponRate) / 20000; // Semi-annual
        bond.lastCouponDate = block.timestamp;

        // Distribute proportionally to all shareholders
        // Implementation depends on distribution mechanism

        emit CouponDistributed(couponAmount, block.timestamp);
    }

    /**
     * @dev Whitelist investor
     */
    function whitelistInvestor(address investor) external onlyRole(MANAGER_ROLE) {
        whitelist[investor] = true;
        emit InvestorWhitelisted(investor);
    }

    /**
     * @dev Handle maturity
     */
    function handleMaturity() external onlyRole(MANAGER_ROLE) {
        require(block.timestamp >= bond.maturityDate, "Not yet matured");
        // Redeem underlying assets and distribute to shareholders
    }
}
```

### Real Estate Tokenization

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "./WIASecurityToken.sol";

/**
 * @title RealEstateToken
 * @dev Tokenized real estate with rental income distribution
 */
contract RealEstateToken is WIASecurityToken {
    struct Property {
        string propertyId;          // Unique property identifier
        string legalDescription;    // Legal property description
        uint256 totalValue;         // Appraised value
        uint256 tokenizedShares;    // Total tokens issued
        uint256 annualRentalIncome; // Expected annual income
        address propertyManager;    // Property management company
    }

    Property public property;

    // Dividend tracking
    uint256 public totalDividendsDistributed;
    mapping(address => uint256) public lastClaimedDividend;
    mapping(address => uint256) public unclaimedDividends;

    // Dividend pool
    uint256 public dividendPool;
    uint256 public dividendPerShare;

    // Operating expenses reserve
    uint256 public operatingReserve;
    uint256 public reserveTarget; // Percentage in basis points

    event DividendDeposited(uint256 amount, uint256 timestamp);
    event DividendClaimed(address indexed investor, uint256 amount);
    event PropertyValueUpdated(uint256 newValue, uint256 timestamp);

    constructor(
        string memory _name,
        string memory _symbol,
        address _identityRegistry,
        address _compliance,
        Property memory _property
    ) WIASecurityToken(_name, _symbol, _identityRegistry, _compliance) {
        property = _property;
    }

    /**
     * @dev Deposit rental income for distribution
     */
    function depositDividend() external payable {
        require(msg.sender == property.propertyManager, "Not property manager");

        // Set aside operating reserve
        uint256 reserveContribution = (msg.value * reserveTarget) / 10000;
        operatingReserve += reserveContribution;

        uint256 distributableAmount = msg.value - reserveContribution;
        dividendPool += distributableAmount;

        if (totalSupply() > 0) {
            dividendPerShare += (distributableAmount * 1e18) / totalSupply();
        }

        emit DividendDeposited(distributableAmount, block.timestamp);
    }

    /**
     * @dev Claim accumulated dividends
     */
    function claimDividend() external {
        uint256 owed = _calculateOwedDividends(msg.sender);
        require(owed > 0, "No dividends to claim");

        lastClaimedDividend[msg.sender] = dividendPerShare;
        unclaimedDividends[msg.sender] = 0;

        payable(msg.sender).transfer(owed);
        totalDividendsDistributed += owed;

        emit DividendClaimed(msg.sender, owed);
    }

    /**
     * @dev Calculate dividends owed to investor
     */
    function _calculateOwedDividends(address investor)
        internal
        view
        returns (uint256)
    {
        uint256 balance = balanceOf(investor);
        uint256 dividendsSinceLastClaim =
            dividendPerShare - lastClaimedDividend[investor];

        return ((balance * dividendsSinceLastClaim) / 1e18) +
            unclaimedDividends[investor];
    }

    /**
     * @dev Override transfer to handle dividend accounting
     */
    function _beforeTokenTransfer(
        address from,
        address to,
        uint256 amount
    ) internal virtual override {
        // Store unclaimed dividends before transfer
        if (from != address(0)) {
            unclaimedDividends[from] = _calculateOwedDividends(from);
            lastClaimedDividend[from] = dividendPerShare;
        }

        if (to != address(0)) {
            unclaimedDividends[to] = _calculateOwedDividends(to);
            lastClaimedDividend[to] = dividendPerShare;
        }

        super._beforeTokenTransfer(from, to, amount);
    }

    /**
     * @dev Update property appraisal value
     */
    function updatePropertyValue(uint256 newValue) external onlyAgent {
        property.totalValue = newValue;
        emit PropertyValueUpdated(newValue, block.timestamp);
    }

    /**
     * @dev Get current dividend yield
     */
    function getCurrentYield() external view returns (uint256) {
        if (property.totalValue == 0) return 0;
        return (property.annualRentalIncome * 10000) / property.totalValue;
    }
}
```

---

## Compliance Modules

### Transfer Restriction Rules

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "./interfaces/ICompliance.sol";
import "./interfaces/IIdentityRegistry.sol";

/**
 * @title ComplianceModule
 * @dev Modular compliance rules for security tokens
 */
contract ComplianceModule is ICompliance {
    IIdentityRegistry public identityRegistry;

    // Maximum number of investors
    uint256 public maxInvestors;
    uint256 public currentInvestors;

    // Investor country restrictions
    mapping(uint16 => bool) public countryBlocked;

    // Maximum ownership percentage (basis points)
    uint256 public maxOwnershipPercentage;

    // Lock-up periods
    mapping(address => uint256) public lockupEndTime;

    // Daily transfer limits
    mapping(address => uint256) public dailyTransferred;
    mapping(address => uint256) public lastTransferDay;
    uint256 public dailyTransferLimit;

    // Accreditation requirements
    bool public requireAccreditation;
    uint256 public constant ACCREDITATION_CLAIM_TOPIC = 10;

    event CountryBlockStatusChanged(uint16 country, bool blocked);
    event MaxInvestorsUpdated(uint256 newMax);
    event LockupSet(address indexed investor, uint256 endTime);

    constructor(address _identityRegistry) {
        identityRegistry = IIdentityRegistry(_identityRegistry);
    }

    /**
     * @dev Check if transfer is compliant
     */
    function canTransfer(
        address from,
        address to,
        uint256 amount
    ) external view override returns (bool) {
        // Check investor count limit
        if (balanceOf(to) == 0 && currentInvestors >= maxInvestors) {
            return false;
        }

        // Check country restrictions
        uint16 toCountry = identityRegistry.investorCountry(to);
        if (countryBlocked[toCountry]) {
            return false;
        }

        // Check lock-up period
        if (lockupEndTime[from] > block.timestamp) {
            return false;
        }

        // Check maximum ownership
        if (maxOwnershipPercentage > 0) {
            uint256 newBalance = balanceOf(to) + amount;
            uint256 ownershipBps = (newBalance * 10000) / totalSupply();
            if (ownershipBps > maxOwnershipPercentage) {
                return false;
            }
        }

        // Check daily transfer limit
        if (dailyTransferLimit > 0) {
            uint256 currentDay = block.timestamp / 1 days;
            uint256 dailyAmount = lastTransferDay[from] == currentDay
                ? dailyTransferred[from] + amount
                : amount;
            if (dailyAmount > dailyTransferLimit) {
                return false;
            }
        }

        return true;
    }

    /**
     * @dev Block or unblock country
     */
    function setCountryBlocked(
        uint16 country,
        bool blocked
    ) external onlyOwner {
        countryBlocked[country] = blocked;
        emit CountryBlockStatusChanged(country, blocked);
    }

    /**
     * @dev Set lock-up period for investor
     */
    function setLockup(
        address investor,
        uint256 endTime
    ) external onlyOwner {
        lockupEndTime[investor] = endTime;
        emit LockupSet(investor, endTime);
    }

    /**
     * @dev Set maximum investors
     */
    function setMaxInvestors(uint256 _max) external onlyOwner {
        maxInvestors = _max;
        emit MaxInvestorsUpdated(_max);
    }

    /**
     * @dev Called after transfer to update state
     */
    function transferred(
        address from,
        address to,
        uint256 amount
    ) external override {
        // Update investor count
        if (balanceOf(from) == 0 && from != address(0)) {
            currentInvestors--;
        }
        if (balanceOf(to) == amount && to != address(0)) {
            currentInvestors++;
        }

        // Update daily transfer tracking
        uint256 currentDay = block.timestamp / 1 days;
        if (lastTransferDay[from] != currentDay) {
            dailyTransferred[from] = 0;
            lastTransferDay[from] = currentDay;
        }
        dailyTransferred[from] += amount;
    }

    // Placeholder functions - actual implementations depend on token contract
    function balanceOf(address) internal view returns (uint256) { return 0; }
    function totalSupply() internal view returns (uint256) { return 0; }
    modifier onlyOwner() { _; }
}
```

---

## Fractional Ownership

### Fractionalization Protocol

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/token/ERC721/IERC721.sol";
import "@openzeppelin/contracts/token/ERC20/ERC20.sol";
import "@openzeppelin/contracts/security/ReentrancyGuard.sol";

/**
 * @title FractionalizationVault
 * @dev Fractionalize NFTs into fungible tokens
 */
contract FractionalizationVault is ERC20, ReentrancyGuard {
    IERC721 public nftContract;
    uint256 public tokenId;

    // Vault state
    enum State { Inactive, Active, Redeemed }
    State public state;

    // Auction parameters for buyout
    uint256 public reservePrice;
    uint256 public auctionEndTime;
    address public highestBidder;
    uint256 public highestBid;

    // Curator (original fractionalizer)
    address public curator;
    uint256 public curatorFee; // Basis points

    event Fractionalized(address indexed curator, uint256 fractions);
    event AuctionStarted(uint256 reservePrice, uint256 endTime);
    event BidPlaced(address indexed bidder, uint256 amount);
    event AuctionWon(address indexed winner, uint256 amount);
    event Redeemed(address indexed redeemer);

    constructor(
        address _nftContract,
        uint256 _tokenId,
        string memory _name,
        string memory _symbol,
        uint256 _totalSupply,
        uint256 _reservePrice,
        uint256 _curatorFee
    ) ERC20(_name, _symbol) {
        nftContract = IERC721(_nftContract);
        tokenId = _tokenId;
        reservePrice = _reservePrice;
        curatorFee = _curatorFee;
        curator = msg.sender;

        // Mint fractions to curator
        _mint(msg.sender, _totalSupply);

        emit Fractionalized(msg.sender, _totalSupply);
    }

    /**
     * @dev Activate vault by depositing NFT
     */
    function activate() external {
        require(state == State.Inactive, "Already active");
        require(msg.sender == curator, "Only curator");

        nftContract.transferFrom(msg.sender, address(this), tokenId);
        state = State.Active;
    }

    /**
     * @dev Start buyout auction
     */
    function startAuction() external payable nonReentrant {
        require(state == State.Active, "Vault not active");
        require(msg.value >= reservePrice, "Below reserve price");
        require(auctionEndTime == 0, "Auction already started");

        auctionEndTime = block.timestamp + 7 days;
        highestBidder = msg.sender;
        highestBid = msg.value;

        emit AuctionStarted(reservePrice, auctionEndTime);
        emit BidPlaced(msg.sender, msg.value);
    }

    /**
     * @dev Place higher bid
     */
    function bid() external payable nonReentrant {
        require(auctionEndTime > 0, "No auction");
        require(block.timestamp < auctionEndTime, "Auction ended");
        require(msg.value > highestBid * 105 / 100, "Bid not 5% higher");

        // Refund previous bidder
        if (highestBidder != address(0)) {
            payable(highestBidder).transfer(highestBid);
        }

        highestBidder = msg.sender;
        highestBid = msg.value;

        // Extend auction if bid in last 15 minutes
        if (auctionEndTime - block.timestamp < 15 minutes) {
            auctionEndTime += 15 minutes;
        }

        emit BidPlaced(msg.sender, msg.value);
    }

    /**
     * @dev End auction and transfer NFT to winner
     */
    function endAuction() external nonReentrant {
        require(auctionEndTime > 0, "No auction");
        require(block.timestamp >= auctionEndTime, "Auction not ended");

        state = State.Redeemed;

        // Transfer NFT to winner
        nftContract.transferFrom(address(this), highestBidder, tokenId);

        // Calculate curator fee
        uint256 fee = (highestBid * curatorFee) / 10000;
        payable(curator).transfer(fee);

        emit AuctionWon(highestBidder, highestBid);
    }

    /**
     * @dev Redeem ETH for fraction tokens after auction
     */
    function redeem() external nonReentrant {
        require(state == State.Redeemed, "Not redeemed");

        uint256 balance = balanceOf(msg.sender);
        require(balance > 0, "No tokens to redeem");

        // Calculate share of proceeds
        uint256 fee = (highestBid * curatorFee) / 10000;
        uint256 proceeds = highestBid - fee;
        uint256 share = (proceeds * balance) / totalSupply();

        _burn(msg.sender, balance);
        payable(msg.sender).transfer(share);

        emit Redeemed(msg.sender);
    }

    /**
     * @dev Allow full token holder to redeem NFT directly
     */
    function redeemNFT() external {
        require(state == State.Active, "Vault not active");
        require(balanceOf(msg.sender) == totalSupply(), "Must own all tokens");

        state = State.Redeemed;
        _burn(msg.sender, totalSupply());
        nftContract.transferFrom(address(this), msg.sender, tokenId);
    }
}
```

---

## Cross-Chain Tokenization

### Multi-Chain Token Bridge

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/token/ERC20/ERC20.sol";
import "@openzeppelin/contracts/security/Pausable.sol";

/**
 * @title BridgeableToken
 * @dev Token with cross-chain bridging capabilities
 */
contract BridgeableToken is ERC20, Pausable {
    // Bridge contracts authorized to mint/burn
    mapping(address => bool) public bridges;

    // Chain-specific supply tracking
    mapping(uint256 => uint256) public chainSupply;
    uint256 public constant HOME_CHAIN_ID = 1; // Ethereum mainnet

    // Cross-chain transfer tracking
    mapping(bytes32 => bool) public processedTransfers;

    event BridgeAdded(address indexed bridge);
    event BridgeRemoved(address indexed bridge);
    event CrossChainTransfer(
        address indexed from,
        address indexed to,
        uint256 amount,
        uint256 indexed destinationChain,
        bytes32 transferId
    );
    event CrossChainReceive(
        address indexed to,
        uint256 amount,
        uint256 indexed sourceChain,
        bytes32 transferId
    );

    modifier onlyBridge() {
        require(bridges[msg.sender], "Not authorized bridge");
        _;
    }

    constructor(
        string memory name,
        string memory symbol,
        uint256 initialSupply
    ) ERC20(name, symbol) {
        _mint(msg.sender, initialSupply);
        chainSupply[HOME_CHAIN_ID] = initialSupply;
    }

    /**
     * @dev Initiate cross-chain transfer
     */
    function bridgeTransfer(
        address to,
        uint256 amount,
        uint256 destinationChain
    ) external whenNotPaused {
        require(to != address(0), "Invalid recipient");
        require(amount > 0, "Zero amount");

        // Generate unique transfer ID
        bytes32 transferId = keccak256(abi.encodePacked(
            msg.sender,
            to,
            amount,
            destinationChain,
            block.timestamp,
            block.number
        ));

        // Burn tokens on source chain
        _burn(msg.sender, amount);
        chainSupply[block.chainid] -= amount;

        emit CrossChainTransfer(msg.sender, to, amount, destinationChain, transferId);
    }

    /**
     * @dev Receive cross-chain transfer (called by bridge)
     */
    function bridgeReceive(
        address to,
        uint256 amount,
        uint256 sourceChain,
        bytes32 transferId
    ) external onlyBridge whenNotPaused {
        require(!processedTransfers[transferId], "Already processed");

        processedTransfers[transferId] = true;

        // Mint tokens on destination chain
        _mint(to, amount);
        chainSupply[block.chainid] += amount;

        emit CrossChainReceive(to, amount, sourceChain, transferId);
    }

    /**
     * @dev Add authorized bridge
     */
    function addBridge(address bridge) external onlyOwner {
        bridges[bridge] = true;
        emit BridgeAdded(bridge);
    }

    /**
     * @dev Remove bridge authorization
     */
    function removeBridge(address bridge) external onlyOwner {
        bridges[bridge] = false;
        emit BridgeRemoved(bridge);
    }

    /**
     * @dev Get total supply across all chains
     */
    function globalTotalSupply() external view returns (uint256) {
        // In practice, would query oracles for other chain supplies
        return chainSupply[block.chainid];
    }

    modifier onlyOwner() {
        // Owner check implementation
        _;
    }
}
```

---

## Key Takeaways

1. **ERC-3643** is the standard for compliant security tokens with built-in transfer restrictions and identity verification
2. **Identity registries** enable KYC/AML compliance by linking wallet addresses to verified identities
3. **RWA tokenization** brings real estate, treasuries, and other traditional assets on-chain
4. **Compliance modules** enforce investor limits, country restrictions, lock-ups, and ownership caps
5. **Fractionalization** enables shared ownership of high-value assets with built-in buyout mechanisms
6. **Cross-chain bridges** allow tokenized assets to move between blockchain networks

## Review Questions

1. What are the key differences between ERC-20 and ERC-3643 token standards?
2. How does the identity registry enable compliant token transfers?
3. What compliance rules can be enforced through smart contracts?
4. How does fractional ownership work for tokenized real estate?
5. What mechanisms enable cross-chain token bridging?
6. How are dividends distributed in tokenized real estate?

---

**Next Chapter Preview:** Chapter 4 explores DeFi protocols in depth, covering automated market makers, lending protocols, and derivatives platforms.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity · Democratize Finance

