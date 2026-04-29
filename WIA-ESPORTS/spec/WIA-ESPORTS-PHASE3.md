# WIA-ESPORTS Specification - PHASE 3
# Security & Performance

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2026-01-12
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## Table of Contents

1. [Security Framework](#security-framework)
2. [Common Vulnerabilities](#common-vulnerabilities)
3. [Security Best Practices](#security-best-practices)
4. [Performance Optimization](#performance-optimization)
5. [Scalability Solutions](#scalability-solutions)
6. [Monitoring & Incident Response](#monitoring--incident-response)

---

## Security Framework

### Security Layers

```
┌─────────────────────────────────────────┐
│    Application Layer Security           │
│  - Input Validation                     │
│  - Access Control                       │
│  - Rate Limiting                        │
├─────────────────────────────────────────┤
│    Smart Contract Security              │
│  - Code Audits                          │
│  - Formal Verification                  │
│  - Bug Bounties                         │
├─────────────────────────────────────────┤
│    Infrastructure Security              │
│  - Node Security                        │
│  - Key Management                       │
│  - Network Security                     │
├─────────────────────────────────────────┤
│    Blockchain Consensus Security        │
│  - 51% Attack Resistance                │
│  - Finality Guarantees                  │
└─────────────────────────────────────────┘
```

### Security Principles

1. **Defense in Depth**: Multiple layers of security controls
2. **Least Privilege**: Minimal access rights for each component
3. **Fail Securely**: System defaults to secure state on failure
4. **Open Design**: Security through transparency, not obscurity
5. **Complete Mediation**: Every access checked
6. **Separation of Duties**: Critical operations require multiple parties

---

## Common Vulnerabilities

### 1. Reentrancy Attacks

**Vulnerability:**
Attacker recursively calls a function before the first call completes, potentially draining funds.

**Example Vulnerable Code:**
```solidity
// VULNERABLE - DO NOT USE
function withdraw(uint256 amount) public {
    require(balances[msg.sender] >= amount);

    // External call before state update
    (bool success, ) = msg.sender.call{value: amount}("");
    require(success);

    balances[msg.sender] -= amount; // Too late!
}
```

**Secure Implementation:**
```solidity
// SECURE - Checks-Effects-Interactions Pattern
function withdraw(uint256 amount) public nonReentrant {
    require(balances[msg.sender] >= amount, "Insufficient balance");

    // Update state BEFORE external call
    balances[msg.sender] -= amount;

    // External call last
    (bool success, ) = msg.sender.call{value: amount}("");
    require(success, "Transfer failed");
}
```

**OpenZeppelin ReentrancyGuard:**
```solidity
import "@openzeppelin/contracts/security/ReentrancyGuard.sol";

contract SecureVault is ReentrancyGuard {
    mapping(address => uint256) public balances;

    function withdraw(uint256 amount) external nonReentrant {
        // Protected from reentrancy
        require(balances[msg.sender] >= amount);
        balances[msg.sender] -= amount;
        payable(msg.sender).transfer(amount);
    }
}
```

### 2. Flash Loan Attacks

**Vulnerability:**
Attackers borrow large amounts without collateral, manipulate prices, profit, and repay within one transaction.

**Attack Vector:**
```solidity
// Attacker borrows 1M tokens
// Dumps on DEX to crash price
// Liquidates others at manipulated price
// Buys back cheap, repays loan, keeps profit
```

**Mitigation Strategies:**

**A. Time-Weighted Average Price (TWAP):**
```solidity
function getTWAP(address pool, uint32 period) public view returns (uint256) {
    uint32[] memory secondsAgos = new uint32[](2);
    secondsAgos[0] = period;
    secondsAgos[1] = 0;

    (int56[] memory tickCumulatives, ) = IUniswapV3Pool(pool).observe(secondsAgos);

    int56 tickCumulativeDelta = tickCumulatives[1] - tickCumulatives[0];
    int24 avgTick = int24(tickCumulativeDelta / int56(uint56(period)));

    return getQuoteAtTick(avgTick);
}
```

**B. Multiple Oracle Sources:**
```solidity
function getSecurePrice(address asset) public view returns (uint256) {
    uint256 chainlinkPrice = getChainlinkPrice(asset);
    uint256 uniswapTWAP = getUniswapTWAP(asset, 1800); // 30 min TWAP
    uint256 bandPrice = getBandPrice(asset);

    // Median of three sources
    uint256[] memory prices = new uint256[](3);
    prices[0] = chainlinkPrice;
    prices[1] = uniswapTWAP;
    prices[2] = bandPrice;

    return getMedian(prices);
}
```

**C. Lending Protocol Protection:**
```solidity
contract SecureLending {
    // Minimum borrow time lock
    mapping(address => uint256) public lastBorrowBlock;

    function borrow(uint256 amount) external {
        require(
            block.number > lastBorrowBlock[msg.sender] + 1,
            "Must wait 1 block"
        );

        lastBorrowBlock[msg.sender] = block.number;

        // Borrow logic...
    }
}
```

### 3. Front-Running

**Vulnerability:**
Malicious actors see pending transactions and submit their own with higher gas to execute first.

**Attack Example:**
```
1. User submits swap: 1000 USDC → ETH (gas: 50 gwei)
2. Bot sees transaction in mempool
3. Bot submits buy order (gas: 100 gwei) - executes first, raises price
4. User's swap executes at worse price
5. Bot sells at profit
```

**Mitigation:**

**A. Slippage Protection:**
```solidity
function swap(
    address tokenIn,
    address tokenOut,
    uint256 amountIn,
    uint256 minAmountOut, // Minimum acceptable output
    uint256 deadline
) external {
    require(block.timestamp <= deadline, "Expired");

    uint256 amountOut = _executeSwap(tokenIn, tokenOut, amountIn);

    require(amountOut >= minAmountOut, "Slippage too high");
}
```

**B. Commit-Reveal Scheme:**
```solidity
contract CommitReveal {
    mapping(address => bytes32) public commits;
    mapping(address => uint256) public commitBlocks;

    function commit(bytes32 hash) external {
        commits[msg.sender] = hash;
        commitBlocks[msg.sender] = block.number;
    }

    function reveal(uint256 value, bytes32 salt) external {
        require(block.number > commitBlocks[msg.sender] + 1, "Too early");
        require(
            keccak256(abi.encodePacked(value, salt)) == commits[msg.sender],
            "Invalid reveal"
        );

        // Execute with revealed value
    }
}
```

**C. Flashbots / Private Mempools:**
```javascript
// Send transaction through Flashbots to avoid public mempool
const flashbotsProvider = await FlashbotsBundleProvider.create(
  provider,
  authSigner
);

const signedBundle = await flashbotsProvider.signBundle([
  { signedTransaction: signedTx }
]);

const simulation = await flashbotsProvider.simulate(signedBundle, targetBlock);
```

### 4. Integer Overflow/Underflow

**Vulnerability:**
Arithmetic operations exceed type bounds, wrapping around.

**Example (Solidity < 0.8):**
```solidity
// VULNERABLE with Solidity < 0.8
uint8 x = 255;
x = x + 1; // Wraps to 0
```

**Mitigation:**

**A. Use Solidity 0.8+:**
```solidity
// Solidity 0.8+ has built-in overflow protection
pragma solidity ^0.8.0;

contract Safe {
    uint8 public counter = 255;

    function increment() public {
        counter++; // Automatically reverts on overflow
    }
}
```

**B. SafeMath Library (for Solidity < 0.8):**
```solidity
import "@openzeppelin/contracts/utils/math/SafeMath.sol";

contract OldVersion {
    using SafeMath for uint256;

    function add(uint256 a, uint256 b) public pure returns (uint256) {
        return a.add(b); // Reverts on overflow
    }
}
```

### 5. Access Control Issues

**Vulnerability:**
Unauthorized users can call privileged functions.

**Example Vulnerable:**
```solidity
// VULNERABLE
contract Vault {
    function withdraw(uint256 amount) public {
        // Anyone can call this!
        payable(msg.sender).transfer(amount);
    }
}
```

**Secure Implementation:**
```solidity
import "@openzeppelin/contracts/access/AccessControl.sol";

contract SecureVault is AccessControl {
    bytes32 public constant ADMIN_ROLE = keccak256("ADMIN_ROLE");
    bytes32 public constant OPERATOR_ROLE = keccak256("OPERATOR_ROLE");

    constructor() {
        _grantRole(DEFAULT_ADMIN_ROLE, msg.sender);
        _grantRole(ADMIN_ROLE, msg.sender);
    }

    function withdraw(uint256 amount) external onlyRole(ADMIN_ROLE) {
        payable(msg.sender).transfer(amount);
    }

    function pause() external onlyRole(OPERATOR_ROLE) {
        _pause();
    }
}
```

### 6. Oracle Manipulation

**Vulnerability:**
Attackers manipulate price oracles to exploit protocols.

**Secure Oracle Implementation:**
```solidity
contract SecurePriceOracle {
    IChainlinkAggregator public chainlink;
    IUniswapV3Pool public uniswapPool;
    uint32 public constant TWAP_PERIOD = 1800; // 30 minutes
    uint256 public constant MAX_DEVIATION = 5; // 5% max deviation

    function getPrice() external view returns (uint256) {
        uint256 chainlinkPrice = getChainlinkPrice();
        uint256 twapPrice = getUniswapTWAP();

        // Verify prices are within acceptable range
        uint256 deviation = calculateDeviation(chainlinkPrice, twapPrice);
        require(deviation <= MAX_DEVIATION, "Price deviation too high");

        // Return average for extra security
        return (chainlinkPrice + twapPrice) / 2;
    }

    function calculateDeviation(uint256 price1, uint256 price2)
        private pure returns (uint256)
    {
        uint256 diff = price1 > price2 ? price1 - price2 : price2 - price1;
        return (diff * 100) / price1;
    }
}
```

---

## Security Best Practices

### Smart Contract Development

#### 1. Use Latest Solidity Version

```solidity
pragma solidity ^0.8.20; // Use latest stable version
```

#### 2. Implement Circuit Breakers

```solidity
import "@openzeppelin/contracts/security/Pausable.sol";

contract ProtectedContract is Pausable {
    function criticalOperation() external whenNotPaused {
        // Safe to execute only when not paused
    }

    function emergencyPause() external onlyOwner {
        _pause();
    }

    function unpause() external onlyOwner {
        _unpause();
    }
}
```

#### 3. Use Pull Over Push for Payments

```solidity
// BETTER: Pull pattern
contract SecurePayments {
    mapping(address => uint256) public pendingWithdrawals;

    function recordPayment(address recipient, uint256 amount) internal {
        pendingWithdrawals[recipient] += amount;
    }

    function withdraw() external {
        uint256 amount = pendingWithdrawals[msg.sender];
        require(amount > 0, "No funds");

        pendingWithdrawals[msg.sender] = 0;
        payable(msg.sender).transfer(amount);
    }
}

// AVOID: Push pattern (vulnerable to DOS)
contract VulnerablePayments {
    function distribute(address[] memory recipients, uint256[] memory amounts)
        external
    {
        for (uint i = 0; i < recipients.length; i++) {
            // If one transfer fails, all fail
            payable(recipients[i]).transfer(amounts[i]);
        }
    }
}
```

#### 4. Implement Rate Limiting

```solidity
contract RateLimited {
    mapping(address => uint256) public lastActionTime;
    uint256 public constant COOLDOWN = 1 hours;

    modifier rateLimit() {
        require(
            block.timestamp >= lastActionTime[msg.sender] + COOLDOWN,
            "Rate limit exceeded"
        );
        lastActionTime[msg.sender] = block.timestamp;
        _;
    }

    function limitedAction() external rateLimit {
        // Can only be called once per hour per address
    }
}
```

#### 5. Careful with Delegatecall

```solidity
// DANGEROUS - delegatecall executes in caller's context
contract Dangerous {
    function doSomething(address target, bytes memory data) external {
        (bool success, ) = target.delegatecall(data);
        require(success);
    }
}

// SAFER - Whitelist allowed targets
contract Safer {
    mapping(address => bool) public allowedTargets;

    function doSomething(address target, bytes memory data) external {
        require(allowedTargets[target], "Target not allowed");
        (bool success, ) = target.delegatecall(data);
        require(success);
    }
}
```

### Testing Strategy

#### Unit Tests

```javascript
describe("LiquidityPool", () => {
  it("should prevent reentrancy attacks", async () => {
    const attacker = await deployAttacker();

    await expect(
      attacker.attack(pool.address)
    ).to.be.revertedWith("ReentrancyGuard: reentrant call");
  });

  it("should enforce slippage protection", async () => {
    const minOut = ethers.utils.parseEther("1.0");

    await expect(
      pool.swap(token0, token1, amountIn, minOut)
    ).to.be.revertedWith("Slippage too high");
  });
});
```

#### Fuzz Testing

```javascript
const { FuzzTest } = require('@fuzzing/framework');

FuzzTest('swap should never create negative balances', async () => {
  const amountIn = randomBigNumber();
  const minOut = randomBigNumber();

  await pool.swap(token0, token1, amountIn, minOut);

  const balance0 = await token0.balanceOf(pool.address);
  const balance1 = await token1.balanceOf(pool.address);

  expect(balance0).to.be.gte(0);
  expect(balance1).to.be.gte(0);
});
```

#### Integration Tests

```javascript
describe("Full Protocol Integration", () => {
  it("should handle complex multi-protocol interactions", async () => {
    // 1. Supply USDC to Aave
    await aave.supply(usdc.address, ethers.utils.parseUnits("10000", 6));

    // 2. Borrow WETH against USDC
    await aave.borrow(weth.address, ethers.utils.parseEther("2"));

    // 3. Provide WETH-USDT liquidity on Uniswap
    await uniswap.addLiquidity(weth.address, usdt.address, ...);

    // 4. Verify health factor
    const health = await aave.getUserHealthFactor(user.address);
    expect(health).to.be.gte(ethers.utils.parseEther("1.5"));
  });
});
```

---

## Performance Optimization

### Gas Optimization Techniques

#### 1. Use Calldata Instead of Memory

```solidity
// MORE GAS: Memory copy
function process(uint256[] memory data) external {
    // ...
}

// LESS GAS: Direct calldata read
function process(uint256[] calldata data) external {
    // ...
}
```

#### 2. Pack Storage Variables

```solidity
// EXPENSIVE: Each variable in separate slot
contract Unoptimized {
    uint8 a;   // slot 0
    uint256 b; // slot 1
    uint8 c;   // slot 2
}

// OPTIMIZED: Pack into fewer slots
contract Optimized {
    uint8 a;   // slot 0
    uint8 c;   // slot 0 (packed)
    uint256 b; // slot 1
}
```

#### 3. Use Immutable and Constant

```solidity
contract Optimized {
    // Computed at compile time, no storage slot
    uint256 public constant FEE_DENOMINATOR = 10000;

    // Set once in constructor, cheaper reads
    address public immutable factory;

    // Expensive: regular storage variable
    address public owner;

    constructor(address _factory) {
        factory = _factory;
        owner = msg.sender;
    }
}
```

#### 4. Batch Operations

```solidity
// EXPENSIVE: Multiple transactions
function claimOne(uint256 tokenId) external {
    _claim(tokenId);
}

// OPTIMIZED: Single transaction
function claimMany(uint256[] calldata tokenIds) external {
    for (uint i = 0; i < tokenIds.length; i++) {
        _claim(tokenIds[i]);
    }
}
```

#### 5. Short-Circuit Evaluation

```solidity
// Expensive check first (bad)
function validate() external view returns (bool) {
    return expensiveCheck() && cheapCheck();
}

// Cheap check first (good)
function validate() external view returns (bool) {
    return cheapCheck() && expensiveCheck();
}
```

### Layer 2 Scaling Solutions

#### Arbitrum Integration

```javascript
const L1_RPC = "https://eth-mainnet.g.alchemy.com/v2/...";
const L2_RPC = "https://arb-mainnet.g.alchemy.com/v2/...";

const l1Provider = new ethers.providers.JsonRpcProvider(L1_RPC);
const l2Provider = new ethers.providers.JsonRpcProvider(L2_RPC);

// Bridge tokens L1 → L2
const l1Bridge = new ethers.Contract(L1_GATEWAY, ABI, l1Wallet);
await l1Bridge.outboundTransfer(
  tokenAddress,
  recipientAddress,
  amount,
  maxGas,
  gasPriceBid,
  data
);
```

#### Optimism Integration

```javascript
const crossChainMessenger = new CrossChainMessenger({
  l1ChainId: 1,
  l2ChainId: 10,
  l1SignerOrProvider: l1Wallet,
  l2SignerOrProvider: l2Wallet,
});

// Deposit tokens
const depositTx = await crossChainMessenger.depositERC20(
  L1_TOKEN,
  L2_TOKEN,
  amount
);
await depositTx.wait();
```

---

## Scalability Solutions

### Transaction Throughput Comparison

| Network | TPS | Block Time | Finality | Gas Cost (2026) |
|---------|-----|------------|----------|-----------------|
| Ethereum L1 | 15-30 | 12s | 12-15 min | $5-20 |
| Arbitrum | 40,000 | 0.25s | 12-15 min | $0.10-0.50 |
| Optimism | 2,000+ | 2s | 7 days | $0.10-0.50 |
| Base | 2,000+ | 2s | 7 days | $0.05-0.30 |
| zkSync Era | 2,000+ | ~10s | 1-24 hours | $0.10-0.40 |
| Polygon zkEVM | 2,000+ | ~10s | 30 min | $0.05-0.20 |

### Database Optimization

```sql
-- Index frequently queried columns
CREATE INDEX idx_pools_tvl ON pools(tvl DESC);
CREATE INDEX idx_swaps_timestamp ON swaps(timestamp DESC);
CREATE INDEX idx_positions_owner ON positions(owner, protocol);

-- Materialized view for aggregated data
CREATE MATERIALIZED VIEW protocol_stats AS
SELECT
  protocol,
  SUM(tvl) as total_tvl,
  SUM(volume_24h) as total_volume,
  COUNT(*) as pool_count
FROM pools
GROUP BY protocol;

-- Refresh periodically
REFRESH MATERIALIZED VIEW CONCURRENTLY protocol_stats;
```

---

## Monitoring & Incident Response

### Monitoring Metrics

```yaml
# Prometheus metrics
defi_protocol_tvl{protocol="aave-v4"} 15432100000.50
defi_protocol_utilization{protocol="aave-v4"} 53.37
defi_pool_volume_24h{pool_id="0x88e6..."} 45678901.23
defi_gas_price_gwei 25
defi_transaction_count_total 1234567
defi_api_request_duration_seconds{endpoint="/v1/pools"} 0.123
```

### Alerting Rules

```yaml
# Alert if TVL drops >10% in 1 hour
- alert: TVL_Drop
  expr: (tvl - tvl offset 1h) / tvl offset 1h < -0.10
  for: 5m
  annotations:
    summary: "TVL dropped >10% in 1 hour"

# Alert if gas price exceeds threshold
- alert: High_Gas_Price
  expr: gas_price_gwei > 100
  for: 10m
  annotations:
    summary: "Gas price >100 gwei"
```

### Incident Response Plan

1. **Detection**: Automated monitoring alerts team
2. **Assessment**: Determine severity (Critical/High/Medium/Low)
3. **Containment**: Pause affected contracts if needed
4. **Investigation**: Analyze root cause
5. **Resolution**: Deploy fixes, resume operations
6. **Post-Mortem**: Document lessons learned

---

© 2026 WIA (World Certification Industry Association)
弘益人間 (Hongik Ingan) - Benefit All Humanity

Licensed under MIT License
