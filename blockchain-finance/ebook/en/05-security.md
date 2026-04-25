# Chapter 5: Smart Contract Security

## Auditing, Formal Verification, and Vulnerability Prevention

### Securing Billions in On-Chain Value

---

## Overview

Smart contract security is paramount in blockchain finance. With billions of dollars at stake, even minor vulnerabilities can lead to catastrophic losses. This chapter covers comprehensive security practices including common vulnerability patterns, audit methodologies, formal verification techniques, and incident response procedures.

---

## Common Vulnerability Patterns

### Vulnerability Classification

**OWASP Smart Contract Top 10:**

| Rank | Vulnerability | Impact | Prevalence |
|------|---------------|--------|------------|
| 1 | Reentrancy | Critical | High |
| 2 | Access Control | Critical | High |
| 3 | Arithmetic Issues | High | Medium |
| 4 | Unchecked Return Values | Medium | High |
| 5 | Denial of Service | High | Medium |
| 6 | Front-Running | Medium | High |
| 7 | Oracle Manipulation | Critical | Medium |
| 8 | Logic Errors | Critical | Medium |
| 9 | Timestamp Dependence | Low | Medium |
| 10 | Gas Griefing | Medium | Low |

### Reentrancy Attacks

**Classic Reentrancy:**

```solidity
// VULNERABLE CODE - DO NOT USE
contract VulnerableVault {
    mapping(address => uint256) public balances;

    function withdraw(uint256 amount) external {
        require(balances[msg.sender] >= amount, "Insufficient balance");

        // VULNERABILITY: External call before state update
        (bool success, ) = msg.sender.call{value: amount}("");
        require(success, "Transfer failed");

        // State updated after external call - can be re-entered
        balances[msg.sender] -= amount;
    }
}

// ATTACK CONTRACT
contract ReentrancyAttacker {
    VulnerableVault public vault;
    uint256 public attackCount;

    constructor(address _vault) {
        vault = VulnerableVault(_vault);
    }

    function attack() external payable {
        vault.deposit{value: msg.value}();
        vault.withdraw(msg.value);
    }

    receive() external payable {
        if (address(vault).balance >= msg.value && attackCount < 10) {
            attackCount++;
            vault.withdraw(msg.value);
        }
    }
}
```

**Secure Implementation:**

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/security/ReentrancyGuard.sol";

/**
 * @title SecureVault
 * @dev Vault with reentrancy protection
 */
contract SecureVault is ReentrancyGuard {
    mapping(address => uint256) public balances;

    event Deposit(address indexed user, uint256 amount);
    event Withdrawal(address indexed user, uint256 amount);

    function deposit() external payable {
        balances[msg.sender] += msg.value;
        emit Deposit(msg.sender, msg.value);
    }

    function withdraw(uint256 amount) external nonReentrant {
        require(balances[msg.sender] >= amount, "Insufficient balance");

        // Checks-Effects-Interactions pattern
        // 1. CHECKS - already done above
        // 2. EFFECTS - update state BEFORE external call
        balances[msg.sender] -= amount;

        // 3. INTERACTIONS - external call last
        (bool success, ) = msg.sender.call{value: amount}("");
        require(success, "Transfer failed");

        emit Withdrawal(msg.sender, amount);
    }

    function getBalance() external view returns (uint256) {
        return balances[msg.sender];
    }
}
```

### Access Control Vulnerabilities

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/access/AccessControl.sol";
import "@openzeppelin/contracts/access/Ownable2Step.sol";

/**
 * @title SecureAccessControl
 * @dev Comprehensive access control patterns
 */
contract SecureAccessControl is AccessControl, Ownable2Step {
    bytes32 public constant ADMIN_ROLE = keccak256("ADMIN_ROLE");
    bytes32 public constant OPERATOR_ROLE = keccak256("OPERATOR_ROLE");
    bytes32 public constant PAUSER_ROLE = keccak256("PAUSER_ROLE");

    // Time-locked operations
    mapping(bytes32 => uint256) public pendingOperations;
    uint256 public constant TIMELOCK_DELAY = 2 days;

    // Multi-signature requirements
    mapping(bytes32 => mapping(address => bool)) public approvals;
    uint256 public requiredApprovals = 2;

    event OperationScheduled(bytes32 indexed operationId, uint256 executeTime);
    event OperationExecuted(bytes32 indexed operationId);
    event OperationApproved(bytes32 indexed operationId, address indexed approver);

    constructor() Ownable(msg.sender) {
        _grantRole(DEFAULT_ADMIN_ROLE, msg.sender);
        _grantRole(ADMIN_ROLE, msg.sender);
    }

    /**
     * @dev Schedule a timelocked operation
     */
    function scheduleOperation(bytes32 operationId) external onlyRole(ADMIN_ROLE) {
        require(pendingOperations[operationId] == 0, "Already scheduled");
        pendingOperations[operationId] = block.timestamp + TIMELOCK_DELAY;
        emit OperationScheduled(operationId, pendingOperations[operationId]);
    }

    /**
     * @dev Approve a pending operation (multi-sig)
     */
    function approveOperation(bytes32 operationId) external onlyRole(ADMIN_ROLE) {
        require(pendingOperations[operationId] != 0, "Not scheduled");
        require(!approvals[operationId][msg.sender], "Already approved");

        approvals[operationId][msg.sender] = true;
        emit OperationApproved(operationId, msg.sender);
    }

    /**
     * @dev Execute a timelocked operation
     */
    function executeOperation(bytes32 operationId) external onlyRole(ADMIN_ROLE) {
        require(pendingOperations[operationId] != 0, "Not scheduled");
        require(
            block.timestamp >= pendingOperations[operationId],
            "Timelock not expired"
        );
        require(_countApprovals(operationId) >= requiredApprovals, "Insufficient approvals");

        pendingOperations[operationId] = 0;
        emit OperationExecuted(operationId);

        // Execute the operation based on operationId
    }

    /**
     * @dev Cancel a pending operation
     */
    function cancelOperation(bytes32 operationId) external onlyRole(ADMIN_ROLE) {
        require(pendingOperations[operationId] != 0, "Not scheduled");
        pendingOperations[operationId] = 0;
    }

    /**
     * @dev Count approvals for an operation
     */
    function _countApprovals(bytes32 operationId) internal view returns (uint256 count) {
        uint256 adminCount = getRoleMemberCount(ADMIN_ROLE);
        for (uint256 i = 0; i < adminCount; i++) {
            if (approvals[operationId][getRoleMember(ADMIN_ROLE, i)]) {
                count++;
            }
        }
    }

    /**
     * @dev Override renounceRole to prevent accidental lockout
     */
    function renounceRole(bytes32 role, address account) public override {
        require(
            role != DEFAULT_ADMIN_ROLE || getRoleMemberCount(DEFAULT_ADMIN_ROLE) > 1,
            "Cannot renounce last admin"
        );
        super.renounceRole(role, account);
    }
}
```

### Arithmetic Vulnerabilities

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

/**
 * @title SafeMath Examples
 * @dev Arithmetic safety patterns (Solidity 0.8+ has built-in overflow checks)
 */
contract ArithmeticSafety {
    // Pre-0.8 vulnerability (no longer applies with default checks)
    // But unchecked blocks can reintroduce issues

    /**
     * @dev Safe percentage calculation avoiding overflow
     */
    function calculatePercentage(
        uint256 value,
        uint256 percentage
    ) public pure returns (uint256) {
        // Avoid overflow: value * percentage could overflow
        // Use: (value / 100) * percentage if precision allows
        // Or: value * percentage / 100 with overflow check

        require(percentage <= 10000, "Percentage too high");

        // For large values, divide first
        if (value > type(uint256).max / percentage) {
            return (value / 100) * percentage;
        }

        return (value * percentage) / 100;
    }

    /**
     * @dev Safe division with rounding control
     */
    function divideRoundUp(
        uint256 numerator,
        uint256 denominator
    ) public pure returns (uint256) {
        require(denominator > 0, "Division by zero");

        // Round up: (a + b - 1) / b
        return (numerator + denominator - 1) / denominator;
    }

    /**
     * @dev Precision-safe token amount calculations
     */
    function convertTokenAmount(
        uint256 amount,
        uint8 fromDecimals,
        uint8 toDecimals
    ) public pure returns (uint256) {
        if (fromDecimals == toDecimals) {
            return amount;
        }

        if (fromDecimals > toDecimals) {
            // Divide - potential precision loss
            return amount / 10**(fromDecimals - toDecimals);
        } else {
            // Multiply - potential overflow
            uint256 factor = 10**(toDecimals - fromDecimals);
            require(
                amount <= type(uint256).max / factor,
                "Overflow"
            );
            return amount * factor;
        }
    }

    /**
     * @dev Safe multiplication with overflow check
     */
    function safeMul(uint256 a, uint256 b) public pure returns (uint256) {
        if (a == 0) return 0;
        uint256 c = a * b;
        require(c / a == b, "Multiplication overflow");
        return c;
    }

    /**
     * @dev Example of dangerous unchecked block
     */
    function unsafeIncrement(uint256 value) public pure returns (uint256) {
        // DANGEROUS: Disables overflow checks
        unchecked {
            return value + 1; // Will wrap around at max uint256
        }
    }
}
```

### Oracle Manipulation

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

/**
 * @title OracleSecurityPatterns
 * @dev Patterns to prevent oracle manipulation
 */
contract OracleSecurityPatterns {
    struct PriceData {
        uint256 price;
        uint256 timestamp;
        uint256 confidence;
    }

    mapping(address => PriceData[]) public priceHistory;

    uint256 public constant MAX_PRICE_DEVIATION = 10; // 10%
    uint256 public constant MIN_OBSERVATIONS = 3;
    uint256 public constant TWAP_PERIOD = 30 minutes;

    /**
     * @dev Get Time-Weighted Average Price
     */
    function getTWAP(address asset) public view returns (uint256) {
        PriceData[] storage history = priceHistory[asset];
        require(history.length >= MIN_OBSERVATIONS, "Insufficient data");

        uint256 weightedSum;
        uint256 totalWeight;
        uint256 cutoff = block.timestamp - TWAP_PERIOD;

        for (uint256 i = history.length; i > 0; i--) {
            PriceData memory data = history[i - 1];
            if (data.timestamp < cutoff) break;

            uint256 weight = data.timestamp - cutoff;
            weightedSum += data.price * weight;
            totalWeight += weight;
        }

        require(totalWeight > 0, "No recent data");
        return weightedSum / totalWeight;
    }

    /**
     * @dev Validate price against deviation threshold
     */
    function validatePriceUpdate(
        address asset,
        uint256 newPrice
    ) public view returns (bool) {
        PriceData[] storage history = priceHistory[asset];
        if (history.length == 0) return true;

        uint256 lastPrice = history[history.length - 1].price;

        // Calculate deviation percentage
        uint256 deviation;
        if (newPrice > lastPrice) {
            deviation = ((newPrice - lastPrice) * 100) / lastPrice;
        } else {
            deviation = ((lastPrice - newPrice) * 100) / lastPrice;
        }

        return deviation <= MAX_PRICE_DEVIATION;
    }

    /**
     * @dev Multi-oracle price aggregation
     */
    function aggregateOraclePrices(
        uint256[] memory prices,
        uint256 threshold
    ) public pure returns (uint256) {
        require(prices.length >= 3, "Need at least 3 oracles");

        // Sort prices
        for (uint256 i = 0; i < prices.length - 1; i++) {
            for (uint256 j = i + 1; j < prices.length; j++) {
                if (prices[j] < prices[i]) {
                    (prices[i], prices[j]) = (prices[j], prices[i]);
                }
            }
        }

        // Use median
        uint256 median = prices[prices.length / 2];

        // Validate all prices are within threshold of median
        for (uint256 i = 0; i < prices.length; i++) {
            uint256 deviation;
            if (prices[i] > median) {
                deviation = ((prices[i] - median) * 100) / median;
            } else {
                deviation = ((median - prices[i]) * 100) / median;
            }
            require(deviation <= threshold, "Oracle deviation too high");
        }

        return median;
    }
}
```

---

## Audit Methodology

### Audit Process

**WIA Audit Framework:**

| Phase | Activities | Deliverables |
|-------|------------|--------------|
| 1. Scoping | Requirements gathering, codebase review | Audit scope document |
| 2. Automated Analysis | Static analysis, fuzzing, symbolic execution | Tool reports |
| 3. Manual Review | Line-by-line code review, logic analysis | Finding list |
| 4. Vulnerability Testing | Exploit development, edge case testing | PoC exploits |
| 5. Reporting | Finding documentation, severity rating | Audit report |
| 6. Remediation | Fix review, retest | Final report |

### Security Checklist

```markdown
## WIA Smart Contract Security Checklist

### Access Control
- [ ] All sensitive functions have proper access modifiers
- [ ] Owner/admin cannot be renounced leaving contract locked
- [ ] Role-based access control uses principle of least privilege
- [ ] Multi-sig required for critical operations
- [ ] Timelock implemented for parameter changes

### Reentrancy
- [ ] External calls follow checks-effects-interactions pattern
- [ ] ReentrancyGuard used on all state-changing functions
- [ ] Cross-function reentrancy considered
- [ ] Cross-contract reentrancy analyzed

### Arithmetic
- [ ] Solidity 0.8+ used (built-in overflow checks)
- [ ] Unchecked blocks justified and reviewed
- [ ] Division by zero prevented
- [ ] Rounding direction appropriate for use case
- [ ] Precision loss acceptable in calculations

### External Calls
- [ ] Return values checked for all external calls
- [ ] Low-level calls (.call) have return value checks
- [ ] Gas stipend appropriate for receiving contracts
- [ ] Fallback/receive functions handle unexpected calls

### Oracle Security
- [ ] TWAP or time-delay used instead of spot prices
- [ ] Multiple oracle sources aggregated
- [ ] Staleness checks implemented
- [ ] Price deviation thresholds enforced
- [ ] Manipulation resistance tested

### Token Handling
- [ ] ERC-20 approve race condition mitigated
- [ ] Fee-on-transfer tokens handled
- [ ] Rebasing tokens handled
- [ ] Token decimals normalized correctly
- [ ] Zero address checks on transfers

### Upgradeability
- [ ] Proxy pattern correctly implemented
- [ ] Storage layout collisions prevented
- [ ] Initializer protected from re-initialization
- [ ] Upgrade authorization secure

### Gas Optimization
- [ ] No unbounded loops that could cause DoS
- [ ] Array operations have reasonable limits
- [ ] Storage access minimized
- [ ] Events emit appropriate data

### Testing
- [ ] 100% line coverage on critical paths
- [ ] Edge cases tested
- [ ] Fuzzing performed
- [ ] Invariant tests written
- [ ] Fork tests against mainnet state
```

### Automated Tools

**Security Tool Suite:**

| Tool | Type | Purpose |
|------|------|---------|
| Slither | Static Analysis | Bug detection, code quality |
| Mythril | Symbolic Execution | Vulnerability detection |
| Echidna | Fuzzer | Property testing |
| Foundry | Testing Framework | Unit/integration tests |
| Certora | Formal Verification | Mathematical proofs |
| 4naly3er | Static Analysis | Gas optimization |

```bash
# Slither analysis
slither . --print human-summary

# Mythril analysis
myth analyze contracts/MyContract.sol

# Echidna fuzzing
echidna-test . --contract MyContractTest --config echidna.yaml

# Foundry testing
forge test --gas-report -vvv
```

---

## Formal Verification

### Certora Prover

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

/**
 * @title VerifiableVault
 * @dev Vault contract with formal verification specs
 */
contract VerifiableVault {
    mapping(address => uint256) public balances;
    uint256 public totalDeposits;

    // Invariant: totalDeposits == sum of all balances
    // Verified by Certora

    function deposit() external payable {
        balances[msg.sender] += msg.value;
        totalDeposits += msg.value;
    }

    function withdraw(uint256 amount) external {
        require(balances[msg.sender] >= amount, "Insufficient");
        balances[msg.sender] -= amount;
        totalDeposits -= amount;
        payable(msg.sender).transfer(amount);
    }

    function transfer(address to, uint256 amount) external {
        require(balances[msg.sender] >= amount, "Insufficient");
        balances[msg.sender] -= amount;
        balances[to] += amount;
        // totalDeposits unchanged - transfer is internal
    }
}
```

**Certora Spec:**

```cvl
// VerifiableVault.spec
methods {
    function balances(address) external returns (uint256) envfree;
    function totalDeposits() external returns (uint256) envfree;
    function deposit() external payable;
    function withdraw(uint256) external;
    function transfer(address, uint256) external;
}

// Ghost variable to track sum of balances
ghost mathint sumBalances {
    init_state axiom sumBalances == 0;
}

// Update ghost on balance changes
hook Sstore balances[KEY address a] uint256 newValue (uint256 oldValue) STORAGE {
    sumBalances = sumBalances + newValue - oldValue;
}

// Invariant: totalDeposits equals sum of all balances
invariant totalEqualsSum()
    to_mathint(totalDeposits()) == sumBalances;

// Rule: deposit increases balance by msg.value
rule depositIncreasesBalance(env e) {
    uint256 balanceBefore = balances(e.msg.sender);
    uint256 depositAmount = e.msg.value;

    deposit(e);

    uint256 balanceAfter = balances(e.msg.sender);
    assert balanceAfter == balanceBefore + depositAmount;
}

// Rule: withdraw decreases balance correctly
rule withdrawDecreasesBalance(env e, uint256 amount) {
    uint256 balanceBefore = balances(e.msg.sender);
    require balanceBefore >= amount;

    withdraw(e, amount);

    uint256 balanceAfter = balances(e.msg.sender);
    assert balanceAfter == balanceBefore - amount;
}

// Rule: transfer preserves total deposits
rule transferPreservesTotal(env e, address to, uint256 amount) {
    uint256 totalBefore = totalDeposits();

    transfer(e, to, amount);

    uint256 totalAfter = totalDeposits();
    assert totalAfter == totalBefore;
}

// Rule: no unauthorized withdrawal
rule noUnauthorizedWithdrawal(env e, uint256 amount) {
    address user;
    require user != e.msg.sender;

    uint256 userBalanceBefore = balances(user);

    withdraw(e, amount);

    uint256 userBalanceAfter = balances(user);
    assert userBalanceAfter >= userBalanceBefore;
}
```

---

## Incident Response

### Security Monitoring

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/security/Pausable.sol";
import "@openzeppelin/contracts/access/AccessControl.sol";

/**
 * @title EmergencyResponse
 * @dev Contract with comprehensive emergency controls
 */
contract EmergencyResponse is Pausable, AccessControl {
    bytes32 public constant GUARDIAN_ROLE = keccak256("GUARDIAN_ROLE");
    bytes32 public constant EMERGENCY_ROLE = keccak256("EMERGENCY_ROLE");

    // Circuit breaker thresholds
    uint256 public maxSingleWithdrawal;
    uint256 public dailyWithdrawalLimit;
    uint256 public cooldownPeriod;

    // Withdrawal tracking
    mapping(address => uint256) public dailyWithdrawn;
    mapping(address => uint256) public lastWithdrawalDay;
    uint256 public totalDailyWithdrawn;
    uint256 public lastResetDay;

    // Emergency shutdown
    bool public emergencyShutdown;
    uint256 public emergencyShutdownTime;

    event EmergencyShutdownActivated(address indexed guardian);
    event CircuitBreakerTriggered(string reason);
    event EmergencyWithdrawal(address indexed user, uint256 amount);

    modifier notShutdown() {
        require(!emergencyShutdown, "Emergency shutdown active");
        _;
    }

    modifier circuitBreaker(uint256 amount) {
        // Check single withdrawal limit
        require(amount <= maxSingleWithdrawal, "Exceeds single limit");

        // Check daily limit
        uint256 currentDay = block.timestamp / 1 days;
        if (currentDay > lastResetDay) {
            totalDailyWithdrawn = 0;
            lastResetDay = currentDay;
        }

        require(
            totalDailyWithdrawn + amount <= dailyWithdrawalLimit,
            "Daily limit exceeded"
        );

        _;

        totalDailyWithdrawn += amount;
    }

    /**
     * @dev Activate emergency shutdown
     */
    function activateEmergencyShutdown() external onlyRole(GUARDIAN_ROLE) {
        emergencyShutdown = true;
        emergencyShutdownTime = block.timestamp;
        _pause();
        emit EmergencyShutdownActivated(msg.sender);
    }

    /**
     * @dev Deactivate emergency shutdown (requires timelock)
     */
    function deactivateEmergencyShutdown() external onlyRole(DEFAULT_ADMIN_ROLE) {
        require(emergencyShutdown, "Not in shutdown");
        require(
            block.timestamp >= emergencyShutdownTime + 7 days,
            "Cooldown not elapsed"
        );
        emergencyShutdown = false;
        _unpause();
    }

    /**
     * @dev Emergency withdrawal during shutdown
     */
    function emergencyWithdraw() external {
        require(emergencyShutdown, "Not in emergency");
        // Allow users to withdraw their funds
        // Implementation depends on contract type
    }

    /**
     * @dev Update circuit breaker parameters
     */
    function updateCircuitBreaker(
        uint256 _maxSingle,
        uint256 _dailyLimit,
        uint256 _cooldown
    ) external onlyRole(DEFAULT_ADMIN_ROLE) {
        maxSingleWithdrawal = _maxSingle;
        dailyWithdrawalLimit = _dailyLimit;
        cooldownPeriod = _cooldown;
    }
}
```

### Bug Bounty Program

**WIA Bug Bounty Tiers:**

| Severity | Impact | Reward Range |
|----------|--------|--------------|
| Critical | Direct fund theft, complete protocol takeover | $100K - $500K |
| High | Fund freeze, significant economic damage | $25K - $100K |
| Medium | Limited impact, partial functionality loss | $5K - $25K |
| Low | Minor issues, informational | $500 - $5K |

---

## Key Takeaways

1. **Reentrancy** remains the most critical vulnerability - always use checks-effects-interactions pattern
2. **Access control** requires multi-layered defense with timelocks and multi-sig
3. **Oracle manipulation** can be mitigated with TWAP, multiple sources, and deviation checks
4. **Formal verification** provides mathematical guarantees for critical invariants
5. **Incident response** requires pre-planned emergency procedures and circuit breakers
6. **Continuous monitoring** and bug bounties are essential for ongoing security

## Review Questions

1. What is the checks-effects-interactions pattern and why is it important?
2. How can oracle manipulation attacks be prevented?
3. What are the key phases of a smart contract security audit?
4. How does formal verification differ from traditional testing?
5. What components should an emergency response system include?
6. How should bug bounty programs be structured for DeFi protocols?

---

**Next Chapter Preview:** Chapter 6 explores the regulatory framework for blockchain finance, covering global regulations, compliance requirements, and legal structuring.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity · Democratize Finance

