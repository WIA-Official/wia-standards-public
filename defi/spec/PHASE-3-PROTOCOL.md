# WIA DeFi Standard
## Phase 3: Protocol Specification

**Version:** 1.0.0
**Standard:** WIA-FIN-006
**Status:** Final
**Last Updated:** December 2025

---

## Table of Contents

1. [Overview](#overview)
2. [Smart Contract Protocols](#smart-contract-protocols)
3. [AMM Mechanisms](#amm-mechanisms)
4. [Lending Protocols](#lending-protocols)
5. [Flash Loan Protocol](#flash-loan-protocol)
6. [Governance Protocol](#governance-protocol)
7. [Oracle Integration](#oracle-integration)
8. [Security Standards](#security-standards)
9. [Cross-Chain Bridges](#cross-chain-bridges)
10. [Upgrade Mechanisms](#upgrade-mechanisms)

---

## Overview

Phase 3 defines the protocol-level specifications for DeFi smart contracts, including AMM algorithms, lending mechanisms, flash loans, governance systems, and security standards. These protocols ensure composability, security, and interoperability across the DeFi ecosystem.

### Core Principles

- **Trustless**: No centralized control or intermediaries
- **Permissionless**: Anyone can participate without approval
- **Composable**: Protocols integrate seamlessly (money legos)
- **Transparent**: All transactions visible on-chain
- **Non-custodial**: Users retain control of their assets

### Philosophy: 弘益人間 (Hongik Ingan)

> "Benefit All Humanity"

DeFi protocols should be designed to benefit all participants fairly, promoting financial inclusion and democratizing access to financial services globally.

---

## Smart Contract Protocols

### ERC-20 Token Interface

Standard interface for fungible tokens:

```solidity
interface IERC20 {
    function totalSupply() external view returns (uint256);
    function balanceOf(address account) external view returns (uint256);
    function transfer(address to, uint256 amount) external returns (bool);
    function allowance(address owner, address spender) external view returns (uint256);
    function approve(address spender, uint256 amount) external returns (bool);
    function transferFrom(address from, address to, uint256 amount) external returns (bool);

    event Transfer(address indexed from, address indexed to, uint256 value);
    event Approval(address indexed owner, address indexed spender, uint256 value);
}
```

### LP Token Standard

Liquidity Provider tokens represent shares in a pool:

```solidity
interface ILPToken is IERC20 {
    function mint(address to, uint256 amount) external;
    function burn(address from, uint256 amount) external;
    function getReserves() external view returns (uint112 reserve0, uint112 reserve1);
    function price0CumulativeLast() external view returns (uint256);
    function price1CumulativeLast() external view returns (uint256);
}
```

---

## AMM Mechanisms

### Constant Product Market Maker (Uniswap V2)

**Formula:** `x * y = k`

Where:
- `x` = Reserve of token 0
- `y` = Reserve of token 1
- `k` = Constant product

```solidity
contract UniswapV2Pair {
    uint112 private reserve0;
    uint112 private reserve1;
    uint256 public kLast;

    // Swap function
    function swap(
        uint amount0Out,
        uint amount1Out,
        address to
    ) external {
        require(amount0Out > 0 || amount1Out > 0, 'INSUFFICIENT_OUTPUT_AMOUNT');

        (uint112 _reserve0, uint112 _reserve1) = getReserves();
        require(amount0Out < _reserve0 && amount1Out < _reserve1, 'INSUFFICIENT_LIQUIDITY');

        uint balance0;
        uint balance1;
        {
            balance0 = IERC20(token0).balanceOf(address(this));
            balance1 = IERC20(token1).balanceOf(address(this));
        }

        uint amount0In = balance0 > _reserve0 - amount0Out ? balance0 - (_reserve0 - amount0Out) : 0;
        uint amount1In = balance1 > _reserve1 - amount1Out ? balance1 - (_reserve1 - amount1Out) : 0;

        require(amount0In > 0 || amount1In > 0, 'INSUFFICIENT_INPUT_AMOUNT');

        {
            uint balance0Adjusted = balance0.mul(1000).sub(amount0In.mul(3));
            uint balance1Adjusted = balance1.mul(1000).sub(amount1In.mul(3));
            require(
                balance0Adjusted.mul(balance1Adjusted) >= uint(_reserve0).mul(_reserve1).mul(1000**2),
                'K'
            );
        }

        _update(balance0, balance1, _reserve0, _reserve1);
        emit Swap(msg.sender, amount0In, amount1In, amount0Out, amount1Out, to);
    }
}
```

**Price Calculation:**

```
price = y / x
```

**Output Amount (with 0.3% fee):**

```
amountOut = (amountIn * 997 * reserveOut) / (reserveIn * 1000 + amountIn * 997)
```

### Concentrated Liquidity (Uniswap V3)

**Formula:** `L = sqrt(x * y)`

Where:
- `L` = Liquidity
- Positions are defined by price ranges [pL, pH]

```solidity
contract UniswapV3Pool {
    struct Position {
        uint128 liquidity;
        uint256 feeGrowthInside0LastX128;
        uint256 feeGrowthInside1LastX128;
        uint128 tokensOwed0;
        uint128 tokensOwed1;
    }

    mapping(bytes32 => Position) public positions;

    // Mint liquidity
    function mint(
        address recipient,
        int24 tickLower,
        int24 tickUpper,
        uint128 amount,
        bytes calldata data
    ) external returns (uint256 amount0, uint256 amount1) {
        require(amount > 0, 'ZERO_LIQUIDITY');

        (int256 amount0Int, int256 amount1Int) = _modifyPosition(
            ModifyPositionParams({
                owner: recipient,
                tickLower: tickLower,
                tickUpper: tickUpper,
                liquidityDelta: int128(amount)
            })
        );

        amount0 = uint256(amount0Int);
        amount1 = uint256(amount1Int);

        if (amount0 > 0) TransferHelper.safeTransferFrom(token0, msg.sender, address(this), amount0);
        if (amount1 > 0) TransferHelper.safeTransferFrom(token1, msg.sender, address(this), amount1);

        emit Mint(msg.sender, recipient, tickLower, tickUpper, amount, amount0, amount1);
    }
}
```

**Price-Liquidity Relationship:**

```
P = (sqrt(P) + L / x)^2
```

### Stable Swap (Curve)

**Formula:**
```
A * n^n * sum(x_i) + D = A * D * n^n + D^(n+1) / (n^n * prod(x_i))
```

Where:
- `A` = Amplification coefficient
- `n` = Number of tokens
- `D` = Total balance
- `x_i` = Balance of token i

```solidity
contract StableSwap {
    uint256 public A;  // Amplification coefficient

    function get_D(uint256[] memory xp, uint256 amp) internal pure returns (uint256) {
        uint256 S;
        uint256 Dprev;
        uint256 D;

        for (uint i = 0; i < xp.length; i++) {
            S += xp[i];
        }

        if (S == 0) return 0;

        D = S;
        uint256 Ann = amp * xp.length;

        for (uint i = 0; i < 255; i++) {
            uint256 D_P = D;
            for (uint j = 0; j < xp.length; j++) {
                D_P = D_P * D / (xp[j] * xp.length);
            }
            Dprev = D;
            D = (Ann * S + D_P * xp.length) * D / ((Ann - 1) * D + (xp.length + 1) * D_P);

            if (D > Dprev) {
                if (D - Dprev <= 1) break;
            } else {
                if (Dprev - D <= 1) break;
            }
        }

        return D;
    }
}
```

---

## Lending Protocols

### Aave V3 Protocol

**Interest Rate Model:**

```
Utilization Rate (U) = Total Borrowed / Total Supplied

Variable Borrow Rate:
- If U < U_optimal: R_base + (U / U_optimal) * R_slope1
- If U >= U_optimal: R_base + R_slope1 + ((U - U_optimal) / (1 - U_optimal)) * R_slope2

Supply Rate = Borrow Rate * U * (1 - Reserve Factor)
```

```solidity
contract AaveV3Pool {
    struct ReserveData {
        ReserveConfigurationMap configuration;
        uint128 liquidityIndex;
        uint128 variableBorrowIndex;
        uint128 currentLiquidityRate;
        uint128 currentVariableBorrowRate;
        uint40 lastUpdateTimestamp;
        address aTokenAddress;
        address stableDebtTokenAddress;
        address variableDebtTokenAddress;
        address interestRateStrategyAddress;
    }

    function supply(
        address asset,
        uint256 amount,
        address onBehalfOf,
        uint16 referralCode
    ) external {
        DataTypes.ReserveData storage reserve = _reserves[asset];

        ValidationLogic.validateSupply(reserve, amount);

        reserve.updateState();
        reserve.updateInterestRates(asset, amount, 0);

        IERC20(asset).safeTransferFrom(msg.sender, reserve.aTokenAddress, amount);

        IAToken(reserve.aTokenAddress).mint(onBehalfOf, amount, reserve.liquidityIndex);

        emit Supply(asset, msg.sender, onBehalfOf, amount, referralCode);
    }

    function borrow(
        address asset,
        uint256 amount,
        uint256 interestRateMode,
        uint16 referralCode,
        address onBehalfOf
    ) external {
        DataTypes.ReserveData storage reserve = _reserves[asset];

        ValidationLogic.validateBorrow(
            asset,
            reserve,
            onBehalfOf,
            amount,
            interestRateMode,
            _reserves,
            _usersConfig[onBehalfOf]
        );

        reserve.updateState();

        uint256 currentVariableDebt = IVariableDebtToken(reserve.variableDebtTokenAddress)
            .balanceOf(onBehalfOf);

        IVariableDebtToken(reserve.variableDebtTokenAddress).mint(
            onBehalfOf,
            onBehalfOf,
            amount,
            reserve.variableBorrowIndex
        );

        reserve.updateInterestRates(asset, 0, amount);

        IAToken(reserve.aTokenAddress).transferUnderlyingTo(msg.sender, amount);

        emit Borrow(asset, msg.sender, onBehalfOf, amount, interestRateMode, reserve.currentVariableBorrowRate, referralCode);
    }
}
```

**Health Factor:**

```
Health Factor = (Total Collateral in ETH * Liquidation Threshold) / Total Debt in ETH

If Health Factor < 1.0, position can be liquidated
```

**Liquidation:**

```solidity
function liquidationCall(
    address collateralAsset,
    address debtAsset,
    address user,
    uint256 debtToCover,
    bool receiveAToken
) external {
    require(healthFactor < 1e18, 'HEALTH_FACTOR_NOT_BELOW_THRESHOLD');

    // Calculate liquidation bonus
    uint256 liquidationBonus = collateralReserve.configuration.getLiquidationBonus();

    // Transfer debt tokens from liquidator
    // Transfer collateral + bonus to liquidator

    emit LiquidationCall(collateralAsset, debtAsset, user, debtToCover, liquidatedCollateralAmount, liquidator, receiveAToken);
}
```

---

## Flash Loan Protocol

### Aave Flash Loan

```solidity
interface IFlashLoanReceiver {
    function executeOperation(
        address[] calldata assets,
        uint256[] calldata amounts,
        uint256[] calldata premiums,
        address initiator,
        bytes calldata params
    ) external returns (bool);
}

contract FlashLoan {
    uint256 public FLASHLOAN_PREMIUM_TOTAL = 9; // 0.09%

    function flashLoan(
        address receiverAddress,
        address[] calldata assets,
        uint256[] calldata amounts,
        uint256[] calldata modes,
        address onBehalfOf,
        bytes calldata params,
        uint16 referralCode
    ) external {
        FlashloanLocalVars memory vars;

        vars.premiums = new uint256[](assets.length);

        for (vars.i = 0; vars.i < assets.length; vars.i++) {
            vars.premiums[vars.i] = amounts[vars.i].mul(FLASHLOAN_PREMIUM_TOTAL).div(10000);

            IAToken(vars.aTokenAddress).transferUnderlyingTo(receiverAddress, amounts[vars.i]);
        }

        require(
            IFlashLoanReceiver(receiverAddress).executeOperation(
                assets,
                amounts,
                vars.premiums,
                msg.sender,
                params
            ),
            'INVALID_FLASHLOAN_EXECUTOR_RETURN'
        );

        for (vars.i = 0; vars.i < assets.length; vars.i++) {
            vars.currentAmount = IERC20(assets[vars.i]).balanceOf(vars.aTokenAddress);
            vars.currentAmountPlusPremium = amounts[vars.i].add(vars.premiums[vars.i]);

            require(
                vars.currentAmount >= vars.currentAmountPlusPremium,
                'INVALID_FLASH_LOAN_BALANCE'
            );
        }

        emit FlashLoan(receiverAddress, msg.sender, assets, amounts, vars.premiums, referralCode);
    }
}
```

**Flash Loan Use Cases:**

1. **Arbitrage:**
   - Borrow asset
   - Buy on DEX A
   - Sell on DEX B
   - Repay loan + fee
   - Keep profit

2. **Collateral Swap:**
   - Borrow debt amount
   - Repay debt
   - Withdraw collateral A
   - Deposit collateral B
   - Borrow new debt
   - Repay flash loan

3. **Liquidation:**
   - Borrow asset
   - Liquidate underwater position
   - Swap collateral for debt asset
   - Repay flash loan
   - Keep bonus

---

## Governance Protocol

### Compound-Style Governor

```solidity
contract GovernorBravo {
    struct Proposal {
        uint id;
        address proposer;
        uint eta;
        address[] targets;
        uint[] values;
        string[] signatures;
        bytes[] calldatas;
        uint startBlock;
        uint endBlock;
        uint forVotes;
        uint againstVotes;
        uint abstainVotes;
        bool canceled;
        bool executed;
        mapping (address => Receipt) receipts;
    }

    struct Receipt {
        bool hasVoted;
        uint8 support;
        uint96 votes;
    }

    function propose(
        address[] memory targets,
        uint[] memory values,
        string[] memory signatures,
        bytes[] memory calldatas,
        string memory description
    ) public returns (uint) {
        require(
            getPriorVotes(msg.sender, sub256(block.number, 1)) > proposalThreshold(),
            "proposer votes below proposal threshold"
        );

        uint proposalId = proposalCount++;
        Proposal storage newProposal = proposals[proposalId];

        newProposal.id = proposalId;
        newProposal.proposer = msg.sender;
        newProposal.targets = targets;
        newProposal.values = values;
        newProposal.signatures = signatures;
        newProposal.calldatas = calldatas;
        newProposal.startBlock = add256(block.number, votingDelay());
        newProposal.endBlock = add256(newProposal.startBlock, votingPeriod());

        emit ProposalCreated(proposalId, msg.sender, targets, values, signatures, calldatas, newProposal.startBlock, newProposal.endBlock, description);

        return proposalId;
    }

    function castVote(uint proposalId, uint8 support) external {
        return _castVote(msg.sender, proposalId, support);
    }

    function _castVote(address voter, uint proposalId, uint8 support) internal {
        require(state(proposalId) == ProposalState.Active, "voting is closed");
        Proposal storage proposal = proposals[proposalId];
        Receipt storage receipt = proposal.receipts[voter];
        require(receipt.hasVoted == false, "voter already voted");

        uint96 votes = getPriorVotes(voter, proposal.startBlock);

        if (support == 0) {
            proposal.againstVotes = add256(proposal.againstVotes, votes);
        } else if (support == 1) {
            proposal.forVotes = add256(proposal.forVotes, votes);
        } else if (support == 2) {
            proposal.abstainVotes = add256(proposal.abstainVotes, votes);
        }

        receipt.hasVoted = true;
        receipt.support = support;
        receipt.votes = votes;

        emit VoteCast(voter, proposalId, support, votes, "");
    }
}
```

**Voting Requirements:**

```
Quorum = 4% of total token supply
Proposal Threshold = 0.25% of total token supply
Voting Period = 3 days (17,280 blocks)
Voting Delay = 1 day (5,760 blocks)
Timelock = 2 days after proposal passes
```

---

## Oracle Integration

### Chainlink Price Feeds

```solidity
interface AggregatorV3Interface {
    function latestRoundData() external view returns (
        uint80 roundId,
        int256 answer,
        uint256 startedAt,
        uint256 updatedAt,
        uint80 answeredInRound
    );
}

contract PriceOracle {
    mapping(address => address) public priceFeeds;

    function getPrice(address token) public view returns (uint256) {
        AggregatorV3Interface priceFeed = AggregatorV3Interface(priceFeeds[token]);

        (
            uint80 roundId,
            int256 price,
            uint256 startedAt,
            uint256 updatedAt,
            uint80 answeredInRound
        ) = priceFeed.latestRoundData();

        require(price > 0, "Invalid price");
        require(updatedAt > block.timestamp - 3600, "Stale price");

        return uint256(price);
    }
}
```

### TWAP Oracle (Uniswap V3)

```solidity
contract TWAPOracle {
    function consult(
        address pool,
        uint32 period
    ) public view returns (uint256 price) {
        uint32[] memory secondsAgos = new uint32[](2);
        secondsAgos[0] = period;
        secondsAgos[1] = 0;

        (int56[] memory tickCumulatives,) = IUniswapV3Pool(pool).observe(secondsAgos);

        int56 tickCumulativesDelta = tickCumulatives[1] - tickCumulatives[0];
        int24 tick = int24(tickCumulativesDelta / period);

        price = getSqrtRatioAtTick(tick);
    }
}
```

---

## Security Standards

### Access Control

```solidity
contract AccessControl {
    bytes32 public constant ADMIN_ROLE = keccak256("ADMIN_ROLE");
    bytes32 public constant PAUSER_ROLE = keccak256("PAUSER_ROLE");

    mapping(bytes32 => mapping(address => bool)) private _roles;

    modifier onlyRole(bytes32 role) {
        require(hasRole(role, msg.sender), "AccessControl: unauthorized");
        _;
    }

    function hasRole(bytes32 role, address account) public view returns (bool) {
        return _roles[role][account];
    }
}
```

### Emergency Pause

```solidity
contract Pausable {
    bool private _paused;

    event Paused(address account);
    event Unpaused(address account);

    modifier whenNotPaused() {
        require(!_paused, "Pausable: paused");
        _;
    }

    modifier whenPaused() {
        require(_paused, "Pausable: not paused");
        _;
    }

    function pause() public onlyRole(PAUSER_ROLE) whenNotPaused {
        _paused = true;
        emit Paused(msg.sender);
    }

    function unpause() public onlyRole(PAUSER_ROLE) whenPaused {
        _paused = false;
        emit Unpaused(msg.sender);
    }
}
```

### Reentrancy Guard

```solidity
contract ReentrancyGuard {
    uint256 private constant _NOT_ENTERED = 1;
    uint256 private constant _ENTERED = 2;
    uint256 private _status;

    modifier nonReentrant() {
        require(_status != _ENTERED, "ReentrancyGuard: reentrant call");
        _status = _ENTERED;
        _;
        _status = _NOT_ENTERED;
    }
}
```

---

## Cross-Chain Bridges

### LayerZero Bridge

```solidity
interface ILayerZeroEndpoint {
    function send(
        uint16 _dstChainId,
        bytes calldata _destination,
        bytes calldata _payload,
        address payable _refundAddress,
        address _zroPaymentAddress,
        bytes calldata _adapterParams
    ) external payable;
}

contract OmniChainToken {
    ILayerZeroEndpoint public lzEndpoint;

    function sendFrom(
        address _from,
        uint16 _dstChainId,
        bytes calldata _toAddress,
        uint _amount,
        address payable _refundAddress,
        address _zroPaymentAddress,
        bytes calldata _adapterParams
    ) public payable {
        _burn(_from, _amount);

        bytes memory payload = abi.encode(_toAddress, _amount);

        lzEndpoint.send{value: msg.value}(
            _dstChainId,
            _toAddress,
            payload,
            _refundAddress,
            _zroPaymentAddress,
            _adapterParams
        );
    }

    function lzReceive(
        uint16 _srcChainId,
        bytes memory _srcAddress,
        uint64 _nonce,
        bytes memory _payload
    ) external {
        require(msg.sender == address(lzEndpoint));

        (address toAddress, uint256 amount) = abi.decode(_payload, (address, uint256));

        _mint(toAddress, amount);
    }
}
```

---

## Upgrade Mechanisms

### Transparent Proxy Pattern

```solidity
contract TransparentUpgradeableProxy is ERC1967Proxy {
    constructor(
        address _logic,
        address admin_,
        bytes memory _data
    ) payable ERC1967Proxy(_logic, _data) {
        _changeAdmin(admin_);
    }

    modifier ifAdmin() {
        if (msg.sender == _getAdmin()) {
            _;
        } else {
            _fallback();
        }
    }

    function upgradeTo(address newImplementation) external ifAdmin {
        _upgradeToAndCall(newImplementation, bytes(""), false);
    }
}
```

---

**Version:** 1.0.0
**Last Updated:** December 2025
**Status:** Final

**弘益人間 (Hongik Ingan) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
