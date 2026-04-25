# Chapter 4: DeFi Protocols

## Automated Market Makers, Lending Protocols, and Derivatives Platforms

### The Building Blocks of Decentralized Finance

---

## Overview

Decentralized Finance (DeFi) protocols form the foundation of programmable financial services. This chapter explores the technical architecture, mathematical models, and implementation patterns of core DeFi primitives including automated market makers, lending protocols, and derivatives platforms.

---

## Automated Market Makers (AMMs)

### Constant Product Market Maker

**Uniswap V2 Model:**

The constant product formula `x * y = k` ensures liquidity at any price point:

```
x = reserve of token X
y = reserve of token Y
k = constant product (invariant)
```

**Price Discovery:**

```
Price of Y in terms of X = x / y
Price of X in terms of Y = y / x
```

**Swap Calculation:**

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

/**
 * @title ConstantProductAMM
 * @dev Basic constant product AMM implementation
 */
contract ConstantProductAMM {
    IERC20 public tokenX;
    IERC20 public tokenY;

    uint256 public reserveX;
    uint256 public reserveY;

    uint256 public totalLiquidity;
    mapping(address => uint256) public liquidity;

    uint256 public constant FEE_NUMERATOR = 997;
    uint256 public constant FEE_DENOMINATOR = 1000;

    event Swap(
        address indexed sender,
        uint256 amountXIn,
        uint256 amountYIn,
        uint256 amountXOut,
        uint256 amountYOut
    );
    event AddLiquidity(
        address indexed provider,
        uint256 amountX,
        uint256 amountY,
        uint256 liquidity
    );
    event RemoveLiquidity(
        address indexed provider,
        uint256 amountX,
        uint256 amountY,
        uint256 liquidity
    );

    constructor(address _tokenX, address _tokenY) {
        tokenX = IERC20(_tokenX);
        tokenY = IERC20(_tokenY);
    }

    /**
     * @dev Add liquidity to the pool
     */
    function addLiquidity(
        uint256 amountX,
        uint256 amountY
    ) external returns (uint256 liquidityMinted) {
        tokenX.transferFrom(msg.sender, address(this), amountX);
        tokenY.transferFrom(msg.sender, address(this), amountY);

        if (totalLiquidity == 0) {
            // Initial liquidity
            liquidityMinted = sqrt(amountX * amountY);
        } else {
            // Proportional liquidity
            liquidityMinted = min(
                (amountX * totalLiquidity) / reserveX,
                (amountY * totalLiquidity) / reserveY
            );
        }

        require(liquidityMinted > 0, "Insufficient liquidity minted");

        liquidity[msg.sender] += liquidityMinted;
        totalLiquidity += liquidityMinted;
        reserveX += amountX;
        reserveY += amountY;

        emit AddLiquidity(msg.sender, amountX, amountY, liquidityMinted);
    }

    /**
     * @dev Remove liquidity from the pool
     */
    function removeLiquidity(
        uint256 liquidityAmount
    ) external returns (uint256 amountX, uint256 amountY) {
        require(liquidity[msg.sender] >= liquidityAmount, "Insufficient liquidity");

        amountX = (liquidityAmount * reserveX) / totalLiquidity;
        amountY = (liquidityAmount * reserveY) / totalLiquidity;

        liquidity[msg.sender] -= liquidityAmount;
        totalLiquidity -= liquidityAmount;
        reserveX -= amountX;
        reserveY -= amountY;

        tokenX.transfer(msg.sender, amountX);
        tokenY.transfer(msg.sender, amountY);

        emit RemoveLiquidity(msg.sender, amountX, amountY, liquidityAmount);
    }

    /**
     * @dev Swap tokenX for tokenY
     */
    function swapXForY(uint256 amountXIn) external returns (uint256 amountYOut) {
        require(amountXIn > 0, "Zero input");

        // Apply fee
        uint256 amountXInWithFee = amountXIn * FEE_NUMERATOR;

        // Calculate output: dy = y * dx / (x + dx)
        amountYOut = (reserveY * amountXInWithFee) /
            (reserveX * FEE_DENOMINATOR + amountXInWithFee);

        require(amountYOut > 0, "Insufficient output");

        // Transfer tokens
        tokenX.transferFrom(msg.sender, address(this), amountXIn);
        tokenY.transfer(msg.sender, amountYOut);

        // Update reserves
        reserveX += amountXIn;
        reserveY -= amountYOut;

        emit Swap(msg.sender, amountXIn, 0, 0, amountYOut);
    }

    /**
     * @dev Swap tokenY for tokenX
     */
    function swapYForX(uint256 amountYIn) external returns (uint256 amountXOut) {
        require(amountYIn > 0, "Zero input");

        uint256 amountYInWithFee = amountYIn * FEE_NUMERATOR;

        amountXOut = (reserveX * amountYInWithFee) /
            (reserveY * FEE_DENOMINATOR + amountYInWithFee);

        require(amountXOut > 0, "Insufficient output");

        tokenY.transferFrom(msg.sender, address(this), amountYIn);
        tokenX.transfer(msg.sender, amountXOut);

        reserveY += amountYIn;
        reserveX -= amountXOut;

        emit Swap(msg.sender, 0, amountYIn, amountXOut, 0);
    }

    /**
     * @dev Get output amount for a given input
     */
    function getAmountOut(
        uint256 amountIn,
        uint256 reserveIn,
        uint256 reserveOut
    ) public pure returns (uint256) {
        uint256 amountInWithFee = amountIn * FEE_NUMERATOR;
        return (reserveOut * amountInWithFee) /
            (reserveIn * FEE_DENOMINATOR + amountInWithFee);
    }

    /**
     * @dev Get current price of Y in terms of X
     */
    function getPriceYinX() external view returns (uint256) {
        return (reserveX * 1e18) / reserveY;
    }

    function sqrt(uint256 x) internal pure returns (uint256) {
        if (x == 0) return 0;
        uint256 z = (x + 1) / 2;
        uint256 y = x;
        while (z < y) {
            y = z;
            z = (x / z + z) / 2;
        }
        return y;
    }

    function min(uint256 a, uint256 b) internal pure returns (uint256) {
        return a < b ? a : b;
    }
}
```

### Concentrated Liquidity (Uniswap V3)

**Price Range Liquidity:**

| Feature | Uniswap V2 | Uniswap V3 |
|---------|------------|------------|
| Liquidity Distribution | Uniform (0, ∞) | Concentrated ranges |
| Capital Efficiency | ~0.5% | Up to 4000x |
| LP Positions | Fungible | NFT (non-fungible) |
| Fee Tiers | 0.3% | 0.01%, 0.05%, 0.3%, 1% |
| Impermanent Loss | Moderate | Higher in range |

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/token/ERC721/ERC721.sol";

/**
 * @title ConcentratedLiquidityAMM
 * @dev Simplified concentrated liquidity implementation
 */
contract ConcentratedLiquidityAMM is ERC721 {
    struct Position {
        uint128 liquidity;
        int24 tickLower;
        int24 tickUpper;
        uint256 tokensOwed0;
        uint256 tokensOwed1;
        uint256 feeGrowthInside0Last;
        uint256 feeGrowthInside1Last;
    }

    struct Tick {
        int128 liquidityNet;
        uint128 liquidityGross;
        uint256 feeGrowthOutside0;
        uint256 feeGrowthOutside1;
        bool initialized;
    }

    IERC20 public token0;
    IERC20 public token1;

    // Current price as sqrt(P) in Q64.96 format
    uint160 public sqrtPriceX96;

    // Current tick
    int24 public currentTick;

    // Liquidity currently in range
    uint128 public liquidity;

    // Fee growth accumulators
    uint256 public feeGrowthGlobal0;
    uint256 public feeGrowthGlobal1;

    // Tick spacing (depends on fee tier)
    int24 public tickSpacing;

    // Fee rate in hundredths of a basis point
    uint24 public fee;

    // Positions and ticks
    mapping(uint256 => Position) public positions;
    mapping(int24 => Tick) public ticks;

    uint256 private _nextPositionId;

    event Mint(
        address indexed owner,
        uint256 indexed positionId,
        int24 tickLower,
        int24 tickUpper,
        uint128 amount
    );
    event Burn(
        uint256 indexed positionId,
        uint128 amount
    );
    event Swap(
        address indexed sender,
        int256 amount0,
        int256 amount1,
        uint160 sqrtPriceX96,
        int24 tick
    );

    constructor(
        address _token0,
        address _token1,
        uint24 _fee
    ) ERC721("CL-LP", "CL-LP") {
        token0 = IERC20(_token0);
        token1 = IERC20(_token1);
        fee = _fee;

        // Set tick spacing based on fee tier
        if (_fee == 100) tickSpacing = 1;
        else if (_fee == 500) tickSpacing = 10;
        else if (_fee == 3000) tickSpacing = 60;
        else if (_fee == 10000) tickSpacing = 200;
    }

    /**
     * @dev Initialize pool with starting price
     */
    function initialize(uint160 _sqrtPriceX96) external {
        require(sqrtPriceX96 == 0, "Already initialized");
        sqrtPriceX96 = _sqrtPriceX96;
        currentTick = getTickAtSqrtRatio(_sqrtPriceX96);
    }

    /**
     * @dev Add liquidity to a price range
     */
    function mint(
        int24 tickLower,
        int24 tickUpper,
        uint128 amount
    ) external returns (uint256 positionId, uint256 amount0, uint256 amount1) {
        require(tickLower < tickUpper, "Invalid tick range");
        require(tickLower % tickSpacing == 0, "Invalid lower tick");
        require(tickUpper % tickSpacing == 0, "Invalid upper tick");

        // Calculate token amounts needed
        (amount0, amount1) = _getAmountsForLiquidity(
            sqrtPriceX96,
            getSqrtRatioAtTick(tickLower),
            getSqrtRatioAtTick(tickUpper),
            amount
        );

        // Transfer tokens
        if (amount0 > 0) token0.transferFrom(msg.sender, address(this), amount0);
        if (amount1 > 0) token1.transferFrom(msg.sender, address(this), amount1);

        // Update ticks
        _updateTick(tickLower, amount, true);
        _updateTick(tickUpper, amount, false);

        // Update liquidity if in range
        if (currentTick >= tickLower && currentTick < tickUpper) {
            liquidity += amount;
        }

        // Mint position NFT
        positionId = _nextPositionId++;
        _mint(msg.sender, positionId);

        positions[positionId] = Position({
            liquidity: amount,
            tickLower: tickLower,
            tickUpper: tickUpper,
            tokensOwed0: 0,
            tokensOwed1: 0,
            feeGrowthInside0Last: _getFeeGrowthInside(tickLower, tickUpper, 0),
            feeGrowthInside1Last: _getFeeGrowthInside(tickLower, tickUpper, 1)
        });

        emit Mint(msg.sender, positionId, tickLower, tickUpper, amount);
    }

    /**
     * @dev Execute swap
     */
    function swap(
        bool zeroForOne,
        int256 amountSpecified,
        uint160 sqrtPriceLimitX96
    ) external returns (int256 amount0, int256 amount1) {
        // Swap implementation with tick crossing
        // Simplified for illustration

        if (zeroForOne) {
            // Swap token0 for token1
            require(sqrtPriceLimitX96 < sqrtPriceX96, "Invalid limit");
        } else {
            // Swap token1 for token0
            require(sqrtPriceLimitX96 > sqrtPriceX96, "Invalid limit");
        }

        // Execute swap steps, crossing ticks as needed
        // ...

        emit Swap(msg.sender, amount0, amount1, sqrtPriceX96, currentTick);
    }

    /**
     * @dev Update tick state
     */
    function _updateTick(
        int24 tick,
        uint128 liquidityDelta,
        bool isLower
    ) internal {
        Tick storage tickInfo = ticks[tick];

        if (!tickInfo.initialized) {
            tickInfo.initialized = true;
            if (tick <= currentTick) {
                tickInfo.feeGrowthOutside0 = feeGrowthGlobal0;
                tickInfo.feeGrowthOutside1 = feeGrowthGlobal1;
            }
        }

        tickInfo.liquidityGross += liquidityDelta;

        if (isLower) {
            tickInfo.liquidityNet += int128(liquidityDelta);
        } else {
            tickInfo.liquidityNet -= int128(liquidityDelta);
        }
    }

    /**
     * @dev Get fee growth inside a tick range
     */
    function _getFeeGrowthInside(
        int24 tickLower,
        int24 tickUpper,
        uint8 tokenIndex
    ) internal view returns (uint256) {
        // Calculate fee growth inside the tick range
        // Implementation details omitted for brevity
        return 0;
    }

    /**
     * @dev Calculate token amounts for liquidity
     */
    function _getAmountsForLiquidity(
        uint160 sqrtRatioX96,
        uint160 sqrtRatioAX96,
        uint160 sqrtRatioBX96,
        uint128 liquidityAmount
    ) internal pure returns (uint256 amount0, uint256 amount1) {
        if (sqrtRatioX96 <= sqrtRatioAX96) {
            // Current price below range - all token0
            amount0 = getAmount0ForLiquidity(sqrtRatioAX96, sqrtRatioBX96, liquidityAmount);
        } else if (sqrtRatioX96 < sqrtRatioBX96) {
            // Current price in range - both tokens
            amount0 = getAmount0ForLiquidity(sqrtRatioX96, sqrtRatioBX96, liquidityAmount);
            amount1 = getAmount1ForLiquidity(sqrtRatioAX96, sqrtRatioX96, liquidityAmount);
        } else {
            // Current price above range - all token1
            amount1 = getAmount1ForLiquidity(sqrtRatioAX96, sqrtRatioBX96, liquidityAmount);
        }
    }

    // Helper functions for price/tick conversions
    function getSqrtRatioAtTick(int24 tick) public pure returns (uint160) {
        // Implementation
        return 0;
    }

    function getTickAtSqrtRatio(uint160 sqrtRatioX96) public pure returns (int24) {
        // Implementation
        return 0;
    }

    function getAmount0ForLiquidity(uint160 sqrtRatioAX96, uint160 sqrtRatioBX96, uint128 liquidityAmount) internal pure returns (uint256) {
        return 0;
    }

    function getAmount1ForLiquidity(uint160 sqrtRatioAX96, uint160 sqrtRatioBX96, uint128 liquidityAmount) internal pure returns (uint256) {
        return 0;
    }
}
```

---

## Lending Protocols

### Compound-Style Money Markets

**Interest Rate Model:**

| Utilization | Borrow Rate | Supply Rate |
|-------------|-------------|-------------|
| 0% | 2% | 0% |
| 50% | 10% | 4.5% |
| 80% (Optimal) | 20% | 14.4% |
| 90% | 80% | 64.8% |
| 100% | 200% | 180% |

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/token/ERC20/IERC20.sol";
import "@openzeppelin/contracts/security/ReentrancyGuard.sol";

/**
 * @title LendingPool
 * @dev Compound-style lending pool with variable interest rates
 */
contract LendingPool is ReentrancyGuard {
    IERC20 public underlyingAsset;

    // Interest rate model parameters (per second, scaled by 1e18)
    uint256 public baseRatePerSecond;
    uint256 public multiplierPerSecond;
    uint256 public jumpMultiplierPerSecond;
    uint256 public optimalUtilization; // Scaled by 1e18

    // Exchange rate parameters
    uint256 public constant INITIAL_EXCHANGE_RATE = 2e16; // 0.02
    uint256 public totalBorrows;
    uint256 public totalReserves;
    uint256 public reserveFactor; // Scaled by 1e18

    // Borrow state
    uint256 public borrowIndex;
    uint256 public lastAccrualTimestamp;

    // User state
    mapping(address => uint256) public supplyBalance; // cToken balance
    mapping(address => uint256) public borrowBalance;
    mapping(address => uint256) public borrowIndex_;

    // Total supply of cTokens
    uint256 public totalSupply;

    event Supply(address indexed user, uint256 amount, uint256 cTokens);
    event Withdraw(address indexed user, uint256 amount, uint256 cTokens);
    event Borrow(address indexed user, uint256 amount);
    event Repay(address indexed payer, address indexed borrower, uint256 amount);
    event AccrueInterest(uint256 interestAccumulated, uint256 borrowIndex, uint256 totalBorrows);

    constructor(
        address _underlying,
        uint256 _baseRate,
        uint256 _multiplier,
        uint256 _jumpMultiplier,
        uint256 _optimal
    ) {
        underlyingAsset = IERC20(_underlying);
        baseRatePerSecond = _baseRate / 365 days;
        multiplierPerSecond = _multiplier / 365 days;
        jumpMultiplierPerSecond = _jumpMultiplier / 365 days;
        optimalUtilization = _optimal;
        borrowIndex = 1e18;
        lastAccrualTimestamp = block.timestamp;
        reserveFactor = 0.1e18; // 10%
    }

    /**
     * @dev Accrue interest and update state
     */
    function accrueInterest() public {
        uint256 currentTimestamp = block.timestamp;
        uint256 timeDelta = currentTimestamp - lastAccrualTimestamp;

        if (timeDelta == 0) return;

        uint256 cashPrior = getCash();
        uint256 borrowsPrior = totalBorrows;
        uint256 reservesPrior = totalReserves;
        uint256 borrowIndexPrior = borrowIndex;

        // Calculate borrow rate
        uint256 borrowRate = getBorrowRateInternal(cashPrior, borrowsPrior, reservesPrior);

        // Calculate interest accumulated
        uint256 interestFactor = borrowRate * timeDelta;
        uint256 interestAccumulated = (borrowsPrior * interestFactor) / 1e18;

        // Update state
        totalBorrows = borrowsPrior + interestAccumulated;
        totalReserves = reservesPrior + (interestAccumulated * reserveFactor) / 1e18;
        borrowIndex = borrowIndexPrior + (borrowIndexPrior * interestFactor) / 1e18;
        lastAccrualTimestamp = currentTimestamp;

        emit AccrueInterest(interestAccumulated, borrowIndex, totalBorrows);
    }

    /**
     * @dev Supply assets to the pool
     */
    function supply(uint256 amount) external nonReentrant returns (uint256) {
        accrueInterest();

        // Transfer underlying
        underlyingAsset.transferFrom(msg.sender, address(this), amount);

        // Calculate cTokens to mint
        uint256 exchangeRate = exchangeRateCurrent();
        uint256 cTokens = (amount * 1e18) / exchangeRate;

        // Update state
        supplyBalance[msg.sender] += cTokens;
        totalSupply += cTokens;

        emit Supply(msg.sender, amount, cTokens);
        return cTokens;
    }

    /**
     * @dev Withdraw assets from the pool
     */
    function withdraw(uint256 cTokenAmount) external nonReentrant returns (uint256) {
        accrueInterest();

        require(supplyBalance[msg.sender] >= cTokenAmount, "Insufficient balance");

        // Calculate underlying to transfer
        uint256 exchangeRate = exchangeRateCurrent();
        uint256 underlyingAmount = (cTokenAmount * exchangeRate) / 1e18;

        require(getCash() >= underlyingAmount, "Insufficient liquidity");

        // Update state
        supplyBalance[msg.sender] -= cTokenAmount;
        totalSupply -= cTokenAmount;

        // Transfer underlying
        underlyingAsset.transfer(msg.sender, underlyingAmount);

        emit Withdraw(msg.sender, underlyingAmount, cTokenAmount);
        return underlyingAmount;
    }

    /**
     * @dev Borrow assets from the pool
     */
    function borrow(uint256 amount) external nonReentrant {
        accrueInterest();

        require(getCash() >= amount, "Insufficient liquidity");

        // Check collateral (simplified - actual implementation needs collateral check)
        // require(checkCollateral(msg.sender, amount), "Insufficient collateral");

        // Update borrow state
        uint256 accountBorrowsPrev = borrowBalanceStored(msg.sender);
        uint256 accountBorrowsNew = accountBorrowsPrev + amount;

        borrowBalance[msg.sender] = accountBorrowsNew;
        borrowIndex_[msg.sender] = borrowIndex;
        totalBorrows += amount;

        // Transfer underlying
        underlyingAsset.transfer(msg.sender, amount);

        emit Borrow(msg.sender, amount);
    }

    /**
     * @dev Repay borrowed assets
     */
    function repay(uint256 amount) external nonReentrant returns (uint256) {
        accrueInterest();

        uint256 accountBorrows = borrowBalanceStored(msg.sender);
        uint256 repayAmount = amount > accountBorrows ? accountBorrows : amount;

        // Transfer underlying
        underlyingAsset.transferFrom(msg.sender, address(this), repayAmount);

        // Update state
        borrowBalance[msg.sender] = accountBorrows - repayAmount;
        borrowIndex_[msg.sender] = borrowIndex;
        totalBorrows -= repayAmount;

        emit Repay(msg.sender, msg.sender, repayAmount);
        return repayAmount;
    }

    /**
     * @dev Get stored borrow balance with accrued interest
     */
    function borrowBalanceStored(address account) public view returns (uint256) {
        if (borrowBalance[account] == 0) return 0;

        uint256 principalTimesIndex = borrowBalance[account] * borrowIndex;
        return principalTimesIndex / borrowIndex_[account];
    }

    /**
     * @dev Calculate current exchange rate
     */
    function exchangeRateCurrent() public view returns (uint256) {
        if (totalSupply == 0) {
            return INITIAL_EXCHANGE_RATE;
        }

        uint256 totalCash = getCash();
        uint256 cashPlusBorrowsMinusReserves = totalCash + totalBorrows - totalReserves;

        return (cashPlusBorrowsMinusReserves * 1e18) / totalSupply;
    }

    /**
     * @dev Get cash (underlying balance)
     */
    function getCash() public view returns (uint256) {
        return underlyingAsset.balanceOf(address(this));
    }

    /**
     * @dev Calculate utilization rate
     */
    function getUtilizationRate() public view returns (uint256) {
        uint256 cash = getCash();
        if (cash + totalBorrows == 0) return 0;

        return (totalBorrows * 1e18) / (cash + totalBorrows - totalReserves);
    }

    /**
     * @dev Calculate borrow rate based on utilization
     */
    function getBorrowRateInternal(
        uint256 cash,
        uint256 borrows,
        uint256 reserves
    ) internal view returns (uint256) {
        uint256 utilization = borrows == 0 ? 0 :
            (borrows * 1e18) / (cash + borrows - reserves);

        if (utilization <= optimalUtilization) {
            return baseRatePerSecond +
                (utilization * multiplierPerSecond) / 1e18;
        } else {
            uint256 normalRate = baseRatePerSecond +
                (optimalUtilization * multiplierPerSecond) / 1e18;
            uint256 excessUtilization = utilization - optimalUtilization;
            return normalRate +
                (excessUtilization * jumpMultiplierPerSecond) / (1e18 - optimalUtilization);
        }
    }

    /**
     * @dev Calculate supply rate
     */
    function getSupplyRate() public view returns (uint256) {
        uint256 borrowRate = getBorrowRateInternal(getCash(), totalBorrows, totalReserves);
        uint256 utilization = getUtilizationRate();
        uint256 oneMinusReserveFactor = 1e18 - reserveFactor;

        return (borrowRate * utilization * oneMinusReserveFactor) / 1e36;
    }
}
```

### Flash Loans

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/token/ERC20/IERC20.sol";
import "@openzeppelin/contracts/security/ReentrancyGuard.sol";

interface IFlashLoanReceiver {
    function executeOperation(
        address asset,
        uint256 amount,
        uint256 premium,
        address initiator,
        bytes calldata params
    ) external returns (bool);
}

/**
 * @title FlashLoanPool
 * @dev Flash loan provider with fee mechanism
 */
contract FlashLoanPool is ReentrancyGuard {
    mapping(address => bool) public supportedAssets;
    uint256 public constant FLASH_LOAN_FEE = 9; // 0.09%

    event FlashLoan(
        address indexed receiver,
        address indexed asset,
        uint256 amount,
        uint256 premium
    );

    /**
     * @dev Execute flash loan
     */
    function flashLoan(
        address receiver,
        address asset,
        uint256 amount,
        bytes calldata params
    ) external nonReentrant {
        require(supportedAssets[asset], "Asset not supported");

        IERC20 token = IERC20(asset);
        uint256 balanceBefore = token.balanceOf(address(this));
        require(balanceBefore >= amount, "Insufficient liquidity");

        // Calculate premium
        uint256 premium = (amount * FLASH_LOAN_FEE) / 10000;

        // Transfer to receiver
        token.transfer(receiver, amount);

        // Execute callback
        require(
            IFlashLoanReceiver(receiver).executeOperation(
                asset,
                amount,
                premium,
                msg.sender,
                params
            ),
            "Flash loan execution failed"
        );

        // Verify repayment
        uint256 balanceAfter = token.balanceOf(address(this));
        require(
            balanceAfter >= balanceBefore + premium,
            "Flash loan not repaid"
        );

        emit FlashLoan(receiver, asset, amount, premium);
    }

    /**
     * @dev Add supported asset
     */
    function addSupportedAsset(address asset) external {
        supportedAssets[asset] = true;
    }
}
```

---

## Derivatives and Synthetic Assets

### Perpetual Futures

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/token/ERC20/IERC20.sol";
import "@openzeppelin/contracts/security/ReentrancyGuard.sol";

/**
 * @title PerpetualExchange
 * @dev Perpetual futures exchange with funding rate mechanism
 */
contract PerpetualExchange is ReentrancyGuard {
    IERC20 public collateralToken;

    struct Position {
        int256 size;           // Positive = long, negative = short
        uint256 collateral;
        uint256 entryPrice;    // Scaled by 1e18
        int256 fundingAccrued;
        uint256 lastFundingTimestamp;
    }

    struct Market {
        uint256 markPrice;
        uint256 indexPrice;
        int256 fundingRate;    // Per 8 hours, scaled by 1e18
        uint256 openInterestLong;
        uint256 openInterestShort;
        uint256 maxLeverage;
        uint256 liquidationThreshold; // Margin ratio threshold
    }

    mapping(address => Position) public positions;
    Market public market;

    // Funding rate constants
    uint256 public constant FUNDING_INTERVAL = 8 hours;
    int256 public constant MAX_FUNDING_RATE = 0.01e18; // 1% per 8 hours

    // Fee structure
    uint256 public makerFee = 2; // 0.02%
    uint256 public takerFee = 5; // 0.05%

    event PositionOpened(address indexed trader, int256 size, uint256 price);
    event PositionClosed(address indexed trader, int256 pnl);
    event Liquidation(address indexed trader, address indexed liquidator, int256 size);
    event FundingPaid(address indexed trader, int256 amount);

    constructor(address _collateral, uint256 _maxLeverage) {
        collateralToken = IERC20(_collateral);
        market.maxLeverage = _maxLeverage;
        market.liquidationThreshold = 50; // 5% margin ratio
    }

    /**
     * @dev Open or increase position
     */
    function openPosition(
        int256 sizeDelta,
        uint256 collateralDelta
    ) external nonReentrant {
        require(sizeDelta != 0, "Zero size");

        Position storage position = positions[msg.sender];

        // Apply pending funding
        _applyFunding(msg.sender);

        // Transfer collateral
        if (collateralDelta > 0) {
            collateralToken.transferFrom(msg.sender, address(this), collateralDelta);
            position.collateral += collateralDelta;
        }

        // Calculate fees
        uint256 notional = abs(sizeDelta) * market.markPrice / 1e18;
        uint256 fee = (notional * takerFee) / 10000;
        position.collateral -= fee;

        // Update position
        if (position.size == 0) {
            position.size = sizeDelta;
            position.entryPrice = market.markPrice;
        } else if (isSameDirection(position.size, sizeDelta)) {
            // Increase position - average entry price
            uint256 totalNotional = abs(position.size) * position.entryPrice +
                abs(sizeDelta) * market.markPrice;
            position.size += sizeDelta;
            position.entryPrice = totalNotional / abs(position.size);
        } else {
            // Reduce or flip position
            _reducePosition(msg.sender, sizeDelta);
        }

        // Update open interest
        if (sizeDelta > 0) {
            market.openInterestLong += uint256(sizeDelta);
        } else {
            market.openInterestShort += uint256(-sizeDelta);
        }

        // Check leverage
        require(
            _getEffectiveLeverage(msg.sender) <= market.maxLeverage * 1e18,
            "Exceeds max leverage"
        );

        emit PositionOpened(msg.sender, sizeDelta, market.markPrice);
    }

    /**
     * @dev Close position
     */
    function closePosition() external nonReentrant {
        Position storage position = positions[msg.sender];
        require(position.size != 0, "No position");

        // Apply pending funding
        _applyFunding(msg.sender);

        // Calculate PnL
        int256 pnl = _calculatePnL(msg.sender);

        // Calculate fees
        uint256 notional = abs(position.size) * market.markPrice / 1e18;
        uint256 fee = (notional * takerFee) / 10000;

        // Update collateral
        int256 finalCollateral = int256(position.collateral) + pnl - int256(fee);
        require(finalCollateral > 0, "Negative collateral");

        // Update open interest
        if (position.size > 0) {
            market.openInterestLong -= uint256(position.size);
        } else {
            market.openInterestShort -= uint256(-position.size);
        }

        // Clear position
        delete positions[msg.sender];

        // Transfer remaining collateral
        collateralToken.transfer(msg.sender, uint256(finalCollateral));

        emit PositionClosed(msg.sender, pnl);
    }

    /**
     * @dev Liquidate undercollateralized position
     */
    function liquidate(address trader) external nonReentrant {
        Position storage position = positions[trader];
        require(position.size != 0, "No position");

        // Apply pending funding
        _applyFunding(trader);

        // Check if liquidatable
        uint256 marginRatio = _getMarginRatio(trader);
        require(marginRatio < market.liquidationThreshold * 1e16, "Not liquidatable");

        // Calculate liquidation reward (5% of remaining collateral)
        uint256 reward = position.collateral * 5 / 100;

        // Update open interest
        if (position.size > 0) {
            market.openInterestLong -= uint256(position.size);
        } else {
            market.openInterestShort -= uint256(-position.size);
        }

        int256 size = position.size;

        // Clear position
        delete positions[trader];

        // Pay liquidator
        collateralToken.transfer(msg.sender, reward);

        emit Liquidation(trader, msg.sender, size);
    }

    /**
     * @dev Apply funding payment
     */
    function _applyFunding(address trader) internal {
        Position storage position = positions[trader];
        if (position.size == 0) return;

        uint256 elapsed = block.timestamp - position.lastFundingTimestamp;
        if (elapsed == 0) return;

        // Calculate funding payment
        int256 fundingPayment = (position.size * market.fundingRate * int256(elapsed)) /
            int256(FUNDING_INTERVAL) / 1e18;

        position.fundingAccrued += fundingPayment;
        position.collateral = uint256(int256(position.collateral) - fundingPayment);
        position.lastFundingTimestamp = block.timestamp;

        emit FundingPaid(trader, fundingPayment);
    }

    /**
     * @dev Update funding rate based on mark/index price difference
     */
    function updateFundingRate() external {
        // Funding rate = (mark price - index price) / index price
        int256 priceDiff = int256(market.markPrice) - int256(market.indexPrice);
        market.fundingRate = (priceDiff * 1e18) / int256(market.indexPrice);

        // Clamp funding rate
        if (market.fundingRate > MAX_FUNDING_RATE) {
            market.fundingRate = MAX_FUNDING_RATE;
        } else if (market.fundingRate < -MAX_FUNDING_RATE) {
            market.fundingRate = -MAX_FUNDING_RATE;
        }
    }

    /**
     * @dev Calculate unrealized PnL
     */
    function _calculatePnL(address trader) internal view returns (int256) {
        Position memory position = positions[trader];
        if (position.size == 0) return 0;

        int256 priceDiff = int256(market.markPrice) - int256(position.entryPrice);
        return (position.size * priceDiff) / 1e18;
    }

    /**
     * @dev Get margin ratio
     */
    function _getMarginRatio(address trader) internal view returns (uint256) {
        Position memory position = positions[trader];
        if (position.size == 0) return type(uint256).max;

        int256 equity = int256(position.collateral) + _calculatePnL(trader);
        if (equity <= 0) return 0;

        uint256 notional = abs(position.size) * market.markPrice / 1e18;
        return (uint256(equity) * 1e18) / notional;
    }

    /**
     * @dev Get effective leverage
     */
    function _getEffectiveLeverage(address trader) internal view returns (uint256) {
        Position memory position = positions[trader];
        if (position.collateral == 0) return type(uint256).max;

        uint256 notional = abs(position.size) * market.markPrice / 1e18;
        return (notional * 1e18) / position.collateral;
    }

    function _reducePosition(address trader, int256 sizeDelta) internal {
        // Implementation for reducing/flipping position
    }

    function isSameDirection(int256 a, int256 b) internal pure returns (bool) {
        return (a > 0 && b > 0) || (a < 0 && b < 0);
    }

    function abs(int256 x) internal pure returns (uint256) {
        return x >= 0 ? uint256(x) : uint256(-x);
    }
}
```

---

## Oracle Integration

### Chainlink Price Feeds

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@chainlink/contracts/src/v0.8/interfaces/AggregatorV3Interface.sol";

/**
 * @title PriceOracle
 * @dev Oracle aggregator for DeFi protocols
 */
contract PriceOracle {
    struct PriceFeed {
        AggregatorV3Interface aggregator;
        uint256 heartbeat;      // Maximum time between updates
        uint8 decimals;
        bool isActive;
    }

    mapping(address => PriceFeed) public priceFeeds;
    mapping(address => address) public fallbackOracles;

    address public owner;

    event PriceFeedUpdated(address indexed asset, address indexed aggregator);
    event PriceQueried(address indexed asset, uint256 price);

    modifier onlyOwner() {
        require(msg.sender == owner, "Not owner");
        _;
    }

    constructor() {
        owner = msg.sender;
    }

    /**
     * @dev Set price feed for asset
     */
    function setPriceFeed(
        address asset,
        address aggregator,
        uint256 heartbeat
    ) external onlyOwner {
        AggregatorV3Interface feed = AggregatorV3Interface(aggregator);

        priceFeeds[asset] = PriceFeed({
            aggregator: feed,
            heartbeat: heartbeat,
            decimals: feed.decimals(),
            isActive: true
        });

        emit PriceFeedUpdated(asset, aggregator);
    }

    /**
     * @dev Get latest price with validation
     */
    function getPrice(address asset) external view returns (uint256) {
        PriceFeed memory feed = priceFeeds[asset];
        require(feed.isActive, "Price feed not active");

        (
            uint80 roundId,
            int256 price,
            ,
            uint256 updatedAt,
            uint80 answeredInRound
        ) = feed.aggregator.latestRoundData();

        // Validate data
        require(price > 0, "Invalid price");
        require(updatedAt > 0, "Round not complete");
        require(answeredInRound >= roundId, "Stale data");
        require(
            block.timestamp - updatedAt <= feed.heartbeat,
            "Price too old"
        );

        // Normalize to 18 decimals
        if (feed.decimals < 18) {
            return uint256(price) * 10**(18 - feed.decimals);
        } else if (feed.decimals > 18) {
            return uint256(price) / 10**(feed.decimals - 18);
        }

        return uint256(price);
    }

    /**
     * @dev Get price with fallback
     */
    function getPriceWithFallback(address asset) external view returns (uint256) {
        try this.getPrice(asset) returns (uint256 price) {
            return price;
        } catch {
            address fallback_ = fallbackOracles[asset];
            require(fallback_ != address(0), "No fallback");
            return PriceOracle(fallback_).getPrice(asset);
        }
    }
}
```

---

## Key Takeaways

1. **Constant product AMMs** use `x * y = k` formula for price discovery and liquidity provision
2. **Concentrated liquidity** increases capital efficiency up to 4000x by allowing custom price ranges
3. **Lending protocols** use utilization-based interest rate models with jump rates above optimal utilization
4. **Flash loans** enable atomic arbitrage and liquidations without upfront capital
5. **Perpetual futures** use funding rates to keep mark price aligned with index price
6. **Oracle integration** requires careful validation including staleness checks and fallback mechanisms

## Review Questions

1. How does the constant product formula determine swap prices and slippage?
2. What are the advantages and risks of concentrated liquidity positions?
3. How do interest rate models incentivize borrowers and lenders at different utilization levels?
4. What safeguards prevent flash loan abuse?
5. How do funding rates maintain parity between perpetual futures and spot prices?
6. What oracle manipulation attacks exist and how can protocols defend against them?

---

**Next Chapter Preview:** Chapter 5 explores smart contract security in depth, covering audit methodologies, common vulnerabilities, and formal verification techniques.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity · Democratize Finance

