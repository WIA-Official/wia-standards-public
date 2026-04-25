# 제4장: DeFi 프로토콜

## 자동화 마켓 메이커, 대출 프로토콜 및 파생상품 플랫폼

### 분산 금융의 구성 요소

---

## 개요

분산 금융(DeFi) 프로토콜은 프로그래머블 금융 서비스의 기반을 형성합니다. 이 장에서는 자동화 마켓 메이커, 대출 프로토콜 및 파생상품 플랫폼을 포함한 핵심 DeFi 프리미티브의 기술 아키텍처, 수학적 모델 및 구현 패턴을 탐구합니다.

---

## 자동화 마켓 메이커(AMM)

### 상수 곱 마켓 메이커

**Uniswap V2 모델:**

상수 곱 공식 `x * y = k`는 모든 가격대에서 유동성을 보장합니다:

```
x = 토큰 X의 준비금
y = 토큰 Y의 준비금
k = 상수 곱 (불변량)
```

**가격 발견:**

```
X 기준 Y 가격 = x / y
Y 기준 X 가격 = y / x
```

**스왑 계산:**

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

/**
 * @title 상수곱AMM
 * @dev 기본 상수 곱 AMM 구현
 */
contract 상수곱AMM {
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
     * @dev 풀에 유동성 추가
     */
    function addLiquidity(
        uint256 amountX,
        uint256 amountY
    ) external returns (uint256 liquidityMinted) {
        tokenX.transferFrom(msg.sender, address(this), amountX);
        tokenY.transferFrom(msg.sender, address(this), amountY);

        if (totalLiquidity == 0) {
            // 초기 유동성
            liquidityMinted = sqrt(amountX * amountY);
        } else {
            // 비례 유동성
            liquidityMinted = min(
                (amountX * totalLiquidity) / reserveX,
                (amountY * totalLiquidity) / reserveY
            );
        }

        require(liquidityMinted > 0, "발행된 유동성이 부족함");

        liquidity[msg.sender] += liquidityMinted;
        totalLiquidity += liquidityMinted;
        reserveX += amountX;
        reserveY += amountY;

        emit AddLiquidity(msg.sender, amountX, amountY, liquidityMinted);
    }

    /**
     * @dev 풀에서 유동성 제거
     */
    function removeLiquidity(
        uint256 liquidityAmount
    ) external returns (uint256 amountX, uint256 amountY) {
        require(liquidity[msg.sender] >= liquidityAmount, "유동성이 부족함");

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
     * @dev tokenX를 tokenY로 스왑
     */
    function swapXForY(uint256 amountXIn) external returns (uint256 amountYOut) {
        require(amountXIn > 0, "입력이 0");

        // 수수료 적용
        uint256 amountXInWithFee = amountXIn * FEE_NUMERATOR;

        // 출력 계산: dy = y * dx / (x + dx)
        amountYOut = (reserveY * amountXInWithFee) /
            (reserveX * FEE_DENOMINATOR + amountXInWithFee);

        require(amountYOut > 0, "출력이 부족함");

        // 토큰 전송
        tokenX.transferFrom(msg.sender, address(this), amountXIn);
        tokenY.transfer(msg.sender, amountYOut);

        // 준비금 업데이트
        reserveX += amountXIn;
        reserveY -= amountYOut;

        emit Swap(msg.sender, amountXIn, 0, 0, amountYOut);
    }

    /**
     * @dev 주어진 입력에 대한 출력량 계산
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

### 집중 유동성 (Uniswap V3)

**가격 범위 유동성:**

| 기능 | Uniswap V2 | Uniswap V3 |
|------|------------|------------|
| 유동성 분포 | 균일 (0, ∞) | 집중 범위 |
| 자본 효율성 | ~0.5% | 최대 4000배 |
| LP 포지션 | 대체 가능 | NFT (대체 불가) |
| 수수료 티어 | 0.3% | 0.01%, 0.05%, 0.3%, 1% |
| 비영구적 손실 | 중간 | 범위 내 높음 |

---

## 대출 프로토콜

### Compound 스타일 머니 마켓

**이자율 모델:**

| 이용률 | 대출 금리 | 예금 금리 |
|--------|----------|----------|
| 0% | 2% | 0% |
| 50% | 10% | 4.5% |
| 80% (최적) | 20% | 14.4% |
| 90% | 80% | 64.8% |
| 100% | 200% | 180% |

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/token/ERC20/IERC20.sol";
import "@openzeppelin/contracts/security/ReentrancyGuard.sol";

/**
 * @title 대출풀
 * @dev 변동 이자율이 있는 Compound 스타일 대출 풀
 */
contract 대출풀 is ReentrancyGuard {
    IERC20 public underlyingAsset;

    // 이자율 모델 매개변수 (초당, 1e18로 스케일)
    uint256 public baseRatePerSecond;
    uint256 public multiplierPerSecond;
    uint256 public jumpMultiplierPerSecond;
    uint256 public optimalUtilization; // 1e18로 스케일

    // 환율 매개변수
    uint256 public constant INITIAL_EXCHANGE_RATE = 2e16; // 0.02
    uint256 public totalBorrows;
    uint256 public totalReserves;
    uint256 public reserveFactor; // 1e18로 스케일

    // 대출 상태
    uint256 public borrowIndex;
    uint256 public lastAccrualTimestamp;

    // 사용자 상태
    mapping(address => uint256) public supplyBalance; // cToken 잔액
    mapping(address => uint256) public borrowBalance;
    mapping(address => uint256) public borrowIndex_;

    // cToken 총 공급량
    uint256 public totalSupply;

    event Supply(address indexed user, uint256 amount, uint256 cTokens);
    event Withdraw(address indexed user, uint256 amount, uint256 cTokens);
    event Borrow(address indexed user, uint256 amount);
    event Repay(address indexed payer, address indexed borrower, uint256 amount);

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
     * @dev 이자 발생 및 상태 업데이트
     */
    function accrueInterest() public {
        uint256 currentTimestamp = block.timestamp;
        uint256 timeDelta = currentTimestamp - lastAccrualTimestamp;

        if (timeDelta == 0) return;

        uint256 cashPrior = getCash();
        uint256 borrowsPrior = totalBorrows;
        uint256 reservesPrior = totalReserves;
        uint256 borrowIndexPrior = borrowIndex;

        // 대출 금리 계산
        uint256 borrowRate = getBorrowRateInternal(cashPrior, borrowsPrior, reservesPrior);

        // 누적 이자 계산
        uint256 interestFactor = borrowRate * timeDelta;
        uint256 interestAccumulated = (borrowsPrior * interestFactor) / 1e18;

        // 상태 업데이트
        totalBorrows = borrowsPrior + interestAccumulated;
        totalReserves = reservesPrior + (interestAccumulated * reserveFactor) / 1e18;
        borrowIndex = borrowIndexPrior + (borrowIndexPrior * interestFactor) / 1e18;
        lastAccrualTimestamp = currentTimestamp;
    }

    /**
     * @dev 풀에 자산 예치
     */
    function supply(uint256 amount) external nonReentrant returns (uint256) {
        accrueInterest();

        // 기초 자산 전송
        underlyingAsset.transferFrom(msg.sender, address(this), amount);

        // 발행할 cToken 계산
        uint256 exchangeRate = exchangeRateCurrent();
        uint256 cTokens = (amount * 1e18) / exchangeRate;

        // 상태 업데이트
        supplyBalance[msg.sender] += cTokens;
        totalSupply += cTokens;

        emit Supply(msg.sender, amount, cTokens);
        return cTokens;
    }

    /**
     * @dev 풀에서 자산 인출
     */
    function withdraw(uint256 cTokenAmount) external nonReentrant returns (uint256) {
        accrueInterest();

        require(supplyBalance[msg.sender] >= cTokenAmount, "잔액이 부족함");

        // 전송할 기초 자산 계산
        uint256 exchangeRate = exchangeRateCurrent();
        uint256 underlyingAmount = (cTokenAmount * exchangeRate) / 1e18;

        require(getCash() >= underlyingAmount, "유동성이 부족함");

        // 상태 업데이트
        supplyBalance[msg.sender] -= cTokenAmount;
        totalSupply -= cTokenAmount;

        // 기초 자산 전송
        underlyingAsset.transfer(msg.sender, underlyingAmount);

        emit Withdraw(msg.sender, underlyingAmount, cTokenAmount);
        return underlyingAmount;
    }

    /**
     * @dev 풀에서 자산 대출
     */
    function borrow(uint256 amount) external nonReentrant {
        accrueInterest();

        require(getCash() >= amount, "유동성이 부족함");

        // 대출 상태 업데이트
        uint256 accountBorrowsPrev = borrowBalanceStored(msg.sender);
        uint256 accountBorrowsNew = accountBorrowsPrev + amount;

        borrowBalance[msg.sender] = accountBorrowsNew;
        borrowIndex_[msg.sender] = borrowIndex;
        totalBorrows += amount;

        // 기초 자산 전송
        underlyingAsset.transfer(msg.sender, amount);

        emit Borrow(msg.sender, amount);
    }

    /**
     * @dev 대출 자산 상환
     */
    function repay(uint256 amount) external nonReentrant returns (uint256) {
        accrueInterest();

        uint256 accountBorrows = borrowBalanceStored(msg.sender);
        uint256 repayAmount = amount > accountBorrows ? accountBorrows : amount;

        // 기초 자산 전송
        underlyingAsset.transferFrom(msg.sender, address(this), repayAmount);

        // 상태 업데이트
        borrowBalance[msg.sender] = accountBorrows - repayAmount;
        borrowIndex_[msg.sender] = borrowIndex;
        totalBorrows -= repayAmount;

        emit Repay(msg.sender, msg.sender, repayAmount);
        return repayAmount;
    }

    /**
     * @dev 누적 이자가 포함된 저장된 대출 잔액 조회
     */
    function borrowBalanceStored(address account) public view returns (uint256) {
        if (borrowBalance[account] == 0) return 0;

        uint256 principalTimesIndex = borrowBalance[account] * borrowIndex;
        return principalTimesIndex / borrowIndex_[account];
    }

    /**
     * @dev 현재 환율 계산
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
     * @dev 현금 (기초 자산 잔액) 조회
     */
    function getCash() public view returns (uint256) {
        return underlyingAsset.balanceOf(address(this));
    }

    /**
     * @dev 이용률 계산
     */
    function getUtilizationRate() public view returns (uint256) {
        uint256 cash = getCash();
        if (cash + totalBorrows == 0) return 0;

        return (totalBorrows * 1e18) / (cash + totalBorrows - totalReserves);
    }

    /**
     * @dev 이용률 기반 대출 금리 계산
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
}
```

### 플래시 론

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
 * @title 플래시론풀
 * @dev 수수료 메커니즘이 있는 플래시 론 제공자
 */
contract 플래시론풀 is ReentrancyGuard {
    mapping(address => bool) public supportedAssets;
    uint256 public constant FLASH_LOAN_FEE = 9; // 0.09%

    event FlashLoan(
        address indexed receiver,
        address indexed asset,
        uint256 amount,
        uint256 premium
    );

    /**
     * @dev 플래시 론 실행
     */
    function flashLoan(
        address receiver,
        address asset,
        uint256 amount,
        bytes calldata params
    ) external nonReentrant {
        require(supportedAssets[asset], "지원되지 않는 자산");

        IERC20 token = IERC20(asset);
        uint256 balanceBefore = token.balanceOf(address(this));
        require(balanceBefore >= amount, "유동성이 부족함");

        // 프리미엄 계산
        uint256 premium = (amount * FLASH_LOAN_FEE) / 10000;

        // 수신자에게 전송
        token.transfer(receiver, amount);

        // 콜백 실행
        require(
            IFlashLoanReceiver(receiver).executeOperation(
                asset,
                amount,
                premium,
                msg.sender,
                params
            ),
            "플래시 론 실행 실패"
        );

        // 상환 확인
        uint256 balanceAfter = token.balanceOf(address(this));
        require(
            balanceAfter >= balanceBefore + premium,
            "플래시 론이 상환되지 않음"
        );

        emit FlashLoan(receiver, asset, amount, premium);
    }
}
```

---

## 파생상품 및 합성 자산

### 무기한 선물

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

/**
 * @title 무기한거래소
 * @dev 펀딩 레이트 메커니즘이 있는 무기한 선물 거래소
 */
contract 무기한거래소 is ReentrancyGuard {
    IERC20 public collateralToken;

    struct Position {
        int256 size;           // 양수 = 롱, 음수 = 숏
        uint256 collateral;
        uint256 entryPrice;    // 1e18로 스케일
        int256 fundingAccrued;
        uint256 lastFundingTimestamp;
    }

    struct Market {
        uint256 markPrice;
        uint256 indexPrice;
        int256 fundingRate;    // 8시간당, 1e18로 스케일
        uint256 openInterestLong;
        uint256 openInterestShort;
        uint256 maxLeverage;
        uint256 liquidationThreshold; // 마진 비율 임계값
    }

    mapping(address => Position) public positions;
    Market public market;

    // 펀딩 레이트 상수
    uint256 public constant FUNDING_INTERVAL = 8 hours;
    int256 public constant MAX_FUNDING_RATE = 0.01e18; // 8시간당 1%

    // 수수료 구조
    uint256 public makerFee = 2; // 0.02%
    uint256 public takerFee = 5; // 0.05%

    event PositionOpened(address indexed trader, int256 size, uint256 price);
    event PositionClosed(address indexed trader, int256 pnl);
    event Liquidation(address indexed trader, address indexed liquidator, int256 size);
    event FundingPaid(address indexed trader, int256 amount);

    constructor(address _collateral, uint256 _maxLeverage) {
        collateralToken = IERC20(_collateral);
        market.maxLeverage = _maxLeverage;
        market.liquidationThreshold = 50; // 5% 마진 비율
    }

    /**
     * @dev 포지션 오픈 또는 증가
     */
    function openPosition(
        int256 sizeDelta,
        uint256 collateralDelta
    ) external nonReentrant {
        require(sizeDelta != 0, "크기가 0");

        Position storage position = positions[msg.sender];

        // 대기 중인 펀딩 적용
        _applyFunding(msg.sender);

        // 담보 전송
        if (collateralDelta > 0) {
            collateralToken.transferFrom(msg.sender, address(this), collateralDelta);
            position.collateral += collateralDelta;
        }

        // 수수료 계산
        uint256 notional = abs(sizeDelta) * market.markPrice / 1e18;
        uint256 fee = (notional * takerFee) / 10000;
        position.collateral -= fee;

        // 포지션 업데이트
        if (position.size == 0) {
            position.size = sizeDelta;
            position.entryPrice = market.markPrice;
        } else if (isSameDirection(position.size, sizeDelta)) {
            // 포지션 증가 - 평균 진입 가격
            uint256 totalNotional = abs(position.size) * position.entryPrice +
                abs(sizeDelta) * market.markPrice;
            position.size += sizeDelta;
            position.entryPrice = totalNotional / abs(position.size);
        }

        // 미결제 약정 업데이트
        if (sizeDelta > 0) {
            market.openInterestLong += uint256(sizeDelta);
        } else {
            market.openInterestShort += uint256(-sizeDelta);
        }

        // 레버리지 확인
        require(
            _getEffectiveLeverage(msg.sender) <= market.maxLeverage * 1e18,
            "최대 레버리지 초과"
        );

        emit PositionOpened(msg.sender, sizeDelta, market.markPrice);
    }

    /**
     * @dev 펀딩 결제 적용
     */
    function _applyFunding(address trader) internal {
        Position storage position = positions[trader];
        if (position.size == 0) return;

        uint256 elapsed = block.timestamp - position.lastFundingTimestamp;
        if (elapsed == 0) return;

        // 펀딩 결제 계산
        int256 fundingPayment = (position.size * market.fundingRate * int256(elapsed)) /
            int256(FUNDING_INTERVAL) / 1e18;

        position.fundingAccrued += fundingPayment;
        position.collateral = uint256(int256(position.collateral) - fundingPayment);
        position.lastFundingTimestamp = block.timestamp;

        emit FundingPaid(trader, fundingPayment);
    }

    /**
     * @dev 마크/인덱스 가격 차이 기반 펀딩 레이트 업데이트
     */
    function updateFundingRate() external {
        // 펀딩 레이트 = (마크 가격 - 인덱스 가격) / 인덱스 가격
        int256 priceDiff = int256(market.markPrice) - int256(market.indexPrice);
        market.fundingRate = (priceDiff * 1e18) / int256(market.indexPrice);

        // 펀딩 레이트 제한
        if (market.fundingRate > MAX_FUNDING_RATE) {
            market.fundingRate = MAX_FUNDING_RATE;
        } else if (market.fundingRate < -MAX_FUNDING_RATE) {
            market.fundingRate = -MAX_FUNDING_RATE;
        }
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

## 오라클 통합

### Chainlink 가격 피드

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@chainlink/contracts/src/v0.8/interfaces/AggregatorV3Interface.sol";

/**
 * @title 가격오라클
 * @dev DeFi 프로토콜을 위한 오라클 집계기
 */
contract 가격오라클 {
    struct PriceFeed {
        AggregatorV3Interface aggregator;
        uint256 heartbeat;      // 업데이트 간 최대 시간
        uint8 decimals;
        bool isActive;
    }

    mapping(address => PriceFeed) public priceFeeds;
    mapping(address => address) public fallbackOracles;

    address public owner;

    event PriceFeedUpdated(address indexed asset, address indexed aggregator);
    event PriceQueried(address indexed asset, uint256 price);

    modifier onlyOwner() {
        require(msg.sender == owner, "소유자가 아님");
        _;
    }

    constructor() {
        owner = msg.sender;
    }

    /**
     * @dev 자산에 대한 가격 피드 설정
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
     * @dev 검증과 함께 최신 가격 조회
     */
    function getPrice(address asset) external view returns (uint256) {
        PriceFeed memory feed = priceFeeds[asset];
        require(feed.isActive, "가격 피드가 활성화되지 않음");

        (
            uint80 roundId,
            int256 price,
            ,
            uint256 updatedAt,
            uint80 answeredInRound
        ) = feed.aggregator.latestRoundData();

        // 데이터 검증
        require(price > 0, "유효하지 않은 가격");
        require(updatedAt > 0, "라운드가 완료되지 않음");
        require(answeredInRound >= roundId, "오래된 데이터");
        require(
            block.timestamp - updatedAt <= feed.heartbeat,
            "가격이 너무 오래됨"
        );

        // 18 소수점으로 정규화
        if (feed.decimals < 18) {
            return uint256(price) * 10**(18 - feed.decimals);
        } else if (feed.decimals > 18) {
            return uint256(price) / 10**(feed.decimals - 18);
        }

        return uint256(price);
    }
}
```

---

## 핵심 내용

1. **상수 곱 AMM**은 가격 발견 및 유동성 공급에 `x * y = k` 공식 사용
2. **집중 유동성**은 커스텀 가격 범위를 허용하여 자본 효율성을 최대 4000배 증가
3. **대출 프로토콜**은 최적 이용률 이상에서 점프 금리가 있는 이용률 기반 이자율 모델 사용
4. **플래시 론**은 선행 자본 없이 원자적 차익거래 및 청산 가능
5. **무기한 선물**은 펀딩 레이트를 사용하여 마크 가격을 인덱스 가격에 맞춤
6. **오라클 통합**은 오래된 데이터 확인 및 폴백 메커니즘을 포함한 신중한 검증 필요

## 복습 문제

1. 상수 곱 공식이 스왑 가격과 슬리피지를 어떻게 결정합니까?
2. 집중 유동성 포지션의 장점과 위험은 무엇입니까?
3. 이자율 모델은 다양한 이용률 수준에서 대출자와 차입자를 어떻게 인센티브화합니까?
4. 플래시 론 남용을 방지하는 안전장치는 무엇입니까?
5. 펀딩 레이트는 무기한 선물과 현물 가격 간의 동등성을 어떻게 유지합니까?
6. 어떤 오라클 조작 공격이 존재하고 프로토콜은 어떻게 방어할 수 있습니까?

---

**다음 장 미리보기:** 제5장에서는 감사 방법론, 일반적인 취약점 및 형식 검증 기술을 포함한 스마트 컨트랙트 보안을 심층적으로 탐구합니다.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · 널리 인간을 이롭게 · 금융을 민주화

