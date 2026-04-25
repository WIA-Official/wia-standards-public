# 제8장: 실전 구현 가이드

## 8.1 개발 환경 설정

### 8.1.1 Foundry 개발 환경

WIA 블록체인 금융 프로젝트는 Foundry를 기본 개발 프레임워크로 채택합니다. Foundry는 Rust로 작성된 고성능 스마트 컨트랙트 개발 도구로, 빠른 컴파일, 강력한 테스트, 네이티브 퍼징 기능을 제공합니다.

**환경 설치:**

```bash
#!/bin/bash
# WIA Blockchain Finance 개발 환경 설정 스크립트

# 1. Foundry 설치
curl -L https://foundry.paradigm.xyz | bash
foundryup

# 2. Foundry 버전 확인
forge --version
cast --version
anvil --version
chisel --version

# 3. 프로젝트 생성
forge init wia-blockchain-finance
cd wia-blockchain-finance

# 4. 의존성 설치
forge install OpenZeppelin/openzeppelin-contracts --no-commit
forge install OpenZeppelin/openzeppelin-contracts-upgradeable --no-commit
forge install foundry-rs/forge-std --no-commit
forge install transmissions11/solmate --no-commit
forge install Uniswap/v4-core --no-commit

# 5. Chainlink 의존성 (CCIP, 오라클)
forge install smartcontractkit/chainlink --no-commit

# 6. LayerZero 의존성
forge install LayerZero-Labs/LayerZero-v2 --no-commit

# 7. remappings 설정
cat > remappings.txt << 'EOF'
@openzeppelin/contracts/=lib/openzeppelin-contracts/contracts/
@openzeppelin/contracts-upgradeable/=lib/openzeppelin-contracts-upgradeable/contracts/
forge-std/=lib/forge-std/src/
solmate/=lib/solmate/src/
@uniswap/v4-core/=lib/v4-core/
@chainlink/=lib/chainlink/
@layerzerolabs/=lib/LayerZero-v2/
EOF

# 8. foundry.toml 설정
cat > foundry.toml << 'EOF'
[profile.default]
src = "src"
out = "out"
libs = ["lib"]
solc = "0.8.23"
optimizer = true
optimizer_runs = 200
via_ir = true
ffi = false
fuzz = { runs = 256 }
invariant = { runs = 256, depth = 15 }

[profile.ci]
fuzz = { runs = 10000 }
invariant = { runs = 1000, depth = 50 }

[profile.production]
optimizer_runs = 1000000

[rpc_endpoints]
mainnet = "${MAINNET_RPC_URL}"
sepolia = "${SEPOLIA_RPC_URL}"
polygon = "${POLYGON_RPC_URL}"
arbitrum = "${ARBITRUM_RPC_URL}"
optimism = "${OPTIMISM_RPC_URL}"

[etherscan]
mainnet = { key = "${ETHERSCAN_API_KEY}" }
sepolia = { key = "${ETHERSCAN_API_KEY}" }
polygon = { key = "${POLYGONSCAN_API_KEY}" }

[fmt]
line_length = 120
tab_width = 4
bracket_spacing = true
EOF
```

### 8.1.2 프로젝트 구조

**권장 디렉토리 구조:**

```
wia-blockchain-finance/
├── src/
│   ├── core/
│   │   ├── WIAToken.sol              # 토큰 컨트랙트
│   │   ├── WIAVault.sol              # 금고/보관소
│   │   └── WIAGovernance.sol         # 거버넌스
│   │
│   ├── defi/
│   │   ├── pools/
│   │   │   ├── WIALiquidityPool.sol  # AMM 풀
│   │   │   └── WIAStablePool.sol     # 스테이블 풀
│   │   ├── lending/
│   │   │   ├── WIALendingPool.sol    # 대출 풀
│   │   │   └── WIAInterestModel.sol  # 이자율 모델
│   │   └── derivatives/
│   │       ├── WIAPerpetual.sol      # 무기한 선물
│   │       └── WIAOptions.sol        # 옵션
│   │
│   ├── security/
│   │   ├── WIASecurityToken.sol      # 증권 토큰
│   │   ├── WIACompliance.sol         # 컴플라이언스
│   │   └── WIAIdentityRegistry.sol   # ID 레지스트리
│   │
│   ├── crosschain/
│   │   ├── WIABridge.sol             # 브릿지
│   │   ├── WIALayerZeroAdapter.sol   # LZ 어댑터
│   │   └── WIACCIPAdapter.sol        # CCIP 어댑터
│   │
│   ├── oracles/
│   │   ├── WIAPriceOracle.sol        # 가격 오라클
│   │   └── WIATWAPOracle.sol         # TWAP 오라클
│   │
│   ├── libraries/
│   │   ├── WIAMath.sol               # 수학 라이브러리
│   │   ├── WIASafeCast.sol           # 안전한 캐스팅
│   │   └── WIAErrors.sol             # 에러 정의
│   │
│   └── interfaces/
│       ├── IWIAToken.sol
│       ├── IWIAVault.sol
│       └── ...
│
├── test/
│   ├── unit/                         # 단위 테스트
│   ├── integration/                  # 통합 테스트
│   ├── fuzz/                         # 퍼즈 테스트
│   ├── invariant/                    # 불변성 테스트
│   └── fork/                         # 포크 테스트
│
├── script/
│   ├── Deploy.s.sol                  # 배포 스크립트
│   ├── Upgrade.s.sol                 # 업그레이드
│   └── Verify.s.sol                  # 검증
│
├── docs/
│   ├── architecture.md
│   ├── security.md
│   └── deployment.md
│
└── audit/                            # 감사 보고서
```

### 8.1.3 개발 워크플로우

**Git 브랜치 전략:**

```typescript
interface 개발워크플로우 {
  브랜치전략: {
    main: "프로덕션 배포 코드";
    develop: "개발 통합 브랜치";
    feature: "기능 개발 (feature/xxx)";
    release: "릴리스 준비 (release/v1.0.0)";
    hotfix: "긴급 수정 (hotfix/xxx)";
  };

  코드리뷰요건: {
    최소승인: 2;
    필수검토자: ["보안팀", "아키텍트"];
    자동검사: ["린터", "테스트", "가스리포트", "커버리지"];
  };

  배포파이프라인: {
    테스트넷: "PR 머지 시 자동 배포";
    스테이징: "release 브랜치 자동 배포";
    메인넷: "수동 승인 후 배포";
  };
}
```

## 8.2 핵심 컨트랙트 구현

### 8.2.1 WIA 토큰 구현

**완전한 ERC-20 토큰 구현:**

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.23;

import {ERC20} from "@openzeppelin/contracts/token/ERC20/ERC20.sol";
import {ERC20Permit} from "@openzeppelin/contracts/token/ERC20/extensions/ERC20Permit.sol";
import {ERC20Votes} from "@openzeppelin/contracts/token/ERC20/extensions/ERC20Votes.sol";
import {ERC20Burnable} from "@openzeppelin/contracts/token/ERC20/extensions/ERC20Burnable.sol";
import {AccessControlEnumerable} from "@openzeppelin/contracts/access/extensions/AccessControlEnumerable.sol";
import {Pausable} from "@openzeppelin/contracts/utils/Pausable.sol";
import {Nonces} from "@openzeppelin/contracts/utils/Nonces.sol";

/**
 * @title WIAToken
 * @author WIA Standards Committee
 * @notice WIA 블록체인 금융 생태계 거버넌스 토큰
 * @dev ERC20 + Permit + Votes + Burnable + AccessControl + Pausable
 */
contract WIAToken is
    ERC20,
    ERC20Permit,
    ERC20Votes,
    ERC20Burnable,
    AccessControlEnumerable,
    Pausable
{
    // ============ 역할 정의 ============
    bytes32 public constant MINTER_ROLE = keccak256("MINTER_ROLE");
    bytes32 public constant PAUSER_ROLE = keccak256("PAUSER_ROLE");
    bytes32 public constant SNAPSHOT_ROLE = keccak256("SNAPSHOT_ROLE");

    // ============ 상태 변수 ============
    uint256 public constant MAX_SUPPLY = 1_000_000_000 * 10**18; // 10억 토큰
    uint256 public immutable deploymentTimestamp;

    // 민트 제한
    uint256 public constant MINT_COOLDOWN = 1 days;
    uint256 public constant MAX_MINT_PER_PERIOD = 10_000_000 * 10**18; // 1%
    uint256 public lastMintTimestamp;
    uint256 public mintedThisPeriod;

    // 전송 제한 (런칭 초기)
    bool public transferRestricted;
    mapping(address => bool) public transferWhitelist;

    // ============ 이벤트 ============
    event TransferRestrictionUpdated(bool restricted);
    event WhitelistUpdated(address indexed account, bool status);
    event EmergencyWithdraw(address indexed token, address indexed to, uint256 amount);

    // ============ 에러 ============
    error MaxSupplyExceeded();
    error MintCooldownActive();
    error MintLimitExceeded();
    error TransferRestricted();
    error ZeroAddress();

    // ============ 생성자 ============
    constructor(
        address _admin,
        address _treasury
    )
        ERC20("WIA Token", "WIA")
        ERC20Permit("WIA Token")
    {
        if (_admin == address(0) || _treasury == address(0)) revert ZeroAddress();

        _grantRole(DEFAULT_ADMIN_ROLE, _admin);
        _grantRole(MINTER_ROLE, _admin);
        _grantRole(PAUSER_ROLE, _admin);

        deploymentTimestamp = block.timestamp;
        transferRestricted = true;
        transferWhitelist[_admin] = true;
        transferWhitelist[_treasury] = true;

        // 초기 발행 (전체 공급량의 20%)
        _mint(_treasury, 200_000_000 * 10**18);
    }

    // ============ 민트 기능 ============
    /**
     * @notice 새 토큰 발행
     * @param to 수령 주소
     * @param amount 발행량
     */
    function mint(address to, uint256 amount) external onlyRole(MINTER_ROLE) {
        if (to == address(0)) revert ZeroAddress();
        if (totalSupply() + amount > MAX_SUPPLY) revert MaxSupplyExceeded();

        // 민트 쿨다운 및 제한 확인
        if (block.timestamp >= lastMintTimestamp + MINT_COOLDOWN) {
            mintedThisPeriod = 0;
            lastMintTimestamp = block.timestamp;
        }

        if (mintedThisPeriod + amount > MAX_MINT_PER_PERIOD) {
            revert MintLimitExceeded();
        }

        mintedThisPeriod += amount;
        _mint(to, amount);
    }

    // ============ 전송 제한 ============
    /**
     * @notice 전송 제한 업데이트
     */
    function setTransferRestricted(bool _restricted) external onlyRole(DEFAULT_ADMIN_ROLE) {
        transferRestricted = _restricted;
        emit TransferRestrictionUpdated(_restricted);
    }

    /**
     * @notice 화이트리스트 업데이트
     */
    function setWhitelist(address _account, bool _status) external onlyRole(DEFAULT_ADMIN_ROLE) {
        transferWhitelist[_account] = _status;
        emit WhitelistUpdated(_account, _status);
    }

    // ============ 일시정지 ============
    function pause() external onlyRole(PAUSER_ROLE) {
        _pause();
    }

    function unpause() external onlyRole(PAUSER_ROLE) {
        _unpause();
    }

    // ============ 오버라이드 ============
    function _update(
        address from,
        address to,
        uint256 value
    ) internal override(ERC20, ERC20Votes) whenNotPaused {
        // 전송 제한 확인
        if (transferRestricted && from != address(0) && to != address(0)) {
            if (!transferWhitelist[from] && !transferWhitelist[to]) {
                revert TransferRestricted();
            }
        }

        super._update(from, to, value);
    }

    function nonces(address owner) public view override(ERC20Permit, Nonces) returns (uint256) {
        return super.nonces(owner);
    }

    // ============ 긴급 기능 ============
    /**
     * @notice 긴급 토큰 회수
     * @dev 잘못 전송된 토큰 회수용
     */
    function emergencyWithdraw(
        address token,
        address to,
        uint256 amount
    ) external onlyRole(DEFAULT_ADMIN_ROLE) {
        if (to == address(0)) revert ZeroAddress();
        require(token != address(this), "Cannot withdraw WIA");

        IERC20(token).transfer(to, amount);
        emit EmergencyWithdraw(token, to, amount);
    }

    // ============ 뷰 함수 ============
    function remainingMintableThisPeriod() external view returns (uint256) {
        if (block.timestamp >= lastMintTimestamp + MINT_COOLDOWN) {
            return MAX_MINT_PER_PERIOD;
        }
        return MAX_MINT_PER_PERIOD - mintedThisPeriod;
    }
}

interface IERC20 {
    function transfer(address to, uint256 amount) external returns (bool);
}
```

### 8.2.2 WIA 금고 (Vault) 구현

**ERC-4626 준수 금고:**

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.23;

import {ERC4626} from "@openzeppelin/contracts/token/ERC20/extensions/ERC4626.sol";
import {ERC20} from "@openzeppelin/contracts/token/ERC20/ERC20.sol";
import {IERC20} from "@openzeppelin/contracts/token/ERC20/IERC20.sol";
import {SafeERC20} from "@openzeppelin/contracts/token/ERC20/utils/SafeERC20.sol";
import {Math} from "@openzeppelin/contracts/utils/math/Math.sol";
import {ReentrancyGuard} from "@openzeppelin/contracts/utils/ReentrancyGuard.sol";
import {AccessControl} from "@openzeppelin/contracts/access/AccessControl.sol";
import {Pausable} from "@openzeppelin/contracts/utils/Pausable.sol";

/**
 * @title WIAVault
 * @notice ERC-4626 준수 수익 창출 금고
 * @dev 다중 전략 지원, 수수료 관리, 긴급 출금 기능
 */
contract WIAVault is ERC4626, ReentrancyGuard, AccessControl, Pausable {
    using SafeERC20 for IERC20;
    using Math for uint256;

    // ============ 역할 ============
    bytes32 public constant STRATEGIST_ROLE = keccak256("STRATEGIST_ROLE");
    bytes32 public constant GUARDIAN_ROLE = keccak256("GUARDIAN_ROLE");

    // ============ 상태 변수 ============

    // 수수료 (basis points, 10000 = 100%)
    uint256 public performanceFee = 1000; // 10%
    uint256 public managementFee = 200;   // 2%
    uint256 public constant MAX_FEE = 3000; // 30%

    // 수수료 수령자
    address public feeRecipient;

    // 마지막 수확 시간
    uint256 public lastHarvestTimestamp;

    // 전략
    struct Strategy {
        address strategyAddress;
        uint256 allocation; // basis points
        uint256 totalDebt;
        uint256 lastReport;
        bool active;
    }

    mapping(address => Strategy) public strategies;
    address[] public strategyList;
    uint256 public totalDebt;

    // 입출금 제한
    uint256 public depositLimit;
    uint256 public minDeposit;
    mapping(address => uint256) public userDepositCap;

    // 출금 대기열 (대규모 출금용)
    struct WithdrawalRequest {
        address user;
        uint256 shares;
        uint256 requestTime;
        bool processed;
    }

    WithdrawalRequest[] public withdrawalQueue;
    uint256 public withdrawalDelay = 1 days;

    // ============ 이벤트 ============
    event StrategyAdded(address indexed strategy, uint256 allocation);
    event StrategyRemoved(address indexed strategy);
    event Harvest(address indexed strategy, uint256 profit, uint256 loss);
    event FeesCollected(address indexed recipient, uint256 amount);
    event WithdrawalQueued(address indexed user, uint256 shares, uint256 requestId);
    event WithdrawalProcessed(address indexed user, uint256 shares, uint256 assets);

    // ============ 에러 ============
    error DepositLimitExceeded();
    error BelowMinDeposit();
    error UserCapExceeded();
    error InvalidStrategy();
    error AllocationExceeded();
    error WithdrawalNotReady();
    error InvalidFee();

    // ============ 생성자 ============
    constructor(
        IERC20 _asset,
        string memory _name,
        string memory _symbol,
        address _admin,
        address _feeRecipient
    ) ERC4626(_asset) ERC20(_name, _symbol) {
        _grantRole(DEFAULT_ADMIN_ROLE, _admin);
        _grantRole(STRATEGIST_ROLE, _admin);
        _grantRole(GUARDIAN_ROLE, _admin);

        feeRecipient = _feeRecipient;
        depositLimit = type(uint256).max;
        minDeposit = 0;
        lastHarvestTimestamp = block.timestamp;
    }

    // ============ 입금 로직 ============
    function _deposit(
        address caller,
        address receiver,
        uint256 assets,
        uint256 shares
    ) internal override nonReentrant whenNotPaused {
        if (assets < minDeposit) revert BelowMinDeposit();
        if (totalAssets() + assets > depositLimit) revert DepositLimitExceeded();

        if (userDepositCap[receiver] > 0) {
            uint256 currentDeposit = convertToAssets(balanceOf(receiver));
            if (currentDeposit + assets > userDepositCap[receiver]) {
                revert UserCapExceeded();
            }
        }

        super._deposit(caller, receiver, assets, shares);
    }

    // ============ 출금 로직 ============
    function _withdraw(
        address caller,
        address receiver,
        address owner,
        uint256 assets,
        uint256 shares
    ) internal override nonReentrant {
        // 대규모 출금은 대기열로
        if (assets > _availableLiquidity() / 10) {
            _queueWithdrawal(owner, shares);
            return;
        }

        // 충분한 유동성이 없으면 전략에서 회수
        if (assets > _availableLiquidity()) {
            _withdrawFromStrategies(assets - _availableLiquidity());
        }

        super._withdraw(caller, receiver, owner, assets, shares);
    }

    /**
     * @notice 출금 요청 대기열 추가
     */
    function _queueWithdrawal(address user, uint256 shares) internal {
        withdrawalQueue.push(WithdrawalRequest({
            user: user,
            shares: shares,
            requestTime: block.timestamp,
            processed: false
        }));

        emit WithdrawalQueued(user, shares, withdrawalQueue.length - 1);
    }

    /**
     * @notice 대기열 출금 처리
     */
    function processWithdrawal(uint256 requestId) external nonReentrant {
        WithdrawalRequest storage request = withdrawalQueue[requestId];

        if (request.processed) revert("Already processed");
        if (block.timestamp < request.requestTime + withdrawalDelay) {
            revert WithdrawalNotReady();
        }

        uint256 assets = convertToAssets(request.shares);

        // 전략에서 자금 회수
        if (assets > _availableLiquidity()) {
            _withdrawFromStrategies(assets - _availableLiquidity());
        }

        request.processed = true;
        _burn(request.user, request.shares);

        IERC20(asset()).safeTransfer(request.user, assets);

        emit WithdrawalProcessed(request.user, request.shares, assets);
    }

    // ============ 전략 관리 ============
    /**
     * @notice 전략 추가
     */
    function addStrategy(
        address _strategy,
        uint256 _allocation
    ) external onlyRole(STRATEGIST_ROLE) {
        if (_strategy == address(0)) revert InvalidStrategy();
        if (_totalAllocation() + _allocation > 10000) revert AllocationExceeded();

        strategies[_strategy] = Strategy({
            strategyAddress: _strategy,
            allocation: _allocation,
            totalDebt: 0,
            lastReport: block.timestamp,
            active: true
        });

        strategyList.push(_strategy);

        emit StrategyAdded(_strategy, _allocation);
    }

    /**
     * @notice 전략 자금 배분
     */
    function allocateToStrategy(address _strategy, uint256 _amount) external onlyRole(STRATEGIST_ROLE) {
        Strategy storage strategy = strategies[_strategy];
        if (!strategy.active) revert InvalidStrategy();

        IERC20(asset()).safeTransfer(_strategy, _amount);
        strategy.totalDebt += _amount;
        totalDebt += _amount;

        IStrategy(_strategy).deposit(_amount);
    }

    /**
     * @notice 전략 수확 (수익 수집)
     */
    function harvest(address _strategy) external onlyRole(STRATEGIST_ROLE) returns (uint256 profit, uint256 loss) {
        Strategy storage strategy = strategies[_strategy];
        if (!strategy.active) revert InvalidStrategy();

        uint256 balanceBefore = IERC20(asset()).balanceOf(address(this));

        (profit, loss) = IStrategy(_strategy).harvest();

        uint256 balanceAfter = IERC20(asset()).balanceOf(address(this));

        // 수익 발생 시 수수료 징수
        if (profit > 0) {
            uint256 fee = profit * performanceFee / 10000;
            IERC20(asset()).safeTransfer(feeRecipient, fee);
            emit FeesCollected(feeRecipient, fee);
        }

        // 손실 발생 시 부채 조정
        if (loss > 0) {
            strategy.totalDebt = strategy.totalDebt > loss ? strategy.totalDebt - loss : 0;
            totalDebt = totalDebt > loss ? totalDebt - loss : 0;
        }

        strategy.lastReport = block.timestamp;
        lastHarvestTimestamp = block.timestamp;

        emit Harvest(_strategy, profit, loss);
    }

    /**
     * @notice 전략에서 자금 회수
     */
    function _withdrawFromStrategies(uint256 _amount) internal {
        uint256 remaining = _amount;

        for (uint i = 0; i < strategyList.length && remaining > 0; i++) {
            Strategy storage strategy = strategies[strategyList[i]];

            if (!strategy.active || strategy.totalDebt == 0) continue;

            uint256 withdrawAmount = remaining.min(strategy.totalDebt);

            IStrategy(strategy.strategyAddress).withdraw(withdrawAmount);

            strategy.totalDebt -= withdrawAmount;
            totalDebt -= withdrawAmount;
            remaining -= withdrawAmount;
        }
    }

    // ============ 총 자산 계산 ============
    function totalAssets() public view override returns (uint256) {
        return IERC20(asset()).balanceOf(address(this)) + totalDebt;
    }

    function _availableLiquidity() internal view returns (uint256) {
        return IERC20(asset()).balanceOf(address(this));
    }

    function _totalAllocation() internal view returns (uint256 total) {
        for (uint i = 0; i < strategyList.length; i++) {
            if (strategies[strategyList[i]].active) {
                total += strategies[strategyList[i]].allocation;
            }
        }
    }

    // ============ 수수료 설정 ============
    function setFees(
        uint256 _performanceFee,
        uint256 _managementFee
    ) external onlyRole(DEFAULT_ADMIN_ROLE) {
        if (_performanceFee + _managementFee > MAX_FEE) revert InvalidFee();

        performanceFee = _performanceFee;
        managementFee = _managementFee;
    }

    // ============ 긴급 기능 ============
    function pause() external onlyRole(GUARDIAN_ROLE) {
        _pause();
    }

    function unpause() external onlyRole(DEFAULT_ADMIN_ROLE) {
        _unpause();
    }

    /**
     * @notice 긴급 출금 (수수료 없이)
     */
    function emergencyWithdraw() external nonReentrant {
        uint256 shares = balanceOf(msg.sender);
        uint256 assets = convertToAssets(shares);

        if (assets > _availableLiquidity()) {
            // 긴급 시 손실 감수하고 비례 출금
            assets = _availableLiquidity() * shares / totalSupply();
        }

        _burn(msg.sender, shares);
        IERC20(asset()).safeTransfer(msg.sender, assets);
    }
}

interface IStrategy {
    function deposit(uint256 amount) external;
    function withdraw(uint256 amount) external returns (uint256);
    function harvest() external returns (uint256 profit, uint256 loss);
    function totalAssets() external view returns (uint256);
}
```

### 8.2.3 WIA 대출 풀 구현

**이자율 모델 기반 대출 풀:**

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.23;

import {IERC20} from "@openzeppelin/contracts/token/ERC20/IERC20.sol";
import {SafeERC20} from "@openzeppelin/contracts/token/ERC20/utils/SafeERC20.sol";
import {ReentrancyGuard} from "@openzeppelin/contracts/utils/ReentrancyGuard.sol";
import {Pausable} from "@openzeppelin/contracts/utils/Pausable.sol";
import {AccessControl} from "@openzeppelin/contracts/access/AccessControl.sol";

/**
 * @title WIALendingPool
 * @notice 담보 기반 대출 풀
 * @dev 변동 이자율, 청산 메커니즘, 플래시론 지원
 */
contract WIALendingPool is ReentrancyGuard, Pausable, AccessControl {
    using SafeERC20 for IERC20;

    // ============ 역할 ============
    bytes32 public constant RISK_ADMIN = keccak256("RISK_ADMIN");
    bytes32 public constant LIQUIDATOR_ROLE = keccak256("LIQUIDATOR_ROLE");

    // ============ 상수 ============
    uint256 public constant PRECISION = 1e18;
    uint256 public constant SECONDS_PER_YEAR = 365 days;
    uint256 public constant FLASH_LOAN_FEE = 9; // 0.09%
    uint256 public constant FLASH_LOAN_FEE_PRECISION = 10000;

    // ============ 시장 파라미터 ============
    struct MarketParams {
        IERC20 asset;
        IERC20 collateral;
        uint256 ltv;                    // Loan-to-Value (basis points)
        uint256 liquidationThreshold;    // 청산 임계값
        uint256 liquidationPenalty;      // 청산 페널티
        uint256 reserveFactor;           // 준비금 비율
        bool isActive;
    }

    // ============ 이자율 모델 파라미터 ============
    struct InterestRateModel {
        uint256 baseRate;           // 기본 이자율
        uint256 slope1;             // 최적 이용률 이하 기울기
        uint256 slope2;             // 최적 이용률 이상 기울기
        uint256 optimalUtilization; // 최적 이용률
    }

    // ============ 시장 상태 ============
    struct MarketState {
        uint256 totalDeposits;
        uint256 totalBorrows;
        uint256 totalReserves;
        uint256 borrowIndex;
        uint256 supplyIndex;
        uint256 lastUpdateTimestamp;
    }

    // ============ 사용자 포지션 ============
    struct UserPosition {
        uint256 depositAmount;
        uint256 borrowAmount;
        uint256 collateralAmount;
        uint256 borrowIndex;      // 사용자의 마지막 borrow index
        uint256 supplyIndex;      // 사용자의 마지막 supply index
    }

    // ============ 상태 변수 ============
    MarketParams public marketParams;
    InterestRateModel public interestRateModel;
    MarketState public marketState;

    mapping(address => UserPosition) public userPositions;

    // 오라클
    address public priceOracle;

    // ============ 이벤트 ============
    event Deposit(address indexed user, uint256 amount);
    event Withdraw(address indexed user, uint256 amount);
    event Borrow(address indexed user, uint256 amount);
    event Repay(address indexed user, uint256 amount);
    event CollateralDeposit(address indexed user, uint256 amount);
    event CollateralWithdraw(address indexed user, uint256 amount);
    event Liquidation(
        address indexed liquidator,
        address indexed borrower,
        uint256 repaidAmount,
        uint256 seizedCollateral
    );
    event FlashLoan(address indexed borrower, uint256 amount, uint256 fee);

    // ============ 에러 ============
    error InsufficientLiquidity();
    error InsufficientCollateral();
    error HealthyPosition();
    error InvalidAmount();
    error MarketNotActive();
    error FlashLoanFailed();

    // ============ 생성자 ============
    constructor(
        address _asset,
        address _collateral,
        address _priceOracle,
        address _admin
    ) {
        marketParams = MarketParams({
            asset: IERC20(_asset),
            collateral: IERC20(_collateral),
            ltv: 7500,                      // 75%
            liquidationThreshold: 8000,      // 80%
            liquidationPenalty: 500,         // 5%
            reserveFactor: 1000,             // 10%
            isActive: true
        });

        interestRateModel = InterestRateModel({
            baseRate: 2e16,              // 2%
            slope1: 4e16,                // 4%
            slope2: 100e16,              // 100%
            optimalUtilization: 80e16    // 80%
        });

        marketState.borrowIndex = PRECISION;
        marketState.supplyIndex = PRECISION;
        marketState.lastUpdateTimestamp = block.timestamp;

        priceOracle = _priceOracle;

        _grantRole(DEFAULT_ADMIN_ROLE, _admin);
        _grantRole(RISK_ADMIN, _admin);
    }

    // ============ 핵심 기능 ============

    /**
     * @notice 자산 예치
     */
    function deposit(uint256 _amount) external nonReentrant whenNotPaused {
        if (_amount == 0) revert InvalidAmount();
        if (!marketParams.isActive) revert MarketNotActive();

        _accrueInterest();

        // 이자 반영된 예치금 업데이트
        UserPosition storage position = userPositions[msg.sender];
        position.depositAmount = _getUpdatedDeposit(msg.sender) + _amount;
        position.supplyIndex = marketState.supplyIndex;

        marketState.totalDeposits += _amount;

        marketParams.asset.safeTransferFrom(msg.sender, address(this), _amount);

        emit Deposit(msg.sender, _amount);
    }

    /**
     * @notice 자산 출금
     */
    function withdraw(uint256 _amount) external nonReentrant {
        _accrueInterest();

        UserPosition storage position = userPositions[msg.sender];
        uint256 updatedDeposit = _getUpdatedDeposit(msg.sender);

        if (_amount > updatedDeposit) revert InvalidAmount();
        if (_amount > _availableLiquidity()) revert InsufficientLiquidity();

        position.depositAmount = updatedDeposit - _amount;
        position.supplyIndex = marketState.supplyIndex;
        marketState.totalDeposits -= _amount;

        marketParams.asset.safeTransfer(msg.sender, _amount);

        emit Withdraw(msg.sender, _amount);
    }

    /**
     * @notice 담보 예치
     */
    function depositCollateral(uint256 _amount) external nonReentrant whenNotPaused {
        if (_amount == 0) revert InvalidAmount();

        userPositions[msg.sender].collateralAmount += _amount;

        marketParams.collateral.safeTransferFrom(msg.sender, address(this), _amount);

        emit CollateralDeposit(msg.sender, _amount);
    }

    /**
     * @notice 대출
     */
    function borrow(uint256 _amount) external nonReentrant whenNotPaused {
        if (_amount == 0) revert InvalidAmount();
        if (_amount > _availableLiquidity()) revert InsufficientLiquidity();

        _accrueInterest();

        UserPosition storage position = userPositions[msg.sender];

        // 대출 후 담보 비율 확인
        uint256 newBorrowAmount = _getUpdatedBorrow(msg.sender) + _amount;
        uint256 maxBorrow = _getMaxBorrow(msg.sender);

        if (newBorrowAmount > maxBorrow) revert InsufficientCollateral();

        position.borrowAmount = newBorrowAmount;
        position.borrowIndex = marketState.borrowIndex;
        marketState.totalBorrows += _amount;

        marketParams.asset.safeTransfer(msg.sender, _amount);

        emit Borrow(msg.sender, _amount);
    }

    /**
     * @notice 대출 상환
     */
    function repay(uint256 _amount) external nonReentrant {
        _accrueInterest();

        UserPosition storage position = userPositions[msg.sender];
        uint256 updatedBorrow = _getUpdatedBorrow(msg.sender);

        uint256 repayAmount = _amount > updatedBorrow ? updatedBorrow : _amount;

        position.borrowAmount = updatedBorrow - repayAmount;
        position.borrowIndex = marketState.borrowIndex;
        marketState.totalBorrows -= repayAmount;

        marketParams.asset.safeTransferFrom(msg.sender, address(this), repayAmount);

        emit Repay(msg.sender, repayAmount);
    }

    /**
     * @notice 청산
     */
    function liquidate(
        address _borrower,
        uint256 _repayAmount
    ) external nonReentrant onlyRole(LIQUIDATOR_ROLE) {
        _accrueInterest();

        // 청산 가능 여부 확인
        if (_getHealthFactor(_borrower) >= PRECISION) revert HealthyPosition();

        UserPosition storage position = userPositions[_borrower];
        uint256 updatedBorrow = _getUpdatedBorrow(_borrower);

        // 최대 청산 금액 (50%)
        uint256 maxRepay = updatedBorrow / 2;
        uint256 repayAmount = _repayAmount > maxRepay ? maxRepay : _repayAmount;

        // 청산 보상 계산
        uint256 collateralPrice = _getCollateralPrice();
        uint256 assetPrice = _getAssetPrice();

        uint256 collateralToSeize = repayAmount * assetPrice *
            (10000 + marketParams.liquidationPenalty) / collateralPrice / 10000;

        if (collateralToSeize > position.collateralAmount) {
            collateralToSeize = position.collateralAmount;
        }

        // 상태 업데이트
        position.borrowAmount = updatedBorrow - repayAmount;
        position.borrowIndex = marketState.borrowIndex;
        position.collateralAmount -= collateralToSeize;
        marketState.totalBorrows -= repayAmount;

        // 토큰 전송
        marketParams.asset.safeTransferFrom(msg.sender, address(this), repayAmount);
        marketParams.collateral.safeTransfer(msg.sender, collateralToSeize);

        emit Liquidation(msg.sender, _borrower, repayAmount, collateralToSeize);
    }

    /**
     * @notice 플래시론
     */
    function flashLoan(
        address _receiver,
        uint256 _amount,
        bytes calldata _data
    ) external nonReentrant {
        if (_amount > _availableLiquidity()) revert InsufficientLiquidity();

        uint256 balanceBefore = marketParams.asset.balanceOf(address(this));
        uint256 fee = _amount * FLASH_LOAN_FEE / FLASH_LOAN_FEE_PRECISION;

        marketParams.asset.safeTransfer(_receiver, _amount);

        IFlashLoanReceiver(_receiver).executeOperation(_amount, fee, _data);

        uint256 balanceAfter = marketParams.asset.balanceOf(address(this));

        if (balanceAfter < balanceBefore + fee) revert FlashLoanFailed();

        // 수수료를 준비금으로
        marketState.totalReserves += fee;

        emit FlashLoan(_receiver, _amount, fee);
    }

    // ============ 이자 계산 ============

    /**
     * @notice 이자 누적
     */
    function _accrueInterest() internal {
        uint256 timeDelta = block.timestamp - marketState.lastUpdateTimestamp;
        if (timeDelta == 0) return;

        uint256 utilization = _getUtilization();
        uint256 borrowRate = _getBorrowRate(utilization);

        // Borrow Index 업데이트
        uint256 borrowInterest = borrowRate * timeDelta / SECONDS_PER_YEAR;
        marketState.borrowIndex = marketState.borrowIndex *
            (PRECISION + borrowInterest) / PRECISION;

        // 총 이자 수익
        uint256 totalInterest = marketState.totalBorrows * borrowInterest / PRECISION;

        // 준비금 적립
        uint256 reserveIncrease = totalInterest * marketParams.reserveFactor / 10000;
        marketState.totalReserves += reserveIncrease;

        // Supply Index 업데이트 (예금자 이자)
        uint256 supplyInterest = totalInterest - reserveIncrease;
        if (marketState.totalDeposits > 0) {
            marketState.supplyIndex = marketState.supplyIndex *
                (PRECISION + supplyInterest * PRECISION / marketState.totalDeposits) / PRECISION;
        }

        marketState.lastUpdateTimestamp = block.timestamp;
    }

    /**
     * @notice 대출 이자율 계산
     */
    function _getBorrowRate(uint256 _utilization) internal view returns (uint256) {
        InterestRateModel memory model = interestRateModel;

        if (_utilization <= model.optimalUtilization) {
            return model.baseRate + _utilization * model.slope1 / model.optimalUtilization;
        } else {
            uint256 excessUtilization = _utilization - model.optimalUtilization;
            uint256 maxExcess = PRECISION - model.optimalUtilization;

            return model.baseRate + model.slope1 +
                excessUtilization * model.slope2 / maxExcess;
        }
    }

    function _getUtilization() internal view returns (uint256) {
        if (marketState.totalDeposits == 0) return 0;
        return marketState.totalBorrows * PRECISION / marketState.totalDeposits;
    }

    // ============ 뷰 함수 ============

    function _getUpdatedDeposit(address _user) internal view returns (uint256) {
        UserPosition storage position = userPositions[_user];
        if (position.supplyIndex == 0) return position.depositAmount;

        return position.depositAmount * marketState.supplyIndex / position.supplyIndex;
    }

    function _getUpdatedBorrow(address _user) internal view returns (uint256) {
        UserPosition storage position = userPositions[_user];
        if (position.borrowIndex == 0) return position.borrowAmount;

        return position.borrowAmount * marketState.borrowIndex / position.borrowIndex;
    }

    function _getMaxBorrow(address _user) internal view returns (uint256) {
        uint256 collateralValue = userPositions[_user].collateralAmount *
            _getCollateralPrice() / PRECISION;

        return collateralValue * marketParams.ltv / 10000;
    }

    function _getHealthFactor(address _user) internal view returns (uint256) {
        uint256 borrowValue = _getUpdatedBorrow(_user) * _getAssetPrice() / PRECISION;
        if (borrowValue == 0) return type(uint256).max;

        uint256 collateralValue = userPositions[_user].collateralAmount *
            _getCollateralPrice() / PRECISION;

        uint256 liquidationValue = collateralValue * marketParams.liquidationThreshold / 10000;

        return liquidationValue * PRECISION / borrowValue;
    }

    function _availableLiquidity() internal view returns (uint256) {
        return marketParams.asset.balanceOf(address(this));
    }

    function _getAssetPrice() internal view returns (uint256) {
        return IPriceOracle(priceOracle).getPrice(address(marketParams.asset));
    }

    function _getCollateralPrice() internal view returns (uint256) {
        return IPriceOracle(priceOracle).getPrice(address(marketParams.collateral));
    }

    // ============ 외부 뷰 함수 ============

    function getHealthFactor(address _user) external view returns (uint256) {
        return _getHealthFactor(_user);
    }

    function getUserPosition(address _user) external view returns (
        uint256 deposit,
        uint256 borrow,
        uint256 collateral,
        uint256 healthFactor
    ) {
        return (
            _getUpdatedDeposit(_user),
            _getUpdatedBorrow(_user),
            userPositions[_user].collateralAmount,
            _getHealthFactor(_user)
        );
    }

    function getCurrentRates() external view returns (
        uint256 borrowRate,
        uint256 supplyRate,
        uint256 utilization
    ) {
        utilization = _getUtilization();
        borrowRate = _getBorrowRate(utilization);

        if (marketState.totalDeposits > 0) {
            supplyRate = borrowRate * utilization *
                (10000 - marketParams.reserveFactor) / PRECISION / 10000;
        }
    }
}

interface IFlashLoanReceiver {
    function executeOperation(
        uint256 amount,
        uint256 fee,
        bytes calldata data
    ) external;
}

interface IPriceOracle {
    function getPrice(address asset) external view returns (uint256);
}
```

## 8.3 테스트 작성

### 8.3.1 단위 테스트

**Foundry 테스트 예제:**

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.23;

import {Test, console2} from "forge-std/Test.sol";
import {WIAToken} from "../src/core/WIAToken.sol";

contract WIATokenTest is Test {
    WIAToken public token;

    address public admin = makeAddr("admin");
    address public treasury = makeAddr("treasury");
    address public user1 = makeAddr("user1");
    address public user2 = makeAddr("user2");

    function setUp() public {
        vm.prank(admin);
        token = new WIAToken(admin, treasury);
    }

    // ============ 초기화 테스트 ============

    function test_InitialState() public view {
        assertEq(token.name(), "WIA Token");
        assertEq(token.symbol(), "WIA");
        assertEq(token.decimals(), 18);
        assertEq(token.totalSupply(), 200_000_000 * 10**18);
        assertEq(token.balanceOf(treasury), 200_000_000 * 10**18);
    }

    function test_AdminRoles() public view {
        assertTrue(token.hasRole(token.DEFAULT_ADMIN_ROLE(), admin));
        assertTrue(token.hasRole(token.MINTER_ROLE(), admin));
        assertTrue(token.hasRole(token.PAUSER_ROLE(), admin));
    }

    // ============ 민트 테스트 ============

    function test_Mint() public {
        uint256 amount = 1_000_000 * 10**18;

        vm.prank(admin);
        token.mint(user1, amount);

        assertEq(token.balanceOf(user1), amount);
    }

    function test_RevertMint_NotMinter() public {
        vm.prank(user1);
        vm.expectRevert();
        token.mint(user1, 1000);
    }

    function test_RevertMint_ExceedsMaxSupply() public {
        uint256 remaining = token.MAX_SUPPLY() - token.totalSupply();

        vm.prank(admin);
        vm.expectRevert(WIAToken.MaxSupplyExceeded.selector);
        token.mint(user1, remaining + 1);
    }

    function test_MintCooldown() public {
        uint256 maxPerPeriod = token.MAX_MINT_PER_PERIOD();

        vm.startPrank(admin);

        // 첫 번째 민트
        token.mint(user1, maxPerPeriod);

        // 쿨다운 내 추가 민트 실패
        vm.expectRevert(WIAToken.MintLimitExceeded.selector);
        token.mint(user1, 1);

        // 1일 후 다시 민트 가능
        vm.warp(block.timestamp + 1 days);
        token.mint(user1, maxPerPeriod);

        vm.stopPrank();
    }

    // ============ 전송 제한 테스트 ============

    function test_TransferRestricted_Whitelisted() public {
        // 화이트리스트된 주소 간 전송 가능
        vm.prank(treasury);
        token.transfer(admin, 1000);

        assertEq(token.balanceOf(admin), 1000);
    }

    function test_RevertTransfer_NotWhitelisted() public {
        // 화이트리스트에 treasury 추가하여 user1에게 전송
        vm.prank(admin);
        token.setWhitelist(user1, true);

        vm.prank(treasury);
        token.transfer(user1, 1000);

        // user1은 화이트리스트, user2는 아님
        // 둘 다 화이트리스트가 아니면 실패
        vm.prank(admin);
        token.setWhitelist(user1, false);

        vm.prank(user1);
        vm.expectRevert(WIAToken.TransferRestricted.selector);
        token.transfer(user2, 500);
    }

    function test_TransferUnrestricted() public {
        vm.prank(admin);
        token.setTransferRestricted(false);

        vm.prank(treasury);
        token.transfer(user1, 1000);

        vm.prank(user1);
        token.transfer(user2, 500);

        assertEq(token.balanceOf(user2), 500);
    }

    // ============ 일시정지 테스트 ============

    function test_Pause() public {
        vm.prank(admin);
        token.pause();

        assertTrue(token.paused());

        vm.prank(treasury);
        vm.expectRevert();
        token.transfer(admin, 1000);
    }

    // ============ Fuzz 테스트 ============

    function testFuzz_Mint(address to, uint256 amount) public {
        vm.assume(to != address(0));
        amount = bound(amount, 1, token.MAX_MINT_PER_PERIOD());

        uint256 supplyBefore = token.totalSupply();

        vm.prank(admin);
        token.mint(to, amount);

        assertEq(token.totalSupply(), supplyBefore + amount);
        assertEq(token.balanceOf(to), amount);
    }

    function testFuzz_Transfer(uint256 amount) public {
        amount = bound(amount, 1, token.balanceOf(treasury));

        vm.prank(admin);
        token.setTransferRestricted(false);

        uint256 balanceBefore = token.balanceOf(user1);

        vm.prank(treasury);
        token.transfer(user1, amount);

        assertEq(token.balanceOf(user1), balanceBefore + amount);
    }
}
```

### 8.3.2 불변성 테스트

**Invariant 테스트:**

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.23;

import {Test} from "forge-std/Test.sol";
import {StdInvariant} from "forge-std/StdInvariant.sol";
import {WIAVault} from "../src/core/WIAVault.sol";
import {MockERC20} from "./mocks/MockERC20.sol";

contract WIAVaultInvariantTest is StdInvariant, Test {
    WIAVault public vault;
    MockERC20 public asset;

    address public admin = makeAddr("admin");
    VaultHandler public handler;

    function setUp() public {
        asset = new MockERC20("Test Asset", "TST", 18);
        vault = new WIAVault(
            asset,
            "WIA Vault",
            "vWIA",
            admin,
            admin
        );

        handler = new VaultHandler(vault, asset);

        // 핸들러에 자산 제공
        asset.mint(address(handler), 1_000_000 * 10**18);

        targetContract(address(handler));
    }

    // 불변성 1: 총 공급량 = 모든 사용자 잔액 합
    function invariant_TotalSupplyEqualsBalances() public view {
        uint256 totalBalance = 0;

        address[] memory actors = handler.getActors();
        for (uint i = 0; i < actors.length; i++) {
            totalBalance += vault.balanceOf(actors[i]);
        }

        assertEq(vault.totalSupply(), totalBalance);
    }

    // 불변성 2: 총 자산 >= 총 공급량 (손실 없는 경우)
    function invariant_AssetsGteSupply() public view {
        // 전략 손실이 없다면 항상 성립
        if (handler.totalLoss() == 0) {
            assertGe(vault.totalAssets(), vault.totalSupply());
        }
    }

    // 불변성 3: 출금 시 자산 받음
    function invariant_WithdrawReceivesAssets() public view {
        // 마지막 출금이 성공했다면 자산을 받았어야 함
        assertTrue(handler.lastWithdrawReceivedAssets());
    }
}

contract VaultHandler is Test {
    WIAVault public vault;
    MockERC20 public asset;

    address[] public actors;
    mapping(address => bool) public isActor;

    uint256 public totalLoss;
    bool public lastWithdrawReceivedAssets = true;

    constructor(WIAVault _vault, MockERC20 _asset) {
        vault = _vault;
        asset = _asset;
    }

    // 예치 핸들러
    function deposit(uint256 _amount, uint256 _actorSeed) public {
        address actor = _getActor(_actorSeed);
        _amount = bound(_amount, 1, asset.balanceOf(address(this)) / 10);

        asset.transfer(actor, _amount);

        vm.startPrank(actor);
        asset.approve(address(vault), _amount);
        vault.deposit(_amount, actor);
        vm.stopPrank();
    }

    // 출금 핸들러
    function withdraw(uint256 _shares, uint256 _actorSeed) public {
        address actor = _getActor(_actorSeed);
        uint256 balance = vault.balanceOf(actor);

        if (balance == 0) return;

        _shares = bound(_shares, 1, balance);

        uint256 assetsBefore = asset.balanceOf(actor);

        vm.prank(actor);
        vault.redeem(_shares, actor, actor);

        lastWithdrawReceivedAssets = asset.balanceOf(actor) > assetsBefore;
    }

    function _getActor(uint256 _seed) internal returns (address) {
        if (actors.length == 0 || _seed % 3 == 0) {
            address newActor = makeAddr(string(abi.encodePacked("actor", actors.length)));
            actors.push(newActor);
            isActor[newActor] = true;
            return newActor;
        }

        return actors[_seed % actors.length];
    }

    function getActors() external view returns (address[] memory) {
        return actors;
    }
}

contract MockERC20 {
    string public name;
    string public symbol;
    uint8 public decimals;
    uint256 public totalSupply;
    mapping(address => uint256) public balanceOf;
    mapping(address => mapping(address => uint256)) public allowance;

    constructor(string memory _name, string memory _symbol, uint8 _decimals) {
        name = _name;
        symbol = _symbol;
        decimals = _decimals;
    }

    function mint(address to, uint256 amount) external {
        balanceOf[to] += amount;
        totalSupply += amount;
    }

    function approve(address spender, uint256 amount) external returns (bool) {
        allowance[msg.sender][spender] = amount;
        return true;
    }

    function transfer(address to, uint256 amount) external returns (bool) {
        balanceOf[msg.sender] -= amount;
        balanceOf[to] += amount;
        return true;
    }

    function transferFrom(address from, address to, uint256 amount) external returns (bool) {
        allowance[from][msg.sender] -= amount;
        balanceOf[from] -= amount;
        balanceOf[to] += amount;
        return true;
    }
}
```

## 8.4 배포 가이드

### 8.4.1 배포 스크립트

**Foundry 배포 스크립트:**

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.23;

import {Script, console2} from "forge-std/Script.sol";
import {WIAToken} from "../src/core/WIAToken.sol";
import {WIAVault} from "../src/core/WIAVault.sol";
import {WIALendingPool} from "../src/defi/lending/WIALendingPool.sol";

contract DeployWIA is Script {
    // 배포 파라미터
    address public admin;
    address public treasury;
    address public priceOracle;

    // 배포된 컨트랙트
    WIAToken public token;
    WIAVault public vault;
    WIALendingPool public lendingPool;

    function setUp() public {
        // 환경 변수에서 로드
        admin = vm.envAddress("ADMIN_ADDRESS");
        treasury = vm.envAddress("TREASURY_ADDRESS");
        priceOracle = vm.envAddress("PRICE_ORACLE");
    }

    function run() public {
        uint256 deployerPrivateKey = vm.envUint("PRIVATE_KEY");

        vm.startBroadcast(deployerPrivateKey);

        // 1. 토큰 배포
        token = new WIAToken(admin, treasury);
        console2.log("WIAToken deployed at:", address(token));

        // 2. 금고 배포
        vault = new WIAVault(
            token,
            "WIA Vault Token",
            "vWIA",
            admin,
            treasury
        );
        console2.log("WIAVault deployed at:", address(vault));

        // 3. 대출 풀 배포
        // 담보로 WETH 사용 가정
        address weth = vm.envAddress("WETH_ADDRESS");

        lendingPool = new WIALendingPool(
            address(token),
            weth,
            priceOracle,
            admin
        );
        console2.log("WIALendingPool deployed at:", address(lendingPool));

        vm.stopBroadcast();

        // 배포 정보 저장
        _saveDeployment();
    }

    function _saveDeployment() internal {
        string memory json = string(abi.encodePacked(
            '{"token":"', vm.toString(address(token)),
            '","vault":"', vm.toString(address(vault)),
            '","lendingPool":"', vm.toString(address(lendingPool)),
            '","chainId":', vm.toString(block.chainid),
            ',"timestamp":', vm.toString(block.timestamp),
            '}'
        ));

        string memory path = string(abi.encodePacked(
            "./deployments/",
            vm.toString(block.chainid),
            ".json"
        ));

        vm.writeFile(path, json);
    }
}
```

### 8.4.2 배포 실행

**배포 명령어:**

```bash
#!/bin/bash
# WIA 배포 스크립트

set -e

# 환경 변수 로드
source .env

# 네트워크별 배포

# Sepolia 테스트넷
echo "Deploying to Sepolia..."
forge script script/Deploy.s.sol:DeployWIA \
    --rpc-url $SEPOLIA_RPC_URL \
    --broadcast \
    --verify \
    --etherscan-api-key $ETHERSCAN_API_KEY \
    -vvvv

# Mainnet (프로덕션)
echo "Deploying to Mainnet..."
forge script script/Deploy.s.sol:DeployWIA \
    --rpc-url $MAINNET_RPC_URL \
    --broadcast \
    --verify \
    --etherscan-api-key $ETHERSCAN_API_KEY \
    --slow \
    -vvvv

# 배포 검증
echo "Verifying deployment..."
forge verify-contract \
    --chain-id 1 \
    --num-of-optimizations 200 \
    --watch \
    --constructor-args $(cast abi-encode "constructor(address,address)" $ADMIN_ADDRESS $TREASURY_ADDRESS) \
    $TOKEN_ADDRESS \
    src/core/WIAToken.sol:WIAToken \
    --etherscan-api-key $ETHERSCAN_API_KEY
```

## 8.5 운영 가이드

### 8.5.1 모니터링 설정

**모니터링 체크리스트:**

```typescript
interface 모니터링체크리스트 {
  온체인지표: {
    TVL: "총 예치 자산 추적";
    거래량: "일일/주간/월간 거래량";
    활성사용자: "고유 주소 수";
    가스사용량: "함수별 가스 소비";
  };

  보안지표: {
    이상거래: "비정상적 대규모 거래";
    가격변동: "오라클 가격 급변";
    청산: "청산 이벤트 모니터링";
    거버넌스: "악의적 제안 감지";
  };

  인프라지표: {
    RPC응답: "노드 응답 시간";
    블록동기화: "블록 동기화 상태";
    서명자상태: "다중서명 서명자 활성";
  };

  알림설정: {
    텔레그램: "실시간 알림";
    이메일: "중요 이벤트 요약";
    PagerDuty: "긴급 알림";
  };
}
```

### 8.5.2 긴급 대응 절차

```markdown
## WIA 긴급 대응 절차

### 레벨 1: 경미한 이상 (관찰)
- 비정상 트래픽 증가
- 경미한 가격 변동
**대응**: 모니터링 강화, 원인 분석

### 레벨 2: 중간 수준 (주의)
- 단일 기능 장애
- 중간 규모 이상 거래
**대응**:
1. 담당자 알림
2. 영향 범위 파악
3. 필요시 해당 기능 일시 정지

### 레벨 3: 심각 (긴급)
- 자금 손실 위험
- 스마트 컨트랙트 취약점 발견
**대응**:
1. 긴급 정지 실행
2. 전체 팀 소집
3. 외부 보안 전문가 연락
4. 커뮤니티 공지

### 긴급 연락망
- 보안팀 리드: [연락처]
- CTO: [연락처]
- 법무팀: [연락처]
- 감사 파트너: [연락처]
```

본 장에서는 WIA 블록체인 금융 시스템의 실전 구현 가이드를 다루었습니다. 개발 환경 설정, 핵심 컨트랙트 구현, 테스트 작성, 배포 및 운영까지 전체 개발 라이프사이클을 살펴보았습니다. 다음 장에서는 미래 트렌드와 WIA의 비전에 대해 논의하겠습니다.
