# 제3장: 토큰화 표준

## 자산 토큰화, 증권 토큰 및 실물자산 프로토콜

### 프로그래머블 금융의 기초

---

## 개요

토큰화는 실물자산을 블록체인 네트워크의 디지털 토큰으로 변환하는 것을 의미합니다. 이 장에서는 규정을 준수하고 상호운용 가능한 토큰화 자산을 생성하기 위한 기술 표준, 규제 프레임워크 및 구현 패턴을 다룹니다.

---

## 토큰 표준 기초

### 이더리움 토큰 표준

**ERC 표준 발전:**

| 표준 | 목적 | 주요 기능 | 채택 |
|------|------|----------|------|
| ERC-20 | 대체 가능 토큰 | 전송, 승인, balanceOf | 보편적 |
| ERC-721 | 대체 불가 토큰 | 고유 소유권, 메타데이터 | 높음 |
| ERC-1155 | 다중 토큰 | 배치 작업, 혼합 유형 | 성장 중 |
| ERC-3643 | 증권 토큰 | 신원, 컴플라이언스 모듈 | 기관용 |
| ERC-4626 | 토큰화 볼트 | 표준화된 수익 창출 | DeFi 표준 |
| ERC-6551 | 토큰 바운드 계정 | NFT를 지갑으로 | 신흥 |

### ERC-3643: 증권 토큰 표준

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/token/ERC20/ERC20.sol";
import "./interfaces/IIdentityRegistry.sol";
import "./interfaces/ICompliance.sol";

/**
 * @title WIA증권토큰
 * @dev 전송 제한이 있는 ERC-3643 준수 증권 토큰
 */
contract WIA증권토큰 is ERC20 {
    IIdentityRegistry public identityRegistry;
    ICompliance public compliance;

    address public tokenIssuer;
    bool public tokenPaused;

    // 컴플라이언스 조치를 위한 에이전트 역할
    mapping(address => bool) public agents;

    // 동결된 주소 (규제 조치)
    mapping(address => bool) public frozen;

    // 다른 주식 등급을 위한 파티션 지원
    mapping(bytes32 => mapping(address => uint256)) public partitionBalances;
    bytes32[] public partitions;

    event TokenFrozen(address indexed account, address indexed agent);
    event TokenUnfrozen(address indexed account, address indexed agent);
    event IdentityRegistryUpdated(address indexed newRegistry);
    event ComplianceUpdated(address indexed newCompliance);

    modifier onlyAgent() {
        require(agents[msg.sender], "호출자가 에이전트가 아님");
        _;
    }

    modifier onlyIssuer() {
        require(msg.sender == tokenIssuer, "호출자가 발행자가 아님");
        _;
    }

    modifier whenNotPaused() {
        require(!tokenPaused, "토큰이 일시 중지됨");
        _;
    }

    modifier whenNotFrozen(address _account) {
        require(!frozen[_account], "계정이 동결됨");
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
     * @dev 컴플라이언스 검사를 포함한 전송 오버라이드
     */
    function transfer(address to, uint256 amount)
        public
        override
        whenNotPaused
        whenNotFrozen(msg.sender)
        whenNotFrozen(to)
        returns (bool)
    {
        require(_canTransfer(msg.sender, to, amount), "전송이 규정을 준수하지 않음");
        return super.transfer(to, amount);
    }

    /**
     * @dev 모든 규칙을 준수하는지 전송 확인
     */
    function _canTransfer(
        address from,
        address to,
        uint256 amount
    ) internal view returns (bool) {
        // 양 당사자가 검증되었는지 확인
        require(
            identityRegistry.isVerified(from),
            "송신자가 검증되지 않음"
        );
        require(
            identityRegistry.isVerified(to),
            "수신자가 검증되지 않음"
        );

        // 컴플라이언스 규칙 확인
        require(
            compliance.canTransfer(from, to, amount),
            "컴플라이언스 검사 실패"
        );

        return true;
    }

    /**
     * @dev 규제 준수를 위한 강제 전송
     */
    function forcedTransfer(
        address from,
        address to,
        uint256 amount,
        bytes calldata reason
    ) external onlyAgent returns (bool) {
        require(identityRegistry.isVerified(to), "수신자가 검증되지 않음");
        _transfer(from, to, amount);
        emit ForcedTransfer(from, to, amount, reason);
        return true;
    }

    /**
     * @dev 계정 동결 (규제 조치)
     */
    function freeze(address account) external onlyAgent {
        frozen[account] = true;
        emit TokenFrozen(account, msg.sender);
    }

    /**
     * @dev 계정 동결 해제
     */
    function unfreeze(address account) external onlyAgent {
        frozen[account] = false;
        emit TokenUnfrozen(account, msg.sender);
    }

    /**
     * @dev 새 토큰 발행 (발행자 전용)
     */
    function mint(address to, uint256 amount) external onlyIssuer {
        require(identityRegistry.isVerified(to), "수신자가 검증되지 않음");
        _mint(to, amount);
    }

    event ForcedTransfer(address indexed from, address indexed to, uint256 amount, bytes reason);
}
```

### 신원 레지스트리

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "./interfaces/IIdentity.sol";
import "./interfaces/IClaimVerifier.sol";

/**
 * @title 신원레지스트리
 * @dev 투자자 신원 및 검증 상태 레지스트리
 */
contract 신원레지스트리 {
    // 지갑 주소에서 신원 컨트랙트로의 매핑
    mapping(address => address) public identity;

    // 지갑 주소에서 국가 코드로의 매핑
    mapping(address => uint16) public investorCountry;

    // 검증에 필요한 클레임 토픽
    uint256[] public requiredClaimTopics;

    // 토픽별 신뢰할 수 있는 클레임 발행자
    mapping(uint256 => address[]) public trustedIssuers;

    // 신원 레지스트리 에이전트
    mapping(address => bool) public agents;

    event IdentityRegistered(address indexed investor, address indexed identityContract);
    event IdentityRemoved(address indexed investor);
    event CountryUpdated(address indexed investor, uint16 indexed country);

    modifier onlyAgent() {
        require(agents[msg.sender], "권한 없음");
        _;
    }

    /**
     * @dev 투자자 신원 등록
     */
    function registerIdentity(
        address _investor,
        address _identity,
        uint16 _country
    ) external onlyAgent {
        require(_investor != address(0), "유효하지 않은 주소");
        require(_identity != address(0), "유효하지 않은 신원");
        require(identity[_investor] == address(0), "이미 등록됨");

        identity[_investor] = _identity;
        investorCountry[_investor] = _country;

        emit IdentityRegistered(_investor, _identity);
    }

    /**
     * @dev 투자자가 검증되었는지 확인
     */
    function isVerified(address _investor) public view returns (bool) {
        if (identity[_investor] == address(0)) return false;

        IIdentity investorIdentity = IIdentity(identity[_investor]);

        // 모든 필수 클레임 토픽 확인
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
     * @dev 신원이 발행자로부터 유효한 클레임을 가지고 있는지 확인
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
                // 클레임이 여전히 유효한지 검증
                IClaimVerifier verifier = IClaimVerifier(_issuer);
                if (verifier.isClaimValid(_identity, topic, signature, data)) {
                    return true;
                }
            }
        }

        return false;
    }
}
```

---

## 실물자산 토큰화

### RWA 카테고리

**토큰화 가능한 자산 등급:**

| 자산 등급 | 예시 | 시장 규모 | 토큰화율 |
|----------|------|----------|---------|
| 국채 증권 | 국채, 채권 | $30조+ | $20억+ 토큰화 |
| 부동산 | 상업용, 주거용 | $300조+ | $5억+ 토큰화 |
| 사모신용 | 대출, 수취채권 | $1.5조 | $50억+ 토큰화 |
| 상품 | 금, 탄소 크레딧 | $5조+ | $2억+ 토큰화 |
| 펀드 | ETF, 뮤추얼 펀드 | $50조+ | $10억+ 토큰화 |
| 예술 및 수집품 | 미술품, 희귀품 | $1조+ | $1억+ 토큰화 |

### 국채 토큰화

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/token/ERC20/extensions/ERC4626.sol";
import "@openzeppelin/contracts/access/AccessControl.sol";

/**
 * @title 토큰화국채
 * @dev 토큰화 국채 증권용 ERC-4626 볼트
 */
contract 토큰화국채 is ERC4626, AccessControl {
    bytes32 public constant ORACLE_ROLE = keccak256("ORACLE_ROLE");
    bytes32 public constant MANAGER_ROLE = keccak256("MANAGER_ROLE");

    // 국채 세부사항
    struct TreasuryBond {
        string cusip;           // CUSIP 번호
        uint256 faceValue;      // 액면가
        uint256 couponRate;     // 연간 이자율 (베이시스 포인트)
        uint256 maturityDate;   // 만기일 Unix 타임스탬프
        uint256 lastCouponDate; // 마지막 이자 지급일
    }

    TreasuryBond public bond;

    // NAV 추적
    uint256 public lastNAV;
    uint256 public lastNAVUpdate;

    // 적격 투자자 화이트리스트
    mapping(address => bool) public whitelist;

    // 최소 투자금액
    uint256 public minimumInvestment;

    event NAVUpdated(uint256 newNAV, uint256 timestamp);
    event CouponDistributed(uint256 amount, uint256 timestamp);
    event InvestorWhitelisted(address indexed investor);

    modifier onlyWhitelisted() {
        require(whitelist[msg.sender], "화이트리스트에 없음");
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
     * @dev 자산 예치 및 주식 발행
     */
    function deposit(uint256 assets, address receiver)
        public
        override
        onlyWhitelisted
        returns (uint256)
    {
        require(assets >= minimumInvestment, "최소 투자금액 미만");
        return super.deposit(assets, receiver);
    }

    /**
     * @dev 오라클에서 NAV 업데이트
     */
    function updateNAV(uint256 newNAV) external onlyRole(ORACLE_ROLE) {
        lastNAV = newNAV;
        lastNAVUpdate = block.timestamp;
        emit NAVUpdated(newNAV, block.timestamp);
    }

    /**
     * @dev 발생 이자를 포함한 총 자산 계산
     */
    function totalAssets() public view override returns (uint256) {
        if (lastNAVUpdate == 0) {
            return super.totalAssets();
        }

        // 마지막 NAV 업데이트 이후 발생 이자 계산
        uint256 daysSinceUpdate = (block.timestamp - lastNAVUpdate) / 1 days;
        uint256 dailyRate = bond.couponRate / 365;
        uint256 accruedInterest = (lastNAV * dailyRate * daysSinceUpdate) / 10000;

        return lastNAV + accruedInterest;
    }

    /**
     * @dev 이자 지급 분배
     */
    function distributeCoupon() external onlyRole(MANAGER_ROLE) {
        require(
            block.timestamp >= bond.lastCouponDate + 182 days,
            "이자 지급일이 아님"
        );

        uint256 couponAmount = (totalAssets() * bond.couponRate) / 20000; // 반기
        bond.lastCouponDate = block.timestamp;

        // 모든 주주에게 비례 분배
        emit CouponDistributed(couponAmount, block.timestamp);
    }

    /**
     * @dev 투자자 화이트리스트 등록
     */
    function whitelistInvestor(address investor) external onlyRole(MANAGER_ROLE) {
        whitelist[investor] = true;
        emit InvestorWhitelisted(investor);
    }
}
```

### 부동산 토큰화

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "./WIA증권토큰.sol";

/**
 * @title 부동산토큰
 * @dev 임대 수익 분배가 있는 토큰화 부동산
 */
contract 부동산토큰 is WIA증권토큰 {
    struct Property {
        string propertyId;          // 고유 부동산 식별자
        string legalDescription;    // 법적 부동산 설명
        uint256 totalValue;         // 감정 가치
        uint256 tokenizedShares;    // 발행된 총 토큰
        uint256 annualRentalIncome; // 예상 연간 임대 수익
        address propertyManager;    // 부동산 관리 회사
    }

    Property public property;

    // 배당 추적
    uint256 public totalDividendsDistributed;
    mapping(address => uint256) public lastClaimedDividend;
    mapping(address => uint256) public unclaimedDividends;

    // 배당 풀
    uint256 public dividendPool;
    uint256 public dividendPerShare;

    // 운영비 준비금
    uint256 public operatingReserve;
    uint256 public reserveTarget; // 베이시스 포인트 비율

    event DividendDeposited(uint256 amount, uint256 timestamp);
    event DividendClaimed(address indexed investor, uint256 amount);
    event PropertyValueUpdated(uint256 newValue, uint256 timestamp);

    constructor(
        string memory _name,
        string memory _symbol,
        address _identityRegistry,
        address _compliance,
        Property memory _property
    ) WIA증권토큰(_name, _symbol, _identityRegistry, _compliance) {
        property = _property;
    }

    /**
     * @dev 분배를 위한 임대 수익 예치
     */
    function depositDividend() external payable {
        require(msg.sender == property.propertyManager, "부동산 관리자가 아님");

        // 운영 준비금 적립
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
     * @dev 누적 배당 청구
     */
    function claimDividend() external {
        uint256 owed = _calculateOwedDividends(msg.sender);
        require(owed > 0, "청구할 배당이 없음");

        lastClaimedDividend[msg.sender] = dividendPerShare;
        unclaimedDividends[msg.sender] = 0;

        payable(msg.sender).transfer(owed);
        totalDividendsDistributed += owed;

        emit DividendClaimed(msg.sender, owed);
    }

    /**
     * @dev 투자자에게 지급해야 할 배당 계산
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
     * @dev 현재 배당 수익률 조회
     */
    function getCurrentYield() external view returns (uint256) {
        if (property.totalValue == 0) return 0;
        return (property.annualRentalIncome * 10000) / property.totalValue;
    }
}
```

---

## 컴플라이언스 모듈

### 전송 제한 규칙

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "./interfaces/ICompliance.sol";
import "./interfaces/IIdentityRegistry.sol";

/**
 * @title 컴플라이언스모듈
 * @dev 증권 토큰을 위한 모듈형 컴플라이언스 규칙
 */
contract 컴플라이언스모듈 is ICompliance {
    IIdentityRegistry public identityRegistry;

    // 최대 투자자 수
    uint256 public maxInvestors;
    uint256 public currentInvestors;

    // 투자자 국가 제한
    mapping(uint16 => bool) public countryBlocked;

    // 최대 소유 비율 (베이시스 포인트)
    uint256 public maxOwnershipPercentage;

    // 락업 기간
    mapping(address => uint256) public lockupEndTime;

    // 일일 전송 한도
    mapping(address => uint256) public dailyTransferred;
    mapping(address => uint256) public lastTransferDay;
    uint256 public dailyTransferLimit;

    // 적격 투자자 요건
    bool public requireAccreditation;
    uint256 public constant ACCREDITATION_CLAIM_TOPIC = 10;

    event CountryBlockStatusChanged(uint16 country, bool blocked);
    event MaxInvestorsUpdated(uint256 newMax);
    event LockupSet(address indexed investor, uint256 endTime);

    constructor(address _identityRegistry) {
        identityRegistry = IIdentityRegistry(_identityRegistry);
    }

    /**
     * @dev 전송이 규정을 준수하는지 확인
     */
    function canTransfer(
        address from,
        address to,
        uint256 amount
    ) external view override returns (bool) {
        // 투자자 수 한도 확인
        if (balanceOf(to) == 0 && currentInvestors >= maxInvestors) {
            return false;
        }

        // 국가 제한 확인
        uint16 toCountry = identityRegistry.investorCountry(to);
        if (countryBlocked[toCountry]) {
            return false;
        }

        // 락업 기간 확인
        if (lockupEndTime[from] > block.timestamp) {
            return false;
        }

        // 최대 소유 확인
        if (maxOwnershipPercentage > 0) {
            uint256 newBalance = balanceOf(to) + amount;
            uint256 ownershipBps = (newBalance * 10000) / totalSupply();
            if (ownershipBps > maxOwnershipPercentage) {
                return false;
            }
        }

        // 일일 전송 한도 확인
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
     * @dev 국가 차단 또는 차단 해제
     */
    function setCountryBlocked(
        uint16 country,
        bool blocked
    ) external onlyOwner {
        countryBlocked[country] = blocked;
        emit CountryBlockStatusChanged(country, blocked);
    }

    /**
     * @dev 투자자 락업 기간 설정
     */
    function setLockup(
        address investor,
        uint256 endTime
    ) external onlyOwner {
        lockupEndTime[investor] = endTime;
        emit LockupSet(investor, endTime);
    }

    /**
     * @dev 최대 투자자 수 설정
     */
    function setMaxInvestors(uint256 _max) external onlyOwner {
        maxInvestors = _max;
        emit MaxInvestorsUpdated(_max);
    }

    // 플레이스홀더 함수 - 실제 구현은 토큰 컨트랙트에 따라 다름
    function balanceOf(address) internal view returns (uint256) { return 0; }
    function totalSupply() internal view returns (uint256) { return 0; }
    modifier onlyOwner() { _; }
}
```

---

## 분할 소유권

### 분할화 프로토콜

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/token/ERC721/IERC721.sol";
import "@openzeppelin/contracts/token/ERC20/ERC20.sol";
import "@openzeppelin/contracts/security/ReentrancyGuard.sol";

/**
 * @title 분할화볼트
 * @dev NFT를 대체 가능 토큰으로 분할화
 */
contract 분할화볼트 is ERC20, ReentrancyGuard {
    IERC721 public nftContract;
    uint256 public tokenId;

    // 볼트 상태
    enum State { Inactive, Active, Redeemed }
    State public state;

    // 바이아웃 경매 매개변수
    uint256 public reservePrice;
    uint256 public auctionEndTime;
    address public highestBidder;
    uint256 public highestBid;

    // 큐레이터 (원래 분할화한 사람)
    address public curator;
    uint256 public curatorFee; // 베이시스 포인트

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

        // 큐레이터에게 분할 발행
        _mint(msg.sender, _totalSupply);

        emit Fractionalized(msg.sender, _totalSupply);
    }

    /**
     * @dev NFT 예치로 볼트 활성화
     */
    function activate() external {
        require(state == State.Inactive, "이미 활성화됨");
        require(msg.sender == curator, "큐레이터만 가능");

        nftContract.transferFrom(msg.sender, address(this), tokenId);
        state = State.Active;
    }

    /**
     * @dev 바이아웃 경매 시작
     */
    function startAuction() external payable nonReentrant {
        require(state == State.Active, "볼트가 활성화되지 않음");
        require(msg.value >= reservePrice, "예약가 미만");
        require(auctionEndTime == 0, "경매가 이미 시작됨");

        auctionEndTime = block.timestamp + 7 days;
        highestBidder = msg.sender;
        highestBid = msg.value;

        emit AuctionStarted(reservePrice, auctionEndTime);
        emit BidPlaced(msg.sender, msg.value);
    }

    /**
     * @dev 더 높은 입찰
     */
    function bid() external payable nonReentrant {
        require(auctionEndTime > 0, "경매 없음");
        require(block.timestamp < auctionEndTime, "경매 종료");
        require(msg.value > highestBid * 105 / 100, "입찰가가 5% 이상 높아야 함");

        // 이전 입찰자에게 환불
        if (highestBidder != address(0)) {
            payable(highestBidder).transfer(highestBid);
        }

        highestBidder = msg.sender;
        highestBid = msg.value;

        // 마지막 15분 내 입찰시 연장
        if (auctionEndTime - block.timestamp < 15 minutes) {
            auctionEndTime += 15 minutes;
        }

        emit BidPlaced(msg.sender, msg.value);
    }

    /**
     * @dev 경매 종료 및 NFT를 낙찰자에게 전송
     */
    function endAuction() external nonReentrant {
        require(auctionEndTime > 0, "경매 없음");
        require(block.timestamp >= auctionEndTime, "경매가 종료되지 않음");

        state = State.Redeemed;

        // 낙찰자에게 NFT 전송
        nftContract.transferFrom(address(this), highestBidder, tokenId);

        // 큐레이터 수수료 계산
        uint256 fee = (highestBid * curatorFee) / 10000;
        payable(curator).transfer(fee);

        emit AuctionWon(highestBidder, highestBid);
    }

    /**
     * @dev 경매 후 분할 토큰으로 ETH 상환
     */
    function redeem() external nonReentrant {
        require(state == State.Redeemed, "상환되지 않음");

        uint256 balance = balanceOf(msg.sender);
        require(balance > 0, "상환할 토큰 없음");

        // 수익 지분 계산
        uint256 fee = (highestBid * curatorFee) / 10000;
        uint256 proceeds = highestBid - fee;
        uint256 share = (proceeds * balance) / totalSupply();

        _burn(msg.sender, balance);
        payable(msg.sender).transfer(share);

        emit Redeemed(msg.sender);
    }
}
```

---

## 크로스체인 토큰화

### 멀티체인 토큰 브릿지

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/token/ERC20/ERC20.sol";
import "@openzeppelin/contracts/security/Pausable.sol";

/**
 * @title 브릿지가능토큰
 * @dev 크로스체인 브릿징 기능이 있는 토큰
 */
contract 브릿지가능토큰 is ERC20, Pausable {
    // 발행/소각 권한이 있는 브릿지 컨트랙트
    mapping(address => bool) public bridges;

    // 체인별 공급량 추적
    mapping(uint256 => uint256) public chainSupply;
    uint256 public constant HOME_CHAIN_ID = 1; // 이더리움 메인넷

    // 크로스체인 전송 추적
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
        require(bridges[msg.sender], "권한 없는 브릿지");
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
     * @dev 크로스체인 전송 시작
     */
    function bridgeTransfer(
        address to,
        uint256 amount,
        uint256 destinationChain
    ) external whenNotPaused {
        require(to != address(0), "유효하지 않은 수신자");
        require(amount > 0, "금액이 0");

        // 고유 전송 ID 생성
        bytes32 transferId = keccak256(abi.encodePacked(
            msg.sender,
            to,
            amount,
            destinationChain,
            block.timestamp,
            block.number
        ));

        // 소스 체인에서 토큰 소각
        _burn(msg.sender, amount);
        chainSupply[block.chainid] -= amount;

        emit CrossChainTransfer(msg.sender, to, amount, destinationChain, transferId);
    }

    /**
     * @dev 크로스체인 전송 수신 (브릿지가 호출)
     */
    function bridgeReceive(
        address to,
        uint256 amount,
        uint256 sourceChain,
        bytes32 transferId
    ) external onlyBridge whenNotPaused {
        require(!processedTransfers[transferId], "이미 처리됨");

        processedTransfers[transferId] = true;

        // 대상 체인에서 토큰 발행
        _mint(to, amount);
        chainSupply[block.chainid] += amount;

        emit CrossChainReceive(to, amount, sourceChain, transferId);
    }

    /**
     * @dev 권한 있는 브릿지 추가
     */
    function addBridge(address bridge) external onlyOwner {
        bridges[bridge] = true;
        emit BridgeAdded(bridge);
    }

    modifier onlyOwner() {
        // 소유자 확인 구현
        _;
    }
}
```

---

## 핵심 내용

1. **ERC-3643**은 내장된 전송 제한 및 신원 검증이 있는 준수 증권 토큰의 표준입니다
2. **신원 레지스트리**는 지갑 주소를 검증된 신원에 연결하여 KYC/AML 준수를 가능하게 합니다
3. **RWA 토큰화**는 부동산, 국채 및 기타 전통 자산을 온체인으로 가져옵니다
4. **컴플라이언스 모듈**은 투자자 한도, 국가 제한, 락업 및 소유 한도를 시행합니다
5. **분할화**는 내장된 바이아웃 메커니즘으로 고가 자산의 공동 소유를 가능하게 합니다
6. **크로스체인 브릿지**는 토큰화 자산이 블록체인 네트워크 간에 이동할 수 있게 합니다

## 복습 문제

1. ERC-20과 ERC-3643 토큰 표준의 주요 차이점은 무엇입니까?
2. 신원 레지스트리가 준수 토큰 전송을 어떻게 가능하게 합니까?
3. 스마트 컨트랙트를 통해 어떤 컴플라이언스 규칙을 시행할 수 있습니까?
4. 토큰화 부동산에서 분할 소유권은 어떻게 작동합니까?
5. 크로스체인 토큰 브릿징을 가능하게 하는 메커니즘은 무엇입니까?
6. 토큰화 부동산에서 배당은 어떻게 분배됩니까?

---

**다음 장 미리보기:** 제4장에서는 자동화 마켓 메이커, 대출 프로토콜 및 파생상품 플랫폼을 포함한 DeFi 프로토콜을 심층적으로 탐구합니다.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · 널리 인간을 이롭게 · 금융을 민주화

