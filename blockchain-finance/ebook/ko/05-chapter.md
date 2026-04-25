# 제5장: 스마트 컨트랙트 보안

## 감사, 형식 검증 및 취약점 방지

### 수십억 달러의 온체인 가치 보호

---

## 개요

스마트 컨트랙트 보안은 블록체인 금융에서 가장 중요합니다. 수십억 달러가 위험에 처해 있어, 사소한 취약점조차도 치명적인 손실로 이어질 수 있습니다. 이 장에서는 일반적인 취약점 패턴, 감사 방법론, 형식 검증 기술 및 사고 대응 절차를 포함한 포괄적인 보안 관행을 다룹니다.

---

## 일반적인 취약점 패턴

### 취약점 분류

**OWASP 스마트 컨트랙트 Top 10:**

| 순위 | 취약점 | 영향 | 빈도 |
|------|--------|------|------|
| 1 | 재진입 | 치명적 | 높음 |
| 2 | 접근 제어 | 치명적 | 높음 |
| 3 | 산술 문제 | 높음 | 중간 |
| 4 | 검사되지 않은 반환 값 | 중간 | 높음 |
| 5 | 서비스 거부 | 높음 | 중간 |
| 6 | 프런트러닝 | 중간 | 높음 |
| 7 | 오라클 조작 | 치명적 | 중간 |
| 8 | 로직 오류 | 치명적 | 중간 |
| 9 | 타임스탬프 의존성 | 낮음 | 중간 |
| 10 | 가스 그리핑 | 중간 | 낮음 |

### 재진입 공격

**고전적 재진입:**

```solidity
// 취약한 코드 - 사용하지 마세요
contract 취약한볼트 {
    mapping(address => uint256) public balances;

    function withdraw(uint256 amount) external {
        require(balances[msg.sender] >= amount, "잔액 부족");

        // 취약점: 상태 업데이트 전 외부 호출
        (bool success, ) = msg.sender.call{value: amount}("");
        require(success, "전송 실패");

        // 외부 호출 후 상태 업데이트 - 재진입 가능
        balances[msg.sender] -= amount;
    }
}

// 공격 컨트랙트
contract 재진입공격자 {
    취약한볼트 public vault;
    uint256 public attackCount;

    constructor(address _vault) {
        vault = 취약한볼트(_vault);
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

**안전한 구현:**

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/security/ReentrancyGuard.sol";

/**
 * @title 안전한볼트
 * @dev 재진입 보호가 있는 볼트
 */
contract 안전한볼트 is ReentrancyGuard {
    mapping(address => uint256) public balances;

    event Deposit(address indexed user, uint256 amount);
    event Withdrawal(address indexed user, uint256 amount);

    function deposit() external payable {
        balances[msg.sender] += msg.value;
        emit Deposit(msg.sender, msg.value);
    }

    function withdraw(uint256 amount) external nonReentrant {
        require(balances[msg.sender] >= amount, "잔액 부족");

        // Checks-Effects-Interactions 패턴
        // 1. CHECKS - 위에서 이미 완료
        // 2. EFFECTS - 외부 호출 전에 상태 업데이트
        balances[msg.sender] -= amount;

        // 3. INTERACTIONS - 외부 호출 마지막
        (bool success, ) = msg.sender.call{value: amount}("");
        require(success, "전송 실패");

        emit Withdrawal(msg.sender, amount);
    }

    function getBalance() external view returns (uint256) {
        return balances[msg.sender];
    }
}
```

### 접근 제어 취약점

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/access/AccessControl.sol";
import "@openzeppelin/contracts/access/Ownable2Step.sol";

/**
 * @title 안전한접근제어
 * @dev 포괄적인 접근 제어 패턴
 */
contract 안전한접근제어 is AccessControl, Ownable2Step {
    bytes32 public constant ADMIN_ROLE = keccak256("ADMIN_ROLE");
    bytes32 public constant OPERATOR_ROLE = keccak256("OPERATOR_ROLE");
    bytes32 public constant PAUSER_ROLE = keccak256("PAUSER_ROLE");

    // 시간 잠금 작업
    mapping(bytes32 => uint256) public pendingOperations;
    uint256 public constant TIMELOCK_DELAY = 2 days;

    // 다중 서명 요건
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
     * @dev 시간 잠금 작업 예약
     */
    function scheduleOperation(bytes32 operationId) external onlyRole(ADMIN_ROLE) {
        require(pendingOperations[operationId] == 0, "이미 예약됨");
        pendingOperations[operationId] = block.timestamp + TIMELOCK_DELAY;
        emit OperationScheduled(operationId, pendingOperations[operationId]);
    }

    /**
     * @dev 대기 중인 작업 승인 (다중 서명)
     */
    function approveOperation(bytes32 operationId) external onlyRole(ADMIN_ROLE) {
        require(pendingOperations[operationId] != 0, "예약되지 않음");
        require(!approvals[operationId][msg.sender], "이미 승인됨");

        approvals[operationId][msg.sender] = true;
        emit OperationApproved(operationId, msg.sender);
    }

    /**
     * @dev 시간 잠금 작업 실행
     */
    function executeOperation(bytes32 operationId) external onlyRole(ADMIN_ROLE) {
        require(pendingOperations[operationId] != 0, "예약되지 않음");
        require(
            block.timestamp >= pendingOperations[operationId],
            "시간 잠금이 만료되지 않음"
        );
        require(_countApprovals(operationId) >= requiredApprovals, "승인 부족");

        pendingOperations[operationId] = 0;
        emit OperationExecuted(operationId);
    }

    /**
     * @dev 작업에 대한 승인 수 계산
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
     * @dev 실수로 인한 잠금 방지를 위한 renounceRole 오버라이드
     */
    function renounceRole(bytes32 role, address account) public override {
        require(
            role != DEFAULT_ADMIN_ROLE || getRoleMemberCount(DEFAULT_ADMIN_ROLE) > 1,
            "마지막 관리자를 포기할 수 없음"
        );
        super.renounceRole(role, account);
    }
}
```

### 오라클 조작

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

/**
 * @title 오라클보안패턴
 * @dev 오라클 조작 방지 패턴
 */
contract 오라클보안패턴 {
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
     * @dev 시간 가중 평균 가격 조회
     */
    function getTWAP(address asset) public view returns (uint256) {
        PriceData[] storage history = priceHistory[asset];
        require(history.length >= MIN_OBSERVATIONS, "데이터 부족");

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

        require(totalWeight > 0, "최근 데이터 없음");
        return weightedSum / totalWeight;
    }

    /**
     * @dev 편차 임계값에 대한 가격 업데이트 검증
     */
    function validatePriceUpdate(
        address asset,
        uint256 newPrice
    ) public view returns (bool) {
        PriceData[] storage history = priceHistory[asset];
        if (history.length == 0) return true;

        uint256 lastPrice = history[history.length - 1].price;

        // 편차 비율 계산
        uint256 deviation;
        if (newPrice > lastPrice) {
            deviation = ((newPrice - lastPrice) * 100) / lastPrice;
        } else {
            deviation = ((lastPrice - newPrice) * 100) / lastPrice;
        }

        return deviation <= MAX_PRICE_DEVIATION;
    }

    /**
     * @dev 다중 오라클 가격 집계
     */
    function aggregateOraclePrices(
        uint256[] memory prices,
        uint256 threshold
    ) public pure returns (uint256) {
        require(prices.length >= 3, "최소 3개 오라클 필요");

        // 가격 정렬
        for (uint256 i = 0; i < prices.length - 1; i++) {
            for (uint256 j = i + 1; j < prices.length; j++) {
                if (prices[j] < prices[i]) {
                    (prices[i], prices[j]) = (prices[j], prices[i]);
                }
            }
        }

        // 중앙값 사용
        uint256 median = prices[prices.length / 2];

        // 모든 가격이 중앙값의 임계값 내에 있는지 검증
        for (uint256 i = 0; i < prices.length; i++) {
            uint256 deviation;
            if (prices[i] > median) {
                deviation = ((prices[i] - median) * 100) / median;
            } else {
                deviation = ((median - prices[i]) * 100) / median;
            }
            require(deviation <= threshold, "오라클 편차가 너무 큼");
        }

        return median;
    }
}
```

---

## 감사 방법론

### 감사 프로세스

**WIA 감사 프레임워크:**

| 단계 | 활동 | 산출물 |
|------|------|--------|
| 1. 범위 지정 | 요구사항 수집, 코드베이스 검토 | 감사 범위 문서 |
| 2. 자동화 분석 | 정적 분석, 퍼징, 기호 실행 | 도구 보고서 |
| 3. 수동 검토 | 라인별 코드 검토, 로직 분석 | 발견 목록 |
| 4. 취약점 테스트 | 익스플로잇 개발, 엣지 케이스 테스트 | PoC 익스플로잇 |
| 5. 보고 | 발견 문서화, 심각도 평가 | 감사 보고서 |
| 6. 수정 | 수정 검토, 재테스트 | 최종 보고서 |

### 보안 체크리스트

```markdown
## WIA 스마트 컨트랙트 보안 체크리스트

### 접근 제어
- [ ] 모든 민감한 함수에 적절한 접근 수정자가 있음
- [ ] 소유자/관리자가 포기하여 컨트랙트가 잠기지 않음
- [ ] 역할 기반 접근 제어가 최소 권한 원칙 사용
- [ ] 중요 작업에 다중 서명 필요
- [ ] 매개변수 변경에 시간 잠금 구현

### 재진입
- [ ] 외부 호출이 checks-effects-interactions 패턴 따름
- [ ] 모든 상태 변경 함수에 ReentrancyGuard 사용
- [ ] 교차 함수 재진입 고려
- [ ] 교차 컨트랙트 재진입 분석

### 산술
- [ ] Solidity 0.8+ 사용 (내장 오버플로 검사)
- [ ] unchecked 블록 정당화 및 검토
- [ ] 0으로 나누기 방지
- [ ] 반올림 방향이 사용 사례에 적절
- [ ] 계산에서 정밀도 손실 허용

### 외부 호출
- [ ] 모든 외부 호출의 반환 값 검사
- [ ] 저수준 호출(.call)에 반환 값 검사
- [ ] 수신 컨트랙트에 가스 배당 적절
- [ ] fallback/receive 함수가 예상치 못한 호출 처리

### 오라클 보안
- [ ] 현물 가격 대신 TWAP 또는 시간 지연 사용
- [ ] 여러 오라클 소스 집계
- [ ] 오래된 데이터 검사 구현
- [ ] 가격 편차 임계값 적용
- [ ] 조작 저항성 테스트
```

### 자동화 도구

**보안 도구 모음:**

| 도구 | 유형 | 목적 |
|------|------|------|
| Slither | 정적 분석 | 버그 탐지, 코드 품질 |
| Mythril | 기호 실행 | 취약점 탐지 |
| Echidna | 퍼저 | 속성 테스트 |
| Foundry | 테스팅 프레임워크 | 단위/통합 테스트 |
| Certora | 형식 검증 | 수학적 증명 |
| 4naly3er | 정적 분석 | 가스 최적화 |

---

## 형식 검증

### Certora Prover

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

/**
 * @title 검증가능볼트
 * @dev 형식 검증 스펙이 있는 볼트 컨트랙트
 */
contract 검증가능볼트 {
    mapping(address => uint256) public balances;
    uint256 public totalDeposits;

    // 불변량: totalDeposits == 모든 잔액의 합
    // Certora로 검증

    function deposit() external payable {
        balances[msg.sender] += msg.value;
        totalDeposits += msg.value;
    }

    function withdraw(uint256 amount) external {
        require(balances[msg.sender] >= amount, "부족");
        balances[msg.sender] -= amount;
        totalDeposits -= amount;
        payable(msg.sender).transfer(amount);
    }

    function transfer(address to, uint256 amount) external {
        require(balances[msg.sender] >= amount, "부족");
        balances[msg.sender] -= amount;
        balances[to] += amount;
        // totalDeposits 변경 없음 - 내부 전송
    }
}
```

**Certora 스펙:**

```cvl
// 검증가능볼트.spec
methods {
    function balances(address) external returns (uint256) envfree;
    function totalDeposits() external returns (uint256) envfree;
    function deposit() external payable;
    function withdraw(uint256) external;
    function transfer(address, uint256) external;
}

// 잔액 합계를 추적하는 고스트 변수
ghost mathint sumBalances {
    init_state axiom sumBalances == 0;
}

// 잔액 변경 시 고스트 업데이트
hook Sstore balances[KEY address a] uint256 newValue (uint256 oldValue) STORAGE {
    sumBalances = sumBalances + newValue - oldValue;
}

// 불변량: totalDeposits는 모든 잔액의 합과 같음
invariant totalEqualsSum()
    to_mathint(totalDeposits()) == sumBalances;

// 규칙: deposit은 msg.value만큼 잔액 증가
rule depositIncreasesBalance(env e) {
    uint256 balanceBefore = balances(e.msg.sender);
    uint256 depositAmount = e.msg.value;

    deposit(e);

    uint256 balanceAfter = balances(e.msg.sender);
    assert balanceAfter == balanceBefore + depositAmount;
}

// 규칙: 무단 출금 없음
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

## 사고 대응

### 보안 모니터링

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/security/Pausable.sol";
import "@openzeppelin/contracts/access/AccessControl.sol";

/**
 * @title 비상대응
 * @dev 포괄적인 비상 제어가 있는 컨트랙트
 */
contract 비상대응 is Pausable, AccessControl {
    bytes32 public constant GUARDIAN_ROLE = keccak256("GUARDIAN_ROLE");
    bytes32 public constant EMERGENCY_ROLE = keccak256("EMERGENCY_ROLE");

    // 회로 차단기 임계값
    uint256 public maxSingleWithdrawal;
    uint256 public dailyWithdrawalLimit;
    uint256 public cooldownPeriod;

    // 출금 추적
    mapping(address => uint256) public dailyWithdrawn;
    mapping(address => uint256) public lastWithdrawalDay;
    uint256 public totalDailyWithdrawn;
    uint256 public lastResetDay;

    // 비상 셧다운
    bool public emergencyShutdown;
    uint256 public emergencyShutdownTime;

    event EmergencyShutdownActivated(address indexed guardian);
    event CircuitBreakerTriggered(string reason);
    event EmergencyWithdrawal(address indexed user, uint256 amount);

    modifier notShutdown() {
        require(!emergencyShutdown, "비상 셧다운 활성화");
        _;
    }

    modifier circuitBreaker(uint256 amount) {
        // 단일 출금 한도 확인
        require(amount <= maxSingleWithdrawal, "단일 한도 초과");

        // 일일 한도 확인
        uint256 currentDay = block.timestamp / 1 days;
        if (currentDay > lastResetDay) {
            totalDailyWithdrawn = 0;
            lastResetDay = currentDay;
        }

        require(
            totalDailyWithdrawn + amount <= dailyWithdrawalLimit,
            "일일 한도 초과"
        );

        _;

        totalDailyWithdrawn += amount;
    }

    /**
     * @dev 비상 셧다운 활성화
     */
    function activateEmergencyShutdown() external onlyRole(GUARDIAN_ROLE) {
        emergencyShutdown = true;
        emergencyShutdownTime = block.timestamp;
        _pause();
        emit EmergencyShutdownActivated(msg.sender);
    }

    /**
     * @dev 비상 셧다운 비활성화 (시간 잠금 필요)
     */
    function deactivateEmergencyShutdown() external onlyRole(DEFAULT_ADMIN_ROLE) {
        require(emergencyShutdown, "셧다운 상태가 아님");
        require(
            block.timestamp >= emergencyShutdownTime + 7 days,
            "쿨다운 경과하지 않음"
        );
        emergencyShutdown = false;
        _unpause();
    }
}
```

### 버그 바운티 프로그램

**WIA 버그 바운티 티어:**

| 심각도 | 영향 | 보상 범위 |
|--------|------|----------|
| 치명적 | 직접적인 자금 도난, 완전한 프로토콜 장악 | $10만 - $50만 |
| 높음 | 자금 동결, 상당한 경제적 손해 | $2.5만 - $10만 |
| 중간 | 제한된 영향, 부분적 기능 손실 | $5천 - $2.5만 |
| 낮음 | 사소한 문제, 정보성 | $500 - $5천 |

---

## 핵심 내용

1. **재진입**은 가장 치명적인 취약점으로 남아 있음 - 항상 checks-effects-interactions 패턴 사용
2. **접근 제어**는 시간 잠금 및 다중 서명이 있는 다계층 방어 필요
3. **오라클 조작**은 TWAP, 여러 소스 및 편차 검사로 완화 가능
4. **형식 검증**은 중요한 불변량에 대한 수학적 보장 제공
5. **사고 대응**은 사전 계획된 비상 절차 및 회로 차단기 필요
6. **지속적인 모니터링** 및 버그 바운티는 지속적인 보안에 필수

## 복습 문제

1. checks-effects-interactions 패턴은 무엇이며 왜 중요합니까?
2. 오라클 조작 공격은 어떻게 방지할 수 있습니까?
3. 스마트 컨트랙트 보안 감사의 주요 단계는 무엇입니까?
4. 형식 검증은 전통적인 테스트와 어떻게 다릅니까?
5. 비상 대응 시스템에는 어떤 구성 요소가 포함되어야 합니까?
6. DeFi 프로토콜에 대한 버그 바운티 프로그램은 어떻게 구성해야 합니까?

---

**다음 장 미리보기:** 제6장에서는 글로벌 규정, 컴플라이언스 요건 및 법적 구조화를 포함한 블록체인 금융의 규제 프레임워크를 탐구합니다.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · 널리 인간을 이롭게 · 금융을 민주화

