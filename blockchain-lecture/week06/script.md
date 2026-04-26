# 6주차 강의 스크립트: 스마트 계약 심화

---

## 강의 정보

| 항목 | 내용 |
|------|------|
| **과목명** | 블록체인 기초 및 응용 |
| **담당교수** | 연삼흠 (대전대학교) |
| **주차** | 6주차 |
| **주제** | 스마트 계약 심화 (Advanced Smart Contract) |
| **WIA 표준** | WIA-FIN-007 (Smart Contract) |
| **시뮬레이터** | https://wiastandards.com/smart-contract/simulator |
| **강의시간** | 총 90분 (전반 40분 + 휴식 15분 + 후반 35분) |

---

## Part 1: 이론 (30분)

### 1-1. 도입 및 지난 주 복습 (5분)

**교수 멘트:**

> 안녕하세요, 여러분. 연삼흠입니다. 지난 주에 Solidity 기초와 Hello World 계약을 만들어 봤죠? 과제로 `require`를 사용해서 소유자만 인사말을 변경할 수 있게 해보라고 했는데, 해보신 분 손 들어보세요.
>
> 좋습니다. 오늘은 그 `require`를 포함해서 스마트 계약의 심화 내용을 다룹니다. 조건문, 반복문, 이벤트, modifier, mapping, 그리고 실전에서 정말 중요한 **가스 최적화**까지 갑니다. 오늘 수업이 끝나면 여러분이 직접 **투표 DApp**을 만들 수 있게 됩니다.

---

### 1-2. 조건문과 반복문 (8분)

**교수 멘트:**

> 먼저 Solidity의 조건문과 반복문을 봅시다. 다른 프로그래밍 언어와 거의 같은데, 한 가지 큰 차이가 있어요. 뭘까요? 바로 **가스**입니다.

#### 조건문

```solidity
// require: 조건 불충족 시 트랜잭션 취소 + 가스 환불
require(msg.sender == owner, "소유자만 호출 가능합니다");

// if-else: 일반적인 조건 분기
if (balance > 100) {
    // 처리 A
} else {
    // 처리 B
}

// assert: 내부 오류 검증 (절대 발생하면 안 되는 조건)
assert(totalSupply >= 0);

// revert: 명시적 트랜잭션 취소
if (amount == 0) {
    revert("금액은 0보다 커야 합니다");
}
```

**교수 멘트:**

> 여기서 `require`와 `assert`의 차이를 꼭 알아두세요. `require`는 사용자 입력 검증에 쓰입니다. "이 조건을 만족해야 실행해 줄게"라는 의미예요. 실패하면 남은 가스를 환불해 줍니다.
>
> 반면에 `assert`는 "이건 절대 발생하면 안 되는 상황"을 체크합니다. assert가 실패하면 심각한 버그가 있다는 뜻이에요. 남은 가스도 환불하지 않습니다. 그래서 사용자 입력 검증에는 `require`, 내부 불변성 검증에는 `assert`를 씁니다.

#### 반복문

```solidity
// for 반복문
for (uint i = 0; i < 10; i++) {
    // 처리
}

// while 반복문
uint count = 0;
while (count < 5) {
    count++;
}
```

**교수 멘트:**

> 반복문을 쓸 때 주의할 점! 블록체인에서 반복문은 위험합니다. 왜냐하면 반복 횟수만큼 가스가 소모되거든요. 만약 배열에 100만 개의 요소가 있는데 전부 순회하면? 가스 한도를 초과해서 트랜잭션이 실패합니다.
>
> 그래서 스마트 계약에서는 반복문 사용을 최소화하고, 가능하면 `mapping`을 활용합니다. 이게 WIA-FIN-007에서도 권장하는 방식이에요.

---

### 1-3. 이벤트와 로깅 (5분)

**교수 멘트:**

> 이벤트는 지난 주에 잠깐 봤는데, 오늘 좀 더 자세히 다루겠습니다. 이벤트는 스마트 계약에서 발생한 일을 로그로 기록하는 겁니다.

```solidity
// 이벤트 선언
event Transfer(address indexed from, address indexed to, uint256 amount);
event VoteCast(address indexed voter, uint256 indexed candidateId, uint256 timestamp);

// 이벤트 발생
emit Transfer(msg.sender, _to, _amount);
emit VoteCast(msg.sender, _candidateId, block.timestamp);
```

**교수 멘트:**

> `indexed` 키워드가 중요합니다. `indexed`를 붙이면 그 필드로 이벤트를 검색할 수 있어요. 예를 들어 "내 주소에서 발생한 모든 Transfer 이벤트"를 빠르게 찾을 수 있죠. 최대 3개까지 `indexed`를 붙일 수 있습니다.
>
> 이벤트는 블록체인에 로그로 저장되는데, 상태 변수에 저장하는 것보다 가스비가 훨씬 저렴합니다. 프론트엔드 앱에서 이 이벤트를 실시간으로 감지해서 UI를 업데이트할 수 있고요.

---

### 1-4. Modifier와 접근 제어 (5분)

**교수 멘트:**

> 자, 지난 주 과제에서 `require(msg.sender == owner)`를 매번 함수에 쓰셨죠? 그런데 이 조건을 10개 함수에 다 넣어야 한다면? 코드가 반복되잖아요. 이걸 해결하는 게 `modifier`입니다.

```solidity
// modifier 정의
modifier onlyOwner() {
    require(msg.sender == owner, "소유자만 호출 가능합니다");
    _; // 원래 함수 코드가 이 위치에서 실행됨
}

modifier validAmount(uint _amount) {
    require(_amount > 0, "금액은 0보다 커야 합니다");
    _;
}

// modifier 사용
function withdraw(uint _amount) public onlyOwner validAmount(_amount) {
    // 소유자만 호출 가능, 금액은 0 초과
    payable(msg.sender).transfer(_amount);
}
```

**교수 멘트:**

> `modifier`는 함수 실행 전에 조건을 체크하는 재사용 가능한 코드 조각입니다. `_`(언더스코어)가 있는 위치에서 원래 함수의 코드가 실행돼요.
>
> `onlyOwner` modifier를 한 번 정의해 놓으면, 관리자만 사용할 수 있는 함수에 간단히 `onlyOwner`를 붙이면 됩니다. 코드 중복도 줄고, 보안 검사도 빠뜨리지 않을 수 있어요. WIA-FIN-007에서도 접근 제어에 modifier 사용을 적극 권장하고 있습니다.

---

### 1-5. Mapping과 가스 최적화 (7분)

**교수 멘트:**

> 마지막으로 mapping과 가스 최적화입니다. 이건 실전에서 가장 중요한 부분이에요.

#### Mapping 심화

```solidity
// 기본 mapping
mapping(address => uint256) public balances;

// 중첩 mapping
mapping(address => mapping(address => uint256)) public allowance;

// struct와 mapping 조합
struct Voter {
    bool hasVoted;
    uint256 candidateId;
    uint256 weight;
}
mapping(address => Voter) public voters;
```

**교수 멘트:**

> `mapping`은 해시 테이블입니다. 키를 넣으면 값을 O(1)으로 가져올 수 있어요. 배열처럼 순회할 필요가 없죠. 그래서 가스 효율이 훨씬 좋습니다.
>
> 중첩 mapping도 많이 씁니다. "A 주소가 B 주소에게 얼마만큼의 토큰 사용을 허락했는가"를 표현할 때 `mapping(address => mapping(address => uint256))`을 사용하죠. ERC-20 토큰에서 꼭 나오는 패턴입니다.

#### 가스 최적화 기법

| 기법 | 설명 | 절약 효과 |
|------|------|-----------|
| 변수 패킹 | uint8, uint16 등 작은 타입을 연속 선언 | 스토리지 슬롯 절약 |
| 배열 대신 mapping | 순회가 불필요한 경우 mapping 사용 | 조회 가스 절약 |
| view/pure 활용 | 읽기 전용 함수에 view/pure 명시 | 호출 가스 무료 |
| 이벤트 활용 | 영구 저장 불필요한 데이터는 이벤트로 기록 | 스토리지 가스 절약 |
| calldata 사용 | external 함수의 매개변수에 calldata 사용 | memory 대비 절약 |
| 짧은 에러 메시지 | require 에러 메시지 짧게 작성 | 배포 가스 절약 |

**교수 멘트:**

> 이더리움 메인넷에서는 가스비가 진짜 비쌉니다. 하나의 트랜잭션에 수만 원이 들 수도 있어요. 그래서 가스 최적화는 "있으면 좋은 것"이 아니라 "반드시 해야 하는 것"입니다.
>
> 특히 변수 패킹 기법은 알아두세요. 이더리움 EVM은 32바이트(256비트) 단위로 스토리지를 읽고 씁니다. uint8 변수 3개를 연속으로 선언하면 하나의 슬롯에 들어가지만, 사이에 uint256을 끼우면 슬롯이 분리되어 가스가 더 들어요.

---

## Part 2: 실습 (60분)

### 2-1. 투표 DApp 설계 (10분)

**교수 멘트:**

> 자, 이제 오늘의 메인 실습입니다. **투표 DApp**을 만들어 봅시다! 탈중앙화된 투표 시스템이에요. 누가 투표했는지, 결과가 어떤지 모두 블록체인에 기록되니까 조작이 불가능합니다.

#### 설계 요구사항

1. 관리자가 후보자를 등록할 수 있다
2. 유권자는 한 번만 투표할 수 있다
3. 누구나 투표 결과를 확인할 수 있다
4. 투표 시 이벤트가 발생한다

---

### 2-2. 투표 계약 코드 작성 (25분)

**교수 멘트:**

> Remix IDE를 열고 `contracts` 폴더에 `Voting.sol` 파일을 새로 만드세요. 같이 코드를 작성해 봅시다.

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

/// @title WIA-FIN-007 표준 준수 투표 계약
/// @author 대전대학교 블록체인 강의
/// @notice 간단한 탈중앙화 투표 시스템
contract Voting {
    // ===== 상태 변수 =====

    // 관리자 주소
    address public admin;

    // 후보자 구조체
    struct Candidate {
        uint256 id;
        string name;
        uint256 voteCount;
    }

    // 유권자 구조체
    struct Voter {
        bool hasVoted;
        uint256 votedCandidateId;
    }

    // 후보자 mapping (id => Candidate)
    mapping(uint256 => Candidate) public candidates;

    // 유권자 mapping (address => Voter)
    mapping(address => Voter) public voters;

    // 후보자 수
    uint256 public candidateCount;

    // 총 투표 수
    uint256 public totalVotes;

    // 투표 진행 상태
    bool public votingOpen;

    // ===== 이벤트 =====
    event CandidateAdded(uint256 indexed candidateId, string name);
    event VoteCast(address indexed voter, uint256 indexed candidateId);
    event VotingStatusChanged(bool isOpen);

    // ===== Modifier =====
    modifier onlyAdmin() {
        require(msg.sender == admin, "관리자만 호출 가능합니다");
        _;
    }

    modifier votingIsOpen() {
        require(votingOpen, "투표가 진행 중이 아닙니다");
        _;
    }

    modifier hasNotVoted() {
        require(!voters[msg.sender].hasVoted, "이미 투표하셨습니다");
        _;
    }

    // ===== 생성자 =====
    constructor() {
        admin = msg.sender;
        votingOpen = false;
    }

    // ===== 함수 =====

    /// @notice 후보자 추가 (관리자만)
    function addCandidate(string memory _name) public onlyAdmin {
        require(!votingOpen, "투표 중에는 후보자를 추가할 수 없습니다");
        candidateCount++;
        candidates[candidateCount] = Candidate(candidateCount, _name, 0);
        emit CandidateAdded(candidateCount, _name);
    }

    /// @notice 투표 시작/종료 (관리자만)
    function setVotingStatus(bool _isOpen) public onlyAdmin {
        votingOpen = _isOpen;
        emit VotingStatusChanged(_isOpen);
    }

    /// @notice 투표하기
    function vote(uint256 _candidateId) public votingIsOpen hasNotVoted {
        require(_candidateId > 0 && _candidateId <= candidateCount, "유효하지 않은 후보자입니다");

        voters[msg.sender] = Voter(true, _candidateId);
        candidates[_candidateId].voteCount++;
        totalVotes++;

        emit VoteCast(msg.sender, _candidateId);
    }

    /// @notice 특정 후보자 득표수 조회
    function getVoteCount(uint256 _candidateId) public view returns (uint256) {
        require(_candidateId > 0 && _candidateId <= candidateCount, "유효하지 않은 후보자입니다");
        return candidates[_candidateId].voteCount;
    }

    /// @notice 선두 후보자 조회
    function getLeadingCandidate() public view returns (string memory name, uint256 voteCount) {
        uint256 maxVotes = 0;
        uint256 leadingId = 0;

        for (uint256 i = 1; i <= candidateCount; i++) {
            if (candidates[i].voteCount > maxVotes) {
                maxVotes = candidates[i].voteCount;
                leadingId = i;
            }
        }

        if (leadingId > 0) {
            return (candidates[leadingId].name, candidates[leadingId].voteCount);
        }
        return ("없음", 0);
    }
}
```

**교수 멘트:**

> 코드가 좀 길죠? 하지만 하나씩 보면 어렵지 않습니다. 오늘 배운 내용이 전부 들어있어요.
>
> `struct`로 후보자와 유권자의 데이터를 구조화했고, `mapping`으로 효율적인 데이터 접근을 구현했습니다. `modifier`로 접근 제어를 깔끔하게 처리했고, `event`로 모든 주요 액션을 로깅하고 있어요.
>
> `getLeadingCandidate` 함수에서 반복문을 썼는데, 이건 `view` 함수라서 가스비가 들지 않습니다. 하지만 후보자가 수천 명이면 문제가 될 수 있겠죠? 실제 서비스라면 다른 방식으로 구현해야 합니다.

---

### 2-3. 컴파일, 배포 및 테스트 (15분)

**교수 멘트:**

> 자, 코드 작성이 끝났으면 테스트해 봅시다!

#### 테스트 시나리오

**Step 1: 컴파일 및 배포**
- 컴파일러 0.8.0 이상 선택 → Compile 클릭
- Remix VM 환경에서 Deploy (생성자에 파라미터 없음)

**Step 2: 후보자 등록 (관리자 계정)**
1. `addCandidate` → `"김블록"` 입력 후 실행
2. `addCandidate` → `"이체인"` 입력 후 실행
3. `addCandidate` → `"박노드"` 입력 후 실행
4. `candidateCount`로 3이 반환되는지 확인

**Step 3: 투표 시작**
- `setVotingStatus` → `true` 입력 후 실행

**Step 4: 투표 (계정을 바꿔가며)**
1. Account를 두 번째 주소로 변경 → `vote(1)` 실행 (김블록에게 투표)
2. Account를 세 번째 주소로 변경 → `vote(2)` 실행 (이체인에게 투표)
3. Account를 네 번째 주소로 변경 → `vote(1)` 실행 (김블록에게 투표)

**Step 5: 결과 확인**
- `getVoteCount(1)` → 2 (김블록)
- `getVoteCount(2)` → 1 (이체인)
- `getLeadingCandidate` → ("김블록", 2)
- `totalVotes` → 3

**Step 6: 중복 투표 테스트**
- 이미 투표한 계정으로 다시 `vote` 실행 → 에러 발생 확인: "이미 투표하셨습니다"

**교수 멘트:**

> 에러 메시지 보이시죠? "이미 투표하셨습니다". `hasNotVoted` modifier가 제대로 작동하고 있는 겁니다. 이렇게 블록체인 기반 투표는 한 사람이 두 번 투표하는 것을 구조적으로 방지합니다.

---

### 2-4. WIA 시뮬레이터 실습 및 가스비 비교 (10분)

**교수 멘트:**

> 마지막으로 WIA 시뮬레이터에서 이 계약을 테스트하고, 가스비를 분석해 봅시다.

**Step 7: WIA 시뮬레이터에서 테스트**

1. https://wiastandards.com/smart-contract/simulator 접속
2. 투표 계약 코드 입력 및 배포
3. 동일한 테스트 시나리오 수행
4. 각 함수 호출의 가스 소모량 기록

**가스비 비교 분석:**

| 함수 | 예상 가스 | 비고 |
|------|-----------|------|
| `addCandidate` | ~50,000 | 스토리지 쓰기 (높음) |
| `vote` | ~70,000 | mapping 2개 업데이트 |
| `getVoteCount` | 0 | view 함수 |
| `getLeadingCandidate` | 0 | view 함수 (반복문 포함) |
| `setVotingStatus` | ~28,000 | bool 상태 변경 |

**교수 멘트:**

> 가스 사용량 차이가 보이시죠? 스토리지에 쓰는 함수는 가스가 많이 들고, 읽기만 하는 `view` 함수는 가스가 안 듭니다. 이 차이를 이해하는 게 가스 최적화의 첫 걸음입니다.
>
> 시뮬레이터의 보안 분석 결과도 확인해 보세요. `getLeadingCandidate`의 반복문에 대해 경고가 뜰 수 있습니다. 후보자 수가 많아지면 가스 한도를 초과할 수 있다는 뜻이에요.

---

## Part 3: 정리 (30분)

### 3-1. 핵심 내용 정리 (10분)

**교수 멘트:**

> 오늘 많은 내용을 다뤘습니다. 정리합시다.

#### 오늘 배운 핵심 내용

1. **조건문**: `require` (입력 검증), `assert` (내부 검증), `revert` (명시적 취소)
2. **반복문**: `for`, `while` - 블록체인에서는 가스 주의!
3. **이벤트**: `event` + `emit` - 로깅, `indexed`로 검색 가능
4. **Modifier**: 함수 실행 전 조건 검사, 재사용 가능한 접근 제어
5. **Mapping**: O(1) 접근, struct와 조합, 중첩 mapping
6. **가스 최적화**: 변수 패킹, mapping 활용, view/pure, calldata 사용

**교수 멘트:**

> 오늘 만든 투표 DApp이 간단해 보이지만, 탈중앙화된 투표 시스템의 핵심 로직이 전부 들어있습니다. 실제로 DAO(탈중앙화 자율 조직)의 거버넌스 투표가 이와 비슷한 구조예요.

---

### 3-2. Q&A 및 토론 (10분)

**교수 멘트:**

> 질문 있으신 분?

**예상 질문과 답변:**

**Q1: modifier를 여러 개 붙이면 실행 순서가 어떻게 되나요?**

> 좋은 질문입니다. 왼쪽에서 오른쪽 순서로 실행됩니다. `function withdraw() public onlyOwner validAmount`라면 `onlyOwner`가 먼저 체크되고, 그 다음 `validAmount`가 체크됩니다.

**Q2: 비밀 투표는 어떻게 만드나요?**

> 지금 만든 투표 시스템은 모든 것이 공개되죠. 비밀 투표를 구현하려면 "커밋-리빌" 패턴을 사용합니다. 먼저 투표 내용을 해시로 제출하고(커밋), 투표 기간이 끝나면 원본을 공개하는(리빌) 방식이에요. 이건 좀 더 고급 주제이니 관심 있는 분은 따로 공부해 보세요.

**Q3: 이더리움 메인넷에 이 계약을 배포하면 가스비가 얼마나 드나요?**

> 배포 가스가 대략 100만 가스 정도 될 거예요. 이더리움 가스 가격이 20 Gwei라고 하면, 약 0.02 ETH, 현재 시세로 5~10만원 정도입니다. 그래서 많은 프로젝트가 가스비가 저렴한 Layer 2 솔루션을 사용합니다.

---

### 3-3. 다음 주 예고 및 과제 (10분)

**교수 멘트:**

> 다음 주에는 **디지털 지갑**으로 넘어갑니다. WIA-FIN-015 표준이에요. MetaMask를 직접 설치하고 테스트넷에서 트랜잭션을 보내볼 거예요. 스마트 계약을 만들 줄 아는 것과, 실제로 지갑에서 트랜잭션을 보내는 건 다른 경험이니까 기대하세요.

#### 과제

1. **투표 계약 기능 확장**: 다음 기능 중 하나 이상 추가
   - 투표 종료 후 승자를 확정하는 `declareWinner` 함수
   - 투표 기간을 타임스탬프로 제한하는 기능 (힌트: `block.timestamp`)
   - 관리자가 유권자를 사전 등록하는 화이트리스트 기능
2. **가스비 분석**: 각 기능의 가스 사용량을 표로 정리
3. **WIA 시뮬레이터**: 확장된 계약을 시뮬레이터에서 검증하고 보안 분석 결과 캡처

**교수 멘트:**

> 세 가지 확장 기능 중 하나만 해도 됩니다. 여유가 되면 셋 다 도전해 보세요. 타임스탬프를 활용한 투표 기간 제한이 가장 실용적이니까, 하나만 할 거면 그걸 추천합니다. 그럼 다음 주에 만나요!

---

*본 강의 자료는 WIA (World Certification Industry Association) 표준 및 시뮬레이터를 활용하여 제작되었습니다.*
*WIA 시뮬레이터: https://wiastandards.com/smart-contract/simulator*
