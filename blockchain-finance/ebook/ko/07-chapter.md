# 제7장: 크로스체인 인프라 및 상호운용성

## 7.1 크로스체인의 필요성과 과제

### 7.1.1 블록체인 단편화 문제

현대 블록체인 생태계는 수백 개의 독립적인 네트워크로 구성되어 있으며, 각 네트워크는 고유한 합의 메커니즘, 프로그래밍 언어, 자산 표준을 가지고 있습니다. 이러한 단편화는 유동성 분산, 사용자 경험 저하, 개발 복잡성 증가라는 심각한 문제를 야기합니다.

**블록체인 생태계 현황 (2025):**

```
Layer 1 블록체인 분포:
├── Ethereum (EVM)
│   ├── TVL: ~$180B
│   ├── 일일 거래: ~1.2M
│   └── 스마트 컨트랙트: ~50M
│
├── Solana (SVM)
│   ├── TVL: ~$15B
│   ├── 일일 거래: ~40M
│   └── 프로그램: ~2M
│
├── Bitcoin
│   ├── TVL: ~$1.8T (시가총액)
│   ├── 일일 거래: ~500K
│   └── Ordinals/BRC-20: ~50M
│
├── Cosmos (IBC)
│   ├── 총 TVL: ~$25B
│   ├── 연결 체인: 80+
│   └── IBC 전송: ~100M/월
│
├── Polkadot
│   ├── TVL: ~$5B
│   ├── 파라체인: 50+
│   └── XCM 메시지: ~10M/월
│
└── 기타 L1 (Avalanche, Near, Sui, Aptos...)
    └── 총 TVL: ~$30B
```

### 7.1.2 크로스체인 핵심 과제

**트릴레마: 보안, 탈중앙화, 일반화 가능성**

```typescript
interface 크로스체인트릴레마 {
  보안: {
    정의: "전송된 자산의 안전성 보장";
    과제: "브릿지 해킹으로 $2B+ 손실 (2021-2024)";
    사례: "Ronin ($625M), Wormhole ($320M), Nomad ($190M)";
  };

  탈중앙화: {
    정의: "소수 검증자 의존 없이 운영";
    과제: "대부분 브릿지 중앙화된 다중서명 사용";
    목표: "무신뢰 검증 메커니즘";
  };

  일반화가능성: {
    정의: "임의의 데이터/메시지 전송 지원";
    과제: "단순 토큰 전송 vs 복잡한 컨트랙트 호출";
    목표: "완전한 크로스체인 컴퓨팅";
  };
}
```

**주요 브릿지 보안 사고 분석:**

| 사건 | 날짜 | 손실액 | 원인 | 교훈 |
|------|------|--------|------|------|
| Ronin | 2022.03 | $625M | 개인키 탈취 | 다중서명 보안 강화 필요 |
| Wormhole | 2022.02 | $320M | 서명 검증 취약점 | 스마트 컨트랙트 감사 중요 |
| Nomad | 2022.08 | $190M | 초기화 버그 | 배포 프로세스 검토 필수 |
| Harmony | 2022.06 | $100M | 다중서명 키 탈취 | 키 관리 분산화 필요 |
| Multichain | 2023.07 | $130M | CEO 체포/키 집중 | 운영 리스크 관리 |

## 7.2 크로스체인 브릿지 아키텍처

### 7.2.1 브릿지 유형 분류

**Lock-and-Mint 방식:**

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

import "@openzeppelin/contracts/token/ERC20/IERC20.sol";
import "@openzeppelin/contracts/token/ERC20/utils/SafeERC20.sol";
import "@openzeppelin/contracts/security/ReentrancyGuard.sol";
import "@openzeppelin/contracts/access/AccessControl.sol";

/**
 * @title LockAndMint브릿지
 * @notice 소스 체인에서 토큰을 잠그고 대상 체인에서 래핑 토큰 발행
 */
contract SourceChainVault is ReentrancyGuard, AccessControl {
    using SafeERC20 for IERC20;

    bytes32 public constant BRIDGE_OPERATOR = keccak256("BRIDGE_OPERATOR");

    struct 잠금정보 {
        address 토큰;
        address 발신자;
        uint256 금액;
        uint256 대상체인ID;
        bytes32 수취인; // 대상 체인 주소 (bytes32로 일반화)
        uint256 타임스탬프;
        bool 완료됨;
    }

    // 논스 => 잠금 정보
    mapping(uint256 => 잠금정보) public 잠금기록;
    uint256 public 현재논스;

    // 토큰 => 총 잠금량
    mapping(address => uint256) public 토큰별잠금량;

    // 지원 토큰 목록
    mapping(address => bool) public 지원토큰;

    // 대상 체인별 최소/최대 금액
    mapping(uint256 => uint256) public 최소금액;
    mapping(uint256 => uint256) public 최대금액;

    event 토큰잠금(
        uint256 indexed 논스,
        address indexed 토큰,
        address indexed 발신자,
        uint256 금액,
        uint256 대상체인ID,
        bytes32 수취인
    );

    event 토큰해제(
        uint256 indexed 논스,
        address indexed 토큰,
        address indexed 수취인,
        uint256 금액
    );

    constructor() {
        _grantRole(DEFAULT_ADMIN_ROLE, msg.sender);
        _grantRole(BRIDGE_OPERATOR, msg.sender);
    }

    /**
     * @notice 토큰을 잠그고 브릿지 요청 생성
     * @param _토큰 잠글 토큰 주소
     * @param _금액 잠글 금액
     * @param _대상체인ID 대상 체인 식별자
     * @param _수취인 대상 체인의 수취인 주소
     */
    function 토큰잠그기(
        address _토큰,
        uint256 _금액,
        uint256 _대상체인ID,
        bytes32 _수취인
    ) external nonReentrant returns (uint256 논스) {
        require(지원토큰[_토큰], "Unsupported token");
        require(_금액 >= 최소금액[_대상체인ID], "Below minimum");
        require(_금액 <= 최대금액[_대상체인ID], "Above maximum");
        require(_수취인 != bytes32(0), "Invalid recipient");

        // 토큰 전송
        IERC20(_토큰).safeTransferFrom(msg.sender, address(this), _금액);

        // 잠금 정보 기록
        논스 = 현재논스++;
        잠금기록[논스] = 잠금정보({
            토큰: _토큰,
            발신자: msg.sender,
            금액: _금액,
            대상체인ID: _대상체인ID,
            수취인: _수취인,
            타임스탬프: block.timestamp,
            완료됨: false
        });

        토큰별잠금량[_토큰] += _금액;

        emit 토큰잠금(논스, _토큰, msg.sender, _금액, _대상체인ID, _수취인);

        return 논스;
    }

    /**
     * @notice 대상 체인에서 민트 확인 후 잠금 완료 처리
     */
    function 잠금완료처리(
        uint256 _논스,
        bytes calldata _증명
    ) external onlyRole(BRIDGE_OPERATOR) {
        잠금정보 storage 정보 = 잠금기록[_논스];
        require(!정보.완료됨, "Already completed");

        // 증명 검증 (실제로는 라이트 클라이언트 또는 오라클 검증)
        require(_검증증명(_증명), "Invalid proof");

        정보.완료됨 = true;
    }

    /**
     * @notice 대상 체인에서 번 후 소스 체인에서 토큰 해제
     */
    function 토큰해제(
        address _토큰,
        address _수취인,
        uint256 _금액,
        uint256 _대상체인논스,
        bytes calldata _번증명
    ) external nonReentrant onlyRole(BRIDGE_OPERATOR) {
        require(토큰별잠금량[_토큰] >= _금액, "Insufficient locked");

        // 번 증명 검증
        require(_검증번증명(_번증명), "Invalid burn proof");

        토큰별잠금량[_토큰] -= _금액;
        IERC20(_토큰).safeTransfer(_수취인, _금액);

        emit 토큰해제(_대상체인논스, _토큰, _수취인, _금액);
    }

    function _검증증명(bytes calldata) internal pure returns (bool) {
        // 실제 구현: 라이트 클라이언트, 오라클, 또는 다중서명 검증
        return true;
    }

    function _검증번증명(bytes calldata) internal pure returns (bool) {
        // 실제 구현: 번 트랜잭션 증명 검증
        return true;
    }

    /**
     * @notice 지원 토큰 추가
     */
    function 토큰추가(
        address _토큰,
        uint256 _체인ID,
        uint256 _최소,
        uint256 _최대
    ) external onlyRole(DEFAULT_ADMIN_ROLE) {
        지원토큰[_토큰] = true;
        최소금액[_체인ID] = _최소;
        최대금액[_체인ID] = _최대;
    }
}
```

**Burn-and-Mint 방식 (대상 체인):**

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

import "@openzeppelin/contracts/token/ERC20/ERC20.sol";
import "@openzeppelin/contracts/token/ERC20/extensions/ERC20Burnable.sol";
import "@openzeppelin/contracts/access/AccessControl.sol";

/**
 * @title 래핑토큰
 * @notice 브릿지를 통해 발행되는 래핑된 토큰
 */
contract WrappedToken is ERC20, ERC20Burnable, AccessControl {
    bytes32 public constant MINTER_ROLE = keccak256("MINTER_ROLE");
    bytes32 public constant BRIDGE_ROLE = keccak256("BRIDGE_ROLE");

    // 원본 토큰 정보
    uint256 public immutable 소스체인ID;
    address public immutable 원본토큰;

    // 민트 기록
    mapping(bytes32 => bool) public 처리된민트;

    event 래핑토큰민트(
        bytes32 indexed 소스트랜잭션,
        address indexed 수취인,
        uint256 금액
    );

    event 래핑토큰번(
        address indexed 발신자,
        uint256 금액,
        bytes32 소스체인수취인
    );

    constructor(
        string memory _이름,
        string memory _심볼,
        uint256 _소스체인ID,
        address _원본토큰
    ) ERC20(_이름, _심볼) {
        소스체인ID = _소스체인ID;
        원본토큰 = _원본토큰;
        _grantRole(DEFAULT_ADMIN_ROLE, msg.sender);
    }

    /**
     * @notice 소스 체인 잠금 확인 후 래핑 토큰 민트
     */
    function 래핑민트(
        address _수취인,
        uint256 _금액,
        bytes32 _소스트랜잭션,
        bytes calldata _잠금증명
    ) external onlyRole(BRIDGE_ROLE) {
        require(!처리된민트[_소스트랜잭션], "Already minted");

        // 잠금 증명 검증
        require(_검증잠금증명(_잠금증명), "Invalid lock proof");

        처리된민트[_소스트랜잭션] = true;
        _mint(_수취인, _금액);

        emit 래핑토큰민트(_소스트랜잭션, _수취인, _금액);
    }

    /**
     * @notice 래핑 토큰 번하고 소스 체인으로 반환 요청
     */
    function 래핑번(
        uint256 _금액,
        bytes32 _소스체인수취인
    ) external {
        require(_소스체인수취인 != bytes32(0), "Invalid recipient");

        _burn(msg.sender, _금액);

        emit 래핑토큰번(msg.sender, _금액, _소스체인수취인);
    }

    function _검증잠금증명(bytes calldata) internal pure returns (bool) {
        return true;
    }
}
```

### 7.2.2 라이트 클라이언트 기반 브릿지

**무신뢰 검증 아키텍처:**

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

/**
 * @title 라이트클라이언트브릿지
 * @notice 소스 체인의 라이트 클라이언트를 통한 무신뢰 검증
 */
contract LightClientBridge {

    struct 블록헤더 {
        bytes32 부모해시;
        bytes32 상태루트;
        bytes32 트랜잭션루트;
        bytes32 영수증루트;
        uint256 블록번호;
        uint256 타임스탬프;
        bytes 추가데이터;
    }

    struct 검증인세트 {
        address[] 검증인목록;
        uint256[] 투표력;
        uint256 총투표력;
        uint256 유효시작블록;
        uint256 유효종료블록;
    }

    // 소스 체인 ID
    uint256 public immutable 소스체인ID;

    // 최신 검증된 블록
    uint256 public 최신블록번호;
    bytes32 public 최신블록해시;

    // 블록 번호 => 블록 헤더
    mapping(uint256 => 블록헤더) public 블록헤더저장소;

    // 에포크 => 검증인 세트
    mapping(uint256 => 검증인세트) public 검증인세트저장소;

    // 최소 합의 요건 (예: 2/3)
    uint256 public constant 합의임계값 = 6667; // 66.67%
    uint256 public constant 합의분모 = 10000;

    event 블록헤더업데이트(uint256 indexed 블록번호, bytes32 블록해시);
    event 검증인세트업데이트(uint256 indexed 에포크);

    constructor(uint256 _소스체인ID) {
        소스체인ID = _소스체인ID;
    }

    /**
     * @notice 새 블록 헤더 제출 및 검증
     */
    function 블록헤더제출(
        블록헤더 calldata _헤더,
        bytes[] calldata _서명들,
        uint256[] calldata _서명인덱스
    ) external {
        require(_헤더.블록번호 > 최신블록번호, "Not newer block");

        // 현재 유효한 검증인 세트 조회
        uint256 에포크 = _헤더.블록번호 / 100; // 예: 100블록마다 에포크
        검증인세트 storage 현재세트 = 검증인세트저장소[에포크];

        // 서명 검증 및 투표력 집계
        uint256 서명된투표력 = 0;
        bytes32 메시지해시 = keccak256(abi.encode(_헤더));

        for (uint i = 0; i < _서명들.length; i++) {
            uint256 idx = _서명인덱스[i];
            address 서명자 = _복구서명자(메시지해시, _서명들[i]);

            require(서명자 == 현재세트.검증인목록[idx], "Invalid signer");
            서명된투표력 += 현재세트.투표력[idx];
        }

        // 합의 임계값 확인
        require(
            서명된투표력 * 합의분모 >= 현재세트.총투표력 * 합의임계값,
            "Insufficient consensus"
        );

        // 블록 헤더 저장
        블록헤더저장소[_헤더.블록번호] = _헤더;
        최신블록번호 = _헤더.블록번호;
        최신블록해시 = keccak256(abi.encode(_헤더));

        emit 블록헤더업데이트(_헤더.블록번호, 최신블록해시);
    }

    /**
     * @notice 트랜잭션 포함 증명 검증
     */
    function 트랜잭션증명검증(
        uint256 _블록번호,
        bytes calldata _트랜잭션,
        bytes32[] calldata _머클증명,
        uint256 _인덱스
    ) external view returns (bool) {
        블록헤더 storage 헤더 = 블록헤더저장소[_블록번호];
        require(헤더.블록번호 == _블록번호, "Block not found");

        bytes32 트랜잭션해시 = keccak256(_트랜잭션);

        return _머클증명검증(
            _머클증명,
            헤더.트랜잭션루트,
            트랜잭션해시,
            _인덱스
        );
    }

    /**
     * @notice 이벤트/영수증 포함 증명 검증
     */
    function 영수증증명검증(
        uint256 _블록번호,
        bytes calldata _영수증,
        bytes32[] calldata _머클증명,
        uint256 _인덱스
    ) external view returns (bool) {
        블록헤더 storage 헤더 = 블록헤더저장소[_블록번호];
        require(헤더.블록번호 == _블록번호, "Block not found");

        bytes32 영수증해시 = keccak256(_영수증);

        return _머클증명검증(
            _머클증명,
            헤더.영수증루트,
            영수증해시,
            _인덱스
        );
    }

    function _머클증명검증(
        bytes32[] calldata _증명,
        bytes32 _루트,
        bytes32 _리프,
        uint256 _인덱스
    ) internal pure returns (bool) {
        bytes32 계산해시 = _리프;

        for (uint i = 0; i < _증명.length; i++) {
            if (_인덱스 % 2 == 0) {
                계산해시 = keccak256(abi.encodePacked(계산해시, _증명[i]));
            } else {
                계산해시 = keccak256(abi.encodePacked(_증명[i], 계산해시));
            }
            _인덱스 /= 2;
        }

        return 계산해시 == _루트;
    }

    function _복구서명자(
        bytes32 _메시지해시,
        bytes calldata _서명
    ) internal pure returns (address) {
        require(_서명.length == 65, "Invalid signature length");

        bytes32 r;
        bytes32 s;
        uint8 v;

        assembly {
            r := calldataload(_서명.offset)
            s := calldataload(add(_서명.offset, 32))
            v := byte(0, calldataload(add(_서명.offset, 64)))
        }

        return ecrecover(_메시지해시, v, r, s);
    }
}
```

## 7.3 메시징 프로토콜

### 7.3.1 LayerZero 통합

**LayerZero V2 아키텍처:**

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

import "@layerzerolabs/lz-evm-oapp-v2/contracts/oapp/OApp.sol";
import "@layerzerolabs/lz-evm-oapp-v2/contracts/oapp/libs/OptionsBuilder.sol";

/**
 * @title WIA크로스체인메시징
 * @notice LayerZero V2 기반 크로스체인 메시지 전송
 */
contract WIACrossChainMessaging is OApp {
    using OptionsBuilder for bytes;

    // 메시지 유형
    enum 메시지유형 {
        토큰전송,
        컨트랙트호출,
        거버넌스투표,
        오라클데이터,
        긴급정지
    }

    struct 크로스체인메시지 {
        메시지유형 유형;
        bytes32 발신자;
        bytes32 수취인;
        bytes 페이로드;
        uint256 타임스탬프;
        uint256 논스;
    }

    // 체인 ID => 피어 컨트랙트 주소
    mapping(uint32 => bytes32) public 피어컨트랙트;

    // 메시지 논스
    mapping(uint32 => uint256) public 발신논스;
    mapping(uint32 => uint256) public 수신논스;

    // 처리된 메시지
    mapping(bytes32 => bool) public 처리됨;

    event 메시지발신(
        uint32 indexed 대상체인,
        메시지유형 indexed 유형,
        bytes32 messageId,
        bytes 페이로드
    );

    event 메시지수신(
        uint32 indexed 소스체인,
        메시지유형 indexed 유형,
        bytes32 messageId,
        bytes 페이로드
    );

    constructor(
        address _endpoint,
        address _owner
    ) OApp(_endpoint, _owner) Ownable(_owner) {}

    /**
     * @notice 크로스체인 메시지 전송
     */
    function 메시지전송(
        uint32 _대상체인,
        메시지유형 _유형,
        bytes32 _수취인,
        bytes calldata _페이로드,
        bytes calldata _옵션
    ) external payable returns (bytes32 messageId) {
        require(피어컨트랙트[_대상체인] != bytes32(0), "Unknown chain");

        uint256 논스 = 발신논스[_대상체인]++;

        크로스체인메시지 memory 메시지 = 크로스체인메시지({
            유형: _유형,
            발신자: bytes32(uint256(uint160(msg.sender))),
            수취인: _수취인,
            페이로드: _페이로드,
            타임스탬프: block.timestamp,
            논스: 논스
        });

        bytes memory 인코딩메시지 = abi.encode(메시지);

        messageId = keccak256(abi.encodePacked(
            block.chainid,
            _대상체인,
            msg.sender,
            논스
        ));

        // LayerZero 메시지 전송
        _lzSend(
            _대상체인,
            인코딩메시지,
            _옵션,
            MessagingFee(msg.value, 0),
            payable(msg.sender)
        );

        emit 메시지발신(_대상체인, _유형, messageId, _페이로드);

        return messageId;
    }

    /**
     * @notice 메시지 수신 처리 (LayerZero 호출)
     */
    function _lzReceive(
        Origin calldata _origin,
        bytes32 _guid,
        bytes calldata _message,
        address,
        bytes calldata
    ) internal override {
        require(!처리됨[_guid], "Already processed");
        처리됨[_guid] = true;

        크로스체인메시지 memory 메시지 = abi.decode(_message, (크로스체인메시지));

        // 논스 검증
        require(메시지.논스 == 수신논스[_origin.srcEid]++, "Invalid nonce");

        // 메시지 유형별 처리
        _메시지처리(메시지, _origin.srcEid);

        emit 메시지수신(_origin.srcEid, 메시지.유형, _guid, 메시지.페이로드);
    }

    function _메시지처리(크로스체인메시지 memory _메시지, uint32 _소스체인) internal {
        if (_메시지.유형 == 메시지유형.토큰전송) {
            _토큰전송처리(_메시지);
        } else if (_메시지.유형 == 메시지유형.컨트랙트호출) {
            _컨트랙트호출처리(_메시지);
        } else if (_메시지.유형 == 메시지유형.거버넌스투표) {
            _거버넌스투표처리(_메시지);
        } else if (_메시지.유형 == 메시지유형.오라클데이터) {
            _오라클데이터처리(_메시지);
        } else if (_메시지.유형 == 메시지유형.긴급정지) {
            _긴급정지처리(_메시지, _소스체인);
        }
    }

    function _토큰전송처리(크로스체인메시지 memory _메시지) internal {
        (address 토큰, uint256 금액) = abi.decode(_메시지.페이로드, (address, uint256));
        address 수취인 = address(uint160(uint256(_메시지.수취인)));

        // 래핑 토큰 민트 또는 잠긴 토큰 해제
        // ... 구현
    }

    function _컨트랙트호출처리(크로스체인메시지 memory _메시지) internal {
        (address 대상, bytes memory 호출데이터) = abi.decode(
            _메시지.페이로드, (address, bytes)
        );

        // 안전한 컨트랙트 호출
        (bool 성공, ) = 대상.call(호출데이터);
        require(성공, "Contract call failed");
    }

    function _거버넌스투표처리(크로스체인메시지 memory _메시지) internal {
        // 크로스체인 거버넌스 투표 집계
    }

    function _오라클데이터처리(크로스체인메시지 memory _메시지) internal {
        // 크로스체인 오라클 데이터 업데이트
    }

    function _긴급정지처리(크로스체인메시지 memory, uint32 _소스체인) internal {
        // 긴급 정지 시 모든 체인 동기화
        require(_소스체인 == 1, "Only main chain can pause"); // Ethereum
        // _pause();
    }

    /**
     * @notice 메시지 전송 수수료 견적
     */
    function 수수료견적(
        uint32 _대상체인,
        bytes calldata _메시지,
        bytes calldata _옵션
    ) external view returns (uint256 네이티브수수료) {
        MessagingFee memory 수수료 = _quote(_대상체인, _메시지, _옵션, false);
        return 수수료.nativeFee;
    }

    /**
     * @notice 피어 컨트랙트 설정
     */
    function 피어설정(
        uint32 _체인ID,
        bytes32 _피어주소
    ) external onlyOwner {
        피어컨트랙트[_체인ID] = _피어주소;
        _setPeer(_체인ID, _피어주소);
    }
}
```

### 7.3.2 Chainlink CCIP 통합

**CCIP 기반 크로스체인 토큰:**

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

import {IRouterClient} from "@chainlink/contracts-ccip/src/v0.8/ccip/interfaces/IRouterClient.sol";
import {Client} from "@chainlink/contracts-ccip/src/v0.8/ccip/libraries/Client.sol";
import {CCIPReceiver} from "@chainlink/contracts-ccip/src/v0.8/ccip/applications/CCIPReceiver.sol";
import {IERC20} from "@openzeppelin/contracts/token/ERC20/IERC20.sol";
import {SafeERC20} from "@openzeppelin/contracts/token/ERC20/utils/SafeERC20.sol";

/**
 * @title WIACCIP토큰브릿지
 * @notice Chainlink CCIP 기반 토큰 브릿지
 */
contract WIACCIPBridge is CCIPReceiver {
    using SafeERC20 for IERC20;

    // CCIP 라우터
    IRouterClient public immutable router;

    // 체인 셀렉터 => 피어 브릿지 주소
    mapping(uint64 => address) public 피어브릿지;

    // 지원 토큰 (로컬 => 원격 매핑)
    mapping(address => mapping(uint64 => address)) public 토큰매핑;

    // 메시지 ID => 처리 상태
    mapping(bytes32 => bool) public 처리됨;

    // LINK 토큰 (수수료 지불용)
    IERC20 public immutable linkToken;

    event 토큰전송시작(
        bytes32 indexed messageId,
        uint64 indexed 대상체인,
        address indexed 발신자,
        address 토큰,
        uint256 금액
    );

    event 토큰수신완료(
        bytes32 indexed messageId,
        uint64 indexed 소스체인,
        address indexed 수취인,
        address 토큰,
        uint256 금액
    );

    constructor(
        address _router,
        address _linkToken
    ) CCIPReceiver(_router) {
        router = IRouterClient(_router);
        linkToken = IERC20(_linkToken);
    }

    /**
     * @notice 토큰 크로스체인 전송
     */
    function 토큰전송(
        uint64 _대상체인셀렉터,
        address _토큰,
        uint256 _금액,
        address _수취인,
        bool _LINK수수료사용
    ) external payable returns (bytes32 messageId) {
        require(피어브릿지[_대상체인셀렉터] != address(0), "Unknown chain");
        require(토큰매핑[_토큰][_대상체인셀렉터] != address(0), "Unknown token");

        // 토큰 전송받기
        IERC20(_토큰).safeTransferFrom(msg.sender, address(this), _금액);

        // CCIP 토큰 전송 메시지 구성
        Client.EVMTokenAmount[] memory 토큰금액 = new Client.EVMTokenAmount[](1);
        토큰금액[0] = Client.EVMTokenAmount({
            token: _토큰,
            amount: _금액
        });

        // 메시지 구성
        Client.EVM2AnyMessage memory 메시지 = Client.EVM2AnyMessage({
            receiver: abi.encode(피어브릿지[_대상체인셀렉터]),
            data: abi.encode(_수취인, 토큰매핑[_토큰][_대상체인셀렉터]),
            tokenAmounts: 토큰금액,
            extraArgs: Client._argsToBytes(
                Client.EVMExtraArgsV1({gasLimit: 200_000})
            ),
            feeToken: _LINK수수료사용 ? address(linkToken) : address(0)
        });

        // 수수료 계산
        uint256 수수료 = router.getFee(_대상체인셀렉터, 메시지);

        // 토큰 승인 (CCIP 라우터)
        IERC20(_토큰).approve(address(router), _금액);

        if (_LINK수수료사용) {
            linkToken.safeTransferFrom(msg.sender, address(this), 수수료);
            linkToken.approve(address(router), 수수료);

            messageId = router.ccipSend(_대상체인셀렉터, 메시지);
        } else {
            require(msg.value >= 수수료, "Insufficient fee");

            messageId = router.ccipSend{value: 수수료}(_대상체인셀렉터, 메시지);

            // 잔액 반환
            if (msg.value > 수수료) {
                payable(msg.sender).transfer(msg.value - 수수료);
            }
        }

        emit 토큰전송시작(messageId, _대상체인셀렉터, msg.sender, _토큰, _금액);

        return messageId;
    }

    /**
     * @notice CCIP 메시지 수신 처리
     */
    function _ccipReceive(
        Client.Any2EVMMessage memory 메시지
    ) internal override {
        require(!처리됨[메시지.messageId], "Already processed");
        처리됨[메시지.messageId] = true;

        // 메시지 디코딩
        (address 수취인, address 토큰) = abi.decode(메시지.data, (address, address));

        // 토큰 전송
        if (메시지.destTokenAmounts.length > 0) {
            uint256 금액 = 메시지.destTokenAmounts[0].amount;
            address 수신토큰 = 메시지.destTokenAmounts[0].token;

            IERC20(수신토큰).safeTransfer(수취인, 금액);

            emit 토큰수신완료(
                메시지.messageId,
                메시지.sourceChainSelector,
                수취인,
                수신토큰,
                금액
            );
        }
    }

    /**
     * @notice 전송 수수료 견적
     */
    function 수수료견적(
        uint64 _대상체인셀렉터,
        address _토큰,
        uint256 _금액,
        address _수취인,
        bool _LINK수수료사용
    ) external view returns (uint256) {
        Client.EVMTokenAmount[] memory 토큰금액 = new Client.EVMTokenAmount[](1);
        토큰금액[0] = Client.EVMTokenAmount({
            token: _토큰,
            amount: _금액
        });

        Client.EVM2AnyMessage memory 메시지 = Client.EVM2AnyMessage({
            receiver: abi.encode(피어브릿지[_대상체인셀렉터]),
            data: abi.encode(_수취인, 토큰매핑[_토큰][_대상체인셀렉터]),
            tokenAmounts: 토큰금액,
            extraArgs: Client._argsToBytes(
                Client.EVMExtraArgsV1({gasLimit: 200_000})
            ),
            feeToken: _LINK수수료사용 ? address(linkToken) : address(0)
        });

        return router.getFee(_대상체인셀렉터, 메시지);
    }

    /**
     * @notice 피어 브릿지 설정
     */
    function 피어설정(
        uint64 _체인셀렉터,
        address _브릿지주소
    ) external {
        // onlyOwner modifier 필요
        피어브릿지[_체인셀렉터] = _브릿지주소;
    }

    /**
     * @notice 토큰 매핑 설정
     */
    function 토큰매핑설정(
        address _로컬토큰,
        uint64 _체인셀렉터,
        address _원격토큰
    ) external {
        // onlyOwner modifier 필요
        토큰매핑[_로컬토큰][_체인셀렉터] = _원격토큰;
    }
}
```

## 7.4 IBC (Inter-Blockchain Communication)

### 7.4.1 IBC 프로토콜 개요

**IBC 아키텍처:**

```typescript
interface IBC아키텍처 {
  전송계층: {
    클라이언트: {
      역할: "상대 체인의 합의 상태 추적";
      유형: ["Tendermint", "Solo Machine", "Localhost"];
      갱신: "헤더 업데이트를 통한 상태 동기화";
    };

    연결: {
      역할: "두 체인 간의 인증된 통신 채널";
      상태: ["INIT", "TRYOPEN", "OPEN"];
      버전협상: "지원 프로토콜 버전 합의";
    };

    채널: {
      역할: "특정 애플리케이션 간 패킷 전송";
      순서: ["ORDERED", "UNORDERED"];
      포트: "애플리케이션 식별자";
    };
  };

  애플리케이션계층: {
    ICS20: {
      명칭: "Fungible Token Transfer";
      기능: "토큰 크로스체인 전송";
      메커니즘: "에스크로 & 민트/번";
    };

    ICS27: {
      명칭: "Interchain Accounts";
      기능: "원격 체인 계정 제어";
      용도: "크로스체인 DeFi, 거버넌스";
    };

    ICS721: {
      명칭: "NFT Transfer";
      기능: "NFT 크로스체인 전송";
      상태: "제안 중";
    };
  };

  릴레이어: {
    역할: "체인 간 패킷 전달";
    인센티브: "수수료 또는 인플레이션 보상";
    무허가: "누구나 릴레이어 운영 가능";
  };
}
```

### 7.4.2 ICS-20 토큰 전송 구현

**Cosmos SDK ICS-20 구현:**

```go
// ICS-20 토큰 전송 메시지 정의
package types

import (
    sdk "github.com/cosmos/cosmos-sdk/types"
    clienttypes "github.com/cosmos/ibc-go/v7/modules/core/02-client/types"
)

// MsgTransfer ICS-20 토큰 전송 메시지
type MsgTransfer struct {
    // 소스 포트 ID
    SourcePort string
    // 소스 채널 ID
    SourceChannel string
    // 전송할 토큰
    Token sdk.Coin
    // 발신자 주소
    Sender string
    // 수취인 주소 (상대 체인)
    Receiver string
    // 타임아웃 높이
    TimeoutHeight clienttypes.Height
    // 타임아웃 타임스탬프
    TimeoutTimestamp uint64
    // 메모 (선택사항)
    Memo string
}

// FungibleTokenPacketData ICS-20 패킷 데이터
type FungibleTokenPacketData struct {
    // 토큰 denom
    Denom string
    // 금액
    Amount string
    // 발신자
    Sender string
    // 수취인
    Receiver string
    // 메모
    Memo string
}

// ICS-20 전송 핸들러 (간소화)
func (k Keeper) SendTransfer(
    ctx sdk.Context,
    sourcePort string,
    sourceChannel string,
    token sdk.Coin,
    sender sdk.AccAddress,
    receiver string,
    timeoutHeight clienttypes.Height,
    timeoutTimestamp uint64,
    memo string,
) (uint64, error) {

    // 1. 채널 존재 및 상태 확인
    channel, found := k.channelKeeper.GetChannel(ctx, sourcePort, sourceChannel)
    if !found {
        return 0, sdkerrors.Wrap(channeltypes.ErrChannelNotFound, sourceChannel)
    }

    // 2. denom 추적 경로 확인
    fullDenomPath := token.Denom

    // 네이티브 토큰인 경우 에스크로
    if !strings.HasPrefix(token.Denom, "ibc/") {
        // 토큰을 에스크로 계정으로 전송
        escrowAddress := types.GetEscrowAddress(sourcePort, sourceChannel)
        if err := k.bankKeeper.SendCoins(
            ctx, sender, escrowAddress, sdk.NewCoins(token),
        ); err != nil {
            return 0, err
        }
    } else {
        // IBC 토큰 (래핑된 토큰)인 경우 번
        if err := k.bankKeeper.BurnCoins(
            ctx, types.ModuleName, sdk.NewCoins(token),
        ); err != nil {
            return 0, err
        }

        // 원래 denom 복원
        denomTrace := k.GetDenomTrace(ctx, token.Denom)
        fullDenomPath = denomTrace.GetFullDenomPath()
    }

    // 3. 패킷 데이터 생성
    packetData := FungibleTokenPacketData{
        Denom:    fullDenomPath,
        Amount:   token.Amount.String(),
        Sender:   sender.String(),
        Receiver: receiver,
        Memo:     memo,
    }

    // 4. IBC 패킷 전송
    sequence, err := k.channelKeeper.SendPacket(
        ctx,
        channelCapability,
        sourcePort,
        sourceChannel,
        timeoutHeight,
        timeoutTimestamp,
        packetData.GetBytes(),
    )

    return sequence, err
}

// 패킷 수신 처리
func (k Keeper) OnRecvPacket(
    ctx sdk.Context,
    packet channeltypes.Packet,
    data FungibleTokenPacketData,
) (string, error) {

    // 1. 토큰 경로 확인
    if types.ReceiverChainIsSource(packet.GetSourcePort(), packet.GetSourceChannel(), data.Denom) {
        // 원본 체인으로 돌아온 경우 - 에스크로에서 해제
        escrowAddress := types.GetEscrowAddress(packet.GetDestPort(), packet.GetDestChannel())

        coins := sdk.NewCoins(sdk.NewCoin(data.Denom, sdk.NewIntFromString(data.Amount)))

        if err := k.bankKeeper.SendCoins(
            ctx, escrowAddress, receiver, coins,
        ); err != nil {
            return "", err
        }
    } else {
        // 새로운 체인으로 이동 - IBC 토큰 민트
        prefixedDenom := types.GetPrefixedDenom(
            packet.GetDestPort(), packet.GetDestChannel(), data.Denom,
        )
        denomTrace := types.ParseDenomTrace(prefixedDenom)

        voucher := sdk.NewCoin(denomTrace.IBCDenom(), sdk.NewIntFromString(data.Amount))

        if err := k.bankKeeper.MintCoins(
            ctx, types.ModuleName, sdk.NewCoins(voucher),
        ); err != nil {
            return "", err
        }

        if err := k.bankKeeper.SendCoinsFromModuleToAccount(
            ctx, types.ModuleName, receiver, sdk.NewCoins(voucher),
        ); err != nil {
            return "", err
        }
    }

    return "", nil
}
```

### 7.4.3 Interchain Accounts (ICS-27)

**ICS-27 원격 계정 제어:**

```typescript
interface ICS27아키텍처 {
  개념: {
    컨트롤러체인: "명령을 발행하는 체인";
    호스트체인: "Interchain Account가 존재하는 체인";
    ICA: "원격으로 제어되는 계정";
  };

  흐름: {
    단계1: "컨트롤러 체인에서 ICA 생성 요청";
    단계2: "호스트 체인에서 새 계정 생성";
    단계3: "컨트롤러 체인에서 트랜잭션 패킷 전송";
    단계4: "호스트 체인에서 ICA로 트랜잭션 실행";
    단계5: "결과를 컨트롤러 체인으로 반환";
  };

  활용사례: {
    크로스체인스테이킹: "다른 체인의 스테이킹 참여";
    크로스체인거버넌스: "여러 체인에서 투표";
    크로스체인DeFi: "원격 체인 DeFi 프로토콜 이용";
    유동성관리: "여러 체인 유동성 통합 관리";
  };
}
```

## 7.5 WIA 통합 크로스체인 솔루션

### 7.5.1 WIA 크로스체인 허브

**통합 크로스체인 관리:**

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

import "@openzeppelin/contracts/access/AccessControl.sol";
import "@openzeppelin/contracts/security/ReentrancyGuard.sol";
import "@openzeppelin/contracts/security/Pausable.sol";

/**
 * @title WIA크로스체인허브
 * @notice 여러 브릿지 프로토콜을 통합 관리하는 허브
 */
contract WIACrossChainHub is AccessControl, ReentrancyGuard, Pausable {

    bytes32 public constant BRIDGE_ADMIN = keccak256("BRIDGE_ADMIN");
    bytes32 public constant ROUTER_ROLE = keccak256("ROUTER_ROLE");

    enum 브릿지유형 {
        LAYER_ZERO,
        CHAINLINK_CCIP,
        AXELAR,
        WORMHOLE,
        IBC,
        NATIVE
    }

    struct 브릿지설정 {
        address 컨트랙트주소;
        bool 활성화;
        uint256 수수료승수;  // basis points (10000 = 1x)
        uint256 최대금액;
        uint256 최소금액;
        uint256 일일한도;
        uint256 오늘사용량;
        uint256 마지막리셋;
    }

    struct 라우팅정보 {
        uint256 대상체인ID;
        브릿지유형 선호브릿지;
        브릿지유형[] 대체브릿지;
        uint256 예상시간; // 초 단위
        uint256 예상수수료; // 네이티브 토큰 단위
    }

    // 체인 ID => 브릿지 유형 => 설정
    mapping(uint256 => mapping(브릿지유형 => 브릿지설정)) public 브릿지설정정보;

    // 체인 ID => 라우팅 정보
    mapping(uint256 => 라우팅정보) public 라우팅테이블;

    // 지원 체인 목록
    uint256[] public 지원체인목록;

    // 트랜잭션 추적
    mapping(bytes32 => 전송상태) public 전송추적;

    enum 전송상태 {
        없음,
        시작됨,
        전송중,
        완료,
        실패,
        환불됨
    }

    event 크로스체인전송시작(
        bytes32 indexed 전송ID,
        uint256 indexed 대상체인,
        address indexed 발신자,
        address 토큰,
        uint256 금액,
        브릿지유형 브릿지
    );

    event 크로스체인전송완료(
        bytes32 indexed 전송ID,
        전송상태 상태
    );

    constructor() {
        _grantRole(DEFAULT_ADMIN_ROLE, msg.sender);
        _grantRole(BRIDGE_ADMIN, msg.sender);
    }

    /**
     * @notice 최적 라우트 계산
     */
    function 최적라우트계산(
        uint256 _대상체인,
        address _토큰,
        uint256 _금액
    ) public view returns (
        브릿지유형 추천브릿지,
        uint256 예상수수료,
        uint256 예상시간
    ) {
        라우팅정보 storage 라우팅 = 라우팅테이블[_대상체인];

        // 선호 브릿지 확인
        브릿지설정 storage 선호설정 = 브릿지설정정보[_대상체인][라우팅.선호브릿지];

        if (_브릿지사용가능(선호설정, _금액)) {
            return (
                라우팅.선호브릿지,
                _수수료계산(선호설정, _금액),
                라우팅.예상시간
            );
        }

        // 대체 브릿지 확인
        for (uint i = 0; i < 라우팅.대체브릿지.length; i++) {
            브릿지설정 storage 대체설정 = 브릿지설정정보[_대상체인][라우팅.대체브릿지[i]];

            if (_브릿지사용가능(대체설정, _금액)) {
                return (
                    라우팅.대체브릿지[i],
                    _수수료계산(대체설정, _금액),
                    라우팅.예상시간 * 2 // 대체 브릿지는 시간 2배 예상
                );
            }
        }

        revert("No available bridge");
    }

    /**
     * @notice 크로스체인 토큰 전송
     */
    function 토큰전송(
        uint256 _대상체인,
        address _토큰,
        uint256 _금액,
        address _수취인,
        브릿지유형 _브릿지 // 0이면 자동 선택
    ) external payable nonReentrant whenNotPaused returns (bytes32 전송ID) {
        // 브릿지 자동 선택
        if (_브릿지 == 브릿지유형.LAYER_ZERO) {
            (브릿지유형 추천, , ) = 최적라우트계산(_대상체인, _토큰, _금액);
            _브릿지 = 추천;
        }

        브릿지설정 storage 설정 = 브릿지설정정보[_대상체인][_브릿지];
        require(_브릿지사용가능(설정, _금액), "Bridge unavailable");

        // 일일 한도 업데이트
        _일일한도업데이트(설정, _금액);

        // 전송 ID 생성
        전송ID = keccak256(abi.encodePacked(
            block.chainid,
            _대상체인,
            msg.sender,
            _수취인,
            _금액,
            block.timestamp
        ));

        전송추적[전송ID] = 전송상태.시작됨;

        // 브릿지별 전송 실행
        _브릿지전송실행(_브릿지, _대상체인, _토큰, _금액, _수취인, 전송ID);

        전송추적[전송ID] = 전송상태.전송중;

        emit 크로스체인전송시작(전송ID, _대상체인, msg.sender, _토큰, _금액, _브릿지);

        return 전송ID;
    }

    function _브릿지사용가능(
        브릿지설정 storage _설정,
        uint256 _금액
    ) internal view returns (bool) {
        if (!_설정.활성화) return false;
        if (_금액 < _설정.최소금액) return false;
        if (_금액 > _설정.최대금액) return false;

        // 일일 한도 확인
        uint256 오늘사용량 = _설정.오늘사용량;
        if (block.timestamp / 1 days > _설정.마지막리셋 / 1 days) {
            오늘사용량 = 0;
        }

        if (오늘사용량 + _금액 > _설정.일일한도) return false;

        return true;
    }

    function _수수료계산(
        브릿지설정 storage _설정,
        uint256 _금액
    ) internal view returns (uint256) {
        return _금액 * _설정.수수료승수 / 10000;
    }

    function _일일한도업데이트(
        브릿지설정 storage _설정,
        uint256 _금액
    ) internal {
        if (block.timestamp / 1 days > _설정.마지막리셋 / 1 days) {
            _설정.오늘사용량 = 0;
            _설정.마지막리셋 = block.timestamp;
        }

        _설정.오늘사용량 += _금액;
    }

    function _브릿지전송실행(
        브릿지유형 _브릿지,
        uint256 _대상체인,
        address _토큰,
        uint256 _금액,
        address _수취인,
        bytes32 _전송ID
    ) internal {
        브릿지설정 storage 설정 = 브릿지설정정보[_대상체인][_브릿지];

        if (_브릿지 == 브릿지유형.LAYER_ZERO) {
            // LayerZero 호출
            // ILayerZeroBridge(설정.컨트랙트주소).send(...)
        } else if (_브릿지 == 브릿지유형.CHAINLINK_CCIP) {
            // Chainlink CCIP 호출
            // ICCIPBridge(설정.컨트랙트주소).send(...)
        } else if (_브릿지 == 브릿지유형.AXELAR) {
            // Axelar 호출
        } else if (_브릿지 == 브릿지유형.WORMHOLE) {
            // Wormhole 호출
        }

        // 실제 구현에서는 각 브릿지 인터페이스 호출
    }

    /**
     * @notice 전송 상태 업데이트 (릴레이어/오라클 호출)
     */
    function 전송상태업데이트(
        bytes32 _전송ID,
        전송상태 _상태
    ) external onlyRole(ROUTER_ROLE) {
        require(전송추적[_전송ID] != 전송상태.없음, "Unknown transfer");
        require(전송추적[_전송ID] != 전송상태.완료, "Already completed");

        전송추적[_전송ID] = _상태;

        emit 크로스체인전송완료(_전송ID, _상태);
    }

    /**
     * @notice 브릿지 설정
     */
    function 브릿지설정(
        uint256 _체인ID,
        브릿지유형 _브릿지,
        브릿지설정 calldata _설정
    ) external onlyRole(BRIDGE_ADMIN) {
        브릿지설정정보[_체인ID][_브릿지] = _설정;
    }

    /**
     * @notice 라우팅 설정
     */
    function 라우팅설정(
        uint256 _체인ID,
        라우팅정보 calldata _라우팅
    ) external onlyRole(BRIDGE_ADMIN) {
        라우팅테이블[_체인ID] = _라우팅;
    }
}
```

### 7.5.2 크로스체인 유동성 집계

**통합 유동성 관리:**

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

/**
 * @title WIA크로스체인유동성집계기
 * @notice 여러 체인의 유동성을 통합 관리
 */
contract WIACrossChainLiquidityAggregator {

    struct 체인유동성 {
        uint256 체인ID;
        address 풀주소;
        uint256 TVL;
        uint256 APY;
        uint256 마지막업데이트;
    }

    struct 통합포지션 {
        address 소유자;
        mapping(uint256 => uint256) 체인별잔액; // 체인ID => 잔액
        uint256 총가치;
        uint256 생성시간;
    }

    // 토큰 => 체인별 유동성 정보
    mapping(address => mapping(uint256 => 체인유동성)) public 유동성정보;

    // 사용자 => 포지션 ID => 포지션
    mapping(address => mapping(uint256 => 통합포지션)) public 사용자포지션;

    /**
     * @notice 최적 유동성 배분 계산
     * @param _토큰 토큰 주소
     * @param _금액 총 배분 금액
     * @param _체인목록 대상 체인 목록
     * @return 체인별배분 각 체인에 배분할 금액
     */
    function 최적배분계산(
        address _토큰,
        uint256 _금액,
        uint256[] calldata _체인목록
    ) external view returns (uint256[] memory 체인별배분) {
        체인별배분 = new uint256[](_체인목록.length);

        // 총 가중치 계산 (APY 기반)
        uint256 총가중치 = 0;
        uint256[] memory 가중치 = new uint256[](_체인목록.length);

        for (uint i = 0; i < _체인목록.length; i++) {
            체인유동성 storage 유동성 = 유동성정보[_토큰][_체인목록[i]];

            // APY를 가중치로 사용 (더 높은 APY에 더 많이 배분)
            가중치[i] = 유동성.APY;
            총가중치 += 유동성.APY;
        }

        // 가중치 비례 배분
        uint256 배분합계 = 0;
        for (uint i = 0; i < _체인목록.length - 1; i++) {
            체인별배분[i] = _금액 * 가중치[i] / 총가중치;
            배분합계 += 체인별배분[i];
        }

        // 마지막 체인에 나머지 배분 (반올림 오차 처리)
        체인별배분[_체인목록.length - 1] = _금액 - 배분합계;

        return 체인별배분;
    }

    /**
     * @notice 리밸런싱 필요 여부 확인
     */
    function 리밸런싱필요(
        address _사용자,
        uint256 _포지션ID,
        uint256 _임계값 // basis points
    ) external view returns (bool 필요, uint256[] memory 조정금액) {
        통합포지션 storage 포지션 = 사용자포지션[_사용자][_포지션ID];

        // 현재 배분 vs 최적 배분 비교
        // ... 구현

        return (false, 조정금액);
    }
}
```

## 7.6 크로스체인 보안 고려사항

### 7.6.1 보안 위험 및 완화

**크로스체인 보안 체크리스트:**

```markdown
## WIA 크로스체인 보안 체크리스트

### 1. 브릿지 컨트랙트 보안
- [ ] 다중 감사 완료 (최소 2개 감사 기관)
- [ ] 형식 검증 적용
- [ ] 버그 바운티 프로그램 운영
- [ ] 업그레이드 가능성 제한적 설계
- [ ] 긴급 정지 기능 구현
- [ ] 다중 서명/거버넌스 제어

### 2. 검증자/릴레이어 보안
- [ ] 검증자 다양성 확보 (지역, 운영자)
- [ ] 담보 요건 충분
- [ ] 슬래싱 메커니즘 구현
- [ ] 검증자 교체 절차 수립
- [ ] 릴레이어 무허가 운영 가능

### 3. 암호학적 보안
- [ ] 최신 서명 체계 사용
- [ ] 머클 증명 검증 구현
- [ ] 재생 공격 방지
- [ ] 타임아웃 메커니즘

### 4. 경제적 보안
- [ ] 브릿지 담보 > 잠긴 자산
- [ ] 속도 제한 (일일/트랜잭션)
- [ ] 대규모 전송 지연
- [ ] 보험/보상 기금

### 5. 운영 보안
- [ ] 24/7 모니터링
- [ ] 이상 탐지 시스템
- [ ] 인시던트 대응 계획
- [ ] 정기 침투 테스트
```

**브릿지 보안 모니터링:**

```typescript
interface 브릿지모니터링시스템 {
  실시간모니터링: {
    TVL변화: "급격한 TVL 감소 감지";
    비정상전송: "대규모/반복 전송 패턴";
    검증자상태: "검증자 온라인/오프라인";
    가스가격이상: "비정상적 가스 사용";
    컨트랙트호출: "예상치 못한 함수 호출";
  };

  알림임계값: {
    TVL감소: "10% 이상 1시간 내 감소";
    단일전송: "일일 한도의 20% 초과";
    검증자오프라인: "3개 이상 동시";
    지연: "평균 처리 시간 3배 초과";
  };

  자동대응: {
    일시정지: "심각한 이상 징후 시";
    한도축소: "중간 수준 이상 시";
    검증자교체: "오프라인 검증자 대체";
    알림발송: "관리자/커뮤니티 통보";
  };
}
```

본 장에서는 블록체인 금융의 크로스체인 인프라를 종합적으로 분석했습니다. 브릿지 아키텍처, 메시징 프로토콜(LayerZero, CCIP), IBC, 그리고 WIA의 통합 크로스체인 솔루션을 살펴보았습니다. 다음 장에서는 실제 구현 가이드와 개발 베스트 프랙티스에 대해 다루겠습니다.
