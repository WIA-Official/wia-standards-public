# WIA Blockchain Finance Standard
## Phase 3: Protocol Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** December 2025

---

## Table of Contents

1. [Overview](#overview)
2. [Cross-Chain Protocol](#cross-chain-protocol)
3. [Message Format](#message-format)
4. [Security Requirements](#security-requirements)
5. [Bridge Mechanisms](#bridge-mechanisms)
6. [Consensus & Validation](#consensus--validation)
7. [Protocol Extensions](#protocol-extensions)

---

## Overview

The WIA Blockchain Finance Protocol defines the standards for secure cross-chain communication, asset transfers, and interoperability between different blockchain networks.

### Protocol Layers

```
┌─────────────────────────────────────────┐
│     Application Layer (DApps)           │
├─────────────────────────────────────────┤
│     WIA Protocol Layer                  │
│  - Cross-chain messaging                │
│  - Asset bridging                       │
│  - State synchronization                │
├─────────────────────────────────────────┤
│     Blockchain Layer                    │
│  - Ethereum, BSC, Polygon, etc.         │
└─────────────────────────────────────────┘
```

### Key Features

- **Cross-Chain Communication**: Trustless messaging between blockchains
- **Asset Bridging**: Lock-mint-burn mechanisms for token transfers
- **State Synchronization**: Real-time state updates across chains
- **Security Guarantees**: Cryptographic proofs and multi-sig validation
- **Scalability**: Optimized for high throughput

---

## Cross-Chain Protocol

### Protocol Architecture

The WIA cross-chain protocol uses a hybrid approach combining:
1. **Light Client Verification**: On-chain verification of source chain state
2. **Relayer Network**: Off-chain message relaying with incentives
3. **Multi-Signature Validation**: Decentralized validator set
4. **Fraud Proofs**: Challenge period for dispute resolution

### Protocol Flow

```
Source Chain              Relayer Network           Destination Chain
    │                            │                          │
    │  1. Lock/Burn Asset        │                          │
    │─────────────────────>      │                          │
    │                            │                          │
    │  2. Emit Event             │                          │
    │─────────────────────>      │                          │
    │                            │                          │
    │                     3. Relay Message                  │
    │                            │─────────────────────>    │
    │                            │                          │
    │                            │    4. Verify Proof       │
    │                            │<─────────────────────    │
    │                            │                          │
    │                            │    5. Mint/Unlock Asset  │
    │                            │─────────────────────>    │
    │                            │                          │
    │                     6. Confirmation                   │
    │<─────────────────────────────────────────────────────│
```

### Message Types

1. **Asset Transfer**: Cross-chain token transfers
2. **Contract Call**: Execute contract on destination chain
3. **State Update**: Synchronize state between chains
4. **Governance**: Cross-chain DAO proposals and voting
5. **Oracle Data**: Cross-chain oracle updates

---

## Message Format

### Base Message Structure

```solidity
struct CrossChainMessage {
    uint256 messageId;           // Unique message identifier
    uint256 sourceChainId;       // Source chain ID
    uint256 destinationChainId;  // Destination chain ID
    address sender;              // Message sender on source chain
    address receiver;            // Message receiver on destination chain
    uint256 nonce;               // Message nonce for ordering
    bytes payload;               // Message payload
    uint256 gasLimit;            // Gas limit for execution
    uint256 timestamp;           // Message timestamp
    bytes32 payloadHash;         // Hash of payload
    bytes proof;                 // Cryptographic proof
    bytes[] signatures;          // Validator signatures
}
```

### Asset Transfer Message

```solidity
struct AssetTransferMessage {
    CrossChainMessage base;
    address token;               // Token contract address
    uint256 amount;              // Transfer amount
    TransferType transferType;   // LOCK_MINT or BURN_UNLOCK
    bytes32 tokenHash;           // Token identifier hash
    address recipient;           // Final recipient
    bytes metadata;              // Additional metadata
}

enum TransferType {
    LOCK_MINT,      // Lock on source, mint on destination
    BURN_UNLOCK     // Burn on source, unlock on destination
}
```

### Contract Call Message

```solidity
struct ContractCallMessage {
    CrossChainMessage base;
    address targetContract;      // Contract to call
    bytes callData;              // Function call data
    uint256 value;               // ETH value to send
    bool requireSuccess;         // Revert if call fails
    bytes returnDataHash;        // Expected return data hash
}
```

### Message Encoding

Messages are encoded using RLP (Recursive Length Prefix) encoding:

```typescript
import { encode, decode } from 'rlp';

const message = {
  messageId: 1,
  sourceChainId: 1,
  destinationChainId: 56,
  sender: '0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb',
  receiver: '0x5aAeb6053F3E94C9b9A09f33669435E7Ef1BeAed',
  nonce: 1,
  payload: '0x...',
  gasLimit: 100000,
  timestamp: 1735124400,
  payloadHash: '0x...',
  proof: '0x...',
  signatures: ['0x...']
};

const encoded = encode(Object.values(message));
const decoded = decode(encoded);
```

---

## Security Requirements

### Cryptographic Proofs

#### 1. Merkle Proof Verification

Each message must include a Merkle proof of inclusion in the source chain:

```solidity
function verifyMerkleProof(
    bytes32 leaf,
    bytes32[] memory proof,
    bytes32 root,
    uint256 index
) public pure returns (bool) {
    bytes32 hash = leaf;

    for (uint256 i = 0; i < proof.length; i++) {
        bytes32 proofElement = proof[i];

        if (index % 2 == 0) {
            hash = keccak256(abi.encodePacked(hash, proofElement));
        } else {
            hash = keccak256(abi.encodePacked(proofElement, hash));
        }

        index = index / 2;
    }

    return hash == root;
}
```

#### 2. Light Client Verification

Destination chain maintains light client of source chain:

```solidity
contract LightClient {
    struct BlockHeader {
        bytes32 parentHash;
        bytes32 stateRoot;
        bytes32 transactionsRoot;
        bytes32 receiptsRoot;
        uint256 blockNumber;
        uint256 timestamp;
        bytes32 blockHash;
    }

    mapping(uint256 => BlockHeader) public headers;

    function submitHeader(BlockHeader memory header, bytes memory proof) external {
        require(verifyHeader(header, proof), "Invalid header");
        headers[header.blockNumber] = header;
    }

    function verifyReceipt(
        uint256 blockNumber,
        bytes memory receipt,
        bytes32[] memory proof
    ) public view returns (bool) {
        BlockHeader memory header = headers[blockNumber];
        bytes32 receiptHash = keccak256(receipt);
        return verifyMerkleProof(receiptHash, proof, header.receiptsRoot, 0);
    }
}
```

### Multi-Signature Validation

#### Validator Set

- Minimum 7 validators
- 2/3+ threshold for message approval
- Validator rotation every 24 hours
- Stake slashing for malicious behavior

#### Signature Aggregation

```solidity
contract ValidatorSet {
    struct Validator {
        address addr;
        uint256 stake;
        bool active;
    }

    mapping(address => Validator) public validators;
    address[] public validatorList;
    uint256 public threshold = 2; // 2/3 threshold

    function verifySignatures(
        bytes32 messageHash,
        bytes[] memory signatures
    ) public view returns (bool) {
        require(
            signatures.length >= (validatorList.length * threshold) / 3,
            "Insufficient signatures"
        );

        address[] memory signers = new address[](signatures.length);

        for (uint256 i = 0; i < signatures.length; i++) {
            address signer = recoverSigner(messageHash, signatures[i]);
            require(validators[signer].active, "Invalid validator");
            signers[i] = signer;
        }

        return true;
    }

    function recoverSigner(
        bytes32 messageHash,
        bytes memory signature
    ) internal pure returns (address) {
        bytes32 r;
        bytes32 s;
        uint8 v;

        assembly {
            r := mload(add(signature, 32))
            s := mload(add(signature, 64))
            v := byte(0, mload(add(signature, 96)))
        }

        return ecrecover(messageHash, v, r, s);
    }
}
```

### Challenge Period

All cross-chain messages have a challenge period:

```solidity
contract ChallengeManager {
    uint256 public constant CHALLENGE_PERIOD = 1 hours;

    struct Message {
        bytes32 messageHash;
        uint256 timestamp;
        MessageStatus status;
    }

    enum MessageStatus {
        PENDING,
        CHALLENGED,
        FINALIZED
    }

    mapping(bytes32 => Message) public messages;

    function submitMessage(bytes32 messageHash) external {
        messages[messageHash] = Message({
            messageHash: messageHash,
            timestamp: block.timestamp,
            status: MessageStatus.PENDING
        });
    }

    function challengeMessage(
        bytes32 messageHash,
        bytes memory fraudProof
    ) external {
        Message storage message = messages[messageHash];
        require(message.status == MessageStatus.PENDING, "Not pending");
        require(
            block.timestamp < message.timestamp + CHALLENGE_PERIOD,
            "Challenge period expired"
        );

        if (verifyFraudProof(messageHash, fraudProof)) {
            message.status = MessageStatus.CHALLENGED;
        }
    }

    function finalizeMessage(bytes32 messageHash) external {
        Message storage message = messages[messageHash];
        require(
            block.timestamp >= message.timestamp + CHALLENGE_PERIOD,
            "Challenge period not expired"
        );
        require(message.status == MessageStatus.PENDING, "Already finalized");

        message.status = MessageStatus.FINALIZED;
    }

    function verifyFraudProof(
        bytes32 messageHash,
        bytes memory fraudProof
    ) internal pure returns (bool) {
        // Verify fraud proof logic
        return true;
    }
}
```

---

## Bridge Mechanisms

### Lock-Mint Bridge

Used for transferring assets to chains where they don't natively exist.

```solidity
contract LockMintBridge {
    mapping(address => uint256) public lockedBalances;

    event Locked(
        address indexed token,
        address indexed sender,
        uint256 amount,
        uint256 destinationChainId,
        bytes32 messageId
    );

    function lock(
        address token,
        uint256 amount,
        uint256 destinationChainId,
        address recipient
    ) external returns (bytes32) {
        IERC20(token).transferFrom(msg.sender, address(this), amount);
        lockedBalances[token] += amount;

        bytes32 messageId = keccak256(
            abi.encodePacked(block.timestamp, msg.sender, amount)
        );

        emit Locked(token, msg.sender, amount, destinationChainId, messageId);

        return messageId;
    }

    function unlock(
        address token,
        address recipient,
        uint256 amount,
        bytes memory proof
    ) external {
        require(verifyProof(proof), "Invalid proof");
        require(lockedBalances[token] >= amount, "Insufficient locked balance");

        lockedBalances[token] -= amount;
        IERC20(token).transfer(recipient, amount);
    }

    function verifyProof(bytes memory proof) internal view returns (bool) {
        // Verify cross-chain proof
        return true;
    }
}
```

### Burn-Unlock Bridge

Used for transferring wrapped assets back to their native chain.

```solidity
contract BurnUnlockBridge {
    mapping(address => bool) public wrappedTokens;

    event Burned(
        address indexed token,
        address indexed sender,
        uint256 amount,
        uint256 destinationChainId,
        bytes32 messageId
    );

    function burn(
        address token,
        uint256 amount,
        uint256 destinationChainId,
        address recipient
    ) external returns (bytes32) {
        require(wrappedTokens[token], "Not a wrapped token");

        IWrappedToken(token).burn(msg.sender, amount);

        bytes32 messageId = keccak256(
            abi.encodePacked(block.timestamp, msg.sender, amount)
        );

        emit Burned(token, msg.sender, amount, destinationChainId, messageId);

        return messageId;
    }
}
```

### Atomic Swap

For trustless peer-to-peer cross-chain swaps.

```solidity
contract AtomicSwap {
    struct Swap {
        address initiator;
        address participant;
        address token;
        uint256 amount;
        bytes32 secretHash;
        uint256 lockTime;
        bool completed;
        bool refunded;
    }

    mapping(bytes32 => Swap) public swaps;

    function initiate(
        address participant,
        address token,
        uint256 amount,
        bytes32 secretHash,
        uint256 lockTime
    ) external returns (bytes32) {
        bytes32 swapId = keccak256(
            abi.encodePacked(msg.sender, participant, block.timestamp)
        );

        IERC20(token).transferFrom(msg.sender, address(this), amount);

        swaps[swapId] = Swap({
            initiator: msg.sender,
            participant: participant,
            token: token,
            amount: amount,
            secretHash: secretHash,
            lockTime: lockTime,
            completed: false,
            refunded: false
        });

        return swapId;
    }

    function redeem(bytes32 swapId, bytes32 secret) external {
        Swap storage swap = swaps[swapId];
        require(!swap.completed && !swap.refunded, "Swap already settled");
        require(msg.sender == swap.participant, "Not participant");
        require(
            keccak256(abi.encodePacked(secret)) == swap.secretHash,
            "Invalid secret"
        );

        swap.completed = true;
        IERC20(swap.token).transfer(swap.participant, swap.amount);
    }

    function refund(bytes32 swapId) external {
        Swap storage swap = swaps[swapId];
        require(!swap.completed && !swap.refunded, "Swap already settled");
        require(msg.sender == swap.initiator, "Not initiator");
        require(block.timestamp >= swap.lockTime, "Lock time not expired");

        swap.refunded = true;
        IERC20(swap.token).transfer(swap.initiator, swap.amount);
    }
}
```

---

## Consensus & Validation

### Validator Economics

#### Staking Requirements

- Minimum stake: 10,000 WIA tokens
- Lock period: 30 days
- Slashing conditions:
  - Double signing: 10% slash
  - Downtime: 1% slash
  - Fraud: 100% slash

#### Reward Distribution

```solidity
contract ValidatorRewards {
    uint256 public constant BLOCK_REWARD = 1 ether;
    uint256 public constant MESSAGE_FEE = 0.001 ether;

    mapping(address => uint256) public rewards;

    function distributeBlockReward(address[] memory validators) external {
        uint256 rewardPerValidator = BLOCK_REWARD / validators.length;

        for (uint256 i = 0; i < validators.length; i++) {
            rewards[validators[i]] += rewardPerValidator;
        }
    }

    function distributeMessageFee(
        address[] memory validators,
        uint256 fee
    ) external {
        uint256 feePerValidator = fee / validators.length;

        for (uint256 i = 0; i < validators.length; i++) {
            rewards[validators[i]] += feePerValidator;
        }
    }

    function claimRewards() external {
        uint256 amount = rewards[msg.sender];
        require(amount > 0, "No rewards");

        rewards[msg.sender] = 0;
        payable(msg.sender).transfer(amount);
    }
}
```

### Message Ordering

Messages are ordered by:
1. Source chain block number
2. Transaction index within block
3. Log index within transaction

```solidity
function compareMessages(
    CrossChainMessage memory a,
    CrossChainMessage memory b
) internal pure returns (int) {
    if (a.sourceChainId != b.sourceChainId) {
        return int(a.sourceChainId) - int(b.sourceChainId);
    }
    // Additional ordering logic
    return int(a.nonce) - int(b.nonce);
}
```

---

## Protocol Extensions

### LayerZero Integration

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

contract WIALayerZeroAdapter {
    ILayerZeroEndpoint public endpoint;

    function sendCrossChainMessage(
        uint16 dstChainId,
        bytes memory payload
    ) external payable {
        endpoint.send{value: msg.value}(
            dstChainId,
            abi.encodePacked(address(this)),
            payload,
            payable(msg.sender),
            address(0),
            bytes("")
        );
    }
}
```

### Cosmos IBC Integration

```solidity
contract WIAIBCAdapter {
    function sendIBCPacket(
        string memory channel,
        bytes memory data,
        uint64 timeoutTimestamp
    ) external {
        // IBC packet sending logic
    }

    function receiveIBCPacket(
        string memory channel,
        bytes memory data
    ) external {
        // IBC packet receiving logic
    }
}
```

### Polkadot XCM Integration

```solidity
contract WIAXCMAdapter {
    function sendXCM(
        uint32 paraId,
        bytes memory message
    ) external {
        // XCM message sending logic
    }
}
```

---

## Security Best Practices

### 1. Rate Limiting

```solidity
contract RateLimiter {
    mapping(address => uint256) public lastMessageTime;
    uint256 public constant MIN_MESSAGE_INTERVAL = 1 minutes;

    modifier rateLimit() {
        require(
            block.timestamp >= lastMessageTime[msg.sender] + MIN_MESSAGE_INTERVAL,
            "Rate limit exceeded"
        );
        lastMessageTime[msg.sender] = block.timestamp;
        _;
    }
}
```

### 2. Circuit Breaker

```solidity
contract CircuitBreaker {
    bool public paused = false;
    address public admin;

    modifier whenNotPaused() {
        require(!paused, "System paused");
        _;
    }

    function pause() external {
        require(msg.sender == admin, "Not admin");
        paused = true;
    }

    function unpause() external {
        require(msg.sender == admin, "Not admin");
        paused = false;
    }
}
```

### 3. Reentrancy Protection

```solidity
abstract contract ReentrancyGuard {
    uint256 private constant _NOT_ENTERED = 1;
    uint256 private constant _ENTERED = 2;
    uint256 private _status;

    constructor() {
        _status = _NOT_ENTERED;
    }

    modifier nonReentrant() {
        require(_status != _ENTERED, "Reentrant call");
        _status = _ENTERED;
        _;
        _status = _NOT_ENTERED;
    }
}
```

---

**弘益人間 · Benefit All Humanity**

© 2025 WIA - World Interoperability Alliance
MIT License
