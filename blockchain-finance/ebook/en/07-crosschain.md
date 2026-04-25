# Chapter 7: Cross-Chain Infrastructure

## Bridges, Interoperability Protocols, and Multi-Chain Standards

### Connecting the Fragmented Blockchain Ecosystem

---

## Overview

The blockchain ecosystem has evolved into a multi-chain world with liquidity, users, and applications fragmented across dozens of networks. This chapter covers the technical architecture, security considerations, and standards for enabling seamless cross-chain communication and value transfer.

---

## Cross-Chain Architecture

### Bridge Taxonomy

**Bridge Categories:**

| Type | Trust Model | Examples | Security |
|------|-------------|----------|----------|
| Lock-and-Mint | Custodial | WBTC, Multichain | Centralized risk |
| Burn-and-Mint | Native/Canonical | Circle CCTP | Issuer trust |
| Atomic Swap | Trustless | THORChain | Cryptographic |
| Light Client | Trustless | IBC, Near Rainbow | Consensus verification |
| Optimistic | Fraud Proof | Connext | Economic security |
| ZK | Validity Proof | zkBridge, Succinct | Cryptographic |

### Lock-and-Mint Bridge

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/token/ERC20/IERC20.sol";
import "@openzeppelin/contracts/security/ReentrancyGuard.sol";
import "@openzeppelin/contracts/access/AccessControl.sol";

/**
 * @title SourceChainBridge
 * @dev Lock tokens on source chain for bridging
 */
contract SourceChainBridge is ReentrancyGuard, AccessControl {
    bytes32 public constant RELAYER_ROLE = keccak256("RELAYER_ROLE");
    bytes32 public constant GUARDIAN_ROLE = keccak256("GUARDIAN_ROLE");

    struct BridgeRequest {
        address token;
        address sender;
        address recipient;
        uint256 amount;
        uint256 destinationChainId;
        uint256 nonce;
        uint256 timestamp;
        bool processed;
    }

    // Supported tokens
    mapping(address => bool) public supportedTokens;
    mapping(address => uint256) public lockedBalance;

    // Bridge requests
    mapping(bytes32 => BridgeRequest) public bridgeRequests;
    uint256 public nonce;

    // Rate limiting
    mapping(address => uint256) public dailyVolume;
    mapping(address => uint256) public lastVolumeReset;
    uint256 public dailyLimit;

    // Fees
    uint256 public bridgeFee; // Basis points
    address public feeRecipient;

    event TokensLocked(
        bytes32 indexed requestId,
        address indexed token,
        address indexed sender,
        address recipient,
        uint256 amount,
        uint256 destinationChainId,
        uint256 nonce
    );
    event TokensUnlocked(
        bytes32 indexed requestId,
        address indexed token,
        address recipient,
        uint256 amount
    );
    event BridgePaused(address indexed guardian);

    bool public paused;

    modifier whenNotPaused() {
        require(!paused, "Bridge is paused");
        _;
    }

    modifier rateLimited(address token, uint256 amount) {
        uint256 currentDay = block.timestamp / 1 days;
        if (lastVolumeReset[token] < currentDay) {
            dailyVolume[token] = 0;
            lastVolumeReset[token] = currentDay;
        }
        require(
            dailyVolume[token] + amount <= dailyLimit,
            "Daily limit exceeded"
        );
        dailyVolume[token] += amount;
        _;
    }

    constructor(uint256 _dailyLimit, uint256 _bridgeFee) {
        _grantRole(DEFAULT_ADMIN_ROLE, msg.sender);
        _grantRole(GUARDIAN_ROLE, msg.sender);
        dailyLimit = _dailyLimit;
        bridgeFee = _bridgeFee;
        feeRecipient = msg.sender;
    }

    /**
     * @dev Lock tokens for bridging to destination chain
     */
    function lockTokens(
        address token,
        uint256 amount,
        address recipient,
        uint256 destinationChainId
    ) external nonReentrant whenNotPaused rateLimited(token, amount) returns (bytes32 requestId) {
        require(supportedTokens[token], "Token not supported");
        require(amount > 0, "Amount must be positive");
        require(recipient != address(0), "Invalid recipient");

        // Calculate fee
        uint256 fee = (amount * bridgeFee) / 10000;
        uint256 netAmount = amount - fee;

        // Transfer tokens
        IERC20(token).transferFrom(msg.sender, address(this), amount);
        if (fee > 0) {
            IERC20(token).transfer(feeRecipient, fee);
        }

        // Create request
        requestId = keccak256(abi.encodePacked(
            token,
            msg.sender,
            recipient,
            netAmount,
            destinationChainId,
            nonce,
            block.timestamp
        ));

        bridgeRequests[requestId] = BridgeRequest({
            token: token,
            sender: msg.sender,
            recipient: recipient,
            amount: netAmount,
            destinationChainId: destinationChainId,
            nonce: nonce,
            timestamp: block.timestamp,
            processed: false
        });

        lockedBalance[token] += netAmount;
        nonce++;

        emit TokensLocked(
            requestId,
            token,
            msg.sender,
            recipient,
            netAmount,
            destinationChainId,
            nonce - 1
        );
    }

    /**
     * @dev Unlock tokens (for reverse bridge or cancellation)
     */
    function unlockTokens(
        bytes32 requestId,
        bytes[] calldata signatures
    ) external nonReentrant onlyRole(RELAYER_ROLE) {
        BridgeRequest storage request = bridgeRequests[requestId];
        require(!request.processed, "Already processed");

        // Verify multi-sig (simplified)
        require(
            _verifySignatures(requestId, signatures),
            "Invalid signatures"
        );

        request.processed = true;
        lockedBalance[request.token] -= request.amount;

        IERC20(request.token).transfer(request.recipient, request.amount);

        emit TokensUnlocked(
            requestId,
            request.token,
            request.recipient,
            request.amount
        );
    }

    /**
     * @dev Emergency pause by guardian
     */
    function emergencyPause() external onlyRole(GUARDIAN_ROLE) {
        paused = true;
        emit BridgePaused(msg.sender);
    }

    /**
     * @dev Verify multi-signature
     */
    function _verifySignatures(
        bytes32 requestId,
        bytes[] calldata signatures
    ) internal view returns (bool) {
        // Implement threshold signature verification
        // Simplified for illustration
        return signatures.length >= 3;
    }

    /**
     * @dev Add supported token
     */
    function addSupportedToken(address token) external onlyRole(DEFAULT_ADMIN_ROLE) {
        supportedTokens[token] = true;
    }
}

/**
 * @title DestinationChainBridge
 * @dev Mint wrapped tokens on destination chain
 */
contract DestinationChainBridge is ReentrancyGuard, AccessControl {
    bytes32 public constant RELAYER_ROLE = keccak256("RELAYER_ROLE");

    // Wrapped token contracts
    mapping(address => address) public wrappedTokens;

    // Processed requests (prevent replay)
    mapping(bytes32 => bool) public processedRequests;

    event TokensMinted(
        bytes32 indexed requestId,
        address indexed wrappedToken,
        address recipient,
        uint256 amount
    );

    /**
     * @dev Mint wrapped tokens based on lock proof
     */
    function mintTokens(
        bytes32 requestId,
        address sourceToken,
        address recipient,
        uint256 amount,
        bytes[] calldata signatures
    ) external nonReentrant onlyRole(RELAYER_ROLE) {
        require(!processedRequests[requestId], "Already processed");
        require(
            _verifyLockProof(requestId, sourceToken, recipient, amount, signatures),
            "Invalid proof"
        );

        processedRequests[requestId] = true;

        address wrappedToken = wrappedTokens[sourceToken];
        require(wrappedToken != address(0), "Wrapped token not configured");

        // Mint wrapped tokens
        IWrappedToken(wrappedToken).mint(recipient, amount);

        emit TokensMinted(requestId, wrappedToken, recipient, amount);
    }

    function _verifyLockProof(
        bytes32 requestId,
        address sourceToken,
        address recipient,
        uint256 amount,
        bytes[] calldata signatures
    ) internal view returns (bool) {
        // Verify signatures from bridge validators
        return signatures.length >= 3;
    }
}

interface IWrappedToken {
    function mint(address to, uint256 amount) external;
    function burn(address from, uint256 amount) external;
}
```

### Light Client Bridge (IBC-Style)

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

/**
 * @title LightClientBridge
 * @dev Trustless bridge using light client verification
 */
contract LightClientBridge {
    struct BlockHeader {
        bytes32 parentHash;
        bytes32 stateRoot;
        bytes32 transactionsRoot;
        bytes32 receiptsRoot;
        uint256 number;
        uint256 timestamp;
        bytes32 validatorSetHash;
    }

    struct ValidatorSet {
        address[] validators;
        uint256[] powers;
        uint256 totalPower;
    }

    // Light client state
    BlockHeader public latestHeader;
    ValidatorSet public currentValidatorSet;
    uint256 public trustedHeight;

    // Consensus parameters
    uint256 public constant VALIDATOR_THRESHOLD = 67; // 2/3 + 1

    // Processed packets
    mapping(bytes32 => bool) public processedPackets;

    event HeaderUpdated(uint256 indexed height, bytes32 stateRoot);
    event PacketReceived(bytes32 indexed packetId, bytes data);

    /**
     * @dev Update light client with new block header
     */
    function updateClient(
        BlockHeader calldata header,
        bytes[] calldata signatures
    ) external {
        // Verify header builds on trusted state
        require(
            header.number > latestHeader.number,
            "Header not newer"
        );

        // Verify validator signatures
        require(
            _verifyConsensus(header, signatures),
            "Insufficient consensus"
        );

        // Update state
        latestHeader = header;
        trustedHeight = header.number;

        // Check for validator set changes
        if (header.validatorSetHash != keccak256(abi.encode(currentValidatorSet))) {
            // Validator set update would be verified here
        }

        emit HeaderUpdated(header.number, header.stateRoot);
    }

    /**
     * @dev Verify and process cross-chain packet
     */
    function receivePacket(
        bytes calldata packetData,
        bytes calldata proof,
        uint256 proofHeight
    ) external {
        // Verify proof against trusted state root
        require(
            proofHeight <= trustedHeight,
            "Proof height too high"
        );

        bytes32 packetId = keccak256(packetData);
        require(!processedPackets[packetId], "Already processed");

        // Verify Merkle proof
        require(
            _verifyMerkleProof(
                packetData,
                proof,
                _getStateRootAtHeight(proofHeight)
            ),
            "Invalid proof"
        );

        processedPackets[packetId] = true;

        // Process packet
        _processPacket(packetData);

        emit PacketReceived(packetId, packetData);
    }

    /**
     * @dev Verify 2/3+ validator consensus
     */
    function _verifyConsensus(
        BlockHeader calldata header,
        bytes[] calldata signatures
    ) internal view returns (bool) {
        bytes32 headerHash = keccak256(abi.encode(header));
        uint256 signedPower = 0;

        for (uint256 i = 0; i < signatures.length; i++) {
            address signer = _recoverSigner(headerHash, signatures[i]);
            uint256 index = _getValidatorIndex(signer);
            if (index != type(uint256).max) {
                signedPower += currentValidatorSet.powers[index];
            }
        }

        return signedPower * 100 >= currentValidatorSet.totalPower * VALIDATOR_THRESHOLD;
    }

    /**
     * @dev Verify Merkle proof against state root
     */
    function _verifyMerkleProof(
        bytes calldata data,
        bytes calldata proof,
        bytes32 root
    ) internal pure returns (bool) {
        // Implement Merkle proof verification
        // This would verify the inclusion of data in the state trie
        return true; // Simplified
    }

    function _getStateRootAtHeight(uint256 height) internal view returns (bytes32) {
        // Return state root for height
        return latestHeader.stateRoot;
    }

    function _processPacket(bytes calldata packetData) internal {
        // Decode and execute packet
    }

    function _recoverSigner(bytes32 hash, bytes calldata sig) internal pure returns (address) {
        // ECDSA recovery
        return address(0);
    }

    function _getValidatorIndex(address validator) internal view returns (uint256) {
        for (uint256 i = 0; i < currentValidatorSet.validators.length; i++) {
            if (currentValidatorSet.validators[i] == validator) {
                return i;
            }
        }
        return type(uint256).max;
    }
}
```

---

## Messaging Protocols

### LayerZero Integration

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@layerzerolabs/contracts/interfaces/ILayerZeroEndpoint.sol";
import "@layerzerolabs/contracts/interfaces/ILayerZeroReceiver.sol";

/**
 * @title CrossChainToken
 * @dev Omnichain Fungible Token using LayerZero
 */
contract CrossChainToken is ILayerZeroReceiver {
    ILayerZeroEndpoint public lzEndpoint;

    mapping(address => uint256) public balances;
    uint256 public totalSupply;

    // Trusted remote contracts per chain
    mapping(uint16 => bytes) public trustedRemotes;

    // Message types
    uint8 public constant TYPE_TRANSFER = 1;
    uint8 public constant TYPE_MINT = 2;

    event CrossChainTransfer(
        uint16 indexed dstChainId,
        address indexed from,
        bytes indexed to,
        uint256 amount
    );
    event CrossChainReceive(
        uint16 indexed srcChainId,
        bytes indexed from,
        address to,
        uint256 amount
    );

    constructor(address _lzEndpoint) {
        lzEndpoint = ILayerZeroEndpoint(_lzEndpoint);
    }

    /**
     * @dev Send tokens to another chain
     */
    function sendTokens(
        uint16 dstChainId,
        bytes calldata toAddress,
        uint256 amount
    ) external payable {
        require(balances[msg.sender] >= amount, "Insufficient balance");
        require(trustedRemotes[dstChainId].length > 0, "Remote not trusted");

        // Burn tokens on source chain
        balances[msg.sender] -= amount;
        totalSupply -= amount;

        // Encode payload
        bytes memory payload = abi.encode(
            TYPE_TRANSFER,
            msg.sender,
            toAddress,
            amount
        );

        // Estimate fees
        (uint256 nativeFee, ) = lzEndpoint.estimateFees(
            dstChainId,
            address(this),
            payload,
            false,
            bytes("")
        );
        require(msg.value >= nativeFee, "Insufficient fee");

        // Send cross-chain message
        lzEndpoint.send{value: nativeFee}(
            dstChainId,
            trustedRemotes[dstChainId],
            payload,
            payable(msg.sender),
            address(0),
            bytes("")
        );

        emit CrossChainTransfer(dstChainId, msg.sender, toAddress, amount);
    }

    /**
     * @dev Receive cross-chain message
     */
    function lzReceive(
        uint16 srcChainId,
        bytes calldata srcAddress,
        uint64 nonce,
        bytes calldata payload
    ) external override {
        require(msg.sender == address(lzEndpoint), "Only endpoint");
        require(
            keccak256(srcAddress) == keccak256(trustedRemotes[srcChainId]),
            "Untrusted source"
        );

        // Decode payload
        (uint8 msgType, , bytes memory toAddress, uint256 amount) = abi.decode(
            payload,
            (uint8, address, bytes, uint256)
        );

        if (msgType == TYPE_TRANSFER) {
            // Mint tokens on destination chain
            address to = _bytesToAddress(toAddress);
            balances[to] += amount;
            totalSupply += amount;

            emit CrossChainReceive(srcChainId, srcAddress, to, amount);
        }
    }

    /**
     * @dev Set trusted remote contract
     */
    function setTrustedRemote(
        uint16 chainId,
        bytes calldata remoteAddress
    ) external {
        trustedRemotes[chainId] = remoteAddress;
    }

    /**
     * @dev Estimate cross-chain fee
     */
    function estimateFee(
        uint16 dstChainId,
        bytes calldata toAddress,
        uint256 amount
    ) external view returns (uint256 nativeFee) {
        bytes memory payload = abi.encode(
            TYPE_TRANSFER,
            msg.sender,
            toAddress,
            amount
        );

        (nativeFee, ) = lzEndpoint.estimateFees(
            dstChainId,
            address(this),
            payload,
            false,
            bytes("")
        );
    }

    function _bytesToAddress(bytes memory b) internal pure returns (address) {
        address addr;
        assembly {
            addr := mload(add(b, 20))
        }
        return addr;
    }
}
```

### Chainlink CCIP

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import {IRouterClient} from "@chainlink/contracts-ccip/src/v0.8/ccip/interfaces/IRouterClient.sol";
import {Client} from "@chainlink/contracts-ccip/src/v0.8/ccip/libraries/Client.sol";
import {CCIPReceiver} from "@chainlink/contracts-ccip/src/v0.8/ccip/applications/CCIPReceiver.sol";

/**
 * @title CCIPBridge
 * @dev Cross-chain bridge using Chainlink CCIP
 */
contract CCIPBridge is CCIPReceiver {
    IRouterClient public router;
    IERC20 public token;

    // Destination chain selectors
    mapping(uint64 => address) public destinationContracts;

    // Message tracking
    mapping(bytes32 => bool) public processedMessages;

    event MessageSent(
        bytes32 indexed messageId,
        uint64 indexed destinationChainSelector,
        address receiver,
        uint256 amount,
        uint256 fees
    );
    event MessageReceived(
        bytes32 indexed messageId,
        uint64 indexed sourceChainSelector,
        address sender,
        uint256 amount
    );

    constructor(address _router, address _token) CCIPReceiver(_router) {
        router = IRouterClient(_router);
        token = IERC20(_token);
    }

    /**
     * @dev Send tokens to another chain via CCIP
     */
    function sendTokens(
        uint64 destinationChainSelector,
        address receiver,
        uint256 amount
    ) external returns (bytes32 messageId) {
        require(
            destinationContracts[destinationChainSelector] != address(0),
            "Destination not configured"
        );

        // Transfer tokens to this contract
        token.transferFrom(msg.sender, address(this), amount);

        // Approve router
        token.approve(address(router), amount);

        // Create token amount struct
        Client.EVMTokenAmount[] memory tokenAmounts = new Client.EVMTokenAmount[](1);
        tokenAmounts[0] = Client.EVMTokenAmount({
            token: address(token),
            amount: amount
        });

        // Create message
        Client.EVM2AnyMessage memory message = Client.EVM2AnyMessage({
            receiver: abi.encode(receiver),
            data: abi.encode(msg.sender, amount),
            tokenAmounts: tokenAmounts,
            extraArgs: Client._argsToBytes(
                Client.EVMExtraArgsV1({gasLimit: 200_000})
            ),
            feeToken: address(0) // Pay in native token
        });

        // Get fee
        uint256 fees = router.getFee(destinationChainSelector, message);
        require(address(this).balance >= fees, "Insufficient fees");

        // Send message
        messageId = router.ccipSend{value: fees}(
            destinationChainSelector,
            message
        );

        emit MessageSent(
            messageId,
            destinationChainSelector,
            receiver,
            amount,
            fees
        );
    }

    /**
     * @dev Handle received CCIP message
     */
    function _ccipReceive(
        Client.Any2EVMMessage memory message
    ) internal override {
        bytes32 messageId = message.messageId;
        require(!processedMessages[messageId], "Already processed");
        processedMessages[messageId] = true;

        // Decode sender and amount
        (address sender, uint256 amount) = abi.decode(
            message.data,
            (address, uint256)
        );

        // Tokens are automatically transferred by CCIP

        emit MessageReceived(
            messageId,
            message.sourceChainSelector,
            sender,
            amount
        );
    }

    /**
     * @dev Estimate CCIP fee
     */
    function estimateFee(
        uint64 destinationChainSelector,
        address receiver,
        uint256 amount
    ) external view returns (uint256) {
        Client.EVMTokenAmount[] memory tokenAmounts = new Client.EVMTokenAmount[](1);
        tokenAmounts[0] = Client.EVMTokenAmount({
            token: address(token),
            amount: amount
        });

        Client.EVM2AnyMessage memory message = Client.EVM2AnyMessage({
            receiver: abi.encode(receiver),
            data: abi.encode(msg.sender, amount),
            tokenAmounts: tokenAmounts,
            extraArgs: Client._argsToBytes(
                Client.EVMExtraArgsV1({gasLimit: 200_000})
            ),
            feeToken: address(0)
        });

        return router.getFee(destinationChainSelector, message);
    }

    /**
     * @dev Configure destination contract
     */
    function setDestinationContract(
        uint64 chainSelector,
        address contractAddress
    ) external {
        destinationContracts[chainSelector] = contractAddress;
    }

    receive() external payable {}
}
```

---

## Intent-Based Bridging

### Cross-Chain Intents

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

/**
 * @title IntentBridge
 * @dev Intent-based cross-chain settlement
 */
contract IntentBridge {
    struct Intent {
        address user;
        address sourceToken;
        address destinationToken;
        uint256 sourceAmount;
        uint256 minDestinationAmount;
        uint256 sourceChainId;
        uint256 destinationChainId;
        address destinationAddress;
        uint256 deadline;
        uint256 nonce;
        bytes32 intentId;
        IntentStatus status;
    }

    enum IntentStatus { Pending, Filled, Cancelled, Expired }

    // Active intents
    mapping(bytes32 => Intent) public intents;

    // Solver deposits
    mapping(address => uint256) public solverDeposits;
    uint256 public minSolverDeposit;

    // Fill tracking
    mapping(bytes32 => address) public filledBy;
    mapping(bytes32 => uint256) public filledAmount;

    event IntentCreated(
        bytes32 indexed intentId,
        address indexed user,
        uint256 sourceAmount,
        uint256 minDestinationAmount
    );
    event IntentFilled(
        bytes32 indexed intentId,
        address indexed solver,
        uint256 destinationAmount
    );
    event IntentSettled(bytes32 indexed intentId);

    /**
     * @dev Create cross-chain intent
     */
    function createIntent(
        address sourceToken,
        address destinationToken,
        uint256 sourceAmount,
        uint256 minDestinationAmount,
        uint256 destinationChainId,
        address destinationAddress,
        uint256 deadline
    ) external returns (bytes32 intentId) {
        require(deadline > block.timestamp, "Invalid deadline");

        intentId = keccak256(abi.encodePacked(
            msg.sender,
            sourceToken,
            destinationToken,
            sourceAmount,
            minDestinationAmount,
            block.chainid,
            destinationChainId,
            block.timestamp,
            block.number
        ));

        // Lock source tokens
        IERC20(sourceToken).transferFrom(msg.sender, address(this), sourceAmount);

        intents[intentId] = Intent({
            user: msg.sender,
            sourceToken: sourceToken,
            destinationToken: destinationToken,
            sourceAmount: sourceAmount,
            minDestinationAmount: minDestinationAmount,
            sourceChainId: block.chainid,
            destinationChainId: destinationChainId,
            destinationAddress: destinationAddress,
            deadline: deadline,
            nonce: block.timestamp,
            intentId: intentId,
            status: IntentStatus.Pending
        });

        emit IntentCreated(
            intentId,
            msg.sender,
            sourceAmount,
            minDestinationAmount
        );
    }

    /**
     * @dev Fill intent on destination chain
     * Called by solver who fronts the tokens
     */
    function fillIntent(
        bytes32 intentId,
        uint256 destinationAmount
    ) external {
        Intent storage intent = intents[intentId];
        require(intent.status == IntentStatus.Pending, "Not pending");
        require(block.timestamp <= intent.deadline, "Expired");
        require(
            destinationAmount >= intent.minDestinationAmount,
            "Below minimum"
        );
        require(
            solverDeposits[msg.sender] >= minSolverDeposit,
            "Insufficient solver deposit"
        );

        intent.status = IntentStatus.Filled;
        filledBy[intentId] = msg.sender;
        filledAmount[intentId] = destinationAmount;

        // Transfer tokens to user on destination chain
        IERC20(intent.destinationToken).transferFrom(
            msg.sender,
            intent.destinationAddress,
            destinationAmount
        );

        emit IntentFilled(intentId, msg.sender, destinationAmount);
    }

    /**
     * @dev Settle intent and release source tokens to solver
     * Called after cross-chain verification
     */
    function settleIntent(
        bytes32 intentId,
        bytes calldata proof
    ) external {
        Intent storage intent = intents[intentId];
        require(intent.status == IntentStatus.Filled, "Not filled");

        // Verify fill on destination chain
        require(_verifyFill(intentId, proof), "Invalid proof");

        address solver = filledBy[intentId];

        // Release source tokens to solver
        IERC20(intent.sourceToken).transfer(solver, intent.sourceAmount);

        emit IntentSettled(intentId);
    }

    /**
     * @dev Cancel expired intent
     */
    function cancelIntent(bytes32 intentId) external {
        Intent storage intent = intents[intentId];
        require(intent.user == msg.sender, "Not owner");
        require(intent.status == IntentStatus.Pending, "Not pending");
        require(block.timestamp > intent.deadline, "Not expired");

        intent.status = IntentStatus.Expired;

        // Return tokens to user
        IERC20(intent.sourceToken).transfer(msg.sender, intent.sourceAmount);
    }

    /**
     * @dev Solver deposit for slashing
     */
    function depositSolverBond() external payable {
        solverDeposits[msg.sender] += msg.value;
    }

    function _verifyFill(
        bytes32 intentId,
        bytes calldata proof
    ) internal view returns (bool) {
        // Verify cross-chain proof of fill
        return true; // Simplified
    }
}
```

---

## Security Considerations

### Bridge Security Best Practices

**Security Checklist:**

| Category | Requirement | Priority |
|----------|-------------|----------|
| Consensus | Multi-sig or threshold signatures | Critical |
| Validation | Merkle proof verification | Critical |
| Rate Limiting | Daily volume caps | High |
| Monitoring | Real-time anomaly detection | High |
| Circuit Breaker | Emergency pause capability | Critical |
| Upgrade | Timelock on contract upgrades | High |
| Audit | Multiple independent audits | Critical |
| Insurance | Coverage for bridge assets | High |

### Historical Bridge Exploits

| Bridge | Date | Loss | Vulnerability |
|--------|------|------|---------------|
| Ronin | Mar 2022 | $625M | Compromised validator keys |
| Wormhole | Feb 2022 | $320M | Signature verification bypass |
| Nomad | Aug 2022 | $190M | Message validation flaw |
| Harmony | Jun 2022 | $100M | Compromised multi-sig |
| Multichain | Jul 2023 | $125M | CEO key compromise |

---

## Key Takeaways

1. **Bridge architectures** range from custodial lock-and-mint to trustless light client verification
2. **Light client bridges** provide highest security through consensus verification
3. **Messaging protocols** like LayerZero and CCIP enable arbitrary cross-chain communication
4. **Intent-based bridging** improves UX by having solvers front liquidity
5. **Security requires** multi-sig, rate limiting, monitoring, and circuit breakers
6. **Historical exploits** highlight risks of key compromise and validation flaws

## Review Questions

1. What are the different bridge architecture types and their trust assumptions?
2. How does a light client bridge achieve trustless verification?
3. How do messaging protocols like LayerZero enable cross-chain communication?
4. What is intent-based bridging and how does it improve user experience?
5. What security measures should cross-chain bridges implement?
6. What were the root causes of major bridge exploits?

---

**Next Chapter Preview:** Chapter 8 covers practical implementation including development environment setup, smart contract deployment, and operations best practices.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity · Democratize Finance

