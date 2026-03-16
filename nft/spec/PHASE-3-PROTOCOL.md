# WIA-FIN-009: NFT Standard - Phase 3: Protocol

## Overview

Phase 3 defines blockchain protocols, smart contract patterns, security standards, and on-chain mechanisms for NFT systems.

## Smart Contract Architecture

### Modular Design Pattern

```solidity
// Core NFT Contract
contract NFTCore is ERC721, ERC721URIStorage, ERC2981 {
    // Base functionality
}

// Minting Module
contract Minting is NFTCore, Ownable {
    uint256 public mintPrice;
    uint256 public maxSupply;

    function mint(address to, string memory uri) public payable {
        require(msg.value >= mintPrice, "Insufficient payment");
        require(totalSupply() < maxSupply, "Max supply reached");
        _safeMint(to, nextTokenId());
        _setTokenURI(tokenId, uri);
    }
}

// Marketplace Module
contract Marketplace {
    struct Listing {
        address seller;
        uint256 price;
        bool active;
    }

    mapping(address => mapping(uint256 => Listing)) public listings;
}
```

## Gas Optimization Strategies

### Storage Optimization

```solidity
// Bad: Multiple storage variables
uint8 status;      // 1 byte + 31 bytes padding
uint256 tokenId;   // 32 bytes
address owner;     // 20 bytes + 12 bytes padding

// Good: Packed into single slot
struct TokenData {
    uint8 status;     // 1 byte
    uint88 tokenId;   // 11 bytes
    address owner;    // 20 bytes
    // Total: 32 bytes = 1 slot
}
```

### Batch Operations

```solidity
function batchMint(address[] calldata recipients, string[] calldata uris)
    external onlyOwner
{
    require(recipients.length == uris.length, "Length mismatch");
    for (uint256 i = 0; i < recipients.length; i++) {
        _safeMint(recipients[i], _nextTokenId());
        _setTokenURI(_currentTokenId(), uris[i]);
    }
}
```

### Lazy Minting

```solidity
struct Voucher {
    uint256 tokenId;
    uint256 price;
    string uri;
    bytes signature;
}

function lazyMint(Voucher calldata voucher) external payable {
    require(_verify(voucher), "Invalid signature");
    require(msg.value >= voucher.price, "Insufficient payment");
    _safeMint(msg.sender, voucher.tokenId);
    _setTokenURI(voucher.tokenId, voucher.uri);
}
```

## Security Protocols

### Reentrancy Protection

```solidity
import "@openzeppelin/contracts/security/ReentrancyGuard.sol";

contract SecureNFT is ERC721, ReentrancyGuard {
    function mint() external payable nonReentrant {
        // Protected from reentrancy attacks
        _safeMint(msg.sender, nextTokenId());
        payable(treasury).transfer(msg.value);
    }
}
```

### Access Control

```solidity
import "@openzeppelin/contracts/access/AccessControl.sol";

contract ManagedNFT is ERC721, AccessControl {
    bytes32 public constant MINTER_ROLE = keccak256("MINTER_ROLE");
    bytes32 public constant BURNER_ROLE = keccak256("BURNER_ROLE");

    function mint(address to, uint256 tokenId)
        public onlyRole(MINTER_ROLE)
    {
        _safeMint(to, tokenId);
    }

    function burn(uint256 tokenId)
        public onlyRole(BURNER_ROLE)
    {
        _burn(tokenId);
    }
}
```

### Signature Verification

```solidity
import "@openzeppelin/contracts/utils/cryptography/ECDSA.sol";

contract SignedMint is ERC721 {
    using ECDSA for bytes32;

    address public signer;

    function verifySignature(
        address recipient,
        uint256 tokenId,
        bytes memory signature
    ) public view returns (bool) {
        bytes32 hash = keccak256(abi.encodePacked(recipient, tokenId));
        address recovered = hash.toEthSignedMessageHash().recover(signature);
        return recovered == signer;
    }
}
```

## Royalty Implementation

### ERC-2981 Standard

```solidity
import "@openzeppelin/contracts/token/common/ERC2981.sol";

contract RoyaltyNFT is ERC721, ERC2981 {
    constructor() ERC721("MyNFT", "MNFT") {
        // 7.5% default royalty
        _setDefaultRoyalty(msg.sender, 750);
    }

    function setTokenRoyalty(
        uint256 tokenId,
        address receiver,
        uint96 feeNumerator
    ) external onlyOwner {
        _setTokenRoyalty(tokenId, receiver, feeNumerator);
    }
}
```

### Payment Splitter

```solidity
import "@openzeppelin/contracts/finance/PaymentSplitter.sol";

contract TeamRoyalty is PaymentSplitter {
    constructor(
        address[] memory payees,
        uint256[] memory shares
    ) PaymentSplitter(payees, shares) {}
}
```

## Cross-Chain Protocols

### Bridge Architecture

```solidity
interface IBridge {
    function lockNFT(
        address nftContract,
        uint256 tokenId,
        uint256 targetChain
    ) external;

    function unlockNFT(
        address nftContract,
        uint256 tokenId,
        bytes calldata proof
    ) external;
}
```

### Message Passing

```solidity
// LayerZero integration
contract CrossChainNFT is ERC721, ILayerZeroReceiver {
    function sendNFT(
        uint16 dstChainId,
        address to,
        uint256 tokenId
    ) external payable {
        _burn(tokenId);
        bytes memory payload = abi.encode(to, tokenId);
        lzEndpoint.send{value: msg.value}(
            dstChainId,
            trustedRemote,
            payload,
            payable(msg.sender),
            address(0),
            bytes("")
        );
    }
}
```

## Metadata On-Chain

### Fully On-Chain NFTs

```solidity
contract OnChainNFT is ERC721 {
    mapping(uint256 => string) private _svgData;

    function tokenURI(uint256 tokenId)
        public view override returns (string memory)
    {
        string memory svg = _svgData[tokenId];
        string memory json = Base64.encode(
            bytes(string(abi.encodePacked(
                '{"name": "Token #', tokenId.toString(), '",',
                '"image": "data:image/svg+xml;base64,',
                Base64.encode(bytes(svg)), '"}'
            )))
        );
        return string(abi.encodePacked('data:application/json;base64,', json));
    }
}
```

## Upgradeability Patterns

### Proxy Pattern

```solidity
import "@openzeppelin/contracts/proxy/transparent/TransparentUpgradeableProxy.sol";

// Implementation contract
contract NFTImplementation is ERC721Upgradeable {
    function initialize(string memory name, string memory symbol)
        public initializer
    {
        __ERC721_init(name, symbol);
    }
}

// Deploy proxy
TransparentUpgradeableProxy proxy = new TransparentUpgradeableProxy(
    implementation,
    admin,
    initializeData
);
```

## Consensus Mechanisms

### Proof of Ownership

```solidity
function proveOwnership(
    uint256 tokenId,
    bytes calldata signature
) external view returns (bool) {
    address owner = ownerOf(tokenId);
    bytes32 message = keccak256(abi.encodePacked(
        "Prove ownership of token",
        tokenId
    ));
    address signer = message.toEthSignedMessageHash().recover(signature);
    return signer == owner;
}
```

## State Channels

### Off-Chain Trading

```solidity
struct TradeProposal {
    address seller;
    address buyer;
    uint256 tokenId;
    uint256 price;
    uint256 nonce;
    bytes sellerSignature;
    bytes buyerSignature;
}

function settleOffChainTrade(TradeProposal calldata trade)
    external
{
    require(verify(trade), "Invalid signatures");
    _transfer(trade.seller, trade.buyer, trade.tokenId);
}
```

## Testing Requirements

### Unit Tests

```javascript
describe("NFT Contract", function() {
    it("Should mint NFT correctly", async function() {
        const tokenId = await nft.mint(owner.address, "ipfs://...");
        expect(await nft.ownerOf(tokenId)).to.equal(owner.address);
    });

    it("Should enforce max supply", async function() {
        await expect(
            nft.mint(owner.address, "ipfs://...")
        ).to.be.revertedWith("Max supply reached");
    });
});
```

### Integration Tests

```javascript
describe("Marketplace Integration", function() {
    it("Should list and sell NFT", async function() {
        await nft.mint(seller.address, "ipfs://...");
        await marketplace.list(nft.address, tokenId, price);
        await marketplace.buy(listingId, {value: price});
        expect(await nft.ownerOf(tokenId)).to.equal(buyer.address);
    });
});
```

## Audit Requirements

### Security Checklist

- [ ] Reentrancy protection on all state-changing functions
- [ ] Access control properly implemented
- [ ] Integer overflow/underflow prevented
- [ ] External call safety verified
- [ ] Gas optimization reviewed
- [ ] Upgrade mechanism secured
- [ ] Emergency pause functionality
- [ ] Event emission for all state changes

### Third-Party Audits

Recommended audit firms:
- OpenZeppelin
- Trail of Bits
- ConsenSys Diligence
- Certik
- Quantstamp

## Deployment Checklist

- [ ] All tests passing (unit, integration, e2e)
- [ ] Security audit completed
- [ ] Gas costs optimized
- [ ] Contract verified on block explorer
- [ ] Multisig wallet configured
- [ ] Emergency procedures documented
- [ ] Monitoring and alerts set up

---

**Status**: Draft
**Version**: 1.0.0
**Last Updated**: 2025-01-15

© 2025 SmileStory Inc. / WIA
