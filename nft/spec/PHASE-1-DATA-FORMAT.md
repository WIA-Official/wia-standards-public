# WIA-FIN-009: NFT Standard - Phase 1: Data Format

## Overview

Phase 1 defines the foundational data structures, metadata schemas, and token format specifications for NFT implementation. This phase establishes the core standards that enable interoperability across platforms, marketplaces, and applications.

## Token Standards

### ERC-721: Non-Fungible Token Standard

ERC-721 is the foundational standard for non-fungible tokens on Ethereum, defining the minimum interface required for unique, indivisible tokens.

#### Core Interface

```solidity
interface IERC721 {
    event Transfer(address indexed from, address indexed to, uint256 indexed tokenId);
    event Approval(address indexed owner, address indexed approved, uint256 indexed tokenId);
    event ApprovalForAll(address indexed owner, address indexed operator, bool approved);

    function balanceOf(address owner) external view returns (uint256 balance);
    function ownerOf(uint256 tokenId) external view returns (address owner);
    function safeTransferFrom(address from, address to, uint256 tokenId, bytes calldata data) external;
    function safeTransferFrom(address from, address to, uint256 tokenId) external;
    function transferFrom(address from, address to, uint256 tokenId) external;
    function approve(address to, uint256 tokenId) external;
    function setApprovalForAll(address operator, bool approved) external;
    function getApproved(uint256 tokenId) external view returns (address operator);
    function isApprovedForAll(address owner, address operator) external view returns (bool);
}
```

#### Token Properties

- **Uniqueness**: Each token has a unique uint256 identifier within the contract
- **Indivisibility**: Tokens cannot be divided into fractional amounts
- **Ownership**: Single address owns each token at any given time
- **Transferability**: Tokens can be transferred between addresses with proper authorization

### ERC-1155: Multi-Token Standard

ERC-1155 enables efficient management of multiple token types (both fungible and non-fungible) within a single contract.

#### Core Interface

```solidity
interface IERC1155 {
    event TransferSingle(address indexed operator, address indexed from, address indexed to, uint256 id, uint256 value);
    event TransferBatch(address indexed operator, address indexed from, address indexed to, uint256[] ids, uint256[] values);
    event ApprovalForAll(address indexed account, address indexed operator, bool approved);
    event URI(string value, uint256 indexed id);

    function balanceOf(address account, uint256 id) external view returns (uint256);
    function balanceOfBatch(address[] calldata accounts, uint256[] calldata ids) external view returns (uint256[] memory);
    function setApprovalForAll(address operator, bool approved) external;
    function isApprovedForAll(address account, address operator) external view returns (bool);
    function safeTransferFrom(address from, address to, uint256 id, uint256 amount, bytes calldata data) external;
    function safeBatchTransferFrom(address from, address to, uint256[] calldata ids, uint256[] calldata amounts, bytes calldata data) external;
}
```

#### Advantages

- **Gas Efficiency**: Batch operations reduce transaction costs significantly
- **Flexibility**: Support for both fungible and non-fungible tokens in single contract
- **Simplified Management**: One contract handles entire token ecosystem

## Metadata Schema

### Standard Metadata Format

NFT metadata follows JSON schema for maximum compatibility:

```json
{
  "name": "Token Name",
  "description": "Detailed description supporting markdown",
  "image": "ipfs://QmHash.../image.png",
  "external_url": "https://project.com/token/1",
  "attributes": [
    {
      "trait_type": "Background",
      "value": "Blue"
    },
    {
      "trait_type": "Rarity",
      "value": "Legendary"
    },
    {
      "trait_type": "Power",
      "value": 95,
      "max_value": 100,
      "display_type": "boost_percentage"
    }
  ],
  "animation_url": "ipfs://QmHash.../animation.mp4",
  "background_color": "0066FF",
  "properties": {
    "created": "2025-01-15T10:30:00Z",
    "creator": "0x123...",
    "edition": "1/100"
  }
}
```

### Field Specifications

#### Required Fields

- **name** (string): Token title displayed in wallets and marketplaces
- **image** (URI): Primary visual representation (IPFS, HTTP, or data URI)
- **description** (string): Detailed description, supports markdown formatting

#### Optional Fields

- **external_url** (URI): Link to view item on project website
- **animation_url** (URI): Multimedia content (video, audio, 3D models)
- **background_color** (hex): Six-character hex color without '#' prefix
- **attributes** (array): Array of trait objects for filtering/display
- **properties** (object): Additional custom metadata

### Attribute Schema

```json
{
  "trait_type": "string",
  "value": "string | number",
  "display_type": "boost_number | boost_percentage | number | date",
  "max_value": 100,
  "trait_count": 42
}
```

#### Display Types

- **boost_number**: Numeric boost (e.g., +10)
- **boost_percentage**: Percentage boost (e.g., +5%)
- **number**: Standard numeric value
- **date**: Unix timestamp for date display

## IPFS Integration

### Content Addressing

IPFS (InterPlanetary File System) uses content-based addressing rather than location-based URLs:

```
ipfs://QmYwAPJzv5CZsnA625s3Xf2nemtYgPpHdWEz79ojWnPbdG
```

### Best Practices

1. **Pinning**: Use pinning services (Pinata, NFT.Storage, Infura) to ensure permanence
2. **CID Version**: Use CIDv1 for better compatibility
3. **Directory Structure**: Organize metadata and assets in logical directories
4. **Redundancy**: Pin content on multiple services for reliability

### Example Structure

```
/collection
  /metadata
    /1.json
    /2.json
    ...
  /images
    /1.png
    /2.png
    ...
  /animations
    /1.mp4
    /2.mp4
    ...
```

## URI Schemes

### Supported Protocols

- **IPFS**: `ipfs://QmHash...` - Decentralized content addressing
- **HTTPS**: `https://domain.com/path` - Traditional web hosting
- **Data URI**: `data:image/svg+xml;base64,...` - Inline encoded data
- **Arweave**: `ar://Hash...` - Permanent storage blockchain

### URI Construction

For ERC-721:
```solidity
function tokenURI(uint256 tokenId) public view returns (string memory) {
    return string(abi.encodePacked(baseURI, tokenId.toString(), ".json"));
}
```

For ERC-1155:
```solidity
function uri(uint256 tokenId) public view returns (string memory) {
    return string(abi.encodePacked(baseURI, "{id}.json"));
}
```

## Royalty Standard (ERC-2981)

### Interface

```solidity
interface IERC2981 {
    function royaltyInfo(
        uint256 tokenId,
        uint256 salePrice
    ) external view returns (
        address receiver,
        uint256 royaltyAmount
    );
}
```

### Implementation

```solidity
function royaltyInfo(uint256 tokenId, uint256 salePrice)
    public view returns (address, uint256)
{
    uint256 royaltyAmount = (salePrice * royaltyBasisPoints) / 10000;
    return (royaltyReceiver, royaltyAmount);
}
```

### Royalty Rates

- **Standard Range**: 5-10% (500-1000 basis points)
- **Premium Art**: 10-15% (1000-1500 basis points)
- **Gaming Assets**: 2.5-5% (250-500 basis points)
- **Music NFTs**: 10-15% (1000-1500 basis points)

## Collection Metadata

### Contract-Level Metadata

```json
{
  "name": "Collection Name",
  "description": "Collection description",
  "image": "ipfs://QmCollectionImage...",
  "external_link": "https://project.com",
  "seller_fee_basis_points": 750,
  "fee_recipient": "0x...",
  "properties": {
    "category": "art",
    "total_supply": 10000,
    "mint_date": "2025-01-15"
  }
}
```

## Data Validation

### Metadata Validation Rules

1. **Required Fields**: Presence of name, image, description
2. **URI Format**: Valid IPFS, HTTPS, or data URI
3. **Attribute Types**: Consistent trait_type and value types
4. **Image Specifications**: Recommended dimensions and file size
5. **JSON Validity**: Well-formed JSON structure

### Quality Checklist

- [ ] All required metadata fields present
- [ ] Images optimized for web (< 2MB)
- [ ] IPFS content pinned on reliable service
- [ ] Attributes follow consistent naming convention
- [ ] External URLs functional and secure (HTTPS)
- [ ] Metadata tested across major marketplaces
- [ ] High-resolution assets available for future use

## File Format Specifications

### Images

- **Formats**: PNG (preferred for art), JPG (photos), SVG (vector)
- **Dimensions**: Minimum 1000x1000px, recommended 2000x2000px
- **File Size**: Target < 2MB for standard display, < 10MB for high-res
- **Color Space**: sRGB for consistent display
- **Transparency**: PNG-24 for alpha channel support

### Animation

- **Video**: MP4 (H.264), WebM, maximum 50MB
- **Audio**: MP3, WAV, FLAC
- **3D Models**: GLTF, GLB for metaverse compatibility
- **Interactive**: HTML with embedded JavaScript

## Security Considerations

### Metadata Immutability

- **Frozen Metadata**: Set tokenURI during minting, prevent changes
- **Decentralized Storage**: Use IPFS/Arweave, avoid centralized servers
- **Backup Strategy**: Multiple pinning services for redundancy
- **Verification**: Provide tools for users to verify metadata integrity

### Content Security

- **IPFS Hashing**: Verify content matches CID
- **CORS Headers**: Configure for cross-origin access
- **Rate Limiting**: Implement for custom API endpoints
- **Access Control**: Restrict metadata update permissions

## Compliance and Standards

### OpenSea Compatibility

- Metadata must conform to OpenSea standard
- Collection metadata at contract level
- Rarity traits properly formatted
- External URL points to project website

### Multi-Marketplace Support

- Test metadata display across platforms
- Ensure attribute filtering works correctly
- Verify image rendering at various sizes
- Check animation playback compatibility

## Version Control

### Metadata Versioning

```json
{
  "version": "1.0.0",
  "schema": "https://schema.org/NFTMetadata/v1",
  "updated": "2025-01-15T10:30:00Z"
}
```

### Changelog Tracking

Maintain transparent record of metadata schema changes:
- Version numbers (semantic versioning)
- Change descriptions
- Migration guides for existing tokens
- Backward compatibility guarantees

## Future Extensions

### Planned Enhancements

- **Multi-resolution Images**: Serve optimal size based on context
- **Localization**: Multi-language metadata support
- **Dynamic Metadata**: Oracle-driven real-time updates
- **Cross-chain Standards**: Interoperability across blockchains
- **Enhanced Royalties**: Programmable royalty curves

---

**Status**: Draft
**Version**: 1.0.0
**Last Updated**: 2025-01-15
**Authors**: WIA Technical Committee

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
