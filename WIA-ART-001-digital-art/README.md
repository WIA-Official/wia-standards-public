# 🎨 WIA-ART-001: Digital Art Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-ART-001
> **Version:** 1.0.0
> **Status:** Active
> **Category:** Digital Arts & Creative Standards
> **Color:** Rose (#E11D48)

---

## 🌟 Overview

The WIA-ART-001 standard defines comprehensive formats, metadata schemas, and protocols for creating, distributing, and preserving digital artworks. It ensures interoperability across creative tools, marketplaces, and archival systems while protecting artist rights and maintaining provenance.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard empowers artists worldwide with universal tools for creative expression and proper attribution.

## 🎯 Key Features

- **Universal Format Support**: Raster, vector, 3D, and procedural art formats
- **Rich Metadata Schema**: EXIF, IPTC, XMP, and blockchain provenance
- **Rights Management**: Copyright, licensing, and attribution tracking
- **Style Classification**: Taxonomies for movements, styles, and techniques
- **Quality Standards**: Resolution, color depth, and archival specifications
- **Interoperability**: Seamless integration with creative tools and platforms

## 📊 Core Components

### 1. Supported Formats

**Raster Graphics:**
- PNG (Lossless, 8/16/32-bit)
- JPEG (Lossy, optimized for photos)
- TIFF (Archival quality)
- WebP, AVIF (Modern web formats)

**Vector Graphics:**
- SVG (Scalable Vector Graphics)
- PDF, EPS, AI (Print-ready)

**3D Assets:**
- glTF 2.0 (PBR materials)
- OBJ, FBX, USDZ

**Animation:**
- GIF, APNG, WebM, MP4

### 2. Color Management

- **Color Spaces**: sRGB, Adobe RGB, Display P3, ProPhoto RGB
- **Bit Depth**: 8-bit (standard), 16-bit (professional), 32-bit (HDR)
- **ICC Profile**: Embedded color profile required
- **Gamut Mapping**: Perceptual, relative colorimetric, absolute colorimetric

### 3. Metadata Standards

```json
{
  "@context": "https://wia.org/standards/ART-001/v1",
  "@type": "DigitalArtwork",
  "id": "uuid:12345678-1234-1234-1234-123456789abc",
  "title": "Sunset Over Mountains",
  "artist": {
    "name": "Jane Doe",
    "id": "did:wia:artist:jane-doe",
    "url": "https://artist.example.com"
  },
  "medium": "digital-painting",
  "created": "2025-01-15T10:30:00Z",
  "dimensions": {
    "width": 4096,
    "height": 2160,
    "unit": "px",
    "dpi": 300
  },
  "format": {
    "mimeType": "image/png",
    "colorSpace": "Adobe RGB",
    "bitDepth": 16,
    "iccProfile": "Adobe RGB (1998)"
  },
  "rights": {
    "license": "CC BY-SA 4.0",
    "copyright": "© 2025 Jane Doe",
    "attribution": "Jane Doe (artist.example.com)"
  },
  "provenance": {
    "blockchain": "ethereum",
    "contract": "0x...",
    "tokenId": "123",
    "hash": "QmX..."
  },
  "tags": ["landscape", "sunset", "mountains", "digital-art"]
}
```

## 🔧 Installation

### Quick Start

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/WIA-ART-001-digital-art

# Run installation script
./install.sh

# Verify installation
wia-art-001 --version
```

### TypeScript SDK

```bash
npm install @wia/art-001
# or
yarn add @wia/art-001
```

## 💻 Usage

### TypeScript Example

```typescript
import { DigitalArt, MetadataBuilder } from '@wia/art-001';

// Create new digital artwork
const artwork = new DigitalArt({
  title: "Sunset Over Mountains",
  artist: "Jane Doe",
  medium: "digital-painting",
  format: "PNG",
  dimensions: { width: 4096, height: 2160 },
  colorSpace: "Adobe RGB",
  bitDepth: 16,
  license: "CC BY-SA 4.0"
});

// Add metadata
const metadata = new MetadataBuilder()
  .setCopyright("© 2025 Jane Doe")
  .setCreationDate(new Date())
  .addTag("landscape")
  .addTag("sunset")
  .setICCProfile("Adobe RGB (1998)")
  .build();

artwork.setMetadata(metadata);

// Validate compliance
const validation = artwork.validate();
console.log(validation.isCompliant); // true

// Export with metadata
await artwork.export({
  path: "./output/sunset.png",
  embedMetadata: true,
  includeICCProfile: true
});
```

### CLI Tool

```bash
# Create new artwork metadata
wia-art-001 create \
  --title "Sunset Over Mountains" \
  --artist "Jane Doe" \
  --format PNG \
  --dimensions 4096x2160 \
  --color-space "Adobe RGB"

# Validate artwork
wia-art-001 validate artwork.png

# Extract metadata
wia-art-001 metadata extract artwork.png

# Convert formats
wia-art-001 convert \
  --input artwork.tiff \
  --output artwork.png \
  --color-space sRGB

# Generate license
wia-art-001 license generate \
  --type "CC BY-SA 4.0" \
  --holder "Jane Doe" \
  --year 2025
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [PHASE-1: Data Format](./spec/PHASE-1-DATA-FORMAT.md) | File formats and data structures |
| [PHASE-2: API Interface](./spec/PHASE-2-API-INTERFACE.md) | SDK and API specifications |
| [PHASE-3: Protocol](./spec/PHASE-3-PROTOCOL.md) | Communication protocols |
| [PHASE-4: Integration](./spec/PHASE-4-INTEGRATION.md) | Platform integration guide |

## 🎨 Use Cases

1. **NFT Marketplaces**: Standardized metadata for digital art trading
2. **Museum Archives**: Long-term preservation with full provenance
3. **Creative Software**: Integration with Adobe, Procreate, Blender
4. **Print Services**: Color-accurate digital-to-physical conversion
5. **Social Platforms**: Artist portfolios with proper attribution
6. **Licensing**: Automated rights management and usage tracking

## 🌐 WIA Integration

This standard integrates with:

- **WIA-INTENT**: Intent-based art search and discovery
- **WIA-OMNI-API**: Universal API gateway
- **WIA-SOCIAL**: Artist communities and sharing
- **WIA-BLOCKCHAIN**: Provenance and authenticity verification
- **WIA-ART-006**: Art authentication protocols

## 🔬 Technical Specifications

### Resolution Standards

- **Web Display**: 72-96 DPI, sRGB
- **Professional Print**: 300+ DPI, Adobe RGB/CMYK
- **Archival**: 600+ DPI, ProPhoto RGB, 16-bit minimum
- **Large Format**: 150-300 DPI depending on viewing distance

### File Size Guidelines

- **Thumbnail**: < 100 KB (JPEG/WebP)
- **Preview**: < 500 KB (PNG/WebP)
- **Full Resolution**: < 50 MB (PNG/TIFF)
- **Archival**: Uncompressed or lossless only

## ⚠️ Best Practices

1. **Always embed ICC profiles** for color accuracy
2. **Use lossless formats** (PNG, TIFF) for archival
3. **Include complete metadata** for provenance
4. **Version control** artwork iterations
5. **Backup original files** before conversion
6. **Test color accuracy** across devices

## 🤝 Contributing

Contributions welcome! Please see our [Contributing Guide](https://github.com/WIA-Official/wia-standards/CONTRIBUTING.md).

## 📄 License

MIT License - see [LICENSE](https://github.com/WIA-Official/wia-standards/LICENSE)

## 🔗 Links

- **Website**: [wiastandards.com](https://wiastandards.com)
- **GitHub**: [github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- **Documentation**: [docs.wiastandards.com](https://docs.wiastandards.com)
- **Certification**: [cert.wiastandards.com](https://cert.wiastandards.com)
- **E-Book**: [View English](./ebook/en/) | [한국어 보기](./ebook/ko/)
- **Simulator**: [Try Interactive Demo](./simulator/)

---

**홍익인간 (弘益人間) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
