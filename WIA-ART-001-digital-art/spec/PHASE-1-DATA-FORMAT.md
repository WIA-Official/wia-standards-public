# WIA-ART-001: Phase 1 - Data Format Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the standardized data formats for digital artwork representation in the WIA-ART-001 standard. All implementations MUST support these formats to ensure interoperability across creative tools, distribution platforms, and archival systems.

## 2. File Format Standards

### 2.1 Raster Graphics

#### 2.1.1 PNG (Portable Network Graphics)
```json
{
  "format": "PNG",
  "version": "1.2",
  "compression": "deflate",
  "interlacing": "optional",
  "bitDepth": [8, 16],
  "colorType": ["grayscale", "truecolor", "indexed", "grayscale-alpha", "truecolor-alpha"],
  "ancillaryChunks": {
    "iCCP": "REQUIRED - ICC color profile",
    "tEXt": "REQUIRED - metadata",
    "iTXt": "REQUIRED - UTF-8 metadata",
    "tIME": "RECOMMENDED - timestamp"
  }
}
```

**Requirements:**
- MUST include embedded ICC color profile (iCCP chunk)
- MUST include UTF-8 metadata (iTXt chunks)
- SHOULD use 16-bit depth for professional work
- MUST NOT use lossy compression

#### 2.1.2 JPEG (Joint Photographic Experts Group)
```json
{
  "format": "JPEG",
  "version": "JFIF 1.02 or EXIF 2.3+",
  "compression": "lossy",
  "quality": {
    "minimum": 85,
    "recommended": 95,
    "maximum": 100
  },
  "colorSpace": ["YCbCr", "RGB", "CMYK"],
  "sampling": "4:4:4 (no chroma subsampling)",
  "progressive": "recommended"
}
```

**Requirements:**
- Quality MUST be >= 85 for distribution
- MUST use 4:4:4 sampling (no chroma subsampling)
- MUST include EXIF 2.3+ metadata
- SHOULD use progressive encoding for web

#### 2.1.3 TIFF (Tagged Image File Format)
```json
{
  "format": "TIFF",
  "version": "6.0+",
  "compression": ["none", "LZW", "ZIP", "PackBits"],
  "bitDepth": [8, 16, 32],
  "colorSpace": ["RGB", "CMYK", "LAB", "Grayscale"],
  "alpha": "supported",
  "layers": "supported (layered TIFF)"
}
```

**Requirements:**
- MUST use lossless compression only
- MUST include ICC profile tag (34675)
- SHOULD use 16-bit or 32-bit for archival
- MUST preserve all metadata tags

### 2.2 Vector Graphics

#### 2.2.1 SVG (Scalable Vector Graphics)
```xml
<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg"
     xmlns:wia="https://wia.org/standards/ART-001/v1"
     version="1.1"
     viewBox="0 0 1000 1000">
  <metadata>
    <rdf:RDF xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
             xmlns:dc="http://purl.org/dc/elements/1.1/">
      <rdf:Description rdf:about="">
        <dc:title>Artwork Title</dc:title>
        <dc:creator>Artist Name</dc:creator>
        <dc:date>2025-01-15</dc:date>
        <dc:rights>© 2025 Artist Name</dc:rights>
        <dc:license>CC BY-SA 4.0</dc:license>
      </rdf:Description>
    </rdf:RDF>
  </metadata>
  <!-- Artwork content -->
</svg>
```

**Requirements:**
- MUST include RDF metadata block
- MUST use UTF-8 encoding
- SHOULD optimize path data
- MAY include embedded raster images (base64 encoded)

### 2.3 3D Assets

#### 2.3.1 glTF 2.0 (GL Transmission Format)
```json
{
  "asset": {
    "version": "2.0",
    "generator": "WIA-ART-001 Exporter v1.0"
  },
  "extensionsUsed": [
    "KHR_materials_pbrSpecularGlossiness",
    "KHR_materials_unlit",
    "WIA_art_metadata"
  ],
  "extensions": {
    "WIA_art_metadata": {
      "title": "3D Sculpture",
      "artist": "Artist Name",
      "created": "2025-01-15T10:30:00Z",
      "license": "CC BY-SA 4.0"
    }
  }
}
```

**Requirements:**
- MUST use glTF 2.0 specification
- MUST include WIA_art_metadata extension
- SHOULD use PBR materials
- MUST embed textures in GLB format for distribution

## 3. Metadata Schema

### 3.1 Core Metadata Structure

```json
{
  "@context": "https://wia.org/standards/ART-001/v1",
  "@type": "DigitalArtwork",
  "id": "string (UUID v4 or URI)",
  "version": "1.0",
  "timestamp": "ISO 8601 datetime",

  "artwork": {
    "title": "string (required, 1-200 chars)",
    "subtitle": "string (optional)",
    "description": "string (optional, markdown supported)",
    "created": "ISO 8601 datetime",
    "modified": "ISO 8601 datetime",
    "editionNumber": "number (optional, for limited editions)",
    "editionTotal": "number (optional)"
  },

  "artist": {
    "name": "string (required)",
    "id": "DID or URI (recommended)",
    "url": "string (website)",
    "bio": "string (optional)",
    "nationality": "ISO 3166-1 alpha-2",
    "birthDate": "ISO 8601 date (optional)",
    "socialMedia": {
      "twitter": "string",
      "instagram": "string",
      "artstation": "string"
    }
  },

  "technical": {
    "medium": "enum (required)",
    "format": {
      "mimeType": "string (required)",
      "extension": "string",
      "version": "string"
    },
    "dimensions": {
      "width": "number (pixels or mm)",
      "height": "number",
      "depth": "number (for 3D)",
      "unit": "enum: px|mm|cm|in",
      "dpi": "number (72-600)",
      "aspectRatio": "string (e.g., '16:9')"
    },
    "color": {
      "space": "enum: sRGB|AdobeRGB|DisplayP3|ProPhotoRGB|CMYK",
      "profile": "string (ICC profile name)",
      "bitDepth": "number: 8|16|32",
      "channels": "number: 1|3|4",
      "hasAlpha": "boolean"
    },
    "fileSize": "number (bytes)",
    "checksum": {
      "algorithm": "SHA-256",
      "value": "string (hex)"
    }
  },

  "classification": {
    "category": "enum: painting|sculpture|photography|mixed-media|generative|other",
    "style": "array of strings",
    "movement": "array of strings",
    "technique": "array of strings",
    "subject": "array of strings",
    "tags": "array of strings"
  },

  "rights": {
    "copyright": "string (required)",
    "copyrightYear": "number (required)",
    "license": {
      "type": "string (SPDX identifier)",
      "url": "string",
      "terms": "string"
    },
    "attribution": {
      "required": "boolean",
      "text": "string",
      "url": "string"
    },
    "usage": {
      "commercial": "boolean",
      "derivatives": "boolean",
      "distribution": "boolean"
    }
  },

  "provenance": {
    "created": {
      "software": "string (e.g., 'Adobe Photoshop 2025')",
      "hardware": "string (optional)",
      "location": "string (optional)"
    },
    "blockchain": {
      "network": "enum: ethereum|polygon|tezos|flow|solana|other",
      "contract": "string (smart contract address)",
      "tokenId": "string",
      "tokenStandard": "enum: ERC-721|ERC-1155|FA2|other",
      "mintDate": "ISO 8601 datetime",
      "hash": "string (IPFS CID or similar)"
    },
    "chain": [
      {
        "event": "enum: created|modified|transferred|exhibited|published",
        "timestamp": "ISO 8601 datetime",
        "actor": "string",
        "location": "string (optional)",
        "txHash": "string (optional)"
      }
    ]
  },

  "exhibition": {
    "history": [
      {
        "venue": "string",
        "location": "string",
        "startDate": "ISO 8601 date",
        "endDate": "ISO 8601 date",
        "type": "enum: physical|virtual|hybrid"
      }
    ]
  },

  "preservation": {
    "archival": {
      "originalFormat": "string",
      "masterFile": "string (URI)",
      "backups": ["array of URIs"],
      "integrityCheck": "ISO 8601 datetime (last verified)"
    },
    "requirements": {
      "temperature": "string (e.g., '15-25°C')",
      "humidity": "string",
      "lightExposure": "string"
    }
  }
}
```

### 3.2 Medium Enumeration

Valid values for `technical.medium`:

- `digital-painting` - Created with digital painting tools
- `3d-modeling` - Three-dimensional computer graphics
- `vector-art` - Vector-based illustration
- `pixel-art` - Pixel-based art
- `generative` - Algorithmically generated
- `photo-manipulation` - Edited photography
- `mixed-media` - Combination of techniques
- `ai-assisted` - Created with AI tools
- `photography` - Digital photography
- `fractal` - Fractal-based art
- `ascii-art` - Text-based art
- `voxel-art` - Volumetric pixel art

### 3.3 Style Classification

**Art Movements:**
- Abstract, Impressionism, Surrealism, Pop Art, Minimalism
- Cyberpunk, Vaporwave, Glitch Art, Low Poly
- Photorealism, Hyperrealism, Fantasy, Sci-Fi

**Techniques:**
- Digital Painting, Photo Bashing, Matte Painting
- Procedural Generation, Particle Systems
- Ray Tracing, Photogrammetry, Sculpting

## 4. Compression Standards

### 4.1 Lossless Compression

**For Archival (REQUIRED):**
- PNG with deflate compression
- TIFF with LZW or ZIP compression
- WebP lossless mode
- FLIF (Free Lossless Image Format)

### 4.2 Lossy Compression

**For Distribution (OPTIONAL):**
- JPEG at quality >= 85
- WebP lossy mode
- AVIF with appropriate quality settings

**MUST NOT use lossy compression for:**
- Master/archival files
- Files requiring further editing
- Line art or text-heavy images

## 5. Color Space Requirements

### 5.1 Standard Color Spaces

| Space | Usage | Gamut | Profile |
|-------|-------|-------|---------|
| sRGB | Web, general use | Standard | IEC 61966-2-1 |
| Adobe RGB | Professional print | Wide | Adobe RGB (1998) |
| Display P3 | Modern displays | Wide | Display P3 |
| ProPhoto RGB | Archival, editing | Ultra-wide | ROMM RGB |
| CMYK | Print production | Varies | ISO Coated v2 |

### 5.2 Color Management Rules

1. **MUST** embed ICC color profile in all files
2. **MUST** specify color space in metadata
3. **SHOULD** use perceptual rendering intent for conversions
4. **MUST** preserve original color space in archival files
5. **SHOULD** provide sRGB version for web distribution

## 6. Quality Tiers

### 6.1 Archival Master

- Format: TIFF or PNG
- Bit Depth: 16-bit minimum
- Color Space: ProPhoto RGB or Adobe RGB
- Compression: None or lossless only
- Resolution: Maximum available
- Metadata: Complete

### 6.2 Professional Distribution

- Format: PNG or high-quality JPEG
- Bit Depth: 8-bit or 16-bit
- Color Space: Adobe RGB or sRGB
- DPI: 300 minimum
- Metadata: Core fields required

### 6.3 Web Display

- Format: WebP, AVIF, or JPEG
- Bit Depth: 8-bit
- Color Space: sRGB
- DPI: 72-96
- File Size: Optimized
- Metadata: Basic fields

## 7. Validation Requirements

All files MUST pass these validation checks:

1. ✓ Valid file format structure
2. ✓ Embedded ICC color profile
3. ✓ Core metadata fields present
4. ✓ Checksum matches content
5. ✓ File size within acceptable limits
6. ✓ No corrupted data blocks
7. ✓ Proper UTF-8 encoding for text
8. ✓ Valid URI/URL references

## 8. File Naming Convention

```
{artist-slug}_{artwork-slug}_{version}_{variant}.{ext}

Examples:
jane-doe_sunset-mountains_v1_master.tiff
jane-doe_sunset-mountains_v1_web.jpg
jane-doe_sunset-mountains_v1_thumb.webp
```

**Rules:**
- Use lowercase
- Replace spaces with hyphens
- Use underscores as separators
- Include version number
- Specify variant (master, web, thumb, print)

## 9. Reference Standards Alignment

### 9.1 File Format References

| Format | Reference |
|--------|-----------|
| PNG | ISO 15948 |
| JPEG | ISO/IEC 10918-1 |
| JPEG 2000 | ISO/IEC 15444-1 |
| HEIF / HEIC | ISO/IEC 23008-12 |
| AVIF | AOMedia AV1 Bitstream & Decoding Process Specification, ISO/IEC 14496-12 container |
| TIFF | ISO 12234-2:2001 (TIFF/EP) |
| SVG | W3C SVG 2 |
| PDF/X | ISO 15930 series |
| glTF 2.0 | Khronos glTF 2.0 Specification |
| COLLADA | ISO/PAS 17506:2012 |
| OpenEXR | High-Dynamic-Range image format under the Academy Software Foundation |

### 9.2 Colour Management References

| Concern | Reference |
|---------|-----------|
| ICC profiles | ISO 15076-1:2010 |
| sRGB | IEC 61966-2-1:1999 |
| Display P3 | DCI-P3 white point with sRGB transfer; Apple ColorSync reference |
| Rec. ITU-R BT.2020 | ITU-R Recommendation BT.2020-2 |
| ProPhoto RGB | ANSI/I3A IT10.7666-2002 (ROMM RGB) |
| CMYK reference | ISO 12647-2:2013 (offset litho), ISO 15930 (PDF/X) |
| Colour appearance | CIECAM02 / CIECAM16 |

### 9.3 Hash and Signature

| Concern | Reference |
|---------|-----------|
| Content hash | FIPS 180-4 (SHA-2 family), FIPS 202 (SHA-3 family) |
| Signing | RFC 8032 (EdDSA), NIST FIPS 186-5 (ECDSA), RFC 8017 (RSA-PSS) |
| Signed manifest container | RFC 9052 (COSE), RFC 8949 (CBOR), RFC 7515 (JWS) |

### 9.4 Provenance and Attribution

| Concern | Reference |
|---------|-----------|
| Provenance | W3C PROV-O |
| Verifiable Credentials | W3C VC Data Model 2.0 |
| Decentralised Identity | W3C DID Core 1.0 |
| Catalog vocabulary | W3C DCAT v3 |
| Schema markup | schema.org `VisualArtwork` / `CreativeWork` |
| Cross-standard provenance | C2PA technical specification 2.x |

### 9.5 Locale and Time

| Concern | Reference |
|---------|-----------|
| Language tags | BCP 47 (RFC 5646) |
| Locale data | Unicode CLDR |
| Time encoding | ISO 8601:2019 |
| Bidirectional text | Unicode Standard Annex #9 |

### 9.6 Conformance

A Phase 1 implementation is conformant when:

1. All files emitted use one of the §9.1 file formats with a declared format identifier in the manifest.
2. ICC colour profiles are embedded in all raster outputs (§5).
3. Content hashes are computed using a §9.3 hash algorithm and declared in the manifest.
4. Signed manifests carry an explicit algorithm identifier from §9.3 and a public-key reference resolvable via the §9.4 identity references.
5. Time fields conform to ISO 8601:2019 and locale fields conform to BCP 47.

All references conform to the WIA Citation & Veracity Policy v1.0 §2.1 ALLOW.

---

**弘益人間 (Benefit All Humanity)**

*© 2025 WIA - World Certification Industry Association*
*MIT License*
