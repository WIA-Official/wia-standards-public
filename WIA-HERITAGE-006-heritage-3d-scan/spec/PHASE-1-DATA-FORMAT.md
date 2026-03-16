# WIA-HERITAGE-006: Phase 1 - Data Format Specification

> 弘益人間 (Benefit All Humanity)

## Overview

Phase 1 defines the standardized data formats for cultural artifact digitization, including 3D models, images, metadata, and associated documentation.

## 1. 3D Model Formats

### 1.1 Supported Formats

| Format | Extension | Use Case | Priority |
|--------|-----------|----------|----------|
| glTF 2.0 | `.gltf`, `.glb` | Web/AR/VR delivery | Primary |
| OBJ | `.obj`, `.mtl` | Archival, compatibility | Secondary |
| PLY | `.ply` | Point clouds, raw scans | Secondary |
| FBX | `.fbx` | Game engines, animation | Tertiary |
| USDZ | `.usdz` | Apple AR Quick Look | Tertiary |

### 1.2 glTF 2.0 Specification

```json
{
  "asset": {
    "version": "2.0",
    "generator": "WIA-HERITAGE-006",
    "copyright": "© 2025 [Institution Name]"
  },
  "scene": 0,
  "scenes": [{ "nodes": [0] }],
  "nodes": [{
    "mesh": 0,
    "name": "artifact_name"
  }],
  "meshes": [{
    "primitives": [{
      "attributes": {
        "POSITION": 0,
        "NORMAL": 1,
        "TEXCOORD_0": 2
      },
      "indices": 3,
      "material": 0
    }]
  }],
  "materials": [{
    "name": "artifact_material",
    "pbrMetallicRoughness": {
      "baseColorTexture": { "index": 0 },
      "metallicFactor": 0.0,
      "roughnessFactor": 0.8
    },
    "normalTexture": { "index": 1 }
  }]
}
```

### 1.3 Model Quality Standards

#### Resolution Levels

- **Archive Quality**: ≥10M polygons, ≤0.1mm accuracy
- **Display Quality**: 1M-5M polygons, ≤0.5mm accuracy
- **Web Quality**: 100K-500K polygons, ≤2mm accuracy
- **Mobile Quality**: 10K-50K polygons, ≤5mm accuracy

#### Texture Requirements

- **Archive**: 8K (8192×8192) TIFF, uncompressed
- **Display**: 4K (4096×4096) PNG/JPEG2000
- **Web**: 2K (2048×2048) PNG, optimized
- **Mobile**: 1K (1024×1024) JPEG, compressed

## 2. Image Formats

### 2.1 Photography Standards

| Purpose | Format | Resolution | Color Space |
|---------|--------|------------|-------------|
| Archive Master | TIFF/DNG | ≥50MP | Adobe RGB/ProPhoto |
| Working Copy | JPEG2000 | 20-50MP | sRGB |
| Web Display | JPEG | 2-10MP | sRGB |
| Thumbnail | WebP | 300-500px | sRGB |

### 2.2 Multi-Spectral Imaging

```yaml
multispectral_capture:
  wavelengths:
    - UV: 365nm
    - Visible: 400-700nm (RGB)
    - IR: 850nm, 940nm
  format: TIFF
  bit_depth: 16-bit per channel
  calibration: ColorChecker reference
  metadata: EXIF + XMP sidecar
```

### 2.3 Image Metadata (EXIF/XMP)

```xml
<rdf:RDF xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
         xmlns:wia="http://wia.org/heritage/006#">
  <rdf:Description rdf:about="artifact_006.tif">
    <wia:artifactID>WIA-ART-2025-006</wia:artifactID>
    <wia:captureDate>2025-01-15T14:30:00Z</wia:captureDate>
    <wia:captureMethod>photogrammetry</wia:captureMethod>
    <wia:resolution>0.1mm</wia:resolution>
    <wia:colorSpace>AdobeRGB</wia:colorSpace>
    <wia:institution>National Museum</wia:institution>
  </rdf:Description>
</rdf:RDF>
```

## 3. Metadata Schema

### 3.1 Dublin Core Mapping

```json
{
  "dc:title": "Ancient Greek Amphora",
  "dc:creator": "Unknown (5th century BCE)",
  "dc:subject": ["Pottery", "Greek", "Classical Period"],
  "dc:description": "Red-figure amphora depicting Dionysus",
  "dc:date": "-450/-400",
  "dc:type": "PhysicalObject",
  "dc:format": "Ceramic, terracotta",
  "dc:identifier": "WIA-ART-2025-006",
  "dc:source": "Archaeological Museum of Athens",
  "dc:language": "grc",
  "dc:coverage": "Athens, Greece",
  "dc:rights": "Public Domain"
}
```

### 3.2 CIDOC-CRM Extensions

```turtle
@prefix crm: <http://www.cidoc-crm.org/cidoc-crm/> .
@prefix wia: <http://wia.org/heritage/006#> .

<artifact:006> a crm:E22_Human-Made_Object ;
    crm:P1_is_identified_by "WIA-ART-2025-006" ;
    crm:P45_consists_of <material:terracotta> ;
    crm:P108i_was_produced_by <production:006> ;
    crm:P52_has_current_owner <institution:museum> ;
    crm:P55_has_current_location <location:athens> ;
    wia:hasDigitalRepresentation <model:006> .

<production:006> a crm:E12_Production ;
    crm:P4_has_time-span <timespan:450-400-bce> ;
    crm:P7_took_place_at <place:athens> .
```

### 3.3 WIA-HERITAGE-006 Extended Schema

```json
{
  "wia": {
    "version": "1.0",
    "standard": "WIA-HERITAGE-006",
    "artifact": {
      "id": "WIA-ART-2025-006",
      "name": "Ancient Greek Amphora",
      "category": "pottery",
      "subcategory": "amphora",
      "period": {
        "name": "Classical Period",
        "startYear": -450,
        "endYear": -400,
        "era": "BCE"
      },
      "origin": {
        "country": "Greece",
        "region": "Attica",
        "site": "Athens",
        "coordinates": {
          "lat": 37.9838,
          "lon": 23.7275
        }
      },
      "physical": {
        "dimensions": {
          "height": 425,
          "width": 280,
          "depth": 280,
          "weight": 3200,
          "unit": "mm/g"
        },
        "materials": [
          {
            "type": "terracotta",
            "composition": "Clay, iron oxide",
            "percentage": 100,
            "analysis": "XRF spectroscopy"
          }
        ],
        "condition": {
          "overall": "good",
          "completeness": 95,
          "damages": [
            {
              "type": "crack",
              "location": "neck",
              "severity": 0.3,
              "stabilized": true
            }
          ]
        }
      },
      "digitization": {
        "method": "photogrammetry",
        "date": "2025-01-15",
        "equipment": "Nikon D850 + 105mm macro",
        "software": "RealityCapture 1.3",
        "photoCount": 247,
        "processingTime": 180,
        "accuracy": 0.08,
        "models": [
          {
            "id": "model-006-archive",
            "format": "gltf",
            "quality": "archive",
            "vertices": 12500000,
            "faces": 25000000,
            "textures": 8192,
            "fileSize": 2147483648
          }
        ]
      },
      "provenance": [
        {
          "date": "1923-03-15",
          "event": "excavation",
          "location": "Athens Agora",
          "responsible": "American School of Classical Studies"
        },
        {
          "date": "1924-01-01",
          "event": "acquisition",
          "owner": "National Archaeological Museum",
          "method": "transfer"
        }
      ],
      "rights": {
        "copyright": "Public Domain",
        "license": "CC0 1.0 Universal",
        "usage": "unrestricted",
        "attribution": "National Archaeological Museum of Athens"
      }
    }
  }
}
```

## 4. Point Cloud Format

### 4.1 PLY Format

```
ply
format binary_little_endian 1.0
comment WIA-HERITAGE-006 scan
element vertex 15000000
property float x
property float y
property float z
property float nx
property float ny
property float nz
property uchar red
property uchar green
property uchar blue
property float confidence
end_header
[binary data]
```

### 4.2 LAS/LAZ Format

```yaml
las_specification:
  version: 1.4
  point_format: 6  # RGB + classification
  coordinate_system: WGS84 / Local
  units: millimeters
  attributes:
    - classification: artifact/background
    - intensity: 0-65535
    - rgb: 16-bit per channel
    - gps_time: capture timestamp
```

## 5. File Naming Convention

### 5.1 Structure

```
{institution}_{collection}_{artifact_id}_{type}_{version}_{quality}.{ext}

Examples:
NAM_GR_WIA-ART-2025-006_model_v1_archive.glb
NAM_GR_WIA-ART-2025-006_texture_v1_4k.png
NAM_GR_WIA-ART-2025-006_metadata_v1.json
NAM_GR_WIA-ART-2025-006_photo_006_raw.tif
```

### 5.2 Directory Structure

```
WIA-ART-2025-006/
├── raw/
│   ├── photos/
│   │   ├── IMG_006.NEF
│   │   ├── IMG_002.NEF
│   │   └── ...
│   └── scans/
│       └── pointcloud_raw.ply
├── processed/
│   ├── models/
│   │   ├── archive.glb
│   │   ├── display.glb
│   │   └── web.glb
│   ├── textures/
│   │   ├── diffuse_8k.tif
│   │   ├── normal_8k.tif
│   │   └── ao_8k.tif
│   └── images/
│       ├── photo_006_master.tif
│       └── photo_006_web.jpg
├── metadata/
│   ├── artifact.json
│   ├── dublin_core.xml
│   ├── cidoc_crm.ttl
│   └── provenance.json
└── documentation/
    ├── capture_report.pdf
    ├── condition_report.pdf
    └── conservation_history.pdf
```

## 6. Compression Standards

### 6.1 3D Model Compression

- **Draco**: For glTF compression (up to 90% reduction)
- **Meshoptimizer**: For runtime optimization
- **Basis Universal**: For texture compression

### 6.2 Image Compression

- **Lossless**: PNG, TIFF (LZW), JPEG 2000 (lossless mode)
- **Lossy**: JPEG (quality 90+), WebP (quality 85+)
- **Archive**: TIFF uncompressed or LZW

## 7. Validation Requirements

### 7.1 Model Validation

- Manifold geometry (no holes, no inverted normals)
- UV coordinates within [0,1] range
- Proper material definitions
- Texture references valid
- Scale in real-world units (mm)

### 7.2 Metadata Validation

- Required Dublin Core fields present
- Valid date formats (ISO 8601)
- Coordinate validation (WGS84)
- Identifier uniqueness
- Rights information complete

## 8. Interoperability

### 8.1 Compatible Standards

- **IIIF**: International Image Interoperability Framework
- **LIDO**: Lightweight Information Describing Objects
- **EAD**: Encoded Archival Description
- **MODS**: Metadata Object Description Schema
- **VRA Core**: Visual Resources Association Core

### 8.2 Export Formats

All data must be exportable in:
- JSON-LD (for semantic web)
- XML (for traditional systems)
- CSV (for spreadsheet analysis)
- RDF (for knowledge graphs)

---

**Phase 1 Compliance**: All implementations must support glTF 2.0, Dublin Core metadata, and TIFF/JPEG2000 imaging as minimum requirements.

**Next**: [Phase 2 - API Interface](PHASE-2-API-INTERFACE.md)

---

弘益人間 (Benefit All Humanity)
© 2025 SmileStory Inc. / WIA
