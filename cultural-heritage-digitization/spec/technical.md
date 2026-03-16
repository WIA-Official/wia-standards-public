# WIA-EDU-023: Technical Specifications
## Cultural Heritage Digitization

**Version:** 1.0.0
**Last Updated:** 2025-01-26

---

## 1. Capture Technologies

### 1.1 Photogrammetry

**Workflow:**
1. Capture 50-300+ overlapping images
2. Camera calibration and lens correction
3. Structure from Motion (SfM) processing
4. Dense point cloud generation
5. Mesh reconstruction
6. Texture atlas creation

**Requirements:**
- Image overlap: 60-80% between adjacent photos
- Camera: 20MP+ with prime lens preferred
- Lighting: Even, diffuse, color-calibrated
- Scale references: Included in capture volume
- Color targets: X-Rite ColorChecker or equivalent

**Software:** Agisoft Metashape, RealityCapture, Meshroom (open source)

### 1.2 LiDAR Scanning

**Types:**

**Terrestrial Laser Scanning (TLS)**
- Range: 0.5m - 350m typical
- Accuracy: ±1mm to ±5mm
- Point acquisition: 50K-2M points/second
- Applications: Buildings, monuments, large artifacts

**Aerial LiDAR**
- Platform: UAV or aircraft
- Ground sampling: 5-20cm typical
- Applications: Archaeological sites, landscapes
- Requires: RTK/PPK GPS for georeferencing

**Structured Light**
- Range: 0.1m - 5m typical
- Accuracy: ±0.01mm to ±0.1mm
- Applications: Small-medium artifacts
- Advantages: High detail, fast capture

### 1.3 CT/X-Ray Imaging

**Applications:**
- Internal structure analysis
- Hidden content discovery
- Non-destructive examination
- Material composition study

**Parameters:**
- Resolution: Voxel size appropriate to object scale
- Energy levels: Adjusted for material density
- Reconstruction: DICOM format standard

---

## 2. Data Processing

### 2.1 Point Cloud Processing

**Cleaning:**
- Outlier removal (statistical or radius-based)
- Noise reduction
- Registration alignment (ICP algorithm)
- Ground plane normalization

**Decimation:**
- Voxel-grid filtering for uniform density
- Preserve edges and features
- Target density based on use case

**Formats:**
- **E57:** ASTM standard, widely supported
- **LAS/LAZ:** Compact, georeferencing support
- **PTS/XYZ:** Simple ASCII formats

### 2.2 Mesh Generation

**Algorithms:**
- Poisson reconstruction (watertight meshes)
- Ball-pivoting (preserves detail)
- Delaunay triangulation
- Screened Poisson (balance detail/smoothness)

**Optimization:**
- Decimation to target polygon count
- UV unwrapping for texturing
- Normal map baking from high-res
- LOD (Level of Detail) generation

**Quality Metrics:**
- Manifold edges (2 faces per edge)
- Inverted normals check
- Non-manifold vertices resolution
- Degenerate triangle removal

### 2.3 Texture Mapping

**Process:**
- UV atlas generation (minimal distortion)
- Photo projection and blending
- Color correction across seams
- Normal/roughness/metallic map baking

**Texture Specifications:**
- Format: PNG (lossless) or JPEG 2000
- Resolution: 4K-8K for medium objects
- Color space: sRGB for display, Adobe RGB for archival
- Mip-mapping: Generate for real-time rendering

---

## 3. File Format Specifications

### 3.1 glTF 2.0 (Recommended for Delivery)

**Advantages:**
- Royalty-free, open standard
- Efficient transmission (binary)
- PBR material support
- Animation support
- Wide software support

**Structure:**
```json
{
  "asset": {"version": "2.0"},
  "scene": 0,
  "scenes": [{"nodes": [0]}],
  "nodes": [{"mesh": 0}],
  "meshes": [{
    "primitives": [{
      "attributes": {"POSITION": 0, "NORMAL": 1, "TEXCOORD_0": 2},
      "indices": 3,
      "material": 0
    }]
  }],
  "materials": [{
    "pbrMetallicRoughness": {
      "baseColorTexture": {"index": 0},
      "metallicFactor": 0.0,
      "roughnessFactor": 1.0
    }
  }]
}
```

### 3.2 E57 (Point Cloud Archival)

- Compact binary format
- Supports RGB color, intensity
- Georeferencing metadata
- Multiple scans per file
- Open specification

### 3.3 X3D (Archival Standard)

- ISO/IEC 19775-1:2013
- XML-based scene graph
- Long-term preservation
- Multiple encodings (XML, binary, compressed)

---

## 4. Metadata Schemas

### 4.1 Technical Metadata

```xml
<technical>
  <acquisition>
    <method>photogrammetry</method>
    <equipment>
      <camera>
        <make>Canon</make>
        <model>EOS R5</model>
        <lens>RF 50mm f/1.2</lens>
        <resolution>45MP</resolution>
      </camera>
      <calibration>
        <date>2025-01-15</date>
        <method>checkerboard</method>
      </calibration>
    </equipment>
    <capture_date>2025-01-20</capture_date>
    <operator>Jane Smith</operator>
    <image_count>250</image_count>
    <lighting>Diffuse LED panels, 5500K</lighting>
  </acquisition>
  <processing>
    <software>
      <name>Agisoft Metashape</name>
      <version>2.0.1</version>
    </software>
    <steps>
      <align_photos quality="high" />
      <build_dense_cloud quality="medium" />
      <build_mesh source="dense_cloud" face_count="2000000" />
      <build_texture mapping="generic" blend="mosaic" size="8192" />
    </steps>
    <processing_date>2025-01-21</processing_date>
  </processing>
  <quality>
    <point_cloud_density>1000 points/mm²</point_cloud_density>
    <geometric_accuracy>±0.05mm</geometric_accuracy>
    <color_accuracy>ΔE 1.8</color_accuracy>
    <completeness>98.5%</completeness>
  </quality>
</technical>
```

### 4.2 Preservation Metadata (PREMIS)

Key elements:
- **Object:** Digital file characteristics
- **Event:** Actions performed on object
- **Agent:** Persons/systems performing events
- **Rights:** Copyright and access restrictions

### 4.3 Descriptive Metadata (Dublin Core + CIDOC-CRM)

See API reference for full schemas.

---

## 5. Storage Architecture

### 5.1 Repository Structure

```
repository/
├── masters/              # Preservation masters
│   ├── pointcloud/      # E57 files
│   ├── mesh/            # High-res OBJ/FBX
│   └── textures/        # Uncompressed TIFF
├── derivatives/         # Access copies
│   ├── web/            # glTF optimized
│   ├── ar/             # USDZ for iOS
│   └── thumbnails/     # Preview images
├── metadata/           # Sidecar files
│   ├── dublin_core/
│   ├── technical/
│   └── preservation/
└── documentation/      # Paradata
    ├── methodology/
    ├── reports/
    └── references/
```

### 5.2 Fixity Checking

**Checksums:**
- MD5 and SHA-256 for all files
- Stored in manifest files
- Automated verification schedule:
  - Hot storage: monthly
  - Warm storage: quarterly
  - Cold storage: annually

**Manifest Format (BagIt):**
```
bag-info.txt
bagit.txt
manifest-md5.txt
manifest-sha256.txt
data/
  ├── object001.glb
  ├── object001_metadata.xml
  └── object001_documentation.pdf
```

---

## 6. Quality Assurance

### 6.1 Validation Checklist

- ✅ Geometric accuracy verified with control measurements
- ✅ Mesh topology valid (manifold, no self-intersections)
- ✅ Textures properly aligned, no visible seams
- ✅ Color calibration verified against target
- ✅ File formats conform to specifications
- ✅ Metadata complete and validated against schema
- ✅ Checksums generated and verified
- ✅ Documentation complete

### 6.2 Peer Review

- Expert validation of reconstruction accuracy
- Community consultation where appropriate
- Accessibility testing with disabled users
- Technical review by preservation specialists

---

## 7. Web Delivery Optimization

### 7.1 Progressive Loading

1. Load low-poly preview mesh
2. Stream high-res geometry
3. Load compressed textures
4. Upgrade to full-resolution

### 7.2 Compression

- **Draco:** Mesh geometry compression (10:1 typical)
- **KTX2/Basis:** Texture compression (GPU-native)
- **glTF-Transform:** Optimization pipeline

### 7.3 Performance Targets

- Initial load: <3 seconds on 4G
- Interactive framerate: 60fps desktop, 30fps mobile
- Total download: <50MB for typical artifact

---

## 8. Security & Integrity

### 8.1 Access Control

- Role-based permissions (public, researcher, admin)
- Watermarking for high-res downloads
- Rate limiting to prevent abuse
- Audit logging of access

### 8.2 Data Integrity

- Cryptographic signatures for authenticity
- Blockchain anchoring for provenance (optional)
- Redundant storage (3-2-1 rule)
- Geographic distribution of copies

---

## Appendix: Software Recommendations

### Capture
- Photogrammetry: Metashape, RealityCapture, Meshroom
- LiDAR Processing: CloudCompare, PDAL, LAStools

### Processing
- Mesh editing: Blender, MeshLab, ZBrush
- UV unwrapping: RizomUV, Blender
- Texture baking: Substance Painter, Blender

### Optimization
- glTF: Blender, glTF-Transform
- Compression: Draco, Meshoptimizer, KTX-Software

### Web Delivery
- Viewers: Three.js, Babylon.js, model-viewer
- Platforms: Sketchfab, Smithsonian Voyager

---

© 2025 WIA - World Certification Industry Association
**License:** MIT
