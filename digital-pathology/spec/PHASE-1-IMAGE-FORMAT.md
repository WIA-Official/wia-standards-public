# WIA-MED-008 Phase 1: Image Format Specification

## Version 1.0.0

**Status:** ✅ Complete  
**Last Updated:** 2025-01-15

---

## 1. Overview

This specification defines the standard image format for digital pathology whole slide images (WSI) in the WIA-MED-008 standard.

### 1.1 Goals

- **Interoperability:** Enable seamless data exchange between systems
- **Efficiency:** Optimize storage and transmission
- **Quality:** Maintain diagnostic-grade image quality
- **Extensibility:** Support future enhancements

---

## 2. File Format

### 2.1 Container Format

**Primary Format:** WIA-DPS (WIA Digital Pathology Standard)
- Base: BigTIFF (TIFF 6.0 with 64-bit offsets)
- Extension: `.wdps` or `.tif`

**Supported Legacy Formats (Read-only):**
- DICOM Supplement 145
- OME-TIFF
- SVS (Aperio)
- NDPI (Hamamatsu)
- MRXS (3DHistech)

### 2.2 File Structure

```
WIA-DPS File Structure:
├── Header
│   ├── Magic Number: "WDPS" (4 bytes)
│   ├── Version: 1.0 (2 bytes)
│   └── Flags (2 bytes)
├── Metadata Section (JSON, gzip compressed)
├── Image Pyramid
│   ├── Level 0 (40x, 0.25 µm/pixel)
│   ├── Level 1 (20x, 0.5 µm/pixel)
│   ├── Level 2 (10x, 1.0 µm/pixel)
│   ├── Level 3 (5x, 2.0 µm/pixel)
│   ├── Level 4 (2.5x, 4.0 µm/pixel)
│   └── Level 5 (1.25x, 8.0 µm/pixel)
├── Annotations (Optional, GeoJSON)
└── AI Results (Optional)
```

---

## 3. Image Specifications

### 3.1 Resolution Requirements

| Magnification | µm/pixel | Use Case | Status |
|--------------|----------|----------|---------|
| 40x | 0.25 | Primary diagnosis | **Required** |
| 20x | 0.5 | General review | **Required** |
| 10x | 1.0 | Overview | Recommended |
| 5x | 2.0 | Navigation | Recommended |
| 2.5x | 4.0 | Thumbnail | Optional |
| 1.25x | 8.0 | Slide overview | Optional |

### 3.2 Color Space

**Standard:** sRGB IEC61966-2.1
- Color depth: 24-bit RGB (8 bits per channel)
- ICC profile: Embedded
- Calibration: Regular color calibration required

### 3.3 Tile Structure

- **Tile Size:** 256×256 pixels (standard)
- **Alternative:** 512×512 pixels (allowed)
- **Format:** JPEG, JPEG2000, or WebP
- **Quality:** 90 (JPEG scale 0-100)

---

## 4. Compression

### 4.1 Recommended Compression

| Stain Type | Compression | Quality | Rationale |
|-----------|-------------|---------|-----------|
| H&E | JPEG | 90 | High compression, diagnostic quality |
| IHC | JPEG2000 (lossless) | - | Preserve color accuracy |
| Fluorescence | Deflate/LZW | - | Quantitative analysis |
| Multispectral | JPEG2000 (lossless) | - | Spectral unmixing |

### 4.2 Compression Ratios

- **H&E:** 15:1 typical (5GB → 330MB)
- **IHC:** 3:1 typical (5GB → 1.7GB)
- **Fluorescence:** 2:1 typical

---

## 5. Metadata Schema

### 5.1 Required Metadata (JSON)

```json
{
  "wia_version": "1.0.0",
  "specimen": {
    "accession_number": "required",
    "specimen_id": "required",
    "organ": "required",
    "specimen_type": "required"
  },
  "scan": {
    "scanner_model": "required",
    "scan_date": "ISO 8601 format",
    "magnification": 40,
    "resolution_um_per_pixel": 0.25
  },
  "image": {
    "width_pixels": "integer",
    "height_pixels": "integer",
    "pyramid_levels": "integer",
    "tile_size": 256,
    "compression": "JPEG|JPEG2000|WebP",
    "color_space": "sRGB"
  },
  "quality": {
    "focus_quality_score": "0.0-1.0",
    "passed_qc": "boolean"
  }
}
```

### 5.2 Optional Metadata

- Processing information (fixation, embedding, staining)
- Clinical information (diagnosis, patient demographics)
- AI analysis results
- Annotations

---

## 6. Quality Metrics

### 6.1 Image Quality Standards

| Metric | Requirement | Measurement |
|--------|-------------|-------------|
| Focus Quality | > 90% | Brenner gradient |
| Color Accuracy | ΔE < 5 | Delta E (H&E) |
| Resolution | MTF50 > 200 lp/mm | MTF measurement |
| SNR | > 40 dB | Signal-to-noise ratio |

### 6.2 Quality Control

- **Daily:** Standard slide scan and QC check
- **Weekly:** Calibration verification
- **Monthly:** Full system calibration

---

## 7. Implementation Guidelines

### 7.1 Reading WIA-DPS Files

```python
import wia_dps

# Open WIA-DPS file
slide = wia_dps.open_slide('specimen.wdps')

# Get metadata
metadata = slide.get_metadata()
print(f"Scanner: {metadata['scan']['scanner_model']}")
print(f"Magnification: {metadata['scan']['magnification']}x")

# Read tile
level = 0  # Highest resolution
x, y = 10000, 15000
tile = slide.read_region((x, y), level, (256, 256))

# Get thumbnail
thumbnail = slide.get_thumbnail((1024, 1024))
```

### 7.2 Writing WIA-DPS Files

```python
import wia_dps

# Create new slide
writer = wia_dps.WDPSWriter('output.wdps')

# Set metadata
writer.set_metadata({
    'specimen': {'accession_number': 'S25-00123'},
    'scan': {'magnification': 40, 'resolution_um_per_pixel': 0.25}
})

# Add image data
writer.write_level(0, image_array)  # Full resolution
writer.write_level(1, downsampled_array)  # 20x

# Finalize
writer.close()
```

---

## 8. Validation

### 8.1 Compliance Testing

Tools: `wia-dps-validate`

```bash
# Validate file
$ wia-dps-validate specimen.wdps

Results:
  ✅ File structure: PASS
  ✅ Metadata schema: PASS
  ✅ Image quality: PASS (FQ: 94.5%)
  ✅ Compression: PASS (JPEG q=90)
  
  Overall: COMPLIANT
```

### 8.2 Certification

- **WIA-MED-008 Certified:** Products passing validation
- **Certification Period:** 3 years
- **Renewal:** Required before expiration

---

## 9. References

- TIFF 6.0 Specification
- DICOM Supplement 145
- OME-TIFF Specification
- sRGB Color Space Standard
- JPEG/JPEG2000 Standards

---

## 10. Appendix

### 10.1 Color Calibration Targets

- Macbeth ColorChecker
- IT8 Chart
- Standard H&E reference slides

### 10.2 Test Datasets

Available at: https://wia.live/datasets/med-008

---

**© 2025 WIA - World Certification Industry Association**  
**License:** MIT License  
**Contact:** standards@wia.live
