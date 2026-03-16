# WIA-EDU-023: Implementation Guide
## Cultural Heritage Digitization

**Version:** 1.0.0
**Last Updated:** 2025-01-26

---

## 1. Getting Started

### 1.1 Prerequisites

**Hardware:**
- Camera (20MP+) or smartphone with good camera
- Computer with GPU for processing (recommended)
- Storage: 1TB+ for projects
- Optional: 3D scanner, LiDAR device

**Software:**
- 3D capture: Agisoft Metashape, RealityCapture, or Meshroom (free)
- 3D editing: Blender (free), MeshLab (free)
- Metadata: Spreadsheet or dedicated system

---

## 2. Quick Start Workflow

### Step 1: Planning
1. Assess artifact characteristics
2. Choose capture method (photogrammetry, LiDAR, etc.)
3. Prepare environment (lighting, background)
4. Set up color/scale references

### Step 2: Capture
1. Take overlapping photos from all angles
2. Include scale bars and color targets
3. Verify coverage completeness
4. Back up raw images immediately

### Step 3: Processing
1. Import images to software
2. Align photos (sparse point cloud)
3. Build dense point cloud
4. Generate mesh
5. Create texture atlas
6. Export to standard formats

### Step 4: Quality Check
1. Verify accuracy with measurements
2. Check for holes or artifacts
3. Validate color accuracy
4. Test model in viewer

### Step 5: Metadata
1. Complete Dublin Core fields
2. Add technical capture details
3. Document processing steps
4. Assign persistent identifier

### Step 6: Preservation
1. Store master files in archival format
2. Generate checksums
3. Create multiple backups
4. Document storage locations

### Step 7: Dissemination
1. Create optimized web versions
2. Generate thumbnails
3. Upload to repository
4. Share with appropriate licenses

---

## 3. Best Practices

### Photogrammetry Tips
- Use manual camera settings (fixed ISO, aperture, white balance)
- Shoot RAW for maximum quality
- Overlap images 60-80%
- Avoid shiny, transparent, or very dark objects (or use scanning spray)
- Circle object in horizontal bands
- Take additional detail shots

### Metadata Essentials
- Document everything immediately
- Use controlled vocabularies
- Link to authority files (Getty, Wikidata)
- Include uncertainty estimates for reconstructions
- Store metadata with files (sidecar approach)

### Accessibility
- Provide multiple formats (3D, 2D images, text)
- Include detailed descriptions
- Test with screen readers
- Offer multiple languages
- Ensure keyboard navigation

---

## 4. Common Issues

### Problem: Incomplete mesh coverage
**Solution:** Retake photos from missing angles

### Problem: Texture seams visible
**Solution:** Improve UV unwrapping, use blending in texture baking

### Problem: File too large for web
**Solution:** Decimate mesh, compress textures, use Draco compression

### Problem: Colors don't match original
**Solution:** Use color calibration targets, shoot in RAW, correct in processing

---

## 5. Tools & Resources

### Free/Open Source
- **Meshroom:** Photogrammetry (Windows/Linux)
- **Blender:** 3D modeling and optimization
- **CloudCompare:** Point cloud processing
- **QGIS:** GIS and georeferencing

### Institutional
- **Agisoft Metashape:** Professional photogrammetry
- **RealityCapture:** Fast photogrammetry
- **Faro Scene:** LiDAR processing

### Validation
- **ColorChecker Passport:** Color calibration
- **Scale bars:** Dimensionally accurate references
- **Control measurements:** Calipers, laser distance meters

---

## 6. Training Resources

- WIA Online Courses: https://training.wiastandards.com
- Cultural Heritage Imaging: http://culturalheritageimaging.org
- Smithsonian Digitization Program: https://3d.si.edu
- Community Forums: https://forum.wiastandards.com

---

## 7. Certification Path

1. Complete training modules
2. Submit portfolio (3 projects minimum)
3. Pass technical assessment
4. Peer review
5. Receive WIA certification

---

## 8. Support

**Technical Support:** support@wiastandards.com
**Community Forum:** https://forum.wiastandards.com
**GitHub Issues:** https://github.com/WIA-Official/wia-standards/issues

---

© 2025 WIA - MIT License
