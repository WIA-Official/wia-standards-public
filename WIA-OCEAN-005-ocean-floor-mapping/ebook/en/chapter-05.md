# Chapter 5: 3D Modeling and Visualization

## 5.1 The Power of Visual Representation

Human brains excel at processing visual information. A well-crafted 3D visualization can reveal patterns, relationships, and features that might remain hidden in tables of numbers or even 2D maps. For seafloor mapping, 3D visualization transforms bathymetric data from abstract depth values into intuitive representations of underwater landscapes, enabling researchers, mariners, and the public to understand the ocean floor's complexity and beauty.

Modern visualization techniques leverage powerful graphics hardware and sophisticated software to render millions of data points in real-time. Users can fly through virtual underwater landscapes, examine features from any angle, and apply different color schemes to highlight various aspects of the topography. These capabilities have revolutionized how we explore, analyze, and communicate information about the seafloor.

The WIA-OCEAN-005 standard addresses visualization requirements to ensure that bathymetric data can be effectively rendered across different software platforms. Standardized formats, coordinate systems, and metadata enable interoperability while allowing innovation in visualization techniques and user interfaces.

## 5.2 Digital Elevation Models

The foundation of 3D seafloor visualization is the Digital Elevation Model (DEM), also called a Digital Terrain Model (DTM) or bathymetric grid. A DEM represents the seafloor as a regular grid of elevation values, providing a continuous surface model from discrete measurements.

**Grid Structure**: Most DEMs use a regular rectangular grid aligned with geographic or projected coordinates. Each cell contains an elevation value (or depth value for bathymetry) representing the seafloor elevation at that location. Grid spacing determines the resolution and file size—finer grids preserve more detail but require more storage and processing power.

**Data Formats**: Numerous file formats store gridded bathymetric data:
- **GeoTIFF**: Widely supported raster format with embedded georeferencing
- **NetCDF**: Self-describing format popular in oceanographic applications
- **BAG (Bathymetric Attributed Grid)**: Specifically designed for bathymetric data, includes uncertainty layers
- **ASCII Grid**: Simple text format, human-readable but inefficient for large datasets
- **Cloud-Optimized GeoTIFF (COG)**: Enables efficient access to portions of large datasets over networks

**Multi-Resolution Representations**: Large-area DEMs often use hierarchical structures storing multiple resolutions. This enables efficient visualization at different zoom levels—coarse resolution for regional views, fine resolution when zoomed to detailed areas. Techniques like quadtrees and tile pyramids organize the multi-resolution data efficiently.

## 5.3 Surface Rendering Techniques

Converting a grid of elevation values into a compelling 3D visualization requires sophisticated rendering techniques:

**Triangulated Irregular Networks (TIN)**: TINs represent surfaces as networks of non-overlapping triangles. Each triangle's vertices have known positions and elevations. TINs efficiently represent surfaces with variable detail—small triangles in complex areas, larger triangles in flat regions. However, creating optimal TINs from gridded data requires sophisticated algorithms.

**Shaded Relief**: Artificial illumination creates the appearance of 3D topography even on 2D displays. The rendering algorithm calculates how much light each surface element receives based on its orientation relative to a virtual light source. Surfaces facing the light appear bright; surfaces facing away appear dark. The result mimics the shadows and highlights that help us perceive shape in the natural world.

**Multiple Directional Hillshading**: Single-source illumination can obscure features in shadows. Multi-directional hillshading combines illumination from multiple angles, ensuring all features are at least partially illuminated. This technique reveals detail in complex topography while maintaining the 3D appearance.

**Texture Mapping**: Overlaying additional information as textures enhances visualization. Backscatter intensity from sonar systems can be draped over bathymetric surfaces, showing both shape and acoustic character. Classified habitat maps, geological interpretations, or other derived products can be displayed as textures.

**Vertical Exaggeration**: Seafloor slopes are often subtle—large areas may have gradients of only a few percent. Vertical exaggeration amplifies the vertical scale relative to horizontal, making subtle features more apparent. However, excessive exaggeration can create misleading impressions. The WIA-OCEAN-005 standard recommends clearly indicating the vertical exaggeration factor.

## 5.4 Color Mapping and Symbolization

Effective color schemes enhance understanding while avoiding misinterpretation:

**Sequential Color Ramps**: These schemes use gradual color transitions to represent continuous depth variations. The traditional blue-to-white scheme represents deeper water as darker blue, shoaling to lighter blue and finally white. Other schemes use blue-green-yellow-red or custom ranges optimized for specific applications.

**Perceptually Uniform Color Scales**: Traditional rainbow color maps can be misleading because perceived color differences don't match data differences uniformly. Modern perceptually uniform schemes like Viridis ensure equal data steps produce equal perceived color steps.

**Bathymetric Color Palettes**: The ocean blue theme (#0EA5E9) specified in WIA-OCEAN-005 provides a starting point, but effective visualization often requires carefully designed color progressions:

```typescript
/**
 * WIA-OCEAN-005: Bathymetric Color Mapping
 * Ocean Blue Theme: #0EA5E9
 */

interface ColorRGB {
  r: number;  // 0-255
  g: number;  // 0-255
  b: number;  // 0-255
}

interface BathymetricColorScheme {
  name: string;
  colors: ColorRGB[];
  depths: number[];
}

/**
 * Create ocean-themed color palette
 */
function createOceanPalette(): BathymetricColorScheme {
  return {
    name: 'WIA Ocean Blue',
    colors: [
      { r: 14, g: 165, b: 233 },   // Shallow: #0EA5E9
      { r: 6, g: 182, b: 212 },    // Mid-shallow: #06B6D4
      { r: 8, g: 145, b: 178 },    // Mid: #0891B2
      { r: 14, g: 116, b: 144 },   // Mid-deep: #0E7490
      { r: 12, g: 74, b: 110 },    // Deep: #0C4A6E
      { r: 3, g: 24, b: 60 }       // Very deep: #03183C
    ],
    depths: [0, -50, -200, -1000, -4000, -6000]
  };
}

/**
 * Interpolate color for a given depth
 */
function getColorForDepth(
  depth: number,
  scheme: BathymetricColorScheme
): ColorRGB {
  // Find bracketing depth values
  let idx = 0;
  while (idx < scheme.depths.length - 1 && depth < scheme.depths[idx + 1]) {
    idx++;
  }

  if (idx === scheme.depths.length - 1) {
    return scheme.colors[idx];
  }

  // Linear interpolation between colors
  const d1 = scheme.depths[idx];
  const d2 = scheme.depths[idx + 1];
  const c1 = scheme.colors[idx];
  const c2 = scheme.colors[idx + 1];

  const t = (depth - d1) / (d2 - d1);

  return {
    r: Math.round(c1.r + t * (c2.r - c1.r)),
    g: Math.round(c1.g + t * (c2.g - c1.g)),
    b: Math.round(c1.b + t * (c2.b - c1.b))
  };
}

/**
 * Apply hillshading to enhance 3D perception
 */
function applyHillshade(
  elevation: number[][],
  azimuth: number,
  altitude: number,
  zFactor: number = 1.0
): number[][] {
  const rows = elevation.length;
  const cols = elevation[0].length;
  const hillshade: number[][] = Array(rows).fill(0).map(() => Array(cols).fill(0));

  const azimuthRad = azimuth * Math.PI / 180;
  const altitudeRad = altitude * Math.PI / 180;

  for (let i = 1; i < rows - 1; i++) {
    for (let j = 1; j < cols - 1; j++) {
      // Calculate slope and aspect
      const dzdx = ((elevation[i][j + 1] + 2 * elevation[i][j + 1] + elevation[i + 1][j + 1]) -
                    (elevation[i][j - 1] + 2 * elevation[i][j - 1] + elevation[i + 1][j - 1])) / 8;

      const dzdy = ((elevation[i + 1][j] + 2 * elevation[i + 1][j] + elevation[i + 1][j + 1]) -
                    (elevation[i - 1][j] + 2 * elevation[i - 1][j] + elevation[i - 1][j + 1])) / 8;

      const slope = Math.atan(zFactor * Math.sqrt(dzdx * dzdx + dzdy * dzdy));
      const aspect = Math.atan2(dzdy, -dzdx);

      // Calculate hillshade value
      const hillshadeValue = 255 * (
        (Math.cos(altitudeRad) * Math.cos(slope)) +
        (Math.sin(altitudeRad) * Math.sin(slope) * Math.cos(azimuthRad - aspect))
      );

      hillshade[i][j] = Math.max(0, Math.min(255, hillshadeValue));
    }
  }

  return hillshade;
}

/**
 * Combine bathymetry color with hillshading
 */
function combineColorAndShade(
  color: ColorRGB,
  shade: number,
  shadeWeight: number = 0.5
): ColorRGB {
  const shadeFactor = shade / 255;
  const blendedFactor = 1 - shadeWeight + shadeWeight * shadeFactor;

  return {
    r: Math.round(color.r * blendedFactor),
    g: Math.round(color.g * blendedFactor),
    b: Math.round(color.b * blendedFactor)
  };
}
```

## 5.5 Interactive 3D Visualization

Modern web technologies enable sophisticated 3D visualization directly in web browsers:

**WebGL Rendering**: The WebGL API provides access to GPU-accelerated 3D graphics in web browsers. Libraries like Three.js, Babylon.js, and Cesium simplify WebGL development, enabling creation of interactive 3D viewers without low-level graphics programming.

**Level-of-Detail Management**: Efficiently rendering large bathymetric datasets requires level-of-detail (LOD) techniques. The visualization system loads and displays only the data needed for the current view, automatically switching between resolution levels as the user zooms in or out.

**User Interaction**: Effective visualizations support intuitive interaction:
- Pan: Move the viewpoint horizontally
- Zoom: Adjust viewing distance
- Rotate: Change viewing angle
- Measure: Calculate distances, areas, and elevation differences
- Query: Display information about specific locations
- Animate: Create flythroughs and time-series visualizations

## 5.6 Advanced Visualization Techniques

Beyond basic surface rendering, advanced techniques reveal additional insights:

**Slope Analysis**: Calculating and visualizing slope gradients highlights steep areas, identifies potential hazards, and reveals geological structures. Slope maps are essential for engineering applications, habitat mapping, and geomorphological interpretation.

**Aspect Analysis**: Aspect (the compass direction a slope faces) affects oceanographic processes like current flow and sediment transport. Aspect visualization can reveal patterns in these processes.

**Curvature Analysis**: Curvature measures how slope changes across the surface. Positive curvature indicates ridges or peaks; negative curvature indicates valleys or depressions. Curvature visualization highlights subtle features invisible in simple depth displays.

**Volumetric Rendering**: For features with vertical extent (water column targets, suspended sediments, gas plumes), volumetric rendering techniques display 3D data volumes rather than just surfaces.

## 5.7 TypeScript 3D Visualization Example

Here's a TypeScript implementation for web-based 3D bathymetric visualization:

```typescript
/**
 * WIA-OCEAN-005: Web-Based 3D Bathymetric Visualization
 * Ocean Blue Theme: #0EA5E9
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';

interface BathymetricGrid {
  width: number;
  height: number;
  resolution: number;  // meters per pixel
  depths: Float32Array;
  bounds: GeographicBounds;
}

class BathymetricViewer {
  private scene: THREE.Scene;
  private camera: THREE.PerspectiveCamera;
  private renderer: THREE.WebGLRenderer;
  private controls: OrbitControls;
  private terrain: THREE.Mesh | null = null;

  constructor(container: HTMLElement) {
    // Initialize Three.js scene
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x001f3f);  // Deep ocean blue

    // Setup camera
    this.camera = new THREE.PerspectiveCamera(
      75,
      container.clientWidth / container.clientHeight,
      0.1,
      10000
    );
    this.camera.position.set(0, 500, 1000);

    // Setup renderer
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(this.renderer.domElement);

    // Setup controls
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;

    // Add lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    this.scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(1, 1, 1);
    this.scene.add(directionalLight);

    // Start animation loop
    this.animate();
  }

  /**
   * Load and display bathymetric data
   */
  loadBathymetry(grid: BathymetricGrid, verticalExaggeration: number = 10): void {
    // Remove existing terrain
    if (this.terrain) {
      this.scene.remove(this.terrain);
    }

    // Create geometry
    const geometry = new THREE.PlaneGeometry(
      grid.width * grid.resolution,
      grid.height * grid.resolution,
      grid.width - 1,
      grid.height - 1
    );

    // Apply elevations to vertices
    const positions = geometry.attributes.position.array as Float32Array;

    for (let i = 0; i < grid.width * grid.height; i++) {
      const depth = grid.depths[i];
      positions[i * 3 + 2] = -depth * verticalExaggeration;  // Z coordinate
    }

    geometry.computeVertexNormals();

    // Create color gradient based on depth
    const colors = new Float32Array(grid.width * grid.height * 3);
    const colorScheme = createOceanPalette();

    for (let i = 0; i < grid.width * grid.height; i++) {
      const depth = -grid.depths[i];
      const color = getColorForDepth(depth, colorScheme);

      colors[i * 3] = color.r / 255;
      colors[i * 3 + 1] = color.g / 255;
      colors[i * 3 + 2] = color.b / 255;
    }

    geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));

    // Create material
    const material = new THREE.MeshStandardMaterial({
      vertexColors: true,
      flatShading: false,
      metalness: 0.2,
      roughness: 0.8
    });

    // Create mesh
    this.terrain = new THREE.Mesh(geometry, material);
    this.terrain.rotation.x = -Math.PI / 2;
    this.scene.add(this.terrain);

    // Center camera on terrain
    this.camera.lookAt(0, 0, 0);
  }

  /**
   * Add contour lines
   */
  addContours(
    grid: BathymetricGrid,
    interval: number,
    verticalExaggeration: number = 10
  ): void {
    const material = new THREE.LineBasicMaterial({
      color: 0xffffff,
      opacity: 0.3,
      transparent: true
    });

    // Find depth range
    let minDepth = Infinity;
    let maxDepth = -Infinity;

    for (let i = 0; i < grid.depths.length; i++) {
      minDepth = Math.min(minDepth, grid.depths[i]);
      maxDepth = Math.max(maxDepth, grid.depths[i]);
    }

    // Generate contours at intervals
    for (let depth = Math.ceil(minDepth / interval) * interval;
         depth <= maxDepth;
         depth += interval) {

      const contour = this.extractContour(grid, depth, verticalExaggeration);
      if (contour) {
        this.scene.add(contour);
      }
    }
  }

  /**
   * Extract contour at specific depth using marching squares
   */
  private extractContour(
    grid: BathymetricGrid,
    depth: number,
    verticalExaggeration: number
  ): THREE.Line | null {
    const points: THREE.Vector3[] = [];

    // Simplified contour extraction
    // Production implementation would use marching squares algorithm

    for (let y = 0; y < grid.height - 1; y++) {
      for (let x = 0; x < grid.width - 1; x++) {
        const idx = y * grid.width + x;
        const d1 = grid.depths[idx];
        const d2 = grid.depths[idx + 1];

        // Check if contour crosses this edge
        if ((d1 <= depth && d2 >= depth) || (d1 >= depth && d2 <= depth)) {
          const t = (depth - d1) / (d2 - d1);
          const px = (x + t) * grid.resolution - (grid.width * grid.resolution) / 2;
          const py = y * grid.resolution - (grid.height * grid.resolution) / 2;
          const pz = -depth * verticalExaggeration;

          points.push(new THREE.Vector3(px, pz, -py));
        }
      }
    }

    if (points.length < 2) return null;

    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    const material = new THREE.LineBasicMaterial({
      color: 0xffffff,
      opacity: 0.3,
      transparent: true
    });

    return new THREE.Line(geometry, material);
  }

  /**
   * Calculate and visualize slope
   */
  createSlopeMap(grid: BathymetricGrid): Float32Array {
    const slopes = new Float32Array(grid.width * grid.height);

    for (let y = 1; y < grid.height - 1; y++) {
      for (let x = 1; x < grid.width - 1; x++) {
        const idx = y * grid.width + x;

        const dzdx = (grid.depths[idx + 1] - grid.depths[idx - 1]) /
                     (2 * grid.resolution);
        const dzdy = (grid.depths[idx + grid.width] - grid.depths[idx - grid.width]) /
                     (2 * grid.resolution);

        const slope = Math.atan(Math.sqrt(dzdx * dzdx + dzdy * dzdy));
        slopes[idx] = slope * 180 / Math.PI;  // Convert to degrees
      }
    }

    return slopes;
  }

  /**
   * Animation loop
   */
  private animate = (): void => {
    requestAnimationFrame(this.animate);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  };

  /**
   * Handle window resize
   */
  resize(width: number, height: number): void {
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  /**
   * Export view as image
   */
  exportImage(): string {
    return this.renderer.domElement.toDataURL('image/png');
  }
}

// Example usage
const viewer = new BathymetricViewer(document.getElementById('viewer')!);

const exampleGrid: BathymetricGrid = {
  width: 100,
  height: 100,
  resolution: 10,  // 10 meters per pixel
  depths: new Float32Array(100 * 100).map(() => Math.random() * 1000),
  bounds: {
    north: 36.0,
    south: 35.0,
    east: 140.0,
    west: 139.0
  }
};

viewer.loadBathymetry(exampleGrid, 20);  // 20x vertical exaggeration
viewer.addContours(exampleGrid, 100, 20);  // 100m contour interval
```

## Summary

3D modeling and visualization transform bathymetric data into intuitive representations that reveal the ocean floor's complexity and beauty. Effective visualization requires understanding of rendering techniques, color theory, human perception, and interactive graphics programming.

This chapter explored digital elevation models, surface rendering techniques, color mapping strategies, interactive visualization, and advanced analysis methods. We demonstrated these concepts through comprehensive TypeScript implementations using modern web graphics technologies.

The next chapter will examine geological feature detection, showing how automated algorithms can identify and classify seafloor features from bathymetric data.

**弘益人間 (Benefit All Humanity)** - Powerful visualization makes ocean floor data accessible to everyone, from scientists to students, fostering understanding and appreciation of the underwater world.

---

*WIA-OCEAN-005: Ocean Floor Mapping*
*© 2025 SmileStory Inc. / WIA - World Certification Industry Association*
*弘益人間 (홍익인간) - Benefit All Humanity*
