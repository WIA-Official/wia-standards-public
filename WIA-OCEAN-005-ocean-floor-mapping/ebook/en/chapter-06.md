# Chapter 6: Geological Feature Detection

## 6.1 Automated Feature Recognition

The ocean floor exhibits extraordinary geological diversity—seamounts rise kilometers above surrounding abyssal plains, submarine canyons carve deep into continental slopes, hydrothermal vents build chimneys of mineral deposits, and tectonic processes create ridges, trenches, and fault systems. Identifying and characterizing these features manually from bathymetric data would be prohibitively time-consuming for large datasets. Automated feature detection algorithms accelerate this process, enabling systematic cataloging of seafloor features across vast ocean areas.

Feature detection combines computer vision techniques, geomorphometric analysis, and domain-specific knowledge about how different geological features appear in bathymetric data. These algorithms analyze surface characteristics like slope, curvature, rugosity, and topographic position to identify and classify features. Machine learning approaches can recognize subtle patterns that distinguish feature types, learning from training data labeled by expert geologists.

The WIA-OCEAN-005 standard specifies requirements for feature detection workflows, including algorithm validation, accuracy assessment, and metadata documentation. Standardized feature catalogs enable comparison across studies and integration of features detected by different algorithms or research groups.

## 6.2 Geomorphometric Analysis

Geomorphometric parameters quantify surface shape characteristics that help identify geological features:

**Slope**: The first derivative of elevation measures surface gradient. Slopes reveal transitions between flat abyssal plains and rising seamounts, delineate canyon walls, and identify unstable areas prone to submarine landslides. Slope analysis provides fundamental information about seafloor geomorphology.

**Aspect**: The compass direction a slope faces affects oceanographic processes and ecological communities. North-facing slopes in the northern hemisphere receive different light patterns than south-facing slopes. Aspect analysis reveals directional patterns in erosion, sedimentation, and current flow.

**Curvature**: The second derivative of elevation characterizes how slope changes across the surface. Profile curvature (parallel to slope direction) distinguishes convex features like ridges from concave features like valleys. Plan curvature (perpendicular to slope) indicates convergence or divergence of flow. Mean curvature combines both components.

**Rugosity**: Surface roughness at various scales indicates substrate type and geological history. Smooth surfaces suggest fine sediment deposition or erosion. Rough surfaces indicate hard substrate, volcanic construction, or tectonic deformation. Multi-scale rugosity analysis reveals patterns invisible at single scales.

**Topographic Position Index (TPI)**: TPI compares each point's elevation to the mean elevation of surrounding areas. Positive values indicate ridges or peaks; negative values indicate valleys or depressions. TPI enables automated identification of topographic highs and lows.

**Bathymetric Position Index (BPI)**: Similar to TPI but specifically designed for bathymetric data. BPI at multiple scales characterizes features from small boulders to large seamounts.

## 6.3 Seamount Detection

Seamounts—underwater mountains that rise at least 1,000 meters above the surrounding seafloor—number in the tens of thousands globally. These features create unique habitats, affect ocean circulation, and hold clues about Earth's volcanic and tectonic history. Automated seamount detection from bathymetric data enables systematic study of these important features.

**Detection Algorithm**: Seamount detection typically involves identifying local topographic highs that meet size and shape criteria. The basic algorithm:
1. Calculate BPI or TPI at appropriate scales
2. Threshold to identify potential summits
3. Apply shape criteria (height, basal area, slope consistency)
4. Merge nearby detections into single features
5. Calculate morphometric parameters for each seamount

**Classification**: Seamounts vary in size, shape, and origin. Classification schemes distinguish:
- Guyots: Flat-topped seamounts (eroded at sea level, then subsided)
- Volcanic cones: Steep-sided features with conical profiles
- Seamount chains: Linear arrangements indicating plate motion over hotspots
- Ridges: Elongated volcanic or tectonic features

**Morphometric Parameters**: Quantifying seamount characteristics enables comparative studies:
- Summit depth
- Base depth
- Height above seafloor
- Basal area and diameter
- Slope statistics
- Elongation and orientation
- Volume

## 6.4 Canyon and Channel Detection

Submarine canyons incise continental slopes and shelves, transporting sediment from shallow to deep water. These features range from small gullies to Grand Canyon-scale systems extending hundreds of kilometers. Automated detection identifies these important conduits for sediment, nutrients, and pollutants.

**Detection Approaches**: Canyon detection exploits the distinctive geomorphometric signature of these features:
- Negative TPI/BPI values (topographic lows)
- High slope walls flanking the canyon axis
- Linear or sinuous planform geometry
- Downslope orientation
- Connection to terrestrial drainage systems (for some canyons)

**Centerline Extraction**: Identifying canyon axes enables quantification of sinuosity, gradient, and connectivity. Algorithms trace the lowest elevation path from canyon heads to mouths, handling bifurcations and confluences.

**Width and Depth Profiling**: Cross-canyon profiles reveal wall steepness, canyon width evolution, and evidence for different formation mechanisms. V-shaped canyons suggest erosion; U-shaped canyons indicate glacial or sustained turbidity current activity.

## 6.5 Tectonic Feature Identification

Tectonic processes create distinctive seafloor features that record Earth's dynamic geology:

**Mid-Ocean Ridges**: These submarine mountain ranges mark divergent plate boundaries where new oceanic crust forms. Ridge detection identifies:
- Linear topographic highs
- Central rift valleys
- Transform faults offsetting ridge segments
- Volcanic construction features

**Subduction Zones**: Where tectonic plates converge, deep ocean trenches form. Detection algorithms identify:
- Linear topographic lows (trenches)
- Parallel volcanic arcs
- Accretionary prisms
- Outer rise features

**Fault Systems**: Seafloor faults create escarpments and lineaments detectable in bathymetry. Automated detection uses:
- Linear feature extraction
- Slope discontinuity analysis
- Directional filtering
- Pattern recognition

## 6.6 TypeScript Feature Detection Implementation

Here's a comprehensive TypeScript implementation for geological feature detection:

```typescript
/**
 * WIA-OCEAN-005: Geological Feature Detection
 * Ocean Blue Theme: #0EA5E9
 */

interface GridCell {
  x: number;
  y: number;
  depth: number;
  slope: number;
  aspect: number;
  curvature: number;
  bpi: number;
}

interface DetectedFeature {
  id: string;
  type: 'seamount' | 'canyon' | 'ridge' | 'trench' | 'fault';
  confidence: number;
  bounds: GeographicBounds;
  properties: Map<string, number>;
  geometry: Position[];
}

/**
 * Calculate Bathymetric Position Index (BPI)
 */
function calculateBPI(
  depths: number[][],
  x: number,
  y: number,
  innerRadius: number,
  outerRadius: number
): number {
  const width = depths[0].length;
  const height = depths.length;

  let sumOuter = 0;
  let countOuter = 0;

  for (let dy = -outerRadius; dy <= outerRadius; dy++) {
    for (let dx = -outerRadius; dx <= outerRadius; dx++) {
      const distance = Math.sqrt(dx * dx + dy * dy);

      if (distance > innerRadius && distance <= outerRadius) {
        const nx = x + dx;
        const ny = y + dy;

        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
          sumOuter += depths[ny][nx];
          countOuter++;
        }
      }
    }
  }

  if (countOuter === 0) return 0;

  const meanOuter = sumOuter / countOuter;
  return depths[y][x] - meanOuter;
}

/**
 * Calculate multi-scale BPI
 */
function calculateMultiScaleBPI(
  depths: number[][],
  scales: Array<{ inner: number; outer: number }>
): Map<string, number[][]> {
  const width = depths[0].length;
  const height = depths.length;
  const bpiMaps = new Map<string, number[][]>();

  for (const scale of scales) {
    const bpi: number[][] = Array(height).fill(0)
      .map(() => Array(width).fill(0));

    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        bpi[y][x] = calculateBPI(depths, x, y, scale.inner, scale.outer);
      }
    }

    bpiMaps.set(`${scale.inner}-${scale.outer}`, bpi);
  }

  return bpiMaps;
}

/**
 * Detect seamounts using BPI analysis
 */
function detectSeamounts(
  depths: number[][],
  resolution: number,
  minHeight: number = 1000,
  minArea: number = 100
): DetectedFeature[] {
  const width = depths[0].length;
  const height = depths.length;

  // Calculate BPI at seamount scale (10-50 pixels)
  const bpiSmall = Array(height).fill(0).map(() => Array(width).fill(0));
  const bpiLarge = Array(height).fill(0).map(() => Array(width).fill(0));

  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      bpiSmall[y][x] = calculateBPI(depths, x, y, 0, 10);
      bpiLarge[y][x] = calculateBPI(depths, x, y, 10, 50);
    }
  }

  // Identify potential seamount summits
  const summits: Array<{ x: number; y: number; bpi: number }> = [];

  for (let y = 1; y < height - 1; y++) {
    for (let x = 1; x < width - 1; x++) {
      // High BPI at both scales indicates seamount
      if (bpiSmall[y][x] > 100 && bpiLarge[y][x] > 50) {
        // Check if local maximum
        let isMaximum = true;
        for (let dy = -1; dy <= 1; dy++) {
          for (let dx = -1; dx <= 1; dx++) {
            if (dx === 0 && dy === 0) continue;
            if (depths[y + dy][x + dx] < depths[y][x]) {
              isMaximum = false;
              break;
            }
          }
          if (!isMaximum) break;
        }

        if (isMaximum) {
          summits.push({ x, y, bpi: bpiSmall[y][x] });
        }
      }
    }
  }

  // Grow seamounts from summits
  const seamounts: DetectedFeature[] = [];

  for (let i = 0; i < summits.length; i++) {
    const summit = summits[i];
    const seamount = growSeamount(
      depths,
      summit.x,
      summit.y,
      resolution,
      minHeight,
      minArea
    );

    if (seamount) {
      seamount.id = `seamount_${i}`;
      seamounts.push(seamount);
    }
  }

  return seamounts;
}

/**
 * Grow seamount from summit using region growing
 */
function growSeamount(
  depths: number[][],
  summitX: number,
  summitY: number,
  resolution: number,
  minHeight: number,
  minArea: number
): DetectedFeature | null {
  const width = depths[0].length;
  const height = depths.length;
  const summitDepth = depths[summitY][summitX];

  // Region growing
  const visited = Array(height).fill(0).map(() => Array(width).fill(false));
  const seamountPixels: Array<{ x: number; y: number }> = [];
  const queue: Array<{ x: number; y: number }> = [{ x: summitX, y: summitY }];

  visited[summitY][summitX] = true;

  while (queue.length > 0) {
    const pixel = queue.shift()!;
    seamountPixels.push(pixel);

    // Check neighbors
    for (let dy = -1; dy <= 1; dy++) {
      for (let dx = -1; dx <= 1; dx++) {
        if (dx === 0 && dy === 0) continue;

        const nx = pixel.x + dx;
        const ny = pixel.y + dy;

        if (nx >= 0 && nx < width && ny >= 0 && ny < height && !visited[ny][nx]) {
          visited[ny][nx] = true;

          // Include if depth is within range of summit
          // and still rising toward summit
          const depthDiff = depths[ny][nx] - summitDepth;
          if (depthDiff > 0 && depthDiff < minHeight) {
            queue.push({ x: nx, y: ny });
          }
        }
      }
    }
  }

  // Check size criteria
  const area = seamountPixels.length * resolution * resolution;
  if (area < minArea * 1e6) return null;  // Convert km² to m²

  // Find base depth
  let baseDepth = summitDepth;
  for (const pixel of seamountPixels) {
    baseDepth = Math.max(baseDepth, depths[pixel.y][pixel.x]);
  }

  const seamountHeight = baseDepth - summitDepth;
  if (seamountHeight < minHeight) return null;

  // Calculate bounds
  let minX = width, maxX = 0, minY = height, maxY = 0;
  for (const pixel of seamountPixels) {
    minX = Math.min(minX, pixel.x);
    maxX = Math.max(maxX, pixel.x);
    minY = Math.min(minY, pixel.y);
    maxY = Math.max(maxY, pixel.y);
  }

  const properties = new Map<string, number>();
  properties.set('summitDepth', summitDepth);
  properties.set('baseDepth', baseDepth);
  properties.set('height', seamountHeight);
  properties.set('area', area);
  properties.set('basalDiameter', Math.sqrt(area / Math.PI) * 2);

  return {
    id: '',
    type: 'seamount',
    confidence: 0.85,
    bounds: {
      north: maxY * resolution,
      south: minY * resolution,
      east: maxX * resolution,
      west: minX * resolution
    },
    properties,
    geometry: seamountPixels.map(p => ({
      latitude: p.y * resolution,
      longitude: p.x * resolution,
      altitude: depths[p.y][p.x]
    }))
  };
}

/**
 * Detect submarine canyons
 */
function detectCanyons(
  depths: number[][],
  resolution: number,
  minLength: number = 5000
): DetectedFeature[] {
  const width = depths[0].length;
  const height = depths.length;

  // Calculate BPI at canyon scale
  const bpi = Array(height).fill(0).map(() => Array(width).fill(0));

  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      bpi[y][x] = calculateBPI(depths, x, y, 5, 20);
    }
  }

  // Identify canyon pixels (negative BPI)
  const canyonPixels = Array(height).fill(0).map(() => Array(width).fill(false));

  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      if (bpi[y][x] < -50) {  // Threshold for canyon
        canyonPixels[y][x] = true;
      }
    }
  }

  // Connected component labeling
  const labels = Array(height).fill(0).map(() => Array(width).fill(0));
  let currentLabel = 1;

  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      if (canyonPixels[y][x] && labels[y][x] === 0) {
        floodFill(canyonPixels, labels, x, y, currentLabel);
        currentLabel++;
      }
    }
  }

  // Extract canyons that meet length criteria
  const canyons: DetectedFeature[] = [];

  for (let label = 1; label < currentLabel; label++) {
    const pixels: Array<{ x: number; y: number }> = [];

    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        if (labels[y][x] === label) {
          pixels.push({ x, y });
        }
      }
    }

    // Calculate canyon length (simplified as max extent)
    let minX = width, maxX = 0, minY = height, maxY = 0;
    for (const pixel of pixels) {
      minX = Math.min(minX, pixel.x);
      maxX = Math.max(maxX, pixel.x);
      minY = Math.min(minY, pixel.y);
      maxY = Math.max(maxY, pixel.y);
    }

    const length = Math.sqrt(
      Math.pow((maxX - minX) * resolution, 2) +
      Math.pow((maxY - minY) * resolution, 2)
    );

    if (length >= minLength) {
      const properties = new Map<string, number>();
      properties.set('length', length);
      properties.set('area', pixels.length * resolution * resolution);

      canyons.push({
        id: `canyon_${label}`,
        type: 'canyon',
        confidence: 0.75,
        bounds: {
          north: maxY * resolution,
          south: minY * resolution,
          east: maxX * resolution,
          west: minX * resolution
        },
        properties,
        geometry: pixels.map(p => ({
          latitude: p.y * resolution,
          longitude: p.x * resolution,
          altitude: depths[p.y][p.x]
        }))
      });
    }
  }

  return canyons;
}

/**
 * Flood fill for connected component labeling
 */
function floodFill(
  binary: boolean[][],
  labels: number[][],
  x: number,
  y: number,
  label: number
): void {
  const width = binary[0].length;
  const height = binary.length;
  const queue: Array<{ x: number; y: number }> = [{ x, y }];

  labels[y][x] = label;

  while (queue.length > 0) {
    const pixel = queue.shift()!;

    for (let dy = -1; dy <= 1; dy++) {
      for (let dx = -1; dx <= 1; dx++) {
        const nx = pixel.x + dx;
        const ny = pixel.y + dy;

        if (nx >= 0 && nx < width && ny >= 0 && ny < height &&
            binary[ny][nx] && labels[ny][nx] === 0) {
          labels[ny][nx] = label;
          queue.push({ x: nx, y: ny });
        }
      }
    }
  }
}

// Example usage
const exampleDepths = Array(200).fill(0).map(() =>
  Array(200).fill(0).map(() => 4000 + Math.random() * 1000)
);

const seamounts = detectSeamounts(exampleDepths, 100, 1000, 100);
console.log(`Detected ${seamounts.length} seamounts`);

const canyons = detectCanyons(exampleDepths, 100, 5000);
console.log(`Detected ${canyons.length} canyons`);
```

## Summary

Automated geological feature detection transforms bathymetric data into structured knowledge about seafloor geology. Geomorphometric analysis quantifies surface characteristics, enabling algorithmic identification of seamounts, canyons, ridges, trenches, and other features. These techniques accelerate research, enable systematic global inventories, and reveal patterns invisible to manual analysis.

This chapter explored geomorphometric parameters, seamount detection algorithms, canyon identification methods, tectonic feature recognition, and practical implementations in TypeScript. The next chapter will examine global mapping initiatives working to chart the entire ocean floor.

**弘益人間 (Benefit All Humanity)** - Understanding seafloor geology helps us protect marine ecosystems, manage resources sustainably, and predict hazards, benefiting all of humanity.

---

*WIA-OCEAN-005: Ocean Floor Mapping*
*© 2025 SmileStory Inc. / WIA - World Certification Industry Association*
*弘益人間 (홍익인간) - Benefit All Humanity*
