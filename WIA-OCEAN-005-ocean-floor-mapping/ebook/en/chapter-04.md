# Chapter 4: Bathymetric Data Processing

## 4.1 From Raw Data to Finished Maps

Raw bathymetric data collected by sonar systems requires extensive processing before it becomes a usable seafloor map. This processing chain involves numerous steps, each designed to remove artifacts, correct errors, and transform millions of individual depth measurements into a coherent representation of seafloor topography. Understanding this workflow is essential for producing high-quality bathymetric products and interpreting their accuracy and limitations.

The data processing pipeline can be conceptualized as a series of transformations, each taking data from one state to a more refined state. Raw sonar returns become corrected soundings. Individual soundings become gridded surfaces. Gridded surfaces become final map products with appropriate metadata and quality indicators. At each stage, human judgment and quality control play crucial roles in ensuring accuracy and reliability.

Modern processing software automates many steps in this workflow, but automation cannot replace the expertise of trained hydrographers. Skilled operators recognize artifacts, understand the physical processes affecting measurements, and make informed decisions about data cleaning, parameter selection, and uncertainty estimation. The WIA-OCEAN-005 standard provides guidelines for processing workflows while recognizing that specific applications may require tailored approaches.

## 4.2 Data Cleaning and Artifact Removal

Acoustic data inevitably contains erroneous soundings caused by various factors. Identifying and removing these artifacts while preserving real seafloor features requires careful analysis:

**Outlier Detection**: Statistical methods identify soundings that deviate significantly from neighboring measurements. Simple techniques include comparison to local median or mean values. More sophisticated approaches use spatial filters, machine learning classifiers, or multi-resolution analysis. The challenge lies in distinguishing genuine outliers from real seafloor features like boulders or pinnacles.

**False Bottom Detection**: Under certain conditions, sonar systems can detect multiple returns from a single transmitted pulse. Side-lobe returns, second-bottom echoes, and water column targets can create spurious depth measurements. Automated detection algorithms look for characteristic patterns, but human review remains important for complex cases.

**Fish and Vegetation**: In shallow water, fish schools, kelp forests, and other biological features can produce returns above the true seafloor. These returns must be identified and removed to avoid creating artificial shallow areas in the final map.

**Ship Motion Artifacts**: Imperfect motion compensation can create systematic errors, particularly in rough seas. These artifacts often appear as undulations with characteristic wavelengths related to wave periods. Specialized filters can remove these artifacts while preserving real seafloor topography.

## 4.3 Sound Velocity Corrections

Accurate knowledge of sound velocity throughout the water column is critical for correct depth calculation and beam steering. Errors in assumed sound velocity directly translate to depth errors:

**Sound Velocity Profiles (SVP)**: Temperature, salinity, and pressure all affect sound speed in seawater. SVPs are typically measured using conductivity-temperature-depth (CTD) sensors or expendable bathythermographs (XBT). Modern surveys measure SVPs at regular intervals throughout the survey, with frequency depending on oceanographic variability.

**Ray Tracing**: Rather than assuming straight-line propagation, sophisticated processing applies ray tracing algorithms that account for sound refraction through the water column. Snell's law governs how sound paths bend when passing through layers with different velocities. Proper ray tracing is particularly important for outer beams and in areas with strong sound velocity gradients.

**Uncertainty Estimation**: Even with frequent SVP measurements, uncertainty in the sound velocity profile contributes to overall depth uncertainty. This contribution can be estimated through sensitivity analysis, testing how depth estimates change with perturbations to the SVP.

## 4.4 Positioning and Datum Transformations

Accurate positioning is fundamental to georeferencing bathymetric data. Modern surveys use Global Navigation Satellite Systems (GNSS), but several processing steps are required:

**Differential Corrections**: Raw GNSS positions have meter-level accuracy. Differential techniques (DGPS, RTK, PPP) improve accuracy to the decimeter or centimeter level. Post-processed kinematic (PPK) processing can achieve high accuracy even when real-time corrections were unavailable during acquisition.

**Antenna Offsets**: The GNSS antenna is typically not co-located with the sonar transducer. Precise measurement of offsets and application of vessel attitude data are required to determine the true position of the sonar at each measurement instant.

**Datum Transformations**: Bathymetric data must be referenced to an appropriate horizontal and vertical datum. Horizontal datums define the reference ellipsoid for latitude and longitude. Vertical datums specify the zero reference for depth measurements (often chart datum or mean lower low water for navigation charts). Proper transformation between datums requires careful attention to geoid models and local datum realizations.

**Tidal Corrections**: In shallow water, tidal variations can be significant relative to total water depth. Tidal corrections reference soundings to a consistent vertical datum, accounting for water level variations during the survey. Modern approaches increasingly use ellipsoidally referenced surveying, eliminating the need for traditional tide corrections.

## 4.5 Gridding and Interpolation

Individual soundings must be interpolated onto a regular grid for visualization and analysis. This seemingly simple step involves important choices:

**Grid Resolution**: The grid cell size should be appropriate for the data density and intended application. As a general rule, grid spacing should be 2-5 times the point spacing in the data. Finer grids do not add information and may create misleading artifacts through over-interpolation. Coarser grids discard information and may miss important features.

**Interpolation Algorithms**: Numerous interpolation methods exist, each with advantages and disadvantages:
- **Nearest neighbor**: Simple and fast, preserves data values, but creates blocky appearance
- **Bilinear/bicubic**: Smooth results, but may create artificial extrema
- **Inverse distance weighting**: Intuitive, locally adaptive, widely used
- **Kriging**: Statistically rigorous, provides uncertainty estimates, computationally intensive
- **Natural neighbor**: Adapts to irregular data distribution, avoids edge effects
- **Spline methods**: Very smooth, but can create unrealistic oscillations

**Uncertainty Propagation**: Grid cells should include not only interpolated depth values but also uncertainty estimates. These estimates account for measurement uncertainty, interpolation uncertainty, and data density. Areas with sparse data have higher uncertainty than densely sampled areas.

## 4.6 Quality Control and Validation

Rigorous quality control ensures the reliability of bathymetric products:

**Cross-line Analysis**: Surveying the same area from perpendicular directions enables comparison of independent measurements. Differences between crossings indicate systematic errors or uncertainty. The WIA-OCEAN-005 standard specifies acceptable cross-line discrepancies for different survey orders.

**Junction Analysis**: Where new surveys overlap existing data, junction analysis compares the datasets. Large discrepancies may indicate errors in the new survey, errors in the legacy data, or genuine seafloor changes.

**Statistical Quality Metrics**: Quantitative metrics characterize data quality:
- **Vertical accuracy**: Typically expressed as 95% confidence level (1.96 standard deviations)
- **Horizontal accuracy**: Position uncertainty of depth measurements
- **Data density**: Soundings per unit area
- **Feature detection**: Minimum detectable object size

**Validation Methods**: Independent validation provides confidence in processed data:
- Comparison with existing datasets
- Tie-line analysis
- Comparison with independent positioning (e.g., diver depths, ROV observations)
- Geomorphological plausibility checks

## 4.7 TypeScript Data Processing Implementation

Here's a comprehensive TypeScript implementation for bathymetric data processing:

```typescript
/**
 * WIA-OCEAN-005: Bathymetric Data Processing
 * Ocean Blue Theme: #0EA5E9
 */

interface BathymetricSounding {
  position: Position;
  depth: number;
  uncertainty: number;
  timestamp: Date;
  quality: number;
  source: string;
}

interface ProcessingParameters {
  outlierThreshold: number;       // Standard deviations
  gridResolution: number;          // Meters
  searchRadius: number;            // Meters
  minDataDensity: number;          // Soundings per cell minimum
  interpolationMethod: 'idw' | 'kriging' | 'natural_neighbor';
}

/**
 * Remove outliers using statistical filtering
 */
function removeOutliers(
  soundings: BathymetricSounding[],
  threshold: number
): BathymetricSounding[] {
  const filtered: BathymetricSounding[] = [];

  for (let i = 0; i < soundings.length; i++) {
    const sounding = soundings[i];
    const neighbors = findNeighbors(soundings, sounding.position, 50); // 50m radius

    if (neighbors.length < 3) {
      // Not enough neighbors for reliable filtering
      filtered.push(sounding);
      continue;
    }

    const depths = neighbors.map(n => n.depth);
    const median = calculateMedian(depths);
    const mad = calculateMAD(depths, median);

    // Modified z-score
    const score = Math.abs(sounding.depth - median) / (1.4826 * mad);

    if (score < threshold || mad === 0) {
      filtered.push(sounding);
    }
  }

  return filtered;
}

function findNeighbors(
  soundings: BathymetricSounding[],
  position: Position,
  radius: number
): BathymetricSounding[] {
  return soundings.filter(s => {
    const distance = calculateDistance(position, s.position);
    return distance <= radius && distance > 0;
  });
}

function calculateDistance(p1: Position, p2: Position): number {
  const R = 6371000; // Earth radius in meters
  const lat1 = p1.latitude * Math.PI / 180;
  const lat2 = p2.latitude * Math.PI / 180;
  const dLat = (p2.latitude - p1.latitude) * Math.PI / 180;
  const dLon = (p2.longitude - p1.longitude) * Math.PI / 180;

  const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
            Math.cos(lat1) * Math.cos(lat2) *
            Math.sin(dLon / 2) * Math.sin(dLon / 2);
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

  return R * c;
}

function calculateMedian(values: number[]): number {
  const sorted = [...values].sort((a, b) => a - b);
  const mid = Math.floor(sorted.length / 2);
  return sorted.length % 2 === 0 ?
    (sorted[mid - 1] + sorted[mid]) / 2 :
    sorted[mid];
}

function calculateMAD(values: number[], median: number): number {
  const deviations = values.map(v => Math.abs(v - median));
  return calculateMedian(deviations);
}

/**
 * Apply sound velocity corrections using ray tracing
 */
interface SoundVelocityLayer {
  depth: number;
  velocity: number;
}

function applySoundVelocityCorrection(
  sounding: BathymetricSounding,
  svp: SoundVelocityLayer[],
  beamAngle: number
): BathymetricSounding {
  // Simplified ray tracing
  // In production, use full Snell's law ray tracing

  let totalDepth = 0;
  let totalHorizontal = 0;
  let currentAngle = beamAngle;

  for (let i = 0; i < svp.length - 1; i++) {
    const layer = svp[i];
    const nextLayer = svp[i + 1];
    const layerThickness = nextLayer.depth - layer.depth;

    // Apply Snell's law at layer boundary
    const refractedAngle = Math.asin(
      (nextLayer.velocity / layer.velocity) * Math.sin(currentAngle * Math.PI / 180)
    ) * 180 / Math.PI;

    // Calculate path through layer
    const pathAngle = (currentAngle + refractedAngle) / 2;
    const slantPath = layerThickness / Math.cos(pathAngle * Math.PI / 180);
    const horizontalPath = slantPath * Math.sin(pathAngle * Math.PI / 180);

    totalDepth += layerThickness;
    totalHorizontal += horizontalPath;
    currentAngle = refractedAngle;

    if (totalDepth >= sounding.depth) break;
  }

  // Return corrected sounding
  return {
    ...sounding,
    depth: totalDepth,
    position: offsetPosition(sounding.position, totalHorizontal, beamAngle)
  };
}

function offsetPosition(
  position: Position,
  distance: number,
  bearing: number
): Position {
  const R = 6371000;
  const lat1 = position.latitude * Math.PI / 180;
  const lon1 = position.longitude * Math.PI / 180;
  const brng = bearing * Math.PI / 180;

  const lat2 = Math.asin(
    Math.sin(lat1) * Math.cos(distance / R) +
    Math.cos(lat1) * Math.sin(distance / R) * Math.cos(brng)
  );

  const lon2 = lon1 + Math.atan2(
    Math.sin(brng) * Math.sin(distance / R) * Math.cos(lat1),
    Math.cos(distance / R) - Math.sin(lat1) * Math.sin(lat2)
  );

  return {
    latitude: lat2 * 180 / Math.PI,
    longitude: lon2 * 180 / Math.PI,
    altitude: position.altitude
  };
}

/**
 * Grid bathymetric data using inverse distance weighting
 */
interface GridCell {
  depth: number;
  uncertainty: number;
  density: number;
}

function gridBathymetry(
  soundings: BathymetricSounding[],
  params: ProcessingParameters
): Map<string, GridCell> {
  const grid = new Map<string, GridCell>();

  // Determine grid bounds
  const bounds = calculateBounds(soundings);

  // Create grid cells
  const minX = Math.floor(bounds.west / params.gridResolution);
  const maxX = Math.ceil(bounds.east / params.gridResolution);
  const minY = Math.floor(bounds.south / params.gridResolution);
  const maxY = Math.ceil(bounds.north / params.gridResolution);

  for (let x = minX; x <= maxX; x++) {
    for (let y = minY; y <= maxY; y++) {
      const cellCenter: Position = {
        latitude: (y + 0.5) * params.gridResolution,
        longitude: (x + 0.5) * params.gridResolution,
        altitude: 0
      };

      const cellSoundings = findNeighbors(
        soundings,
        cellCenter,
        params.searchRadius
      );

      if (cellSoundings.length >= params.minDataDensity) {
        const gridCell = interpolateCell(
          cellCenter,
          cellSoundings,
          params.interpolationMethod
        );

        grid.set(`${x},${y}`, gridCell);
      }
    }
  }

  return grid;
}

function calculateBounds(soundings: BathymetricSounding[]): GeographicBounds {
  let minLat = Infinity, maxLat = -Infinity;
  let minLon = Infinity, maxLon = -Infinity;

  for (const sounding of soundings) {
    minLat = Math.min(minLat, sounding.position.latitude);
    maxLat = Math.max(maxLat, sounding.position.latitude);
    minLon = Math.min(minLon, sounding.position.longitude);
    maxLon = Math.max(maxLon, sounding.position.longitude);
  }

  return { north: maxLat, south: minLat, east: maxLon, west: minLon };
}

function interpolateCell(
  center: Position,
  soundings: BathymetricSounding[],
  method: string
): GridCell {
  if (method === 'idw') {
    return inverseDistanceWeighting(center, soundings, 2);  // power = 2
  }
  // Other methods would be implemented similarly
  return inverseDistanceWeighting(center, soundings, 2);
}

function inverseDistanceWeighting(
  center: Position,
  soundings: BathymetricSounding[],
  power: number
): GridCell {
  let weightedDepth = 0;
  let weightedUncertainty = 0;
  let totalWeight = 0;

  for (const sounding of soundings) {
    const distance = calculateDistance(center, sounding.position);
    const weight = distance === 0 ? 1e10 : 1 / Math.pow(distance, power);

    weightedDepth += sounding.depth * weight;
    weightedUncertainty += sounding.uncertainty * sounding.uncertainty * weight;
    totalWeight += weight;
  }

  return {
    depth: weightedDepth / totalWeight,
    uncertainty: Math.sqrt(weightedUncertainty / totalWeight),
    density: soundings.length
  };
}

/**
 * Perform cross-line analysis for quality control
 */
interface CrossLineStats {
  meanDifference: number;
  stdDeviation: number;
  maxDifference: number;
  pointCount: number;
}

function crossLineAnalysis(
  mainLines: BathymetricSounding[],
  crossLines: BathymetricSounding[],
  tolerance: number
): CrossLineStats {
  const differences: number[] = [];

  for (const crossPoint of crossLines) {
    const nearestMain = findNearest(mainLines, crossPoint.position);

    if (nearestMain) {
      const distance = calculateDistance(
        crossPoint.position,
        nearestMain.position
      );

      if (distance < tolerance) {
        differences.push(crossPoint.depth - nearestMain.depth);
      }
    }
  }

  const mean = differences.reduce((a, b) => a + b, 0) / differences.length;
  const variance = differences.reduce(
    (sum, d) => sum + Math.pow(d - mean, 2),
    0
  ) / differences.length;

  return {
    meanDifference: mean,
    stdDeviation: Math.sqrt(variance),
    maxDifference: Math.max(...differences.map(Math.abs)),
    pointCount: differences.length
  };
}

function findNearest(
  soundings: BathymetricSounding[],
  position: Position
): BathymetricSounding | null {
  let nearest: BathymetricSounding | null = null;
  let minDistance = Infinity;

  for (const sounding of soundings) {
    const distance = calculateDistance(position, sounding.position);
    if (distance < minDistance) {
      minDistance = distance;
      nearest = sounding;
    }
  }

  return nearest;
}

// Example usage
const soundings: BathymetricSounding[] = [
  {
    position: { latitude: 35.5, longitude: 139.7, altitude: 0 },
    depth: 100.5,
    uncertainty: 0.5,
    timestamp: new Date(),
    quality: 95,
    source: 'multibeam'
  },
  // ... more soundings
];

const params: ProcessingParameters = {
  outlierThreshold: 3.0,
  gridResolution: 10,  // 10 meter grid
  searchRadius: 50,
  minDataDensity: 5,
  interpolationMethod: 'idw'
};

// Process the data
const cleaned = removeOutliers(soundings, params.outlierThreshold);
const gridded = gridBathymetry(cleaned, params);

console.log(`Cleaned ${soundings.length - cleaned.length} outliers`);
console.log(`Generated grid with ${gridded.size} cells`);
```

## Summary

Bathymetric data processing transforms raw sonar measurements into accurate, high-quality seafloor maps through a series of corrections, filtering, and interpolation steps. Understanding this workflow is essential for producing reliable bathymetric products and correctly interpreting their accuracy and limitations.

This chapter examined data cleaning and artifact removal, sound velocity corrections, positioning and datum transformations, gridding and interpolation methods, and quality control procedures. We demonstrated these concepts through comprehensive TypeScript implementations.

The next chapter will explore 3D modeling and visualization techniques that bring bathymetric data to life, enabling intuitive understanding of seafloor topography.

**弘益人間 (Benefit All Humanity)** - Rigorous data processing ensures that bathymetric maps are accurate and reliable, supporting safe navigation, scientific research, and sustainable ocean management for all.

---

*WIA-OCEAN-005: Ocean Floor Mapping*
*© 2025 SmileStory Inc. / WIA - World Certification Industry Association*
*弘益人間 (홍익인간) - Benefit All Humanity*
