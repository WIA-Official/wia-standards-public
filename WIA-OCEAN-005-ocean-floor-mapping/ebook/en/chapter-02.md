# Chapter 2: Multibeam Sonar Systems

## 2.1 Principles of Multibeam Echo Sounding

Multibeam echo sounders (MBES) represent the workhorse technology for modern seafloor mapping. These sophisticated acoustic systems emit multiple sound beams in a fan-shaped pattern perpendicular to the vessel's direction of travel, enabling simultaneous depth measurements across a wide swath of seafloor. Unlike single-beam echo sounders that measure only directly beneath the vessel, multibeam systems can map swaths ranging from 3 to 7 times the water depth, dramatically improving survey efficiency.

The fundamental principle behind multibeam sonar is straightforward: measure the time required for an acoustic pulse to travel from the sonar transducer to the seafloor and back, then use the known speed of sound in seawater to calculate distance. However, the implementation of this principle in a multibeam system involves sophisticated engineering and signal processing to achieve the accuracy and resolution required for modern applications.

A typical multibeam system consists of two perpendicular arrays of transducers: one for transmitting and one for receiving. The transmit array creates a narrow beam along the ship's track and a wide beam perpendicular to the track. The receive array creates narrow beams perpendicular to the track, enabling precise angle determination for each return signal. By combining precise travel time measurements with accurate angle information, the system calculates the three-dimensional position of each seafloor point within the swath.

Modern MBES systems can produce hundreds of individual depth measurements per ping, with ping rates exceeding 50 Hz in shallow water. This high data rate enables dense seafloor coverage even at slow survey speeds. A single day's survey with a modern multibeam system can generate millions of individual depth soundings, creating a detailed picture of the seafloor topography.

## 2.2 System Architecture and Components

A complete multibeam sonar system consists of several integrated components, each playing a critical role in data acquisition:

**Sonar Transducers**: The transducer arrays convert electrical signals into acoustic waves for transmission and convert received acoustic waves back into electrical signals. Transducers are typically mounted in a hull-mounted dome or on a retractable keel to minimize flow noise and ensure optimal acoustic coupling with the water. Modern transducers use piezoelectric ceramics or composite materials capable of efficiently converting energy across the system's operating frequency range.

**Signal Processing Unit**: The sonar processor controls transmission timing, applies beamforming algorithms to separate individual beams, detects bottom echoes, and calculates preliminary depth estimates. Modern processors use field-programmable gate arrays (FPGAs) and digital signal processors (DSPs) to handle the enormous computational demands of real-time beamforming and bottom detection for hundreds of beams at high ping rates.

**Motion Reference Unit (MRU)**: Since the survey vessel is constantly moving in response to waves, accurate measurement of vessel motion is essential for correcting depth measurements. The MRU measures roll, pitch, heave, and heading with high precision using inertial sensors and GPS/GNSS receivers. Modern MRUs can measure heave with centimeter-level accuracy, crucial for maintaining bathymetric accuracy in rough seas.

**Sound Velocity Profiler (SVP)**: The speed of sound in seawater varies with temperature, salinity, and pressure. Accurate knowledge of the sound velocity profile throughout the water column is critical for correct depth calculation and beam steering. SVPs measure sound speed at different depths, typically using a probe lowered from the vessel or attached to an AUV.

**Positioning System**: Precise knowledge of the vessel's position is required to georeference bathymetric data. Modern surveys use Global Navigation Satellite Systems (GNSS) with Real-Time Kinematic (RTK) or Precise Point Positioning (PPP) techniques to achieve positioning accuracy at the decimeter level or better.

**Acquisition Software**: The acquisition system integrates data from all sensors, applies real-time corrections, displays data for operator review, and records data for post-processing. Modern acquisition systems provide sophisticated visualization tools enabling operators to assess data quality during the survey and make adjustments as needed.

## 2.3 Frequency Selection and Performance Tradeoffs

Multibeam systems operate across a wide range of acoustic frequencies, typically from 12 kHz to 400 kHz or higher. Frequency selection involves fundamental tradeoffs between range, resolution, and beam width:

**Low Frequency Systems (12-50 kHz)**: Lower frequencies propagate farther through water with less attenuation, making them suitable for deep-water mapping. A 12 kHz system can operate effectively in water depths exceeding 6,000 meters. However, lower frequencies have larger wavelengths, resulting in larger acoustic footprints and lower resolution. Deep-water systems typically achieve horizontal resolution of several meters to tens of meters.

**Medium Frequency Systems (70-200 kHz)**: These systems represent a compromise between range and resolution, suitable for continental shelf and slope mapping. They can operate effectively to depths of 3,000-4,000 meters while providing horizontal resolution of 1-5 meters depending on water depth and system configuration.

**High Frequency Systems (300-400 kHz and above)**: High-frequency systems offer superior resolution but limited range due to increased acoustic attenuation. These systems excel in shallow water applications, coastal surveys, and detailed inspections where sub-meter resolution is required. Some specialized systems operate at even higher frequencies (1 MHz or more) for very high-resolution imaging in shallow water.

The relationship between frequency and resolution follows from basic acoustic principles. The size of the acoustic footprint on the seafloor is determined by the beam width and range. Beam width, in turn, depends on the acoustic wavelength and the size of the transducer array. Higher frequencies (shorter wavelengths) enable narrower beams and smaller footprints, yielding higher resolution.

## 2.4 Swath Coverage and Survey Planning

One of the key advantages of multibeam systems is their ability to map a wide swath of seafloor in a single pass. The swath width depends on several factors:

**Maximum Usable Swath**: While multibeam systems can theoretically achieve very wide swaths, practical limitations constrain the usable swath width. Outer beams arrive at the seafloor at oblique angles, resulting in larger footprints and lower resolution. The signal-to-noise ratio decreases for outer beams due to increased path length and beam spreading. Most surveys limit the usable swath to maintain consistent data quality across the full width.

**Depth Dependency**: Swath width scales with water depth. In 100 meters of water, a system with a 140-degree swath angle can map approximately 350 meters of seafloor width. In 4,000 meters of water, the same system covers 14 kilometers. This scaling means that survey efficiency actually improves with increasing water depth (in terms of area covered per unit time), though this is partially offset by slower vessel speeds often necessary in deeper water.

**Line Spacing and Overlap**: Survey planners must determine appropriate line spacing to ensure complete coverage without excessive overlap. Typically, adjacent survey lines are planned with 10-20% overlap to ensure no gaps and provide redundant data for quality assessment. The overlap percentage may be increased in areas of complex topography or when very high data density is required.

**Survey Speed**: Vessel speed affects data density along the track. Faster speeds increase survey efficiency but reduce along-track data density. The optimal speed depends on system ping rate, desired data density, sea conditions, and vessel capabilities. Typical survey speeds range from 4-12 knots depending on these factors.

## 2.5 Data Quality and Uncertainty

Understanding and quantifying uncertainty is crucial for appropriate use of bathymetric data. Multiple factors contribute to the overall uncertainty budget:

**Measurement Uncertainty**: The fundamental precision of travel time measurement determines the theoretical vertical resolution. Modern systems measure travel times with nanosecond precision, enabling sub-centimeter range resolution. However, other factors typically dominate the overall uncertainty budget.

**Sound Velocity Uncertainty**: Errors in the assumed sound velocity profile directly translate to depth errors. A 1% error in sound velocity produces a 1% error in computed depth. In 4,000 meters of water, this could represent 40 meters of vertical error. Regular sound velocity profiles are essential for minimizing this error source.

**Motion Compensation**: Imperfect measurement or compensation of vessel motion introduces vertical and horizontal errors. Modern high-performance MRUs minimize this error source, but in very rough seas, motion-induced errors can still be significant, particularly for outer beams.

**Positioning Errors**: Horizontal positioning errors affect the geographic location of depth measurements. With modern GNSS positioning, horizontal errors are typically well below one meter, negligible compared to other error sources for most applications.

**Seafloor Characteristics**: The nature of the seafloor affects measurement quality. Hard, smooth surfaces produce strong, clear returns. Soft sediments absorb acoustic energy, weakening returns. Complex microtopography can produce confusing return signals. These factors affect bottom detection reliability and accuracy.

## 2.6 Advanced Capabilities

Modern multibeam systems offer capabilities beyond simple depth measurement:

**Backscatter Imaging**: In addition to measuring depth, multibeam systems record the intensity of the returned signal. Backscatter intensity depends on seafloor composition, roughness, and angle of incidence. Processing backscatter data produces acoustic images that reveal seafloor texture and composition, complementing bathymetric data.

**Water Column Imaging**: Some multibeam systems can record the full water column return, not just the seafloor echo. Water column data reveals fish schools, gas seeps, hydrothermal plumes, and other mid-water features. This capability transforms the multibeam system from a seafloor mapping tool into a comprehensive ocean observation instrument.

**Snippet Data**: Snippet data consists of time-series samples around each beam's bottom detection. This additional information enables more sophisticated bottom detection algorithms in post-processing and provides insight into seafloor scattering characteristics.

**Multiple Frequencies**: Some advanced systems can transmit at multiple frequencies simultaneously or sequentially, providing complementary data with different range-resolution tradeoffs. This capability is particularly useful for mapping complex seafloor environments.

## 2.7 TypeScript Implementation Example

Here's a TypeScript implementation demonstrating key multibeam data processing concepts:

```typescript
/**
 * WIA-OCEAN-005: Multibeam Sonar Data Processing
 * Ocean Blue Theme: #0EA5E9
 */

interface MultibeamPing {
  timestamp: Date;
  vesselPosition: Position;
  vesselAttitude: Attitude;
  beams: BeamData[];
  soundVelocityProfile: SoundVelocity[];
}

interface Position {
  latitude: number;
  longitude: number;
  altitude: number;
}

interface Attitude {
  roll: number;      // degrees
  pitch: number;     // degrees
  heading: number;   // degrees true
  heave: number;     // meters
}

interface BeamData {
  beamNumber: number;
  angle: number;           // degrees from nadir
  travelTime: number;      // seconds
  backscatter: number;     // dB
  quality: number;         // 0-100
}

interface SoundVelocity {
  depth: number;           // meters
  velocity: number;        // meters/second
}

interface SeafloorPoint {
  position: Position;
  depth: number;
  uncertainty: number;
  backscatter: number;
}

/**
 * Process a multibeam ping to calculate seafloor positions
 */
function processPing(ping: MultibeamPing): SeafloorPoint[] {
  const points: SeafloorPoint[] = [];

  for (const beam of ping.beams) {
    // Skip low-quality beams
    if (beam.quality < 50) continue;

    // Calculate slant range using ray tracing through sound velocity profile
    const slantRange = calculateRange(
      beam.travelTime,
      ping.soundVelocityProfile
    );

    // Apply motion compensation
    const correctedAngle = applyMotionCorrection(
      beam.angle,
      ping.vesselAttitude
    );

    // Calculate seafloor point in vessel reference frame
    const acrossTrack = slantRange * Math.sin(correctedAngle * Math.PI / 180);
    const depth = slantRange * Math.cos(correctedAngle * Math.PI / 180);

    // Transform to geographic coordinates
    const position = transformToGeographic(
      ping.vesselPosition,
      ping.vesselAttitude.heading,
      acrossTrack,
      0  // along-track offset
    );

    // Estimate uncertainty
    const uncertainty = estimateUncertainty(
      depth,
      beam.angle,
      ping.soundVelocityProfile
    );

    points.push({
      position: position,
      depth: ping.vesselPosition.altitude - depth,
      uncertainty: uncertainty,
      backscatter: beam.backscatter
    });
  }

  return points;
}

/**
 * Calculate range using ray tracing through sound velocity profile
 */
function calculateRange(
  travelTime: number,
  svp: SoundVelocity[]
): number {
  // Simplified calculation assuming constant sound velocity
  // In practice, ray tracing through the SVP is required
  const avgVelocity = svp.reduce((sum, sv) => sum + sv.velocity, 0) / svp.length;
  return (travelTime * avgVelocity) / 2;  // Divide by 2 for two-way travel
}

/**
 * Apply motion correction to beam angle
 */
function applyMotionCorrection(
  beamAngle: number,
  attitude: Attitude
): number {
  // Apply roll correction
  // In practice, full 3D rotation matrix is required
  return beamAngle + attitude.roll;
}

/**
 * Transform from vessel reference frame to geographic coordinates
 */
function transformToGeographic(
  vesselPos: Position,
  heading: number,
  acrossTrack: number,
  alongTrack: number
): Position {
  // Convert heading to radians
  const headingRad = heading * Math.PI / 180;

  // Calculate offsets in meters
  const eastOffset = acrossTrack * Math.cos(headingRad) +
                     alongTrack * Math.sin(headingRad);
  const northOffset = -acrossTrack * Math.sin(headingRad) +
                       alongTrack * Math.cos(headingRad);

  // Convert to lat/lon (simplified - use proper geodetic calculations in production)
  const metersPerDegreeLat = 111320;
  const metersPerDegreeLon = 111320 * Math.cos(vesselPos.latitude * Math.PI / 180);

  return {
    latitude: vesselPos.latitude + northOffset / metersPerDegreeLat,
    longitude: vesselPos.longitude + eastOffset / metersPerDegreeLon,
    altitude: vesselPos.altitude
  };
}

/**
 * Estimate depth uncertainty based on multiple factors
 */
function estimateUncertainty(
  depth: number,
  beamAngle: number,
  svp: SoundVelocity[]
): number {
  // Sound velocity uncertainty (typically 0.5%)
  const svUncertainty = depth * 0.005;

  // Beam angle uncertainty increases with angle
  const angleUncertainty = depth * Math.tan(1.0 * Math.PI / 180) *
                          (Math.abs(beamAngle) / 60);

  // Combine uncertainties (root sum square)
  return Math.sqrt(svUncertainty ** 2 + angleUncertainty ** 2);
}

/**
 * Grid bathymetric points using weighted averaging
 */
interface GridCell {
  depth: number;
  uncertainty: number;
  density: number;
}

function gridBathymetry(
  points: SeafloorPoint[],
  gridResolution: number
): Map<string, GridCell> {
  const grid = new Map<string, GridCell>();

  // Accumulate weighted points in each cell
  const accumulator = new Map<string, {
    weightedDepth: number;
    weightSum: number;
    count: number;
  }>();

  for (const point of points) {
    // Determine grid cell
    const cellX = Math.floor(point.position.longitude / gridResolution);
    const cellY = Math.floor(point.position.latitude / gridResolution);
    const cellKey = `${cellX},${cellY}`;

    // Weight by inverse uncertainty squared
    const weight = 1 / (point.uncertainty ** 2);

    const cell = accumulator.get(cellKey) || {
      weightedDepth: 0,
      weightSum: 0,
      count: 0
    };

    cell.weightedDepth += point.depth * weight;
    cell.weightSum += weight;
    cell.count += 1;

    accumulator.set(cellKey, cell);
  }

  // Calculate final grid values
  for (const [key, cell] of accumulator) {
    grid.set(key, {
      depth: cell.weightedDepth / cell.weightSum,
      uncertainty: Math.sqrt(1 / cell.weightSum),
      density: cell.count
    });
  }

  return grid;
}

// Example usage
const examplePing: MultibeamPing = {
  timestamp: new Date(),
  vesselPosition: {
    latitude: 35.5,
    longitude: 139.7,
    altitude: 0
  },
  vesselAttitude: {
    roll: 2.5,
    pitch: -1.2,
    heading: 045,
    heave: 0.8
  },
  beams: Array.from({ length: 256 }, (_, i) => ({
    beamNumber: i,
    angle: (i - 128) * 0.5,  // 0.5 degree spacing
    travelTime: 0.5 + Math.random() * 0.1,
    backscatter: -20 + Math.random() * 10,
    quality: 80 + Math.random() * 20
  })),
  soundVelocityProfile: [
    { depth: 0, velocity: 1500 },
    { depth: 100, velocity: 1490 },
    { depth: 500, velocity: 1485 }
  ]
};

// Process the ping
const seafloorPoints = processPing(examplePing);
console.log(`Processed ${seafloorPoints.length} seafloor points`);

// Grid the data
const grid = gridBathymetry(seafloorPoints, 0.0001); // ~10m resolution
console.log(`Generated grid with ${grid.size} cells`);
```

## Summary

Multibeam echo sounders are the primary technology for modern seafloor mapping, offering unprecedented efficiency and data quality. Understanding the principles, capabilities, and limitations of these systems is essential for planning effective surveys and interpreting bathymetric data correctly.

This chapter examined the physical principles underlying multibeam sonar, system architecture and components, frequency selection and performance tradeoffs, survey planning considerations, data quality and uncertainty estimation, and advanced capabilities including backscatter and water column imaging. We also explored practical implementation through TypeScript code examples.

The next chapter will examine side-scan sonar technology, which complements multibeam systems by providing high-resolution acoustic imagery of the seafloor.

**弘益人間 (Benefit All Humanity)** - Advanced multibeam sonar technology enables us to map the seafloor with unprecedented detail, supporting scientific discovery, sustainable resource management, and safe maritime operations for the benefit of all.

---

*WIA-OCEAN-005: Ocean Floor Mapping*
*© 2025 SmileStory Inc. / WIA - World Certification Industry Association*
*弘益人間 (홍익인간) - Benefit All Humanity*
