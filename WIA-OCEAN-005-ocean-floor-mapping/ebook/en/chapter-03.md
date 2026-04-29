# Chapter 3: Side-Scan Sonar Technology

## 3.1 Introduction to Side-Scan Sonar

While multibeam echo sounders excel at measuring seafloor depth, side-scan sonar specializes in creating detailed acoustic images of the seafloor surface. These complementary technologies work together to provide a comprehensive picture of the underwater environment—multibeam reveals the shape and depth, while side-scan reveals the texture, composition, and small-scale features.

Side-scan sonar systems emit acoustic pulses toward the seafloor at oblique angles, then record the intensity of the returned echoes. Hard, rough surfaces reflect more energy back to the receiver than soft, smooth surfaces. Shadows cast by raised features create distinctive patterns in the imagery. The result is an acoustic image resembling a black-and-white photograph, revealing details invisible to depth-measuring systems alone.

The history of side-scan sonar dates to the 1950s when early systems were developed for mine hunting and submarine detection. Over the decades, the technology evolved from simple towed systems to sophisticated platforms capable of producing imagery with resolution measured in centimeters. Modern side-scan sonar finds applications ranging from archaeological surveys to pipeline inspection, from habitat mapping to search and recovery operations.

## 3.2 Operating Principles

Side-scan sonar operates on relatively simple principles, though implementation requires sophisticated engineering:

**Acoustic Beam Pattern**: The sonar transducer emits a fan-shaped beam that is narrow in the along-track direction and wide in the vertical plane. This beam pattern creates a thin slice of insonification on the seafloor, extending outward from both sides of the towfish or vessel.

**Oblique Incidence**: Unlike multibeam systems that primarily measure near-nadir angles, side-scan sonar insonifies the seafloor at grazing angles, typically between 10 and 80 degrees from horizontal. This oblique geometry creates strong reflections from vertical or near-vertical surfaces while casting distinct shadows behind raised features.

**Time-Series Recording**: The system records the return signal as a continuous time series. Early returns come from the seafloor closest to the sensor (near range), while later returns come from more distant areas (far range). The relationship between time and range is straightforward: range equals half the round-trip travel time multiplied by sound speed.

**Imagery Generation**: Each transmitted pulse creates one scan line in the final image. As the platform moves forward, successive scan lines are assembled into a continuous image strip. The along-track resolution depends on the platform's speed and the ping rate, while across-track resolution depends primarily on the pulse length and beam width.

## 3.3 System Configurations

Side-scan sonar systems come in various configurations optimized for different applications:

**Towed Systems**: The most common configuration involves towing a sonar "fish" behind a vessel using an armored cable that provides both mechanical support and electrical connectivity. Towed systems offer several advantages: they can be deployed from relatively small vessels, positioned close to the seafloor for optimal imaging, and easily transported between survey sites. However, towing introduces challenges including fish motion, cable strumming, and limited positioning accuracy.

**Hull-Mounted Systems**: Some vessels integrate side-scan transducers directly into the hull. Hull-mounted systems benefit from precise positioning and integration with other sensors, but they operate farther from the seafloor, limiting resolution and maximum range. They are particularly suitable for rapid reconnaissance surveys in deeper water.

**AUV-Mounted Systems**: Autonomous underwater vehicles carrying side-scan sonars combine the positioning advantages of hull-mounted systems with the close-to-bottom operation of towed fish. AUVs can maintain consistent altitude above the seafloor, producing uniform imagery even over variable topography. The autonomous operation enables surveys in hazardous environments or beneath ice.

**ROV-Mounted Systems**: Remotely operated vehicles offer the ultimate flexibility, allowing operators to position the sonar optimally for specific targets. ROV-mounted systems excel at detailed inspections of small areas, though they are less efficient for large-area surveys.

## 3.4 Frequency Selection and Resolution

Like multibeam systems, side-scan sonars operate across a range of frequencies, each offering different tradeoffs:

**Low Frequency (50-100 kHz)**: These systems achieve ranges exceeding 1,000 meters per side, making them suitable for reconnaissance surveys and search operations over large areas. Resolution typically ranges from 0.5 to 2 meters. Low-frequency systems are relatively insensitive to water clarity and can penetrate some distance into soft sediments.

**Medium Frequency (300-500 kHz)**: Medium-frequency systems balance range and resolution, typically operating at ranges of 100-400 meters per side with resolution of 10-50 cm. These systems are popular for general-purpose seafloor mapping, habitat assessment, and infrastructure inspection.

**High Frequency (900 kHz - 1.8 MHz)**: High-frequency systems provide exceptional resolution, often better than 5 cm, enabling detection of very small objects and fine-scale seafloor features. However, range is limited to 50-150 meters per side, and these systems are sensitive to water clarity and suspended sediments.

**Synthetic Aperture Sonar (SAS)**: SAS systems use sophisticated signal processing to achieve resolution independent of range, overcoming a fundamental limitation of conventional side-scan sonar. SAS can provide centimeter-scale resolution at ranges of hundreds of meters, though at the cost of increased complexity and data processing requirements.

## 3.5 Image Interpretation

Interpreting side-scan sonar imagery requires understanding how different seafloor characteristics affect acoustic returns:

**Acoustic Backscatter**: Hard surfaces like rock, coral, or metal reflect acoustic energy strongly, appearing bright in side-scan images. Soft sediments absorb energy, appearing dark. This relationship enables interpretation of seafloor composition and sediment distribution.

**Shadows**: Objects protruding above the seafloor cast acoustic shadows—areas that receive no direct insonification and therefore appear dark. Shadow length depends on the height of the feature and the grazing angle. Measuring shadow length enables estimation of feature height, a critical capability for identifying and characterizing objects.

**Texture**: Surface roughness at scales comparable to the acoustic wavelength affects backscatter strength and creates distinctive textural patterns. Rippled sand, cobble fields, and bedrock outcrops each produce characteristic textures recognizable to experienced interpreters.

**Linear Features**: Side-scan excels at detecting linear features such as pipelines, cables, geological lineaments, and archaeological structures. The oblique geometry and high resolution make these features stand out clearly against the background seafloor.

## 3.6 Data Processing and Mosaicking

Raw side-scan sonar data requires significant processing to produce high-quality imagery:

**Slant Range Correction**: Raw side-scan data is recorded in slant range (the direct distance from sonar to seafloor). Converting to ground range (horizontal distance) requires knowledge of the sensor altitude and seafloor slope. This correction ensures features appear in their correct geometric positions.

**Radiometric Corrections**: Several factors affect backscatter intensity beyond seafloor properties. These include beam pattern effects, time-varying gain settings, and absorption losses. Radiometric corrections normalize the imagery, enabling comparison of backscatter strengths across the image.

**Geometric Corrections**: Variations in platform motion, sound velocity, and altitude must be corrected to produce geometrically accurate imagery. Modern processing software uses navigation and attitude data to dewarp the imagery, correcting for these variations.

**Mosaicking**: Individual side-scan tracks are assembled into a continuous mosaic covering the survey area. This process involves selecting appropriate portions of overlapping data, balancing brightness between adjacent tracks, and blending seams to create seamless imagery.

## 3.7 TypeScript Implementation Example

Here's a TypeScript implementation for side-scan sonar data processing:

```typescript
/**
 * WIA-OCEAN-005: Side-Scan Sonar Data Processing
 * Ocean Blue Theme: #0EA5E9
 */

interface SideScanPing {
  timestamp: Date;
  position: Position;
  altitude: number;
  heading: number;
  portData: number[];    // Port side return amplitudes
  starboardData: number[];  // Starboard side return amplitudes
  sampleRate: number;    // Samples per second
  soundVelocity: number; // m/s
}

interface SideScanImage {
  width: number;
  height: number;
  pixels: Uint8Array;
  resolution: number;    // meters per pixel
  bounds: GeographicBounds;
}

interface GeographicBounds {
  north: number;
  south: number;
  east: number;
  west: number;
}

/**
 * Convert slant range to ground range
 */
function slantToGroundRange(
  slantRange: number,
  altitude: number
): number {
  // Using Pythagorean theorem
  if (slantRange <= altitude) {
    return 0;  // Directly beneath sensor
  }
  return Math.sqrt(slantRange * slantRange - altitude * altitude);
}

/**
 * Process side-scan data with geometric corrections
 */
function processSideScanPing(ping: SideScanPing): ProcessedScanLine {
  const samplesPerSide = ping.portData.length;
  const maxSlantRange = (samplesPerSide / ping.sampleRate) *
                        (ping.soundVelocity / 2);

  // Process port and starboard separately
  const portGroundRange: number[] = [];
  const starboardGroundRange: number[] = [];
  const portAmplitudes: number[] = [];
  const starboardAmplitudes: number[] = [];

  for (let i = 0; i < samplesPerSide; i++) {
    const slantRange = (i / ping.sampleRate) * (ping.soundVelocity / 2);
    const groundRange = slantToGroundRange(slantRange, ping.altitude);

    // Apply time-varying gain (TVG) correction
    const tvgCorrection = 20 * Math.log10(slantRange + 1);

    portGroundRange.push(groundRange);
    starboardGroundRange.push(groundRange);

    // Apply TVG and convert to dB scale
    portAmplitudes.push(ping.portData[i] + tvgCorrection);
    starboardAmplitudes.push(ping.starboardData[i] + tvgCorrection);
  }

  return {
    timestamp: ping.timestamp,
    position: ping.position,
    heading: ping.heading,
    portGroundRange,
    starboardGroundRange,
    portAmplitudes,
    starboardAmplitudes
  };
}

interface ProcessedScanLine {
  timestamp: Date;
  position: Position;
  heading: number;
  portGroundRange: number[];
  starboardGroundRange: number[];
  portAmplitudes: number[];
  starboardAmplitudes: number[];
}

/**
 * Detect objects and features in side-scan imagery
 */
interface DetectedFeature {
  position: Position;
  length: number;
  width: number;
  height: number;
  shadowLength: number;
  confidence: number;
  type: 'object' | 'linear' | 'geological';
}

function detectFeatures(
  scanLines: ProcessedScanLine[],
  threshold: number
): DetectedFeature[] {
  const features: DetectedFeature[] = [];

  for (let i = 0; i < scanLines.length; i++) {
    const line = scanLines[i];

    // Look for high-backscatter targets followed by shadows
    for (let side of ['port', 'starboard']) {
      const amplitudes = side === 'port' ?
                        line.portAmplitudes :
                        line.starboardAmplitudes;
      const ranges = side === 'port' ?
                     line.portGroundRange :
                     line.starboardGroundRange;

      let inHighlight = false;
      let highlightStart = 0;
      let shadowStart = 0;

      for (let j = 0; j < amplitudes.length - 1; j++) {
        // Detect transition from high to low amplitude (object with shadow)
        if (!inHighlight && amplitudes[j] > threshold) {
          inHighlight = true;
          highlightStart = j;
        } else if (inHighlight && amplitudes[j] < threshold * 0.5) {
          shadowStart = j;
          const shadowEnd = findShadowEnd(amplitudes, j, threshold);

          if (shadowEnd - shadowStart > 3) {  // Minimum shadow length
            const feature = createFeature(
              line,
              ranges,
              highlightStart,
              shadowStart,
              shadowEnd,
              side === 'port' ? -1 : 1
            );
            features.push(feature);
          }

          inHighlight = false;
        }
      }
    }
  }

  return features;
}

function findShadowEnd(
  amplitudes: number[],
  start: number,
  threshold: number
): number {
  for (let i = start; i < amplitudes.length; i++) {
    if (amplitudes[i] > threshold * 0.7) {
      return i;
    }
  }
  return amplitudes.length - 1;
}

function createFeature(
  line: ProcessedScanLine,
  ranges: number[],
  highlightStart: number,
  shadowStart: number,
  shadowEnd: number,
  side: number  // -1 for port, 1 for starboard
): DetectedFeature {
  const highlightLength = ranges[shadowStart] - ranges[highlightStart];
  const shadowLength = ranges[shadowEnd] - ranges[shadowStart];

  // Estimate height from shadow length
  // Assuming 45-degree grazing angle for simplification
  const estimatedHeight = shadowLength * 0.7;

  // Calculate geographic position
  const rangeToTarget = (ranges[highlightStart] + ranges[shadowStart]) / 2;
  const headingRad = line.heading * Math.PI / 180;
  const crossTrackAngle = headingRad + side * Math.PI / 2;

  const eastOffset = rangeToTarget * Math.cos(crossTrackAngle);
  const northOffset = rangeToTarget * Math.sin(crossTrackAngle);

  const metersPerDegreeLat = 111320;
  const metersPerDegreeLon = 111320 *
                             Math.cos(line.position.latitude * Math.PI / 180);

  const featurePosition: Position = {
    latitude: line.position.latitude + northOffset / metersPerDegreeLat,
    longitude: line.position.longitude + eastOffset / metersPerDegreeLon,
    altitude: line.position.altitude
  };

  return {
    position: featurePosition,
    length: highlightLength,
    width: 1.0,  // Would need multiple scan lines to determine
    height: estimatedHeight,
    shadowLength: shadowLength,
    confidence: 0.8,
    type: 'object'
  };
}

/**
 * Create georeferenced mosaic from processed scan lines
 */
function createMosaic(
  scanLines: ProcessedScanLine[],
  resolution: number  // meters per pixel
): SideScanImage {
  // Determine geographic bounds
  let minLat = Infinity, maxLat = -Infinity;
  let minLon = Infinity, maxLon = -Infinity;
  let maxRange = 0;

  for (const line of scanLines) {
    minLat = Math.min(minLat, line.position.latitude);
    maxLat = Math.max(maxLat, line.position.latitude);
    minLon = Math.min(minLon, line.position.longitude);
    maxLon = Math.max(maxLon, line.position.longitude);

    maxRange = Math.max(
      maxRange,
      Math.max(...line.portGroundRange),
      Math.max(...line.starboardGroundRange)
    );
  }

  // Add buffer for sonar range
  const metersPerDegreeLat = 111320;
  const metersPerDegreeLon = 111320 * Math.cos((minLat + maxLat) / 2 * Math.PI / 180);

  minLat -= maxRange / metersPerDegreeLat;
  maxLat += maxRange / metersPerDegreeLat;
  minLon -= maxRange / metersPerDegreeLon;
  maxLon += maxRange / metersPerDegreeLon;

  // Create image grid
  const width = Math.ceil((maxLon - minLon) * metersPerDegreeLon / resolution);
  const height = Math.ceil((maxLat - minLat) * metersPerDegreeLat / resolution);
  const pixels = new Uint8Array(width * height).fill(128);  // Gray background
  const counts = new Uint16Array(width * height).fill(0);   // For averaging

  // Rasterize scan lines
  for (const line of scanLines) {
    rasterizeScanLine(
      line,
      pixels,
      counts,
      width,
      height,
      minLon,
      minLat,
      resolution,
      metersPerDegreeLon,
      metersPerDegreeLat
    );
  }

  // Average overlapping pixels
  for (let i = 0; i < pixels.length; i++) {
    if (counts[i] > 1) {
      pixels[i] = Math.floor(pixels[i] / counts[i]);
    }
  }

  return {
    width,
    height,
    pixels,
    resolution,
    bounds: {
      north: maxLat,
      south: minLat,
      east: maxLon,
      west: minLon
    }
  };
}

function rasterizeScanLine(
  line: ProcessedScanLine,
  pixels: Uint8Array,
  counts: Uint16Array,
  width: number,
  height: number,
  minLon: number,
  minLat: number,
  resolution: number,
  metersPerDegreeLon: number,
  metersPerDegreeLat: number
): void {
  const headingRad = line.heading * Math.PI / 180;

  // Process both port and starboard
  for (const side of ['port', 'starboard']) {
    const amplitudes = side === 'port' ?
                      line.portAmplitudes :
                      line.starboardAmplitudes;
    const ranges = side === 'port' ?
                   line.portGroundRange :
                   line.starboardGroundRange;
    const sideMultiplier = side === 'port' ? -1 : 1;

    for (let i = 0; i < amplitudes.length; i++) {
      const range = ranges[i];
      const crossTrackAngle = headingRad + sideMultiplier * Math.PI / 2;

      const eastOffset = range * Math.cos(crossTrackAngle);
      const northOffset = range * Math.sin(crossTrackAngle);

      const lon = line.position.longitude + eastOffset / metersPerDegreeLon;
      const lat = line.position.latitude + northOffset / metersPerDegreeLat;

      const x = Math.floor((lon - minLon) * metersPerDegreeLon / resolution);
      const y = Math.floor((lat - minLat) * metersPerDegreeLat / resolution);

      if (x >= 0 && x < width && y >= 0 && y < height) {
        const idx = y * width + x;

        // Convert amplitude to grayscale (0-255)
        const normalizedAmplitude = Math.max(0, Math.min(255,
          Math.floor((amplitudes[i] + 60) * 255 / 60)
        ));

        pixels[idx] += normalizedAmplitude;
        counts[idx] += 1;
      }
    }
  }
}

// Example usage
const examplePing: SideScanPing = {
  timestamp: new Date(),
  position: { latitude: 35.5, longitude: 139.7, altitude: 0 },
  altitude: 50,
  heading: 90,
  portData: Array.from({ length: 1000 }, () => Math.random() * 100 - 60),
  starboardData: Array.from({ length: 1000 }, () => Math.random() * 100 - 60),
  sampleRate: 50000,
  soundVelocity: 1500
};

const processed = processSideScanPing(examplePing);
console.log('Processed side-scan ping');
```

## Summary

Side-scan sonar technology provides high-resolution acoustic imagery of the seafloor, complementing depth measurements from multibeam systems. Understanding the operating principles, system configurations, frequency selection, image interpretation techniques, and data processing workflows is essential for effective use of this powerful technology.

This chapter explored how side-scan sonar creates acoustic images through oblique insonification, examined various system configurations and their applications, discussed frequency selection and resolution tradeoffs, and demonstrated data processing through TypeScript code examples.

The next chapter will examine bathymetric data processing techniques that transform raw measurements into accurate, high-quality seafloor maps.

**弘익人間 (Benefit All Humanity)** - Side-scan sonar technology reveals the hidden details of the seafloor, enabling archaeological discoveries, environmental assessments, and maritime safety improvements that benefit all of humanity.

---

*WIA-OCEAN-005: Ocean Floor Mapping*
*© 2025 SmileStory Inc. / WIA - World Certification Industry Association*
*弘益人間 (홍익인간) - Benefit All Humanity*
