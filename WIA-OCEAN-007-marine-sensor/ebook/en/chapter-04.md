# Chapter 4: Biological Sensors (DNA, Plankton, Fish)

## Detecting Life in the Ocean

Biological sensors detect organisms, quantify biodiversity, and monitor ecosystem health. From microscopic bacteria to massive whales, from DNA fragments to whole organisms, biological sensors reveal the ocean's living fabric. These technologies enable biodiversity surveys, fisheries assessment, harmful algal bloom detection, invasive species monitoring, and marine protected area enforcement.

### Environmental DNA (eDNA) Sensors

Every organism sheds DNA into its environment through skin cells, mucus, feces, gametes, and decomposition. **Environmental DNA (eDNA)** detection identifies species from water samples without observing the organisms themselves.

#### eDNA Principles and Applications

**Sources of eDNA:**
- Fish: mucus, scales, feces, gametes
- Marine mammals: skin cells, feces
- Plankton: cell lysis, grazing
- Bacteria: constant turnover
- Viruses: released from infected cells

**eDNA Degradation:**
- Half-life in seawater: hours to weeks
- UV light degrades DNA
- Nucleases enzymatically degrade DNA
- Longer persistence in cold, dark, deep water

```typescript
interface eDNASensor {
  type: "qPCR" | "metabarcoding" | "ESP" | "nanopore";
  sensorId: string;

  // Sample collection
  sampling: {
    volume: number;              // liters
    filtrationPoreSize: number;  // μm, typically 0.22-5 μm
    preservationMethod: "ethanol" | "Longmire" | "frozen" | "immediate_extraction";
    replicates: number;
  };

  // DNA extraction
  extraction: {
    method: "manual" | "automated";
    kit: string;
    yield: number;               // ng DNA
    quality: {
      A260_280: number;          // Absorbance ratio, purity indicator
      concentration: number;     // ng/μL
    };
  };

  // Detection method
  detection: {
    method: "qPCR" | "ddPCR" | "HTS" | "Oxford_Nanopore";
    target: "species_specific" | "metabarcoding" | "metagenomics";
    markerGene?: string;         // COI, 16S rRNA, 18S rRNA, 12S
  };
}
```

#### Species-Specific eDNA Detection (qPCR)

**Quantitative PCR (qPCR)** detects and quantifies target species DNA:

**Process:**
1. Design species-specific primers and probe
2. Extract DNA from water sample
3. Amplify target sequence with PCR
4. Detect fluorescence as DNA amplifies
5. Quantify based on cycle threshold (Ct)

**Applications:**
- Rare/endangered species surveys
- Invasive species early detection
- Harmful algal bloom monitoring
- Fish stock assessment

```typescript
interface qPCRAssay {
  targetSpecies: string;
  markerGene: string;            // e.g., "cytochrome oxidase I (COI)"

  // Primer design
  primers: {
    forward: string;             // DNA sequence 5' to 3'
    reverse: string;
    probeSequence: string;       // TaqMan probe
    amplicon Length: number;     // bp, typically 80-150
    specificity: {
      testedSpecies: string[];
      crossReactivity: string[]; // Species that also amplify
    };
  };

  // qPCR conditions
  qPCRProtocol: {
    polymerase: string;
    annealingTemp: number;       // °C
    cycles: number;              // typically 40
    reactionVolume: number;      // μL
  };

  // Calibration
  standardCurve: {
    standards: {
      copies: number;            // DNA copies/μL
      Ct: number;                // Cycle threshold
    }[];
    efficiency: number;          // %, ideal 90-110%
    r2: number;                  // Standard curve fit
    LOD: number;                 // Limit of detection, copies/μL
    LOQ: number;                 // Limit of quantification
  };
}

interface qPCRResult {
  sampleId: string;
  targetSpecies: string;

  // Raw data
  Ct: number | "undetermined";   // Cycle threshold
  fluorescence: number[];        // By cycle

  // Quantification
  copiesPerReaction: number;     // DNA copies in qPCR reaction
  copiesPerLiter: number;        // DNA copies in original water sample

  // Interpretation
  detection: "positive" | "negative" | "uncertain";
  concentration: "high" | "medium" | "low" | "trace";

  // Quality control
  controls: {
    positiveControl: "pass" | "fail";
    negativeControl: "pass" | "fail";
    inhibition: boolean;         // PCR inhibitors present
  };

  qualityFlag: QualityFlag;
}

function quantifyeDNA_qPCR(
  Ct: number,
  standardCurve: { slope: number; intercept: number },
  sampleVolume: number,         // liters
  extractionVolume: number,     // μL DNA extract
  qPCR_volume: number           // μL DNA template in qPCR
): qPCRResult {
  if (Ct === null || Ct > 40) {
    return {
      sampleId: "sample001",
      targetSpecies: "Thunnus thynnus",
      Ct: "undetermined",
      fluorescence: [],
      copiesPerReaction: 0,
      copiesPerLiter: 0,
      detection: "negative",
      concentration: "trace",
      controls: { positiveControl: "pass", negativeControl: "pass", inhibition: false },
      qualityFlag: { status: "good", automated: true, tests: [] }
    };
  }

  // Calculate copies from Ct using standard curve
  // Ct = slope * log10(copies) + intercept
  const log10Copies = (Ct - standardCurve.intercept) / standardCurve.slope;
  const copiesPerReaction = Math.pow(10, log10Copies);

  // Scale to original water volume
  const copiesPerLiter = copiesPerReaction *
                        (extractionVolume / qPCR_volume) /
                        sampleVolume;

  const detection = copiesPerLiter > 10 ? "positive" :
                   copiesPerLiter > 1 ? "uncertain" : "negative";

  const concentration = copiesPerLiter > 1000 ? "high" :
                       copiesPerLiter > 100 ? "medium" :
                       copiesPerLiter > 10 ? "low" : "trace";

  return {
    sampleId: "sample001",
    targetSpecies: "Thunnus thynnus",
    Ct,
    fluorescence: [],
    copiesPerReaction,
    copiesPerLiter,
    detection,
    concentration,
    controls: { positiveControl: "pass", negativeControl: "pass", inhibition: false },
    qualityFlag: { status: "good", automated: true, tests: [] }
  };
}
```

#### eDNA Metabarcoding

**Metabarcoding** identifies all species in a sample using high-throughput sequencing:

**Process:**
1. Extract DNA from water sample
2. PCR-amplify universal barcode gene (e.g., COI, 16S)
3. High-throughput sequencing (Illumina)
4. Bioinformatic analysis: quality filter, denoise, assign taxonomy
5. Generate species list and relative abundances

**Advantages:**
- Detect hundreds of species per sample
- Reveal unexpected/cryptic species
- Community composition analysis

**Challenges:**
- Primer bias (different amplification efficiency)
- Incomplete reference databases
- DNA from dead organisms
- Quantification uncertain

```typescript
interface MetabarcodingAnalysis {
  sampleId: string;
  location: GeographicPosition;
  date: Date;

  // Sequencing data
  sequencing: {
    platform: "Illumina_MiSeq" | "Illumina_NovaSeq" | "Oxford_Nanopore";
    markerGene: "COI" | "16S_rRNA" | "18S_rRNA" | "12S_mtDNA";
    primerSet: string;
    totalReads: number;
    qualityFiltered: number;
  };

  // Bioinformatic processing
  processing: {
    pipeline: "QIIME2" | "mothur" | "DADA2" | "OBITools";
    clusteringMethod: "OTU_97" | "ASV" | "ZOTU";
    taxonomyDatabase: "NCBI" | "BOLD" | "SILVA" | "PR2";
    taxonomyMethod: "BLAST" | "RDP" | "SINTAX";
  };

  // Results
  species: {
    scientificName: string;
    taxonomyId: string;          // NCBI taxonomy ID
    confidence: number;          // 0-1, assignment confidence
    reads: number;               // Number of sequence reads
    relativeAbundance: number;   // Percentage of total reads
    detectionStatus: "certain" | "probable" | "uncertain";
  }[];

  // Diversity metrics
  diversity: {
    speciesRichness: number;     // Number of species (OTUs/ASVs)
    Shannon: number;             // Shannon diversity index
    Simpson: number;             // Simpson diversity index
    evenness: number;            // Pielou's evenness
  };

  // Quality
  controls: {
    positiveControl: string[];   // Expected species
    negativeControl: string[];   // Contaminants
    mockCommunity?: {
      expected: string[];
      detected: string[];
      accuracy: number;          // % correctly identified
    };
  };

  qualityFlag: QualityFlag;
}
```

#### Environmental Sample Processors (ESP)

**ESP** performs autonomous in-situ DNA analysis:

**Capabilities:**
- Collect water samples on schedule or trigger
- Filter particles, extract DNA
- Perform qPCR or sandwich hybridization assays
- Transmit results via satellite
- Deploy for weeks to months

**Applications:**
- Harmful algal bloom early warning
- Pathogen detection
- Invasive species monitoring
- Marine mammal eDNA surveys

```typescript
interface EnvironmentalSampleProcessor {
  deviceId: string;
  location: GeographicPosition;
  depth: number;                 // meters

  // Sample collection
  sampler: {
    filterType: string;
    sampleVolume: number;        // liters per sample
    sampleCapacity: number;      // Total samples before servicing
    schedule: "fixed_interval" | "event_triggered";
    interval?: number;           // hours
    trigger?: {
      parameter: string;         // e.g., "chlorophyll"
      threshold: number;
    };
  };

  // DNA processing
  processor: {
    extractionMethod: string;
    assays: qPCRAssay[];         // Configured assays
    maxAssaysPerSample: number;
    processingTime: number;      // minutes per sample
  };

  // Communication
  telemetry: {
    system: "Iridium" | "acoustic" | "cable";
    transmitSchedule: string;
    lastTransmission: Date;
  };

  // Power
  power: {
    batteryCapacity: number;     // Wh
    solarPanel?: boolean;
    estimatedEndurance: number;  // days
  };

  // Status
  status: {
    samplesProcessed: number;
    samplesRemaining: number;
    reagentsRemaining: number;   // %
    faults: string[];
  };
}
```

### Plankton Sensors

Plankton (phytoplankton and zooplankton) form the ocean food web base. Monitoring plankton abundance, size distribution, and species composition reveals ecosystem health and productivity.

#### Fluorometers (Chlorophyll Sensors)

**Chlorophyll fluorescence** indicates phytoplankton biomass:

**Principle:** Excite chlorophyll with blue light (470 nm); measure red fluorescence (685 nm); fluorescence proportional to chlorophyll concentration

**Advantages:**
- Real-time measurement
- No water sample needed
- Deployable on all platforms
- Low power consumption

**Limitations:**
- Proxy measurement (not chlorophyll concentration)
- Affected by non-photochemical quenching
- Species-specific fluorescence yield
- Requires calibration to extracted chlorophyll

```typescript
interface FluorometerSensor {
  type: "chlorophyll_fluorometer";
  serialNumber: string;

  // Optical configuration
  optics: {
    excitationWavelength: number;    // nm, typically 470
    emissionWavelength: number;      // nm, typically 685
    excitationBandwidth: number;     // nm
    emissionBandwidth: number;       // nm
  };

  // Calibration
  calibration: {
    date: Date;
    method: "manufacturer" | "field_extraction";
    darkCounts: number;          // Background signal
    scaleFactor: number;         // Converts counts to μg/L
    chlorophyllStandard?: {
      concentration: number;     // μg/L
      fluorescence: number;      // counts
    }[];
  };

  // Environmental corrections
  corrections: {
    temperatureCompensation: boolean;
    NPQ_correction: boolean;     // Non-photochemical quenching
  };

  // Performance
  range: { min: number; max: number }; // μg/L chlorophyll
  resolution: number;            // μg/L
  sensitivity: number;           // counts per μg/L
}

function calculateChlorophyll(
  sensor: FluorometerSensor,
  counts: number,
  temperature: number,
  depth: number
): number {
  // Subtract dark counts
  const signal = counts - sensor.calibration.darkCounts;

  // Apply scale factor
  let chlorophyll = signal * sensor.calibration.scaleFactor;

  // Non-photochemical quenching correction for daytime surface measurements
  if (depth < 50 && isDaytime()) {
    // NPQ reduces fluorescence in high light
    // Correction increases measured chlorophyll
    const NPQ_factor = 2.0;  // Empirical, depth and light dependent
    chlorophyll *= NPQ_factor;
  }

  return Math.max(0, chlorophyll);
}

function isDaytime(): boolean {
  // Simplified day/night detection
  const hour = new Date().getUTCHours();
  return hour >= 6 && hour < 18;
}
```

#### Optical Plankton Counters and Imagers

**Imaging systems** photograph plankton for identification and counting:

**Technologies:**
- **FlowCam:** Pumps water through flow cell, photographs particles
- **ISIIS:** In-Situ Ichthyoplankton Imaging System, towed camera
- **UVP:** Underwater Vision Profiler, descending camera
- **IFCB:** Imaging FlowCytobot, autonomous flow cytometer with imaging

```typescript
interface PlanktonImagingSystem {
  type: "FlowCam" | "ISIIS" | "UVP" | "IFCB";
  serialNumber: string;

  // Optical system
  imaging: {
    magnification: number;       // e.g., 4x, 10x, 20x
    resolution: number;          // μm per pixel
    depthOfField: number;        // μm
    frameRate: number;           // Hz
    pixelArray: [number, number]; // [width, height] pixels
  };

  // Sample volume
  sampling: {
    flowRate?: number;           // mL/min for flow systems
    imageVolume: number;         // mL per image
    imagesPerSecond: number;
  };

  // Detection capabilities
  detection: {
    minSize: number;             // μm
    maxSize: number;             // μm
    sizeClasses: number[];       // μm, size bin edges
  };

  // Image analysis
  analysis: {
    method: "manual" | "semi-automated" | "AI_classifier";
    features: string[];          // Area, perimeter, aspect ratio, etc.
    classifier?: {
      model: "CNN" | "Random_Forest" | "SVM";
      trainingSet: string;
      accuracy: number;          // %
      classes: string[];         // Taxonomic categories
    };
  };
}

interface PlanktonImage {
  imageId: string;
  timestamp: Date;
  location: GeographicPosition;
  depth: number;

  // Image properties
  image: {
    width: number;               // pixels
    height: number;              // pixels
    fileFormat: "PNG" | "TIFF" | "JPEG";
    filepath: string;
  };

  // Particle measurements
  particle: {
    area: number;                // μm²
    perimeter: number;           // μm
    length: number;              // μm, major axis
    width: number;               // μm, minor axis
    aspectRatio: number;
    biovolume: number;           // μm³
    transparency: number;        // 0-1
    circularity: number;         // 0-1
  };

  // Classification
  classification: {
    category: string;            // Copepod, diatom, dinoflagellate, etc.
    species?: string;
    confidence: number;          // 0-1
    manualVerification: boolean;
  };

  qualityFlag: QualityFlag;
}

function classifyPlanktonImage_AI(
  image: PlanktonImage,
  model: ConvolutionalNeuralNetwork
): { category: string; confidence: number } {
  // Extract image features
  const features = extractImageFeatures(image);

  // Run through trained CNN
  const predictions = model.predict(features);

  // Get top prediction
  const topPrediction = predictions.reduce((max, pred) =>
    pred.confidence > max.confidence ? pred : max
  );

  return {
    category: topPrediction.class,
    confidence: topPrediction.confidence
  };
}

// Placeholder for CNN model
interface ConvolutionalNeuralNetwork {
  predict(features: any): { class: string; confidence: number }[];
}

function extractImageFeatures(image: PlanktonImage): any {
  // Extract relevant features for classification
  return {
    area: image.particle.area,
    aspectRatio: image.particle.aspectRatio,
    circularity: image.particle.circularity,
    transparency: image.particle.transparency,
    // Plus: texture, shape descriptors, etc.
  };
}
```

### Fish Detection Sensors

Fish abundance and distribution are monitored using acoustic and optical methods:

#### Acoustic Fish Finders (Echo Sounders)

**Scientific echo sounders** detect fish by transmitting sound pulses and measuring reflected echoes:

**Frequencies:**
- **38 kHz:** Large fish, deep water, long range
- **120 kHz:** Medium fish, better resolution
- **200 kHz:** Small fish, high resolution, shorter range

```typescript
interface ScientificEchoSounder {
  frequency: number;             // kHz
  serialNumber: string;

  // Acoustic parameters
  transducer: {
    beamWidth: number;           // degrees, -3dB beam angle
    power: number;               // watts
    pulseLength: number;         // milliseconds
    pingRate: number;            // pings per second
  };

  // Data collection
  measurement: {
    rangeResolution: number;     // meters
    maximumRange: number;        // meters
    samplingInterval: number;    // seconds between pings
  };

  // Calibration
  calibration: {
    date: Date;
    method: "standard_sphere";   // Copper or tungsten carbide sphere
    targetStrength: number;      // dB re 1 m²
    receiverSensitivity: number; // dB
  };
}

interface AcousticBackscatter {
  timestamp: Date;
  location: GeographicPosition;

  // Backscatter data
  depth: number[];               // meters, range bins
  Sv: number[];                  // dB re 1 m⁻¹, volume backscattering strength
  Sp: number[];                  // dB re 1 m², point backscatter (individual fish)

  // Interpretation
  layers: {
    depthMin: number;            // meters
    depthMax: number;            // meters
    meanSv: number;              // dB
    biomass: number;             // kg/m²
    fishType: "small_pelagic" | "large_pelagic" | "demersal" | "unknown";
  }[];

  // Environmental
  soundSpeed: number;            // m/s
  absorption: number;            // dB/m
}

function estimateFishBiomass(
  Sv: number,                    // Volume backscattering strength, dB
  targetStrength: number,        // Individual fish target strength, dB
  fishWeight: number             // Average fish weight, kg
): number {
  // Convert Sv to linear scale
  const sv_linear = Math.pow(10, Sv / 10);

  // Convert TS to linear scale
  const sigma_bs = Math.pow(10, targetStrength / 10);

  // Estimate fish density (fish/m³)
  const density = sv_linear / sigma_bs;

  // Convert to biomass (kg/m³)
  const biomass = density * fishWeight;

  return biomass;
}
```

#### Underwater Camera Systems

Cameras provide direct visual observation:

**Types:**
- **Stationary cameras:** Mounted on moorings, observe passing fish
- **Towed cameras:** Survey large areas
- **ROV/AUV cameras:** Targeted surveys
- **Stereo cameras:** Measure fish length

```typescript
interface UnderwaterCameraSystem {
  type: "moored" | "towed" | "ROV" | "AUV" | "lander";
  serialNumber: string;

  // Camera specifications
  camera: {
    resolution: [number, number]; // [width, height] pixels
    frameRate: number;           // fps
    fieldOfView: number;         // degrees
    minimumIllumination: number; // lux
    recordingFormat: "H.264" | "H.265" | "ProRes";
  };

  // Illumination
  lighting: {
    type: "LED" | "strobe" | "laser";
    wavelength: number[];        // nm, red light less disturbing to fish
    intensity: number;           // lumens
    batteryLife: number;         // hours
  };

  // Stereo calibration (if stereo system)
  stereo?: {
    baseline: number;            // cm, distance between cameras
    calibrationDate: Date;
    measurementAccuracy: number; // % for length measurement
  };

  // Data storage
  storage: {
    capacity: number;            // TB
    recordingDuration: number;   // hours at current settings
  };
}

interface FishObservation {
  timestamp: Date;
  location: GeographicPosition;
  depth: number;

  // Fish detection
  fish: {
    boundingBox: { x: number; y: number; width: number; height: number };
    species?: string;
    length?: number;             // cm, from stereo measurement
    count: number;               // Number in frame
    behavior: string;            // Swimming, feeding, schooling, etc.
    confidence: number;          // 0-1, AI classification confidence
  }[];

  // Environmental
  visibility: number;            // meters
  current: number;               // m/s
  temperature: number;           // °C

  // Image quality
  imageQuality: {
    sharpness: number;           // 0-1
    contrast: number;            // 0-1
    illumination: number;        // 0-1
  };

  qualityFlag: QualityFlag;
}
```

### Philosophy: 弘益人間 (Benefit All Humanity)

Biological sensors serve humanity by revealing and protecting ocean life:

**Biodiversity Conservation:** eDNA sensors detect rare and endangered species, informing protection efforts

**Fisheries Management:** Acoustic and imaging sensors assess fish stocks, supporting sustainable harvest and food security

**Ecosystem Monitoring:** Plankton sensors track ecosystem health, detecting harmful algal blooms and regime shifts

**Early Warning:** Biological sensors detect harmful species before blooms or invasions cause damage

**Shared Understanding:** Biological ocean data supports global biodiversity assessments and conservation planning

The ocean's living diversity sustains humanity. Monitoring it with advanced sensors and sharing that knowledge openly benefits all people, especially those who depend on ocean resources.

---

**Next Chapter:** We'll explore optical sensors that measure turbidity, fluorescence, and light properties - revealing water clarity, suspended particles, and photosynthetic processes.

---

© 2025 SmileStory Inc. / WIA
弘익人間 (Hongik Ingan) · Benefit All Humanity
