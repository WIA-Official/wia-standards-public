# Chapter 5: Optical Sensors (Turbidity, Fluorescence)

## Light and Particles in the Ocean

Optical sensors measure how light interacts with seawater and suspended particles, revealing water clarity, particle concentration, phytoplankton biomass, and ocean color. These measurements are fundamental for understanding primary productivity, sediment transport, water quality, and ecosystem dynamics. From satellite ocean color sensors observing global chlorophyll patterns to in-situ turbidity sensors detecting sediment plumes, optical measurements span vast spatial scales.

### Light in the Ocean

Sunlight entering the ocean is absorbed and scattered by water molecules, dissolved substances, and suspended particles. Understanding light propagation is essential for interpreting optical sensor measurements.

#### Inherent Optical Properties (IOPs)

**IOPs** depend only on the medium, not the ambient light field:

**Absorption (a):** Light converted to heat by molecules
**Scattering (b):** Light deflected by particles and molecules
**Beam attenuation (c):** Total light removal, c = a + b
**Backscattering (bb):** Light scattered backwards

```typescript
interface InherentOpticalProperties {
  wavelength: number;            // nm
  depth: number;                 // meters

  // Core IOPs
  absorption: number;            // m⁻¹, a(λ)
  scattering: number;            // m⁻¹, b(λ)
  beamAttenuation: number;       // m⁻¹, c(λ) = a + b
  backscattering: number;        // m⁻¹, bb(λ)

  // Partitioned absorption
  absorptionComponents: {
    water: number;               // m⁻¹, pure seawater
    phytoplankton: number;       // m⁻¹, aph(λ)
    CDOM: number;                // m⁻¹, aCDOM(λ), colored dissolved organic matter
    detritus: number;            // m⁻¹, ad(λ), non-algal particles
  };

  // Partitioned scattering
  scatteringComponents: {
    water: number;               // m⁻¹, molecular scattering
    particles: number;           // m⁻¹, bp(λ)
  };

  // Derived properties
  singleScatteringAlbedo: number; // b / c, probability of scattering vs absorption
  backscatteringRatio: number;    // bb / b, fraction scattered backwards
}

function calculateBeamAttenuation(
  absorption: number,
  scattering: number
): number {
  return absorption + scattering;
}

function calculateAbsorption(
  aWater: number,
  aPhyto: number,
  aCDOM: number,
  aDetritus: number
): number {
  return aWater + aPhyto + aCDOM + aDetritus;
}
```

#### Apparent Optical Properties (AOPs)

**AOPs** depend on both the medium and the light field (sun angle, sky conditions):

**Diffuse attenuation (Kd):** Rate of downwelling irradiance decrease with depth
**Remote sensing reflectance (Rrs):** Ocean color measured by satellites
**Photosynthetically Available Radiation (PAR):** Light usable by phytoplankton (400-700 nm)

```typescript
interface ApparentOpticalProperties {
  wavelength: number;            // nm
  depth: number;                 // meters

  // Downwelling irradiance
  Ed: number;                    // W/m²/nm
  Kd: number;                    // m⁻¹, diffuse attenuation coefficient

  // Upwelling radiance
  Lu: number;                    // W/m²/sr/nm
  Ku: number;                    // m⁻¹, upwelling attenuation

  // Remote sensing reflectance
  Rrs: number;                   // sr⁻¹, Lu / Ed just below surface

  // PAR
  PAR?: number;                  // μmol photons/m²/s, integrated 400-700 nm

  // Sun geometry
  sunZenithAngle: number;        // degrees
  skyCondition: "clear" | "cloudy" | "overcast";
}

function calculateEuphoticDepth(Kd_PAR: number): number {
  // Euphotic zone: depth where PAR = 1% of surface value
  // Ed(z) = Ed(0) * exp(-Kd * z)
  // 0.01 = exp(-Kd * z_eu)
  const z_euphotic = -Math.log(0.01) / Kd_PAR;
  return z_euphotic;
}

function estimateChlorophyllFromOceanColor(
  Rrs_443: number,               // sr⁻¹, blue band
  Rrs_555: number                // sr⁻¹, green band
): number {
  // OC3 algorithm for chlorophyll from ocean color
  // Used by MODIS, VIIRS satellites

  const ratio = Math.max(Rrs_443, Rrs_490) / Rrs_555;
  const log10_ratio = Math.log10(ratio);

  // Polynomial coefficients
  const a = [0.2424, -2.7423, 1.8017, 0.0015, -1.2280];

  const log10_chl = a[0] +
                    a[1] * log10_ratio +
                    a[2] * Math.pow(log10_ratio, 2) +
                    a[3] * Math.pow(log10_ratio, 3) +
                    a[4] * Math.pow(log10_ratio, 4);

  const chlorophyll = Math.pow(10, log10_chl);

  return chlorophyll;  // mg/m³
}
```

### Turbidity Sensors

**Turbidity** measures water cloudiness from suspended particles (sediment, plankton, detritus). High turbidity reduces light penetration, affecting photosynthesis, fish habitat, and coral reefs.

#### Nephelometric Turbidity Sensors

**Principle:** Measure light scattered by particles at a specific angle (typically 90° or 45°)

**Units:**
- **NTU:** Nephelometric Turbidity Units
- **FNU:** Formazin Nephelometric Units
- **FTU:** Formazin Turbidity Units

```typescript
interface TurbiditySensor {
  type: "nephelometric" | "backscatter";
  serialNumber: string;

  // Optical configuration
  optics: {
    lightSource: "LED" | "tungsten_lamp";
    wavelength: number;          // nm, typically 860 (infrared)
    scatteringAngle: number;     // degrees, typically 90
    detectorAngle: number;       // degrees
  };

  // Calibration
  calibration: {
    date: Date;
    standard: "formazin" | "StablCal";
    calibrationPoints: {
      standardValue: number;     // NTU
      sensorReading: number;     // counts or voltage
    }[];
    slope: number;
    offset: number;
  };

  // Measurement range
  range: {
    min: number;                 // NTU
    max: number;                 // NTU
  };
  resolution: number;            // NTU
  accuracy: number;              // NTU or % of reading

  // Environmental effects
  temperatureCompensation: boolean;
  ambientLightRejection: boolean;
}

function calculateTurbidity(
  sensor: TurbiditySensor,
  scatterSignal: number          // Detector output
): number {
  // Apply calibration
  const turbidity = sensor.calibration.slope * scatterSignal +
                   sensor.calibration.offset;

  return Math.max(0, turbidity);
}

function convertTurbidityToSSC(
  turbidity: number,             // NTU
  siteSpecificFactor: number     // mg/L per NTU, from regression
): number {
  // Convert turbidity to suspended sediment concentration
  // Requires site-specific calibration

  const SSC = turbidity * siteSpecificFactor;

  return SSC;  // mg/L
}
```

#### Optical Backscatter Sensors (OBS)

**OBS** sensors measure light backscattered at angles >90°, sensitive to particle concentration:

```typescript
interface OpticalBackscatterSensor {
  type: "OBS";
  serialNumber: string;

  // Configuration
  optics: {
    wavelength: number[];        // nm, often multiple wavelengths
    scatteringAngle: number;     // degrees, typically 140-170
  };

  // Calibration
  calibration: {
    date: Date;
    sedimentType: string;        // Calibration dependent on particle type
    calibrationCurve: {
      concentration: number[];   // mg/L
      signal: number[];          // counts
    };
  };

  // Multi-wavelength capability
  spectralBackscatter?: {
    wavelengths: number[];       // nm
    backscatterCoefficient: number[]; // m⁻¹ sr⁻¹
    particleSizeEstimate?: number;    // μm, from spectral slope
  };
}

function estimateParticleSize_Spectral(
  bb_470: number,                // m⁻¹ sr⁻¹ at 470 nm
  bb_700: number                 // m⁻¹ sr⁻¹ at 700 nm
): number {
  // Spectral slope of backscattering indicates particle size
  // Smaller slope (flatter spectrum) = larger particles

  const gamma = -Math.log(bb_700 / bb_470) / Math.log(700 / 470);

  // Approximate particle size from gamma
  // Empirical relationship: smaller gamma = larger particles

  let particleSize: number;
  if (gamma < 0.5) {
    particleSize = 10;  // μm, large particles
  } else if (gamma < 1.0) {
    particleSize = 2;   // μm, medium particles
  } else {
    particleSize = 0.5; // μm, small particles
  }

  return particleSize;
}
```

### Beam Transmissometers

**Transmissometers** measure light transmission through a fixed path length, quantifying beam attenuation:

**Principle:** Compare transmitted light to source intensity; attenuation follows Beer-Lambert Law

**Beer-Lambert Law:** I = I₀ * exp(-c * L)
- I: Transmitted intensity
- I₀: Source intensity
- c: Beam attenuation coefficient (m⁻¹)
- L: Path length (m)

```typescript
interface BeamTransmissometer {
  type: "transmissometer";
  serialNumber: string;

  // Optical configuration
  optics: {
    wavelength: number;          // nm, typically 660 (red)
    pathLength: number;          // meters, typically 0.25
    beamDiameter: number;        // cm
    acceptanceAngle: number;     // degrees, collimated beam
  };

  // Calibration
  calibration: {
    date: Date;
    airCalibration: number;      // Counts in air (I₀)
    waterCalibration: number;    // Counts in pure water
  };

  // Measurement
  range: {
    cMin: number;                // m⁻¹
    cMax: number;                // m⁻¹
  };
  resolution: number;            // m⁻¹
  accuracy: number;              // m⁻¹
}

function calculateBeamAttenuationCoefficient(
  sensor: BeamTransmissometer,
  transmittedSignal: number      // Counts
): number {
  const I = transmittedSignal;
  const I0 = sensor.calibration.airCalibration;
  const L = sensor.optics.pathLength;

  // Beer-Lambert Law: I = I0 * exp(-c * L)
  // Solve for c: c = -ln(I/I0) / L

  const c = -Math.log(I / I0) / L;

  return c;  // m⁻¹
}

function calculateTransmittance(c: number, pathLength: number): number {
  // Transmittance: fraction of light transmitted
  const transmittance = Math.exp(-c * pathLength);
  return transmittance * 100;  // Percentage
}
```

### Fluorometers

Beyond chlorophyll fluorescence (Chapter 4), fluorometers detect other fluorescent substances:

#### CDOM Fluorescence

**Colored Dissolved Organic Matter (CDOM)** fluoresces when excited with UV/blue light:

**Sources:** Terrestrial runoff, phytoplankton degradation, bacterial activity
**Applications:** Trace freshwater inputs, organic matter dynamics, UV attenuation

```typescript
interface CDOMFluorometer {
  type: "CDOM_fluorometer";
  serialNumber: string;

  // Optical configuration
  optics: {
    excitationWavelength: number;  // nm, typically 370
    emissionWavelength: number;    // nm, typically 460
    excitationBandwidth: number;
    emissionBandwidth: number;
  };

  // Calibration
  calibration: {
    date: Date;
    standard: "quinine_sulfate" | "fluorescein";
    // Quinine Sulfate Units (QSU) or ppb equivalents
    darkCounts: number;
    scaleFactor: number;         // QSU per count
  };

  // Measurement
  range: { min: number; max: number }; // QSU or ppb
  resolution: number;
  accuracy: number;
}

function calculateCDOM(
  sensor: CDOMFluorometer,
  fluorescenceCounts: number,
  temperature: number
): number {
  // Subtract dark counts
  const signal = fluorescenceCounts - sensor.calibration.darkCounts;

  // Apply scale factor
  let CDOM = signal * sensor.calibration.scaleFactor;

  // Temperature compensation (fluorescence decreases ~1%/°C)
  const tempCorrection = 1 + 0.01 * (temperature - 20);
  CDOM *= tempCorrection;

  return CDOM;  // QSU or ppb
}
```

#### Oil Fluorescence Sensors

**Oil detection** uses fluorescence in the UV range:

**Principle:** Aromatic hydrocarbons in oil fluoresce when excited with UV light (254-365 nm)

**Applications:** Oil spill detection, produced water monitoring, leak detection

```typescript
interface OilFluorescenceSensor {
  type: "oil_fluorometer";
  serialNumber: string;

  // UV excitation
  optics: {
    excitationWavelength: number;  // nm, typically 254 or 365
    emissionRange: [number, number]; // nm, e.g., [300, 400]
    sensitivity: string;           // ppm oil
  };

  // Oil type calibration
  calibration: {
    oilType: "crude" | "diesel" | "gasoline" | "aromatic_standard";
    concentrationCurve: {
      concentration: number[];   // ppm
      fluorescence: number[];    // counts
    };
  };

  // Detection limits
  minimumDetectable: number;     // ppm
  range: { min: number; max: number }; // ppm
}
```

### Radiometers and Irradiance Sensors

Radiometers measure light intensity at specific wavelengths or bands:

#### PAR Sensors

**Photosynthetically Available Radiation (PAR)** sensors measure light for photosynthesis:

**Range:** 400-700 nm
**Units:** μmol photons/m²/s

```typescript
interface PARSensor {
  type: "PAR_sensor";
  serialNumber: string;

  // Spectral response
  spectralRange: [number, number]; // nm, [400, 700]
  quantumEfficiency: number;     // %, uniformity across PAR range

  // Calibration
  calibration: {
    date: Date;
    method: "lamp_standard" | "outdoor_comparison";
    immersionCoefficient: number; // Correction for in-water measurements
    sensitivity: number;         // μmol/m²/s per volt
  };

  // Angular response
  cosineResponse: {
    ideal: boolean;              // True if perfect cosine collector
    deviation: number;           // % deviation from cosine at 45°
  };

  // Measurement
  range: { min: number; max: number }; // μmol/m²/s
  resolution: number;
  accuracy: number;              // % of reading
}

function calculatePAR(
  sensor: PARSensor,
  voltage: number
): number {
  const PAR = voltage * sensor.calibration.sensitivity;

  // Apply immersion correction if underwater
  const PAR_corrected = PAR * sensor.calibration.immersionCoefficient;

  return PAR_corrected;  // μmol/m²/s
}

function estimatePrimaryProduction(
  PAR: number,                   // μmol/m²/s
  chlorophyll: number,           // mg/m³
  depth: number                  // meters
): number {
  // Simplified primary production model
  // Webb et al. (1974) model

  const Pmax = 5.0;              // mg C / mg Chl / hour
  const alpha = 0.02;            // mg C / mg Chl / hour / (μmol/m²/s)

  // Production vs irradiance curve
  const production = Pmax * (1 - Math.exp(-alpha * PAR / Pmax));

  // Scale by chlorophyll
  const volumetricProduction = production * chlorophyll;

  return volumetricProduction;  // mg C/m³/hour
}
```

#### Hyperspectral Radiometers

**Hyperspectral radiometers** measure radiance/irradiance at many narrow wavelength bands:

**Applications:** Ocean color, phytoplankton functional types, CDOM absorption, water quality

```typescript
interface HyperspectralRadiometer {
  type: "hyperspectral_radiometer";
  serialNumber: string;

  // Spectral characteristics
  spectralRange: [number, number]; // nm, e.g., [350, 800]
  spectralChannels: number;        // Number of bands
  spectralResolution: number;      // nm, bandwidth per channel

  // Radiometric units
  measurementType: "radiance" | "irradiance";
  units: "W/m²/nm" | "W/m²/nm/sr";

  // Calibration
  calibration: {
    date: Date;
    lampStandard: string;
    coefficients: number[];      // Sensitivity per channel
  };

  // Measurement
  integrationTime: number;       // milliseconds
  dynamicRange: number;          // bits, e.g., 16-bit
}

interface HyperspectralMeasurement {
  timestamp: Date;
  location: GeographicPosition;

  // Spectral data
  wavelengths: number[];         // nm
  radiance: number[];            // W/m²/nm/sr or W/m²/nm

  // Derived products
  remoteSensingReflectance?: number[]; // Rrs(λ)
  chlorophyll?: number;          // mg/m³
  CDOM_absorption?: number[];    // m⁻¹
  suspendedSediment?: number;    // mg/L

  qualityFlag: QualityFlag;
}
```

### Integrated Optical Packages

Modern platforms combine multiple optical sensors:

```typescript
interface IntegratedOpticalPackage {
  packageId: string;
  manufacturer: string;

  // Sensor suite
  sensors: {
    transmissometer?: BeamTransmissometer;
    backscatterSensor?: OpticalBackscatterSensor;
    chlorophyllFluorometer?: any;
    CDOMFluorometer?: CDOMFluorometer;
    turbidity?: TurbiditySensor;
    PAR?: PARSensor;
  };

  // Deployment platform
  platform: "CTD_rosette" | "mooring" | "glider" | "float" | "AUV";

  // Sampling
  samplingRate: number;          // Hz
  dataStorage: string;
  powerConsumption: number;      // Watts

  // Data products
  derivedProducts: {
    particleConcentration: boolean;
    phytoplanktonBiomass: boolean;
    dissolvedOrganicMatter: boolean;
    waterClarity: boolean;
  };
}
```

### Applications and Case Studies

#### Harmful Algal Bloom Detection

Optical sensors detect algal blooms through chlorophyll fluorescence, water color changes, and optical properties:

```typescript
interface AlgalBloomDetection {
  location: GeographicPosition;
  timestamp: Date;

  // Optical indicators
  chlorophyll: number;           // mg/m³, elevated
  CDOM: number;                  // QSU, from cell lysis
  backscatter: number;           // m⁻¹, from cells
  Rrs_670: number;               // sr⁻¹, red tide signature

  // Bloom assessment
  bloomStatus: "normal" | "developing" | "bloom" | "severe_bloom";
  bloomType?: "diatom" | "dinoflagellate" | "cyanobacteria" | "unknown";
  toxicPotential: "low" | "medium" | "high";

  // Alert
  alertLevel: "none" | "advisory" | "warning" | "closure";
}
```

#### Sediment Plume Monitoring

Turbidity and backscatter sensors track sediment from dredging, storms, river discharge:

```typescript
interface SedimentPlumeMonitoring {
  source: "dredging" | "river_discharge" | "storm" | "coastal_erosion";
  location: GeographicPosition;

  // Measurements
  turbidity: number;             // NTU
  suspendedSediment: number;     // mg/L
  beamAttenuation: number;       // m⁻¹
  lightAttenuation_Kd: number;   // m⁻¹

  // Impact assessment
  exceedance: {
    baseline: number;            // NTU, ambient conditions
    threshold: number;           // NTU, regulatory limit
    exceeded: boolean;
    duration: number;            // hours above threshold
  };

  // Ecological impact
  coralReefImpact?: {
    distance: number;            // km from reef
    lightReduction: number;      // % at reef depth
    risk: "low" | "moderate" | "high";
  };
}
```

### Philosophy: 弘益人間 (Benefit All Humanity)

Optical sensors embody 弘益人間 by revealing ocean health and productivity:

**Water Quality Protection:** Turbidity and optical sensors monitor coastal water quality, protecting public health and ecosystems

**Primary Production Monitoring:** PAR and ocean color sensors track phytoplankton globally, quantifying ocean productivity that supports all marine life

**Early Warning:** Optical sensors detect harmful algal blooms, protecting fisheries and coastal communities

**Climate Research:** Ocean color satellites monitor global phytoplankton, revealing carbon cycle dynamics and climate change impacts

**Shared Observation:** Satellite ocean color data is freely available worldwide, enabling environmental monitoring for all nations

Light in the ocean sustains all marine life through photosynthesis. Measuring it accurately and sharing that knowledge serves all humanity.

---

**Next Chapter:** We'll explore acoustic sensors including hydrophones and ADCPs that use sound to probe the ocean, from marine mammal calls to current measurements.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
