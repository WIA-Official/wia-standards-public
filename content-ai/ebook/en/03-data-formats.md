# Chapter 3: Data Formats

## Content Credential Schemas and Metadata Standards

This chapter defines the data structures and formats for AI content credentials, including C2PA manifest schemas, detection result formats, and provenance metadata.

---

## C2PA Manifest Structure

### Core Manifest Schema

```typescript
// WIA Content AI - C2PA Manifest Implementation
// Based on C2PA Specification v1.3

/**
 * C2PA Manifest - Complete content credential
 */
interface C2PAManifest {
  claimGenerator: string;           // Tool that created this claim
  title: string;                    // Content title
  format: string;                   // MIME type
  instanceId: string;               // Unique instance identifier
  claim: Claim;                     // Core claim data
  signature: ManifestSignature;     // Cryptographic signature
  assertions: Assertion[];          // Content assertions
  credentials: VerifiableCredential[];  // Issuer credentials
  ingredients: Ingredient[];        // Source materials
}

/**
 * Claim - Core provenance information
 */
interface Claim {
  dcTitle: string;                  // Dublin Core title
  dcCreator?: string;               // Creator name
  dcFormat: string;                 // Media format
  claimGeneratorInfo: GeneratorInfo;
  signatureInfo: SignatureInfo;
  assertionStore: AssertionRef[];
  ingredientStore: IngredientRef[];
  redactionStore?: RedactionRef[];
  alg: SignatureAlgorithm;
}

interface GeneratorInfo {
  name: string;                     // Generator name
  version: string;                  // Version string
  icon?: ContentURI;                // Generator icon
}

interface SignatureInfo {
  alg: SignatureAlgorithm;
  issuer: string;                   // Signer identity
  time: string;                     // ISO 8601 timestamp
  certChain?: string[];             // Certificate chain
}

type SignatureAlgorithm =
  | "ES256"      // ECDSA P-256
  | "ES384"      // ECDSA P-384
  | "ES512"      // ECDSA P-521
  | "PS256"      // RSA-PSS SHA-256
  | "PS384"      // RSA-PSS SHA-384
  | "PS512"      // RSA-PSS SHA-512
  | "Ed25519";   // EdDSA

interface AssertionRef {
  url: string;
  hash: HashValue;
}

interface IngredientRef {
  url: string;
  hash: HashValue;
  relationship: IngredientRelationship;
}

type IngredientRelationship =
  | "parentOf"        // This content derived from ingredient
  | "componentOf"     // Ingredient is a component
  | "inputTo";        // Ingredient was input

interface RedactionRef {
  url: string;
  reason: string;
}

interface HashValue {
  alg: HashAlgorithm;
  hash: string;       // Base64 encoded
}

type HashAlgorithm = "SHA-256" | "SHA-384" | "SHA-512" | "SHA3-256";

type ContentURI = string;

/**
 * C2PA Manifest Builder
 */
class C2PAManifestBuilder {
  private manifest: Partial<C2PAManifest> = {};
  private assertions: Assertion[] = [];
  private ingredients: Ingredient[] = [];

  /**
   * Set generator info
   */
  setGenerator(name: string, version: string): this {
    this.manifest.claimGenerator = `${name}/${version}`;
    return this;
  }

  /**
   * Set content metadata
   */
  setContentInfo(title: string, format: string): this {
    this.manifest.title = title;
    this.manifest.format = format;
    this.manifest.instanceId = crypto.randomUUID();
    return this;
  }

  /**
   * Add AI generation assertion
   */
  addAIGenerationAssertion(
    model: string,
    provider: string,
    parameters: AIGenerationParameters
  ): this {
    const assertion: AIGeneratedAssertion = {
      label: "c2pa.ai_generated",
      data: {
        model,
        provider,
        generatedAt: new Date().toISOString(),
        parameters,
        softwareAgent: {
          name: provider,
          version: parameters.modelVersion || "unknown"
        }
      }
    };
    this.assertions.push(assertion);
    return this;
  }

  /**
   * Add creative work assertion
   */
  addCreativeWorkAssertion(
    author: string,
    dateCreated: string,
    copyrightNotice?: string
  ): this {
    const assertion: CreativeWorkAssertion = {
      label: "stds.schema-org.CreativeWork",
      data: {
        "@type": "CreativeWork",
        author: {
          "@type": "Person",
          name: author
        },
        dateCreated,
        copyrightNotice
      }
    };
    this.assertions.push(assertion);
    return this;
  }

  /**
   * Add actions assertion (edit history)
   */
  addActionsAssertion(actions: ContentAction[]): this {
    const assertion: ActionsAssertion = {
      label: "c2pa.actions",
      data: {
        actions: actions.map(action => ({
          action: action.action,
          when: action.timestamp,
          softwareAgent: action.tool,
          parameters: action.parameters,
          digitalSourceType: action.sourceType
        }))
      }
    };
    this.assertions.push(assertion);
    return this;
  }

  /**
   * Add ingredient (source content)
   */
  addIngredient(
    title: string,
    format: string,
    hash: string,
    relationship: IngredientRelationship,
    manifest?: C2PAManifest
  ): this {
    const ingredient: Ingredient = {
      title,
      format,
      documentId: crypto.randomUUID(),
      instanceId: crypto.randomUUID(),
      hash: {
        alg: "SHA-256",
        hash
      },
      relationship,
      validationStatus: manifest ? "VALID" : "UNKNOWN",
      manifest
    };
    this.ingredients.push(ingredient);
    return this;
  }

  /**
   * Build the complete manifest
   */
  async build(signer: ManifestSigner): Promise<C2PAManifest> {
    // Construct claim
    const claim: Claim = {
      dcTitle: this.manifest.title!,
      dcFormat: this.manifest.format!,
      claimGeneratorInfo: {
        name: this.manifest.claimGenerator!.split("/")[0],
        version: this.manifest.claimGenerator!.split("/")[1]
      },
      signatureInfo: {
        alg: signer.algorithm,
        issuer: signer.issuer,
        time: new Date().toISOString()
      },
      assertionStore: this.assertions.map((a, i) => ({
        url: `self#assertion/${i}`,
        hash: this.computeAssertionHash(a)
      })),
      ingredientStore: this.ingredients.map((ing, i) => ({
        url: `self#ingredient/${i}`,
        hash: ing.hash,
        relationship: ing.relationship
      })),
      alg: signer.algorithm
    };

    // Sign the manifest
    const signature = await signer.sign(claim);

    return {
      claimGenerator: this.manifest.claimGenerator!,
      title: this.manifest.title!,
      format: this.manifest.format!,
      instanceId: this.manifest.instanceId!,
      claim,
      signature,
      assertions: this.assertions,
      credentials: signer.credentials,
      ingredients: this.ingredients
    };
  }

  private computeAssertionHash(assertion: Assertion): HashValue {
    // Compute SHA-256 hash of assertion
    return {
      alg: "SHA-256",
      hash: "" // Computed hash
    };
  }
}

interface ManifestSigner {
  algorithm: SignatureAlgorithm;
  issuer: string;
  credentials: VerifiableCredential[];
  sign(claim: Claim): Promise<ManifestSignature>;
}

interface ManifestSignature {
  alg: SignatureAlgorithm;
  sig: string;            // Base64 signature
  pad?: string;           // Padding for embedding
  certChain: string[];    // X.509 certificate chain
  timestamp?: string;     // RFC 3161 timestamp
}
```

---

## Assertion Types

### AI Generation Assertion

```typescript
/**
 * AI Generation Assertion - Declares AI-generated content
 */
interface AIGeneratedAssertion extends Assertion {
  label: "c2pa.ai_generated";
  data: {
    model: string;              // Model identifier
    provider: string;           // AI provider
    generatedAt: string;        // ISO 8601 timestamp
    parameters: AIGenerationParameters;
    softwareAgent: SoftwareAgent;
    trainingDataInfo?: TrainingDataInfo;
  };
}

interface AIGenerationParameters {
  prompt?: string;              // Generation prompt (may be redacted)
  promptHash?: string;          // Hash of prompt if redacted
  seed?: number;                // Random seed
  guidance?: number;            // Guidance scale
  steps?: number;               // Inference steps
  modelVersion?: string;        // Model version
  samplerType?: string;         // Sampling method
  negativePrompt?: string;      // Negative prompt
  imageSize?: { width: number; height: number };
  duration?: number;            // For audio/video
  temperature?: number;         // For text generation
  maxTokens?: number;           // For text generation
}

interface SoftwareAgent {
  name: string;
  version: string;
  icon?: string;
  url?: string;
}

interface TrainingDataInfo {
  datasets?: string[];          // Training dataset names
  trainingCutoff?: string;      // Training data cutoff date
  finetuned?: boolean;          // Custom fine-tuning
  license?: string;             // Training data license
}

/**
 * Example: DALL-E generation assertion
 */
const dalleAssertion: AIGeneratedAssertion = {
  label: "c2pa.ai_generated",
  data: {
    model: "dall-e-3",
    provider: "OpenAI",
    generatedAt: "2025-01-10T12:00:00Z",
    parameters: {
      prompt: "A serene mountain landscape at sunset",
      imageSize: { width: 1024, height: 1024 },
      seed: 42,
      modelVersion: "3.0"
    },
    softwareAgent: {
      name: "DALL-E",
      version: "3.0",
      url: "https://openai.com/dall-e-3"
    },
    trainingDataInfo: {
      trainingCutoff: "2023-04",
      license: "OpenAI Terms of Service"
    }
  }
};
```

### Actions Assertion

```typescript
/**
 * Actions Assertion - Edit history tracking
 */
interface ActionsAssertion extends Assertion {
  label: "c2pa.actions";
  data: {
    actions: ActionEntry[];
  };
}

interface ActionEntry {
  action: ActionType;
  when?: string;                // ISO 8601 timestamp
  softwareAgent?: SoftwareAgent;
  parameters?: ActionParameters;
  digitalSourceType?: DigitalSourceType;
  changed?: ChangedRegion[];
  instanceId?: string;
  reason?: string;
}

type ActionType =
  // Creation actions
  | "c2pa.created"              // Original creation
  | "c2pa.opened"               // Opened existing content
  | "c2pa.placed"               // Placed ingredient

  // AI-specific actions
  | "c2pa.prompted"             // AI prompt submitted
  | "c2pa.ai_generated"         // AI generation
  | "c2pa.ai_modified"          // AI modification
  | "c2pa.upscaled"             // AI upscaling
  | "c2pa.inpainted"            // AI inpainting
  | "c2pa.outpainted"           // AI outpainting
  | "c2pa.style_transferred"    // Style transfer

  // Traditional editing actions
  | "c2pa.cropped"              // Cropping
  | "c2pa.color_adjusted"       // Color correction
  | "c2pa.filtered"             // Filter applied
  | "c2pa.orientation"          // Rotation/flip
  | "c2pa.resized"              // Size change
  | "c2pa.drawing"              // Drawing/annotation
  | "c2pa.unknown"              // Unspecified edit

  // Export actions
  | "c2pa.converted"            // Format conversion
  | "c2pa.published"            // Publishing action
  | "c2pa.transcoded";          // Transcoding

type DigitalSourceType =
  | "http://cv.iptc.org/newscodes/digitalsourcetype/trainedAlgorithmicMedia"
  | "http://cv.iptc.org/newscodes/digitalsourcetype/compositeWithTrainedAlgorithmicMedia"
  | "http://cv.iptc.org/newscodes/digitalsourcetype/algorithmicMedia"
  | "http://cv.iptc.org/newscodes/digitalsourcetype/digitalCapture"
  | "http://cv.iptc.org/newscodes/digitalsourcetype/digitalArt"
  | "http://cv.iptc.org/newscodes/digitalsourcetype/virtualRecording";

interface ActionParameters {
  [key: string]: any;
  // Common parameters
  description?: string;
  ingredientRef?: string;
  // AI parameters
  prompt?: string;
  model?: string;
  strength?: number;
}

interface ChangedRegion {
  region: Region;
  description?: string;
}

interface Region {
  type: "RECTANGLE" | "POLYGON" | "TIME_RANGE";
  // For RECTANGLE
  x?: number;
  y?: number;
  width?: number;
  height?: number;
  // For TIME_RANGE
  start?: number;
  end?: number;
  // For POLYGON
  points?: Array<{ x: number; y: number }>;
}

/**
 * Example: Image editing history
 */
const editHistoryAssertion: ActionsAssertion = {
  label: "c2pa.actions",
  data: {
    actions: [
      {
        action: "c2pa.created",
        when: "2025-01-10T10:00:00Z",
        softwareAgent: { name: "Sony Alpha", version: "A7IV" },
        digitalSourceType: "http://cv.iptc.org/newscodes/digitalsourcetype/digitalCapture"
      },
      {
        action: "c2pa.opened",
        when: "2025-01-10T11:00:00Z",
        softwareAgent: { name: "Adobe Photoshop", version: "25.0" }
      },
      {
        action: "c2pa.cropped",
        when: "2025-01-10T11:05:00Z",
        softwareAgent: { name: "Adobe Photoshop", version: "25.0" },
        parameters: { description: "Cropped to 16:9 aspect ratio" }
      },
      {
        action: "c2pa.ai_modified",
        when: "2025-01-10T11:10:00Z",
        softwareAgent: { name: "Adobe Firefly", version: "2.0" },
        digitalSourceType: "http://cv.iptc.org/newscodes/digitalsourcetype/compositeWithTrainedAlgorithmicMedia",
        changed: [{
          region: { type: "RECTANGLE", x: 100, y: 100, width: 200, height: 200 },
          description: "Generative fill - removed power lines"
        }]
      },
      {
        action: "c2pa.published",
        when: "2025-01-10T12:00:00Z",
        softwareAgent: { name: "Adobe Photoshop", version: "25.0" }
      }
    ]
  }
};
```

---

## Ingredient Structure

```typescript
/**
 * Ingredient - Source content reference
 */
interface Ingredient {
  title: string;                    // Ingredient title
  format: string;                   // MIME type
  documentId?: string;              // Document identifier
  instanceId: string;               // Instance identifier
  hash: HashValue;                  // Content hash
  relationship: IngredientRelationship;
  validationStatus: ValidationStatus;
  manifest?: C2PAManifest;          // Embedded manifest if available
  thumbnail?: Thumbnail;
  metadata?: IngredientMetadata;
}

type ValidationStatus =
  | "VALID"           // Manifest valid and trusted
  | "INVALID"         // Manifest validation failed
  | "UNKNOWN"         // No manifest or unverified
  | "EXPIRED"         // Certificate expired
  | "REVOKED";        // Certificate revoked

interface Thumbnail {
  format: "image/jpeg" | "image/png";
  data: string;       // Base64 encoded
}

interface IngredientMetadata {
  dateCreated?: string;
  dateModified?: string;
  author?: string;
  description?: string;
  license?: string;
  aiGenerated?: boolean;
}

/**
 * Example: Composite image with AI-generated ingredient
 */
const compositeIngredients: Ingredient[] = [
  {
    title: "Original photograph",
    format: "image/jpeg",
    instanceId: "uuid-1234-photo",
    hash: { alg: "SHA-256", hash: "abc123..." },
    relationship: "parentOf",
    validationStatus: "VALID",
    metadata: {
      dateCreated: "2025-01-01T10:00:00Z",
      author: "John Photographer",
      aiGenerated: false
    }
  },
  {
    title: "AI-generated background",
    format: "image/png",
    instanceId: "uuid-5678-ai",
    hash: { alg: "SHA-256", hash: "def456..." },
    relationship: "componentOf",
    validationStatus: "VALID",
    metadata: {
      dateCreated: "2025-01-05T14:00:00Z",
      aiGenerated: true,
      description: "Generated sunset sky"
    }
  }
];
```

---

## Detection Result Format

```typescript
/**
 * AI Detection Result Schema
 * Standardized format for detection outputs
 */
interface AIDetectionResult {
  version: "1.0";
  requestId: string;
  timestamp: string;

  content: ContentInfo;
  detection: DetectionVerdict;
  analysis: DetectionAnalysis;
  provenance?: ProvenanceInfo;
  confidence: ConfidenceMetrics;

  metadata: DetectionMetadata;
}

interface ContentInfo {
  type: "IMAGE" | "VIDEO" | "AUDIO" | "TEXT" | "MULTIMODAL";
  format: string;
  size: number;
  hash: HashValue;
  dimensions?: { width: number; height: number };
  duration?: number;
  wordCount?: number;
}

interface DetectionVerdict {
  isAIGenerated: boolean;
  confidence: number;              // 0.0 - 1.0
  generationType?: GenerationType;
  manipulationType?: ManipulationType[];
}

type GenerationType =
  | "FULLY_SYNTHETIC"              // Entirely AI-generated
  | "PARTIALLY_SYNTHETIC"          // AI modifications to real content
  | "HUMAN_CREATED"                // No AI detected
  | "UNCERTAIN";                   // Unable to determine

type ManipulationType =
  | "FACE_SWAP"                    // Deepfake face swap
  | "FACE_REENACTMENT"             // Puppet manipulation
  | "LIP_SYNC"                     // Audio-driven lip sync
  | "INPAINTING"                   // Region replacement
  | "STYLE_TRANSFER"               // Style modification
  | "TEXT_TO_IMAGE"                // Generated from text
  | "IMAGE_TO_IMAGE"               // Image transformation
  | "VOICE_CLONING"                // Synthetic voice
  | "TEXT_GENERATION"              // AI text
  | "UPSCALING"                    // AI enhancement
  | "COMPOSITING";                 // AI-assisted composition

interface DetectionAnalysis {
  models: ModelResult[];
  features: FeatureAnalysis;
  regions?: RegionalAnalysis[];
  timeline?: TemporalAnalysis[];
}

interface ModelResult {
  modelId: string;
  modelVersion: string;
  score: number;
  threshold: number;
  prediction: string;
  processingTime: number;
}

interface FeatureAnalysis {
  // Image features
  frequencyAnomalies?: number;
  compressionArtifacts?: number;
  colorHistogramDeviation?: number;
  noisePatterns?: number;
  edgeCoherence?: number;

  // Video features
  temporalConsistency?: number;
  blinkingPatterns?: number;
  lipSyncScore?: number;

  // Audio features
  spectralAnalysis?: number;
  prosodyScore?: number;
  breathingPatterns?: number;

  // Text features
  perplexityScore?: number;
  burstiness?: number;
  stylometricScore?: number;
}

interface RegionalAnalysis {
  region: Region;
  confidence: number;
  type: ManipulationType;
  description: string;
}

interface TemporalAnalysis {
  startTime: number;
  endTime: number;
  confidence: number;
  type: ManipulationType;
  description: string;
}

interface ProvenanceInfo {
  hasC2PA: boolean;
  manifestValid?: boolean;
  signerIdentity?: string;
  claimHistory?: ClaimSummary[];
  aiDisclosed?: boolean;
}

interface ClaimSummary {
  generator: string;
  timestamp: string;
  aiGenerated: boolean;
}

interface ConfidenceMetrics {
  overall: number;
  modelAgreement: number;
  featureStrength: number;
  provenanceBoost: number;
  uncertaintyFactors: string[];
}

interface DetectionMetadata {
  processingTime: number;
  modelsUsed: string[];
  apiVersion: string;
  region: string;
  cost?: number;
}

/**
 * Example: Deepfake detection result
 */
const deepfakeDetectionResult: AIDetectionResult = {
  version: "1.0",
  requestId: "det-abc123",
  timestamp: "2025-01-10T12:00:00Z",

  content: {
    type: "VIDEO",
    format: "video/mp4",
    size: 15000000,
    hash: { alg: "SHA-256", hash: "abc..." },
    dimensions: { width: 1920, height: 1080 },
    duration: 30
  },

  detection: {
    isAIGenerated: true,
    confidence: 0.94,
    generationType: "PARTIALLY_SYNTHETIC",
    manipulationType: ["FACE_SWAP", "LIP_SYNC"]
  },

  analysis: {
    models: [
      {
        modelId: "face-forensics-v3",
        modelVersion: "3.2.1",
        score: 0.96,
        threshold: 0.5,
        prediction: "MANIPULATED",
        processingTime: 2500
      },
      {
        modelId: "temporal-consistency",
        modelVersion: "1.1.0",
        score: 0.89,
        threshold: 0.6,
        prediction: "ANOMALIES_DETECTED",
        processingTime: 1800
      }
    ],
    features: {
      temporalConsistency: 0.72,
      blinkingPatterns: 0.45,
      lipSyncScore: 0.38
    },
    regions: [
      {
        region: { type: "RECTANGLE", x: 200, y: 100, width: 150, height: 200 },
        confidence: 0.94,
        type: "FACE_SWAP",
        description: "Face region shows synthesis artifacts"
      }
    ],
    timeline: [
      {
        startTime: 5.0,
        endTime: 25.0,
        confidence: 0.92,
        type: "LIP_SYNC",
        description: "Audio-visual mismatch in speech segment"
      }
    ]
  },

  confidence: {
    overall: 0.94,
    modelAgreement: 0.91,
    featureStrength: 0.85,
    provenanceBoost: 0,
    uncertaintyFactors: ["Low resolution source", "Heavy compression"]
  },

  metadata: {
    processingTime: 4500,
    modelsUsed: ["face-forensics-v3", "temporal-consistency"],
    apiVersion: "2.1.0",
    region: "us-west-2"
  }
};
```

---

## Summary

| Format | Purpose | Key Fields |
|--------|---------|------------|
| **C2PA Manifest** | Content provenance | Claims, assertions, signatures |
| **AI Generation Assertion** | AI disclosure | Model, parameters, timestamp |
| **Actions Assertion** | Edit history | Action sequence, tools, regions |
| **Ingredient** | Source tracking | Hash, relationship, validation |
| **Detection Result** | AI detection | Verdict, confidence, analysis |

---

**Next Chapter:** [Chapter 4: API Interface](./04-api-interface.md) - Authentication and detection API specifications.

---

© 2025 World Industry Association (WIA). All rights reserved.
弘益人間 (홍익인간) · Benefit All Humanity
