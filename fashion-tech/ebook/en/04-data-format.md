# Chapter 4: Data Format Specifications

## Learning Objectives

By the end of this chapter, you will understand:
- WIA Fashion JSON schema structure
- Garment data format with complete metadata
- Material database schema and properties
- Sustainability metrics data format
- 3D asset specifications and formats
- NFT metadata standards

---

## 4.1 Garment Interchange Format (GIF)

The core JSON schema for garments:

### 4.1.1 Complete Garment Schema

```typescript
interface WIAFashionGarment {
  // Schema metadata
  $schema: 'https://wiastandards.com/schemas/fashion/v1.0/garment.json';
  wia_standard: 'WIA-IND-001';
  version: '1.0.0';

  // Unique identification
  garment: {
    id: string;                    // Unique garment ID
    sku: string;                   // Stock keeping unit
    barcode?: string;              // UPC/EAN barcode
    nfcTag?: string;               // NFC chip ID (for phygital)

    // Basic information
    type: 'dress' | 'shirt' | 'pants' | 'jacket' | 'skirt' | 'accessory';
    brand: string;
    name: string;
    description: string;
    season: string;                // e.g., "Spring 2026"
    year: number;
    collection?: string;

    // Classification
    gender: 'women' | 'men' | 'unisex' | 'kids';
    category: string;              // e.g., "dresses"
    subcategory: string;           // e.g., "midi_dress"
    tags: string[];                // Search tags

    // Design details
    design: {
      style: string;               // e.g., "A-line"
      silhouette: string;
      neckline: string;
      sleeves: string;
      length: string;
      closure: string;
      pockets?: boolean;
      lining?: boolean;
      padding?: boolean;
    };

    // Materials composition
    materials: Array<{
      type: string;                // Material code
      percentage: number;          // 0-100
      weight_gsm: number;          // Grams per square meter
      origin: string;              // Country of origin
      certification?: string;      // e.g., "GOTS", "Fair Trade"
      sustainability_score: number; // 0-100
      supplier?: string;
      lotNumber?: string;
    }>;

    // Colors and patterns
    colors: Array<{
      name: string;
      hex: string;                 // #RRGGBB
      pantone?: string;            // Pantone code
      rgb: [number, number, number];
      cmyk?: [number, number, number, number];
      primary: boolean;            // Is primary color?
    }>;
    patterns?: Array<{
      type: string;                // "floral", "stripe", "solid"
      scale: 'small' | 'medium' | 'large';
      placement: string;
    }>;

    // Sizing
    sizes: {
      system: 'WIA_Universal' | 'US' | 'EU' | 'UK' | 'JP';
      available: string[];         // ["XS", "S", "M", "L", "XL"]
      measurements: {
        [size: string]: {
          chest?: number;          // cm
          waist?: number;
          hips?: number;
          length?: number;
          inseam?: number;
          shoulder?: number;
          sleeve?: number;
        };
      };
      fit: 'tight' | 'fitted' | 'regular' | 'relaxed' | 'oversized';
    };

    // 3D digital assets
    assets_3d: {
      models: Array<{
        format: 'glTF' | 'FBX' | 'OBJ' | 'USD' | 'USDZ';
        lod: 'high' | 'medium' | 'low' | 'ultralow';
        url: string;
        fileSize: number;          // bytes
        polygonCount: number;
        vertexCount: number;
        boundingBox?: {
          min: [number, number, number];
          max: [number, number, number];
        };
      }>;

      textures: {
        baseColor?: string;        // URL to texture
        normal?: string;
        metallic?: string;
        roughness?: string;
        ambientOcclusion?: string;
        emission?: string;
        opacity?: string;
      };

      animations?: Array<{
        name: string;              // e.g., "walking", "idle"
        url: string;
        duration: number;          // seconds
      }>;
    };

    // 2D assets (images)
    assets_2d: {
      images: Array<{
        type: 'product' | 'lifestyle' | 'detail' | 'model' | '360';
        url: string;
        width: number;
        height: number;
        angle?: number;            // For 360° views
        primary?: boolean;
      }>;
      videos?: Array<{
        type: 'product' | 'campaign' | 'howto';
        url: string;
        duration: number;
        thumbnail: string;
      }>;
    };

    // Physics properties (for simulation)
    physics: {
      clothType: 'cotton' | 'silk' | 'denim' | 'leather' | 'knit' | 'synthetic';
      stiffness: number;           // Spring constant (N/m)
      damping: number;             // 0-1
      mass: number;                // kg
      stretch: [number, number];   // [warp, weft] direction
      friction: number;            // Surface friction 0-1
      airResistance: number;       // 0-1
    };

    // Sustainability metrics
    sustainability: {
      totalScore: number;          // 0-100
      environmentalScore: number;  // 0-100
      socialScore: number;         // 0-100
      circularScore: number;       // 0-100

      carbonFootprint: {
        material: number;          // kg CO₂e
        manufacturing: number;
        transport: number;
        use: number;
        endOfLife: number;
        total: number;
        perWear: number;
      };

      waterFootprint: {
        material: number;          // liters
        manufacturing: number;
        use: number;
        total: number;
      };

      certifications: string[];    // ["GOTS", "Fair Trade", "B-Corp"]
      recyclability: number;       // 0-1
      biodegradable: boolean;
      repairability: number;       // 0-100
      expectedLifespan: number;    // years
    };

    // Care instructions
    care: {
      washing: {
        temperature: number;       // °C
        method: 'machine' | 'hand' | 'dry_clean_only';
        detergent: string;
        restrictions?: string[];
      };
      drying: {
        method: 'tumble_dry' | 'line_dry' | 'flat_dry';
        temperature?: 'low' | 'medium' | 'high';
      };
      ironing: {
        allowed: boolean;
        temperature?: 'low' | 'medium' | 'high';
      };
      dryCleaning: boolean;
      bleaching: boolean;
      specialInstructions?: string[];
    };

    // Pricing
    pricing: {
      retailPrice: number;
      currency: string;
      market: string;              // e.g., "US", "EU"
      vat?: number;
      discountPrice?: number;
      costPrice?: number;          // Wholesale
      msrp?: number;               // Manufacturer suggested
    };

    // Supply chain & traceability
    supplyChain?: {
      stages: Array<{
        stage: string;
        location: string;
        facility: string;
        timestamp: string;         // ISO 8601
        certification?: string;
        carbonEmitted?: number;
        verified: boolean;
        verifier?: string;
      }>;
    };

    // Blockchain integration
    blockchain?: {
      enabled: boolean;
      network: 'ethereum' | 'polygon' | 'solana' | 'other';
      contractAddress: string;
      tokenId?: number;
      materialPassport?: string;   // IPFS hash
      nftMetadata?: string;        // IPFS hash
    };

    // Metadata timestamps
    metadata: {
      createdAt: string;           // ISO 8601
      updatedAt: string;
      publishedAt?: string;
      deprecatedAt?: string;
      version: string;             // Semantic versioning
    };
  };
}
```

### 4.1.2 Example: Complete Dress Data

```json
{
  "$schema": "https://wiastandards.com/schemas/fashion/v1.0/garment.json",
  "wia_standard": "WIA-IND-001",
  "version": "1.0.0",

  "garment": {
    "id": "WIA-DRESS-2026-12345",
    "sku": "EF-CD-2026-001",
    "type": "dress",
    "brand": "EcoFashion",
    "name": "Coral Summer Dress",
    "description": "Sustainable A-line midi dress in organic cotton with flutter sleeves",
    "season": "Spring/Summer 2026",
    "year": 2026,
    "collection": "Sustainable Elegance",

    "gender": "women",
    "category": "dresses",
    "subcategory": "midi_dress",
    "tags": ["sustainable", "organic", "summer", "feminine", "coral", "a-line"],

    "design": {
      "style": "A-line",
      "silhouette": "fitted_bodice_flared_skirt",
      "neckline": "v-neck",
      "sleeves": "flutter_sleeves",
      "length": "midi",
      "closure": "back_zipper",
      "pockets": false,
      "lining": false,
      "padding": false
    },

    "materials": [
      {
        "type": "organic_cotton",
        "percentage": 95,
        "weight_gsm": 180,
        "origin": "India",
        "certification": "GOTS",
        "sustainability_score": 85,
        "supplier": "Organic Cotton Co.",
        "lotNumber": "OCT-2025-789"
      },
      {
        "type": "elastane",
        "percentage": 5,
        "weight_gsm": 9,
        "origin": "China",
        "sustainability_score": 28
      }
    ],

    "colors": [
      {
        "name": "Coral Pink",
        "hex": "#FF6B9D",
        "pantone": "17-2034 TCX",
        "rgb": [255, 107, 157],
        "primary": true
      }
    ],

    "sizes": {
      "system": "WIA_Universal",
      "available": ["XS", "S", "M", "L", "XL"],
      "measurements": {
        "S": {
          "chest": 86,
          "waist": 68,
          "hips": 92,
          "length": 105,
          "shoulder": 38
        },
        "M": {
          "chest": 90,
          "waist": 72,
          "hips": 96,
          "length": 107,
          "shoulder": 40
        },
        "L": {
          "chest": 94,
          "waist": 76,
          "hips": 100,
          "length": 109,
          "shoulder": 42
        }
      },
      "fit": "fitted"
    },

    "assets_3d": {
      "models": [
        {
          "format": "glTF",
          "lod": "high",
          "url": "https://cdn.ecofashion.com/models/dress-12345-high.glb",
          "fileSize": 8500000,
          "polygonCount": 35000,
          "vertexCount": 18000
        },
        {
          "format": "glTF",
          "lod": "medium",
          "url": "https://cdn.ecofashion.com/models/dress-12345-med.glb",
          "fileSize": 2100000,
          "polygonCount": 8000,
          "vertexCount": 4200
        },
        {
          "format": "USD",
          "lod": "high",
          "url": "https://cdn.ecofashion.com/models/dress-12345.usdz",
          "fileSize": 9200000,
          "polygonCount": 35000,
          "vertexCount": 18000
        }
      ],

      "textures": {
        "baseColor": "https://cdn.ecofashion.com/textures/dress-12345-color-4k.png",
        "normal": "https://cdn.ecofashion.com/textures/dress-12345-normal-4k.png",
        "roughness": "https://cdn.ecofashion.com/textures/dress-12345-rough-2k.png",
        "ambientOcclusion": "https://cdn.ecofashion.com/textures/dress-12345-ao-2k.png"
      }
    },

    "physics": {
      "clothType": "cotton",
      "stiffness": 150,
      "damping": 0.15,
      "mass": 0.3,
      "stretch": [1.05, 1.03],
      "friction": 0.4,
      "airResistance": 0.1
    },

    "sustainability": {
      "totalScore": 85,
      "environmentalScore": 82,
      "socialScore": 87,
      "circularScore": 86,

      "carbonFootprint": {
        "material": 0.63,
        "manufacturing": 1.0,
        "transport": 0.03,
        "use": 7.5,
        "endOfLife": -1.0,
        "total": 8.16,
        "perWear": 0.109
      },

      "waterFootprint": {
        "material": 2100,
        "manufacturing": 50,
        "use": 1250,
        "total": 3400
      },

      "certifications": ["GOTS", "Fair Trade", "Carbon Neutral"],
      "recyclability": 0.95,
      "biodegradable": true,
      "repairability": 85,
      "expectedLifespan": 5
    },

    "care": {
      "washing": {
        "temperature": 30,
        "method": "machine",
        "detergent": "mild, eco-friendly",
        "restrictions": ["Do not bleach", "Wash with similar colors"]
      },
      "drying": {
        "method": "line_dry"
      },
      "ironing": {
        "allowed": true,
        "temperature": "medium"
      },
      "dryCleaning": false,
      "bleaching": false
    },

    "pricing": {
      "retailPrice": 89.99,
      "currency": "USD",
      "market": "US",
      "vat": 0,
      "msrp": 89.99
    },

    "blockchain": {
      "enabled": true,
      "network": "polygon",
      "contractAddress": "0x742d35Cc6634C0532925a3b8",
      "tokenId": 12345,
      "materialPassport": "ipfs://QmXxxx.../passport.json"
    },

    "metadata": {
      "createdAt": "2025-12-15T10:00:00Z",
      "updatedAt": "2025-12-20T14:30:00Z",
      "publishedAt": "2026-01-01T00:00:00Z",
      "version": "1.0.0"
    }
  }
}
```

---

## 4.2 Material Database Schema

### 4.2.1 Material Properties Format

```typescript
interface Material {
  // Identification
  id: string;
  code: string;                    // Short code: "ORG_COT"
  name: string;
  category: 'natural' | 'synthetic' | 'semi_synthetic' | 'innovative';
  subcategory: string;

  // Physical properties
  physical: {
    density: number;               // g/cm³
    weightRange: [number, number]; // g/m² [min, max]
    thickness: number;             // mm
    tensileStrength: number;       // MPa
    elongation: number;            // % at break
    abrasionResistance: number;    // 0-100 scale
    tearStrength: number;          // N
    pilling: number;               // 0-5 (0=none, 5=severe)
  };

  // Mechanical properties
  mechanical: {
    stretch: number;               // 1.0-2.0 (factor)
    recovery: number;              // 0-1 (percentage)
    drapeCoefficient: number;      // 0-1 (0=stiff, 1=fluid)
    bendingRigidity: number;       // µN·m
    stiffness: number;             // N/m (for simulation)
    damping: number;               // 0-1
  };

  // Optical properties
  optical: {
    baseRoughness: number;         // 0-1 for PBR
    metallicness: number;          // 0-1 for PBR
    specularIntensity: number;     // 0-1
    translucency: number;          // 0-1
    sheenIntensity?: number;       // For fabrics like satin
    anisotropy?: number;           // Directional reflection
  };

  // Thermal properties
  thermal: {
    insulation: number;            // 0-1 (0=none, 1=excellent)
    breathability: number;         // 0-1
    moistureWicking: number;       // 0-1
    uvProtection: number;          // UPF rating
  };

  // Environmental impact
  sustainability: {
    carbonFactor: number;          // kg CO₂e per kg
    waterUsage: number;            // L per kg
    energyUse: number;             // kWh per kg
    chemicalIntensity: number;     // 0-1 (0=none, 1=high)
    toxicChemicals: string[];      // List of harmful chemicals
    biodegradable: boolean;
    biodegradationTime?: number;   // years
    recyclability: number;         // 0-1
    recycledContent?: number;      // 0-1 (if applicable)
    renewableSource: boolean;
    certifications: string[];
    sustainabilityScore: number;   // 0-100
  };

  // Production information
  production: {
    mainProducers: string[];       // Countries
    averageCost: number;           // $ per kg
    availability: 'abundant' | 'common' | 'limited' | 'rare';
    leadTime: number;              // days
    minimumOrder?: number;         // kg
  };

  // Care requirements
  care: {
    washing: {
      maxTemperature: number;      // °C
      machineWashable: boolean;
      handWashOnly: boolean;
      drycleanRecommended: boolean;
    };
    drying: {
      tumbleDryable: boolean;
      maxTemperature?: number;
      lineDryRecommended: boolean;
    };
    ironing: {
      maxTemperature?: number;
      steamSafe: boolean;
    };
    shrinkage: number;             // % expected
    colorFastness: number;         // 1-5 scale
  };

  // Usage recommendations
  usage: {
    bestFor: string[];             // ["dresses", "shirts", "sportswear"]
    notRecommendedFor: string[];
    seasons: string[];             // ["spring", "summer"]
    occasions: string[];           // ["casual", "formal"]
  };

  // Supply chain
  supplyChain?: {
    origin: string;
    processor: string;
    certifications: string[];
    traceability: boolean;
  };
}
```

### 4.2.2 Example: Organic Cotton Material

```json
{
  "id": "MAT-ORG-COT-001",
  "code": "ORG_COT",
  "name": "Organic Cotton",
  "category": "natural",
  "subcategory": "cellulosic",

  "physical": {
    "density": 1.54,
    "weightRange": [80, 300],
    "thickness": 0.5,
    "tensileStrength": 287,
    "elongation": 7.5,
    "abrasionResistance": 65,
    "tearStrength": 150,
    "pilling": 3
  },

  "mechanical": {
    "stretch": 1.05,
    "recovery": 0.95,
    "drapeCoefficient": 0.65,
    "bendingRigidity": 45,
    "stiffness": 150,
    "damping": 0.15
  },

  "optical": {
    "baseRoughness": 0.6,
    "metallicness": 0.0,
    "specularIntensity": 0.1,
    "translucency": 0.0
  },

  "thermal": {
    "insulation": 0.6,
    "breathability": 0.9,
    "moistureWicking": 0.6,
    "uvProtection": 15
  },

  "sustainability": {
    "carbonFactor": 2.1,
    "waterUsage": 7000,
    "energyUse": 12,
    "chemicalIntensity": 0.2,
    "toxicChemicals": [],
    "biodegradable": true,
    "biodegradationTime": 0.5,
    "recyclability": 0.8,
    "renewableSource": true,
    "certifications": ["GOTS", "USDA Organic", "Fair Trade"],
    "sustainabilityScore": 85
  },

  "production": {
    "mainProducers": ["India", "Turkey", "China", "USA"],
    "averageCost": 3.50,
    "availability": "common",
    "leadTime": 45,
    "minimumOrder": 100
  },

  "care": {
    "washing": {
      "maxTemperature": 60,
      "machineWashable": true,
      "handWashOnly": false,
      "drycleanRecommended": false
    },
    "drying": {
      "tumbleDryable": true,
      "maxTemperature": 70,
      "lineDryRecommended": true
    },
    "ironing": {
      "maxTemperature": 200,
      "steamSafe": true
    },
    "shrinkage": 3,
    "colorFastness": 4
  },

  "usage": {
    "bestFor": ["shirts", "dresses", "casual_wear", "t-shirts", "underwear"],
    "notRecommendedFor": ["outerwear", "swimwear"],
    "seasons": ["spring", "summer", "fall"],
    "occasions": ["casual", "work", "everyday"]
  },

  "supplyChain": {
    "origin": "Tamil Nadu, India",
    "processor": "Organic Textile Mill Ltd.",
    "certifications": ["GOTS", "Fair Trade"],
    "traceability": true
  }
}
```

---

## 4.3 3D Asset Specifications

### 4.3.1 glTF 2.0 Format (Primary)

```typescript
interface GLTFAsset {
  asset: {
    version: '2.0';
    generator: string;             // e.g., "Blender 3.6"
    copyright?: string;
    extensions?: string[];
  };

  scene: number;                   // Default scene index
  scenes: Scene[];
  nodes: Node[];
  meshes: Mesh[];
  materials: Material[];
  textures: Texture[];
  images: Image[];
  animations?: Animation[];

  // WIA-specific extensions
  extensions?: {
    'WIA_fashion_physics': {
      clothType: string;
      stiffness: number;
      damping: number;
      mass: number;
      stretch: [number, number];
    };
    'WIA_fashion_metadata': {
      garmentId: string;
      sustainabilityScore: number;
      materialPassport: string;
    };
  };
}

// Mesh requirements
interface WIAMeshRequirements {
  topology: {
    type: 'quad_dominant';         // Prefer quads over triangles
    manifold: true;                // No holes or non-manifold geometry
    cleanEdges: true;              // No overlapping faces
    uvMapped: true;                // Must have UV coordinates
  };

  polycount: {
    lod0: { min: 20000, max: 50000 };
    lod1: { min: 5000, max: 10000 };
    lod2: { min: 1000, max: 3000 };
    lod3: { min: 500, max: 1000 };
  };

  textures: {
    baseColor: { resolution: '4096x4096', format: 'PNG' | 'JPEG' };
    normal: { resolution: '4096x4096', format: 'PNG' };
    roughness: { resolution: '2048x2048', format: 'PNG' };
    ao: { resolution: '2048x2048', format: 'PNG' };
  };

  // File size limits
  fileSize: {
    lod0: { max: 15_000_000 },     // 15 MB
    lod1: { max: 5_000_000 },      // 5 MB
    lod2: { max: 1_000_000 },      // 1 MB
    lod3: { max: 500_000 }         // 500 KB
  };
}
```

---

## 4.4 NFT Metadata Schema

### 4.4.1 Extended NFT Metadata for Fashion

```typescript
interface WIAFashionNFT {
  // Standard NFT fields (ERC-721/OpenSea)
  name: string;
  description: string;
  image: string;                   // IPFS URL to preview image
  external_url: string;            // Link to external website
  animation_url?: string;          // IPFS URL to 3D model

  // Attributes (for marketplace filters)
  attributes: Array<{
    trait_type: string;
    value: string | number;
    display_type?: 'string' | 'number' | 'boost_percentage' | 'boost_number' | 'date';
    max_value?: number;
  }>;

  // WIA Fashion-specific extension
  wia_fashion: {
    standard: 'WIA-IND-001';
    version: '1.0.0';

    // Digital vs. Phygital
    type: 'digital_only' | 'phygital';

    // Multi-platform 3D models
    models_3d: {
      web: string;                 // glTF for web AR
      decentraland: string;        // GLB
      sandbox: string;             // VXM
      roblox: string;              // RBXM
      vrchat: string;              // FBX
      ios: string;                 // USDZ for AR Quick Look
    };

    // Garment data reference
    garmentData: string;           // IPFS hash to full WIA JSON

    // Sustainability (even for digital!)
    sustainability: {
      digitalOnly: boolean;
      carbonOffset: boolean;       // Is rendering carbon offset?
      sustainableDesign: boolean;
      score: number;               // 0-100
    };

    // Rarity and edition
    rarity: 'common' | 'uncommon' | 'rare' | 'epic' | 'legendary';
    edition: {
      number: number;              // This edition number
      total: number;               // Total editions
      unique: boolean;             // Is it 1 of 1?
    };

    // Unlockable content
    unlockables?: {
      physicalRedemption?: {
        available: boolean;
        oneTime: boolean;
        customSizing: boolean;
        estimatedValue: number;
      };
      arFilter?: string;           // Instagram/Snapchat filter
      designFiles?: string;        // For 3D printing
      exclusiveAccess?: string[];  // Events, communities, etc.
      physicalTwin?: {
        manufacturingCost: number;
        shippingEstimate: number;
        productionTime: number;    // days
      };
    };

    // Creator information
    creator: {
      address: string;             // Wallet address
      name: string;
      portfolio?: string;
      royalties: number;           // Percentage (e.g., 10 for 10%)
    };

    // Licensing
    license: {
      type: 'CC0' | 'CC-BY' | 'CC-BY-NC' | 'CC-BY-SA' | 'All Rights Reserved';
      commercialUse: boolean;
      derivatives: boolean;
      platformRestrictions?: string[];
    };
  };
}
```

### 4.4.2 Example: Digital Fashion NFT

```json
{
  "name": "Digital Couture Dress #42",
  "description": "Exclusive digital dress from the Cyber Elegance collection. Wearable in Decentraland, The Sandbox, and other metaverse platforms.",
  "image": "ipfs://QmXxxx.../preview.png",
  "external_url": "https://digitalfashion.example/dress/42",
  "animation_url": "ipfs://QmYyyy.../dress-42.glb",

  "attributes": [
    { "trait_type": "Designer", "value": "CyberCouture" },
    { "trait_type": "Collection", "value": "Cyber Elegance" },
    { "trait_type": "Rarity", "value": "Legendary" },
    { "trait_type": "Type", "value": "Dress" },
    { "trait_type": "Style", "value": "Futuristic" },
    { "trait_type": "Primary Color", "value": "Holographic Silver" },
    { "trait_type": "Sustainability Score", "value": 100, "max_value": 100 },
    { "trait_type": "Edition", "value": "42 of 100" }
  ],

  "wia_fashion": {
    "standard": "WIA-IND-001",
    "version": "1.0.0",
    "type": "digital_only",

    "models_3d": {
      "web": "ipfs://QmZzzz.../dress-42-web.glb",
      "decentraland": "ipfs://QmWwww.../dress-42-dcl.glb",
      "sandbox": "ipfs://QmQqqq.../dress-42-sandbox.vxm",
      "roblox": "ipfs://QmRrrr.../dress-42-roblox.rbxm",
      "vrchat": "ipfs://QmSsss.../dress-42-vrchat.fbx",
      "ios": "ipfs://QmTttt.../dress-42-ios.usdz"
    },

    "garmentData": "ipfs://QmUuuu.../full-garment-data.json",

    "sustainability": {
      "digitalOnly": true,
      "carbonOffset": true,
      "sustainableDesign": true,
      "score": 100
    },

    "rarity": "legendary",
    "edition": {
      "number": 42,
      "total": 100,
      "unique": false
    },

    "unlockables": {
      "arFilter": "ipfs://QmVvvv.../ar-filter.zip",
      "exclusiveAccess": [
        "Designer Discord Server",
        "Exclusive Metaverse Events",
        "Future Collection Early Access"
      ]
    },

    "creator": {
      "address": "0x742d35Cc6634C0532925a3b8",
      "name": "CyberCouture",
      "portfolio": "https://cybercouture.example",
      "royalties": 10
    },

    "license": {
      "type": "CC-BY-NC",
      "commercialUse": false,
      "derivatives": true,
      "platformRestrictions": []
    }
  }
}
```

---

## Review Questions

1. **What are the four main sections of the WIA Garment schema?**
   <details>
   <summary>Answer</summary>
   Basic information, design details, materials/sizing, and digital assets (3D/2D), plus sustainability and care instructions.
   </details>

2. **What is the recommended polygon count for LOD1 (medium quality) models?**
   <details>
   <summary>Answer</summary>
   5,000-10,000 polygons for medium detail level.
   </details>

3. **Name three sustainability properties tracked in the material schema.**
   <details>
   <summary>Answer</summary>
   Carbon factor (kg CO₂e/kg), water usage (L/kg), recyclability (0-1), biodegradability, certifications, sustainability score.
   </details>

4. **What is the primary 3D format recommended by WIA-IND-001?**
   <details>
   <summary>Answer</summary>
   glTF 2.0 (Graphics Library Transmission Format) for web and AR compatibility.
   </details>

5. **What's the difference between "digital_only" and "phygital" NFT types?**
   <details>
   <summary>Answer</summary>
   Digital-only exists purely in virtual worlds; phygital combines a physical garment with a digital NFT twin.
   </details>

---

## Next Steps

Now that you understand the data formats, let's explore how to interact with this data through [**Chapter 5: API Interface**](05-api-interface.md).

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
