# 4장: 데이터 형식 사양

## 학습 목표

이 장을 마치면 다음을 이해할 수 있습니다:
- WIA 패션 JSON 스키마 구조
- 완전한 메타데이터를 포함한 의류 데이터 형식
- 소재 데이터베이스 스키마 및 속성
- 지속가능성 지표 데이터 형식
- 3D 에셋 사양 및 형식
- NFT 메타데이터 표준

---

## 4.1 의류 교환 형식 (GIF)

핵심 JSON 스키마입니다:

### 4.1.1 완전한 의류 스키마

```typescript
interface WIAFashionGarment {
  // 스키마 메타데이터
  $schema: 'https://wiastandards.com/schemas/fashion/v1.0/garment.json';
  wia_standard: 'WIA-IND-001';
  version: '1.0.0';

  // 고유 식별
  garment: {
    id: string;                    // 고유 의류 ID
    sku: string;                   // 재고 관리 단위
    barcode?: string;              // UPC/EAN 바코드
    nfcTag?: string;               // NFC 칩 ID (피지털용)

    // 기본 정보
    type: 'dress' | 'shirt' | 'pants' | 'jacket' | 'skirt' | 'accessory';
    brand: string;
    name: string;
    description: string;
    season: string;                // 예: "2026년 봄"
    year: number;
    collection?: string;

    // 분류
    gender: 'women' | 'men' | 'unisex' | 'kids';
    category: string;              // 예: "dresses"
    subcategory: string;           // 예: "midi_dress"
    tags: string[];                // 검색 태그

    // 디자인 세부사항
    design: {
      style: string;               // 예: "A-line"
      silhouette: string;
      neckline: string;
      sleeves: string;
      length: string;
      closure: string;
      pockets?: boolean;
      lining?: boolean;
      padding?: boolean;
    };

    // 소재 구성
    materials: Array<{
      type: string;                // 소재 코드
      percentage: number;          // 0-100
      weight_gsm: number;          // 평방미터당 그램
      origin: string;              // 원산지 국가
      certification?: string;      // 예: "GOTS", "Fair Trade"
      sustainability_score: number; // 0-100
      supplier?: string;
      lotNumber?: string;
    }>;

    // 색상 및 패턴
    colors: Array<{
      name: string;
      hex: string;                 // #RRGGBB
      pantone?: string;            // 팬톤 코드
      rgb: [number, number, number];
      cmyk?: [number, number, number, number];
      primary: boolean;            // 기본 색상인가?
    }>;
    patterns?: Array<{
      type: string;                // "floral", "stripe", "solid"
      scale: 'small' | 'medium' | 'large';
      placement: string;
    }>;

    // 사이즈
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

    // 3D 디지털 에셋
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
        baseColor?: string;        // 텍스처 URL
        normal?: string;
        metallic?: string;
        roughness?: string;
        ambientOcclusion?: string;
        emission?: string;
        opacity?: string;
      };

      animations?: Array<{
        name: string;              // 예: "walking", "idle"
        url: string;
        duration: number;          // 초
      }>;
    };

    // 2D 에셋 (이미지)
    assets_2d: {
      images: Array<{
        type: 'product' | 'lifestyle' | 'detail' | 'model' | '360';
        url: string;
        width: number;
        height: number;
        angle?: number;            // 360° 뷰용
        primary?: boolean;
      }>;
      videos?: Array<{
        type: 'product' | 'campaign' | 'howto';
        url: string;
        duration: number;
        thumbnail: string;
      }>;
    };

    // 물리 속성 (시뮬레이션용)
    physics: {
      clothType: 'cotton' | 'silk' | 'denim' | 'leather' | 'knit' | 'synthetic';
      stiffness: number;           // 스프링 상수 (N/m)
      damping: number;             // 0-1
      mass: number;                // kg
      stretch: [number, number];   // [날실, 씨실] 방향
      friction: number;            // 표면 마찰 0-1
      airResistance: number;       // 0-1
    };

    // 지속가능성 지표
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
        material: number;          // 리터
        manufacturing: number;
        use: number;
        total: number;
      };

      certifications: string[];    // ["GOTS", "Fair Trade", "B-Corp"]
      recyclability: number;       // 0-1
      biodegradable: boolean;
      repairability: number;       // 0-100
      expectedLifespan: number;    // 년
    };

    // 관리 지침
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

    // 가격
    pricing: {
      retailPrice: number;
      currency: string;
      market: string;              // 예: "US", "EU"
      vat?: number;
      discountPrice?: number;
      costPrice?: number;          // 도매
      msrp?: number;               // 제조업체 권장
    };

    // 공급망 및 추적성
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

    // 블록체인 통합
    blockchain?: {
      enabled: boolean;
      network: 'ethereum' | 'polygon' | 'solana' | 'other';
      contractAddress: string;
      tokenId?: number;
      materialPassport?: string;   // IPFS 해시
      nftMetadata?: string;        // IPFS 해시
    };

    // 메타데이터 타임스탬프
    metadata: {
      createdAt: string;           // ISO 8601
      updatedAt: string;
      publishedAt?: string;
      deprecatedAt?: string;
      version: string;             // 시맨틱 버전 관리
    };
  };
}
```

### 4.1.2 예제: 완전한 드레스 데이터

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
    "name": "코랄 여름 드레스",
    "description": "유기농 면의 지속가능한 A라인 미디 드레스, 플러터 슬리브",
    "season": "2026년 봄/여름",
    "year": 2026,
    "collection": "지속가능한 우아함",

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
        "name": "코랄 핑크",
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
        "restrictions": ["표백 금지", "유사 색상과 함께 세탁"]
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

## 4.2 소재 데이터베이스 스키마

### 4.2.1 소재 속성 형식

```typescript
interface Material {
  // 식별
  id: string;
  code: string;                    // 짧은 코드: "ORG_COT"
  name: string;
  category: 'natural' | 'synthetic' | 'semi_synthetic' | 'innovative';
  subcategory: string;

  // 물리적 속성
  physical: {
    density: number;               // g/cm³
    weightRange: [number, number]; // g/m² [최소, 최대]
    thickness: number;             // mm
    tensileStrength: number;       // MPa
    elongation: number;            // 파단시 %
    abrasionResistance: number;    // 0-100 척도
    tearStrength: number;          // N
    pilling: number;               // 0-5 (0=없음, 5=심각)
  };

  // 기계적 속성
  mechanical: {
    stretch: number;               // 1.0-2.0 (계수)
    recovery: number;              // 0-1 (백분율)
    drapeCoefficient: number;      // 0-1 (0=뻣뻣함, 1=유동적)
    bendingRigidity: number;       // µN·m
    stiffness: number;             // N/m (시뮬레이션용)
    damping: number;               // 0-1
  };

  // 광학적 속성
  optical: {
    baseRoughness: number;         // 0-1 PBR용
    metallicness: number;          // 0-1 PBR용
    specularIntensity: number;     // 0-1
    translucency: number;          // 0-1
    sheenIntensity?: number;       // 새틴 같은 원단용
    anisotropy?: number;           // 방향성 반사
  };

  // 열적 속성
  thermal: {
    insulation: number;            // 0-1 (0=없음, 1=우수)
    breathability: number;         // 0-1
    moistureWicking: number;       // 0-1
    uvProtection: number;          // UPF 등급
  };

  // 환경 영향
  sustainability: {
    carbonFactor: number;          // kg CO₂e per kg
    waterUsage: number;            // L per kg
    energyUse: number;             // kWh per kg
    chemicalIntensity: number;     // 0-1 (0=없음, 1=높음)
    toxicChemicals: string[];      // 유해 화학물질 목록
    biodegradable: boolean;
    biodegradationTime?: number;   // 년
    recyclability: number;         // 0-1
    recycledContent?: number;      // 0-1 (해당되는 경우)
    renewableSource: boolean;
    certifications: string[];
    sustainabilityScore: number;   // 0-100
  };

  // 생산 정보
  production: {
    mainProducers: string[];       // 국가
    averageCost: number;           // $ per kg
    availability: 'abundant' | 'common' | 'limited' | 'rare';
    leadTime: number;              // 일
    minimumOrder?: number;         // kg
  };

  // 관리 요구사항
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
    shrinkage: number;             // 예상 %
    colorFastness: number;         // 1-5 척도
  };

  // 사용 권장사항
  usage: {
    bestFor: string[];             // ["dresses", "shirts", "sportswear"]
    notRecommendedFor: string[];
    seasons: string[];             // ["spring", "summer"]
    occasions: string[];           // ["casual", "formal"]
  };

  // 공급망
  supplyChain?: {
    origin: string;
    processor: string;
    certifications: string[];
    traceability: boolean;
  };
}
```

### 4.2.2 예제: 유기농 면 소재

```json
{
  "id": "MAT-ORG-COT-001",
  "code": "ORG_COT",
  "name": "유기농 면",
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
    "mainProducers": ["인도", "터키", "중국", "미국"],
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
    "origin": "타밀나두, 인도",
    "processor": "Organic Textile Mill Ltd.",
    "certifications": ["GOTS", "Fair Trade"],
    "traceability": true
  }
}
```

---

## 4.3 3D 에셋 사양

### 4.3.1 glTF 2.0 형식 (기본)

```typescript
interface GLTFAsset {
  asset: {
    version: '2.0';
    generator: string;             // 예: "Blender 3.6"
    copyright?: string;
    extensions?: string[];
  };

  scene: number;                   // 기본 장면 인덱스
  scenes: Scene[];
  nodes: Node[];
  meshes: Mesh[];
  materials: Material[];
  textures: Texture[];
  images: Image[];
  animations?: Animation[];

  // WIA 특정 확장
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

// 메시 요구사항
interface WIAMeshRequirements {
  topology: {
    type: 'quad_dominant';         // 삼각형보다 사각형 선호
    manifold: true;                // 구멍이나 비다양체 형상 없음
    cleanEdges: true;              // 겹치는 면 없음
    uvMapped: true;                // UV 좌표 필수
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

  // 파일 크기 제한
  fileSize: {
    lod0: { max: 15_000_000 },     // 15 MB
    lod1: { max: 5_000_000 },      // 5 MB
    lod2: { max: 1_000_000 },      // 1 MB
    lod3: { max: 500_000 }         // 500 KB
  };
}
```

---

## 4.4 NFT 메타데이터 스키마

### 4.4.1 패션용 확장 NFT 메타데이터

```typescript
interface WIAFashionNFT {
  // 표준 NFT 필드 (ERC-721/OpenSea)
  name: string;
  description: string;
  image: string;                   // 미리보기 이미지 IPFS URL
  external_url: string;            // 외부 웹사이트 링크
  animation_url?: string;          // 3D 모델 IPFS URL

  // 속성 (마켓플레이스 필터용)
  attributes: Array<{
    trait_type: string;
    value: string | number;
    display_type?: 'string' | 'number' | 'boost_percentage' | 'boost_number' | 'date';
    max_value?: number;
  }>;

  // WIA 패션 특정 확장
  wia_fashion: {
    standard: 'WIA-IND-001';
    version: '1.0.0';

    // 디지털 vs. 피지털
    type: 'digital_only' | 'phygital';

    // 다중 플랫폼 3D 모델
    models_3d: {
      web: string;                 // 웹 AR용 glTF
      decentraland: string;        // GLB
      sandbox: string;             // VXM
      roblox: string;              // RBXM
      vrchat: string;              // FBX
      ios: string;                 // AR Quick Look용 USDZ
    };

    // 의류 데이터 참조
    garmentData: string;           // 전체 WIA JSON의 IPFS 해시

    // 지속가능성 (디지털에도!)
    sustainability: {
      digitalOnly: boolean;
      carbonOffset: boolean;       // 렌더링 탄소가 상쇄되었나?
      sustainableDesign: boolean;
      score: number;               // 0-100
    };

    // 희귀도 및 에디션
    rarity: 'common' | 'uncommon' | 'rare' | 'epic' | 'legendary';
    edition: {
      number: number;              // 이 에디션 번호
      total: number;               // 총 에디션
      unique: boolean;             // 1/1인가?
    };

    // 잠금 해제 가능한 콘텐츠
    unlockables?: {
      physicalRedemption?: {
        available: boolean;
        oneTime: boolean;
        customSizing: boolean;
        estimatedValue: number;
      };
      arFilter?: string;           // Instagram/Snapchat 필터
      designFiles?: string;        // 3D 프린팅용
      exclusiveAccess?: string[];  // 이벤트, 커뮤니티 등
      physicalTwin?: {
        manufacturingCost: number;
        shippingEstimate: number;
        productionTime: number;    // 일
      };
    };

    // 크리에이터 정보
    creator: {
      address: string;             // 지갑 주소
      name: string;
      portfolio?: string;
      royalties: number;           // 백분율 (예: 10%의 경우 10)
    };

    // 라이선스
    license: {
      type: 'CC0' | 'CC-BY' | 'CC-BY-NC' | 'CC-BY-SA' | 'All Rights Reserved';
      commercialUse: boolean;
      derivatives: boolean;
      platformRestrictions?: string[];
    };
  };
}
```

### 4.4.2 예제: 디지털 패션 NFT

```json
{
  "name": "디지털 쿠튀르 드레스 #42",
  "description": "사이버 우아함 컬렉션의 독점 디지털 드레스. Decentraland, The Sandbox 및 기타 메타버스 플랫폼에서 착용 가능.",
  "image": "ipfs://QmXxxx.../preview.png",
  "external_url": "https://digitalfashion.example/dress/42",
  "animation_url": "ipfs://QmYyyy.../dress-42.glb",

  "attributes": [
    { "trait_type": "디자이너", "value": "CyberCouture" },
    { "trait_type": "컬렉션", "value": "사이버 우아함" },
    { "trait_type": "희귀도", "value": "전설적" },
    { "trait_type": "유형", "value": "드레스" },
    { "trait_type": "스타일", "value": "미래적" },
    { "trait_type": "기본 색상", "value": "홀로그래픽 실버" },
    { "trait_type": "지속가능성 점수", "value": 100, "max_value": 100 },
    { "trait_type": "에디션", "value": "100개 중 42" }
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
        "디자이너 Discord 서버",
        "독점 메타버스 이벤트",
        "향후 컬렉션 조기 액세스"
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

## 복습 질문

1. **WIA 의류 스키마의 네 가지 주요 섹션은 무엇인가?**
   <details>
   <summary>답변</summary>
   기본 정보, 디자인 세부사항, 소재/사이징, 디지털 에셋(3D/2D), 그리고 지속가능성 및 관리 지침.
   </details>

2. **LOD1(중간 품질) 모델의 권장 폴리곤 수는?**
   <details>
   <summary>답변</summary>
   중간 디테일 레벨의 경우 5,000-10,000 폴리곤.
   </details>

3. **소재 스키마에서 추적되는 세 가지 지속가능성 속성을 말하시오.**
   <details>
   <summary>답변</summary>
   탄소 계수(kg CO₂e/kg), 물 사용량(L/kg), 재활용성(0-1), 생분해성, 인증, 지속가능성 점수.
   </details>

4. **WIA-IND-001에서 권장하는 기본 3D 형식은 무엇인가?**
   <details>
   <summary>답변</summary>
   웹 및 AR 호환성을 위한 glTF 2.0(Graphics Library Transmission Format).
   </details>

5. **"digital_only"와 "phygital" NFT 유형의 차이는 무엇인가?**
   <details>
   <summary>답변</summary>
   Digital-only는 순수하게 가상 세계에만 존재하며, phygital은 물리적 의류와 디지털 NFT 트윈을 결합합니다.
   </details>

---

## 다음 단계

이제 데이터 형식을 이해했으니, [**5장: API 인터페이스**](05-api-interface.md)에서 이 데이터와 상호작용하는 방법을 알아보겠습니다.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
