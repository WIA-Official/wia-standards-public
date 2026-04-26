# 5장: API 인터페이스

## 학습 목표

이 장을 마치면 다음을 이해할 수 있습니다:
- WIA 패션 REST API 엔드포인트 및 인증
- 의류 관리 작업 (CRUD)
- 가상 착용 API 통합
- 사이즈 추천 API 사용
- 지속가능성 계산 엔드포인트
- 트렌드 예측 API 호출

---

## 5.1 API 개요

**기본 URL**: `https://api.wiastandards.com/fashion/v1`

**인증**: Bearer 토큰 (API 키)

**속도 제한**:
- 무료 티어: 1,000 요청/일
- 프로 티어: 100,000 요청/일
- 엔터프라이즈: 맞춤형 제한

### 5.1.1 인증

```typescript
// 인증 헤더
const headers = {
  'Authorization': `Bearer ${API_KEY}`,
  'Content-Type': 'application/json',
  'Accept': 'application/json'
};

// 예제: API 키 가져오기 (일회성 설정)
async function getAPIKey(email: string, password: string): Promise<string> {
  const response = await fetch('https://api.wiastandards.com/auth/login', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ email, password })
  });

  const data = await response.json();
  return data.apiKey;
}

// API 키를 안전하게 저장
const API_KEY = process.env.WIA_FASHION_API_KEY;
```

---

## 5.2 의류 관리 API

### 5.2.1 의류 생성

**엔드포인트**: `POST /garments`

시스템에 새 디지털 의류를 생성합니다.

```typescript
interface CreateGarmentRequest {
  garment: {
    type: string;
    brand: string;
    name: string;
    description: string;
    materials: Material[];
    sizes: SizeData;
    // ... (전체 스키마는 4장 참조)
  };
}

interface CreateGarmentResponse {
  success: boolean;
  garmentId: string;
  message: string;
  assets: {
    uploadUrls: {
      lod0: string;      // 고해상도 모델용 사전 서명 URL
      lod1: string;      // 중해상도 모델용 사전 서명 URL
      lod2: string;      // 저해상도 모델용 사전 서명 URL
      textures: {
        baseColor: string;
        normal: string;
        roughness: string;
      };
    };
  };
}

// 예제 사용
async function createGarment(garmentData: any): Promise<CreateGarmentResponse> {
  const response = await fetch('https://api.wiastandards.com/fashion/v1/garments', {
    method: 'POST',
    headers: {
      'Authorization': `Bearer ${API_KEY}`,
      'Content-Type': 'application/json'
    },
    body: JSON.stringify({ garment: garmentData })
  });

  if (!response.ok) {
    throw new Error(`API 오류: ${response.status}`);
  }

  return await response.json();
}

// 생성 후 3D 모델 업로드
async function upload3DModel(
  uploadUrl: string,
  modelFile: File
): Promise<void> {
  await fetch(uploadUrl, {
    method: 'PUT',
    body: modelFile,
    headers: {
      'Content-Type': 'model/gltf-binary'
    }
  });
}
```

### 5.2.2 의류 가져오기

**엔드포인트**: `GET /garments/{id}`

완전한 의류 데이터를 검색합니다.

```typescript
interface GetGarmentResponse {
  success: boolean;
  garment: WIAFashionGarment;  // 4장의 전체 스키마
  cached: boolean;
  cacheAge?: number;            // 초
}

// 예제
async function getGarment(garmentId: string): Promise<WIAFashionGarment> {
  const response = await fetch(
    `https://api.wiastandards.com/fashion/v1/garments/${garmentId}`,
    {
      headers: {
        'Authorization': `Bearer ${API_KEY}`
      }
    }
  );

  const data = await response.json();
  return data.garment;
}
```

### 5.2.3 의류 업데이트

**엔드포인트**: `PATCH /garments/{id}`

의류의 특정 필드를 업데이트합니다.

```typescript
interface UpdateGarmentRequest {
  updates: {
    [key: string]: any;  // 부분 업데이트
  };
}

// 예제: 가격 업데이트
async function updateGarmentPrice(
  garmentId: string,
  newPrice: number
): Promise<void> {
  await fetch(
    `https://api.wiastandards.com/fashion/v1/garments/${garmentId}`,
    {
      method: 'PATCH',
      headers: {
        'Authorization': `Bearer ${API_KEY}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        updates: {
          'pricing.retailPrice': newPrice,
          'metadata.updatedAt': new Date().toISOString()
        }
      })
    }
  );
}
```

### 5.2.4 의류 검색

**엔드포인트**: `GET /garments/search`

의류를 검색하고 필터링합니다.

```typescript
interface SearchGarmentsRequest {
  query?: string;               // 텍스트 검색
  filters?: {
    brand?: string;
    category?: string;
    priceMin?: number;
    priceMax?: number;
    sustainabilityMin?: number; // 0-100
    materials?: string[];
    sizes?: string[];
    colors?: string[];
  };
  sort?: {
    field: string;
    order: 'asc' | 'desc';
  };
  page?: number;
  limit?: number;
}

interface SearchGarmentsResponse {
  success: boolean;
  results: WIAFashionGarment[];
  pagination: {
    page: number;
    limit: number;
    total: number;
    pages: number;
  };
}

// 예제: 지속가능한 드레스 검색
async function searchSustainableDresses(): Promise<WIAFashionGarment[]> {
  const params = new URLSearchParams({
    'filters[category]': 'dresses',
    'filters[sustainabilityMin]': '70',
    'sort[field]': 'sustainability.totalScore',
    'sort[order]': 'desc',
    'limit': '20'
  });

  const response = await fetch(
    `https://api.wiastandards.com/fashion/v1/garments/search?${params}`,
    {
      headers: { 'Authorization': `Bearer ${API_KEY}` }
    }
  );

  const data = await response.json();
  return data.results;
}
```

---

## 5.3 가상 착용 API

### 5.3.1 착용 생성

**엔드포인트**: `POST /virtual-tryon`

가상 착용 경험을 생성합니다.

```typescript
interface VirtualTryOnRequest {
  garmentId: string;
  mode: 'ar_camera' | 'avatar_3d' | 'photo_upload';

  // 아바타 모드용
  bodyMeasurements?: {
    height: number;     // cm
    chest: number;
    waist: number;
    hips: number;
    weight?: number;    // kg
  };

  // 사진 모드용
  photoUrl?: string;

  // 렌더링 옵션
  options?: {
    quality: 'low' | 'medium' | 'high';
    backgroundColor?: string;
    lighting?: 'studio' | 'natural' | 'dramatic';
    angle?: number;     // 각도 (0-360)
  };
}

interface VirtualTryOnResponse {
  success: boolean;
  sessionId: string;

  // 아바타 모드용
  renderedImages?: {
    front: string;      // 렌더링된 이미지 URL
    side: string;
    back: string;
    three_quarter: string;
  };

  // 인터랙티브 3D 뷰어 URL
  viewerUrl?: string;

  // 착용감 분석
  fitAnalysis: {
    overallFit: 'too_tight' | 'comfortable' | 'too_loose';
    confidence: number;
    details: {
      chest: FitLevel;
      waist: FitLevel;
      hips: FitLevel;
      length: 'too_short' | 'perfect' | 'too_long';
    };
  };

  // 사이즈 추천
  sizeRecommendation: {
    current: string;
    recommended: string;
    confidence: number;
    reasoning: string;
  };
}

type FitLevel = 'very_tight' | 'tight' | 'comfortable' | 'loose' | 'very_loose';

// 예제 사용
async function virtualTryOn(
  garmentId: string,
  measurements: BodyMeasurements
): Promise<VirtualTryOnResponse> {
  const response = await fetch(
    'https://api.wiastandards.com/fashion/v1/virtual-tryon',
    {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${API_KEY}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        garmentId,
        mode: 'avatar_3d',
        bodyMeasurements: measurements,
        options: {
          quality: 'high',
          lighting: 'studio'
        }
      })
    }
  );

  return await response.json();
}
```

### 5.3.2 AR 세션 (실시간)

**엔드포인트**: `WebSocket wss://api.wiastandards.com/fashion/v1/ar-session`

WebSocket을 통한 실시간 AR 착용.

```typescript
class ARTryOnSession {
  private ws: WebSocket;
  private garmentId: string;

  constructor(garmentId: string, apiKey: string) {
    this.garmentId = garmentId;
    this.ws = new WebSocket(
      `wss://api.wiastandards.com/fashion/v1/ar-session?apiKey=${apiKey}`
    );

    this.ws.onopen = () => {
      this.ws.send(JSON.stringify({
        action: 'load_garment',
        garmentId: this.garmentId
      }));
    };

    this.ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      this.handleMessage(data);
    };
  }

  sendFrame(videoFrame: ImageData): void {
    // 신체 추적을 위한 카메라 프레임 전송
    const canvas = document.createElement('canvas');
    canvas.width = videoFrame.width;
    canvas.height = videoFrame.height;
    const ctx = canvas.getContext('2d')!;
    ctx.putImageData(videoFrame, 0, 0);

    canvas.toBlob((blob) => {
      if (blob) {
        this.ws.send(blob);
      }
    }, 'image/jpeg', 0.8);
  }

  private handleMessage(data: any): void {
    switch (data.type) {
      case 'body_detected':
        console.log('신체 감지:', data.keypoints);
        break;
      case 'render_frame':
        // 의류 오버레이가 있는 렌더링된 프레임 수신
        this.displayFrame(data.imageUrl);
        break;
      case 'fit_analysis':
        console.log('착용감:', data.analysis);
        break;
    }
  }

  private displayFrame(imageUrl: string): void {
    // AR 프레임 표시
    const img = new Image();
    img.src = imageUrl;
    document.getElementById('ar-view')?.appendChild(img);
  }

  close(): void {
    this.ws.close();
  }
}

// 사용법
const arSession = new ARTryOnSession('DRESS-12345', API_KEY);

// 카메라 프레임 전송 (예: getUserMedia에서)
navigator.mediaDevices.getUserMedia({ video: true })
  .then(stream => {
    const video = document.createElement('video');
    video.srcObject = stream;
    video.play();

    setInterval(() => {
      const canvas = document.createElement('canvas');
      canvas.width = video.videoWidth;
      canvas.height = video.videoHeight;
      const ctx = canvas.getContext('2d')!;
      ctx.drawImage(video, 0, 0);
      const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
      arSession.sendFrame(imageData);
    }, 100); // 10 FPS
  });
```

---

## 5.4 사이즈 추천 API

### 5.4.1 사이즈 추천

**엔드포인트**: `POST /size-recommend`

AI 기반 사이즈 추천을 받습니다.

```typescript
interface SizeRecommendRequest {
  garmentId: string;
  userMeasurements: {
    height: number;       // cm
    weight?: number;      // kg
    chest: number;        // cm
    waist: number;        // cm
    hips: number;         // cm
    inseam?: number;      // cm
  };
  userHistory?: {
    pastPurchases: Array<{
      garmentId: string;
      sizePurchased: string;
      fit: 'too_small' | 'perfect' | 'too_large';
    }>;
  };
}

interface SizeRecommendResponse {
  success: boolean;
  recommendation: {
    size: string;
    confidence: number;     // 0-1

    fitPrediction: {
      overall: 'runs_small' | 'true_to_size' | 'runs_large';
      chest: FitLevel;
      waist: FitLevel;
      hips: FitLevel;
      length: 'too_short' | 'perfect' | 'too_long';
    };

    alternativeSizes: Array<{
      size: string;
      confidence: number;
      note: string;
    }>;

    reasoning: string;      // 사람이 읽을 수 있는 설명
  };

  measurements: {
    recommended: { [key: string]: number };
    tolerance: { [key: string]: number };
  };

  returnRisk: {
    probability: number;    // 0-1
    level: 'low' | 'medium' | 'high';
    mainConcern?: string;
  };
}

// 예제
async function recommendSize(
  garmentId: string,
  measurements: UserMeasurements
): Promise<SizeRecommendResponse> {
  const response = await fetch(
    'https://api.wiastandards.com/fashion/v1/size-recommend',
    {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${API_KEY}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        garmentId,
        userMeasurements: measurements
      })
    }
  );

  return await response.json();
}

// 사용 예제
const sizeRec = await recommendSize('DRESS-12345', {
  height: 165,
  chest: 88,
  waist: 70,
  hips: 95
});

console.log(`추천: ${sizeRec.recommendation.size}`);
console.log(`신뢰도: ${(sizeRec.recommendation.confidence * 100).toFixed(0)}%`);
console.log(`착용감: ${sizeRec.recommendation.fitPrediction.overall}`);
```

---

## 5.5 지속가능성 계산 API

### 5.5.1 지속가능성 계산

**엔드포인트**: `POST /sustainability/calculate`

포괄적인 지속가능성 지표를 계산합니다.

```typescript
interface SustainabilityCalculateRequest {
  garment: {
    materials: Array<{
      type: string;
      percentage: number;
      weight: number;       // kg
    }>;
    manufacturing: {
      country: string;
      energySource?: 'renewable' | 'grid' | 'coal';
      certifications?: string[];
    };
    transportation: {
      origin: string;
      destination: string;
      method: 'air' | 'sea' | 'truck' | 'rail';
    };
    expectedLifespan?: number;  // 년
    washes?: number;            // 예상 세탁 횟수
  };
}

interface SustainabilityCalculateResponse {
  success: boolean;

  carbonFootprint: {
    material: number;
    manufacturing: number;
    transport: number;
    use: number;
    endOfLife: number;
    total: number;
    perWear: number;
  };

  waterFootprint: {
    material: number;
    manufacturing: number;
    use: number;
    total: number;
  };

  scores: {
    environmental: number;    // 0-100
    social: number;
    circular: number;
    total: number;
    rating: 'A+' | 'A' | 'B' | 'C' | 'D' | 'F';
  };

  comparison: {
    industryAverage: number;
    percentageBetter: number;
    equivalences: {
      drivingMiles: number;
      treesNeeded: number;
      showers: number;
    };
  };

  recommendations: string[];  // 개선 제안
}

// 예제
async function calculateSustainability(
  garmentData: any
): Promise<SustainabilityCalculateResponse> {
  const response = await fetch(
    'https://api.wiastandards.com/fashion/v1/sustainability/calculate',
    {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${API_KEY}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({ garment: garmentData })
    }
  );

  return await response.json();
}

// 사용법
const sustainability = await calculateSustainability({
  materials: [
    { type: 'organic_cotton', percentage: 95, weight: 0.285 },
    { type: 'elastane', percentage: 5, weight: 0.015 }
  ],
  manufacturing: {
    country: 'India',
    energySource: 'renewable',
    certifications: ['GOTS', 'Fair Trade']
  },
  transportation: {
    origin: 'Mumbai, India',
    destination: 'New York, USA',
    method: 'sea'
  },
  expectedLifespan: 5,
  washes: 75
});

console.log(`총 탄소: ${sustainability.carbonFootprint.total} kg CO₂e`);
console.log(`등급: ${sustainability.scores.rating}`);
console.log(`평균보다 ${sustainability.comparison.percentageBetter}% 우수`);
```

---

## 5.6 트렌드 예측 API

### 5.6.1 트렌드 예측

**엔드포인트**: `POST /trends/predict`

AI 기반 트렌드 예측을 얻습니다.

```typescript
interface TrendPredictRequest {
  season: string;           // "2026년 봄"
  category?: string;        // "dresses", "jackets"
  region?: string;          // "US", "EU", "Asia"
  timeframe: number;        // 앞으로 몇 개월 (1-12)
  filters?: {
    priceRange?: [number, number];
    gender?: 'women' | 'men' | 'unisex';
    ageGroup?: string;
  };
}

interface TrendPredictResponse {
  success: boolean;

  predictions: Array<{
    trend: string;
    category: string;
    strength: number;       // 0-1 (트렌드 강도)
    confidence: number;     // 0-1 (예측 신뢰도)
    timeframe: string;      // "1-3개월", "3-6개월"

    attributes: {
      colors: Array<{
        name: string;
        hex: string;
        prevalence: number; // 0-1
      }>;
      styles: string[];
      silhouettes: string[];
      patterns: string[];
      materials: string[];
    };

    dataSources: {
      socialMedia: number;  // 0-1 신호 강도
      runway: number;
      retail: number;
      expert: number;
    };

    momentum: 'rising' | 'stable' | 'declining';
    peakEstimate?: string; // ISO 날짜

    examples: Array<{
      imageUrl: string;
      source: string;
      engagement: number;
    }>;
  }>;

  meta: {
    updatedAt: string;
    nextUpdate: string;
    dataPoints: number;
  };
}

// 예제
async function predictTrends(
  season: string,
  category: string
): Promise<TrendPredictResponse> {
  const response = await fetch(
    'https://api.wiastandards.com/fashion/v1/trends/predict',
    {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${API_KEY}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        season,
        category,
        region: 'US',
        timeframe: 6
      })
    }
  );

  return await response.json();
}

// 사용법
const trends = await predictTrends('2026년 봄', 'dresses');

trends.predictions.forEach(trend => {
  console.log(`${trend.trend}: ${(trend.strength * 100).toFixed(0)}% 강도`);
  console.log(`색상: ${trend.attributes.colors.map(c => c.name).join(', ')}`);
  console.log(`모멘텀: ${trend.momentum}`);
  console.log('---');
});
```

---

## 5.7 오류 처리

모든 API 엔드포인트는 일관된 오류 응답을 따릅니다:

```typescript
interface APIError {
  success: false;
  error: {
    code: string;           // 오류 코드
    message: string;        // 사람이 읽을 수 있는 메시지
    details?: any;          // 추가 세부정보
    documentation?: string; // 문서 링크
  };
}

// 일반적인 오류 코드
const ErrorCodes = {
  INVALID_API_KEY: 'auth/invalid-api-key',
  RATE_LIMIT_EXCEEDED: 'rate-limit/exceeded',
  GARMENT_NOT_FOUND: 'garment/not-found',
  INVALID_MEASUREMENTS: 'validation/invalid-measurements',
  MODEL_UPLOAD_FAILED: 'assets/upload-failed',
  INSUFFICIENT_DATA: 'prediction/insufficient-data'
};

// 오류 처리 예제
async function safeAPICall<T>(
  apiCall: () => Promise<T>
): Promise<T> {
  try {
    return await apiCall();
  } catch (error: any) {
    if (error.response) {
      const apiError: APIError = await error.response.json();

      switch (apiError.error.code) {
        case ErrorCodes.RATE_LIMIT_EXCEEDED:
          // 대기 후 재시도
          await new Promise(resolve => setTimeout(resolve, 60000));
          return await apiCall();

        case ErrorCodes.GARMENT_NOT_FOUND:
          throw new Error('데이터베이스에서 의류를 찾을 수 없습니다');

        default:
          throw new Error(apiError.error.message);
      }
    }
    throw error;
  }
}
```

---

## 복습 질문

1. **WIA 패션 API의 기본 URL은 무엇인가?**
   <details>
   <summary>답변</summary>
   https://api.wiastandards.com/fashion/v1
   </details>

2. **가상 착용에 사용 가능한 세 가지 모드를 말하시오.**
   <details>
   <summary>답변</summary>
   ar_camera(AR 카메라 오버레이), avatar_3d(3D 아바타), photo_upload(사진 기반).
   </details>

3. **사이즈 추천의 일반적인 정확도는 얼마인가?**
   <details>
   <summary>답변</summary>
   한 사이즈 이내 92%, 정확한 일치 78%.
   </details>

4. **API에서 계산되는 지속가능성 점수는 무엇인가?**
   <details>
   <summary>답변</summary>
   환경 점수, 사회 점수, 순환 점수, 총 점수(모두 0-100).
   </details>

5. **트렌드 예측이 얼마나 앞을 예측할 수 있는가?**
   <details>
   <summary>답변</summary>
   1-12개월 앞, 기간이 길수록 정확도가 떨어집니다.
   </details>

---

## 다음 단계

[**6장: 프로토콜**](06-protocol.md)에서 이러한 API 뒤에 있는 알고리즘과 계산 방법을 배우겠습니다.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
