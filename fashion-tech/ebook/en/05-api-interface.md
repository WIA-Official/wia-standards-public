# Chapter 5: API Interface

## Learning Objectives

By the end of this chapter, you will understand:
- WIA Fashion REST API endpoints and authentication
- Garment management operations (CRUD)
- Virtual try-on API integration
- Size recommendation API usage
- Sustainability calculation endpoints
- Trend prediction API calls

---

## 5.1 API Overview

**Base URL**: `https://api.wiastandards.com/fashion/v1`

**Authentication**: Bearer token (API key)

**Rate Limits**:
- Free tier: 1,000 requests/day
- Pro tier: 100,000 requests/day
- Enterprise: Custom limits

### 5.1.1 Authentication

```typescript
// Authentication header
const headers = {
  'Authorization': `Bearer ${API_KEY}`,
  'Content-Type': 'application/json',
  'Accept': 'application/json'
};

// Example: Get API key (one-time setup)
async function getAPIKey(email: string, password: string): Promise<string> {
  const response = await fetch('https://api.wiastandards.com/auth/login', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ email, password })
  });

  const data = await response.json();
  return data.apiKey;
}

// Store API key securely
const API_KEY = process.env.WIA_FASHION_API_KEY;
```

---

## 5.2 Garment Management API

### 5.2.1 Create Garment

**Endpoint**: `POST /garments`

Creates a new digital garment in the system.

```typescript
interface CreateGarmentRequest {
  garment: {
    type: string;
    brand: string;
    name: string;
    description: string;
    materials: Material[];
    sizes: SizeData;
    // ... (see Chapter 4 for full schema)
  };
}

interface CreateGarmentResponse {
  success: boolean;
  garmentId: string;
  message: string;
  assets: {
    uploadUrls: {
      lod0: string;      // Pre-signed URL for high-res model
      lod1: string;      // Pre-signed URL for medium-res model
      lod2: string;      // Pre-signed URL for low-res model
      textures: {
        baseColor: string;
        normal: string;
        roughness: string;
      };
    };
  };
}

// Example usage
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
    throw new Error(`API Error: ${response.status}`);
  }

  return await response.json();
}

// Upload 3D model after creation
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

### 5.2.2 Get Garment

**Endpoint**: `GET /garments/{id}`

Retrieves complete garment data.

```typescript
interface GetGarmentResponse {
  success: boolean;
  garment: WIAFashionGarment;  // Full schema from Chapter 4
  cached: boolean;
  cacheAge?: number;            // seconds
}

// Example
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

### 5.2.3 Update Garment

**Endpoint**: `PATCH /garments/{id}`

Updates specific fields of a garment.

```typescript
interface UpdateGarmentRequest {
  updates: {
    [key: string]: any;  // Partial updates
  };
}

// Example: Update pricing
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

### 5.2.4 Search Garments

**Endpoint**: `GET /garments/search`

Search and filter garments.

```typescript
interface SearchGarmentsRequest {
  query?: string;               // Text search
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

// Example: Search sustainable dresses
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

## 5.3 Virtual Try-On API

### 5.3.1 Generate Try-On

**Endpoint**: `POST /virtual-tryon`

Creates a virtual try-on experience.

```typescript
interface VirtualTryOnRequest {
  garmentId: string;
  mode: 'ar_camera' | 'avatar_3d' | 'photo_upload';

  // For avatar mode
  bodyMeasurements?: {
    height: number;     // cm
    chest: number;
    waist: number;
    hips: number;
    weight?: number;    // kg
  };

  // For photo mode
  photoUrl?: string;

  // Rendering options
  options?: {
    quality: 'low' | 'medium' | 'high';
    backgroundColor?: string;
    lighting?: 'studio' | 'natural' | 'dramatic';
    angle?: number;     // degrees (0-360)
  };
}

interface VirtualTryOnResponse {
  success: boolean;
  sessionId: string;

  // For avatar mode
  renderedImages?: {
    front: string;      // URL to rendered image
    side: string;
    back: string;
    three_quarter: string;
  };

  // Interactive 3D viewer URL
  viewerUrl?: string;

  // Fit analysis
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

  // Size recommendation
  sizeRecommendation: {
    current: string;
    recommended: string;
    confidence: number;
    reasoning: string;
  };
}

type FitLevel = 'very_tight' | 'tight' | 'comfortable' | 'loose' | 'very_loose';

// Example usage
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

### 5.3.2 AR Session (Real-time)

**Endpoint**: `WebSocket wss://api.wiastandards.com/fashion/v1/ar-session`

Real-time AR try-on via WebSocket.

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
    // Send camera frame for body tracking
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
        console.log('Body detected:', data.keypoints);
        break;
      case 'render_frame':
        // Receive rendered frame with garment overlay
        this.displayFrame(data.imageUrl);
        break;
      case 'fit_analysis':
        console.log('Fit:', data.analysis);
        break;
    }
  }

  private displayFrame(imageUrl: string): void {
    // Display the AR frame
    const img = new Image();
    img.src = imageUrl;
    document.getElementById('ar-view')?.appendChild(img);
  }

  close(): void {
    this.ws.close();
  }
}

// Usage
const arSession = new ARTryOnSession('DRESS-12345', API_KEY);

// Send camera frames (e.g., from getUserMedia)
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

## 5.4 Size Recommendation API

### 5.4.1 Recommend Size

**Endpoint**: `POST /size-recommend`

Get AI-powered size recommendation.

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

    reasoning: string;      // Human-readable explanation
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

// Example
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

// Usage example
const sizeRec = await recommendSize('DRESS-12345', {
  height: 165,
  chest: 88,
  waist: 70,
  hips: 95
});

console.log(`Recommended: ${sizeRec.recommendation.size}`);
console.log(`Confidence: ${(sizeRec.recommendation.confidence * 100).toFixed(0)}%`);
console.log(`Fit: ${sizeRec.recommendation.fitPrediction.overall}`);
```

---

## 5.5 Sustainability Calculation API

### 5.5.1 Calculate Sustainability

**Endpoint**: `POST /sustainability/calculate`

Calculate comprehensive sustainability metrics.

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
    expectedLifespan?: number;  // years
    washes?: number;            // Expected number of washes
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

  recommendations: string[];  // Improvement suggestions
}

// Example
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

// Usage
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

console.log(`Total Carbon: ${sustainability.carbonFootprint.total} kg CO₂e`);
console.log(`Rating: ${sustainability.scores.rating}`);
console.log(`${sustainability.comparison.percentageBetter}% better than average`);
```

---

## 5.6 Trend Prediction API

### 5.6.1 Predict Trends

**Endpoint**: `POST /trends/predict`

Get AI-powered trend predictions.

```typescript
interface TrendPredictRequest {
  season: string;           // "Spring 2026"
  category?: string;        // "dresses", "jackets"
  region?: string;          // "US", "EU", "Asia"
  timeframe: number;        // Months ahead (1-12)
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
    strength: number;       // 0-1 (trend strength)
    confidence: number;     // 0-1 (prediction confidence)
    timeframe: string;      // "1-3 months", "3-6 months"

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
      socialMedia: number;  // 0-1 signal strength
      runway: number;
      retail: number;
      expert: number;
    };

    momentum: 'rising' | 'stable' | 'declining';
    peakEstimate?: string; // ISO date

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

// Example
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

// Usage
const trends = await predictTrends('Spring 2026', 'dresses');

trends.predictions.forEach(trend => {
  console.log(`${trend.trend}: ${(trend.strength * 100).toFixed(0)}% strength`);
  console.log(`Colors: ${trend.attributes.colors.map(c => c.name).join(', ')}`);
  console.log(`Momentum: ${trend.momentum}`);
  console.log('---');
});
```

---

## 5.7 Error Handling

All API endpoints follow consistent error responses:

```typescript
interface APIError {
  success: false;
  error: {
    code: string;           // Error code
    message: string;        // Human-readable message
    details?: any;          // Additional details
    documentation?: string; // Link to docs
  };
}

// Common error codes
const ErrorCodes = {
  INVALID_API_KEY: 'auth/invalid-api-key',
  RATE_LIMIT_EXCEEDED: 'rate-limit/exceeded',
  GARMENT_NOT_FOUND: 'garment/not-found',
  INVALID_MEASUREMENTS: 'validation/invalid-measurements',
  MODEL_UPLOAD_FAILED: 'assets/upload-failed',
  INSUFFICIENT_DATA: 'prediction/insufficient-data'
};

// Error handling example
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
          // Wait and retry
          await new Promise(resolve => setTimeout(resolve, 60000));
          return await apiCall();

        case ErrorCodes.GARMENT_NOT_FOUND:
          throw new Error('Garment not found in database');

        default:
          throw new Error(apiError.error.message);
      }
    }
    throw error;
  }
}
```

---

## Review Questions

1. **What is the base URL for the WIA Fashion API?**
   <details>
   <summary>Answer</summary>
   https://api.wiastandards.com/fashion/v1
   </details>

2. **Name three modes available for virtual try-on.**
   <details>
   <summary>Answer</summary>
   ar_camera (AR camera overlay), avatar_3d (3D avatar), photo_upload (photo-based).
   </details>

3. **What's the typical accuracy rate for size recommendations?**
   <details>
   <summary>Answer</summary>
   92% within one size, 78% exact match.
   </details>

4. **What sustainability scores are calculated by the API?**
   <details>
   <summary>Answer</summary>
   Environmental score, social score, circular score, and total score (all 0-100).
   </details>

5. **How far ahead can trend predictions forecast?**
   <details>
   <summary>Answer</summary>
   1-12 months ahead, with accuracy decreasing for longer timeframes.
   </details>

---

## Next Steps

Continue to [**Chapter 6: Protocol**](06-protocol.md) to learn about the algorithms and calculation methods behind these APIs.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
