# 08장: 구현 가이드

## 학습 목표

이 장을 마치면 다음을 이해하게 됩니다:
- WIA 패션 테크 구현을 위한 단계별 가이드
- 3D 모델링 워크플로우 및 도구
- AI 모델 훈련 및 배포
- AR/VR 배포 전략
- 프로덕션 모범 사례 및 성능 최적화

---

## 8.1 시작하기

### 8.1.1 전제 조건 및 설정

**필수 도구**:

```bash
# Node.js 및 npm 설치
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs

# WIA 패션 SDK 설치
npm install @wiastandards/fashion-sdk

# 3D 도구 설치 (하나 이상 선택)
# - Blender (무료, 오픈소스)
# - CLO3D (전문 의류 디자인)
# - Marvelous Designer (천 시뮬레이션)

# ML 모델용 Python 설치
sudo apt-get install python3.10 python3-pip

# ML 라이브러리 설치
pip install tensorflow torch scikit-learn xgboost
```

**프로젝트 구조**:

```
fashion-tech-project/
├── assets/
│   ├── 3d-models/          # glTF, FBX 파일
│   ├── textures/           # PNG, JPG 텍스처
│   └── materials/          # 소재 정의
├── src/
│   ├── api/                # API 통합
│   ├── components/         # UI 컴포넌트
│   ├── ml/                 # ML 모델
│   └── utils/              # 헬퍼 함수
├── config/
│   └── wia-config.json     # WIA 설정
├── tests/
│   ├── unit/
│   └── integration/
├── package.json
└── README.md
```

---

## 8.2 3D 모델링 워크플로우

### 8.2.1 Blender에서 디지털 의류 생성

**1단계: 기본 메시 생성**

```python
# Blender Python 스크립트: create_dress_base.py
import bpy

def create_dress_mesh():
    # 기존 메시 지우기
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

    # 드레스 몸체를 위한 기본 원통 생성
    bpy.ops.mesh.primitive_cylinder_add(
        radius=0.5,
        depth=2.0,
        location=(0, 0, 1)
    )

    dress_body = bpy.context.active_object
    dress_body.name = "DressBody"

    # 편집 모드 진입
    bpy.ops.object.mode_set(mode='EDIT')

    # 몸통 상단을 맞춤형 바디스로 스케일 조정
    bpy.ops.mesh.select_all(action='DESELECT')
    bpy.ops.mesh.select_mode(type='VERT')

    # 상단 정점 선택
    bpy.ops.object.mode_set(mode='OBJECT')
    for v in dress_body.data.vertices:
        if v.co.z > 1.5:
            v.select = True

    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.transform.resize(value=(0.6, 0.6, 1.0))

    # 부드러운 표면을 위한 서브디비전 추가
    bpy.ops.object.mode_set(mode='OBJECT')
    mod = dress_body.modifiers.new(name="Subdivision", type='SUBSURF')
    mod.levels = 2
    mod.render_levels = 3

    return dress_body

# 함수 실행
dress = create_dress_mesh()
print(f"드레스 메시 생성: {dress.name}")
```

**2단계: UV 언래핑**

```python
# UV 언래핑 스크립트
def unwrap_dress(obj):
    # 오브젝트 선택
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode='EDIT')

    # 모든 면 선택
    bpy.ops.mesh.select_all(action='SELECT')

    # 스마트 UV 프로젝트
    bpy.ops.uv.smart_project(
        angle_limit=66.0,
        island_margin=0.02,
        area_weight=0.0
    )

    # UV 아일랜드 패킹
    bpy.ops.uv.pack_islands(margin=0.01)

    bpy.ops.object.mode_set(mode='OBJECT')
    print("UV 언래핑 완료")

unwrap_dress(dress)
```

**3단계: 소재 생성 (PBR)**

```python
def create_pbr_material(obj, name, base_color, roughness=0.6):
    # 새 소재 생성
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links

    # 기본 노드 지우기
    nodes.clear()

    # Principled BSDF 추가
    bsdf = nodes.new(type='ShaderNodeBsdfPrincipled')
    bsdf.location = (0, 0)
    bsdf.inputs['Base Color'].default_value = (*base_color, 1.0)
    bsdf.inputs['Roughness'].default_value = roughness
    bsdf.inputs['Metallic'].default_value = 0.0

    # Material Output 추가
    output = nodes.new(type='ShaderNodeOutputMaterial')
    output.location = (300, 0)

    # 노드 연결
    links.new(bsdf.outputs['BSDF'], output.inputs['Surface'])

    # 오브젝트에 소재 할당
    if obj.data.materials:
        obj.data.materials[0] = mat
    else:
        obj.data.materials.append(mat)

    print(f"소재 생성: {name}")
    return mat

# 코랄 핑크 소재 생성
create_pbr_material(
    dress,
    "CoralPinkFabric",
    base_color=(1.0, 0.42, 0.62),  # #FF6B9D RGB 값
    roughness=0.6
)
```

**4단계: glTF로 내보내기**

```python
def export_to_gltf(obj, filepath, lod='high'):
    # LOD에 따른 내보내기 설정
    settings = {
        'high': {
            'export_apply': False,
            'export_yup': True,
            'export_extras': True,
            'export_materials': 'EXPORT'
        },
        'medium': {
            'export_apply': True,  # 모디파이어 적용
            'export_yup': True,
            'export_extras': True,
            'export_materials': 'EXPORT'
        },
        'low': {
            'export_apply': True,
            'export_yup': True,
            'export_extras': False,
            'export_materials': 'EXPORT'
        }
    }

    # 오브젝트 선택
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)

    # 내보내기
    bpy.ops.export_scene.gltf(
        filepath=filepath,
        use_selection=True,
        **settings[lod]
    )

    print(f"{lod} LOD를 {filepath}로 내보냄")

# 모든 LOD 내보내기
export_to_gltf(dress, "./dress-high.glb", 'high')
export_to_gltf(dress, "./dress-medium.glb", 'medium')
export_to_gltf(dress, "./dress-low.glb", 'low')
```

### 8.2.2 천 시뮬레이션 설정

```python
def add_cloth_simulation(obj):
    # 천 모디파이어 추가
    cloth_mod = obj.modifiers.new(name="Cloth", type='CLOTH')

    # 천 설정 구성
    cloth = cloth_mod.settings

    # 물리적 특성 (면)
    cloth.quality = 5
    cloth.mass = 0.3               # kg
    cloth.air_damping = 1.0
    cloth.bending_model = 'LINEAR'

    # 강성도
    cloth.tension_stiffness = 15   # 장력
    cloth.compression_stiffness = 15
    cloth.shear_stiffness = 5      # 전단 저항
    cloth.bending_stiffness = 0.5  # 굽힘 저항

    # 감쇠
    cloth.tension_damping = 5
    cloth.compression_damping = 5
    cloth.shear_damping = 5
    cloth.bending_damping = 0.5

    # 충돌
    cloth_mod.collision_settings.use_collision = True
    cloth_mod.collision_settings.distance_min = 0.001
    cloth_mod.collision_settings.collision_quality = 4

    print("천 시뮬레이션 추가됨")

add_cloth_simulation(dress)
```

---

## 8.3 AI 모델 훈련

### 8.3.1 사이즈 추천 모델

**데이터 준비**:

```python
import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler

# 과거 구매 데이터 로드
data = pd.read_csv('purchase_history.csv')
# 컬럼: user_height, user_chest, user_waist, user_hips,
#       garment_id, size_purchased, fit_rating (-1=작음, 0=완벽, 1=큼)

# 특징 엔지니어링
def extract_features(df):
    features = []

    for idx, row in df.iterrows():
        # 의류 측정값 가져오기
        garment = get_garment_data(row['garment_id'])
        size_measurements = garment['sizes']['measurements'][row['size_purchased']]

        # 차이 계산
        chest_diff = size_measurements['chest'] - row['user_chest']
        waist_diff = size_measurements['waist'] - row['user_waist']
        hips_diff = size_measurements['hips'] - row['user_hips']

        # 비율
        chest_ratio = row['user_chest'] / size_measurements['chest']
        waist_ratio = row['user_waist'] / size_measurements['waist']
        hips_ratio = row['user_hips'] / size_measurements['hips']

        # 원단 신축성
        stretch = get_fabric_stretch(garment)

        features.append([
            row['user_height'],
            row['user_chest'],
            row['user_waist'],
            row['user_hips'],
            chest_diff,
            waist_diff,
            hips_diff,
            chest_ratio,
            waist_ratio,
            hips_ratio,
            stretch
        ])

    return np.array(features)

# 특징 추출
X = extract_features(data)
y = data['fit_rating'].values  # -1, 0, 1

# 데이터 분할
X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.2, random_state=42
)

# 정규화
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.transform(X_test)
```

**XGBoost 모델 훈련**:

```python
import xgboost as xgb
from sklearn.metrics import accuracy_score, classification_report

# XGBoost 분류기 훈련
model = xgb.XGBClassifier(
    n_estimators=300,
    max_depth=6,
    learning_rate=0.1,
    subsample=0.8,
    colsample_bytree=0.8,
    random_state=42
)

model.fit(
    X_train_scaled,
    y_train,
    eval_set=[(X_test_scaled, y_test)],
    early_stopping_rounds=10,
    verbose=True
)

# 평가
y_pred = model.predict(X_test_scaled)
accuracy = accuracy_score(y_test, y_pred)
print(f"정확도: {accuracy:.2%}")
print(classification_report(y_test, y_pred))

# 모델 저장
model.save_model('size_recommendation_model.json')
scaler_params = {'mean': scaler.mean_.tolist(), 'scale': scaler.scale_.tolist()}
import json
with open('scaler_params.json', 'w') as f:
    json.dump(scaler_params, f)

print("모델 저장 완료!")
```

**API로 배포**:

```python
from flask import Flask, request, jsonify
import xgboost as xgb
import json

app = Flask(__name__)

# 시작 시 모델 로드
model = xgb.XGBClassifier()
model.load_model('size_recommendation_model.json')

with open('scaler_params.json', 'r') as f:
    scaler_params = json.load(f)

@app.route('/recommend-size', methods=['POST'])
def recommend_size():
    data = request.json

    # 특징 추출
    features = extract_features_from_request(data)

    # 특징 스케일링
    features_scaled = (features - scaler_params['mean']) / scaler_params['scale']

    # 예측
    fit_prediction = model.predict([features_scaled])[0]
    probabilities = model.predict_proba([features_scaled])[0]

    # 결과 해석
    current_size = data['current_size']
    available_sizes = data['available_sizes']

    if fit_prediction == -1:  # 너무 작음
        recommended = get_next_size(current_size, available_sizes)
        reason = "측정값을 기반으로 이 사이즈는 조금 작을 수 있습니다. 한 사이즈 큰 것을 추천합니다."
    elif fit_prediction == 1:  # 너무 큼
        recommended = get_prev_size(current_size, available_sizes)
        reason = "이 사이즈는 조금 클 수 있습니다. 한 사이즈 작은 것을 추천합니다."
    else:  # 완벽한 핏
        recommended = current_size
        reason = "이 사이즈가 완벽하게 맞을 것입니다!"

    return jsonify({
        'recommended_size': recommended,
        'confidence': float(max(probabilities)),
        'fit_prediction': int(fit_prediction),
        'reasoning': reason
    })

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
```

---

## 8.4 AR 피팅 배포

### 8.4.1 Three.js를 사용한 웹 기반 AR

```typescript
// ar-tryon-viewer.ts
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';

class ARTryOnViewer {
  private scene: THREE.Scene;
  private camera: THREE.PerspectiveCamera;
  private renderer: THREE.WebGLRenderer;
  private controls: OrbitControls;
  private garmentModel?: THREE.Object3D;

  constructor(container: HTMLElement) {
    // 씬 초기화
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xf0f0f0);

    // 카메라 초기화
    this.camera = new THREE.PerspectiveCamera(
      45,
      container.clientWidth / container.clientHeight,
      0.1,
      1000
    );
    this.camera.position.set(0, 1.5, 3);

    // 렌더러 초기화
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(container.clientWidth, container.clientHeight);
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.shadowMap.enabled = true;
    container.appendChild(this.renderer.domElement);

    // 컨트롤 추가
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 1, 0);
    this.controls.update();

    // 조명 추가
    this.setupLighting();

    // 렌더 루프 시작
    this.animate();
  }

  private setupLighting(): void {
    // 앰비언트 라이트
    const ambient = new THREE.AmbientLight(0xffffff, 0.5);
    this.scene.add(ambient);

    // 디렉셔널 라이트 (태양)
    const directional = new THREE.DirectionalLight(0xffffff, 0.8);
    directional.position.set(5, 10, 5);
    directional.castShadow = true;
    this.scene.add(directional);

    // 헤미스피어 라이트
    const hemisphere = new THREE.HemisphereLight(0xffffff, 0x444444, 0.6);
    this.scene.add(hemisphere);
  }

  async loadGarment(modelUrl: string): Promise<void> {
    const loader = new GLTFLoader();

    return new Promise((resolve, reject) => {
      loader.load(
        modelUrl,
        (gltf) => {
          // 이전 모델 제거
          if (this.garmentModel) {
            this.scene.remove(this.garmentModel);
          }

          this.garmentModel = gltf.scene;

          // 그림자 활성화
          this.garmentModel.traverse((child) => {
            if ((child as THREE.Mesh).isMesh) {
              child.castShadow = true;
              child.receiveShadow = true;
            }
          });

          this.scene.add(this.garmentModel);
          resolve();
        },
        undefined,
        reject
      );
    });
  }

  async loadAvatar(avatarUrl: string): Promise<void> {
    const loader = new GLTFLoader();

    return new Promise((resolve, reject) => {
      loader.load(
        avatarUrl,
        (gltf) => {
          const avatar = gltf.scene;
          avatar.position.set(0, 0, 0);

          // 그림자 활성화
          avatar.traverse((child) => {
            if ((child as THREE.Mesh).isMesh) {
              child.receiveShadow = true;
            }
          });

          this.scene.add(avatar);
          resolve();
        },
        undefined,
        reject
      );
    });
  }

  private animate = (): void => {
    requestAnimationFrame(this.animate);

    // 컨트롤 업데이트
    this.controls.update();

    // 씬 렌더링
    this.renderer.render(this.scene, this.camera);
  };

  // 의류 회전
  rotateGarment(angle: number): void {
    if (this.garmentModel) {
      this.garmentModel.rotation.y = angle;
    }
  }

  // 스크린샷 촬영
  takeScreenshot(): string {
    return this.renderer.domElement.toDataURL('image/png');
  }

  // 정리
  dispose(): void {
    this.renderer.dispose();
    this.controls.dispose();
  }
}

// 사용 예제
const viewer = new ARTryOnViewer(
  document.getElementById('ar-viewer')!
);

await viewer.loadAvatar('https://cdn.example.com/avatar.glb');
await viewer.loadGarment('https://cdn.example.com/dress-medium.glb');

// UI 컨트롤 추가
document.getElementById('rotate-btn')?.addEventListener('click', () => {
  viewer.rotateGarment(Math.PI);
});

document.getElementById('screenshot-btn')?.addEventListener('click', () => {
  const imageData = viewer.takeScreenshot();
  // 다운로드 또는 공유
});
```

### 8.4.2 AR.js를 사용한 모바일 AR

```html
<!-- ar-tryon-mobile.html -->
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>WIA 패션 AR 피팅</title>
  <script src="https://aframe.io/releases/1.4.0/aframe.min.js"></script>
  <script src="https://cdn.jsdelivr.net/gh/AR-js-org/AR.js/aframe/build/aframe-ar.js"></script>
</head>
<body style="margin: 0; overflow: hidden;">
  <a-scene
    embedded
    arjs="sourceType: webcam; debugUIEnabled: false;"
    vr-mode-ui="enabled: false">

    <!-- 카메라 -->
    <a-camera gps-camera rotation-reader></a-camera>

    <!-- 3D 의류 모델 -->
    <a-entity
      id="garment"
      gltf-model="url(https://cdn.example.com/dress-medium.glb)"
      scale="1 1 1"
      position="0 0 -2"
      rotation="0 0 0">
    </a-entity>

    <!-- 조명 -->
    <a-light type="ambient" color="#fff" intensity="0.5"></a-light>
    <a-light type="directional" color="#fff" intensity="0.8" position="1 1 1"></a-light>

  </a-scene>

  <script>
    // AR.js 초기화 대기
    window.addEventListener('load', () => {
      const garment = document.getElementById('garment');

      // 감지된 표면에 의류 배치
      garment.addEventListener('model-loaded', () => {
        console.log('의류 모델 로드됨');

        // 자동 회전
        setInterval(() => {
          const currentRotation = garment.getAttribute('rotation');
          garment.setAttribute('rotation', {
            x: currentRotation.x,
            y: currentRotation.y + 1,
            z: currentRotation.z
          });
        }, 50);
      });
    });
  </script>
</body>
</html>
```

---

## 8.5 프로덕션 모범 사례

### 8.5.1 성능 최적화

**3D 에셋 최적화**:

```bash
# gltfpack으로 glTF 파일 최적화
gltfpack -i dress-high.glb -o dress-optimized.glb \
  -cc -tc \
  -si 1.0 \
  -vp 14 -vt 12 -vn 8

# 파일 크기를 70-90% 줄입니다
```

**텍스처 압축**:

```bash
# 텍스처를 KTX2 형식으로 변환 (GPU 압축)
toktx --bcmp --genmipmap \
  dress-basecolor.ktx2 dress-basecolor.png

# 동일한 시각적 품질로 4-8배 작은 파일 크기
```

**CDN 설정**:

```javascript
// 3D 에셋에 CDN 사용
const CDN_BASE = 'https://cdn.wiastandards.com/fashion';

const assetUrls = {
  high: `${CDN_BASE}/dress-12345/high.glb`,
  medium: `${CDN_BASE}/dress-12345/medium.glb`,
  low: `${CDN_BASE}/dress-12345/low.glb`
};

// 점진적 로딩
async function loadGarmentProgressive() {
  // 즉각적인 프리뷰를 위해 먼저 저해상도 로드
  await loadModel(assetUrls.low);

  // 그런 다음 백그라운드에서 중해상도 로드
  loadModel(assetUrls.medium).then(model => {
    replaceModel(model);
  });
}
```

### 8.5.2 모니터링 및 분석

```typescript
// 주요 메트릭 추적
interface FashionTechMetrics {
  tryOnSessions: number;
  sizeRecommendations: number;
  recommendationAccuracy: number;
  returnRate: number;
  conversionRate: number;
  avgLoadTime: number;
}

class MetricsCollector {
  async track(event: string, data: any): Promise<void> {
    await fetch('https://analytics.wiastandards.com/track', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        event,
        data,
        timestamp: new Date().toISOString()
      })
    });
  }

  trackTryOnSession(garmentId: string, duration: number): void {
    this.track('tryon_session', { garmentId, duration });
  }

  trackSizeRecommendation(
    garmentId: string,
    recommended: string,
    selected: string
  ): void {
    this.track('size_recommendation', {
      garmentId,
      recommended,
      selected,
      match: recommended === selected
    });
  }

  trackPurchase(
    garmentId: string,
    size: string,
    usedTryOn: boolean
  ): void {
    this.track('purchase', { garmentId, size, usedTryOn });
  }
}
```

### 8.5.3 캐싱 전략

```typescript
// 서비스 워커를 사용한 3D 모델 캐싱
// service-worker.js
const CACHE_NAME = 'wia-fashion-models-v1';
const MODEL_CACHE = [
  '/models/dress-low.glb',
  '/models/dress-medium.glb',
  '/textures/common-materials.ktx2'
];

self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME).then((cache) => {
      return cache.addAll(MODEL_CACHE);
    })
  );
});

self.addEventListener('fetch', (event) => {
  // 모델 파일에 대한 캐시 우선 전략
  if (event.request.url.includes('/models/') ||
      event.request.url.includes('/textures/')) {
    event.respondWith(
      caches.match(event.request).then((response) => {
        return response || fetch(event.request).then((fetchResponse) => {
          return caches.open(CACHE_NAME).then((cache) => {
            cache.put(event.request, fetchResponse.clone());
            return fetchResponse;
          });
        });
      })
    );
  }
});
```

### 8.5.4 에러 처리

```typescript
// 강력한 에러 처리
class WIAFashionSDK {
  async loadGarment(garmentId: string): Promise<WIAFashionGarment> {
    try {
      const response = await fetch(
        `https://api.wiastandards.com/fashion/v1/garments/${garmentId}`,
        {
          headers: {
            'Authorization': `Bearer ${this.apiKey}`
          }
        }
      );

      if (!response.ok) {
        throw new WIAAPIError(
          `의류 로드 실패: ${response.status}`,
          response.status
        );
      }

      const data = await response.json();
      return data;

    } catch (error) {
      if (error instanceof WIAAPIError) {
        // API 에러 처리
        console.error('WIA API 에러:', error.message);

        if (error.statusCode === 404) {
          throw new Error('의류를 찾을 수 없습니다');
        } else if (error.statusCode === 401) {
          throw new Error('유효하지 않은 API 키입니다');
        }
      } else if (error instanceof TypeError) {
        // 네트워크 에러
        throw new Error('네트워크 연결을 확인해주세요');
      }

      throw error;
    }
  }
}

class WIAAPIError extends Error {
  constructor(message: string, public statusCode: number) {
    super(message);
    this.name = 'WIAAPIError';
  }
}
```

---

## 8.6 테스트 전략

### 8.6.1 단위 테스트

```typescript
// __tests__/size-recommendation.test.ts
import { SizeRecommender } from '../src/ml/size-recommender';

describe('SizeRecommender', () => {
  let recommender: SizeRecommender;

  beforeEach(() => {
    recommender = new SizeRecommender();
  });

  test('정확한 사이즈 추천', () => {
    const measurements = {
      height: 170,
      chest: 88,
      waist: 70,
      hips: 95
    };

    const recommendation = recommender.recommend(
      'WIA-DRESS-2026-12345',
      measurements
    );

    expect(recommendation.size).toBe('M');
    expect(recommendation.confidence).toBeGreaterThan(0.8);
  });

  test('사이즈 범위 밖 처리', () => {
    const measurements = {
      height: 150,
      chest: 70,
      waist: 55,
      hips: 75
    };

    const recommendation = recommender.recommend(
      'WIA-DRESS-2026-12345',
      measurements
    );

    expect(recommendation.size).toBe('XS');
    expect(recommendation.warnings).toContain('범위 밖');
  });
});
```

### 8.6.2 통합 테스트

```typescript
// __tests__/integration/garment-loading.test.ts
import { WIAFashionSDK } from '../../src';

describe('의류 로딩 통합 테스트', () => {
  let sdk: WIAFashionSDK;

  beforeAll(() => {
    sdk = new WIAFashionSDK({
      apiKey: process.env.TEST_API_KEY
    });
  });

  test('API에서 의류 로드', async () => {
    const garment = await sdk.loadGarment('WIA-DRESS-2026-12345');

    expect(garment.garment.id).toBe('WIA-DRESS-2026-12345');
    expect(garment.garment.assets_3d.models).toHaveLength(3);
    expect(garment.garment.sustainability.totalScore).toBeGreaterThan(0);
  });

  test('3D 모델 로드', async () => {
    const garment = await sdk.loadGarment('WIA-DRESS-2026-12345');
    const modelUrl = garment.garment.assets_3d.models[0].url;

    const response = await fetch(modelUrl);
    expect(response.ok).toBe(true);
    expect(response.headers.get('content-type')).toContain('model/gltf');
  });
});
```

### 8.6.3 E2E 테스트

```typescript
// e2e/virtual-tryon.spec.ts (Playwright 사용)
import { test, expect } from '@playwright/test';

test('가상 피팅 플로우', async ({ page }) => {
  // 제품 페이지로 이동
  await page.goto('https://example.com/products/wia-dress-12345');

  // 가상 피팅 버튼 클릭
  await page.click('.wia-tryon-button');

  // 모달이 나타나기를 기다림
  await page.waitForSelector('#wia-tryon-modal');

  // 3D 모델이 로드될 때까지 대기
  await page.waitForSelector('canvas', { timeout: 10000 });

  // 스크린샷 촬영
  await page.screenshot({ path: 'tryon-session.png' });

  // 회전 버튼 테스트
  await page.click('#rotate-btn');
  await page.waitForTimeout(1000);

  // 모달 닫기
  await page.click('.close');
  await expect(page.locator('#wia-tryon-modal')).not.toBeVisible();
});
```

---

## 8.7 배포

### 8.7.1 Docker 컨테이너화

```dockerfile
# Dockerfile
FROM node:18-alpine

WORKDIR /app

# 의존성 설치
COPY package*.json ./
RUN npm ci --only=production

# 애플리케이션 코드 복사
COPY . .

# 프로덕션 빌드
RUN npm run build

EXPOSE 3000

CMD ["npm", "start"]
```

```yaml
# docker-compose.yml
version: '3.8'

services:
  web:
    build: .
    ports:
      - "3000:3000"
    environment:
      - NODE_ENV=production
      - WIA_API_KEY=${WIA_API_KEY}
    volumes:
      - ./models:/app/models
    depends_on:
      - redis

  redis:
    image: redis:7-alpine
    ports:
      - "6379:6379"

  nginx:
    image: nginx:alpine
    ports:
      - "80:80"
      - "443:443"
    volumes:
      - ./nginx.conf:/etc/nginx/nginx.conf
      - ./ssl:/etc/nginx/ssl
    depends_on:
      - web
```

### 8.7.2 Kubernetes 배포

```yaml
# k8s/deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: wia-fashion-app
spec:
  replicas: 3
  selector:
    matchLabels:
      app: wia-fashion
  template:
    metadata:
      labels:
        app: wia-fashion
    spec:
      containers:
      - name: web
        image: wiastandards/fashion-app:1.0.0
        ports:
        - containerPort: 3000
        env:
        - name: WIA_API_KEY
          valueFrom:
            secretKeyRef:
              name: wia-secrets
              key: api-key
        resources:
          requests:
            memory: "256Mi"
            cpu: "250m"
          limits:
            memory: "512Mi"
            cpu: "500m"
---
apiVersion: v1
kind: Service
metadata:
  name: wia-fashion-service
spec:
  selector:
    app: wia-fashion
  ports:
  - protocol: TCP
    port: 80
    targetPort: 3000
  type: LoadBalancer
```

---

## 8.8 보안 모범 사례

### 8.8.1 API 키 관리

```typescript
// .env 파일 사용 (절대 커밋하지 마세요!)
// .env
WIA_API_KEY=wia_prod_abc123xyz789
STRIPE_SECRET_KEY=EXAMPLE_API_KEY_REPLACE_ME
DATABASE_URL=postgresql://...

// config.ts
import dotenv from 'dotenv';
dotenv.config();

export const config = {
  wiaApiKey: process.env.WIA_API_KEY,
  stripeSecretKey: process.env.STRIPE_SECRET_KEY,
  databaseUrl: process.env.DATABASE_URL
};

// 프로덕션에서는 환경 변수 또는 비밀 관리 서비스 사용
// (AWS Secrets Manager, Google Secret Manager, HashiCorp Vault)
```

### 8.8.2 입력 검증

```typescript
// Zod를 사용한 입력 검증
import { z } from 'zod';

const BodyMeasurementsSchema = z.object({
  height: z.number().min(120).max(220),
  chest: z.number().min(60).max(150),
  waist: z.number().min(50).max(130),
  hips: z.number().min(60).max(150)
});

function validateMeasurements(data: unknown): BodyMeasurements {
  try {
    return BodyMeasurementsSchema.parse(data);
  } catch (error) {
    throw new Error('유효하지 않은 측정값입니다');
  }
}

// API 엔드포인트에서 사용
app.post('/recommend-size', (req, res) => {
  try {
    const measurements = validateMeasurements(req.body.measurements);
    // 측정값 처리...
  } catch (error) {
    res.status(400).json({ error: error.message });
  }
});
```

### 8.8.3 레이트 리미팅

```typescript
// Express rate limiter
import rateLimit from 'express-rate-limit';

const apiLimiter = rateLimit({
  windowMs: 15 * 60 * 1000, // 15분
  max: 100, // 요청 제한
  message: '너무 많은 요청을 보냈습니다. 나중에 다시 시도해주세요.'
});

// API 라우트에 적용
app.use('/api/', apiLimiter);
```

---

## 복습 질문

1. **내보내야 하는 세 가지 LOD 레벨은 무엇입니까?**
   <details>
   <summary>답변</summary>
   High (20K-50K 폴리곤), Medium (5K-10K 폴리곤), Low (1K-3K 폴리곤).
   </details>

2. **사이즈 추천에 권장되는 ML 알고리즘은 무엇입니까?**
   <details>
   <summary>답변</summary>
   XGBoost (Gradient Boosted Trees) 300개 트리, 최대 깊이 6으로 분류.
   </details>

3. **WebGL에 가장 적합한 압축을 제공하는 텍스처 형식은 무엇입니까?**
   <details>
   <summary>답변</summary>
   Basis Universal 압축을 사용한 KTX2 형식 (동일한 품질로 4-8배 작음).
   </details>

4. **웹 기반 AR에 사용되는 두 가지 JavaScript 라이브러리는 무엇입니까?**
   <details>
   <summary>답변</summary>
   Three.js (3D 렌더링)와 AR.js 또는 A-Frame (AR 프레임워크).
   </details>

5. **glTF 파일을 최적화하는 데 사용되는 도구는 무엇입니까?**
   <details>
   <summary>답변</summary>
   gltfpack (파일 크기를 70-90% 줄일 수 있음).
   </details>

6. **Blender에서 PBR 소재를 만드는 데 사용되는 노드는 무엇입니까?**
   <details>
   <summary>답변</summary>
   Principled BSDF 노드 - 베이스 컬러, 러프니스, 메탈릭 등을 제어합니다.
   </details>

7. **천 시뮬레이션의 세 가지 주요 물리적 속성은 무엇입니까?**
   <details>
   <summary>답변</summary>
   질량 (mass), 강성도 (stiffness: tension, compression, shear, bending), 감쇠 (damping).
   </details>

8. **프로덕션에서 3D 모델을 로딩하는 권장 전략은 무엇입니까?**
   <details>
   <summary>답변</summary>
   점진적 로딩: 먼저 저해상도 모델을 즉시 표시하고, 백그라운드에서 고해상도 모델을 로드합니다.
   </details>

---

## 실습 프로젝트

### 프로젝트 1: 완전한 가상 피팅 앱

**목표**: Three.js와 WIA SDK를 사용하여 완전히 작동하는 가상 피팅 웹 앱을 구축합니다.

**요구사항**:
1. 3개의 LOD 레벨로 Blender에서 의류 생성
2. 3D 뷰어에서 의류 로드
3. 아바타 커스터마이징 구현
4. 스크린샷 및 소셜 공유 기능 추가
5. Vercel 또는 Netlify에 배포

### 프로젝트 2: AI 사이즈 추천 시스템

**목표**: XGBoost를 사용하여 사이즈 추천 ML 모델을 훈련하고 배포합니다.

**요구사항**:
1. 구매 이력 데이터셋 수집 또는 생성
2. 특징 엔지니어링 수행
3. XGBoost 모델 훈련 (>85% 정확도 목표)
4. Flask API로 배포
5. 프론트엔드 UI 통합

### 프로젝트 3: 패션 NFT 마켓플레이스

**목표**: WIA 표준을 사용하여 패션 NFT 마켓플레이스를 구축합니다.

**요구사항**:
1. 테스트넷에 ERC-721 컨트랙트 배포
2. IPFS에 의류 데이터 업로드
3. NFT 발행 인터페이스 구현
4. NFT 갤러리 및 상세 페이지 생성
5. OpenSea 호환성 확인

---

## 다음 단계

**축하합니다!**

WIA 패션 테크 표준 전자책을 완료했습니다. 이제 다음을 수행할 수 있는 지식을 갖추었습니다:

- ✅ 디지털 패션 시스템 구축
- ✅ 가상 피팅 경험 구현
- ✅ 지속가능성 지표 계산
- ✅ AI 추천 모델 배포
- ✅ 전자상거래 및 메타버스 플랫폼과 통합
- ✅ 패션 NFT 생성 및 관리

### 추가 학습 리소스

**공식 문서**:
- WIA 패션 API: https://api.wiastandards.com/docs
- Blender 문서: https://docs.blender.org
- Three.js 문서: https://threejs.org/docs
- XGBoost 가이드: https://xgboost.readthedocs.io

**커뮤니티**:
1. **WIA 커뮤니티 가입**: https://wiastandards.com/community
2. **Discord 서버**: https://discord.gg/wiastandards
3. **GitHub 토론**: https://github.com/WIA-Official/wia-standards/discussions

**API 및 도구**:
1. **API 액세스 받기**: https://api.wiastandards.com/signup
2. **SDK 다운로드**: https://github.com/WIA-Official/fashion-sdk
3. **샘플 프로젝트**: https://github.com/WIA-Official/fashion-examples

**기여하기**:
- 표준 개선 제안: https://github.com/WIA-Official/wia-standards/issues
- 코드 기여: https://github.com/WIA-Official/fashion-sdk/pulls
- 문서 개선: https://github.com/WIA-Official/docs/pulls

### 구현 공유

구현을 완료했나요? 커뮤니티와 공유해주세요!

1. **소셜 미디어**: @WIAStandards 태그
2. **쇼케이스**: showcase@wiastandards.com으로 이메일 보내기
3. **블로그 게시물**: 공식 블로그에 기고

---

## 부록: 문제 해결

### 일반적인 문제

**문제: 3D 모델이 로드되지 않음**
```typescript
// 해결책: CORS 헤더 확인
// 서버에서 다음을 활성화:
Access-Control-Allow-Origin: *
Access-Control-Allow-Methods: GET, POST
```

**문제: 사이즈 추천이 부정확함**
```python
# 해결책: 더 많은 훈련 데이터 수집
# 최소 1000개 샘플, 클래스당 균형 있게
# 특징 스케일링 확인
```

**문제: AR 뷰어 성능 저하**
```javascript
// 해결책: LOD 및 프로그레시브 로딩 사용
// 모바일에는 low-poly 모델만 로드
if (isMobile()) {
  loadModel(assetUrls.low);
} else {
  loadModel(assetUrls.medium);
}
```

### 디버깅 팁

```typescript
// 상세한 로깅 활성화
const sdk = new WIAFashionSDK({
  apiKey: '...',
  debug: true,
  logLevel: 'verbose'
});

// 성능 프로파일링
console.time('모델 로드');
await loadGarmentModel('WIA-DRESS-2026-12345');
console.timeEnd('모델 로드');

// 메모리 사용량 모니터링
console.log('메모리:', performance.memory.usedJSHeapSize / 1048576, 'MB');
```

---

## 용어집

**LOD (Level of Detail)**: 카메라 거리에 따라 다양한 복잡도의 3D 모델

**PBR (Physically Based Rendering)**: 현실적인 조명을 위한 셰이딩 모델

**glTF**: GL Transmission Format - 3D 모델용 표준 파일 형식

**UV 언래핑**: 3D 표면을 2D 텍스처 좌표로 매핑

**XGBoost**: 극단적 그래디언트 부스팅 - ML 알고리즘

**IPFS**: 분산 파일 저장 시스템

**ERC-721**: NFT용 이더리움 토큰 표준

**Three.js**: 웹용 3D 그래픽 라이브러리

**AR (Augmented Reality)**: 증강 현실

**VR (Virtual Reality)**: 가상 현실

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
