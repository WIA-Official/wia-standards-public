# Chapter 8: Implementation Guide

## Learning Objectives

By the end of this chapter, you will understand:
- Step-by-step guide to implementing WIA Fashion Tech
- 3D modeling workflow and tools
- AI model training and deployment
- AR/VR deployment strategies
- Production best practices and performance optimization

---

## 8.1 Getting Started

### 8.1.1 Prerequisites and Setup

**Required Tools**:

```bash
# Install Node.js and npm
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs

# Install WIA Fashion SDK
npm install @wiastandards/fashion-sdk

# Install 3D tools (choose one or more)
# - Blender (free, open-source)
# - CLO3D (professional garment design)
# - Marvelous Designer (cloth simulation)

# Install Python for ML models
sudo apt-get install python3.10 python3-pip

# Install ML libraries
pip install tensorflow torch scikit-learn xgboost
```

**Project Structure**:

```
fashion-tech-project/
├── assets/
│   ├── 3d-models/          # glTF, FBX files
│   ├── textures/           # PNG, JPG textures
│   └── materials/          # Material definitions
├── src/
│   ├── api/                # API integration
│   ├── components/         # UI components
│   ├── ml/                 # ML models
│   └── utils/              # Helper functions
├── config/
│   └── wia-config.json     # WIA configuration
├── tests/
│   ├── unit/
│   └── integration/
├── package.json
└── README.md
```

---

## 8.2 3D Modeling Workflow

### 8.2.1 Creating Digital Garments in Blender

**Step 1: Base Mesh Creation**

```python
# Blender Python script: create_dress_base.py
import bpy

def create_dress_mesh():
    # Clear existing mesh
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

    # Create base cylinder for dress body
    bpy.ops.mesh.primitive_cylinder_add(
        radius=0.5,
        depth=2.0,
        location=(0, 0, 1)
    )

    dress_body = bpy.context.active_object
    dress_body.name = "DressBody"

    # Enter edit mode
    bpy.ops.object.mode_set(mode='EDIT')

    # Scale top for fitted bodice
    bpy.ops.mesh.select_all(action='DESELECT')
    bpy.ops.mesh.select_mode(type='VERT')

    # Select top vertices
    bpy.ops.object.mode_set(mode='OBJECT')
    for v in dress_body.data.vertices:
        if v.co.z > 1.5:
            v.select = True

    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.transform.resize(value=(0.6, 0.6, 1.0))

    # Add subdivision for smooth surface
    bpy.ops.object.mode_set(mode='OBJECT')
    mod = dress_body.modifiers.new(name="Subdivision", type='SUBSURF')
    mod.levels = 2
    mod.render_levels = 3

    return dress_body

# Run the function
dress = create_dress_mesh()
print(f"Created dress mesh: {dress.name}")
```

**Step 2: UV Unwrapping**

```python
# UV unwrapping script
def unwrap_dress(obj):
    # Select object
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode='EDIT')

    # Select all faces
    bpy.ops.mesh.select_all(action='SELECT')

    # Smart UV project
    bpy.ops.uv.smart_project(
        angle_limit=66.0,
        island_margin=0.02,
        area_weight=0.0
    )

    # Pack UV islands
    bpy.ops.uv.pack_islands(margin=0.01)

    bpy.ops.object.mode_set(mode='OBJECT')
    print("UV unwrapping complete")

unwrap_dress(dress)
```

**Step 3: Material Creation (PBR)**

```python
def create_pbr_material(obj, name, base_color, roughness=0.6):
    # Create new material
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links

    # Clear default nodes
    nodes.clear()

    # Add Principled BSDF
    bsdf = nodes.new(type='ShaderNodeBsdfPrincipled')
    bsdf.location = (0, 0)
    bsdf.inputs['Base Color'].default_value = (*base_color, 1.0)
    bsdf.inputs['Roughness'].default_value = roughness
    bsdf.inputs['Metallic'].default_value = 0.0

    # Add Material Output
    output = nodes.new(type='ShaderNodeOutputMaterial')
    output.location = (300, 0)

    # Link nodes
    links.new(bsdf.outputs['BSDF'], output.inputs['Surface'])

    # Assign material to object
    if obj.data.materials:
        obj.data.materials[0] = mat
    else:
        obj.data.materials.append(mat)

    print(f"Created material: {name}")
    return mat

# Create coral pink material
create_pbr_material(
    dress,
    "CoralPinkFabric",
    base_color=(1.0, 0.42, 0.62),  # #FF6B9D in RGB
    roughness=0.6
)
```

**Step 4: Export to glTF**

```python
def export_to_gltf(obj, filepath, lod='high'):
    # Set export settings based on LOD
    settings = {
        'high': {
            'export_apply': False,
            'export_yup': True,
            'export_extras': True,
            'export_materials': 'EXPORT'
        },
        'medium': {
            'export_apply': True,  # Apply modifiers
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

    # Select object
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)

    # Export
    bpy.ops.export_scene.gltf(
        filepath=filepath,
        use_selection=True,
        **settings[lod]
    )

    print(f"Exported {lod} LOD to {filepath}")

# Export all LODs
export_to_gltf(dress, "./dress-high.glb", 'high')
export_to_gltf(dress, "./dress-medium.glb", 'medium')
export_to_gltf(dress, "./dress-low.glb", 'low')
```

### 8.2.2 Cloth Simulation Setup

```python
def add_cloth_simulation(obj):
    # Add cloth modifier
    cloth_mod = obj.modifiers.new(name="Cloth", type='CLOTH')

    # Configure cloth settings
    cloth = cloth_mod.settings

    # Physical properties (cotton)
    cloth.quality = 5
    cloth.mass = 0.3               # kg
    cloth.air_damping = 1.0
    cloth.bending_model = 'LINEAR'

    # Stiffness
    cloth.tension_stiffness = 15   # Tension
    cloth.compression_stiffness = 15
    cloth.shear_stiffness = 5      # Shear resistance
    cloth.bending_stiffness = 0.5  # Bend resistance

    # Damping
    cloth.tension_damping = 5
    cloth.compression_damping = 5
    cloth.shear_damping = 5
    cloth.bending_damping = 0.5

    # Collision
    cloth_mod.collision_settings.use_collision = True
    cloth_mod.collision_settings.distance_min = 0.001
    cloth_mod.collision_settings.collision_quality = 4

    print("Cloth simulation added")

add_cloth_simulation(dress)
```

---

## 8.3 AI Model Training

### 8.3.1 Size Recommendation Model

**Data Preparation**:

```python
import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler

# Load historical purchase data
data = pd.read_csv('purchase_history.csv')
# Columns: user_height, user_chest, user_waist, user_hips,
#          garment_id, size_purchased, fit_rating (-1=small, 0=perfect, 1=large)

# Feature engineering
def extract_features(df):
    features = []

    for idx, row in df.iterrows():
        # Get garment measurements
        garment = get_garment_data(row['garment_id'])
        size_measurements = garment['sizes']['measurements'][row['size_purchased']]

        # Calculate differences
        chest_diff = size_measurements['chest'] - row['user_chest']
        waist_diff = size_measurements['waist'] - row['user_waist']
        hips_diff = size_measurements['hips'] - row['user_hips']

        # Ratios
        chest_ratio = row['user_chest'] / size_measurements['chest']
        waist_ratio = row['user_waist'] / size_measurements['waist']
        hips_ratio = row['user_hips'] / size_measurements['hips']

        # Fabric stretch
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

# Extract features
X = extract_features(data)
y = data['fit_rating'].values  # -1, 0, 1

# Split data
X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.2, random_state=42
)

# Normalize
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.transform(X_test)
```

**Training XGBoost Model**:

```python
import xgboost as xgb
from sklearn.metrics import accuracy_score, classification_report

# Train XGBoost classifier
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

# Evaluate
y_pred = model.predict(X_test_scaled)
accuracy = accuracy_score(y_test, y_pred)
print(f"Accuracy: {accuracy:.2%}")
print(classification_report(y_test, y_pred))

# Save model
model.save_model('size_recommendation_model.json')
scaler_params = {'mean': scaler.mean_.tolist(), 'scale': scaler.scale_.tolist()}
import json
with open('scaler_params.json', 'w') as f:
    json.dump(scaler_params, f)

print("Model saved!")
```

**Deployment as API**:

```python
from flask import Flask, request, jsonify
import xgboost as xgb
import json

app = Flask(__name__)

# Load model at startup
model = xgb.XGBClassifier()
model.load_model('size_recommendation_model.json')

with open('scaler_params.json', 'r') as f:
    scaler_params = json.load(f)

@app.route('/recommend-size', methods=['POST'])
def recommend_size():
    data = request.json

    # Extract features
    features = extract_features_from_request(data)

    # Scale features
    features_scaled = (features - scaler_params['mean']) / scaler_params['scale']

    # Predict
    fit_prediction = model.predict([features_scaled])[0]
    probabilities = model.predict_proba([features_scaled])[0]

    # Interpret result
    current_size = data['current_size']
    available_sizes = data['available_sizes']

    if fit_prediction == -1:  # Too small
        recommended = get_next_size(current_size, available_sizes)
        reason = "Based on your measurements, this size may be tight. We recommend going up one size."
    elif fit_prediction == 1:  # Too large
        recommended = get_prev_size(current_size, available_sizes)
        reason = "This size may be loose for you. We recommend going down one size."
    else:  # Perfect fit
        recommended = current_size
        reason = "This size should fit you perfectly!"

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

## 8.4 AR Try-On Deployment

### 8.4.1 Web-Based AR with Three.js

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
    // Initialize scene
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xf0f0f0);

    // Initialize camera
    this.camera = new THREE.PerspectiveCamera(
      45,
      container.clientWidth / container.clientHeight,
      0.1,
      1000
    );
    this.camera.position.set(0, 1.5, 3);

    // Initialize renderer
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(container.clientWidth, container.clientHeight);
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.shadowMap.enabled = true;
    container.appendChild(this.renderer.domElement);

    // Add controls
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 1, 0);
    this.controls.update();

    // Add lights
    this.setupLighting();

    // Start render loop
    this.animate();
  }

  private setupLighting(): void {
    // Ambient light
    const ambient = new THREE.AmbientLight(0xffffff, 0.5);
    this.scene.add(ambient);

    // Directional light (sun)
    const directional = new THREE.DirectionalLight(0xffffff, 0.8);
    directional.position.set(5, 10, 5);
    directional.castShadow = true;
    this.scene.add(directional);

    // Hemisphere light
    const hemisphere = new THREE.HemisphereLight(0xffffff, 0x444444, 0.6);
    this.scene.add(hemisphere);
  }

  async loadGarment(modelUrl: string): Promise<void> {
    const loader = new GLTFLoader();

    return new Promise((resolve, reject) => {
      loader.load(
        modelUrl,
        (gltf) => {
          // Remove previous model
          if (this.garmentModel) {
            this.scene.remove(this.garmentModel);
          }

          this.garmentModel = gltf.scene;

          // Enable shadows
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

          // Enable shadows
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

    // Update controls
    this.controls.update();

    // Render scene
    this.renderer.render(this.scene, this.camera);
  };

  // Rotate garment
  rotateGarment(angle: number): void {
    if (this.garmentModel) {
      this.garmentModel.rotation.y = angle;
    }
  }

  // Take screenshot
  takeScreenshot(): string {
    return this.renderer.domElement.toDataURL('image/png');
  }

  // Cleanup
  dispose(): void {
    this.renderer.dispose();
    this.controls.dispose();
  }
}

// Usage
const viewer = new ARTryOnViewer(
  document.getElementById('ar-viewer')!
);

await viewer.loadAvatar('https://cdn.example.com/avatar.glb');
await viewer.loadGarment('https://cdn.example.com/dress-medium.glb');

// Add UI controls
document.getElementById('rotate-btn')?.addEventListener('click', () => {
  viewer.rotateGarment(Math.PI);
});

document.getElementById('screenshot-btn')?.addEventListener('click', () => {
  const imageData = viewer.takeScreenshot();
  // Download or share
});
```

### 8.4.2 Mobile AR with AR.js

```html
<!-- ar-tryon-mobile.html -->
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>WIA Fashion AR Try-On</title>
  <script src="https://aframe.io/releases/1.4.0/aframe.min.js"></script>
  <script src="https://cdn.jsdelivr.net/gh/AR-js-org/AR.js/aframe/build/aframe-ar.js"></script>
</head>
<body style="margin: 0; overflow: hidden;">
  <a-scene
    embedded
    arjs="sourceType: webcam; debugUIEnabled: false;"
    vr-mode-ui="enabled: false">

    <!-- Camera -->
    <a-camera gps-camera rotation-reader></a-camera>

    <!-- 3D Garment Model -->
    <a-entity
      id="garment"
      gltf-model="url(https://cdn.example.com/dress-medium.glb)"
      scale="1 1 1"
      position="0 0 -2"
      rotation="0 0 0">
    </a-entity>

    <!-- Lighting -->
    <a-light type="ambient" color="#fff" intensity="0.5"></a-light>
    <a-light type="directional" color="#fff" intensity="0.8" position="1 1 1"></a-light>

  </a-scene>

  <script>
    // Wait for AR.js to initialize
    window.addEventListener('load', () => {
      const garment = document.getElementById('garment');

      // Position garment on detected surface
      garment.addEventListener('model-loaded', () => {
        console.log('Garment model loaded');

        // Auto-rotate
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

## 8.5 Production Best Practices

### 8.5.1 Performance Optimization

**3D Asset Optimization**:

```bash
# Optimize glTF files with gltfpack
gltfpack -i dress-high.glb -o dress-optimized.glb \
  -cc -tc \
  -si 1.0 \
  -vp 14 -vt 12 -vn 8

# This reduces file size by 70-90%
```

**Texture Compression**:

```bash
# Convert textures to KTX2 format (GPU-compressed)
toktx --bcmp --genmipmap \
  dress-basecolor.ktx2 dress-basecolor.png

# Results in 4-8x smaller file size with same visual quality
```

**CDN Configuration**:

```javascript
// Use CDN for 3D assets
const CDN_BASE = 'https://cdn.wiastandards.com/fashion';

const assetUrls = {
  high: `${CDN_BASE}/dress-12345/high.glb`,
  medium: `${CDN_BASE}/dress-12345/medium.glb`,
  low: `${CDN_BASE}/dress-12345/low.glb`
};

// Progressive loading
async function loadGarmentProgressive() {
  // Load low-res first for instant preview
  await loadModel(assetUrls.low);

  // Then load medium-res in background
  loadModel(assetUrls.medium).then(model => {
    replaceModel(model);
  });
}
```

### 8.5.2 Monitoring and Analytics

```typescript
// Track key metrics
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

---

## Review Questions

1. **What are the three LOD levels that should be exported?**
   <details>
   <summary>Answer</summary>
   High (20K-50K polygons), Medium (5K-10K polygons), Low (1K-3K polygons).
   </details>

2. **Which ML algorithm is recommended for size recommendations?**
   <details>
   <summary>Answer</summary>
   XGBoost (Gradient Boosted Trees) for classification with 300 trees, max depth 6.
   </details>

3. **What texture format provides the best compression for WebGL?**
   <details>
   <summary>Answer</summary>
   KTX2 format with Basis Universal compression (4-8x smaller with same quality).
   </details>

4. **Name two JavaScript libraries used for web-based AR.**
   <details>
   <summary>Answer</summary>
   Three.js (3D rendering) and AR.js or A-Frame (AR frameworks).
   </details>

5. **What tool is used to optimize glTF files?**
   <details>
   <summary>Answer</summary>
   gltfpack (can reduce file size by 70-90%).
   </details>

---

## Congratulations!

You've completed the WIA Fashion Tech Standard ebook. You now have the knowledge to:

- ✅ Build digital fashion systems
- ✅ Implement virtual try-on experiences
- ✅ Calculate sustainability metrics
- ✅ Deploy AI recommendation models
- ✅ Integrate with e-commerce and metaverse platforms
- ✅ Create and manage fashion NFTs

### Next Steps

1. **Join the WIA Community**: https://wiastandards.com/community
2. **Get API Access**: https://api.wiastandards.com/signup
3. **Contribute**: https://github.com/WIA-Official/wia-standards
4. **Share Your Implementation**: Tag @WIAStandards on social media

---

© 2025 WIA Standards Committee. 弘익人間 (홍익인간) - Benefit All Humanity
