# WIA-HERITAGE-001: Phase 4 - Integration Specification

> 弘益人間 (Benefit All Humanity)

## Overview

Phase 4 defines integration strategies for museum management systems, digital asset management platforms, virtual reality applications, and public access portals.

## 1. Museum Management Systems (TMS/DAMS)

### 1.1 Gallery Systems TMS Integration

```javascript
// TMS API Integration
const tmsConfig = {
  endpoint: 'https://museum.tms.org/api/v5',
  apiKey: process.env.TMS_API_KEY,
  department: 'Ancient_Art',
  sync: 'bidirectional'
};

// Sync artifact to TMS
async function syncToTMS(artifact) {
  const tmsObject = {
    accessionNumber: artifact.id,
    title: artifact.name,
    objectName: artifact.classification.category,
    dated: artifact.dating.period,
    medium: artifact.materials.map(m => m.type).join(', '),
    dimensions: `H: ${artifact.dimensions.height}mm`,
    department: 'Ancient Mediterranean',
    creditLine: artifact.provenance[0]?.owner?.name,
    currentLocation: artifact.location?.institution,
    webDisplay: true,
    publicAccess: true,
    customFields: {
      wia_artifact_id: artifact.id,
      wia_model_url: artifact.digitization.models[0]?.url,
      wia_3d_viewer: `https://wia.org/view/${artifact.id}`
    }
  };

  await tms.objects.create(tmsObject);
}
```

### 1.2 CollectionsSpace Integration

```xml
<!-- CollectionSpace XML Record -->
<document name="collectionobjects">
  <ns2:collectionobjects_common>
    <objectNumber>art-001</objectNumber>
    <otherNumberList>
      <otherNumber>
        <numberValue>WIA-ART-2025-001</numberValue>
        <numberType>WIA Heritage ID</numberType>
      </otherNumber>
    </otherNumberList>
    <title>Ancient Greek Amphora</title>
    <objectNameList>
      <objectNameGroup>
        <objectName>Amphora</objectName>
      </objectNameGroup>
    </objectNameList>
    <briefDescriptions>
      <briefDescription>Red-figure amphora</briefDescription>
    </briefDescriptions>
    <wia_heritage>
      <model3d>https://wia.org/models/art-001.glb</model3d>
      <viewerUrl>https://wia.org/view/art-001</viewerUrl>
      <iiifManifest>https://iiif.wia.org/art-001/manifest.json</iiifManifest>
    </wia_heritage>
  </ns2:collectionobjects_common>
</document>
```

## 2. Digital Asset Management (DAM)

### 2.1 Adobe Experience Manager Integration

```javascript
// AEM DAM Integration
const aemConfig = {
  host: 'https://museum.aem.adobe.com',
  repository: '/content/dam/heritage',
  workflow: 'heritage-asset-processing'
};

async function uploadToAEM(artifact, files) {
  const assetPath = `/content/dam/heritage/${artifact.id}`;

  // Create folder structure
  await aem.createFolder(assetPath);

  // Upload files
  for (const file of files) {
    const metadata = {
      'dc:title': artifact.name,
      'dc:description': artifact.description,
      'wia:artifactId': artifact.id,
      'wia:period': artifact.period,
      'dam:size': file.size,
      'dam:MIMEtype': file.mimeType
    };

    await aem.assets.upload(file.path, `${assetPath}/${file.name}`, metadata);
  }

  // Trigger processing workflow
  await aem.workflows.start('heritage-asset-processing', assetPath);
}
```

### 2.2 Bynder Integration

```yaml
bynder_integration:
  api_endpoint: https://museum.bynder.com/api/v4
  collections:
    - name: Cultural Artifacts
      id: col-001
      public: true

  metadata_mapping:
    wia_id: property_artifact_id
    period: property_period
    origin: property_origin
    3d_model: property_model_url

  automation:
    - trigger: artifact_created
      action: create_asset
    - trigger: model_generated
      action: attach_rendition
```

## 3. Virtual Reality Platforms

### 3.1 Unity Integration

```csharp
// Unity Heritage Viewer
using UnityEngine;
using GLTFast;
using WIA.Heritage;

public class HeritageArtifactLoader : MonoBehaviour
{
    private string wiaApiEndpoint = "https://api.wia.org/heritage/v1";
    private string apiKey;

    async void LoadArtifact(string artifactId)
    {
        // Fetch artifact metadata
        var client = new WIAHeritageClient(wiaApiEndpoint, apiKey);
        var artifact = await client.GetArtifact(artifactId);

        // Load 3D model (glTF)
        var gltf = gameObject.AddComponent<GltfAsset>();
        var modelUrl = artifact.Digitization.Models
            .FirstOrDefault(m => m.Quality == "web")?.Url;

        await gltf.Load(modelUrl);

        // Add metadata UI
        var infoPanel = CreateInfoPanel(artifact);
        infoPanel.SetActive(true);

        // Add interaction
        gameObject.AddComponent<ArtifactInteraction>();
    }

    GameObject CreateInfoPanel(Artifact artifact)
    {
        var panel = new GameObject("InfoPanel");
        var text = panel.AddComponent<TextMeshProUGUI>();
        text.text = $@"
            <b>{artifact.Name}</b>
            Period: {artifact.Period}
            Origin: {artifact.Origin.Country}
            Material: {string.Join(", ", artifact.Materials.Select(m => m.Type))}
        ";
        return panel;
    }
}
```

### 3.2 Unreal Engine Integration

```cpp
// Unreal Engine C++ Integration
#include "HeritageArtifactActor.h"
#include "glTFRuntimeAsset.h"
#include "HttpModule.h"

AHeritageArtifactActor::AHeritageArtifactActor()
{
    PrimaryActorTick.bCanEverTick = true;
}

void AHeritageArtifactActor::LoadArtifact(FString ArtifactID)
{
    FString ApiUrl = FString::Printf(
        TEXT("https://api.wia.org/heritage/v1/artifacts/%s"),
        *ArtifactID
    );

    TSharedRef<IHttpRequest> Request = FHttpModule::Get()
        .CreateRequest();
    Request->SetVerb("GET");
    Request->SetURL(ApiUrl);
    Request->SetHeader("Authorization", FString("Bearer ") + ApiKey);
    Request->OnProcessRequestComplete().BindUObject(
        this, &AHeritageArtifactActor::OnArtifactLoaded
    );
    Request->ProcessRequest();
}

void AHeritageArtifactActor::OnArtifactLoaded(
    FHttpRequestPtr Request,
    FHttpResponsePtr Response,
    bool bWasSuccessful)
{
    if (bWasSuccessful)
    {
        TSharedPtr<FJsonObject> JsonObject;
        TSharedRef<TJsonReader<>> Reader =
            TJsonReaderFactory<>::Create(Response->GetContentAsString());

        if (FJsonSerializer::Deserialize(Reader, JsonObject))
        {
            FString ModelUrl = JsonObject->GetStringField("modelUrl");
            LoadGLTFModel(ModelUrl);
        }
    }
}
```

### 3.3 WebXR Integration

```javascript
// Three.js + WebXR Heritage Viewer
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { VRButton } from 'three/examples/jsm/webxr/VRButton.js';

class HeritageViewer {
  constructor(containerId, artifactId) {
    this.container = document.getElementById(containerId);
    this.artifactId = artifactId;
    this.init();
  }

  async init() {
    // Setup scene
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(75,
      window.innerWidth / window.innerHeight, 0.1, 1000);
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.xr.enabled = true;

    this.container.appendChild(this.renderer.domElement);
    this.container.appendChild(VRButton.createButton(this.renderer));

    // Load artifact
    const artifact = await this.fetchArtifact(this.artifactId);
    await this.loadModel(artifact.digitization.models[0].url);

    // Add lighting
    const light = new THREE.DirectionalLight(0xffffff, 1);
    light.position.set(5, 10, 7.5);
    this.scene.add(light);
    this.scene.add(new THREE.AmbientLight(0x404040));

    // Start rendering
    this.renderer.setAnimationLoop(() => this.render());
  }

  async fetchArtifact(id) {
    const response = await fetch(
      `https://api.wia.org/heritage/v1/artifacts/${id}`,
      { headers: { 'Authorization': `Bearer ${this.apiKey}` }}
    );
    return response.json();
  }

  async loadModel(url) {
    const loader = new GLTFLoader();
    const gltf = await loader.loadAsync(url);
    this.scene.add(gltf.scene);
  }

  render() {
    this.renderer.render(this.scene, this.camera);
  }
}
```

## 4. Public Access Portals

### 4.1 Europeana Integration

```json
{
  "@context": "http://www.europeana.eu/schemas/edm/",
  "@id": "http://data.europeana.eu/item/wia/art-001",
  "@type": "edm:ProvidedCHO",
  "dc:title": "Ancient Greek Amphora",
  "dc:creator": "Unknown",
  "dc:date": "-450/-400",
  "dc:type": "PhysicalObject",
  "dcterms:medium": "Terracotta",
  "dcterms:extent": "H: 425mm",
  "edm:type": "3D",
  "edm:rights": "http://creativecommons.org/publicdomain/mark/1.0/",
  "edm:isShownAt": "https://wia.org/artifact/art-001",
  "edm:isShownBy": "https://iiif.wia.org/art-001/full/full/0/default.jpg",
  "edm:object": "https://wia.org/models/art-001.glb",
  "svcs:has_service": "https://iiif.wia.org/art-001/info.json"
}
```

### 4.2 Digital Public Library of America (DPLA)

```json
{
  "@context": "http://dp.la/api/items/context",
  "id": "wia--art-001",
  "sourceResource": {
    "title": "Ancient Greek Amphora",
    "creator": ["Unknown"],
    "date": {
      "displayDate": "450-400 BCE",
      "begin": "-450",
      "end": "-400"
    },
    "description": "Red-figure amphora depicting Dionysus",
    "type": "image",
    "format": ["Ceramic", "Terracotta"],
    "subject": [
      {"name": "Pottery"},
      {"name": "Ancient Greece"},
      {"name": "Classical Period"}
    ],
    "spatial": [
      {"name": "Athens, Greece"}
    ],
    "rights": "http://creativecommons.org/publicdomain/mark/1.0/"
  },
  "isShownAt": "https://wia.org/artifact/art-001",
  "object": "https://iiif.wia.org/art-001/full/400,/0/default.jpg",
  "provider": {
    "name": "WIA Heritage Standards"
  },
  "dataProvider": "National Archaeological Museum"
}
```

## 5. Social Media Integration

### 5.1 Open Graph Protocol

```html
<meta property="og:type" content="article" />
<meta property="og:title" content="Ancient Greek Amphora" />
<meta property="og:description" content="Red-figure amphora from 450-400 BCE" />
<meta property="og:image" content="https://iiif.wia.org/art-001/full/1200,/0/default.jpg" />
<meta property="og:url" content="https://wia.org/artifact/art-001" />
<meta property="og:site_name" content="WIA Heritage" />

<!-- 3D Model Meta -->
<meta property="og:type" content="website.3d" />
<meta property="og:model" content="https://wia.org/models/art-001.glb" />
<meta property="og:model:type" content="model/gltf-binary" />
```

### 5.2 Twitter Cards

```html
<meta name="twitter:card" content="summary_large_image" />
<meta name="twitter:title" content="Ancient Greek Amphora" />
<meta name="twitter:description" content="Explore this 3D digitized artifact" />
<meta name="twitter:image" content="https://iiif.wia.org/art-001/full/1200,/0/default.jpg" />
<meta name="twitter:player" content="https://wia.org/embed/art-001" />
<meta name="twitter:player:width" content="800" />
<meta name="twitter:player:height" content="600" />
```

## 6. Analytics & Monitoring

### 6.1 Usage Analytics

```javascript
// Google Analytics 4 Integration
gtag('event', 'view_artifact', {
  artifact_id: 'art-001',
  artifact_name: 'Ancient Greek Amphora',
  period: 'Classical Period',
  viewer_type: '3d',
  engagement_time_msec: 45000
});

// Custom dimensions
gtag('config', 'GA_MEASUREMENT_ID', {
  custom_map: {
    dimension1: 'artifact_period',
    dimension2: 'artifact_origin',
    metric1: 'model_load_time'
  }
});
```

### 6.2 Performance Monitoring

```yaml
monitoring:
  apm: New Relic / Datadog
  metrics:
    - model_load_time
    - texture_streaming_bandwidth
    - api_response_time
    - user_engagement_duration

  alerts:
    - condition: model_load_time > 10s
      action: notify_ops_team
    - condition: api_error_rate > 5%
      action: escalate
```

## 7. Content Delivery Network (CDN)

### 7.1 CDN Configuration

```yaml
cdn_provider: Cloudflare / AWS CloudFront

cache_rules:
  3d_models:
    path: /models/*
    ttl: 7 days
    compression: brotli

  images:
    path: /images/*
    ttl: 30 days
    optimization: auto

  api:
    path: /api/*
    ttl: 5 minutes
    bypass: authenticated_users

edge_locations:
  - us-east-1
  - eu-west-1
  - ap-southeast-1
```

## 8. Backup & Disaster Recovery

```yaml
backup_strategy:
  primary:
    provider: AWS S3
    region: us-east-1
    versioning: enabled
    lifecycle:
      - archive_after: 90 days (Glacier)
      - delete_after: 7 years

  secondary:
    provider: Backblaze B2
    region: eu-central-1
    replication: cross-region

  archival:
    provider: Arweave
    permanence: true
    redundancy: 200+ nodes

disaster_recovery:
  rto: 4 hours
  rpo: 1 hour
  automated_failover: true
  backup_frequency: hourly
```

---

**Phase 4 Compliance**: All implementations must support at least one museum system integration, IIIF manifest generation, and public access portal compatibility.

**Complete Implementation**: All 4 phases implemented = Full WIA-HERITAGE-001 compliance

---

弘益人間 (Benefit All Humanity)
© 2025 SmileStory Inc. / WIA
