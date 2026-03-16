# WIA Smart Breeding Integration Standard
## Phase 4 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #84CC16 (Lime - Agriculture)

---

## Table of Contents

1. [Overview](#overview)
2. [Gene Bank Integration](#gene-bank-integration)
3. [Research Institute Integration](#research-institute-integration)
4. [Breeding Company Integration](#breeding-company-integration)
5. [Phenotyping Platform Integration](#phenotyping-platform-integration)
6. [Genomic Service Provider Integration](#genomic-service-provider-integration)
7. [Farm Management System Integration](#farm-management-system-integration)
8. [Government Registry Integration](#government-registry-integration)
9. [AI/ML Model Integration](#aiml-model-integration)
10. [IoT Device Integration](#iot-device-integration)
11. [Interoperability Standards](#interoperability-standards)
12. [Deployment Architecture](#deployment-architecture)

---

## Overview

### 1.1 Purpose

The WIA Smart Breeding Integration Standard defines integration patterns, APIs, and protocols for connecting breeding systems with gene banks, research institutes, breeding companies, phenotyping platforms, genomic service providers, and farm management systems.

**Integration Objectives**:
- Enable seamless data flow between breeding ecosystem partners
- Support standardized interfaces for gene banks (NCBI, EBI, CGIAR)
- Integrate with national/international breeding evaluation systems
- Connect phenotyping platforms for automated data collection
- Link genomic service providers (Illumina, Neogen, Zoetis)
- Support farm management system integration for operational data
- Enable government registry integration for certification

### 1.2 Integration Architecture

```
┌─────────────────────────────────────────────────────────┐
│              WIA Breeding Platform (Hub)                │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐             │
│  │ Genomic  │  │ Pedigree │  │ Phenotype│             │
│  │   Data   │  │   Data   │  │   Data   │             │
│  └──────────┘  └──────────┘  └──────────┘             │
└───────┬──────────┬──────────┬──────────┬──────────┬───┘
        │          │          │          │          │
   ┌────▼───┐  ┌──▼───┐  ┌───▼────┐  ┌──▼────┐  ┌─▼─────┐
   │ Gene   │  │Research│ │Breeding│  │Pheno- │  │ Farm  │
   │ Banks  │  │Institu-│ │Company │  │typing │  │  Mgmt │
   │        │  │tes     │ │        │  │       │  │       │
   └────────┘  └────────┘ └────────┘  └───────┘  └───────┘
```

---

## Gene Bank Integration

### 2.1 NCBI GenBank Integration

**Sequence Upload to GenBank:**

```python
# WIA → NCBI GenBank
import requests

def upload_to_genbank(sequence_data):
    url = "https://www.ncbi.nlm.nih.gov/genbank/submit/"

    payload = {
        "submitter": {
            "organization": "National Livestock Research Institute",
            "email": "breeding@nias.go.kr"
        },
        "sequence": {
            "organism": "Bos taurus",
            "breed": "Holstein",
            "individual_id": "BULL-2025-001",
            "sequence_type": "WHOLE_GENOME",
            "sequence_data": sequence_data,
            "coverage": "30x",
            "sequencing_platform": "Illumina NovaSeq 6000"
        },
        "wia_standard_compliance": {
            "version": "1.0.0",
            "data_format": "PHASE-1",
            "quality_score": 0.98
        }
    }

    response = requests.post(url, json=payload, headers={
        "Authorization": f"Bearer {NCBI_API_KEY}",
        "Content-Type": "application/json"
    })

    return response.json()
```

**GenBank Accession Linkage:**

```json
{
  "individual_id": "BULL-2025-001",
  "genbank_accessions": [
    {
      "accession": "NC_037328.1",
      "chromosome": 1,
      "length": 158534110,
      "submission_date": "2025-01-15"
    }
  ],
  "wia_breeding_id": "did:wia:breeding:BULL-2025-001",
  "cross_reference": "https://www.ncbi.nlm.nih.gov/nuccore/NC_037328.1"
}
```

### 2.2 CGIAR Genebank Platform Integration

**Germplasm Request:**

```http
POST /api/v1/germplasm/request
Host: genebank.cgiar.org
Authorization: Bearer {CGIAR_TOKEN}
Content-Type: application/json

{
  "requestor": {
    "organization": "National Livestock Research Institute",
    "country": "KR",
    "purpose": "BREEDING_RESEARCH"
  },
  "germplasm": {
    "species": "RICE",
    "variety": "Japonica",
    "traits_of_interest": ["DROUGHT_TOLERANCE", "YIELD"],
    "quantity": 100,
    "form": "SEED"
  },
  "material_transfer_agreement": "MTA-2025-001",
  "wia_standard_id": "WIA-AGRI-010"
}
```

---

## Research Institute Integration

### 3.1 Interbull Integration (International Dairy Evaluation)

**Submit Breeding Values to Interbull:**

```json
{
  "submission_type": "NATIONAL_EVALUATION",
  "country": "KR",
  "evaluation_date": "2025-01-15",
  "breed": "HOLSTEIN",
  "trait": "MILK_YIELD",
  "individuals": [
    {
      "national_id": "BULL-2025-001",
      "interbull_id": "KORHOLM000001",
      "ebv": 920,
      "reliability": 0.88,
      "base": "2020",
      "proof_type": "GENOMIC"
    }
  ],
  "genetic_trend": "+45 kg/year",
  "wia_certification": {
    "certified": true,
    "standard_version": "1.0.0",
    "certification_id": "WIA-CERT-2025-001"
  }
}
```

**Receive International EBVs:**

```json
{
  "evaluation_type": "MACE",
  "evaluation_date": "2025-02-01",
  "breed": "HOLSTEIN",
  "trait": "MILK_YIELD",
  "bulls": [
    {
      "interbull_id": "KORHOLM000001",
      "national_id": "BULL-2025-001",
      "mace_ebv": 935,
      "reliability": 0.91,
      "daughter_equivalent": 2500,
      "countries_included": ["KR", "US", "CA", "NZ", "AU"]
    }
  ]
}
```

### 3.2 USDA AGIL (Animal Genomics & Improvement Laboratory)

**Genomic Data Exchange:**

```python
# Submit genomic data to USDA AGIL
def submit_to_agil(individual_data):
    url = "https://agil.barc.usda.gov/api/v1/genomic-data"

    payload = {
        "individual_id": individual_data["id"],
        "species": "CATTLE",
        "genotyping_platform": "Illumina BovineSNP50",
        "genotype_file_url": individual_data["genotype_url"],
        "pedigree": {
            "sire": individual_data["sire"],
            "dam": individual_data["dam"]
        },
        "phenotypes": individual_data["phenotypes"],
        "wia_standard": {
            "version": "1.0.0",
            "data_format_phase": "PHASE-1"
        }
    }

    return requests.post(url, json=payload, headers={
        "Authorization": f"Bearer {AGIL_API_KEY}"
    })
```

---

## Breeding Company Integration

### 4.1 Genus PLC Integration

**Semen Sales Integration:**

```http
POST /api/v1/semen/inventory
Host: api.genusplc.com
Authorization: Bearer {GENUS_TOKEN}

{
  "bull_id": "BULL-2025-001",
  "breed": "HOLSTEIN",
  "gebv": {
    "milk_yield": 920,
    "fat_percentage": 0.15,
    "protein_percentage": 0.08
  },
  "genomic_certified": true,
  "wia_certification_id": "WIA-CERT-2025-001",
  "semen_inventory": {
    "straws_available": 5000,
    "price_per_straw_usd": 25,
    "collection_date": "2025-01-10"
  },
  "marketing_category": "ELITE_GENOMIC",
  "export_eligible_countries": ["US", "CA", "AU", "NZ", "EU"]
}
```

### 4.2 Topigs Norsvin Integration (Pig Breeding)

**Mating Recommendation API:**

```python
# Get optimal mating recommendations
def get_mating_recommendations(farm_id):
    url = "https://api.topigsnorsvin.com/v1/mating/recommendations"

    response = requests.get(url, params={
        "farm_id": farm_id,
        "objective": "BALANCED",
        "max_inbreeding": 0.0625,
        "traits": ["DAILY_GAIN", "BACKFAT", "LITTER_SIZE"]
    }, headers={
        "Authorization": f"Bearer {TOPIGS_API_KEY}"
    })

    return response.json()

# Example response:
{
  "farm_id": "FARM-KR-001",
  "recommendations": [
    {
      "boar_id": "BOAR-2024-123",
      "sow_id": "SOW-2023-456",
      "expected_index": 125,
      "expected_inbreeding": 0.032,
      "recommendation_score": 0.95
    }
  ],
  "wia_compliant": true
}
```

---

## Phenotyping Platform Integration

### 5.1 BREEDPLAN Integration (Cattle Evaluation)

**Submit Performance Data:**

```json
{
  "submission_type": "PERFORMANCE_DATA",
  "breed": "ANGUS",
  "herd_code": "KR-ANGUS-001",
  "animals": [
    {
      "animal_id": "ANGUS-2025-001",
      "birth_date": "2025-01-15",
      "sire_id": "ANGUS-2020-045",
      "dam_id": "ANGUS-2019-123",
      "contemporary_group": "KR_2025_SPRING",
      "traits": [
        {
          "trait_code": "200_DAY_WEIGHT",
          "value": 235,
          "unit": "kg",
          "measurement_date": "2025-07-15"
        },
        {
          "trait_code": "400_DAY_WEIGHT",
          "value": 445,
          "unit": "kg",
          "measurement_date": "2026-02-15"
        }
      ]
    }
  ],
  "wia_standard_version": "1.0.0"
}
```

### 5.2 High-Throughput Phenotyping Integration

**Automated Phenotype Data Stream:**

```python
# Connect to phenotyping platform via WebSocket
import asyncio
import websockets
import json

async def stream_phenotype_data():
    uri = "wss://pheno-platform.wia-breeding.org/stream"

    async with websockets.connect(uri) as websocket:
        # Subscribe to specific traits
        await websocket.send(json.dumps({
            "action": "subscribe",
            "farm_id": "FARM-KR-001",
            "traits": ["BODY_WEIGHT", "ACTIVITY", "FEED_INTAKE"]
        }))

        # Receive real-time data
        async for message in websocket:
            data = json.loads(message)
            print(f"Received: {data}")

            # Example: {"animal_id": "COW-2025-001", "trait": "BODY_WEIGHT",
            #           "value": 645, "timestamp": "2025-01-15T10:30:00Z"}

            # Process and store in WIA format
            process_phenotype(data)

def process_phenotype(data):
    wia_format = {
        "individual_id": data["animal_id"],
        "measurements": [{
            "trait_code": data["trait"],
            "value": data["value"],
            "measurement_date": data["timestamp"],
            "method": "AUTOMATED_SENSOR"
        }],
        "wia_standard_version": "1.0.0"
    }
    # Store to database
    save_to_database(wia_format)
```

---

## Genomic Service Provider Integration

### 6.1 Illumina Agrigenomics Integration

**Order Genotyping Service:**

```http
POST /api/v1/orders
Host: agrigenomics.illumina.com
Authorization: Bearer {ILLUMINA_TOKEN}

{
  "order_id": "WIA-ORDER-2025-001",
  "customer": {
    "organization": "National Livestock Research Institute",
    "country": "KR"
  },
  "service_type": "GENOTYPING",
  "product": "BovineSNP50 v3",
  "samples": [
    {
      "sample_id": "BULL-2025-001",
      "species": "CATTLE",
      "tissue_type": "BLOOD",
      "concentration": "50 ng/μL"
    }
  ],
  "delivery": {
    "data_format": "VCF",
    "delivery_method": "SFTP",
    "wia_standard_compliant": true
  }
}
```

**Receive Genotyping Results:**

```json
{
  "order_id": "WIA-ORDER-2025-001",
  "status": "COMPLETED",
  "completion_date": "2025-01-20",
  "samples": [
    {
      "sample_id": "BULL-2025-001",
      "call_rate": 0.987,
      "total_snps": 54001,
      "data_file_url": "sftp://results.illumina.com/WIA-ORDER-2025-001/BULL-2025-001.vcf.gz",
      "quality_report_url": "https://results.illumina.com/reports/WIA-ORDER-2025-001.pdf"
    }
  ],
  "wia_compliance_verified": true
}
```

### 6.2 Neogen GeneSeek Integration

**Genomic Imputation Service:**

```python
def request_imputation(low_density_genotypes):
    url = "https://api.neogen.com/v1/imputation"

    payload = {
        "project_id": "WIA-IMPUTE-2025-001",
        "input_data": {
            "file_url": "https://data.wia-breeding.org/LD-genotypes.vcf.gz",
            "format": "VCF",
            "marker_count": 10000,
            "individuals": 500
        },
        "imputation_parameters": {
            "target_density": "HD_777K",
            "reference_panel": "CATTLE_HD_REF_2024",
            "imputation_software": "FImpute",
            "quality_threshold": 0.90
        },
        "output_format": "WIA_BREEDING_V1"
    }

    response = requests.post(url, json=payload, headers={
        "Authorization": f"Bearer {NEOGEN_API_KEY}"
    })

    return response.json()
```

---

## Farm Management System Integration

### 6.1 Granular Integration

**Sync Breeding Records:**

```http
POST /api/v1/sync/breeding-records
Host: api.granular.ag
Authorization: Bearer {GRANULAR_TOKEN}

{
  "farm_id": "FARM-KR-001",
  "sync_date": "2025-01-15",
  "breeding_events": [
    {
      "event_type": "ARTIFICIAL_INSEMINATION",
      "cow_id": "COW-2023-101",
      "bull_id": "BULL-2025-001",
      "breeding_date": "2025-01-10",
      "technician": "John Doe",
      "expected_calving_date": "2025-10-18",
      "wia_mating_recommendation": {
        "recommendation_id": "MATE-2025-001",
        "expected_offspring_gebv": 485,
        "expected_inbreeding": 0.032
      }
    }
  ]
}
```

### 6.2 FarmLogs Integration

**Import Phenotype Data:**

```python
def import_from_farmlogs(farm_id, start_date, end_date):
    url = "https://api.farmlogs.com/v1/livestock/weights"

    response = requests.get(url, params={
        "farm_id": farm_id,
        "start_date": start_date,
        "end_date": end_date
    }, headers={
        "Authorization": f"Bearer {FARMLOGS_API_KEY}"
    })

    farmlogs_data = response.json()

    # Convert to WIA format
    wia_phenotypes = []
    for record in farmlogs_data["weights"]:
        wia_phenotypes.append({
            "individual_id": record["animal_id"],
            "measurements": [{
                "trait_code": "BODY_WEIGHT",
                "value": record["weight_kg"],
                "measurement_date": record["date"],
                "method": "SCALE"
            }],
            "source": "FARMLOGS",
            "wia_standard_version": "1.0.0"
        })

    return wia_phenotypes
```

---

## Government Registry Integration

### 7.1 National Livestock Registry

**Register New Individual:**

```http
POST /api/v1/registry/register
Host: registry.livestock.go.kr
Authorization: Bearer {REGISTRY_TOKEN}

{
  "registration_type": "BIRTH",
  "individual": {
    "individual_id": "BULL-2025-001",
    "species": "CATTLE",
    "breed": "HOLSTEIN",
    "sex": "MALE",
    "birth_date": "2025-01-15",
    "farm_id": "FARM-KR-001"
  },
  "pedigree": {
    "sire_registration_number": "HOLKOR-2020-000045",
    "dam_registration_number": "HOLKOR-2018-000123"
  },
  "genomic_certification": {
    "certified": true,
    "wia_certification_id": "WIA-CERT-2025-001",
    "genomic_verified": true
  }
}
```

**Response:**

```json
{
  "registration_number": "HOLKOR-2025-001234",
  "individual_id": "BULL-2025-001",
  "registration_date": "2025-01-15",
  "status": "REGISTERED",
  "certificate_url": "https://registry.livestock.go.kr/certificates/HOLKOR-2025-001234.pdf",
  "blockchain_tx": "0xabcdef123456..."
}
```

---

## AI/ML Model Integration

### 8.1 TensorFlow Genomic Prediction Model

**Deploy Genomic Prediction Model:**

```python
import tensorflow as tf
import numpy as np

# Load WIA-compliant genomic data
def load_wia_genotypes(individual_ids):
    # Fetch from WIA API
    genotypes = fetch_genotypes(individual_ids)

    # Convert to matrix format
    # Shape: (n_individuals, n_markers)
    G = np.array([[snp["genotype"] for snp in ind["markers"]]
                  for ind in genotypes])

    return G

# Train genomic prediction model
def train_gebv_model(genotypes, phenotypes):
    model = tf.keras.Sequential([
        tf.keras.layers.Dense(512, activation='relu', input_shape=(50000,)),
        tf.keras.layers.Dropout(0.3),
        tf.keras.layers.Dense(256, activation='relu'),
        tf.keras.layers.Dropout(0.3),
        tf.keras.layers.Dense(128, activation='relu'),
        tf.keras.layers.Dense(1)  # GEBV output
    ])

    model.compile(optimizer='adam', loss='mse', metrics=['mae'])
    model.fit(genotypes, phenotypes, epochs=100, batch_size=32, validation_split=0.2)

    return model

# Predict GEBVs for new individuals
def predict_gebv(model, individual_id):
    genotype = load_wia_genotypes([individual_id])
    gebv = model.predict(genotype)[0][0]

    # Return in WIA format
    return {
        "individual_id": individual_id,
        "method": "DEEP_LEARNING",
        "gebv": float(gebv),
        "model_version": "DL-v1.0",
        "wia_standard_version": "1.0.0"
    }
```

### 8.2 PyTorch Phenotype Prediction from Images

**Cattle Body Condition Scoring:**

```python
import torch
import torchvision.models as models
from PIL import Image

# Load pre-trained model
model = models.resnet50(pretrained=True)
model.fc = torch.nn.Linear(2048, 9)  # 9 body condition scores (1-9)
model.load_state_dict(torch.load('bcs_model.pth'))
model.eval()

def predict_body_condition_score(image_path, individual_id):
    # Load and preprocess image
    image = Image.open(image_path)
    transform = torchvision.transforms.Compose([
        torchvision.transforms.Resize(256),
        torchvision.transforms.CenterCrop(224),
        torchvision.transforms.ToTensor(),
        torchvision.transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                         std=[0.229, 0.224, 0.225])
    ])
    image_tensor = transform(image).unsqueeze(0)

    # Predict BCS
    with torch.no_grad():
        output = model(image_tensor)
        bcs = torch.argmax(output, dim=1).item() + 1  # 1-9 scale

    # Return in WIA format
    return {
        "individual_id": individual_id,
        "measurements": [{
            "trait_code": "BODY_CONDITION_SCORE",
            "value": bcs,
            "unit": "BCS_1_9_SCALE",
            "measurement_date": datetime.now().isoformat(),
            "method": "AI_IMAGE_ANALYSIS",
            "confidence": 0.92
        }],
        "wia_standard_version": "1.0.0"
    }
```

---

## IoT Device Integration

### 9.1 Smart Collar Integration

**Allflex SenseHub Integration:**

```python
# Connect to Allflex SenseHub API
def stream_collar_data(farm_id):
    url = "wss://api.allflex.com/v1/stream"

    async with websockets.connect(url) as websocket:
        await websocket.send(json.dumps({
            "farm_id": farm_id,
            "data_types": ["ACTIVITY", "RUMINATION", "ESTRUS"]
        }))

        async for message in websocket:
            data = json.loads(message)

            # Convert to WIA format
            wia_phenotype = {
                "individual_id": data["animal_id"],
                "measurements": [
                    {
                        "trait_code": "ACTIVITY_INDEX",
                        "value": data["activity"],
                        "unit": "index",
                        "measurement_date": data["timestamp"]
                    },
                    {
                        "trait_code": "RUMINATION_TIME",
                        "value": data["rumination_minutes"],
                        "unit": "minutes",
                        "measurement_date": data["timestamp"]
                    }
                ],
                "sensor_id": data["collar_id"],
                "wia_standard_version": "1.0.0"
            }

            # Send to WIA platform
            post_phenotype(wia_phenotype)
```

---

## Interoperability Standards

### 10.1 Format Conversion Matrix

| Source Format | Target Format | Conversion Library |
|---------------|---------------|-------------------|
| VCF | PLINK (PED/MAP) | `bcftools`, `plink` |
| PLINK | VCF | `plink --recode vcf` |
| HapMap | VCF | Custom parser |
| WIA JSON | VCF | `wia-to-vcf` library |
| VCF | WIA JSON | `vcf-to-wia` library |

### 10.2 API Gateway Pattern

```
┌────────────────────────────────────────┐
│         WIA API Gateway                │
│  (Protocol Translation, Routing)       │
└────────────────┬───────────────────────┘
                 │
     ┌───────────┼───────────┐
     │           │           │
┌────▼────┐ ┌───▼────┐ ┌───▼─────┐
│ Interbull│ │ NCBI   │ │ Illumina│
│ (Custom) │ │(REST)  │ │ (SOAP)  │
└──────────┘ └────────┘ └─────────┘
```

---

## Deployment Architecture

### 11.1 Cloud Deployment (AWS)

```yaml
# Kubernetes deployment for WIA Breeding Platform
apiVersion: apps/v1
kind: Deployment
metadata:
  name: wia-breeding-api
spec:
  replicas: 3
  selector:
    matchLabels:
      app: wia-breeding-api
  template:
    metadata:
      labels:
        app: wia-breeding-api
    spec:
      containers:
      - name: api
        image: wia/breeding-api:1.0.0
        ports:
        - containerPort: 8080
        env:
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: db-secret
              key: url
        - name: NCBI_API_KEY
          valueFrom:
            secretKeyRef:
              name: external-apis
              key: ncbi-key
```

---

**© 2025 WIA (World Certification Industry Association)**
**弘益人間 · Benefit All Humanity**
**License**: MIT
