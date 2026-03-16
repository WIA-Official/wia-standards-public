# WIA-AGRI-025: Insect Protein Standard
## PHASE 4: Integration Specification

**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-12-26
**Standard ID:** WIA-AGRI-025

---

## 1. Overview

This specification provides comprehensive integration guides, SDK documentation, code examples, and testing frameworks for implementing the WIA-AGRI-025 Insect Protein Standard in production systems.

---

## 2. SDK Libraries

### 2.1 TypeScript/JavaScript SDK

**Installation:**
```bash
npm install @wia/agri-025
# or
yarn add @wia/agri-025
```

**Basic Usage:**
```typescript
import { WIAInsectProtein } from '@wia/agri-025';

// Initialize client
const client = new WIAInsectProtein({
  apiKey: process.env.WIA_API_KEY,
  environment: 'production' // or 'staging', 'development'
});

// Create a product
const product = await client.products.create({
  species: {
    commonName: 'House Cricket',
    scientificName: 'Acheta domesticus'
  },
  productForm: 'powder',
  batchNumber: 'BATCH-2025-001',
  productionDate: new Date('2025-12-20'),
  farmInfo: {
    farmId: 'FARM-KR-001',
    farmName: 'Seoul Cricket Farm'
  },
  nutrition: {
    macronutrients: {
      protein: { value: 65.5, unit: 'g/100g' },
      fat: { value: 15.2, unit: 'g/100g' }
    }
  }
});

console.log('Product created:', product.id);

// Query products
const products = await client.products.list({
  species: 'cricket',
  certified: true,
  minProtein: 60,
  page: 1,
  limit: 10
});

for (const p of products.data) {
  console.log(`${p.batchNumber}: ${p.nutrition.macronutrients.protein.value}% protein`);
}

// Get detailed nutrition
const nutrition = await client.nutrition.get('PRODUCT-001');
console.log('Amino acids:', nutrition.aminoAcids);

// Subscribe to real-time updates
client.on('product.created', (data) => {
  console.log('New product:', data.productId);
});

client.on('certification.issued', (data) => {
  console.log('New certification:', data.certificationId);
});
```

**Advanced Features:**
```typescript
// Batch operations
const products = await client.products.batchCreate([
  { /* product 1 */ },
  { /* product 2 */ },
  { /* product 3 */ }
]);

// Nutrition comparison
const comparison = await client.nutrition.compare([
  'PRODUCT-001',
  'PRODUCT-002',
  'PRODUCT-003'
]);

// Safety alerts
await client.safety.reportAlert({
  productId: 'PRODUCT-001',
  alertType: 'contamination',
  severity: 'high',
  description: 'Batch contamination detected',
  affectedBatches: ['BATCH-2025-001']
});

// Certification request
const certRequest = await client.certifications.request({
  productId: 'PRODUCT-001',
  requestedBy: {
    name: 'John Doe',
    email: 'john@example.com'
  }
});
```

### 2.2 Python SDK

**Installation:**
```bash
pip install wia-agri-025
```

**Basic Usage:**
```python
from wia_agri_025 import WIAInsectProtein
from datetime import datetime

# Initialize client
client = WIAInsectProtein(
    api_key=os.environ['WIA_API_KEY'],
    environment='production'
)

# Create a product
product = client.products.create(
    species={
        'commonName': 'House Cricket',
        'scientificName': 'Acheta domesticus'
    },
    product_form='powder',
    batch_number='BATCH-2025-001',
    production_date=datetime(2025, 12, 20),
    farm_info={
        'farmId': 'FARM-KR-001',
        'farmName': 'Seoul Cricket Farm'
    },
    nutrition={
        'macronutrients': {
            'protein': {'value': 65.5, 'unit': 'g/100g'},
            'fat': {'value': 15.2, 'unit': 'g/100g'}
        }
    }
)

print(f'Product created: {product.id}')

# Query products
products = client.products.list(
    species='cricket',
    certified=True,
    min_protein=60,
    limit=10
)

for p in products:
    print(f'{p.batch_number}: {p.nutrition.protein.value}% protein')

# Async support
import asyncio

async def main():
    async with WIAInsectProtein(api_key='...') as client:
        product = await client.products.get('PRODUCT-001')
        nutrition = await client.nutrition.get(product.id)
        print(f'Protein: {nutrition.macronutrients.protein.value}%')

asyncio.run(main())
```

**Data Analysis with Pandas:**
```python
import pandas as pd
from wia_agri_025 import WIAInsectProtein

client = WIAInsectProtein(api_key='...')

# Get all products
products = client.products.list(limit=100)

# Convert to DataFrame
df = pd.DataFrame([{
    'id': p.id,
    'species': p.species.common_name,
    'form': p.product_form,
    'protein': p.nutrition.macronutrients.protein.value,
    'carbon_footprint': p.sustainability.carbon_footprint.total.value
} for p in products])

# Analysis
print(df.groupby('species')['protein'].mean())
print(df.groupby('form')['carbon_footprint'].mean())

# Visualization
import matplotlib.pyplot as plt
df.plot(x='species', y='protein', kind='bar')
plt.show()
```

### 2.3 Go SDK

**Installation:**
```bash
go get github.com/wia-official/agri-025-go
```

**Basic Usage:**
```go
package main

import (
    "context"
    "fmt"
    "log"
    "time"

    wia "github.com/wia-official/agri-025-go"
)

func main() {
    // Initialize client
    client := wia.NewClient(os.Getenv("WIA_API_KEY"))

    ctx := context.Background()

    // Create a product
    product, err := client.Products.Create(ctx, &wia.CreateProductRequest{
        Species: wia.Species{
            CommonName:     "House Cricket",
            ScientificName: "Acheta domesticus",
        },
        ProductForm:    wia.ProductFormPowder,
        BatchNumber:    "BATCH-2025-001",
        ProductionDate: time.Now(),
        FarmInfo: wia.FarmInfo{
            FarmID:   "FARM-KR-001",
            FarmName: "Seoul Cricket Farm",
        },
        Nutrition: wia.NutritionProfile{
            Macronutrients: wia.Macronutrients{
                Protein: wia.NutrientValue{
                    Value: 65.5,
                    Unit:  "g/100g",
                },
            },
        },
    })
    if err != nil {
        log.Fatal(err)
    }

    fmt.Printf("Product created: %s\n", product.ID)

    // Query products
    products, err := client.Products.List(ctx, &wia.ListProductsRequest{
        Species:    "cricket",
        Certified:  true,
        MinProtein: 60,
        Limit:      10,
    })
    if err != nil {
        log.Fatal(err)
    }

    for _, p := range products.Data {
        fmt.Printf("%s: %.1f%% protein\n",
            p.BatchNumber,
            p.Nutrition.Macronutrients.Protein.Value)
    }

    // Get nutrition details
    nutrition, err := client.Nutrition.Get(ctx, "PRODUCT-001")
    if err != nil {
        log.Fatal(err)
    }

    fmt.Printf("Amino acids: %+v\n", nutrition.AminoAcids)
}
```

**Concurrent Processing:**
```go
package main

import (
    "context"
    "sync"
    wia "github.com/wia-official/agri-025-go"
)

func processProducts(client *wia.Client, productIDs []string) {
    ctx := context.Background()
    var wg sync.WaitGroup

    for _, id := range productIDs {
        wg.Add(1)
        go func(productID string) {
            defer wg.Done()

            product, err := client.Products.Get(ctx, productID)
            if err != nil {
                log.Printf("Error getting product %s: %v", productID, err)
                return
            }

            nutrition, err := client.Nutrition.Get(ctx, productID)
            if err != nil {
                log.Printf("Error getting nutrition %s: %v", productID, err)
                return
            }

            fmt.Printf("%s: %.1f%% protein\n",
                product.BatchNumber,
                nutrition.Macronutrients.Protein.Value)
        }(id)
    }

    wg.Wait()
}
```

---

## 3. Integration Patterns

### 3.1 Farm Management System Integration

**Scenario:** Integrate insect protein tracking into an existing farm management system.

```typescript
import { WIAInsectProtein } from '@wia/agri-025';

class FarmManagementSystem {
  private wiaClient: WIAInsectProtein;

  constructor(apiKey: string) {
    this.wiaClient = new WIAInsectProtein({ apiKey });
  }

  async recordHarvest(harvestData: HarvestData) {
    // Create batch in farm system
    const batch = await this.createInternalBatch(harvestData);

    // Register with WIA
    const product = await this.wiaClient.products.create({
      species: {
        commonName: harvestData.speciesName,
        scientificName: harvestData.scientificName
      },
      batchNumber: batch.id,
      productionDate: harvestData.harvestDate,
      farmInfo: {
        farmId: this.farmId,
        farmName: this.farmName
      }
    });

    // Link internal batch to WIA product
    await this.linkBatchToWIA(batch.id, product.id);

    return product;
  }

  async updateNutritionData(batchId: string, labResults: LabResults) {
    const wiaProductId = await this.getWIAProductId(batchId);

    await this.wiaClient.products.update(wiaProductId, {
      nutrition: {
        macronutrients: labResults.macronutrients,
        aminoAcids: labResults.aminoAcids,
        vitamins: labResults.vitamins,
        minerals: labResults.minerals
      }
    });

    // Update internal records
    await this.updateInternalNutrition(batchId, labResults);
  }
}
```

### 3.2 E-Commerce Platform Integration

**Scenario:** Display insect protein product information on an e-commerce website.

```typescript
import { WIAInsectProtein } from '@wia/agri-025';

class ProductCatalog {
  private wiaClient: WIAInsectProtein;

  async getProductDetails(productId: string) {
    const [product, nutrition, certification] = await Promise.all([
      this.wiaClient.products.get(productId),
      this.wiaClient.nutrition.get(productId),
      this.wiaClient.certifications.getByProduct(productId)
    ]);

    return {
      name: `${product.species.commonName} ${product.productForm}`,
      description: this.generateDescription(product),
      nutrition: this.formatNutrition(nutrition),
      certifications: certification.map(c => c.certificationBody.name),
      sustainability: this.formatSustainability(product.sustainability),
      batchNumber: product.batchNumber,
      origin: product.farmInfo.location.country
    };
  }

  private generateDescription(product): string {
    return `
      Premium ${product.species.commonName} protein in ${product.productForm} form.
      Sustainably farmed with ${product.sustainability.carbonFootprint.total.value} kg CO2e/kg.
      High protein content: ${product.nutrition.macronutrients.protein.value}g per 100g.
    `;
  }
}
```

### 3.3 Certification Body Integration

**Scenario:** Automate certification workflow.

```python
from wia_agri_025 import WIAInsectProtein

class CertificationWorkflow:
    def __init__(self, api_key):
        self.client = WIAInsectProtein(api_key=api_key)

    def process_certification_request(self, product_id):
        # Get product data
        product = self.client.products.get(product_id)

        # Verify all required data is present
        if not self.verify_complete_data(product):
            return {'status': 'incomplete', 'message': 'Missing required data'}

        # Check safety compliance
        safety = self.client.safety.get(product_id)
        if not self.check_safety_compliance(safety):
            return {'status': 'failed', 'message': 'Safety tests failed'}

        # Verify sustainability metrics
        sustainability = self.client.sustainability.get(product_id)
        if not self.check_sustainability_standards(sustainability):
            return {'status': 'failed', 'message': 'Sustainability requirements not met'}

        # Issue certification
        certification = self.client.certifications.issue({
            'productId': product_id,
            'certificationBody': {
                'name': 'WIA Certification Authority',
                'accreditation': 'ISO/IEC 17065'
            },
            'scope': 'Full product certification',
            'validityPeriod': 365  # days
        })

        return {'status': 'certified', 'certificationId': certification.id}

    def verify_complete_data(self, product):
        required_fields = [
            'species',
            'batchNumber',
            'nutrition.macronutrients.protein',
            'safety.microbiological',
            'sustainability.carbonFootprint'
        ]
        return all(self.has_field(product, field) for field in required_fields)
```

---

## 4. Testing Framework

### 4.1 Unit Testing

**TypeScript (Jest):**
```typescript
import { WIAInsectProtein } from '@wia/agri-025';
import { MockWIAClient } from '@wia/agri-025/testing';

describe('Product Management', () => {
  let client: WIAInsectProtein;

  beforeEach(() => {
    client = new MockWIAClient();
  });

  test('should create product successfully', async () => {
    const product = await client.products.create({
      species: {
        commonName: 'House Cricket',
        scientificName: 'Acheta domesticus'
      },
      productForm: 'powder',
      batchNumber: 'TEST-BATCH-001'
    });

    expect(product.id).toBeDefined();
    expect(product.species.scientificName).toBe('Acheta domesticus');
  });

  test('should validate protein content range', async () => {
    await expect(
      client.products.create({
        nutrition: {
          macronutrients: {
            protein: { value: 150, unit: 'g/100g' } // Invalid
          }
        }
      })
    ).rejects.toThrow('Protein content must be between 0 and 100');
  });
});
```

**Python (pytest):**
```python
import pytest
from wia_agri_025 import WIAInsectProtein
from wia_agri_025.testing import MockWIAClient

@pytest.fixture
def client():
    return MockWIAClient()

def test_create_product(client):
    product = client.products.create(
        species={'commonName': 'House Cricket', 'scientificName': 'Acheta domesticus'},
        product_form='powder',
        batch_number='TEST-BATCH-001'
    )

    assert product.id is not None
    assert product.species.scientific_name == 'Acheta domesticus'

def test_protein_content_validation(client):
    with pytest.raises(ValueError, match='Protein content must be between 0 and 100'):
        client.products.create(
            nutrition={'macronutrients': {'protein': {'value': 150, 'unit': 'g/100g'}}}
        )
```

### 4.2 Integration Testing

**End-to-End Test:**
```typescript
import { WIAInsectProtein } from '@wia/agri-025';

describe('Full Workflow Integration', () => {
  let client: WIAInsectProtein;

  beforeAll(() => {
    client = new WIAInsectProtein({
      apiKey: process.env.WIA_TEST_API_KEY,
      environment: 'staging'
    });
  });

  test('complete product lifecycle', async () => {
    // 1. Create product
    const product = await client.products.create({
      species: { commonName: 'House Cricket', scientificName: 'Acheta domesticus' },
      productForm: 'powder',
      batchNumber: `TEST-${Date.now()}`
    });

    expect(product.id).toBeDefined();

    // 2. Add nutrition data
    await client.products.update(product.id, {
      nutrition: {
        macronutrients: {
          protein: { value: 65.5, unit: 'g/100g' }
        }
      }
    });

    // 3. Request certification
    const certRequest = await client.certifications.request({
      productId: product.id
    });

    expect(certRequest.status).toBe('pending');

    // 4. Verify product data
    const updatedProduct = await client.products.get(product.id);
    expect(updatedProduct.nutrition.macronutrients.protein.value).toBe(65.5);

    // 5. Cleanup
    await client.products.delete(product.id);
  });
});
```

### 4.3 Performance Testing

**Load Test (Artillery):**
```yaml
config:
  target: 'https://api.wia-agri.org'
  phases:
    - duration: 60
      arrivalRate: 10
      name: "Warm up"
    - duration: 300
      arrivalRate: 50
      name: "Sustained load"
  defaults:
    headers:
      Authorization: "Bearer {{ $env.WIA_API_KEY }}"
      Content-Type: "application/json"

scenarios:
  - name: "List and view products"
    flow:
      - get:
          url: "/v1/products?limit=20"
      - think: 2
      - get:
          url: "/v1/products/{{ productId }}"
      - get:
          url: "/v1/nutrition/{{ productId }}"
```

---

## 5. Error Handling

### 5.1 Error Types

```typescript
try {
  const product = await client.products.get('INVALID-ID');
} catch (error) {
  if (error instanceof WIANotFoundError) {
    console.error('Product not found');
  } else if (error instanceof WIAAuthenticationError) {
    console.error('Invalid API key');
  } else if (error instanceof WIARateLimitError) {
    console.error('Rate limit exceeded, retry after:', error.retryAfter);
  } else if (error instanceof WIAValidationError) {
    console.error('Validation errors:', error.details);
  } else {
    console.error('Unexpected error:', error);
  }
}
```

### 5.2 Retry Logic

```typescript
import pRetry from 'p-retry';

const product = await pRetry(
  async () => {
    return await client.products.get('PRODUCT-001');
  },
  {
    retries: 3,
    onFailedAttempt: (error) => {
      console.log(`Attempt ${error.attemptNumber} failed. ${error.retriesLeft} retries left.`);
    }
  }
);
```

---

## 6. Deployment Guide

### 6.1 Environment Configuration

```bash
# .env file
WIA_API_KEY=wia_live_abc123xyz789
WIA_ENVIRONMENT=production
WIA_WEBHOOK_SECRET=whsec_xyz789abc123
WIA_TIMEOUT=30000
WIA_RETRY_ATTEMPTS=3
```

### 6.2 Docker Deployment

```dockerfile
FROM node:18-alpine

WORKDIR /app

COPY package*.json ./
RUN npm ci --only=production

COPY . .

ENV WIA_API_KEY=${WIA_API_KEY}
ENV WIA_ENVIRONMENT=production

CMD ["node", "dist/index.js"]
```

### 6.3 Kubernetes Deployment

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: insect-protein-service
spec:
  replicas: 3
  selector:
    matchLabels:
      app: insect-protein
  template:
    metadata:
      labels:
        app: insect-protein
    spec:
      containers:
      - name: api
        image: your-registry/insect-protein:latest
        env:
        - name: WIA_API_KEY
          valueFrom:
            secretKeyRef:
              name: wia-credentials
              key: api-key
        - name: WIA_ENVIRONMENT
          value: "production"
```

---

## 7. Best Practices

### 7.1 API Key Management

- Store API keys in environment variables or secret management systems
- Use separate keys for development, staging, and production
- Rotate keys regularly (every 90 days)
- Never commit API keys to version control

### 7.2 Error Handling

- Always handle errors gracefully
- Implement retry logic for transient failures
- Log errors with sufficient context for debugging

### 7.3 Performance Optimization

- Use pagination for large result sets
- Implement caching for frequently accessed data
- Batch requests when possible
- Use WebSocket for real-time updates instead of polling

### 7.4 Security

- Always use HTTPS
- Validate and sanitize all input data
- Implement rate limiting to prevent abuse
- Keep SDK libraries up to date

---

**弘益人間 (Benefit All Humanity)**

© 2025 SmileStory Inc. / WIA
