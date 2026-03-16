# WIA-AGRI-032: Food Waste Reduction Standard
## Phase 4: Integration Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-12-26

---

## 1. Overview

This specification defines integration patterns, connector implementations, and interoperability requirements for food waste reduction systems across retail, inventory management, donation networks, and analytics platforms.

### 1.1 Integration Architecture

```
┌─────────────────────────────────────────────────────────┐
│           WIA-AGRI-032 Integration Layer                │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌────────┐ │
│  │ Retail   │  │Inventory │  │Donation  │  │Analytics│ │
│  │ Systems  │  │Management│  │ Networks │  │Platforms│ │
│  └──────────┘  └──────────┘  └──────────┘  └────────┘ │
│       │             │             │             │       │
│  ┌────┴─────────────┴─────────────┴─────────────┴────┐ │
│  │         WIA-AGRI-032 Core API Layer                │ │
│  └──────────────────────────────────────────────────┬─┘ │
│                                                      │   │
│  ┌───────────────────────────────────────────────────┘   │
│  │   Data Format Converters & Protocol Adapters        │
│  └──────────────────────────────────────────────────┐   │
│                                                      │   │
└──────────────────────────────────────────────────────┴───┘
```

---

## 2. Retail System Integration

### 2.1 Point of Sale (POS) Systems

#### Square POS Integration

**Authentication:**
```javascript
const squareClient = new Client({
  accessToken: process.env.SQUARE_ACCESS_TOKEN,
  environment: Environment.Production
});
```

**Sync Inventory to WIA:**
```javascript
async function syncSquareInventory() {
  // 1. Fetch inventory from Square
  const { result } = await squareClient.catalogApi.listCatalog(
    undefined,
    'ITEM'
  );

  // 2. Transform to WIA format
  const wiaItems = result.objects.map(item => ({
    standardVersion: 'WIA-AGRI-032-v1.0',
    facilityId: 'FACILITY-SQUARE-001',
    product: {
      name: item.itemData.name,
      sku: item.id,
      category: mapSquareCategoryToWIA(item.itemData.categoryId),
      quantity: getSquareInventoryCount(item.id)
    },
    dates: {
      received: new Date().toISOString(),
      expiration: calculateExpirationDate(item)
    }
  }));

  // 3. Send to WIA API
  await wiaClient.inventory.batchCreate(wiaItems);
}
```

**Receive Expiration Alerts:**
```javascript
wiaClient.webhooks.on('expiration_alert', async (event) => {
  // Update Square inventory with markdown pricing
  await squareClient.catalogApi.upsertCatalogObject({
    object: {
      type: 'ITEM',
      id: event.data.sku,
      itemData: {
        variations: [{
          priceMoney: {
            amount: calculateMarkdownPrice(event.data),
            currency: 'USD'
          }
        }]
      }
    }
  });
});
```

#### Toast POS (Restaurant Systems)

**Menu Item Tracking:**
```javascript
const toastAPI = new ToastAPI(apiKey);

// Track prepared food with short shelf life
async function trackPreparedFood(order) {
  const preparedItems = order.items.filter(i => i.preparedOnSite);

  for (const item of preparedItems) {
    await wiaClient.inventory.create({
      facilityId: 'RESTAURANT-001',
      product: {
        name: item.name,
        category: 'prepared',
        quantity: 1,
        unit: 'portions'
      },
      dates: {
        manufactured: new Date(),
        expiration: new Date(Date.now() + item.shelfLifeHours * 3600000)
      },
      customData: {
        prepTime: item.prepTime,
        station: item.kitchenStation
      }
    });
  }
}
```

### 2.2 E-commerce Platforms

#### Shopify Integration

**Install Webhook:**
```javascript
// Shopify app installation
POST https://your-store.myshopify.com/admin/api/2024-01/webhooks.json
{
  "webhook": {
    "topic": "inventory_levels/update",
    "address": "https://wia-connector.yourapp.com/webhooks/shopify",
    "format": "json"
  }
}
```

**Sync Products:**
```javascript
const Shopify = require('shopify-api-node');
const shopify = new Shopify({
  shopName: 'your-store',
  apiKey: process.env.SHOPIFY_API_KEY,
  password: process.env.SHOPIFY_PASSWORD
});

async function syncShopifyProducts() {
  const products = await shopify.product.list({ limit: 250 });

  for (const product of products) {
    for (const variant of product.variants) {
      // Check if perishable
      if (product.tags.includes('perishable')) {
        await wiaClient.inventory.create({
          product: {
            name: `${product.title} - ${variant.title}`,
            sku: variant.sku,
            barcode: variant.barcode,
            quantity: variant.inventory_quantity
          },
          dates: {
            expiration: getExpirationFromMetafield(product)
          }
        });
      }
    }
  }
}
```

---

## 3. Inventory Management Systems

### 3.1 NetSuite ERP Integration

**SuiteTalk RESTlet:**
```javascript
// NetSuite RESTlet endpoint
function handleWIASync(request) {
  var items = JSON.parse(request.body).items;

  items.forEach(function(item) {
    // Create or update inventory item in NetSuite
    var record = nlapiCreateRecord('inventoryitem');
    record.setFieldValue('itemid', item.product.sku);
    record.setFieldValue('displayname', item.product.name);
    record.setFieldValue('custitem_expiration_date', item.dates.expiration);
    record.setFieldValue('custitem_wia_tracking_id', item.trackingId);

    var recordId = nlapiSubmitRecord(record);

    // Create custom record for waste tracking
    if (item.status.current === 'expired') {
      var wasteRecord = nlapiCreateRecord('customrecord_waste_tracking');
      wasteRecord.setFieldValue('custrecord_item', recordId);
      wasteRecord.setFieldValue('custrecord_waste_date', new Date());
      wasteRecord.setFieldValue('custrecord_waste_reason', item.wasteReason);
      nlapiSubmitRecord(wasteRecord);
    }
  });

  return { success: true, processed: items.length };
}
```

**Scheduled Script for Daily Sync:**
```javascript
function scheduledSync() {
  // Search for items expiring soon
  var search = nlapiCreateSearch('inventoryitem', [
    ['custitem_expiration_date', 'within', 'today', 'next7days']
  ]);

  var results = search.runSearch();
  var items = [];

  results.forEachResult(function(result) {
    items.push({
      trackingId: result.getValue('custitem_wia_tracking_id'),
      status: 'near_expiry',
      daysUntilExpiry: calculateDays(result.getValue('custitem_expiration_date'))
    });
    return true;
  });

  // Send to WIA API
  sendToWIA(items);
}
```

### 3.2 Fishbowl Inventory

**Database Integration (MySQL):**
```sql
-- Create view for WIA sync
CREATE VIEW wia_inventory_sync AS
SELECT
  p.sku,
  p.description AS name,
  pc.name AS category,
  i.quantity,
  i.location_id,
  p.uom_id,
  DATE_ADD(i.date_received, INTERVAL p.shelf_life_days DAY) AS expiration_date,
  i.cost_per_unit
FROM part p
JOIN part_category pc ON p.category_id = pc.id
JOIN inventory i ON p.id = i.part_id
WHERE p.is_perishable = 1;
```

**Python Connector:**
```python
import mysql.connector
import requests
from datetime import datetime, timedelta

class FishbowlWIAConnector:
    def __init__(self, db_config, wia_api_key):
        self.db = mysql.connector.connect(**db_config)
        self.wia_api = f"https://api.wia-foodwaste.org/v1"
        self.headers = {"Authorization": f"Bearer {wia_api_key}"}

    def sync_inventory(self):
        cursor = self.db.cursor(dictionary=True)
        cursor.execute("SELECT * FROM wia_inventory_sync")

        items = []
        for row in cursor:
            items.append({
                "facilityId": "FACILITY-FISHBOWL-001",
                "product": {
                    "name": row['name'],
                    "sku": row['sku'],
                    "category": self.map_category(row['category']),
                    "quantity": row['quantity'],
                    "unit": self.map_unit(row['uom_id'])
                },
                "dates": {
                    "expiration": row['expiration_date'].isoformat()
                },
                "economics": {
                    "costPerUnit": float(row['cost_per_unit'])
                }
            })

        # Batch create in WIA
        response = requests.post(
            f"{self.wia_api}/inventory/batch",
            json={"items": items},
            headers=self.headers
        )
        return response.json()
```

---

## 4. Donation Network Integration

### 4.1 Feeding America MealConnect

**API Integration:**
```javascript
const MealConnect = require('meal-connect-api');

class MealConnectWIABridge {
  constructor(apiKey) {
    this.mcApi = new MealConnect(apiKey);
  }

  async createDonationFromWIA(wiaData) {
    // 1. Find nearby food banks
    const foodBanks = await this.mcApi.findFoodBanks({
      latitude: wiaData.donor.location.latitude,
      longitude: wiaData.donor.location.longitude,
      radius: 10, // miles
      acceptsCategories: wiaData.products.map(p => p.category)
    });

    // 2. Create donation listing
    const donation = await this.mcApi.createDonation({
      donorId: wiaData.donor.facilityId,
      availableDate: new Date(),
      expirationDate: wiaData.products[0].expirationDate,
      items: wiaData.products.map(p => ({
        description: p.name,
        quantity: p.quantity,
        unit: p.unit,
        category: this.mapToMCCategory(p.category)
      })),
      pickupInfo: {
        address: wiaData.donor.address,
        instructions: "Use loading dock entrance",
        availableHours: "9AM-5PM Mon-Fri"
      }
    });

    // 3. Update WIA with donation ID
    await wiaClient.donations.update(wiaData.donationId, {
      externalIds: {
        mealConnect: donation.id
      },
      status: 'listed_for_pickup'
    });

    return donation;
  }

  async handlePickupConfirmation(mcDonation) {
    // When food bank confirms pickup in MealConnect
    await wiaClient.donations.update(mcDonation.wiaId, {
      status: 'pickup_confirmed',
      logistics: {
        pickupScheduled: mcDonation.pickupTime,
        recipient: mcDonation.foodBank.name
      }
    });

    // Send notification
    await wiaClient.notifications.send({
      type: 'donation_confirmed',
      recipient: mcDonation.donor.email,
      data: mcDonation
    });
  }
}
```

### 4.2 Too Good To Go Integration

**Marketplace Listing:**
```javascript
const TGTG = require('too-good-to-go-api');

async function listSurplusOnTGTG(wiaInventory) {
  const tgtg = new TGTG(process.env.TGTG_API_KEY);

  // Filter items suitable for TGTG
  const suitableItems = wiaInventory.filter(item =>
    item.status.current === 'near_expiry' &&
    item.daysUntilExpiry >= 1 &&
    item.daysUntilExpiry <= 3
  );

  // Create surprise bags
  const bags = groupIntoBags(suitableItems);

  for (const bag of bags) {
    await tgtg.createListing({
      store_id: 'STORE-001',
      item: {
        name: 'Surprise Bag - Fresh Groceries',
        description: 'Assorted fresh items near expiration',
        item_category: 'GROCERIES',
        item_value: calculateRetailValue(bag.items),
        item_price: calculateDiscountPrice(bag.items, 0.7), // 70% off
        items_available: 1
      },
      pickup_interval: {
        start: new Date(Date.now() + 3600000).toISOString(), // 1 hour from now
        end: new Date(Date.now() + 7200000).toISOString() // 2 hours from now
      }
    });

    // Mark items as listed
    await wiaClient.inventory.updateMany(
      bag.items.map(i => i.trackingId),
      { status: 'listed_for_sale', customData: { tgtg: true } }
    );
  }
}
```

### 4.3 Olio Food Sharing

**Community Distribution:**
```javascript
const Olio = require('olio-api');

async function shareOnOlio(wiaItems) {
  const olio = new Olio(process.env.OLIO_API_KEY);

  for (const item of wiaItems) {
    // Create listing on Olio
    const listing = await olio.listings.create({
      title: item.product.name,
      description: `${item.product.quantity} ${item.product.unit}`,
      category: mapToOlioCategory(item.product.category),
      collection_info: {
        type: 'specific_location',
        address: item.donor.address,
        dates: [
          {
            start: new Date(),
            end: new Date(item.dates.expiration)
          }
        ]
      },
      images: item.metadata.photos || []
    });

    // Update WIA
    await wiaClient.inventory.update(item.trackingId, {
      status: 'shared_community',
      externalIds: { olio: listing.id }
    });
  }

  // Listen for collection confirmations
  olio.on('listing_collected', async (event) => {
    await wiaClient.donations.create({
      donor: { facilityId: event.listing.facilityId },
      recipient: { type: 'community_member', olioUserId: event.collector.id },
      products: [{ trackingId: event.listing.wiaTrackingId }],
      type: 'community_sharing'
    });
  });
}
```

---

## 5. Analytics Platform Integration

### 5.1 LeanPath Waste Tracking

**Data Export to LeanPath:**
```javascript
const LeanPath = require('leanpath-api');

async function exportToLeanPath(wasteEvents) {
  const leanpath = new LeanPath(process.env.LEANPATH_API_KEY);

  for (const event of wasteEvents) {
    await leanpath.wasteEvents.create({
      site_id: event.facilityId,
      recorded_at: event.timestamp,
      waste_type: mapWasteType(event.wasteType),
      items: event.products.map(p => ({
        food_category: mapToLeanPathCategory(p.category),
        weight: p.quantity,
        weight_unit: p.unit,
        value: p.economicValue,
        reason: p.reason
      })),
      disposal_method: event.disposal.method,
      prevented: event.preventionAnalysis.preventable
    });
  }
}
```

**Import LeanPath Insights:**
```javascript
async function importLeanPathInsights() {
  const insights = await leanpath.analytics.getInsights({
    site_id: 'SITE-001',
    period: 'last_30_days'
  });

  // Store in WIA for unified reporting
  await wiaClient.analytics.storeExternal({
    source: 'leanpath',
    facilityId: 'FACILITY-001',
    period: insights.period,
    metrics: {
      totalWaste: insights.total_waste_kg,
      costImpact: insights.cost_impact,
      topWasteCategories: insights.top_categories,
      trends: insights.trends,
      recommendations: insights.ai_recommendations
    }
  });
}
```

### 5.2 Winnow Solutions

**Computer Vision Integration:**
```javascript
const Winnow = require('winnow-api');

// Winnow's AI camera captures waste images
winnow.on('waste_captured', async (event) => {
  // AI identifies food items from image
  const identified = event.aiAnalysis;

  // Create WIA waste event
  await wiaClient.waste.track({
    facilityId: event.kitchenId,
    wasteType: 'preparation_waste',
    products: identified.items.map(item => ({
      name: item.foodName,
      category: item.category,
      quantity: item.estimatedWeight,
      unit: 'kg',
      reason: item.wasteReason
    })),
    metadata: {
      capturedBy: 'winnow_ai',
      imageUrl: event.imageUrl,
      confidence: identified.confidence
    }
  });
});
```

---

## 6. IoT Device Integration

### 6.1 Smart Refrigerators

**Samsung SmartThings:**
```javascript
const SmartThings = require('smartthings-api');

async function monitorSmartFridge() {
  const st = new SmartThings(process.env.SMARTTHINGS_TOKEN);

  // Subscribe to temperature alerts
  st.subscribeToCapability('temperatureMeasurement', async (event) => {
    const temp = event.value;
    const deviceId = event.deviceId;

    if (temp > 7 || temp < 2) { // Outside safe range for refrigeration
      // Find products in this fridge
      const products = await wiaClient.inventory.query({
        storageLocation: deviceId,
        temperatureZone: 'refrigerated'
      });

      // Create temperature alert
      await wiaClient.alerts.create({
        type: 'temperature_violation',
        severity: 'high',
        affectedProducts: products.map(p => p.trackingId),
        details: {
          currentTemp: temp,
          safeRange: [2, 7],
          duration: event.duration
        },
        recommendations: [
          'Check refrigerator door seal',
          'Verify products are still safe',
          'Consider moving products to backup storage'
        ]
      });
    }
  });
}
```

### 6.2 Automated Waste Scales

**METTLER TOLEDO Integration:**
```javascript
const MettlerToledo = require('mettler-toledo-scale');

const scale = new MettlerToledo({
  port: '/dev/ttyUSB0',
  baudRate: 9600
});

scale.on('stable_weight', async (weight) => {
  // Barcode scanner identifies product
  const barcode = await scanner.scan();

  // Look up product in WIA
  const product = await wiaClient.inventory.findByBarcode(barcode);

  if (product && weight.value > 0.1) {
    // Record waste event
    await wiaClient.waste.track({
      facilityId: 'FACILITY-001',
      products: [{
        trackingId: product.trackingId,
        name: product.product.name,
        quantity: weight.value,
        unit: weight.unit
      }],
      metadata: {
        measuredBy: 'automated_scale',
        scaleId: scale.serialNumber,
        accuracy: 'high'
      }
    });

    // Update inventory
    await wiaClient.inventory.update(product.trackingId, {
      status: 'wasted',
      quantity: product.quantity - weight.value
    });
  }
});
```

---

## 7. Cross-Platform Data Flow

### 7.1 End-to-End Example: Restaurant to Food Bank

```javascript
// Day 1: Restaurant receives delivery
async function receiveDelivery(deliveryData) {
  // 1. Create in POS (Toast)
  const toastItem = await toast.inventory.create(deliveryData);

  // 2. Sync to WIA
  const wiaItem = await wiaClient.inventory.create({
    facilityId: 'RESTAURANT-001',
    product: {
      name: deliveryData.productName,
      quantity: deliveryData.quantity,
      sku: toastItem.sku
    },
    dates: {
      received: new Date(),
      expiration: deliveryData.expirationDate
    }
  });

  // 3. Store in ERP (NetSuite)
  await netsuite.inventory.sync(wiaItem);
}

// Day 5: Item approaching expiration
wiaClient.on('expiration_alert', async (alert) => {
  if (alert.daysUntilExpiry === 2) {
    // 1. Create donation
    const donation = await wiaClient.donations.create({
      donor: { facilityId: 'RESTAURANT-001' },
      products: [{ trackingId: alert.trackingId }]
    });

    // 2. List on MealConnect
    const mcDonation = await mealConnect.createListing(donation);

    // 3. Send to analytics
    await leanpath.recordPreventedWaste(donation);
  }
});

// Day 6: Food bank confirms pickup
mealConnect.on('pickup_confirmed', async (event) => {
  // 1. Update WIA
  await wiaClient.donations.update(event.donationId, {
    status: 'completed',
    impact: {
      mealsProvided: event.estimatedMeals,
      co2Prevented: calculateCO2Savings(event.weight)
    }
  });

  // 2. Update inventory in POS
  await toast.inventory.adjustQuantity(event.toastSku, -event.quantity);

  // 3. Create accounting entry in NetSuite
  await netsuite.createDonationJournalEntry(event);

  // 4. Send thank you email
  await sendDonationReceipt(event);
});
```

---

## 8. Integration Testing

### 8.1 Test Scenarios

**Scenario 1: Full Lifecycle Test**
```javascript
describe('Food Item Lifecycle', () => {
  it('should track from receipt to donation', async () => {
    // 1. Receive item
    const item = await wiaClient.inventory.create(testData.item);
    expect(item.status).toBe('fresh');

    // 2. Simulate time passing
    await simulateDays(5);

    // 3. Check for expiration alert
    const alerts = await wiaClient.alerts.query({ trackingId: item.trackingId });
    expect(alerts).toHaveLength(1);
    expect(alerts[0].type).toBe('expiration_alert');

    // 4. Create donation
    const donation = await wiaClient.donations.create({
      products: [{ trackingId: item.trackingId }]
    });
    expect(donation.status).toBe('pending');

    // 5. Complete donation
    await wiaClient.donations.update(donation.id, { status: 'completed' });

    // 6. Verify item updated
    const updatedItem = await wiaClient.inventory.get(item.trackingId);
    expect(updatedItem.status).toBe('donated');
  });
});
```

### 8.2 Integration Health Monitoring

```javascript
class IntegrationMonitor {
  async checkHealth() {
    const results = {
      timestamp: new Date().toISOString(),
      integrations: {}
    };

    // Test each integration
    results.integrations.square = await this.testSquare();
    results.integrations.netsuite = await this.testNetSuite();
    results.integrations.mealConnect = await this.testMealConnect();
    results.integrations.leanpath = await this.testLeanPath();

    // Overall health
    const allHealthy = Object.values(results.integrations)
      .every(i => i.status === 'healthy');
    results.overallStatus = allHealthy ? 'healthy' : 'degraded';

    return results;
  }

  async testSquare() {
    try {
      await squareClient.catalogApi.listCatalog(undefined, 'ITEM', 1);
      return { status: 'healthy', latency: Date.now() - start };
    } catch (error) {
      return { status: 'unhealthy', error: error.message };
    }
  }
}
```

---

## 9. Migration Guide

### 9.1 Migrating from Legacy System

**Step 1: Data Export**
```sql
-- Export from legacy database
SELECT
  product_id,
  product_name,
  category,
  quantity,
  unit,
  expiration_date,
  cost
FROM legacy_inventory
WHERE is_perishable = 1
INTO OUTFILE '/tmp/inventory_export.csv'
FIELDS TERMINATED BY ','
ENCLOSED BY '"'
LINES TERMINATED BY '\n';
```

**Step 2: Transform and Import**
```python
import csv
import requests

def migrate_inventory(csv_file, wia_api_key):
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        batch = []

        for row in reader:
            batch.append({
                "facilityId": "FACILITY-MIGRATED-001",
                "product": {
                    "name": row['product_name'],
                    "sku": row['product_id'],
                    "category": map_category(row['category']),
                    "quantity": float(row['quantity']),
                    "unit": row['unit']
                },
                "dates": {
                    "expiration": row['expiration_date']
                },
                "economics": {
                    "costPerUnit": float(row['cost'])
                }
            })

            # Batch in groups of 100
            if len(batch) >= 100:
                send_batch(batch, wia_api_key)
                batch = []

        # Send remaining
        if batch:
            send_batch(batch, wia_api_key)
```

---

## 10. Best Practices

### 10.1 Error Handling

```javascript
async function robustIntegration() {
  try {
    await syncInventory();
  } catch (error) {
    // Log error
    logger.error('Inventory sync failed', { error, timestamp: new Date() });

    // Retry with exponential backoff
    await retryWithBackoff(syncInventory, {
      maxAttempts: 3,
      backoffMs: 1000
    });

    // If still failing, queue for later
    await queue.add('inventory-sync', {}, {
      delay: 300000, // 5 minutes
      attempts: 5
    });

    // Alert operations team
    await alerting.notify({
      severity: 'high',
      message: 'Integration sync failing',
      integration: 'inventory'
    });
  }
}
```

### 10.2 Rate Limiting

```javascript
const Bottleneck = require('bottleneck');

// Respect partner API rate limits
const squareLimiter = new Bottleneck({
  minTime: 100, // 10 req/sec max
  maxConcurrent: 5
});

const syncItem = squareLimiter.wrap(async (item) => {
  return await squareClient.catalogApi.upsertCatalogObject(item);
});
```

---

**Related Documentation:**
- [Phase 1: Data Format](./PHASE-1-DATA-FORMAT.md)
- [Phase 2: API Interface](./PHASE-2-API-INTERFACE.md)
- [Phase 3: Protocol](./PHASE-3-PROTOCOL.md)

---

© 2025 WIA Standards · MIT License
弘益人間 · Benefit All Humanity
