# WIA-AGRI-031: Urban Farming Standard
## Phase 4: Integration Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines integration patterns for urban farming systems with city platforms, food supply chains, educational institutions, and community services.

### 1.1 Integration Objectives

- **City Services**: Connect with smart city platforms and municipal systems
- **Food Systems**: Integrate with local food supply chains and markets
- **Education**: Link with schools and educational programs
- **Community**: Connect with community apps and social platforms
- **Sustainability**: Integrate with environmental monitoring systems

---

## 2. Smart City Platform Integration

### 2.1 City Data Exchange

**Integration Type:** Bidirectional API

**Data Flows:**
- **Urban farms → City**: Production data, green space metrics, community engagement
- **City → Urban farms**: Weather alerts, water advisories, city events

### 2.2 API Endpoints

**Export to City Platform:**
```http
POST https://api.smartcity.gov/v1/urban-agriculture/data
Authorization: Bearer CITY_API_TOKEN
Content-Type: application/json

{
  "farmId": "UF-NYC-001",
  "period": "2025-06",
  "metrics": {
    "productionVolume": 234.5,
    "greenSpaceArea": 150,
    "waterUsage": 4500,
    "co2Sequestered": 145.5,
    "communityMembers": 24,
    "foodDonated": 78.5
  }
}
```

**Response:**
```json
{
  "status": "accepted",
  "recordId": "CITY-AGRI-12345",
  "acknowledgment": "Data received and processed"
}
```

**Import from City Platform:**
```http
GET https://api.smartcity.gov/v1/services/urban-agriculture/UF-NYC-001
Authorization: Bearer CITY_API_TOKEN
```

**Response:**
```json
{
  "farmId": "UF-NYC-001",
  "cityServices": {
    "waterAdvisory": {
      "status": "NORMAL",
      "restrictions": false
    },
    "weatherAlerts": [
      {
        "type": "HEAT_WAVE",
        "severity": "MODERATE",
        "startDate": "2025-07-15",
        "recommendations": "Increase watering frequency"
      }
    ],
    "events": [
      {
        "eventId": "CITY-EVENT-789",
        "name": "Sustainability Fair",
        "date": "2025-08-10",
        "inviteFarms": true
      }
    ]
  }
}
```

### 2.3 GIS Integration

**Export Farm Locations:**
```json
{
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "geometry": {
        "type": "Point",
        "coordinates": [-73.9442, 40.6782]
      },
      "properties": {
        "farmId": "UF-NYC-001",
        "name": "Brooklyn Community Garden",
        "type": "COMMUNITY",
        "area": 150,
        "status": "ACTIVE",
        "members": 24
      }
    }
  ]
}
```

---

## 3. Food Supply Chain Integration

### 3.1 Local Food Hub Connection

**Integration Type:** REST API + EDI

**Use Cases:**
- Surplus crop distribution
- Farm-to-table tracking
- Demand forecasting
- Price discovery

### 3.2 Harvest Notification

```http
POST https://api.localfoodhub.org/v1/harvests
Authorization: Bearer FOOD_HUB_TOKEN
Content-Type: application/json

{
  "farmId": "UF-NYC-001",
  "harvestId": "HRV-2025-001234",
  "crop": {
    "name": "Tomatoes",
    "variety": "Heirloom Beefsteak",
    "quantity": 12.5,
    "unit": "kg",
    "quality": "GRADE_A",
    "organic": true
  },
  "availability": {
    "date": "2025-07-20",
    "location": "Brooklyn Community Garden",
    "coordinates": {
      "lat": 40.6782,
      "lng": -73.9442
    }
  },
  "pricing": {
    "askingPrice": 6.00,
    "unit": "per_kg",
    "currency": "USD",
    "negotiable": true
  }
}
```

**Response:**
```json
{
  "status": "listed",
  "listingId": "FH-LISTING-456",
  "matches": [
    {
      "buyerId": "BUYER-789",
      "buyerName": "Brooklyn Food Co-op",
      "interest": "HIGH",
      "requestedQuantity": 10.0
    }
  ]
}
```

### 3.3 Food Safety Compliance

**Traceability Data:**
```json
{
  "traceabilityId": "TRACE-001",
  "farmId": "UF-NYC-001",
  "harvestId": "HRV-2025-001234",
  "crop": "Tomatoes",
  "harvestDate": "2025-07-20",
  "certifications": ["ORGANIC", "PESTICIDE_FREE"],
  "soilTests": {
    "lastTestDate": "2025-04-01",
    "results": "PASSED",
    "heavyMetals": "SAFE",
    "contaminants": "NONE"
  },
  "growingHistory": {
    "plantedDate": "2025-05-01",
    "waterSource": "MUNICIPAL",
    "fertilizers": ["COMPOST"],
    "pesticides": []
  }
}
```

---

## 4. Educational Platform Integration

### 4.1 School Program Connection

**Integration Type:** REST API + LMS Integration

**Data Flows:**
- Field trip scheduling
- Student participation tracking
- Educational content sharing
- Learning outcomes

### 4.2 Register Educational Visit

```http
POST https://api.schoolprogram.edu/v1/field-trips
Authorization: Bearer EDU_TOKEN
Content-Type: application/json

{
  "farmId": "UF-NYC-001",
  "schoolId": "PS-123",
  "visitDate": "2025-06-25",
  "students": 25,
  "gradeLevel": "4-5",
  "learningObjectives": [
    "Plant life cycles",
    "Sustainable agriculture",
    "Food systems"
  ],
  "activities": [
    "Garden tour",
    "Planting workshop",
    "Composting demo"
  ],
  "contactPerson": {
    "name": "Sarah Johnson",
    "email": "sjohnson@ps123.edu",
    "phone": "+1-555-0123"
  }
}
```

**Response:**
```json
{
  "status": "confirmed",
  "visitId": "VISIT-456",
  "assignedGuide": "Jane Smith",
  "materials": [
    "Planting guide for teachers",
    "Student activity worksheets",
    "Garden safety rules"
  ]
}
```

### 4.3 Learning Management System (LMS)

**Export Learning Data:**
```json
{
  "farmId": "UF-NYC-001",
  "program": "Junior Gardener",
  "students": [
    {
      "studentId": "STU-789",
      "name": "Alex Chen",
      "schoolId": "PS-123",
      "participationHours": 12,
      "skillsLearned": [
        "Seed planting",
        "Composting",
        "Pest identification"
      ],
      "assessments": {
        "preTest": 65,
        "postTest": 92,
        "improvement": 27
      },
      "badges": [
        "First Harvest",
        "Compost Master",
        "Plant Identifier"
      ]
    }
  ]
}
```

---

## 5. Community Mobile App Integration

### 5.1 Mobile App Features

- Plot status and crop tracking
- Community event calendar
- Volunteer hour logging
- Harvest sharing
- Real-time notifications

### 5.2 Mobile API Endpoints

**Get User's Farm Info:**
```http
GET https://api.urbanfarming.example.com/v1/mobile/user/{userId}
Authorization: Bearer USER_TOKEN
```

**Response:**
```json
{
  "userId": "U-12345",
  "name": "Jane Smith",
  "farms": [
    {
      "farmId": "UF-NYC-001",
      "role": "MEMBER",
      "plots": [
        {
          "plotId": "UF-NYC-001-P01",
          "crops": ["Tomatoes", "Basil"],
          "status": "ACTIVE",
          "nextTask": {
            "task": "Watering",
            "dueDate": "2025-06-16",
            "priority": "MEDIUM"
          }
        }
      ]
    }
  ],
  "notifications": [
    {
      "type": "WATERING_REMINDER",
      "message": "Your plot needs watering",
      "timestamp": "2025-06-15T08:00:00Z"
    }
  ]
}
```

**Log Volunteer Hours:**
```http
POST https://api.urbanfarming.example.com/v1/mobile/volunteer-hours
Authorization: Bearer USER_TOKEN
Content-Type: application/json

{
  "userId": "U-12345",
  "farmId": "UF-NYC-001",
  "date": "2025-06-15",
  "hours": 3.5,
  "activity": "Weeding and maintenance",
  "notes": "Helped prepare Plot B for new planting"
}
```

### 5.3 Push Notifications

**Register Device:**
```http
POST https://api.urbanfarming.example.com/v1/mobile/devices
Content-Type: application/json

{
  "userId": "U-12345",
  "deviceToken": "fcm_token_abc123",
  "platform": "IOS",
  "notificationPreferences": {
    "wateringReminders": true,
    "harvestAlerts": true,
    "communityEvents": true,
    "weatherAlerts": true
  }
}
```

---

## 6. Weather Service Integration

### 6.1 Weather API Connection

**Provider Examples:**
- OpenWeatherMap
- Weather Underground
- NOAA Weather API

**Fetch Local Forecast:**
```http
GET https://api.weather.example.com/v1/forecast?lat=40.6782&lon=-73.9442
Authorization: Bearer WEATHER_API_KEY
```

**Response:**
```json
{
  "location": {
    "lat": 40.6782,
    "lon": -73.9442
  },
  "forecast": [
    {
      "date": "2025-06-16",
      "temperature": {
        "high": 28,
        "low": 18,
        "unit": "celsius"
      },
      "precipitation": {
        "probability": 30,
        "amount": 2.5,
        "unit": "mm"
      },
      "conditions": "Partly cloudy",
      "uvIndex": 7,
      "windSpeed": 15
    }
  ],
  "alerts": [
    {
      "type": "HEAT_ADVISORY",
      "severity": "MODERATE",
      "message": "High temperatures expected",
      "recommendations": "Increase watering frequency"
    }
  ]
}
```

### 6.2 Automated Irrigation Adjustment

**Weather-Based Logic:**
```javascript
const adjustIrrigation = (forecast, currentSettings) => {
  if (forecast.precipitation.probability > 70) {
    return {
      action: "SKIP",
      reason: "Rain expected",
      nextScheduled: calculateNextDay(forecast)
    };
  }

  if (forecast.temperature.high > 30) {
    return {
      action: "INCREASE",
      multiplier: 1.5,
      reason: "High temperature"
    };
  }

  return {
    action: "NORMAL",
    settings: currentSettings
  };
};
```

---

## 7. Sustainability Dashboard Integration

### 7.1 Carbon Tracking

**Export to Carbon Dashboard:**
```http
POST https://api.sustainability.example.com/v1/carbon
Authorization: Bearer SUSTAIN_TOKEN
Content-Type: application/json

{
  "organizationId": "UF-NYC-001",
  "period": "2025-Q2",
  "carbonSequestered": 145.5,
  "carbonAvoided": 89.2,
  "methodology": "WIA-AGRI-031",
  "verification": {
    "verified": true,
    "verifier": "Third-party auditor",
    "date": "2025-07-01"
  },
  "details": {
    "greenSpaceArea": 150,
    "biomassGrown": 234.5,
    "composting": 185,
    "transportAvoided": 2400
  }
}
```

### 7.2 Resource Efficiency Metrics

```json
{
  "farmId": "UF-NYC-001",
  "period": "2025-Q2",
  "waterEfficiency": {
    "totalUsed": 4500,
    "savedVsConventional": 2100,
    "efficiency": 46.7,
    "unit": "liters"
  },
  "energyEfficiency": {
    "solarGenerated": 0,
    "gridUsed": 150,
    "efficiency": "MODERATE"
  },
  "wasteReduction": {
    "composted": 185,
    "diversionRate": 95,
    "unit": "kg"
  },
  "biodiversity": {
    "plantSpecies": 24,
    "pollinatorSpecies": 8,
    "biodiversityIndex": 82
  }
}
```

---

## 8. Payment and Donation Integration

### 8.1 Payment Gateway

**Process Plot Fee:**
```http
POST https://api.payment.example.com/v1/charges
Authorization: Bearer PAYMENT_TOKEN
Content-Type: application/json

{
  "amount": 50.00,
  "currency": "USD",
  "description": "Annual plot fee - UF-NYC-001-P01",
  "customer": "cus_12345",
  "metadata": {
    "farmId": "UF-NYC-001",
    "plotId": "UF-NYC-001-P01",
    "gardenerId": "G-2345",
    "year": "2025"
  }
}
```

### 8.2 Donation Platform

**Record Donation:**
```http
POST https://api.donations.example.com/v1/donations
Content-Type: application/json

{
  "farmId": "UF-NYC-001",
  "donor": {
    "name": "Community Supporter",
    "email": "donor@example.com",
    "anonymous": false
  },
  "amount": 100.00,
  "currency": "USD",
  "purpose": "General support",
  "taxDeductible": true,
  "receiptRequested": true
}
```

---

## 9. Social Media Integration

### 9.1 Auto-Post Harvests

**Instagram/Facebook:**
```javascript
const postHarvest = async (harvestData) => {
  const post = {
    imageUrl: harvestData.photo,
    caption: `Fresh ${harvestData.crop} harvested today! 🍅\n` +
             `${harvestData.quantity}kg from ${harvestData.farmName}\n` +
             `#UrbanFarming #CommunityGarden #LocalFood`,
    location: {
      name: harvestData.farmName,
      coordinates: harvestData.coordinates
    }
  };

  await socialMedia.post(post);
};
```

### 9.2 Community Updates

**Twitter/X API:**
```http
POST https://api.twitter.com/2/tweets
Authorization: Bearer TWITTER_TOKEN
Content-Type: application/json

{
  "text": "🌱 Brooklyn Community Garden harvest update: 12.5kg of heirloom tomatoes! Thanks to all our volunteers. #UrbanFarming #Brooklyn"
}
```

---

## 10. Data Synchronization

### 10.1 Sync Strategy

**Incremental Sync:**
```http
GET https://api.urbanfarming.example.com/v1/sync?since=2025-06-15T00:00:00Z
Authorization: Bearer API_TOKEN
```

**Response:**
```json
{
  "lastSyncTime": "2025-06-15T12:00:00Z",
  "changes": {
    "harvests": {
      "created": 5,
      "updated": 2,
      "deleted": 0
    },
    "plots": {
      "created": 0,
      "updated": 3,
      "deleted": 0
    },
    "members": {
      "created": 2,
      "updated": 1,
      "deleted": 0
    }
  },
  "data": {
    "harvests": [...],
    "plots": [...],
    "members": [...]
  }
}
```

### 10.2 Conflict Resolution

**Last-Write-Wins:**
```json
{
  "conflictId": "CONFLICT-123",
  "resource": "plot",
  "plotId": "UF-NYC-001-P01",
  "conflicts": [
    {
      "source": "mobile_app",
      "timestamp": "2025-06-15T12:00:00Z",
      "data": {"status": "ACTIVE"}
    },
    {
      "source": "web_admin",
      "timestamp": "2025-06-15T12:01:00Z",
      "data": {"status": "MAINTENANCE"}
    }
  ],
  "resolution": {
    "strategy": "LAST_WRITE_WINS",
    "winner": "web_admin",
    "finalState": {"status": "MAINTENANCE"}
  }
}
```

---

## 11. Integration Testing

### 11.1 Test Environment

```
Test Base URLs:
- API: https://test-api.urbanfarming.example.com/v1
- MQTT: mqtt://test-mqtt.urbanfarming.example.com:1883
- WebSocket: wss://test-ws.urbanfarming.example.com/v1
```

### 11.2 Sample Test Cases

```javascript
describe('City Platform Integration', () => {
  it('should export farm data to city platform', async () => {
    const data = {
      farmId: 'TEST-FARM-001',
      metrics: { productionVolume: 100 }
    };

    const response = await cityAPI.exportData(data);
    expect(response.status).toBe('accepted');
  });

  it('should import city alerts', async () => {
    const alerts = await cityAPI.getAlerts('TEST-FARM-001');
    expect(alerts).toBeInstanceOf(Array);
  });
});
```

---

## 12. Integration Checklist

- [ ] API keys and credentials configured
- [ ] Webhooks registered and verified
- [ ] Data mapping completed
- [ ] Error handling implemented
- [ ] Rate limiting configured
- [ ] Security review completed
- [ ] Integration tests passing
- [ ] Monitoring and logging enabled
- [ ] Documentation updated
- [ ] Stakeholders notified

---

**Previous Phase:** [PHASE-3-PROTOCOL.md](PHASE-3-PROTOCOL.md)

© 2025 WIA - World Certification Industry Association
弘益人間 · Benefit All Humanity
