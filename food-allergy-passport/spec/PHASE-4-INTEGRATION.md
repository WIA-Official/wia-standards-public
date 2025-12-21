# WIA Food Allergy Passport Integration Standard
## Phase 4 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #84CC16 (Lime)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [System Architecture](#system-architecture)
4. [Restaurant Integration](#restaurant-integration)
5. [Airline Integration](#airline-integration)
6. [Healthcare Integration](#healthcare-integration)
7. [Mobile Applications](#mobile-applications)
8. [POS System Integration](#pos-system-integration)
9. [Deployment](#deployment)
10. [Certification & Compliance](#certification--compliance)
11. [Integration Examples](#integration-examples)
12. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Food Allergy Passport Integration Standard defines comprehensive integration guidelines for embedding allergy passport capabilities into food service ecosystems, including restaurants, airlines, hospitals, and consumer applications. This Phase 4 specification completes the WIA Food Allergy Passport standard suite, providing deployment and certification requirements.

**Core Objectives**:
- Enable seamless integration with existing restaurant POS systems
- Support airline catering and meal selection systems
- Integrate with healthcare/hospital food service systems
- Provide mobile app SDKs for iOS, Android, and Web
- Define certification requirements for compliant implementations
- Ensure global interoperability across food service providers

### 1.2 Scope

This standard defines:

| Component | Description |
|-----------|-------------|
| **System Architecture** | End-to-end ecosystem design |
| **Integration Patterns** | Common integration scenarios |
| **SDK Libraries** | Client libraries for major platforms |
| **Deployment Models** | Cloud, on-premise, hybrid options |
| **Certification Process** | Requirements for WIA certification |

### 1.3 Integration Levels

| Level | Description | Requirements |
|-------|-------------|--------------|
| **Level 1: Basic** | QR code scanning only | QR scanner, basic display |
| **Level 2: Standard** | API integration, real-time alerts | API client, WebSocket support |
| **Level 3: Advanced** | Full ecosystem integration | All APIs, translations, analytics |
| **Level 4: Certified** | WIA certified implementation | Passes all certification tests |

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Integration Partner** | Organization implementing WIA standard |
| **POS System** | Point-of-Sale system in restaurants |
| **KDS** | Kitchen Display System |
| **Catering System** | Airline meal planning and preparation system |
| **EHR** | Electronic Health Record system |
| **SDK** | Software Development Kit |
| **Webhook** | HTTP callback for event notifications |

### 2.2 Integration Types

| Type | Description | Use Case |
|------|-------------|----------|
| **Direct API** | RESTful API integration | Modern cloud-based systems |
| **SDK Integration** | Pre-built library integration | Mobile apps, web apps |
| **QR Code Only** | Offline QR scanning | Legacy systems, offline scenarios |
| **Webhook Events** | Push-based event notifications | Real-time alerting |
| **Database Sync** | Direct database integration | Enterprise systems |

---

## System Architecture

### 3.1 Overall Ecosystem Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         WIA Cloud Platform                          │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐  │
│  │  Passport API    │  │  Translation     │  │  QR Code         │  │
│  │  Service         │  │  Service         │  │  Service         │  │
│  └──────────────────┘  └──────────────────┘  └──────────────────┘  │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐  │
│  │  Verification    │  │  WebSocket       │  │  Analytics       │  │
│  │  Service         │  │  Event Hub       │  │  Service         │  │
│  └──────────────────┘  └──────────────────┘  └──────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
                                 │
                    ┌────────────┴────────────┐
                    │                         │
          ┌─────────▼─────────┐     ┌────────▼────────┐
          │  HTTPS/REST API   │     │  WebSocket API  │
          └─────────┬─────────┘     └────────┬────────┘
                    │                        │
    ┌───────────────┼────────────────────────┼───────────────┐
    │               │                        │               │
┌───▼───┐      ┌───▼───┐              ┌────▼────┐      ┌───▼────┐
│Mobile │      │  Web  │              │Restaurant│      │Airline │
│ Apps  │      │  Apps │              │   POS   │      │Systems │
└───────┘      └───────┘              └────┬────┘      └───┬────┘
                                           │                │
                                      ┌────▼────┐      ┌───▼────┐
                                      │ Kitchen │      │Catering│
                                      │Display  │      │ System │
                                      └─────────┘      └────────┘
```

### 3.2 Component Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Client Application                       │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │  UI Layer    │  │  Business    │  │  Data Layer  │      │
│  │              │  │  Logic       │  │              │      │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘      │
│         │                 │                 │              │
│  ┌──────▼─────────────────▼─────────────────▼───────┐      │
│  │         WIA Food Allergy Passport SDK            │      │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐       │      │
│  │  │API Client│  │QR Scanner│  │WebSocket │       │      │
│  │  └──────────┘  └──────────┘  └──────────┘       │      │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐       │      │
│  │  │Translation│  │Encryption│  │ Storage  │       │      │
│  │  └──────────┘  └──────────┘  └──────────┘       │      │
│  └──────────────────────────────────────────────────┘      │
│                         │                                   │
│                    ┌────▼────┐                             │
│                    │ Network │                             │
│                    │  Layer  │                             │
│                    └────┬────┘                             │
└─────────────────────────┼──────────────────────────────────┘
                          │
                   ┌──────▼──────┐
                   │ WIA API     │
                   │  Gateway    │
                   └─────────────┘
```

### 3.3 Data Flow Diagram

```
Passport Holder          Restaurant Staff          Kitchen              Server
     │                         │                      │                  │
     ├─ Show QR Code ─────────>│                      │                  │
     │                         │                      │                  │
     │                         ├─ Scan QR ────────────┼──────────────────>│
     │                         │                      │                  │
     │                         │                      │         WIA API  │
     │                         │                      │        ┌─────────▼────────┐
     │                         │                      │        │ Decode QR        │
     │                         │                      │        │ Verify Passport  │
     │                         │                      │        │ Get Allergies    │
     │                         │                      │        └─────────┬────────┘
     │                         │                      │                  │
     │                         │<─ Allergy Info ──────┼──────────────────┤
     │                         │  (Peanut Allergy)    │                  │
     │                         │                      │                  │
     │                         ├─ Alert Kitchen ─────>│                  │
     │                         │  "Table 15: Severe   │                  │
     │                         │   Peanut Allergy"    │                  │
     │                         │                      │                  │
     │                         │                      ├─ Acknowledge ────>│
     │                         │                      │                  │
     │<─ Confirmation ─────────┤<─ Kitchen Notified ──┤                  │
     │  "Kitchen Aware"        │                      │                  │
     │                         │                      │                  │
```

---

## Restaurant Integration

### 4.1 POS System Integration

#### Integration Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Restaurant POS System                    │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │  Order Mgmt  │  │  Table Mgmt  │  │  Billing     │      │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘      │
│         │                 │                 │              │
│  ┌──────▼─────────────────▼─────────────────▼───────┐      │
│  │        WIA Allergy Passport Plugin               │      │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐       │      │
│  │  │QR Scanner│  │ Alert UI │  │API Client│       │      │
│  │  └──────────┘  └──────────┘  └──────────┘       │      │
│  └──────────────────────────────────────────────────┘      │
│                         │                                   │
│  ┌──────────────────────▼────────────────────────┐         │
│  │         Kitchen Display System (KDS)          │         │
│  │  - Real-time Allergy Alerts                   │         │
│  │  - Order Modifications                        │         │
│  │  - Cross-contamination Warnings               │         │
│  └───────────────────────────────────────────────┘         │
└─────────────────────────────────────────────────────────────┘
```

#### Sample POS Plugin Implementation

```typescript
// POS Plugin for WIA Food Allergy Passport
import { AllergyPassport, QRScanner } from '@wia/food-allergy-passport-pos';

class RestaurantPOSPlugin {
  private passport: AllergyPassport;
  private scanner: QRScanner;

  constructor(config: POSConfig) {
    this.passport = new AllergyPassport({
      apiKey: config.apiKey,
      restaurantId: config.restaurantId
    });

    this.scanner = new QRScanner({
      autoScan: true,
      alertOnSevere: true
    });
  }

  // Scan customer's allergy passport
  async scanPassport(tableNumber: string): Promise<AllergyInfo> {
    try {
      // 1. Scan QR code
      const qrData = await this.scanner.scan();

      // 2. Decode and verify
      const allergyInfo = await this.passport.scanQRCode(qrData);

      // 3. Attach to table
      await this.attachToTable(tableNumber, allergyInfo);

      // 4. Alert kitchen if severe allergies
      if (this.hasSevereAllergies(allergyInfo)) {
        await this.alertKitchen(tableNumber, allergyInfo);
      }

      // 5. Display on POS screen
      this.displayAllergyAlert(allergyInfo);

      return allergyInfo;
    } catch (error) {
      console.error('Failed to scan passport:', error);
      throw error;
    }
  }

  // Alert kitchen display system
  private async alertKitchen(table: string, info: AllergyInfo) {
    const alert = {
      type: 'SEVERE_ALLERGY',
      table: table,
      allergies: info.allergies.map(a => ({
        allergen: a.allergen,
        severity: a.severity
      })),
      message: this.generateAlertMessage(info),
      requiresAcknowledgment: true
    };

    await this.passport.notifyRestaurant(
      this.config.restaurantId,
      alert
    );
  }

  // Check menu item safety
  async checkMenuItem(menuItemId: string, passportId: string): Promise<SafetyCheck> {
    const safetyCheck = await this.passport.checkMenuItemSafety(
      passportId,
      menuItemId
    );

    if (!safetyCheck.isSafe) {
      this.showWarning(
        `⚠️ ${safetyCheck.menuItem.name} contains ${safetyCheck.conflictingAllergens.join(', ')}`
      );
    }

    return safetyCheck;
  }

  // Get safe menu recommendations
  async getSafeMenuItems(passportId: string): Promise<MenuItem[]> {
    const menu = await this.passport.getRestaurantMenu(
      this.config.restaurantId,
      passportId
    );

    return menu.filter(item => item.safetyScore >= 90);
  }
}
```

### 4.2 Kitchen Display System (KDS) Integration

```typescript
class KitchenDisplayIntegration {
  private eventStream: WebSocket;

  constructor(restaurantId: string) {
    // Connect to real-time event stream
    this.eventStream = new WebSocket(
      `wss://ws.wia.live/food-allergy-passport/v1/restaurant/${restaurantId}`
    );

    this.eventStream.on('message', this.handleEvent.bind(this));
  }

  private handleEvent(event: AllergyEvent) {
    if (event.type === 'alert.allergy.critical') {
      this.displayCriticalAlert(event);
    } else if (event.type === 'order.allergy-modified') {
      this.updateOrderDisplay(event);
    }
  }

  private displayCriticalAlert(event: CriticalAllergyEvent) {
    // Display prominent alert on kitchen screens
    const alert = {
      priority: 'CRITICAL',
      table: event.data.tableNumber,
      allergens: event.data.allergies,
      message: `⚠️ SEVERE ALLERGY: ${event.data.allergies.join(', ')}`,
      color: '#FF0000',
      soundAlert: true,
      requiresChefAcknowledgment: true
    };

    this.showAlert(alert);
  }
}
```

### 4.3 Menu Integration

```json
{
  "menuItem": {
    "id": "MENU-123",
    "name": "Thai Peanut Curry",
    "description": "Spicy curry with peanut sauce",
    "ingredients": [
      "chicken",
      "coconut milk",
      "peanuts",
      "red curry paste",
      "vegetables"
    ],
    "allergens": [
      {
        "allergenCode": "FDA-PEANUT",
        "name": "Peanuts",
        "level": "primary-ingredient"
      },
      {
        "allergenCode": "FDA-SHELLFISH",
        "name": "Shrimp (in curry paste)",
        "level": "trace"
      }
    ],
    "allergyWarnings": {
      "contains": ["peanuts"],
      "mayContain": ["shellfish", "tree nuts"],
      "crossContamination": ["shared kitchen equipment with peanuts"]
    },
    "modifications": {
      "noPeanuts": {
        "available": false,
        "reason": "Peanuts are primary ingredient"
      }
    }
  }
}
```

---

## Airline Integration

### 5.1 Airline Catering System Integration

```
┌─────────────────────────────────────────────────────────────┐
│              Airline Reservation System                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │  Booking     │  │  Seat        │  │  Special     │      │
│  │  System      │  │  Assignment  │  │  Requests    │      │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘      │
│         │                 │                 │              │
│  ┌──────▼─────────────────▼─────────────────▼───────┐      │
│  │    WIA Allergy Passport Integration             │      │
│  │  - Meal Selection based on Allergies            │      │
│  │  - Automatic Special Meal Requests (SPML)       │      │
│  │  - Crew Notifications                           │      │
│  └──────────────────────────────────────────────────┘      │
│                         │                                   │
│  ┌──────────────────────▼────────────────────────┐         │
│  │         Catering Management System             │         │
│  │  - Prepare Allergy-Safe Meals                 │         │
│  │  - Label Meals with Allergy Info              │         │
│  │  - Track Special Meal Delivery                │         │
│  └───────────────────────────────────────────────┘         │
└─────────────────────────────────────────────────────────────┘
```

### 5.2 Flight Meal Selection

```typescript
import { AllergyPassport } from '@wia/food-allergy-passport-airline';

class AirlineMealSystem {
  private passport: AllergyPassport;

  async submitPassportForFlight(
    passportId: string,
    pnr: string,
    flightNumber: string
  ): Promise<MealRecommendation> {
    // 1. Submit passport to airline
    const submission = await this.passport.submitToAirline(passportId, {
      airline: this.config.airlineCode,
      flightNumber: flightNumber,
      pnr: pnr
    });

    // 2. Get safe meal options
    const meals = await this.passport.getAirlineMealOptions(
      passportId,
      submission.flightId
    );

    // 3. Automatically select safest meal
    const safeMeal = meals.find(m => m.safe && m.safetyScore >= 95);

    if (safeMeal) {
      await this.selectMeal(pnr, safeMeal.code);
    }

    // 4. Notify crew
    await this.notifyFlightCrew(flightNumber, passportId);

    return {
      selectedMeal: safeMeal,
      alternativeMeals: meals.filter(m => m.safe),
      unsafeMeals: meals.filter(m => !m.safe)
    };
  }
}
```

### 5.3 Special Meal Codes (IATA SPML)

```typescript
const allergyToSPML = {
  'FDA-PEANUT': ['NLML'],      // No Nut Meal
  'FDA-TREENUT': ['NLML'],     // No Nut Meal
  'FDA-SHELLFISH': ['SFML'],   // Seafood Meal (exclude shellfish)
  'FDA-MILK': ['LPML'],        // Low Lactose Meal
  'FDA-WHEAT': ['GFML'],       // Gluten Free Meal
  'FDA-EGG': ['AVML'],         // Asian Vegetarian Meal (egg-free)
};

// Map allergies to appropriate IATA meal codes
function mapAllergiesToMealCodes(allergies: AllergyEntry[]): string[] {
  const mealCodes = new Set<string>();

  allergies.forEach(allergy => {
    const codes = allergyToSPML[allergy.allergenCode];
    if (codes) {
      codes.forEach(code => mealCodes.add(code));
    }
  });

  return Array.from(mealCodes);
}
```

---

## Healthcare Integration

### 6.1 Hospital Food Service Integration

```typescript
class HospitalFoodService {
  private passport: AllergyPassport;
  private ehr: EHRSystem;

  async integrateWithEHR(patientId: string, passportId: string) {
    // 1. Fetch allergy passport
    const allergyData = await this.passport.getPassport(passportId);

    // 2. Convert to EHR allergy format (HL7 FHIR)
    const fhirAllergies = this.convertToFHIR(allergyData);

    // 3. Update EHR
    await this.ehr.updatePatientAllergies(patientId, fhirAllergies);

    // 4. Configure hospital meal plan
    await this.createMealPlan(patientId, allergyData);
  }

  private convertToFHIR(allergyData: PassportRecord): FHIRAllergyIntolerance[] {
    return allergyData.allergies.map(allergy => ({
      resourceType: 'AllergyIntolerance',
      clinicalStatus: {
        coding: [{
          system: 'http://terminology.hl7.org/CodeSystem/allergyintolerance-clinical',
          code: 'active'
        }]
      },
      verificationStatus: {
        coding: [{
          system: 'http://terminology.hl7.org/CodeSystem/allergyintolerance-verification',
          code: allergyData.verification.medicallyVerified ? 'confirmed' : 'unconfirmed'
        }]
      },
      type: 'allergy',
      category: ['food'],
      criticality: this.mapSeverityToCriticality(allergy.severity),
      code: {
        coding: [{
          system: 'http://snomed.info/sct',
          code: this.getSNOMEDCode(allergy.allergenCode),
          display: allergy.allergen
        }],
        text: allergy.allergen
      },
      patient: {
        reference: `Patient/${patientId}`
      },
      onsetDateTime: allergy.firstDiagnosed,
      reaction: allergy.symptoms.map(symptom => ({
        manifestation: [{
          coding: [{
            system: 'http://snomed.info/sct',
            display: symptom.description
          }]
        }],
        severity: allergy.severity
      }))
    }));
  }
}
```

---

## Mobile Applications

### 7.1 iOS SDK Integration

```swift
import WIAFoodAllergyPassport

class AllergyPassportViewController: UIViewController {
    private let passport = AllergyPassport(
        config: AllergyPassportConfig(
            apiKey: "your-api-key",
            environment: .production
        )
    )

    // Create new passport
    func createPassport() async throws {
        let passportData = PassportCreationData(
            holder: HolderInfo(
                userId: "USER-001",
                displayName: "John Doe",
                dateOfBirth: "1990-05-15"
            ),
            allergies: [
                AllergyEntry(
                    allergen: "peanuts",
                    allergenCode: "FDA-PEANUT",
                    icdCode: "Z91.010",
                    severity: .anaphylaxis,
                    symptoms: [
                        Symptom(
                            type: .respiratory,
                            description: "difficulty breathing",
                            onsetTime: "5-10 minutes"
                        )
                    ]
                )
            ],
            medications: Medications(
                epipen: EpiPen(
                    brand: "EpiPen",
                    dosage: "0.3mg",
                    location: "always carried"
                )
            )
        )

        let newPassport = try await passport.createPassport(data: passportData)
        print("Passport created: \(newPassport.passportId)")

        // Generate QR code
        let qrCode = try await passport.generateQRCode(
            passportId: newPassport.passportId,
            options: QROptions(size: 512, errorCorrection: .high)
        )

        // Display QR code
        qrCodeImageView.image = UIImage(data: qrCode.imageData)
    }

    // Scan restaurant QR code
    func scanRestaurantQR() {
        let scanner = QRCodeScanner()
        scanner.scan { result in
            switch result {
            case .success(let qrData):
                Task {
                    let info = try await self.passport.scanQRCode(qrData)
                    self.displayAllergyInfo(info)
                }
            case .failure(let error):
                self.showError(error)
            }
        }
    }
}
```

### 7.2 Android SDK Integration

```kotlin
import live.wia.allergypassport.AllergyPassport
import live.wia.allergypassport.models.*

class AllergyPassportActivity : AppCompatActivity() {
    private lateinit var passport: AllergyPassport

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        passport = AllergyPassport(
            config = AllergyPassportConfig(
                apiKey = "your-api-key",
                environment = Environment.PRODUCTION
            )
        )
    }

    // Create passport
    suspend fun createPassport() {
        val passportData = PassportCreationData(
            holder = HolderInfo(
                userId = "USER-001",
                displayName = "John Doe",
                dateOfBirth = "1990-05-15"
            ),
            allergies = listOf(
                AllergyEntry(
                    allergen = "peanuts",
                    allergenCode = "FDA-PEANUT",
                    icdCode = "Z91.010",
                    severity = Severity.ANAPHYLAXIS,
                    symptoms = listOf(
                        Symptom(
                            type = SymptomType.RESPIRATORY,
                            description = "difficulty breathing",
                            onsetTime = "5-10 minutes"
                        )
                    )
                )
            )
        )

        val newPassport = passport.createPassport(passportData)
        Log.d("Passport", "Created: ${newPassport.passportId}")

        // Generate QR code
        val qrCode = passport.generateQRCode(
            passportId = newPassport.passportId,
            options = QROptions(size = 512, errorCorrection = ErrorCorrection.HIGH)
        )

        // Display QR code
        qrCodeImageView.setImageBitmap(qrCode.bitmap)
    }

    // Real-time alerts
    fun subscribeToAlerts(passportId: String) {
        passport.subscribeToEvents(passportId) { event ->
            when (event) {
                is AllergyEvent.CriticalAlert -> {
                    showCriticalAlert(event.data)
                }
                is AllergyEvent.PassportShared -> {
                    logShareEvent(event.data)
                }
            }
        }
    }
}
```

### 7.3 Web SDK Integration

```javascript
import { AllergyPassport, QRCodeGenerator } from '@wia/food-allergy-passport-web';

const passport = new AllergyPassport({
  apiKey: 'your-api-key',
  environment: 'production'
});

// Create passport
async function createPassport() {
  const newPassport = await passport.createPassport({
    holder: {
      userId: 'USER-001',
      displayName: 'John Doe',
      dateOfBirth: '1990-05-15'
    },
    allergies: [
      {
        allergen: 'peanuts',
        allergenCode: 'FDA-PEANUT',
        severity: 'anaphylaxis',
        symptoms: [
          {
            type: 'respiratory',
            description: 'difficulty breathing'
          }
        ]
      }
    ],
    medications: {
      epipen: {
        brand: 'EpiPen',
        dosage: '0.3mg'
      }
    }
  });

  console.log('Passport created:', newPassport.passportId);

  // Generate and display QR code
  const qrCode = await passport.generateQRCode(newPassport.passportId, {
    size: 512,
    format: 'png'
  });

  document.getElementById('qr-code').src = qrCode.data;
}

// Subscribe to real-time events
passport.on('alert.allergy.critical', (event) => {
  showNotification({
    title: 'Critical Allergy Alert',
    message: event.data.message,
    urgent: true
  });
});
```

---

## POS System Integration

### 8.1 Integration Patterns

#### Pattern 1: Plugin/Extension

```typescript
// Square POS Plugin Example
export default class WIAAllergyPlugin extends SquarePlugin {
  async onOrderCreated(order: Order, context: PluginContext) {
    // Check if table has allergy passport
    const allergyInfo = await this.getAllergyInfo(order.tableId);

    if (allergyInfo) {
      // Validate order items against allergies
      const unsafeItems = await this.validateOrderItems(
        order.items,
        allergyInfo.allergies
      );

      if (unsafeItems.length > 0) {
        context.showWarning({
          title: '⚠️ Allergy Warning',
          message: `The following items conflict with customer allergies: ${unsafeItems.join(', ')}`,
          actions: ['Cancel', 'Override']
        });
      }
    }
  }
}
```

#### Pattern 2: API Integration

```python
# Toast POS API Integration
from wia_food_allergy_passport import AllergyPassport

class ToastPOSIntegration:
    def __init__(self, toast_api_key, wia_api_key):
        self.toast_api = ToastAPI(toast_api_key)
        self.passport = AllergyPassport(api_key=wia_api_key)

    async def process_order(self, order_guid):
        # Get order details from Toast
        order = await self.toast_api.get_order(order_guid)

        # Check if table has allergy passport scanned
        allergy_data = await self.get_table_allergy_data(order.table_id)

        if allergy_data:
            # Validate menu items
            for item in order.items:
                safety_check = await self.passport.check_menu_item_safety(
                    passport_id=allergy_data.passport_id,
                    menu_item_id=item.menu_item_guid
                )

                if not safety_check.is_safe:
                    # Flag order item
                    await self.toast_api.add_order_note(
                        order_guid,
                        f"⚠️ ALLERGY WARNING: {item.name} - {safety_check.reason}"
                    )
```

---

## Deployment

### 9.1 Cloud Deployment Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      Load Balancer                          │
│                    (AWS ALB / CloudFlare)                   │
└─────────────────┬───────────────────────┬───────────────────┘
                  │                       │
        ┌─────────▼─────────┐   ┌────────▼────────┐
        │  API Gateway      │   │  WebSocket      │
        │  (Kong / AWS)     │   │  Gateway        │
        └─────────┬─────────┘   └────────┬────────┘
                  │                      │
    ┌─────────────┼──────────────────────┼─────────────┐
    │             │                      │             │
┌───▼───┐   ┌────▼────┐          ┌──────▼──────┐  ┌───▼───┐
│Passport│   │  QR     │          │  WebSocket  │  │Trans- │
│Service │   │  Code   │          │  Event Hub  │  │lation │
│        │   │ Service │          │             │  │Service│
└───┬───┘   └────┬────┘          └──────┬──────┘  └───┬───┘
    │            │                      │             │
    └────────────┼──────────────────────┼─────────────┘
                 │                      │
         ┌───────▼──────────────────────▼────────┐
         │         Database Layer                │
         │  ┌──────────┐      ┌──────────┐       │
         │  │PostgreSQL│      │  Redis   │       │
         │  │(Primary) │      │ (Cache)  │       │
         │  └──────────┘      └──────────┘       │
         └────────────────────────────────────────┘
```

### 9.2 Deployment Options

| Option | Description | Use Case |
|--------|-------------|----------|
| **Cloud SaaS** | WIA-hosted cloud service | Small-medium restaurants |
| **On-Premise** | Self-hosted installation | Large enterprises, hospitals |
| **Hybrid** | Cloud + local caching | Airlines, international chains |
| **Edge** | CDN-distributed | Global restaurant chains |

### 9.3 Docker Deployment

```yaml
# docker-compose.yml
version: '3.8'

services:
  api-gateway:
    image: wia/food-allergy-passport-api:latest
    ports:
      - "443:443"
    environment:
      - DATABASE_URL=postgresql://user:pass@db:5432/allergy_passport
      - REDIS_URL=redis://redis:6379
      - JWT_SECRET=${JWT_SECRET}
    depends_on:
      - db
      - redis

  websocket-service:
    image: wia/food-allergy-passport-ws:latest
    ports:
      - "8080:8080"
    environment:
      - REDIS_URL=redis://redis:6379

  qr-service:
    image: wia/food-allergy-passport-qr:latest
    environment:
      - API_URL=http://api-gateway

  db:
    image: postgres:15
    volumes:
      - postgres-data:/var/lib/postgresql/data
    environment:
      - POSTGRES_DB=allergy_passport
      - POSTGRES_USER=user
      - POSTGRES_PASSWORD=${DB_PASSWORD}

  redis:
    image: redis:7-alpine
    volumes:
      - redis-data:/data

volumes:
  postgres-data:
  redis-data:
```

---

## Certification & Compliance

### 10.1 WIA Certification Levels

| Level | Requirements | Benefits |
|-------|--------------|----------|
| **Bronze** | Basic QR support, API integration | Listed in WIA directory |
| **Silver** | Real-time alerts, translations | Marketing materials usage |
| **Gold** | Full API, medical verification | Priority support, co-branding |
| **Platinum** | 99.9% uptime, security audit | Featured partner status |

### 10.2 Certification Process

```
┌────────────────────────────────────────────────────────────┐
│  Step 1: Application                                       │
│  - Submit integration details                              │
│  - Provide technical architecture                          │
│  - Pay certification fee                                   │
└────────────────┬───────────────────────────────────────────┘
                 │
┌────────────────▼───────────────────────────────────────────┐
│  Step 2: Technical Review                                  │
│  - API compliance testing                                  │
│  - Security audit                                          │
│  - Data format validation                                  │
└────────────────┬───────────────────────────────────────────┘
                 │
┌────────────────▼───────────────────────────────────────────┐
│  Step 3: Field Testing                                     │
│  - 30-day pilot program                                    │
│  - User acceptance testing                                 │
│  - Performance monitoring                                  │
└────────────────┬───────────────────────────────────────────┘
                 │
┌────────────────▼───────────────────────────────────────────┐
│  Step 4: Certification Issued                              │
│  - Certificate and badge                                   │
│  - Listed in partner directory                             │
│  - Marketing support                                       │
└────────────────────────────────────────────────────────────┘
```

### 10.3 Compliance Requirements

#### HIPAA Compliance (Healthcare)

```
- Encrypted data at rest and in transit
- Audit logging of all access
- Business Associate Agreement (BAA)
- Regular security assessments
- Breach notification procedures
```

#### GDPR Compliance (EU)

```
- User consent for data processing
- Right to data portability
- Right to be forgotten
- Data minimization
- Privacy by design
```

#### FDA Regulations (US)

```
- Accurate allergen labeling
- Compliance with FALCPA
- Major allergen identification
- Cross-contamination disclosure
```

---

## Integration Examples

### 10.1 Complete Restaurant Integration Example

```typescript
// Complete end-to-end restaurant integration
import { AllergyPassport, RestaurantConfig } from '@wia/food-allergy-passport';

class RestaurantSystem {
  private passport: AllergyPassport;
  private pos: POSSystem;
  private kds: KitchenDisplay;

  async initialize() {
    // 1. Initialize WIA SDK
    this.passport = new AllergyPassport({
      apiKey: process.env.WIA_API_KEY,
      restaurantId: 'REST-12345',
      integrations: {
        restaurants: {
          posSystem: 'Square',
          kdsEnabled: true
        }
      }
    });

    // 2. Subscribe to real-time alerts
    this.passport.on('alert.allergy.critical', this.handleCriticalAlert.bind(this));

    // 3. Connect POS system
    await this.connectPOS();

    // 4. Connect kitchen display
    await this.connectKDS();
  }

  // Handle QR code scan at table
  async onCustomerSeated(tableNumber: string, qrData: string) {
    try {
      // 1. Scan and validate passport
      const allergyInfo = await this.passport.scanQRCode(qrData);

      // 2. Store in POS system
      await this.pos.attachCustomerData(tableNumber, {
        allergyPassportId: allergyInfo.passportId,
        allergies: allergyInfo.allergies,
        severity: allergyInfo.maxSeverity
      });

      // 3. Alert kitchen if severe allergies
      if (allergyInfo.maxSeverity >= 'severe') {
        await this.kds.showAlert({
          type: 'SEVERE_ALLERGY',
          table: tableNumber,
          allergies: allergyInfo.allergies,
          message: this.generateKitchenMessage(allergyInfo)
        });
      }

      // 4. Get safe menu items
      const safeMenu = await this.passport.getRestaurantMenu(
        'REST-12345',
        allergyInfo.passportId
      );

      // 5. Display on POS
      await this.pos.showSafeMenuItems(tableNumber, safeMenu);

      return {
        success: true,
        allergyInfo,
        safeMenuCount: safeMenu.length
      };
    } catch (error) {
      console.error('Failed to process allergy passport:', error);
      throw error;
    }
  }

  // Validate order before sending to kitchen
  async validateOrder(order: Order) {
    const allergyInfo = await this.pos.getCustomerData(order.tableNumber);

    if (!allergyInfo) return { valid: true };

    const violations = [];

    for (const item of order.items) {
      const safety = await this.passport.checkMenuItemSafety(
        allergyInfo.allergyPassportId,
        item.menuItemId
      );

      if (!safety.isSafe) {
        violations.push({
          item: item.name,
          reason: safety.reason,
          allergens: safety.conflictingAllergens
        });
      }
    }

    if (violations.length > 0) {
      return {
        valid: false,
        violations,
        action: 'REQUIRES_MANAGER_OVERRIDE'
      };
    }

    return { valid: true };
  }
}
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

<div align="center">

**WIA Food Allergy Passport Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
