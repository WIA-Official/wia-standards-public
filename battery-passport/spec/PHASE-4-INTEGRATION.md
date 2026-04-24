# WIA-BATTERY-PASSPORT: Phase 4 - Ecosystem Integration

**EU 배터리 여권 생태계 통합**
*EU compliance, automotive, energy grid, and recycling integrations*

홍익인간 (弘益人間) - Benefit All Humanity

---

## 1. Overview

This document specifies integrations with:
1. EU Battery Passport Infrastructure
2. WIA Ecosystem Standards
3. Automotive Industry Systems
4. Energy Grid Systems
5. Recycling Industry Networks
6. Supply Chain Platforms

---

## 2. EU Battery Passport Infrastructure

### 2.1 EU Central Registry

The EU Battery Passport requires registration with central infrastructure.

```typescript
interface EURegistration {
  // EU-assigned unique identifier
  eu_unique_identifier: string;      // Format: EU-BAT-YYYY-NNNNNNNNNN

  // Economic operator
  economic_operator_id: string;       // EORI number
  eu_representative_id?: string;      // For non-EU manufacturers

  // Registry status
  registration_date: ISO8601;
  status: "active" | "suspended" | "withdrawn";

  // Verification
  notified_body: string;
  certificate_of_conformity: string;
}
```

#### Integration Flow

```
┌─────────────────┐     ┌──────────────────┐     ┌───────────────────┐
│  WIA Battery    │────▶│  WIA Gateway     │────▶│  EU Central       │
│  Passport API   │     │  (Translation)   │     │  Registry         │
└─────────────────┘     └──────────────────┘     └───────────────────┘
                               │
                               ▼
                        ┌──────────────────┐
                        │  Catena-X        │
                        │  Battery Pass    │
                        └──────────────────┘
```

### 2.2 Catena-X Battery Pass Integration

```python
# Catena-X data model mapping
def map_to_catena_x(passport: BatteryPassport) -> CatenaXBatteryPass:
    """
    Map WIA Battery Passport to Catena-X Battery Pass format.
    """
    return CatenaXBatteryPass(
        # Identity
        identification=Identification(
            batteryIDDMCCode=passport.qr_code,
            manufactureId=passport.manufacturer.registration_number
        ),

        # Technical data
        technicalData=TechnicalData(
            stateOfCertifiedEnergy=passport.specifications.rated_capacity_wh,
            physicalDimensions=PhysicalDimensions(
                length=passport.identity.dimensions.length_mm,
                width=passport.identity.dimensions.width_mm,
                height=passport.identity.dimensions.height_mm
            )
        ),

        # Carbon footprint
        carbonFootprint=CarbonFootprint(
            productionCarbonFootprint=passport.carbon_footprint.total_kg_co2e,
            preProductionCarbonFootprint=passport.carbon_footprint.raw_material_acquisition,
            recycledCarbonFootprint=0
        ),

        # State
        stateOfHealth=StateOfHealth(
            stateOfHealthValue=passport.health.state_of_health_percent,
            lastUpdate=passport.health.last_bms_sync
        ),

        # Materials
        batteryMaterialsAndComposition=MaterialsAndComposition(
            criticalRawMaterials=[
                CriticalRawMaterial(
                    name="Cobalt",
                    weight=passport.materials.cobalt.weight_kg,
                    recycledContent=passport.materials.recycled_cobalt_percent
                ),
                CriticalRawMaterial(
                    name="Lithium",
                    weight=passport.materials.lithium.weight_kg,
                    recycledContent=passport.materials.recycled_lithium_percent
                )
            ]
        )
    )
```

### 2.3 Global Battery Alliance (GBA) Integration

```typescript
interface GBAPassportMapping {
  // GBA Battery Passport fields
  gba_id: string;
  provenance: GBAProvenance;
  traceability: GBATraceability;
  sustainability: GBASustainability;
  circularity: GBACircularity;
}

// Export to GBA format
async function exportToGBA(passport: BatteryPassport): Promise<GBAPassport> {
  return {
    version: "1.0",
    battery_id: {
      unique_identifier: passport.id,
      manufacturer_code: passport.manufacturer.registration_number,
      production_date: passport.identity.production_date
    },
    sustainability_metrics: {
      carbon_footprint: {
        total_co2e: passport.carbon_footprint.total_kg_co2e,
        per_kwh: passport.carbon_footprint.per_kwh_kg_co2e
      },
      recycled_content: {
        cobalt: passport.materials.recycled_cobalt_percent,
        lithium: passport.materials.recycled_lithium_percent,
        nickel: passport.materials.recycled_nickel_percent
      }
    }
  };
}
```

---

## 3. WIA Ecosystem Integration

### 3.1 WIA-INTENT Integration

Natural language queries for battery information.

```typescript
// INTENT query examples
const intentQueries = [
  {
    query: "배터리 건강 상태 알려줘",
    intent: "battery_health_status",
    response: {
      soh: 94.5,
      soc: 78.2,
      remaining_life_months: 72,
      recommendation: "정상 상태입니다"
    }
  },
  {
    query: "What's the carbon footprint of my EV battery?",
    intent: "carbon_footprint_query",
    response: {
      total_kg_co2e: 5250.5,
      class: "C",
      comparison: "Industry average is Class D"
    }
  },
  {
    query: "Is my battery ready for recycling?",
    intent: "recycling_readiness",
    response: {
      ready: false,
      current_soh: 94.5,
      recommendation: "Battery still has significant useful life"
    }
  }
];
```

### 3.2 WIA-OMNI-API Integration

```typescript
// Register with OMNI-API
const batteryCapabilities: OMNICapability = {
  standard: "WIA-BATTERY-PASSPORT",
  version: "1.0.0",
  endpoints: [
    {
      path: "/passport",
      methods: ["GET", "POST", "PUT"],
      description: "Battery passport CRUD"
    },
    {
      path: "/health",
      methods: ["GET", "POST"],
      description: "Health data management"
    },
    {
      path: "/soh/calculate",
      methods: ["GET"],
      description: "Calculate SOH"
    }
  ],
  events: [
    "health_update",
    "soh_threshold",
    "lifecycle_event"
  ]
};
```

### 3.3 WIA-PQ-CRYPTO Integration

Post-quantum secure signatures for long-term validity.

```typescript
interface PQSignature {
  // Battery passports may exist for decades
  // Quantum-resistant signatures ensure long-term validity
  algorithm: "CRYSTALS-Dilithium" | "SPHINCS+";
  public_key: string;
  signature: string;
  signed_at: ISO8601;
  valid_until: ISO8601;  // e.g., 50 years
}

// Sign passport with PQ crypto
async function signWithPQ(passport: BatteryPassport): Promise<PQSignature> {
  const { publicKey, privateKey } = await pqcrypto.generateKeyPair("CRYSTALS-Dilithium");

  const dataToSign = JSON.stringify({
    id: passport.id,
    identity: passport.identity,
    carbon_footprint: passport.carbon_footprint
  });

  const signature = await pqcrypto.sign(dataToSign, privateKey);

  return {
    algorithm: "CRYSTALS-Dilithium",
    public_key: publicKey,
    signature: signature,
    signed_at: new Date().toISOString(),
    valid_until: new Date(Date.now() + 50 * 365 * 24 * 60 * 60 * 1000).toISOString()
  };
}
```

---

## 4. Automotive Industry Integration

### 4.1 Vehicle Integration

```typescript
interface VehicleIntegration {
  // Vehicle identification
  vehicle_vin: string;
  vehicle_manufacturer: string;
  vehicle_model: string;
  vehicle_year: number;

  // Battery position
  position: "main" | "auxiliary" | "hvac";

  // BMS connection
  bms_protocol: "CAN" | "OBD-II" | "UDS";
  bms_interface_id: string;

  // OTA update capability
  ota_enabled: boolean;
}
```

#### OBD-II Integration

```python
# Read battery data via OBD-II
def read_battery_obd2(connection: OBD2Connection) -> BatteryHealthData:
    """
    Read battery health data via OBD-II PIDs.
    Standard EV battery PIDs (Mode 22).
    """

    # Manufacturer-specific PIDs (example: BMW i-series)
    pids = {
        'soc': 0x028C,           # State of Charge
        'soh': 0x028D,           # State of Health
        'pack_voltage': 0x028E,  # HV Battery Voltage
        'pack_current': 0x028F,  # HV Battery Current
        'temp_max': 0x0290,      # Max Cell Temperature
        'temp_min': 0x0291,      # Min Cell Temperature
    }

    data = {}
    for name, pid in pids.items():
        response = connection.query(0x22, pid)
        data[name] = decode_response(response)

    return BatteryHealthData(
        soc_percent=data['soc'] / 100,
        soh_percent=data['soh'] / 100,
        pack_voltage_v=data['pack_voltage'] / 10,
        pack_current_a=data['pack_current'] / 10 - 3276.8,  # signed
        temp_max_c=data['temp_max'] - 40,
        temp_min_c=data['temp_min'] - 40
    )
```

### 4.2 Manufacturer System Integration

```
┌────────────────┐     ┌──────────────────┐     ┌────────────────┐
│  BMW Connected │────▶│  WIA Battery     │◀────│  Tesla Fleet   │
│  Drive API     │     │  Passport API    │     │  API           │
└────────────────┘     └──────────────────┘     └────────────────┘
                              ▲
                              │
              ┌───────────────┴───────────────┐
              │                               │
     ┌────────┴────────┐             ┌────────┴────────┐
     │  VW WeConnect   │             │  Mercedes me    │
     │  API            │             │  API            │
     └─────────────────┘             └─────────────────┘
```

#### Example: BMW Integration

```typescript
class BMWBatteryPassportIntegration {
  private bmwApi: BMWConnectedDriveAPI;
  private wiaApi: WIABatteryPassportAPI;

  async syncBatteryData(vin: string): Promise<void> {
    // Get data from BMW
    const bmwData = await this.bmwApi.getBatteryStatus(vin);

    // Map to WIA format
    const healthUpdate = {
      state_of_charge_percent: bmwData.chargingStatus.chargingLevelPercent,
      state_of_health_percent: bmwData.batteryHealth?.stateOfHealth || null,
      pack_voltage_v: bmwData.electricalSystemStatus?.voltage,
      recorded_at: new Date().toISOString()
    };

    // Update WIA passport
    await this.wiaApi.updateHealth(bmwData.passportId, healthUpdate);
  }
}
```

---

## 5. Energy Grid Integration

### 5.1 Vehicle-to-Grid (V2G) Integration

```typescript
interface V2GIntegration {
  // Grid operator connection
  grid_operator_id: string;
  metering_point_id: string;

  // V2G capabilities
  bidirectional_capable: boolean;
  max_discharge_power_kw: number;
  max_charge_power_kw: number;

  // Energy trading
  energy_contract_id: string;
  current_tariff: Tariff;
}

// V2G session with battery passport data
interface V2GSession {
  session_id: string;
  passport_id: string;
  start_time: ISO8601;
  end_time?: ISO8601;

  // Energy flow
  energy_imported_kwh: number;
  energy_exported_kwh: number;

  // Battery impact
  soh_at_start: number;
  soh_at_end: number;
  cycles_added: number;

  // Revenue
  revenue_earned: Money;
}
```

### 5.2 Stationary Storage Integration

For second-life batteries in grid storage:

```typescript
interface StationaryStorageIntegration {
  // System identification
  storage_system_id: string;
  site_location: GeoLocation;
  grid_connection_point: string;

  // Battery configuration
  batteries: BatteryInSystem[];
  total_capacity_kwh: number;
  total_power_kw: number;

  // Aggregated health
  system_soh_percent: number;
  weakest_battery_soh: number;

  // Operations
  cycles_per_day: number;
  revenue_per_kwh: Money;
}

interface BatteryInSystem {
  passport_id: string;
  position: string;               // e.g., "Rack-A-1"
  original_application: string;   // e.g., "BMW i3"
  installation_date: ISO8601;
  current_soh: number;
}
```

### 5.3 ISO 15118 Integration

Plug & Charge with battery passport:

```typescript
// ISO 15118 Certificate with battery passport link
interface ISO15118Certificate {
  contract_id: string;
  vehicle_certificate: X509Certificate;
  battery_passport_url: string;

  // Authorization
  emaid: string;                  // E-Mobility Account ID
  contract_expiry: ISO8601;
}

// Charging session with passport update
async function chargingSession(
  vehicleCert: ISO15118Certificate,
  charger: EVSEInterface
): Promise<ChargingSessionResult> {

  // Authenticate
  const auth = await charger.authorizeISO15118(vehicleCert);

  // Get current battery state
  const passport = await getBatteryPassport(vehicleCert.battery_passport_url);

  // Start charging
  const session = await charger.startCharging({
    targetSoc: 80,
    maxPower: getOptimalChargePower(passport.health)
  });

  // Monitor and update passport
  session.on('complete', async (result) => {
    await updatePassportAfterCharging(passport.id, {
      energy_added_kwh: result.energyDelivered,
      charge_type: result.dcFastCharge ? 'fast' : 'slow',
      duration_minutes: result.duration
    });
  });

  return session;
}
```

---

## 6. Recycling Industry Integration

### 6.1 Recycler Network

```typescript
interface RecyclerNetwork {
  // Certified recyclers
  recyclers: CertifiedRecycler[];

  // Material exchange
  material_marketplace: MaterialExchange;

  // Traceability
  recycling_chain: RecyclingChainEvent[];
}

interface CertifiedRecycler {
  id: string;
  name: string;
  license_number: string;
  country: ISO3166Alpha2;

  // Certifications
  certifications: RecyclerCertification[];

  // Capabilities
  processes: RecyclingProcess[];
  accepted_chemistries: BatteryChemistry[];
  capacity_tons_per_year: number;

  // Performance
  avg_recovery_efficiency: MaterialRecoveryRates;
}

enum RecyclingProcess {
  PYROMETALLURGICAL = "pyrometallurgical",
  HYDROMETALLURGICAL = "hydrometallurgical",
  DIRECT_RECYCLING = "direct_recycling",
  MECHANICAL = "mechanical"
}
```

### 6.2 End-of-Life Flow

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│   EV Owner   │────▶│  Collection  │────▶│  Assessment  │
│              │     │  Point       │     │  Center      │
└──────────────┘     └──────────────┘     └──────────────┘
                                                │
                         ┌──────────────────────┼──────────────────────┐
                         ▼                      ▼                      ▼
                  ┌──────────────┐     ┌──────────────┐     ┌──────────────┐
                  │  Second-Life │     │   Repair/    │     │  Recycling   │
                  │  Application │     │   Refurbish  │     │  Facility    │
                  └──────────────┘     └──────────────┘     └──────────────┘
                                                                  │
                                                                  ▼
                                                          ┌──────────────┐
                                                          │  Material    │
                                                          │  Recovery    │
                                                          └──────────────┘
                                                                  │
                                                                  ▼
                                                          ┌──────────────┐
                                                          │  New Battery │
                                                          │  Production  │
                                                          └──────────────┘
```

### 6.3 Material Traceability

```typescript
interface MaterialTraceability {
  // From mine to battery
  origin: {
    mine_id: string;
    mine_name: string;
    country: ISO3166Alpha2;
    extraction_date: ISO8601;
  };

  // Processing chain
  processing_chain: ProcessingStep[];

  // To battery
  battery_integration: {
    passport_id: string;
    integration_date: ISO8601;
    component: string;            // e.g., "cathode"
  };

  // After recycling
  recycled_into?: {
    new_passport_id: string;
    recycling_date: ISO8601;
    recovery_rate: number;
  };
}
```

---

## 7. Supply Chain Platforms

### 7.1 SAP Integration

```typescript
// SAP S/4HANA Material Management integration
interface SAPMaterialManagement {
  // Material master
  material_number: string;
  plant: string;
  storage_location: string;

  // Batch management
  batch_number: string;
  production_date: Date;
  expiry_date?: Date;

  // Quality management
  qm_inspection_lot: string;
  quality_status: "unrestricted" | "blocked" | "quality_inspection";

  // Link to passport
  passport_id: string;
}

// Integration via SAP Business Technology Platform
class SAPBatteryPassportIntegration {
  async createMaterialWithPassport(passport: BatteryPassport): Promise<string> {
    const materialDoc = {
      MaterialNumber: generateMaterialNumber(passport),
      Plant: passport.identity.production_facility_id,
      BatchNumber: passport.identity.batch_number,
      WIAPassportID: passport.id,
      // ... additional SAP fields
    };

    return await this.sapApi.createMaterial(materialDoc);
  }
}
```

### 7.2 Oracle SCM Integration

```typescript
interface OracleSCMIntegration {
  // Product Lifecycle Management
  plm_item_number: string;
  revision: string;

  // Supply Chain Planning
  planning_item: string;
  supply_sources: SupplySource[];

  // Quality Management
  quality_inspection_id: string;
}
```

---

## 8. Mobile SDK

### 8.1 iOS SDK

```swift
import WIABatteryPassport

class BatteryPassportViewController: UIViewController {

    let sdk = WIABatteryPassportSDK(apiKey: "your_api_key")

    func scanQRCode() {
        let scanner = sdk.createQRScanner()

        scanner.scan { result in
            switch result {
            case .success(let passport):
                self.displayPassport(passport)
            case .failure(let error):
                self.showError(error)
            }
        }
    }

    func displayPassport(_ passport: BatteryPassport) {
        // Health summary
        let health = passport.health
        self.sohLabel.text = "\(health.stateOfHealthPercent)%"
        self.socLabel.text = "\(health.stateOfChargePercent)%"

        // Carbon footprint
        let carbon = passport.carbonFootprint
        self.carbonLabel.text = "\(carbon.totalKgCO2e) kg CO2e"
        self.carbonClassView.setClass(carbon.performanceClass)

        // RUL prediction
        sdk.predictRUL(passportId: passport.id) { rul in
            self.rulLabel.text = "\(rul.remainingMonths) months"
        }
    }

    func checkSecondLife() {
        sdk.assessSecondLife(passportId: currentPassportId) { result in
            if result.eligible {
                self.showSecondLifeOptions(result.suggestedApplications)
            }
        }
    }
}
```

### 8.2 Android SDK

```kotlin
import org.wia.batterypassport.sdk.*

class BatteryPassportActivity : AppCompatActivity() {

    private val sdk = WIABatteryPassportSDK.Builder()
        .apiKey("your_api_key")
        .build()

    fun scanQRCode() {
        val scanner = sdk.createQRScanner(this)

        scanner.scan { result ->
            when (result) {
                is ScanResult.Success -> displayPassport(result.passport)
                is ScanResult.Error -> showError(result.error)
            }
        }
    }

    fun displayPassport(passport: BatteryPassport) {
        // Health summary
        binding.sohText.text = "${passport.health.stateOfHealthPercent}%"
        binding.socText.text = "${passport.health.stateOfChargePercent}%"

        // Carbon footprint
        binding.carbonText.text = "${passport.carbonFootprint.totalKgCO2e} kg CO2e"
        binding.carbonClassBadge.setClass(passport.carbonFootprint.performanceClass)

        // Lifecycle visualization
        binding.lifecycleTimeline.setEvents(passport.lifecycle)
    }

    suspend fun getSupplyChainInfo() {
        val verification = sdk.verifySupplyChain(currentPassportId)

        binding.supplyChainRisk.text = when (verification.overallRiskScore) {
            in 0..30 -> "Low Risk"
            in 31..60 -> "Medium Risk"
            else -> "High Risk"
        }

        binding.materialList.adapter = MaterialVerificationAdapter(
            verification.materialVerifications
        )
    }
}
```

### 8.3 Flutter SDK

```dart
import 'package:wia_battery_passport/wia_battery_passport.dart';

class BatteryPassportScreen extends StatefulWidget {
  @override
  _BatteryPassportScreenState createState() => _BatteryPassportScreenState();
}

class _BatteryPassportScreenState extends State<BatteryPassportScreen> {
  final sdk = WIABatteryPassportSDK(apiKey: 'your_api_key');
  BatteryPassport? passport;

  Future<void> scanQR() async {
    final result = await sdk.scanQRCode();
    setState(() {
      passport = result;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text('Battery Passport')),
      body: passport == null
          ? Center(child: ElevatedButton(
              onPressed: scanQR,
              child: Text('Scan QR Code'),
            ))
          : BatteryPassportView(
              passport: passport!,
              onCheckSecondLife: () => _checkSecondLife(),
              onViewCarbonDetails: () => _showCarbonDetails(),
            ),
    );
  }

  Widget BatteryPassportView({
    required BatteryPassport passport,
    required VoidCallback onCheckSecondLife,
    required VoidCallback onViewCarbonDetails,
  }) {
    return SingleChildScrollView(
      child: Column(
        children: [
          // Health Card
          HealthSummaryCard(
            soh: passport.health.stateOfHealthPercent,
            soc: passport.health.stateOfChargePercent,
            rul: passport.health.remainingUsefulLifeMonths,
          ),

          // Carbon Card
          CarbonFootprintCard(
            totalCO2e: passport.carbonFootprint.totalKgCO2e,
            perKwh: passport.carbonFootprint.perKwhKgCO2e,
            carbonClass: passport.carbonFootprint.performanceClass,
          ),

          // Lifecycle Timeline
          LifecycleTimeline(events: passport.lifecycle),

          // Actions
          ActionButtons(
            onCheckSecondLife: onCheckSecondLife,
            onViewCarbonDetails: onViewCarbonDetails,
          ),
        ],
      ),
    );
  }
}
```

---

## 9. IoT Device Integration

### 9.1 BMS IoT Gateway

```typescript
interface BMSIoTGateway {
  // Device identification
  device_id: string;
  firmware_version: string;

  // Connectivity
  connection_type: "cellular" | "wifi" | "lorawan";
  mqtt_broker: string;
  mqtt_topic: string;

  // Data transmission
  upload_interval_seconds: number;
  batch_upload: boolean;

  // Security
  device_certificate: X509Certificate;
  data_encryption: "AES-256-GCM";
}

// MQTT message format
interface BMSMQTTMessage {
  topic: "wia/battery/{passport_id}/health";
  payload: {
    timestamp: ISO8601;
    soc: number;
    soh: number;
    voltage: number;
    current: number;
    temperature: number;
    cell_voltages?: number[];
  };
  qos: 1;
  retain: false;
}
```

### 9.2 Edge Computing

```python
# Edge device for local processing
class BatteryPassportEdgeDevice:
    """
    Edge device that processes BMS data locally
    and syncs to cloud when connected.
    """

    def __init__(self, passport_id: str, bms_interface: BMSInterface):
        self.passport_id = passport_id
        self.bms = bms_interface
        self.local_db = SQLiteDatabase("battery_data.db")
        self.cloud_api = WIABatteryPassportAPI()

    async def collect_data(self):
        """Continuous data collection."""
        while True:
            data = await self.bms.read_data()

            # Local processing
            processed = self.process_locally(data)

            # Store locally
            await self.local_db.insert(processed)

            # Sync if connected
            if self.cloud_api.is_connected():
                await self.sync_to_cloud()

            await asyncio.sleep(60)  # Every minute

    def process_locally(self, raw_data: BMSRawData) -> ProcessedData:
        """Edge processing - anomaly detection, averaging."""
        return ProcessedData(
            timestamp=datetime.utcnow(),
            soc=raw_data.soc,
            soh=self.calculate_soh_locally(raw_data),
            anomalies=self.detect_anomalies(raw_data)
        )

    async def sync_to_cloud(self):
        """Batch sync to WIA cloud."""
        pending = await self.local_db.get_unsynced()

        for batch in chunks(pending, 100):
            await self.cloud_api.batch_upload(self.passport_id, batch)
            await self.local_db.mark_synced(batch)
```

---

## 10. Compliance Dashboard

### 10.1 EU Compliance Monitoring

```typescript
interface ComplianceDashboard {
  // Overall compliance status
  eu_battery_regulation_compliant: boolean;
  compliance_score: number;  // 0-100

  // Individual requirements
  requirements: ComplianceRequirement[];

  // Upcoming deadlines
  deadlines: ComplianceDeadline[];

  // Audit trail
  audit_log: AuditEvent[];
}

interface ComplianceRequirement {
  requirement_id: string;
  description: string;
  category: "mandatory" | "recommended";
  status: "compliant" | "non_compliant" | "pending" | "not_applicable";
  evidence_url?: string;
  last_checked: ISO8601;
}

// Example requirements check
const requirements: ComplianceRequirement[] = [
  {
    requirement_id: "EU-BR-2027-01",
    description: "Digital battery passport required",
    category: "mandatory",
    status: "compliant",
    evidence_url: "https://battery.wia.org/p/..."
  },
  {
    requirement_id: "EU-BR-2027-02",
    description: "Carbon footprint declared",
    category: "mandatory",
    status: "compliant",
    evidence_url: "..."
  },
  {
    requirement_id: "EU-BR-2031-01",
    description: "Recycled content minimum (cobalt 16%)",
    category: "mandatory",
    status: "pending",  // Requirement starts 2031
  }
];
```

---

**Document ID**: WIA-BATTERY-PASSPORT-PHASE-4
**Version**: 1.0.0
**Date**: 2025-12-16
**Status**: Draft
