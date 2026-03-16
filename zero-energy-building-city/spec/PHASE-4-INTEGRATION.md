# WIA-CITY-005: Zero Energy Building - PHASE 4 INTEGRATION

**Version:** 1.0
**Status:** Active
**Category:** CITY
**Last Updated:** 2025-12-25

---

## 1. Overview

This document specifies how Zero Energy Buildings integrate with renewable energy systems, energy storage, smart grids, building management systems, and external services to achieve optimal performance.

---

## 2. Solar PV Integration

### 2.1 System Architecture

```
┌──────────────────────────────────────────────────────┐
│                  SOLAR PV SYSTEM                     │
├──────────────────────────────────────────────────────┤
│                                                      │
│  ☀️ Solar Panels                                    │
│  ├── Rooftop Arrays                                 │
│  ├── Facade-Integrated PV (BIPV)                    │
│  ├── Carport/Canopy Systems                         │
│  └── Ground-Mounted (if space available)            │
│         │                                            │
│         ├── DC Optimizers (per panel or string)     │
│         │                                            │
│         ▼                                            │
│  ⚡ Inverters                                        │
│  ├── String Inverters                               │
│  ├── Microinverters                                 │
│  └── Central Inverters (large systems)              │
│         │                                            │
│         ▼                                            │
│  📊 Monitoring & Control                            │
│  ├── Production Metering                            │
│  ├── Performance Analytics                          │
│  ├── Fault Detection                                │
│  └── MPPT Optimization                              │
│         │                                            │
│         ▼                                            │
│  🔌 AC Distribution                                 │
│  ├── Building Loads                                 │
│  ├── ESS Charging                                   │
│  └── Grid Export                                    │
│                                                      │
└──────────────────────────────────────────────────────┘
```

### 2.2 Integration Requirements

#### 2.2.1 Electrical Integration

**Interconnection:**
- Comply with IEEE 1547 (grid interconnection)
- Anti-islanding protection
- Voltage and frequency regulation
- Power quality (THD < 5%)
- Ground fault protection

**AC Coupling:**
- Direct connection to building electrical panel
- Load-side or line-side interconnection
- Proper sizing of backfeed breakers
- Busbar rating compliance

**DC Coupling (with ESS):**
- Shared DC bus with battery storage
- Hybrid inverter or separate charge controller
- More efficient for self-consumption
- Reduced conversion losses

#### 2.2.2 Communication Protocols

**Supported Standards:**
- **Modbus RTU/TCP**: Industry standard for monitoring
- **SunSpec**: Solar-specific Modbus extension
- **MQTT**: Lightweight IoT messaging
- **REST API**: Web-based integration
- **DNP3**: Utility-grade protocol for large systems

**Data Points:**
- DC voltage, current, power (per string)
- AC voltage, current, power, frequency
- Total energy production (kWh)
- Inverter efficiency and temperature
- Fault codes and alarms

**Example Modbus Register Map:**

| Register | Description | Unit | Type |
|----------|-------------|------|------|
| 40001 | DC Voltage | V × 10 | uint16 |
| 40002 | DC Current | A × 100 | uint16 |
| 40003 | DC Power | W | uint16 |
| 40004 | AC Voltage L1 | V × 10 | uint16 |
| 40005 | AC Current L1 | A × 100 | uint16 |
| 40006 | AC Power Total | W | uint32 |
| 40008 | Energy Today | Wh | uint32 |
| 40010 | Energy Total | kWh | uint32 |
| 40012 | Temperature | °C | int16 |
| 40013 | Status | Enum | uint16 |

### 2.3 Solar Monitoring Integration

#### 2.3.1 Real-Time Monitoring

```javascript
// Example: SunSpec Modbus integration
class SolarInverter {
  constructor(modbusClient, slaveId) {
    this.client = modbusClient;
    this.slaveId = slaveId;
  }

  async getCurrentProduction() {
    // Read SunSpec common model (101)
    const data = await this.client.readHoldingRegisters(
      this.slaveId,
      40000,  // Starting register
      50      // Number of registers
    );

    return {
      dc_voltage: data[0] / 10,      // Scaled
      dc_current: data[1] / 100,
      dc_power: data[2],
      ac_power: data[5],
      ac_voltage: data[3] / 10,
      ac_current: data[4] / 100,
      ac_frequency: data[6] / 100,
      energy_today: data[7],
      energy_total: data[8],
      temperature: data[11],
      status: this.decodeStatus(data[12])
    };
  }

  decodeStatus(statusCode) {
    const statuses = {
      1: 'Off',
      2: 'Sleeping',
      3: 'Starting',
      4: 'MPPT',
      5: 'Throttled',
      6: 'Shutting Down',
      7: 'Fault',
      8: 'Standby'
    };
    return statuses[statusCode] || 'Unknown';
  }
}
```

#### 2.3.2 Performance Analytics

**Key Performance Indicators (KPIs):**

1. **Performance Ratio (PR)**
   ```
   PR = Actual Energy Output / Theoretical Output

   Theoretical Output = Irradiance × Array Area × Panel Efficiency
   ```
   - Target: PR ≥ 0.80
   - Indicates overall system health

2. **Capacity Factor**
   ```
   Capacity Factor = Actual Output / (Rated Capacity × Hours)
   ```
   - Typical: 15-25% depending on location
   - Higher is better

3. **Specific Yield**
   ```
   Specific Yield = Annual Energy Output / Installed Capacity (kWh/kWp)
   ```
   - Location-dependent
   - Useful for comparing systems

**Degradation Monitoring:**
- Track performance ratio over time
- Expected degradation: 0.5-0.7% per year
- Alert if degradation exceeds 1% annually

### 2.4 Solar Optimization

#### 2.4.1 Maximum Power Point Tracking (MPPT)

**Distributed MPPT:**
- Panel-level optimizers (e.g., SolarEdge, Tigo)
- Benefits: Shade tolerance, individual panel monitoring
- Cost: Higher upfront investment

**String-level MPPT:**
- Multiple MPPT inputs on inverter
- Group panels by orientation/shading
- Balance cost and performance

#### 2.4.2 Predictive Optimization

**Weather-based Forecasting:**

```python
import requests
from datetime import datetime, timedelta

class SolarForecast:
    def __init__(self, api_key, latitude, longitude):
        self.api_key = api_key
        self.lat = latitude
        self.lon = longitude

    def get_production_forecast(self, days=7):
        """Get solar production forecast using weather API"""
        url = f"https://api.weather.io/v1/forecast"
        params = {
            "lat": self.lat,
            "lon": self.lon,
            "days": days,
            "key": self.api_key
        }

        weather = requests.get(url, params=params).json()

        forecast = []
        for day in weather['forecast']:
            # Simplified solar production model
            irradiance = day['irradiance_wh_m2']  # Wh/m²
            cloud_cover = day['cloud_cover']       # 0-1

            # Adjust for cloud cover
            effective_irradiance = irradiance * (1 - cloud_cover * 0.75)

            # Calculate expected production
            # Assuming 100 kW system, 18% efficiency
            expected_kwh = (effective_irradiance * 100 * 0.18) / 1000

            forecast.append({
                "date": day['date'],
                "expected_production_kwh": expected_kwh,
                "confidence": 1 - (cloud_cover * 0.3)
            })

        return forecast
```

**Use Cases:**
- ESS pre-charging strategies
- Load shifting recommendations
- Grid export optimization
- Maintenance scheduling

---

## 3. Energy Storage System (ESS) Integration

### 3.1 System Architecture

```
┌──────────────────────────────────────────────────────┐
│              ENERGY STORAGE SYSTEM (ESS)             │
├──────────────────────────────────────────────────────┤
│                                                      │
│  🔋 Battery Bank                                    │
│  ├── Lithium-Ion Modules                           │
│  ├── Battery Management System (BMS)               │
│  ├── Thermal Management                            │
│  └── Safety Systems (HVAC, Fire Suppression)       │
│         │                                            │
│         ▼                                            │
│  ⚡ Power Conversion System (PCS)                   │
│  ├── Bi-directional Inverter                       │
│  ├── Charge Controller                             │
│  └── Grid-Forming Capability                       │
│         │                                            │
│         ▼                                            │
│  🧠 Energy Management System (EMS)                  │
│  ├── State of Charge (SoC) Management              │
│  ├── Charge/Discharge Optimization                 │
│  ├── Peak Shaving Logic                            │
│  ├── Load Shifting                                  │
│  └── Grid Services (if applicable)                  │
│         │                                            │
│         ▼                                            │
│  🔌 Integration Points                              │
│  ├── Solar PV                                       │
│  ├── Building Loads                                 │
│  ├── Grid Connection                                │
│  └── Backup Loads (critical circuits)               │
│                                                      │
└──────────────────────────────────────────────────────┘
```

### 3.2 ESS Control Strategies

#### 3.2.1 Self-Consumption Optimization

**Goal:** Maximize use of on-site solar production

```javascript
class SelfConsumptionController {
  constructor(ess, solar, loads) {
    this.ess = ess;
    this.solar = solar;
    this.loads = loads;
  }

  async optimize() {
    const solarPower = await this.solar.getCurrentPower();  // kW
    const loadPower = await this.loads.getCurrentPower();   // kW
    const essSoC = await this.ess.getStateOfCharge();       // %

    const surplus = solarPower - loadPower;

    if (surplus > 0) {
      // Surplus: charge battery
      if (essSoC < 95) {
        const chargeRate = Math.min(
          surplus,
          this.ess.maxChargeRate,
          this.ess.getRemainingCapacity() / 0.25  // Fill in 15 min
        );
        await this.ess.charge(chargeRate);
        return {
          action: 'charge',
          rate: chargeRate,
          gridExport: Math.max(0, surplus - chargeRate)
        };
      } else {
        // Battery full, export to grid
        return {
          action: 'export',
          rate: surplus
        };
      }
    } else {
      // Deficit: discharge battery or import from grid
      const deficit = Math.abs(surplus);

      if (essSoC > 20) {
        const dischargeRate = Math.min(
          deficit,
          this.ess.maxDischargeRate,
          this.ess.getAvailableEnergy() * 4  // Discharge over 15 min
        );
        await this.ess.discharge(dischargeRate);
        return {
          action: 'discharge',
          rate: dischargeRate,
          gridImport: Math.max(0, deficit - dischargeRate)
        };
      } else {
        // Battery low, import from grid
        return {
          action: 'import',
          rate: deficit
        };
      }
    }
  }
}
```

#### 3.2.2 Time-of-Use (TOU) Optimization

**Goal:** Minimize energy costs with TOU tariffs

```javascript
class TOUOptimizer {
  constructor(ess, tariff) {
    this.ess = ess;
    this.tariff = tariff;  // TOU rate schedule
  }

  getCurrentRate() {
    const hour = new Date().getHours();
    return this.tariff.getRateForHour(hour);
  }

  async optimize(solarPower, loadPower) {
    const currentRate = this.getCurrentRate();
    const nextHourRate = this.tariff.getRateForHour(
      (new Date().getHours() + 1) % 24
    );

    const surplus = solarPower - loadPower;

    if (currentRate === 'peak') {
      // Peak hours: discharge battery to avoid high rates
      if (surplus < 0 && this.ess.soc > 30) {
        await this.ess.discharge(Math.min(
          Math.abs(surplus),
          this.ess.maxDischargeRate
        ));
      }
    } else if (currentRate === 'off_peak') {
      // Off-peak: charge battery if solar insufficient
      if (surplus < 0 && this.ess.soc < 90) {
        // Only charge if next hours are peak
        if (nextHourRate === 'peak' || nextHourRate === 'mid_peak') {
          await this.ess.charge(Math.min(
            this.ess.maxChargeRate,
            this.ess.remainingCapacity / 0.5
          ));
        }
      }
    }

    // Always prioritize solar charging
    if (surplus > 0 && this.ess.soc < 95) {
      await this.ess.charge(Math.min(surplus, this.ess.maxChargeRate));
    }
  }
}
```

#### 3.2.3 Backup Power Management

**Islanding Mode:**

```javascript
class BackupController {
  constructor(ess, criticalLoads, transfer_switch) {
    this.ess = ess;
    this.criticalLoads = criticalLoads;
    this.transferSwitch = transfer_switch;
    this.islandMode = false;
  }

  async onGridOutage() {
    console.log('Grid outage detected. Switching to island mode.');

    // Open grid connection
    await this.transferSwitch.disconnect();

    // Switch to island mode
    this.islandMode = true;

    // Configure ESS for grid-forming mode
    await this.ess.setMode('grid_forming');

    // Shed non-critical loads
    await this.shedNonCriticalLoads();

    // Monitor battery level
    this.startBackupMonitoring();
  }

  async onGridRestored() {
    console.log('Grid restored. Returning to grid-tied mode.');

    // Synchronize with grid
    await this.ess.synchronize();

    // Reconnect to grid
    await this.transferSwitch.connect();

    // Resume normal operation
    this.islandMode = false;
    await this.ess.setMode('grid_tied');

    // Restore all loads
    await this.restoreAllLoads();
  }

  async shedNonCriticalLoads() {
    const totalLoad = await this.getCurrentLoad();
    const criticalLoad = await this.criticalLoads.getTotalPower();
    const availableEnergy = this.ess.getAvailableEnergy();

    // Calculate backup duration
    const backupHours = availableEnergy / criticalLoad;

    console.log(`Backup available for ${backupHours.toFixed(1)} hours`);

    // Disconnect non-critical circuits
    // Implementation depends on controllable breakers/relays
  }

  startBackupMonitoring() {
    this.backupMonitor = setInterval(async () => {
      const soc = await this.ess.getStateOfCharge();

      if (soc < 20) {
        console.warn('Battery critically low. Consider load shedding.');
        await this.emergencyLoadShedding();
      }

      if (soc < 10) {
        console.error('Battery nearly depleted. Initiating shutdown.');
        await this.gracefulShutdown();
      }
    }, 60000);  // Check every minute
  }
}
```

### 3.3 Battery Management System (BMS) Integration

#### 3.3.1 BMS Communication

**CAN Bus Protocol:**

```python
import can

class BMSInterface:
    def __init__(self, can_interface='can0'):
        self.bus = can.interface.Bus(channel=can_interface, bustype='socketcan')

    def read_battery_status(self):
        # Request battery status (CAN ID 0x100)
        msg = can.Message(
            arbitration_id=0x100,
            data=[0x01],  # Status request
            is_extended_id=False
        )
        self.bus.send(msg)

        # Receive response (CAN ID 0x101)
        response = self.bus.recv(timeout=1.0)

        if response:
            return self.parse_status(response.data)
        return None

    def parse_status(self, data):
        return {
            'voltage': int.from_bytes(data[0:2], 'big') / 10,  # V
            'current': int.from_bytes(data[2:4], 'big', signed=True) / 10,  # A
            'soc': data[4],  # %
            'temperature': data[5] - 40,  # °C
            'health': data[6],  # %
            'status': data[7]  # Bitmap
        }

    def set_charge_current(self, current_amps):
        # Set charge current limit (CAN ID 0x200)
        current_raw = int(current_amps * 10)
        msg = can.Message(
            arbitration_id=0x200,
            data=current_raw.to_bytes(2, 'big'),
            is_extended_id=False
        )
        self.bus.send(msg)
```

#### 3.3.2 Safety Monitoring

**Critical Parameters:**

| Parameter | Normal Range | Warning | Critical | Action |
|-----------|--------------|---------|----------|--------|
| Cell Voltage | 3.0-4.2 V | <2.8 or >4.3 V | <2.5 or >4.5 V | Shutdown |
| Temperature | 15-35°C | <5 or >40°C | <0 or >50°C | Shutdown |
| Current | Within rated | >1.2× rated | >1.5× rated | Limit/Stop |
| SoC | 20-90% | <10 or >95% | <5 or >98% | Limit charge/discharge |

**Safety Algorithms:**

```javascript
class BatterySafety {
  constructor(bms) {
    this.bms = bms;
    this.alerts = [];
  }

  async checkSafety() {
    const status = await this.bms.getStatus();
    const cells = await this.bms.getCellVoltages();

    // Check cell voltage balance
    const maxCell = Math.max(...cells);
    const minCell = Math.min(...cells);
    const imbalance = maxCell - minCell;

    if (imbalance > 0.05) {  // 50 mV
      this.alert('CELL_IMBALANCE', 'warning');
      if (imbalance > 0.1) {  // 100 mV
        this.alert('CELL_IMBALANCE', 'critical');
        await this.bms.initiateBalancing();
      }
    }

    // Check temperature
    if (status.temperature > 40) {
      this.alert('HIGH_TEMPERATURE', 'warning');
      await this.bms.reduceChargeCurrent(0.5);
    }
    if (status.temperature > 50) {
      this.alert('HIGH_TEMPERATURE', 'critical');
      await this.bms.emergencyShutdown();
    }

    // Check state of health (SoH)
    if (status.health < 80) {
      this.alert('DEGRADED_BATTERY', 'warning');
    }
    if (status.health < 70) {
      this.alert('DEGRADED_BATTERY', 'critical');
      // Recommend battery replacement
    }

    return this.alerts;
  }

  alert(type, severity) {
    this.alerts.push({
      type,
      severity,
      timestamp: new Date().toISOString()
    });

    // Send notification
    this.sendNotification(type, severity);
  }
}
```

---

## 4. Smart Grid Integration

### 4.1 Grid Interconnection

#### 4.1.1 IEEE 1547 Compliance

**Requirements:**
- **Voltage Regulation**: ±5% of nominal
- **Frequency Regulation**: ±0.5 Hz
- **Power Quality**: THD < 5%
- **Anti-Islanding**: Detect within 2 seconds
- **Reconnection**: 5-minute delay after restoration

#### 4.1.2 Net Metering Integration

```javascript
class NetMeterInterface {
  constructor(meterId, utilityAPI) {
    this.meterId = meterId;
    this.api = utilityAPI;
  }

  async getCurrentFlow() {
    const data = await this.api.getMeterData(this.meterId);

    return {
      import_kw: data.delivered_power,
      export_kw: data.received_power,
      net_kw: data.delivered_power - data.received_power,
      import_kwh_today: data.delivered_energy_today,
      export_kwh_today: data.received_energy_today,
      net_kwh_today: data.delivered_energy_today - data.received_energy_today
    };
  }

  async getMonthlyStatement() {
    const data = await this.api.getMonthlyData(
      this.meterId,
      new Date().getFullYear(),
      new Date().getMonth() + 1
    );

    return {
      import_kwh: data.total_delivered,
      export_kwh: data.total_received,
      net_kwh: data.total_delivered - data.total_received,
      charges: data.delivery_charges,
      credits: data.generation_credits,
      net_bill: data.delivery_charges - data.generation_credits
    };
  }
}
```

### 4.2 Demand Response Integration

#### 4.2.1 OpenADR Protocol

```python
from openadr import OpenADRClient

class DemandResponseClient:
    def __init__(self, ven_id, vtn_url):
        self.client = OpenADRClient(
            ven_name=ven_id,
            vtn_url=vtn_url
        )
        self.client.add_handler('on_event', self.handle_event)

    async def handle_event(self, event):
        """Handle demand response event from utility"""
        event_signals = event['event_signals']

        for signal in event_signals:
            signal_name = signal['signal_name']
            signal_type = signal['signal_type']
            intervals = signal['intervals']

            if signal_name == 'LOAD_DISPATCH':
                # Utility requesting load reduction
                for interval in intervals:
                    start_time = interval['dtstart']
                    duration = interval['duration']
                    value = interval['signal_payload']  # kW to shed

                    await self.schedule_load_reduction(
                        start_time,
                        duration,
                        value
                    )

            elif signal_name == 'PRICE':
                # Dynamic pricing signal
                for interval in intervals:
                    price = interval['signal_payload']
                    # Adjust ESS strategy based on price
                    await self.adjust_ess_strategy(price)

        # Respond to event
        await self.client.respond_to_event(
            event_id=event['event_id'],
            opt_in=True
        )

    async def schedule_load_reduction(self, start, duration, amount_kw):
        """Implement load reduction strategy"""
        # 1. Increase ESS discharge
        # 2. Reduce non-critical loads
        # 3. Adjust HVAC setpoints
        pass
```

### 4.3 Virtual Power Plant (VPP) Integration

```javascript
class VPPAggregator {
  constructor(buildingId, vppAPI) {
    this.buildingId = buildingId;
    this.api = vppAPI;
  }

  async registerBuilding(capacity_kw, ess_capacity_kwh) {
    return await this.api.register({
      building_id: this.buildingId,
      dispatchable_capacity_kw: capacity_kw,
      energy_storage_kwh: ess_capacity_kwh,
      capabilities: [
        'frequency_regulation',
        'peak_shaving',
        'load_shifting',
        'backup_power'
      ]
    });
  }

  async respondToDispatch(dispatch_signal) {
    const { action, power_kw, duration_minutes } = dispatch_signal;

    switch (action) {
      case 'charge':
        // Grid needs to absorb excess power
        await this.ess.charge(power_kw);
        break;

      case 'discharge':
        // Grid needs additional power
        await this.ess.discharge(power_kw);
        break;

      case 'reduce_load':
        // Reduce building consumption
        await this.reduceLoads(power_kw);
        break;
    }

    // Report compliance
    await this.api.reportDispatchResponse({
      building_id: this.buildingId,
      signal_id: dispatch_signal.id,
      actual_power_kw: await this.getMeterPower(),
      status: 'compliant'
    });
  }
}
```

---

## 5. Building Management System (BMS) Integration

### 5.1 BACnet Integration

#### 5.1.1 BACnet Object Mapping

**ZEB Energy Objects:**

```
ANALOG_INPUT
├── AI-1: Solar Production (kW)
├── AI-2: Building Consumption (kW)
├── AI-3: ESS State of Charge (%)
├── AI-4: ESS Power (kW, positive=charge)
├── AI-5: Grid Import (kW)
├── AI-6: Grid Export (kW)
├── AI-7: Net Power (kW)
└── AI-8: Daily Energy Balance (kWh)

ANALOG_VALUE
├── AV-1: ZEB Grade (1-5)
├── AV-2: Balance Ratio (%)
├── AV-3: Self-Sufficiency (%)
├── AV-4: Carbon Intensity (g CO2/kWh)
└── AV-5: Cost Savings Today ($)

BINARY_INPUT
├── BI-1: Grid Connected
├── BI-2: Solar Online
├── BI-3: ESS Online
└── BI-4: Carbon Neutral Status

MULTI_STATE_INPUT
├── MSI-1: Operating Mode (Grid-Tied/Island/Backup)
└── MSI-2: ESS Status (Idle/Charging/Discharging)
```

#### 5.1.2 BACnet Integration Example

```python
from bacpypes.primitivedata import Real
from bacpypes.object import AnalogInputObject
from bacpypes.local.device import LocalDeviceObject

class ZEBBACnetDevice:
    def __init__(self, device_id, device_name):
        self.device = LocalDeviceObject(
            objectName=device_name,
            objectIdentifier=device_id,
            maxApduLengthAccepted=1024,
            segmentationSupported='segmentedBoth',
            vendorIdentifier=999
        )

        self.setup_objects()

    def setup_objects(self):
        # Solar production
        solar_ai = AnalogInputObject(
            objectIdentifier=('analogInput', 1),
            objectName='Solar-Production-kW',
            presentValue=0.0,
            units='kilowatts'
        )
        self.device.add_object(solar_ai)

        # Building consumption
        consumption_ai = AnalogInputObject(
            objectIdentifier=('analogInput', 2),
            objectName='Building-Consumption-kW',
            presentValue=0.0,
            units='kilowatts'
        )
        self.device.add_object(consumption_ai)

        # ESS State of Charge
        soc_ai = AnalogInputObject(
            objectIdentifier=('analogInput', 3),
            objectName='ESS-SoC',
            presentValue=0.0,
            units='percent'
        )
        self.device.add_object(soc_ai)

    async def update_values(self, solar_kw, consumption_kw, ess_soc):
        # Update BACnet object values
        self.device['analogInput:1'].presentValue = Real(solar_kw)
        self.device['analogInput:2'].presentValue = Real(consumption_kw)
        self.device['analogInput:3'].presentValue = Real(ess_soc)
```

### 5.2 HVAC Optimization Integration

```javascript
class HVACOptimizer {
  constructor(bms, weatherAPI, occupancyAPI) {
    this.bms = bms;
    this.weather = weatherAPI;
    this.occupancy = occupancyAPI;
  }

  async optimizeForZEB() {
    const solarForecast = await this.weather.getSolarForecast();
    const occupancy = await this.occupancy.getPrediction();
    const currentLoad = await this.bms.getHVACLoad();

    // Pre-cool/pre-heat during high solar production
    if (solarForecast.next_3_hours > solarForecast.average) {
      // Surplus solar expected
      if (this.needsCooling()) {
        await this.bms.setSetpoint('cooling', 22);  // Cool to lower temp
      } else if (this.needsHeating()) {
        await this.bms.setSetpoint('heating', 24);  // Heat to higher temp
      }
    } else {
      // Limited solar, conserve energy
      await this.bms.setSetpoint('cooling', 24);
      await this.bms.setSetpoint('heating', 20);
    }

    // Adjust based on occupancy
    if (occupancy.current < 0.3) {
      // Low occupancy, reduce HVAC
      await this.bms.setVentilationRate(0.5);  // 50% of normal
    }
  }

  async demandLimitHVAC(limit_kw) {
    // Limit HVAC power during peak demand or low production
    const currentLoad = await this.bms.getHVACLoad();

    if (currentLoad > limit_kw) {
      // Reduce load by adjusting setpoints
      await this.bms.adjustSetpoint('cooling', +2);   // Warmer
      await this.bms.adjustSetpoint('heating', -2);   // Cooler
      await this.bms.setFanSpeed(0.8);  // 80% speed
    }
  }
}
```

---

## 6. Cloud Platform Integration

### 6.1 WIA Cloud API

#### 6.1.1 Authentication

```javascript
const axios = require('axios');

class WIACloudClient {
  constructor(apiKey) {
    this.apiKey = apiKey;
    this.baseURL = 'https://api.wia.official/v1';
  }

  async authenticate() {
    const response = await axios.post(`${this.baseURL}/auth/token`, {
      api_key: this.apiKey,
      grant_type: 'client_credentials'
    });

    this.accessToken = response.data.access_token;
    this.tokenExpiry = Date.now() + (response.data.expires_in * 1000);
  }

  async request(method, endpoint, data = null) {
    // Refresh token if expired
    if (Date.now() >= this.tokenExpiry) {
      await this.authenticate();
    }

    const config = {
      method,
      url: `${this.baseURL}${endpoint}`,
      headers: {
        'Authorization': `Bearer ${this.accessToken}`,
        'Content-Type': 'application/json'
      }
    };

    if (data) {
      config.data = data;
    }

    return await axios(config);
  }
}
```

#### 6.1.2 Data Upload

```javascript
class ZEBDataUploader {
  constructor(buildingId, wiaClient) {
    this.buildingId = buildingId;
    this.client = wiaClient;
  }

  async uploadDailyData(date, data) {
    return await this.client.request('POST',
      `/buildings/${this.buildingId}/data/daily`,
      {
        date: date,
        production: {
          solar_kwh: data.solar,
          wind_kwh: data.wind,
          geothermal_kwh: data.geothermal,
          total_kwh: data.totalProduction
        },
        consumption: {
          hvac_kwh: data.hvac,
          lighting_kwh: data.lighting,
          equipment_kwh: data.equipment,
          total_kwh: data.totalConsumption
        },
        grid: {
          import_kwh: data.gridImport,
          export_kwh: data.gridExport
        },
        carbon: {
          net_kg_co2: data.carbonNet
        }
      }
    );
  }

  async getCertificateStatus() {
    const response = await this.client.request('GET',
      `/buildings/${this.buildingId}/certificate`
    );

    return response.data;
  }
}
```

### 6.2 Data Analytics Integration

```python
from dataclasses import dataclass
from datetime import datetime, timedelta
import numpy as np

@dataclass
class PerformanceAnalytics:
    building_id: str

    def analyze_monthly_performance(self, data):
        """Analyze monthly ZEB performance"""
        production = np.array([d['production'] for d in data])
        consumption = np.array([d['consumption'] for d in data])

        # Calculate metrics
        balance_ratio = production.sum() / consumption.sum()
        avg_daily_balance = (production - consumption).mean()

        # Identify trends
        from scipy import stats
        production_trend = stats.linregress(
            range(len(production)), production
        ).slope

        # Detect anomalies (days with unusually low production)
        production_mean = production.mean()
        production_std = production.std()
        anomalies = np.where(
            production < (production_mean - 2 * production_std)
        )[0]

        return {
            'balance_ratio': balance_ratio,
            'avg_daily_surplus': avg_daily_balance,
            'production_trend': production_trend,
            'anomaly_days': anomalies.tolist(),
            'performance_score': self.calculate_score(balance_ratio)
        }

    def calculate_score(self, balance_ratio):
        """ZEB performance score (0-100)"""
        if balance_ratio >= 1.2:
            return 100
        elif balance_ratio >= 1.0:
            return 90 + ((balance_ratio - 1.0) * 50)
        elif balance_ratio >= 0.8:
            return 70 + ((balance_ratio - 0.8) * 100)
        elif balance_ratio >= 0.6:
            return 50 + ((balance_ratio - 0.6) * 100)
        else:
            return balance_ratio * 83.3  # 0-50 score for <0.6
```

---

## 7. External Service Integration

### 7.1 Weather Service Integration

```javascript
class WeatherService {
  constructor(apiKey, location) {
    this.apiKey = apiKey;
    this.lat = location.latitude;
    this.lon = location.longitude;
  }

  async getCurrentConditions() {
    const response = await fetch(
      `https://api.weather.com/v1/current?lat=${this.lat}&lon=${this.lon}&key=${this.apiKey}`
    );
    const data = await response.json();

    return {
      temperature_c: data.temperature,
      cloud_cover: data.cloud_cover,
      wind_speed_ms: data.wind_speed,
      irradiance_w_m2: data.solar_irradiance,
      timestamp: new Date(data.timestamp)
    };
  }

  async getForecast(hours = 24) {
    const response = await fetch(
      `https://api.weather.com/v1/forecast?lat=${this.lat}&lon=${this.lon}&hours=${hours}&key=${this.apiKey}`
    );
    return await response.json();
  }
}
```

### 7.2 Carbon Accounting Service

```python
class CarbonAccountingService:
    def __init__(self, api_key, grid_region):
        self.api_key = api_key
        self.grid_region = grid_region

    async def get_emission_factor(self, timestamp):
        """Get real-time marginal emission factor"""
        response = await requests.get(
            f"https://api.carbonintensity.org/v1/region/{self.grid_region}",
            params={'timestamp': timestamp.isoformat()}
        )
        data = response.json()

        return {
            'emission_factor_g_co2_kwh': data['intensity'],
            'fuel_mix': data['fuel_mix'],
            'renewable_percent': data['renewable_percent']
        }

    def calculate_avoided_emissions(self, production_kwh, emission_factor):
        """Calculate CO2 avoided by renewable generation"""
        return production_kwh * emission_factor / 1000  # kg CO2

    def calculate_net_emissions(self, consumption_kwh, production_kwh,
                                grid_import_kwh, grid_export_kwh,
                                emission_factor):
        """Calculate net carbon emissions"""
        # Emissions from grid import
        import_emissions = grid_import_kwh * emission_factor / 1000

        # Credits from grid export (avoided emissions)
        export_credits = grid_export_kwh * emission_factor / 1000

        # Net emissions
        net_emissions = import_emissions - export_credits

        return {
            'import_emissions_kg_co2': import_emissions,
            'export_credits_kg_co2': export_credits,
            'net_emissions_kg_co2': net_emissions,
            'carbon_neutral': abs(net_emissions) < 1.0,  # Within 1 kg
            'carbon_negative': net_emissions < -1.0
        }
```

---

## 8. Integration Testing & Validation

### 8.1 System Integration Test Plan

**Test Scenarios:**

1. **Normal Operation**
   - Solar production during day
   - ESS charging from surplus
   - Grid export when ESS full
   - ESS discharge at night
   - Grid import when ESS low

2. **Peak Production**
   - Maximum solar output
   - ESS reaches full capacity
   - Grid export at maximum rate
   - System stability under high production

3. **Peak Consumption**
   - Maximum building load
   - ESS discharge at maximum rate
   - Grid import if needed
   - Demand limiting if configured

4. **Grid Outage**
   - Anti-islanding detection
   - Transfer to island mode
   - ESS backup power
   - Critical load prioritization
   - Grid reconnection

5. **ESS Edge Cases**
   - Very low SoC (< 10%)
   - Very high SoC (> 95%)
   - Temperature extremes
   - High charge/discharge rates
   - Cell balancing

6. **Communication Failures**
   - Modbus timeout
   - BACnet device offline
   - Cloud API unavailable
   - Fallback modes

### 8.2 Performance Benchmarks

| System | Response Time | Uptime | Accuracy |
|--------|---------------|--------|----------|
| **Solar Monitoring** | < 5 seconds | 99.5% | ±2% |
| **ESS Control** | < 1 second | 99.9% | ±1% |
| **Grid Metering** | < 1 second | 99.99% | ±0.5% |
| **BMS Integration** | < 10 seconds | 99% | ±5% |
| **Cloud API** | < 30 seconds | 99.5% | - |

---

**弘益人間 · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
WIA-CITY-005 v1.0 - Integration Specification
https://wia.official/standards/CITY/005/integration
