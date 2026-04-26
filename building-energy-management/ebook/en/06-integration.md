# Chapter 6: System Integration (Phase 4)

## 6.1 Integration Architecture Overview

### 6.1.1 External System Landscape

Phase 4 of WIA-BEMS extends building energy management beyond the building boundary, connecting to external systems that enable participation in grid services, maximize renewable energy utilization, and automate compliance reporting.

**Integration Ecosystem:**

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      EXTERNAL INTEGRATION LAYER                          │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌─────────────┐ ┌───────────────┐ ┌──────────────┐ ┌──────────────────┐│
│  │  SMART GRID │ │   RENEWABLE   │ │     BAS      │ │   COMPLIANCE     ││
│  │  SERVICES   │ │    ENERGY     │ │  INTEGRATION │ │   PLATFORMS      ││
│  │             │ │               │ │              │ │                  ││
│  │ • OpenADR   │ │ • Solar PV    │ │ • BACnet     │ │ • Energy Star    ││
│  │ • IEEE 2030 │ │ • Wind        │ │ • Modbus     │ │ • GRESB          ││
│  │ • DR Signals│ │ • Battery     │ │ • OPC UA     │ │ • LEED           ││
│  │ • Pricing   │ │ • Virtual PP  │ │ • Haystack   │ │ • CDP            ││
│  └──────┬──────┘ └───────┬───────┘ └──────┬───────┘ └────────┬─────────┘│
│         │                │                │                  │          │
└─────────┼────────────────┼────────────────┼──────────────────┼──────────┘
          │                │                │                  │
          └────────────────┴────────────────┴──────────────────┘
                                    │
                          ┌─────────┴─────────┐
                          │    WIA-BEMS       │
                          │    Platform       │
                          └───────────────────┘
```

### 6.1.2 Integration Patterns

**Common Integration Patterns:**

| Pattern | Use Case | Protocol | Direction |
|---------|----------|----------|-----------|
| Event-Driven | DR signals, Alarms | WebSocket, MQTT | Inbound |
| Polling | Meter data, Status | REST API | Outbound |
| Streaming | Real-time telemetry | WebSocket, MQTT | Bidirectional |
| Batch | Historical export | REST, SFTP | Outbound |
| Command | Control actions | REST, BACnet | Outbound |

## 6.2 Smart Grid Integration

### 6.2.1 OpenADR 2.0b Implementation

**OpenADR Architecture:**

```typescript
interface OpenADRIntegration {
  roles: {
    VTN: "Virtual Top Node - Utility/Aggregator";
    VEN: "Virtual End Node - Building BEMS";
  };

  eventTypes: {
    SIMPLE: "Signal intensity level (0-3)";
    ELECTRICITY_PRICE: "Real-time or day-ahead prices";
    LOAD_DISPATCH: "Specific load level commands";
    LOAD_CONTROL: "Direct load control";
  };

  signalTypes: {
    level: "0 = normal, 1 = moderate, 2 = high, 3 = special";
    price: "$/kWh price signal";
    priceMultiplier: "Multiplier on base rate";
    priceRelative: "Offset from normal price";
    setpoint: "Absolute demand target";
  };
}
```

**OpenADR VEN Implementation:**

```python
from openleadr import OpenADRClient
import asyncio

class WIABEMSOpenADRClient:
    """
    OpenADR 2.0b Virtual End Node for WIA-BEMS
    """

    def __init__(self, config: OpenADRConfig):
        self.config = config
        self.client = OpenADRClient(
            ven_name=config.ven_name,
            vtn_url=config.vtn_url,
            cert=config.certificate_path,
            key=config.private_key_path
        )

        # Register handlers
        self.client.add_handler('on_event', self.handle_event)
        self.client.add_handler('on_update_event', self.handle_event_update)

        # BEMS interface
        self.bems = WIABEMSController(config.bems_url)

    async def start(self):
        """Start OpenADR client"""
        await self.client.run()

    async def handle_event(self, event: dict) -> str:
        """
        Handle incoming DR event from VTN
        """
        event_id = event['event_id']
        event_type = event['event_descriptor']['event_type']
        signals = event['event_signals']

        # Parse event timing
        start_time = event['active_period']['dtstart']
        duration = event['active_period']['duration']

        # Process based on event type
        if event_type == 'SIMPLE':
            response = await self._handle_simple_event(signals, start_time, duration)

        elif event_type == 'ELECTRICITY_PRICE':
            response = await self._handle_price_event(signals, start_time, duration)

        elif event_type == 'LOAD_DISPATCH':
            response = await self._handle_dispatch_event(signals, start_time, duration)

        # Return opt-in status
        return 'optIn' if response.accepted else 'optOut'

    async def _handle_simple_event(
        self,
        signals: list,
        start_time: datetime,
        duration: timedelta
    ) -> DRResponse:
        """
        Handle simple signal event (0-3 levels)
        """
        signal_level = signals[0]['signal_payload']

        # Map signal level to control actions
        actions = {
            0: [],  # Normal operation
            1: [  # Moderate
                {'type': 'SETPOINT_OFFSET', 'params': {'cooling': +1.0}},
                {'type': 'LIGHTING_DIM', 'params': {'percent': 10}},
            ],
            2: [  # High
                {'type': 'SETPOINT_OFFSET', 'params': {'cooling': +2.0}},
                {'type': 'LIGHTING_DIM', 'params': {'percent': 25}},
                {'type': 'EQUIPMENT_SHED', 'params': {'priority': 'low'}},
            ],
            3: [  # Special/Emergency
                {'type': 'SETPOINT_OFFSET', 'params': {'cooling': +3.0}},
                {'type': 'LIGHTING_DIM', 'params': {'percent': 40}},
                {'type': 'EQUIPMENT_SHED', 'params': {'priority': 'medium'}},
                {'type': 'EV_CHARGING_PAUSE', 'params': {}},
            ],
        }

        # Schedule actions with BEMS
        scheduled = await self.bems.schedule_dr_actions(
            actions=actions.get(signal_level, []),
            start=start_time,
            duration=duration
        )

        # Estimate load reduction
        estimated_kw = await self.bems.estimate_reduction(actions.get(signal_level, []))

        return DRResponse(
            accepted=scheduled,
            estimated_reduction_kw=estimated_kw
        )

    async def _handle_price_event(
        self,
        signals: list,
        start_time: datetime,
        duration: timedelta
    ) -> DRResponse:
        """
        Handle price signal event
        """
        price_schedule = []

        for interval in signals:
            price_schedule.append({
                'start': interval['start'],
                'duration': interval['duration'],
                'price': interval['signal_payload']
            })

        # Send to BEMS for price-responsive control
        await self.bems.update_price_schedule(price_schedule)

        return DRResponse(accepted=True, estimated_reduction_kw=0)

    async def report_telemetry(self):
        """
        Report actual load data back to VTN
        """
        # Get current power from BEMS
        power_data = await self.bems.get_current_power()

        # Report to VTN
        await self.client.report_telemetry(
            report_name='TELEMETRY_USAGE',
            data={
                'power_real': power_data.kw,
                'energy': power_data.cumulative_kwh,
                'timestamp': datetime.utcnow()
            }
        )
```

### 6.2.2 IEEE 2030.5 (SEP 2.0)

**Smart Energy Profile Integration:**

```typescript
interface IEEE2030_5Integration {
  resourceTypes: {
    DeviceCapability: "Device features and limits";
    EndDevice: "Physical device representation";
    MeterReading: "Energy usage data";
    DemandResponseProgram: "DR program enrollment";
    FlowReservationRequest: "EV charging reservations";
    DERProgram: "Distributed energy resource programs";
  };

  securityModel: {
    transport: "TLS 1.2+ with mutual authentication";
    certificates: "X.509 certificates from IEEE PKI";
    authorization: "Role-based access per resource";
  };
}

// IEEE 2030.5 Client Implementation
class SEP2Client {
  private baseUrl: string;
  private certificates: TLSCertificates;

  async registerDevice(device: EndDevice): Promise<string> {
    const response = await this.post('/edev', device);
    return response.headers['location'];
  }

  async getDemandResponsePrograms(): Promise<DRProgram[]> {
    const response = await this.get('/dr');
    return response.DemandResponseProgramList.DemandResponseProgram;
  }

  async enrollInProgram(programId: string): Promise<void> {
    await this.post(`/dr/${programId}/drec`, {
      status: 1,  // Enrolled
      mRID: this.deviceId
    });
  }

  async reportMeterReading(reading: MeterReading): Promise<void> {
    await this.post('/mup', reading);
  }

  async getFlowReservation(evseId: string): Promise<FlowReservation> {
    return await this.get(`/edev/${evseId}/frq`);
  }
}
```

### 6.2.3 Dynamic Pricing Integration

**Real-Time Pricing Handler:**

```python
class DynamicPricingIntegration:
    """
    Handle real-time and day-ahead electricity pricing
    """

    def __init__(self, config: PricingConfig):
        self.config = config
        self.price_schedule: Dict[datetime, float] = {}
        self.price_threshold = config.response_threshold

    async def fetch_day_ahead_prices(self) -> List[PriceInterval]:
        """
        Fetch next-day prices from ISO/Utility
        """
        # Example: CAISO LMP prices
        url = f"{self.config.iso_url}/prices/dam"
        params = {
            'node': self.config.pricing_node,
            'date': (datetime.now() + timedelta(days=1)).strftime('%Y-%m-%d')
        }

        response = await self.http_client.get(url, params=params)
        prices = response.json()

        # Store in schedule
        for interval in prices['intervals']:
            self.price_schedule[interval['timestamp']] = interval['lmp']

        return prices['intervals']

    async def get_real_time_price(self) -> float:
        """
        Get current real-time LMP
        """
        url = f"{self.config.iso_url}/prices/rtm"
        params = {'node': self.config.pricing_node}

        response = await self.http_client.get(url, params=params)
        return response.json()['price']

    def should_respond(self, current_price: float) -> bool:
        """
        Determine if price warrants response
        """
        avg_price = sum(self.price_schedule.values()) / len(self.price_schedule)
        return current_price > avg_price * self.price_threshold

    async def generate_response_schedule(self) -> List[DRAction]:
        """
        Create optimal response schedule based on price forecast
        """
        actions = []

        for timestamp, price in sorted(self.price_schedule.items()):
            if self.should_respond(price):
                # High price period - schedule load reduction
                actions.append(DRAction(
                    start=timestamp,
                    type='LOAD_SHED',
                    magnitude=self._calculate_shed_magnitude(price)
                ))
            elif price < self.config.pre_cool_threshold:
                # Low price - pre-condition building
                actions.append(DRAction(
                    start=timestamp,
                    type='PRE_COOL',
                    magnitude=self.config.pre_cool_offset
                ))

        return self._optimize_schedule(actions)
```

## 6.3 Renewable Energy Integration

### 6.3.1 Solar PV Integration

**PV System Integration:**

```python
class SolarPVIntegration:
    """
    Integration with on-site solar PV systems
    """

    def __init__(self, config: PVConfig):
        self.config = config
        self.inverters = [InverterClient(inv) for inv in config.inverters]
        self.forecast_model = SolarForecastModel()

    async def get_current_generation(self) -> PVGeneration:
        """
        Get real-time generation from all inverters
        """
        total_power = 0
        inverter_data = []

        for inverter in self.inverters:
            data = await inverter.get_status()
            total_power += data.ac_power_kw
            inverter_data.append(data)

        return PVGeneration(
            timestamp=datetime.now(),
            total_power_kw=total_power,
            total_energy_today_kwh=sum(i.energy_today_kwh for i in inverter_data),
            inverters=inverter_data
        )

    async def get_generation_forecast(
        self,
        horizon_hours: int = 48
    ) -> List[ForecastInterval]:
        """
        Get generation forecast based on weather
        """
        # Get weather forecast
        weather = await self.weather_client.get_forecast(
            lat=self.config.location.lat,
            lon=self.config.location.lon,
            hours=horizon_hours
        )

        # Generate power forecast
        forecast = []
        for hour in weather.hourly:
            # Clear sky model
            clear_sky = self._clear_sky_power(hour.timestamp)

            # Apply cloud cover factor
            cloud_factor = 1 - (hour.cloud_cover / 100) * 0.75

            # Temperature derating
            temp_factor = 1 - 0.004 * max(0, hour.temperature - 25)

            # Predicted power
            power_kw = clear_sky * cloud_factor * temp_factor

            forecast.append(ForecastInterval(
                timestamp=hour.timestamp,
                power_kw=power_kw,
                confidence=self._calculate_confidence(hour)
            ))

        return forecast

    def _clear_sky_power(self, timestamp: datetime) -> float:
        """
        Calculate theoretical clear sky power output
        """
        from pvlib import location, irradiance, pvsystem

        site = location.Location(
            self.config.location.lat,
            self.config.location.lon,
            tz=self.config.location.timezone
        )

        solar_position = site.get_solarposition(timestamp)

        # Get POA irradiance
        poa = irradiance.get_total_irradiance(
            surface_tilt=self.config.array_tilt,
            surface_azimuth=self.config.array_azimuth,
            dni=1000,  # Clear sky assumption
            ghi=1000,
            dhi=100,
            solar_zenith=solar_position['zenith'],
            solar_azimuth=solar_position['azimuth']
        )

        # Calculate DC power
        dc_power = poa['poa_global'] * self.config.capacity_kw / 1000

        return dc_power * self.config.system_efficiency
```

### 6.3.2 Battery Storage Integration

**Battery Energy Storage System:**

```typescript
interface BatteryStorageIntegration {
  operatingModes: {
    SELF_CONSUMPTION: "Maximize solar self-consumption";
    PEAK_SHAVING: "Reduce demand peaks";
    TIME_OF_USE: "Arbitrage based on TOU rates";
    BACKUP: "Reserve for outages";
    GRID_SERVICES: "Provide frequency regulation, etc.";
  };

  constraints: {
    maxChargeRate: number;  // kW
    maxDischargeRate: number;  // kW
    minStateOfCharge: number;  // % reserved for backup
    maxStateOfCharge: number;  // % to preserve battery life
    roundTripEfficiency: number;
  };
}

class BatteryController {
  private config: BatteryConfig;
  private currentSoC: number;
  private currentMode: OperatingMode;

  async optimizeDispatch(
    loadForecast: number[],
    solarForecast: number[],
    priceForecast: number[],
    horizon: number
  ): Promise<BatterySchedule> {
    // Linear programming optimization
    const schedule = await this.solver.solve({
      objective: 'MINIMIZE_COST',

      variables: {
        charge: Array(horizon).fill({ min: 0, max: this.config.maxChargeRate }),
        discharge: Array(horizon).fill({ min: 0, max: this.config.maxDischargeRate }),
      },

      constraints: [
        // Energy balance
        ...this.energyBalanceConstraints(loadForecast, solarForecast),

        // SoC limits
        ...this.socConstraints(),

        // Rate limits
        ...this.rateConstraints(),

        // Grid export limits (if applicable)
        ...this.exportConstraints(),
      ],

      coefficients: priceForecast,  // Minimize cost
    });

    return schedule;
  }

  async executeSchedule(schedule: BatterySchedule): Promise<void> {
    const currentHour = Math.floor(Date.now() / 3600000) % 24;
    const action = schedule.hourly[currentHour];

    if (action.charge > 0) {
      await this.setChargePower(action.charge);
    } else if (action.discharge > 0) {
      await this.setDischargePower(action.discharge);
    } else {
      await this.setIdle();
    }
  }

  async handleGridEvent(event: GridEvent): Promise<void> {
    switch (event.type) {
      case 'FREQUENCY_DEVIATION':
        // Fast response for frequency regulation
        if (event.frequency < 59.95) {
          await this.setDischargePower(this.config.maxDischargeRate);
        } else if (event.frequency > 60.05) {
          await this.setChargePower(this.config.maxChargeRate);
        }
        break;

      case 'DEMAND_PEAK':
        // Discharge to shave peak
        await this.setDischargePower(event.targetReduction);
        break;

      case 'OUTAGE':
        // Switch to backup mode
        await this.setMode('BACKUP');
        break;
    }
  }
}
```

### 6.3.3 Virtual Power Plant Participation

**VPP Integration:**

```python
class VirtualPowerPlantClient:
    """
    Integration with Virtual Power Plant aggregators
    """

    def __init__(self, config: VPPConfig):
        self.config = config
        self.connection = VPPConnection(config.aggregator_url)
        self.registered_resources: List[DERResource] = []

    async def register_resources(self):
        """
        Register building's DER resources with VPP
        """
        resources = []

        # Register solar PV
        if self.config.has_solar:
            resources.append(DERResource(
                type='SOLAR_PV',
                capacity_kw=self.config.solar_capacity,
                controllable=False,  # Non-dispatchable
                forecast_available=True
            ))

        # Register battery
        if self.config.has_battery:
            resources.append(DERResource(
                type='BATTERY',
                capacity_kw=self.config.battery_power,
                capacity_kwh=self.config.battery_energy,
                controllable=True,
                response_time_seconds=1
            ))

        # Register flexible loads
        resources.append(DERResource(
            type='FLEXIBLE_LOAD',
            capacity_kw=self.config.flexible_load_capacity,
            controllable=True,
            response_time_seconds=60,
            constraints={
                'max_duration_hours': 4,
                'max_events_per_day': 2,
                'comfort_limits': True
            }
        ))

        # Register with aggregator
        for resource in resources:
            registration = await self.connection.register(resource)
            resource.registration_id = registration.id
            self.registered_resources.append(resource)

    async def handle_dispatch(self, dispatch: VPPDispatch) -> DispatchResponse:
        """
        Handle dispatch signal from VPP aggregator
        """
        responses = []

        for command in dispatch.commands:
            resource = self._find_resource(command.resource_id)

            if resource.type == 'BATTERY':
                result = await self._dispatch_battery(command)
            elif resource.type == 'FLEXIBLE_LOAD':
                result = await self._dispatch_loads(command)
            else:
                result = DispatchResult(success=False, reason='Resource not dispatchable')

            responses.append(result)

        return DispatchResponse(
            dispatch_id=dispatch.id,
            results=responses,
            total_response_kw=sum(r.actual_kw for r in responses if r.success)
        )

    async def provide_availability(self) -> AvailabilityBid:
        """
        Submit availability bid to VPP market
        """
        # Calculate available capacity for next trading period
        solar_forecast = await self.solar.get_forecast(hours=24)
        battery_soc = await self.battery.get_soc()
        current_load = await self.bems.get_current_load()

        # Available flexibility
        up_flexibility = []
        down_flexibility = []

        for hour in range(24):
            # Upward flexibility (can increase consumption or reduce generation)
            up_kw = (
                self.config.flexible_load_capacity +  # Can shed
                min(battery_soc * self.config.battery_power, self.config.battery_power)  # Battery discharge
            )

            # Downward flexibility (can reduce consumption or increase generation)
            down_kw = (
                solar_forecast[hour].power_kw +  # Solar available
                (1 - battery_soc) * self.config.battery_power  # Battery charge headroom
            )

            up_flexibility.append(up_kw)
            down_flexibility.append(down_kw)

        return AvailabilityBid(
            resource_id=self.config.resource_id,
            period_start=datetime.now().replace(minute=0, second=0),
            up_mw=[kw/1000 for kw in up_flexibility],
            down_mw=[kw/1000 for kw in down_flexibility],
            price_up=self.config.bid_price_up,
            price_down=self.config.bid_price_down
        )
```

## 6.4 Building Automation System Integration

### 6.4.1 BACnet Integration

**BACnet Gateway:**

```python
import BAC0

class BACnetGateway:
    """
    BACnet/IP gateway for legacy BAS integration
    """

    def __init__(self, config: BACnetConfig):
        self.network = BAC0.connect(ip=config.local_ip, port=config.port)
        self.devices: Dict[int, BACnetDevice] = {}

    async def discover_devices(self):
        """
        Discover BACnet devices on network
        """
        devices = self.network.discover()

        for device_id in devices:
            device = self.network[device_id]
            self.devices[device_id] = BACnetDevice(
                id=device_id,
                name=device.properties.objectName,
                vendor=device.properties.vendorIdentifier,
                points=await self._discover_points(device)
            )

    async def _discover_points(self, device) -> List[BACnetPoint]:
        """
        Discover points on a device
        """
        points = []

        for obj in device.objects:
            point = BACnetPoint(
                object_type=obj.type,
                instance=obj.instance,
                name=obj.properties.objectName,
                units=obj.properties.get('units'),
                present_value=obj.properties.presentValue
            )
            points.append(point)

        return points

    def read_point(self, device_id: int, object_type: str,
                   instance: int) -> Any:
        """
        Read single point value
        """
        address = f"{device_id}/{object_type}/{instance}"
        return self.network.read(address, 'presentValue')

    def write_point(self, device_id: int, object_type: str,
                    instance: int, value: Any, priority: int = 8) -> bool:
        """
        Write value to BACnet point
        """
        address = f"{device_id}/{object_type}/{instance}"

        try:
            self.network.write(
                address,
                'presentValue',
                value,
                priority=priority
            )
            return True
        except Exception as e:
            logger.error(f"BACnet write failed: {e}")
            return False

    async def subscribe_cov(
        self,
        device_id: int,
        object_type: str,
        instance: int,
        callback: Callable
    ):
        """
        Subscribe to Change of Value notifications
        """
        address = f"{device_id}/{object_type}/{instance}"

        self.network.subscribeCOV(
            address,
            confirmed=True,
            lifetime=3600,
            callback=callback
        )
```

### 6.4.2 Modbus Integration

**Modbus TCP Gateway:**

```typescript
import { ModbusTCPClient } from 'jsmodbus';

class ModbusGateway {
  private clients: Map<string, ModbusTCPClient> = new Map();
  private pointMap: ModbusPointMap;

  async connect(devices: ModbusDevice[]): Promise<void> {
    for (const device of devices) {
      const client = new ModbusTCPClient(device.host, device.port);
      await client.connect();
      this.clients.set(device.id, client);
    }
  }

  async readRegisters(
    deviceId: string,
    startAddress: number,
    count: number
  ): Promise<number[]> {
    const client = this.clients.get(deviceId);
    if (!client) throw new Error(`Device ${deviceId} not found`);

    const response = await client.readHoldingRegisters(startAddress, count);
    return response.body.values;
  }

  async readPoint(point: ModbusPoint): Promise<number> {
    const registers = await this.readRegisters(
      point.deviceId,
      point.address,
      point.registerCount
    );

    // Apply scaling and data type conversion
    let value = this.combineRegisters(registers, point.dataType);
    value = value * point.scale + point.offset;

    return value;
  }

  async writePoint(point: ModbusPoint, value: number): Promise<void> {
    const client = this.clients.get(point.deviceId);
    if (!client) throw new Error(`Device ${point.deviceId} not found`);

    // Apply inverse scaling
    const rawValue = (value - point.offset) / point.scale;

    // Convert to register values
    const registers = this.splitToRegisters(rawValue, point.dataType);

    await client.writeMultipleRegisters(point.address, registers);
  }

  private combineRegisters(registers: number[], dataType: string): number {
    switch (dataType) {
      case 'INT16':
        return registers[0] > 32767 ? registers[0] - 65536 : registers[0];

      case 'UINT16':
        return registers[0];

      case 'INT32':
        const int32 = (registers[0] << 16) | registers[1];
        return int32 > 2147483647 ? int32 - 4294967296 : int32;

      case 'FLOAT32':
        const buffer = Buffer.alloc(4);
        buffer.writeUInt16BE(registers[0], 0);
        buffer.writeUInt16BE(registers[1], 2);
        return buffer.readFloatBE(0);

      default:
        return registers[0];
    }
  }
}
```

## 6.5 Compliance and Reporting Integration

### 6.5.1 Energy Star Portfolio Manager

**Portfolio Manager Integration:**

```python
class EnergyStarPortfolioManager:
    """
    Integration with EPA Energy Star Portfolio Manager
    """

    def __init__(self, config: PortfolioManagerConfig):
        self.client = PortfolioManagerClient(
            username=config.username,
            password=config.password,
            base_url='https://portfoliomanager.energystar.gov/wsv2'
        )

    async def sync_property(self, building: Building) -> PropertySync:
        """
        Sync building data with Portfolio Manager
        """
        # Get or create property
        pm_property = await self._get_or_create_property(building)

        # Update property use details
        await self._update_use_details(pm_property.id, building)

        # Submit meter data
        await self._submit_meter_data(pm_property.id, building)

        # Get updated metrics
        metrics = await self.client.get_metrics(pm_property.id)

        return PropertySync(
            property_id=pm_property.id,
            energy_star_score=metrics.score,
            source_eui=metrics.source_eui,
            site_eui=metrics.site_eui,
            last_sync=datetime.now()
        )

    async def _submit_meter_data(
        self,
        property_id: int,
        building: Building
    ):
        """
        Submit monthly meter data
        """
        # Get meters for property
        meters = await self.client.get_meters(property_id)

        for meter in meters:
            # Get consumption data from BEMS
            consumption = await self.bems.get_monthly_consumption(
                building_id=building.id,
                meter_type=meter.type,
                start_date=meter.last_entry_date + timedelta(days=1),
                end_date=datetime.now()
            )

            # Submit to Portfolio Manager
            for month_data in consumption:
                await self.client.submit_meter_consumption(
                    meter_id=meter.id,
                    start_date=month_data.start_date,
                    end_date=month_data.end_date,
                    usage=month_data.consumption,
                    cost=month_data.cost
                )

    async def get_score(self, building_id: str) -> Optional[int]:
        """
        Get current Energy Star score
        """
        property_id = self.property_mapping.get(building_id)
        if not property_id:
            return None

        metrics = await self.client.get_metrics(property_id)
        return metrics.score if metrics.eligible_for_score else None
```

### 6.5.2 GRESB Reporting

**GRESB Data Automation:**

```python
class GRESBReporter:
    """
    Automated GRESB assessment data preparation
    """

    def __init__(self, portfolio: Portfolio):
        self.portfolio = portfolio
        self.reporting_year = datetime.now().year - 1

    async def prepare_assessment_data(self) -> GRESBAssessment:
        """
        Prepare data for GRESB Real Estate Assessment
        """
        assessment = GRESBAssessment(year=self.reporting_year)

        # Energy data
        assessment.energy = await self._prepare_energy_data()

        # GHG emissions
        assessment.emissions = await self._prepare_emissions_data()

        # Water (if tracked)
        assessment.water = await self._prepare_water_data()

        # Waste (if tracked)
        assessment.waste = await self._prepare_waste_data()

        # Certifications
        assessment.certifications = await self._get_certifications()

        return assessment

    async def _prepare_energy_data(self) -> GRESBEnergy:
        """
        Prepare energy performance data
        """
        energy_data = GRESBEnergy()

        for building in self.portfolio.buildings:
            # Get annual consumption by fuel type
            consumption = await self.bems.get_annual_consumption(
                building_id=building.id,
                year=self.reporting_year
            )

            # Convert to GRESB format
            energy_data.add_building(
                building_id=building.external_id,
                floor_area=building.gla_sqm,
                electricity_kwh=consumption.electricity_kwh,
                natural_gas_kwh=consumption.gas_kwh,
                district_heating_kwh=consumption.district_heat_kwh,
                district_cooling_kwh=consumption.district_cool_kwh,
                fuel_oil_kwh=consumption.fuel_oil_kwh,
                renewable_on_site_kwh=consumption.solar_kwh
            )

        # Calculate like-for-like coverage
        energy_data.lfl_coverage = self._calculate_lfl_coverage()

        # Calculate intensity
        energy_data.intensity_kwh_sqm = (
            energy_data.total_kwh / energy_data.total_sqm
        )

        return energy_data

    async def _prepare_emissions_data(self) -> GRESBEmissions:
        """
        Calculate Scope 1, 2, and 3 emissions
        """
        emissions = GRESBEmissions()

        for building in self.portfolio.buildings:
            consumption = await self.bems.get_annual_consumption(
                building_id=building.id,
                year=self.reporting_year
            )

            # Scope 1: Direct emissions (on-site combustion)
            scope1 = (
                consumption.gas_kwh * 0.181 +  # kg CO2/kWh natural gas
                consumption.fuel_oil_kwh * 0.267  # kg CO2/kWh fuel oil
            )

            # Scope 2: Indirect emissions (electricity)
            grid_factor = await self._get_grid_emission_factor(building.location)
            scope2 = consumption.electricity_kwh * grid_factor

            # Scope 3: Other indirect (tenant energy if applicable)
            scope3 = consumption.tenant_electricity_kwh * grid_factor

            emissions.add_building(
                building_id=building.external_id,
                scope1_kg=scope1,
                scope2_location_kg=scope2,
                scope2_market_kg=scope2 * 0.8,  # If purchasing RECs
                scope3_kg=scope3
            )

        return emissions
```

## 6.6 Integration Security

### 6.6.1 Security Architecture

**Integration Security Framework:**

```yaml
security_layers:
  transport:
    - TLS 1.3 for all external connections
    - Certificate pinning for critical integrations
    - Mutual TLS for B2B connections

  authentication:
    - OAuth 2.0 for user-facing APIs
    - API keys for service-to-service
    - Certificate-based for device connections

  authorization:
    - Role-based access control
    - Resource-level permissions
    - Time-based access restrictions

  data_protection:
    - Encryption at rest (AES-256)
    - Field-level encryption for sensitive data
    - Data masking in logs

  monitoring:
    - Real-time anomaly detection
    - API usage analytics
    - Integration health monitoring
```

---

**Chapter Summary:**

This chapter covered System Integration (Phase 4):

- Smart grid integration via OpenADR and IEEE 2030.5
- Dynamic pricing and demand response
- Renewable energy (solar, battery, VPP) integration
- Legacy BAS integration (BACnet, Modbus)
- Compliance reporting (Energy Star, GRESB)
- Integration security best practices

In the next chapter, we will explore the Security Framework, covering authentication, encryption, and compliance requirements in depth.

---

© 2025 World Certification Industry Association (WIA)
弘益人間 (Hongik Ingan) - Benefit All Humanity
