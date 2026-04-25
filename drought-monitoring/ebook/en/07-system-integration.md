# Chapter 7: System Integration

## Connecting Drought Monitoring with Agricultural and Environmental Systems

---

## 7.1 Farm Management System Integration

### Integration Overview

Farm management information systems (FMIS) serve as the central hub for agricultural decision-making. Integrating drought monitoring with FMIS enables:

| Capability | Benefit | Implementation |
|------------|---------|----------------|
| Automated alerts | Timely drought awareness | Webhook/API integration |
| Decision support | Informed crop planning | Data layer integration |
| Historical analysis | Long-term planning | Data warehouse sync |
| Field-level assessment | Precision management | Spatial data integration |

### FMIS Architecture Integration

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     Farm Management System Integration                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────┐                           ┌─────────────────┐         │
│  │  Drought        │                           │  Farm           │         │
│  │  Monitoring     │                           │  Management     │         │
│  │  System         │                           │  System         │         │
│  │                 │                           │                 │         │
│  │  ┌───────────┐  │      Integration Layer    │  ┌───────────┐  │         │
│  │  │ Index API │──┼──────────────────────────▶│  │ Crop      │  │         │
│  │  └───────────┘  │           REST            │  │ Planning  │  │         │
│  │                 │                           │  └───────────┘  │         │
│  │  ┌───────────┐  │      ┌───────────────┐    │                 │         │
│  │  │ Alert     │──┼─────▶│  Integration  │───▶│  ┌───────────┐  │         │
│  │  │ Service   │  │      │  Middleware   │    │  │ Irrigation│  │         │
│  │  └───────────┘  │      └───────────────┘    │  │ Scheduler │  │         │
│  │                 │                           │  └───────────┘  │         │
│  │  ┌───────────┐  │           Webhook         │                 │         │
│  │  │ Forecast  │──┼──────────────────────────▶│  ┌───────────┐  │         │
│  │  │ Service   │  │                           │  │ Financial │  │         │
│  │  └───────────┘  │                           │  │ Planning  │  │         │
│  │                 │                           │  └───────────┘  │         │
│  └─────────────────┘                           └─────────────────┘         │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Integration Patterns

**Pattern 1: Pull-Based Integration**

```python
# FMIS pulls drought data on demand
class DroughtDataConnector:
    def __init__(self, api_key, base_url):
        self.client = DroughtMonitorClient(api_key=api_key, base_url=base_url)

    def get_field_drought_status(self, field_geometry, date=None):
        """
        Get drought status for a specific field.

        Parameters:
        -----------
        field_geometry : GeoJSON
            Field boundary geometry
        date : str
            Query date (default: today)

        Returns:
        --------
        dict : Drought status for field
        """
        if date is None:
            date = datetime.now().strftime('%Y-%m-%d')

        # Query polygon endpoint
        result = self.client.locations.polygons.query(
            geometry=field_geometry,
            indices=['pdsi', 'spi_3', 'soil_moisture', 'ndvi_anomaly'],
            date=date
        )

        # Calculate composite status
        status = self.calculate_field_status(result)

        return {
            'field_id': field_geometry.get('properties', {}).get('id'),
            'date': date,
            'drought_status': status,
            'indices': result['results'],
            'recommendation': self.generate_recommendation(status)
        }

    def calculate_field_status(self, drought_data):
        """Calculate overall field drought status."""
        pdsi = drought_data['results'].get('pdsi', {}).get('mean', 0)
        spi = drought_data['results'].get('spi_3', {}).get('mean', 0)
        sm = drought_data['results'].get('soil_moisture', {}).get('percentile', 50)

        # Weighted classification
        composite = (pdsi * 0.4 + spi * 2 * 0.3 + (sm - 50) / 12.5 * 0.3)

        if composite < -3:
            return 'severe'
        elif composite < -2:
            return 'moderate'
        elif composite < -1:
            return 'abnormal'
        else:
            return 'normal'

    def generate_recommendation(self, status):
        """Generate management recommendation based on status."""
        recommendations = {
            'severe': 'Consider emergency irrigation, review insurance options',
            'moderate': 'Increase irrigation frequency, monitor crop stress',
            'abnormal': 'Monitor conditions, prepare contingency plans',
            'normal': 'Continue standard management practices'
        }
        return recommendations.get(status, 'Monitor conditions')
```

**Pattern 2: Push-Based Integration (Webhooks)**

```python
# Webhook handler for FMIS to receive drought alerts
from flask import Flask, request, jsonify

app = Flask(__name__)

@app.route('/drought-webhook', methods=['POST'])
def handle_drought_alert():
    """
    Receive and process drought alerts from monitoring system.
    """
    payload = request.json

    # Validate webhook signature
    if not validate_webhook_signature(request):
        return jsonify({'error': 'Invalid signature'}), 401

    # Extract alert information
    alert = payload.get('notification', {}).get('alert', {})
    location = payload.get('notification', {}).get('location', {})
    status = payload.get('notification', {}).get('current_status', {})

    # Find affected fields
    affected_fields = find_fields_in_region(location['region_id'])

    # Process alert for each field
    for field in affected_fields:
        # Update field drought status
        update_field_drought_status(
            field_id=field['id'],
            status=status,
            alert_severity=alert.get('severity')
        )

        # Generate notifications
        if alert.get('severity') in ['high', 'critical']:
            notify_farm_manager(
                field=field,
                alert=alert,
                status=status
            )

        # Update irrigation schedule if needed
        if status.get('drought_category') in ['D2', 'D3', 'D4']:
            adjust_irrigation_schedule(
                field_id=field['id'],
                drought_level=status['drought_category']
            )

    return jsonify({
        'status': 'processed',
        'affected_fields': len(affected_fields)
    })
```

### Data Mapping Standards

```json
{
  "fmis_drought_mapping": {
    "crop_planning": {
      "source_fields": ["pdsi", "spi_12", "forecast_3m"],
      "target_fields": ["soil_moisture_status", "planting_risk", "yield_forecast_adjustment"]
    },
    "irrigation_scheduling": {
      "source_fields": ["soil_moisture", "et_fraction", "ndvi_anomaly"],
      "target_fields": ["water_deficit_mm", "irrigation_priority", "stress_indicator"]
    },
    "financial_planning": {
      "source_fields": ["pdsi_trend", "drought_probability", "historical_correlation"],
      "target_fields": ["yield_risk_factor", "insurance_trigger", "revenue_adjustment"]
    }
  }
}
```

---

## 7.2 Irrigation Control System Connectivity

### Smart Irrigation Integration Architecture

Modern precision irrigation systems can receive drought intelligence to optimize water application:

```
Irrigation Control Integration:
===============================

Drought Monitoring System
         │
         ▼
┌─────────────────────────┐
│  Integration Gateway    │
│  ┌───────────────────┐  │
│  │ Protocol Adapter  │  │
│  │ (REST → MQTT/     │  │
│  │  Modbus/Custom)   │  │
│  └───────────────────┘  │
└─────────────────────────┘
         │
    ┌────┴────┬────────────┐
    │         │            │
    ▼         ▼            ▼
┌───────┐ ┌───────┐ ┌───────────┐
│ Pivot │ │ Drip  │ │ Zone      │
│ System│ │System │ │Controllers│
└───────┘ └───────┘ └───────────┘
```

### Protocol Translation

```python
class IrrigationIntegrationGateway:
    """
    Gateway to translate drought data to irrigation control protocols.
    """

    def __init__(self, drought_client, irrigation_config):
        self.drought_client = drought_client
        self.config = irrigation_config
        self.protocol_handlers = {
            'mqtt': MQTTHandler(irrigation_config['mqtt']),
            'modbus': ModbusHandler(irrigation_config['modbus']),
            'rest': RESTHandler(irrigation_config['rest']),
            'proprietary': ProprietaryHandler(irrigation_config['proprietary'])
        }

    def update_irrigation_zone(self, zone_id, drought_data):
        """
        Update irrigation zone settings based on drought data.

        Parameters:
        -----------
        zone_id : str
            Irrigation zone identifier
        drought_data : dict
            Current drought conditions
        """
        # Calculate irrigation adjustment
        adjustment = self.calculate_irrigation_adjustment(drought_data)

        # Get zone configuration
        zone_config = self.config['zones'][zone_id]
        protocol = zone_config['protocol']

        # Translate to irrigation command
        command = self.translate_to_irrigation_command(
            zone_config,
            adjustment
        )

        # Send to appropriate protocol handler
        handler = self.protocol_handlers[protocol]
        handler.send_command(zone_config['address'], command)

        # Log the update
        self.log_irrigation_update(zone_id, drought_data, adjustment, command)

    def calculate_irrigation_adjustment(self, drought_data):
        """
        Calculate irrigation adjustment factor based on drought severity.

        Returns:
        --------
        dict : Adjustment parameters
        """
        soil_moisture_pct = drought_data.get('soil_moisture', {}).get('percentile', 50)
        et_fraction = drought_data.get('et_fraction', 1.0)
        ndvi_anomaly = drought_data.get('ndvi_anomaly', 0)

        # Calculate adjustment factor
        # Below normal soil moisture → increase irrigation
        sm_factor = max(0.8, min(1.5, (50 - soil_moisture_pct) / 50 + 1))

        # High evaporative demand → increase irrigation
        et_factor = max(0.8, min(1.3, et_fraction))

        # Vegetation stress → increase irrigation
        stress_factor = max(0.9, min(1.3, 1 - ndvi_anomaly * 2))

        # Combined adjustment
        adjustment_factor = (sm_factor * 0.5 + et_factor * 0.3 + stress_factor * 0.2)

        return {
            'factor': adjustment_factor,
            'soil_moisture_contribution': sm_factor,
            'et_contribution': et_factor,
            'stress_contribution': stress_factor,
            'recommendation': self.get_irrigation_recommendation(adjustment_factor)
        }

    def translate_to_irrigation_command(self, zone_config, adjustment):
        """
        Translate adjustment to specific irrigation command.
        """
        base_runtime = zone_config['base_runtime_minutes']
        adjusted_runtime = int(base_runtime * adjustment['factor'])

        return {
            'command': 'SET_RUNTIME',
            'zone_id': zone_config['zone_id'],
            'runtime_minutes': adjusted_runtime,
            'start_time': zone_config.get('preferred_start_time', '06:00'),
            'priority': 'high' if adjustment['factor'] > 1.2 else 'normal',
            'reason': f"Drought adjustment factor: {adjustment['factor']:.2f}"
        }


class MQTTHandler:
    """Handle MQTT protocol for IoT irrigation controllers."""

    def __init__(self, config):
        import paho.mqtt.client as mqtt
        self.client = mqtt.Client()
        self.client.connect(config['broker'], config['port'])
        self.topic_prefix = config['topic_prefix']

    def send_command(self, address, command):
        """Send irrigation command via MQTT."""
        topic = f"{self.topic_prefix}/{address}/command"
        payload = json.dumps(command)
        self.client.publish(topic, payload, qos=1)
```

### Irrigation Scheduling Integration

```python
def integrate_drought_with_irrigation_schedule(farm_id, drought_forecast):
    """
    Integrate drought forecast with weekly irrigation schedule.

    Parameters:
    -----------
    farm_id : str
        Farm identifier
    drought_forecast : dict
        7-day drought forecast

    Returns:
    --------
    dict : Adjusted irrigation schedule
    """
    # Get current schedule
    current_schedule = get_irrigation_schedule(farm_id)

    # Get field drought status
    fields = get_farm_fields(farm_id)

    adjusted_schedule = []

    for day_forecast in drought_forecast['daily']:
        day = day_forecast['date']
        et0 = day_forecast['et0_forecast']
        precip_prob = day_forecast['precipitation_probability']
        precip_amount = day_forecast['precipitation_forecast_mm']

        for field in fields:
            scheduled = current_schedule.get(field['id'], {}).get(day)

            if scheduled:
                # Adjust based on forecast
                adjusted = adjust_scheduled_irrigation(
                    scheduled=scheduled,
                    et0=et0,
                    precip_prob=precip_prob,
                    precip_amount=precip_amount,
                    soil_moisture=day_forecast.get('soil_moisture_forecast', 50)
                )
                adjusted_schedule.append(adjusted)

    return {
        'farm_id': farm_id,
        'schedule': adjusted_schedule,
        'adjustments_made': len([s for s in adjusted_schedule if s['adjusted']]),
        'water_savings_estimated': calculate_water_savings(current_schedule, adjusted_schedule)
    }
```

---

## 7.3 Early Warning System Architecture

### Multi-Tier Warning System

```
Early Warning System Architecture:
==================================

Tier 1: Monitoring & Detection
├── Continuous index monitoring
├── Threshold exceedance detection
├── Trend analysis
└── Forecast integration

Tier 2: Assessment & Classification
├── Severity classification
├── Impact assessment
├── Affected area delineation
└── Confidence evaluation

Tier 3: Warning Generation
├── Warning message composition
├── Target audience identification
├── Priority assignment
└── Multi-language support

Tier 4: Dissemination
├── Emergency broadcast
├── SMS/Email/App push
├── Agency coordination
└── Public communication

Tier 5: Response Coordination
├── Resource mobilization
├── Agency coordination
├── Public guidance
└── Feedback collection
```

### Warning Message Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-ENV-003 Drought Warning Message",
  "type": "object",
  "required": ["header", "warning", "affected_area", "guidance"],
  "properties": {
    "header": {
      "type": "object",
      "properties": {
        "message_id": {"type": "string"},
        "issued_time": {"type": "string", "format": "date-time"},
        "issuing_agency": {"type": "string"},
        "message_type": {
          "type": "string",
          "enum": ["watch", "warning", "emergency", "advisory", "update", "cancellation"]
        },
        "version": {"type": "integer"}
      }
    },
    "warning": {
      "type": "object",
      "properties": {
        "severity": {
          "type": "string",
          "enum": ["minor", "moderate", "severe", "extreme"]
        },
        "certainty": {
          "type": "string",
          "enum": ["observed", "likely", "possible", "unlikely"]
        },
        "urgency": {
          "type": "string",
          "enum": ["immediate", "expected", "future", "past"]
        },
        "headline": {"type": "string"},
        "description": {"type": "string"},
        "effective_time": {"type": "string", "format": "date-time"},
        "expiration_time": {"type": "string", "format": "date-time"}
      }
    },
    "affected_area": {
      "type": "object",
      "properties": {
        "area_description": {"type": "string"},
        "regions": {
          "type": "array",
          "items": {"type": "string"}
        },
        "geometry": {
          "type": "object",
          "description": "GeoJSON geometry"
        },
        "population_affected": {"type": "integer"},
        "agricultural_area_km2": {"type": "number"}
      }
    },
    "conditions": {
      "type": "object",
      "properties": {
        "current_indices": {
          "type": "object",
          "properties": {
            "pdsi": {"type": "number"},
            "spi_3": {"type": "number"},
            "soil_moisture_percentile": {"type": "number"}
          }
        },
        "trend": {"type": "string"},
        "forecast": {"type": "string"}
      }
    },
    "guidance": {
      "type": "object",
      "properties": {
        "general_public": {"type": "string"},
        "agricultural": {"type": "string"},
        "water_utilities": {"type": "string"},
        "emergency_managers": {"type": "string"}
      }
    },
    "resources": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "name": {"type": "string"},
          "url": {"type": "string", "format": "uri"},
          "phone": {"type": "string"}
        }
      }
    }
  }
}
```

### Warning Dissemination System

```python
class DroughtWarningDisseminator:
    """
    Disseminate drought warnings through multiple channels.
    """

    def __init__(self, config):
        self.channels = {
            'sms': SMSChannel(config['sms']),
            'email': EmailChannel(config['email']),
            'push': PushNotificationChannel(config['push']),
            'broadcast': EmergencyBroadcastChannel(config['broadcast']),
            'api': APIChannel(config['api']),
            'social': SocialMediaChannel(config['social'])
        }
        self.subscriber_db = SubscriberDatabase(config['database'])

    async def disseminate_warning(self, warning_message):
        """
        Disseminate warning through all appropriate channels.

        Parameters:
        -----------
        warning_message : dict
            Warning message conforming to WIA-ENV-003 schema
        """
        # Get affected subscribers
        affected_area = warning_message['affected_area']
        subscribers = self.subscriber_db.get_subscribers_in_area(
            affected_area['regions'],
            affected_area.get('geometry')
        )

        # Group by channel preference
        by_channel = self.group_by_channel(subscribers)

        # Determine channels based on severity
        severity = warning_message['warning']['severity']
        active_channels = self.get_active_channels(severity)

        # Disseminate through each channel
        results = {}
        tasks = []

        for channel_name in active_channels:
            if channel_name in by_channel:
                channel = self.channels[channel_name]
                recipients = by_channel[channel_name]

                # Format message for channel
                formatted = self.format_for_channel(
                    warning_message,
                    channel_name
                )

                # Send asynchronously
                task = channel.send(recipients, formatted)
                tasks.append((channel_name, task))

        # Wait for all channels
        for channel_name, task in tasks:
            try:
                result = await task
                results[channel_name] = {
                    'success': True,
                    'sent': result['sent'],
                    'failed': result.get('failed', 0)
                }
            except Exception as e:
                results[channel_name] = {
                    'success': False,
                    'error': str(e)
                }

        # Log dissemination
        self.log_dissemination(warning_message, results)

        return results

    def get_active_channels(self, severity):
        """Determine which channels to activate based on severity."""
        channel_map = {
            'minor': ['email', 'api'],
            'moderate': ['email', 'push', 'api'],
            'severe': ['sms', 'email', 'push', 'api', 'social'],
            'extreme': ['broadcast', 'sms', 'email', 'push', 'api', 'social']
        }
        return channel_map.get(severity, ['email', 'api'])

    def format_for_channel(self, warning, channel_name):
        """Format warning message for specific channel."""
        if channel_name == 'sms':
            return self.format_sms(warning)
        elif channel_name == 'email':
            return self.format_email(warning)
        elif channel_name == 'push':
            return self.format_push(warning)
        else:
            return warning

    def format_sms(self, warning):
        """Format warning for SMS (160 char limit)."""
        severity = warning['warning']['severity'].upper()
        headline = warning['warning']['headline'][:80]
        area = warning['affected_area']['area_description'][:40]

        return f"{severity} DROUGHT: {headline}. Area: {area}. Details: drought.gov"
```

---

## 7.4 Satellite Data Pipeline Integration

### Satellite Data Ingestion Architecture

```
Satellite Data Pipeline:
========================

Data Sources
├── MODIS (NASA DAAC)
├── Landsat (USGS)
├── Sentinel-2 (ESA)
├── VIIRS (NOAA)
└── GOES (NOAA)
         │
         ▼
┌─────────────────────────┐
│  Data Acquisition       │
│  ├── API polling        │
│  ├── FTP sync           │
│  ├── Cloud notification │
│  └── Direct broadcast   │
└─────────────────────────┘
         │
         ▼
┌─────────────────────────┐
│  Ingestion Pipeline     │
│  ├── Format conversion  │
│  ├── Reprojection       │
│  ├── Quality filtering  │
│  └── Metadata extract   │
└─────────────────────────┘
         │
         ▼
┌─────────────────────────┐
│  Processing Pipeline    │
│  ├── Atmospheric corr.  │
│  ├── Cloud masking      │
│  ├── Index calculation  │
│  └── Compositing        │
└─────────────────────────┘
         │
         ▼
┌─────────────────────────┐
│  Publication            │
│  ├── STAC catalog       │
│  ├── COG generation     │
│  ├── API update         │
│  └── Alert trigger      │
└─────────────────────────┘
```

### Satellite Integration Code

```python
class SatelliteDataPipeline:
    """
    Automated satellite data acquisition and processing pipeline.
    """

    def __init__(self, config):
        self.config = config
        self.data_sources = {
            'modis': MODISDataSource(config['modis']),
            'landsat': LandsatDataSource(config['landsat']),
            'sentinel2': Sentinel2DataSource(config['sentinel2'])
        }
        self.processor = DroughtProcessor(config['processing'])
        self.catalog = STACCatalog(config['stac'])

    async def run_daily_pipeline(self):
        """
        Execute daily satellite data pipeline.
        """
        # Query for new data
        new_scenes = await self.discover_new_data()

        # Process each scene
        results = []
        for scene in new_scenes:
            try:
                result = await self.process_scene(scene)
                results.append(result)
            except Exception as e:
                logging.error(f"Failed to process {scene['id']}: {e}")

        # Update catalog
        await self.update_catalog(results)

        # Trigger alerts if needed
        await self.check_alert_triggers(results)

        return results

    async def discover_new_data(self):
        """
        Discover new satellite data from all sources.
        """
        new_scenes = []

        for source_name, source in self.data_sources.items():
            # Query for scenes since last check
            last_check = self.get_last_check_time(source_name)

            scenes = await source.query(
                datetime_range=(last_check, datetime.utcnow()),
                bbox=self.config['area_of_interest'],
                cloud_cover_max=self.config['max_cloud_cover']
            )

            for scene in scenes:
                scene['source'] = source_name
                new_scenes.append(scene)

            self.update_last_check_time(source_name)

        return new_scenes

    async def process_scene(self, scene):
        """
        Process a single satellite scene through the drought pipeline.
        """
        source = self.data_sources[scene['source']]

        # Download scene
        local_path = await source.download(scene['id'], scene['assets'])

        # Load bands
        bands = self.processor.load_bands(local_path, scene['source'])

        # Atmospheric correction
        surface_reflectance = self.processor.atmospheric_correction(
            bands,
            scene['metadata']
        )

        # Cloud masking
        cloud_mask = self.processor.cloud_masking(
            surface_reflectance,
            scene['metadata']
        )

        # Calculate NDVI
        ndvi_result = self.processor.calculate_ndvi(
            surface_reflectance,
            cloud_mask
        )

        # Generate COG
        cog_path = self.generate_cog(
            ndvi_result,
            scene['id'],
            scene['metadata']
        )

        return {
            'scene_id': scene['id'],
            'source': scene['source'],
            'acquisition_date': scene['metadata']['datetime'],
            'products': {
                'ndvi': cog_path,
                'cloud_mask': cloud_mask
            },
            'quality_metrics': ndvi_result['quality_metrics']
        }

    def generate_cog(self, data, scene_id, metadata):
        """
        Generate Cloud Optimized GeoTIFF.
        """
        from rio_cogeo.cogeo import cog_translate
        from rio_cogeo.profiles import cog_profiles

        output_path = f"{self.config['output_dir']}/{scene_id}_NDVI.tif"

        # Write intermediate GeoTIFF
        temp_path = f"/tmp/{scene_id}_temp.tif"
        self.write_geotiff(data, temp_path, metadata)

        # Convert to COG
        cog_translate(
            temp_path,
            output_path,
            cog_profiles.get("deflate"),
            overview_level=4
        )

        return output_path
```

---

## 7.5 Weather Station Network Integration

### Station Network Integration Architecture

```python
class WeatherStationIntegration:
    """
    Integrate multiple weather station networks into drought monitoring.
    """

    def __init__(self, config):
        self.networks = {
            'ghcn': GHCNNetwork(config['ghcn']),
            'scan': SCANNetwork(config['scan']),
            'mesonet': MesonetNetwork(config['mesonet']),
            'cwop': CWOPNetwork(config['cwop']),
            'private': PrivateNetworkAdapter(config['private'])
        }
        self.qc_processor = StationQCProcessor()
        self.interpolator = SpatialInterpolator()

    async def collect_observations(self, date_range, bbox):
        """
        Collect observations from all configured networks.

        Parameters:
        -----------
        date_range : tuple
            (start_datetime, end_datetime)
        bbox : tuple
            Bounding box (west, south, east, north)

        Returns:
        --------
        dict : Collected and QC'd observations
        """
        all_observations = []

        # Collect from each network
        for network_name, network in self.networks.items():
            try:
                obs = await network.get_observations(
                    start_time=date_range[0],
                    end_time=date_range[1],
                    bbox=bbox
                )

                # Add network identifier
                for ob in obs:
                    ob['network'] = network_name

                all_observations.extend(obs)

            except Exception as e:
                logging.warning(f"Failed to collect from {network_name}: {e}")

        # Apply quality control
        qc_results = self.qc_processor.process(all_observations)

        return qc_results

    def create_gridded_product(self, observations, grid_config):
        """
        Create gridded product from point observations.

        Parameters:
        -----------
        observations : list
            QC'd point observations
        grid_config : dict
            Grid configuration (resolution, extent, etc.)

        Returns:
        --------
        dict : Gridded data product
        """
        # Extract variables to grid
        variables = ['temperature', 'precipitation', 'relative_humidity']

        gridded_products = {}

        for var in variables:
            # Extract values and coordinates
            values = []
            coords = []

            for obs in observations:
                if var in obs and obs['quality'][var] == 'good':
                    values.append(obs[var])
                    coords.append((obs['longitude'], obs['latitude']))

            if len(values) < 10:
                logging.warning(f"Insufficient data for {var}")
                continue

            # Interpolate to grid
            grid = self.interpolator.kriging(
                coords=coords,
                values=values,
                grid_config=grid_config
            )

            gridded_products[var] = grid

        return gridded_products
```

### Station Data Harmonization

```json
{
  "station_variable_mapping": {
    "temperature": {
      "ghcn": {"field": "TMAX", "unit": "celsius_10ths", "conversion": 0.1},
      "scan": {"field": "TAIR", "unit": "fahrenheit", "conversion_formula": "(x-32)*5/9"},
      "mesonet": {"field": "T2M", "unit": "celsius", "conversion": 1.0}
    },
    "precipitation": {
      "ghcn": {"field": "PRCP", "unit": "mm_10ths", "conversion": 0.1},
      "scan": {"field": "PREC", "unit": "inches", "conversion": 25.4},
      "mesonet": {"field": "RAIN", "unit": "mm", "conversion": 1.0}
    },
    "soil_moisture": {
      "scan": {"field": "SMS", "unit": "percent", "depths": [5, 10, 20, 50, 100]},
      "mesonet": {"field": "VSM", "unit": "m3/m3", "conversion": 100}
    }
  }
}
```

---

## 7.6 GIS Platform Integration

### GIS Integration Patterns

| Platform | Integration Method | Capabilities |
|----------|-------------------|--------------|
| ArcGIS Online | REST API, Image Service | Visualization, analysis |
| QGIS | WMS/WFS, Python plugin | Desktop analysis |
| Google Earth Engine | API integration | Large-scale processing |
| PostGIS | Direct database connection | Spatial queries |
| GeoServer | OGC services | Web mapping |

### ArcGIS Integration Example

```python
from arcgis.gis import GIS
from arcgis.mapping import WebMap
from arcgis.features import FeatureLayer

class ArcGISIntegration:
    """
    Integrate drought monitoring with ArcGIS Online/Enterprise.
    """

    def __init__(self, portal_url, username, password):
        self.gis = GIS(portal_url, username, password)

    def publish_drought_layer(self, drought_data, layer_name):
        """
        Publish drought data as hosted feature layer.
        """
        # Convert to GeoDataFrame
        gdf = self.drought_to_geodataframe(drought_data)

        # Publish as feature layer
        layer = gdf.spatial.to_featurelayer(
            layer_name,
            gis=self.gis,
            tags=['drought', 'WIA-ENV-003', 'monitoring']
        )

        return layer

    def update_drought_webmap(self, webmap_id, drought_layer):
        """
        Update web map with latest drought data.
        """
        webmap = WebMap(self.gis.content.get(webmap_id))

        # Update drought layer
        for layer in webmap.layers:
            if 'drought' in layer.title.lower():
                layer.layer_definition = self.create_drought_renderer()

        webmap.update()

        return webmap

    def create_drought_renderer(self):
        """
        Create drought severity renderer.
        """
        return {
            "type": "classBreaks",
            "field": "pdsi",
            "classBreakInfos": [
                {"minValue": -10, "maxValue": -4, "symbol": {"color": [139, 0, 0, 200]}, "label": "D4: Exceptional"},
                {"minValue": -4, "maxValue": -3, "symbol": {"color": [255, 0, 0, 200]}, "label": "D3: Extreme"},
                {"minValue": -3, "maxValue": -2, "symbol": {"color": [255, 165, 0, 200]}, "label": "D2: Severe"},
                {"minValue": -2, "maxValue": -1, "symbol": {"color": [255, 255, 0, 200]}, "label": "D1: Moderate"},
                {"minValue": -1, "maxValue": 0, "symbol": {"color": [255, 255, 204, 200]}, "label": "D0: Abnormally Dry"},
                {"minValue": 0, "maxValue": 10, "symbol": {"color": [255, 255, 255, 200]}, "label": "Normal/Wet"}
            ]
        }
```

---

## 7.7 Mobile Application Integration

### Mobile API Design

```yaml
# Mobile-optimized API endpoints
mobile_api:
  base_url: /v1/mobile

  endpoints:
    # Lightweight status check
    /status/location:
      method: GET
      params:
        lat: number
        lon: number
      response_size: <5KB
      caching: 1 hour

    # Offline-capable data package
    /offline/region/{region_id}:
      method: GET
      includes:
        - current_conditions
        - 7_day_forecast
        - alert_thresholds
      response_size: <100KB
      caching: 24 hours

    # Push notification registration
    /notifications/register:
      method: POST
      body:
        device_token: string
        locations: array
        alert_preferences: object
```

### Mobile SDK Integration

```swift
// iOS Swift example
class DroughtMonitorSDK {
    static let shared = DroughtMonitorSDK()

    private let baseURL = "https://api.drought-monitor.org/v1/mobile"

    func getCurrentConditions(lat: Double, lon: Double) async throws -> DroughtStatus {
        let url = URL(string: "\(baseURL)/status/location?lat=\(lat)&lon=\(lon)")!

        let (data, _) = try await URLSession.shared.data(from: url)
        return try JSONDecoder().decode(DroughtStatus.self, from: data)
    }

    func downloadOfflinePackage(regionId: String) async throws -> OfflinePackage {
        let url = URL(string: "\(baseURL)/offline/region/\(regionId)")!

        let (data, _) = try await URLSession.shared.data(from: url)

        // Store in local database
        let package = try JSONDecoder().decode(OfflinePackage.self, from: data)
        LocalDatabase.shared.store(package)

        return package
    }

    func registerForAlerts(locations: [Location], preferences: AlertPreferences) async throws {
        let url = URL(string: "\(baseURL)/notifications/register")!
        var request = URLRequest(url: url)
        request.httpMethod = "POST"

        let body = NotificationRegistration(
            deviceToken: DeviceTokenManager.shared.currentToken,
            locations: locations,
            alertPreferences: preferences
        )

        request.httpBody = try JSONEncoder().encode(body)
        _ = try await URLSession.shared.data(for: request)
    }
}
```

---

## 7.8 Data Warehouse and Analytics Integration

### Analytics Data Model

```sql
-- Drought data warehouse schema
CREATE SCHEMA drought_analytics;

-- Fact table: Daily drought observations
CREATE TABLE drought_analytics.fact_drought_daily (
    date_key INTEGER REFERENCES dim_date(date_key),
    location_key INTEGER REFERENCES dim_location(location_key),
    pdsi DECIMAL(5,2),
    spi_1 DECIMAL(5,2),
    spi_3 DECIMAL(5,2),
    spi_6 DECIMAL(5,2),
    spi_12 DECIMAL(5,2),
    soil_moisture_pct DECIMAL(5,2),
    ndvi DECIMAL(5,3),
    drought_category VARCHAR(5),
    data_quality_score DECIMAL(3,2),
    PRIMARY KEY (date_key, location_key)
) PARTITION BY RANGE (date_key);

-- Dimension: Location
CREATE TABLE drought_analytics.dim_location (
    location_key SERIAL PRIMARY KEY,
    location_id VARCHAR(50),
    location_name VARCHAR(200),
    location_type VARCHAR(50),
    state_province VARCHAR(100),
    country VARCHAR(100),
    latitude DECIMAL(9,6),
    longitude DECIMAL(9,6),
    elevation_m INTEGER,
    climate_zone VARCHAR(50),
    primary_land_use VARCHAR(50),
    geometry GEOMETRY(Polygon, 4326)
);

-- Aggregate table: Monthly drought summary
CREATE TABLE drought_analytics.agg_drought_monthly (
    year_month DATE,
    location_key INTEGER REFERENCES dim_location(location_key),
    avg_pdsi DECIMAL(5,2),
    min_pdsi DECIMAL(5,2),
    max_pdsi DECIMAL(5,2),
    drought_days INTEGER,
    severe_drought_days INTEGER,
    PRIMARY KEY (year_month, location_key)
);
```

### Analytics Integration Code

```python
class DroughtAnalyticsConnector:
    """
    Connect drought monitoring to analytics platforms.
    """

    def __init__(self, warehouse_config):
        self.warehouse = WarehouseConnection(warehouse_config)

    def sync_drought_data(self, date_range):
        """
        Sync drought data to analytics warehouse.
        """
        # Extract data from operational system
        drought_data = self.extract_operational_data(date_range)

        # Transform to warehouse schema
        transformed = self.transform_for_warehouse(drought_data)

        # Load to warehouse
        self.warehouse.bulk_insert(
            'drought_analytics.fact_drought_daily',
            transformed
        )

        # Update aggregates
        self.warehouse.execute("""
            INSERT INTO drought_analytics.agg_drought_monthly
            SELECT
                DATE_TRUNC('month', d.date_key) as year_month,
                location_key,
                AVG(pdsi) as avg_pdsi,
                MIN(pdsi) as min_pdsi,
                MAX(pdsi) as max_pdsi,
                COUNT(*) FILTER (WHERE drought_category IN ('D1','D2','D3','D4')) as drought_days,
                COUNT(*) FILTER (WHERE drought_category IN ('D2','D3','D4')) as severe_drought_days
            FROM drought_analytics.fact_drought_daily f
            JOIN dim_date d ON f.date_key = d.date_key
            WHERE d.date BETWEEN %s AND %s
            GROUP BY 1, 2
            ON CONFLICT (year_month, location_key) DO UPDATE SET
                avg_pdsi = EXCLUDED.avg_pdsi,
                min_pdsi = EXCLUDED.min_pdsi,
                max_pdsi = EXCLUDED.max_pdsi,
                drought_days = EXCLUDED.drought_days,
                severe_drought_days = EXCLUDED.severe_drought_days
        """, [date_range[0], date_range[1]])
```

---

## 7.9 Review Questions and Key Takeaways

### Review Questions

1. **FMIS Integration**: A farm management system needs to display field-level drought status on a dashboard. Design the data flow from the drought monitoring API to the dashboard display.

2. **Irrigation Automation**: A center pivot irrigation system runs on a fixed schedule. How would you modify the control logic to incorporate drought monitoring data while maintaining fail-safe operation?

3. **Early Warning Design**: Design a multi-tier early warning system for a state with diverse stakeholders (farmers, water utilities, emergency managers). How would you customize alerts for each audience?

4. **Satellite Pipeline**: A satellite data pipeline experiences a 48-hour outage. How should the system handle the data gap when service resumes?

5. **Station Integration**: Weather station networks report data at different frequencies (5-minute, hourly, daily). How should these be harmonized for drought index calculation?

6. **GIS Integration**: A water utility uses ArcGIS Enterprise for asset management. Design an integration that overlays drought risk on their pipe network.

7. **Mobile Optimization**: Mobile users in rural areas have limited connectivity. Design an offline-capable drought monitoring feature for a mobile app.

8. **Analytics Architecture**: Design a data warehouse schema that supports both real-time operational queries and historical trend analysis for drought monitoring.

### Key Takeaways

1. **Integration is Multi-Dimensional**: Drought monitoring connects with farm management, irrigation control, early warning, satellite pipelines, weather networks, GIS platforms, mobile apps, and analytics systems.

2. **Pull and Push Patterns**: Both pull-based (on-demand queries) and push-based (webhooks, notifications) integration patterns serve different use cases.

3. **Protocol Translation**: Integration gateways translate between drought monitoring APIs and diverse downstream protocols (MQTT, Modbus, proprietary).

4. **Early Warning Architecture**: Multi-tier warning systems separate monitoring, assessment, warning generation, dissemination, and response coordination.

5. **Satellite Automation**: Automated pipelines handle data discovery, download, processing, quality control, and publication with minimal manual intervention.

6. **Station Harmonization**: Integrating multiple weather networks requires variable mapping, unit conversion, and quality control harmonization.

7. **GIS Compatibility**: Standard OGC services (WMS, WFS) enable integration with diverse GIS platforms while custom APIs support richer functionality.

8. **Mobile Optimization**: Mobile integration requires lightweight APIs, offline capabilities, and efficient push notifications for field users.

9. **Analytics Integration**: Data warehouse schemas support both operational and analytical workloads through appropriate fact/dimension modeling.

10. **Resilience Design**: Integration systems must handle failures gracefully—network outages, data gaps, and service disruptions should not propagate failures.

---

## Chapter Summary

This chapter has detailed the integration patterns connecting drought monitoring systems with the broader ecosystem of agricultural, environmental, and emergency management systems. Effective drought response requires not just accurate monitoring but seamless information flow to decision-makers across diverse platforms.

Farm management integration enables field-level drought assessment and recommendation generation. Irrigation control connectivity allows automated water management adjustments based on drought conditions. Early warning systems disseminate alerts through multiple channels to reach all affected stakeholders.

Satellite data pipelines automate the flow from raw imagery to drought products. Weather station integration harmonizes observations from multiple networks. GIS platform integration enables spatial visualization and analysis.

Mobile optimization addresses the needs of users in the field with limited connectivity. Analytics integration supports both operational monitoring and long-term trend analysis.

Throughout, the emphasis is on standards-based integration using the WIA-ENV-003 specifications—common data formats, APIs, and protocols that enable interoperable systems. These standards reduce integration complexity while ensuring consistent, reliable drought information across all connected systems.

---

**Next Chapter: [Chapter 8: Implementation Guide](08-implementation.md)**
