# WIA-ENE-055: Sea Level Rise Response
## PHASE 4 - Integration Specification

**Version:** 1.0.0
**Status:** Standard
**Category:** Energy & Environment (ENE)

---

## Overview

This specification defines integration patterns, reference implementations, and best practices for connecting WIA-ENE-055 with coastal cities, disaster agencies, climate models, port authorities, and other critical systems.

---

## 1. Coastal Cities Integration

### 1.1 Urban Planning Systems

**Integration Architecture:**

```
┌─────────────────────┐         ┌──────────────────┐
│  City Planning GIS  │◄────────┤  WIA-ENE-055 API │
│  (ArcGIS/QGIS)      │         │                  │
└─────────────────────┘         └──────────────────┘
         │                               │
         ▼                               ▼
┌─────────────────────┐         ┌──────────────────┐
│  Zoning Database    │         │  Flood Risk DB   │
└─────────────────────┘         └──────────────────┘
```

**Implementation Example:**

```python
from wia_ene_055 import SeaLevelRiseClient
import arcpy

# Initialize WIA client
slr_client = SeaLevelRiseClient(api_key=os.getenv('WIA_API_KEY'))

# Get flood risk for coastal parcels
parcels = arcpy.da.SearchCursor('coastal_parcels', ['PARCEL_ID', 'ELEVATION', 'SHAPE@'])

for parcel_id, elevation, geometry in parcels:
    # Get centroid coordinates
    centroid = geometry.centroid

    # Assess flood risk
    risk = slr_client.assess_flood_risk(
        latitude=centroid.Y,
        longitude=centroid.X,
        elevation_msl=elevation,
        scenario='RCP8.5',
        year=2050
    )

    # Update zoning restrictions based on risk
    if risk['risk_level'] == 'HIGH':
        arcpy.da.UpdateCursor('coastal_parcels',
            ['PARCEL_ID', 'ZONING_RESTRICTIONS'],
            where_clause=f"PARCEL_ID = '{parcel_id}'",
            update_row=[parcel_id, 'COASTAL_HIGH_RISK']
        )

        # Require elevated construction
        minimum_elevation = risk['recommended_elevation']
        update_building_code(parcel_id, minimum_elevation)
```

### 1.2 GIS Data Exchange

**Export Flood Zones to Shapefile:**

```python
import geopandas as gpd
from shapely.geometry import Polygon

# Get flood zone data from WIA-ENE-055
flood_zones = slr_client.get_flood_zones(
    region='miami-dade',
    scenario='RCP8.5',
    year=2050
)

# Convert to GeoDataFrame
geometries = []
properties = []

for zone in flood_zones['features']:
    geom = Polygon(zone['geometry']['coordinates'][0])
    geometries.append(geom)
    properties.append(zone['properties'])

gdf = gpd.GeoDataFrame(properties, geometry=geometries, crs='EPSG:4326')

# Export to shapefile
gdf.to_file('flood_zones_2050.shp')

# Import into ArcGIS
arcpy.conversion.FeatureClassToFeatureClass(
    'flood_zones_2050.shp',
    'city_planning.gdb',
    'sea_level_rise_zones_2050'
)
```

### 1.3 Real-Time Dashboard Integration

**Web Dashboard Example:**

```javascript
import { SeaLevelRiseAPI } from '@wia/ene-055';
import mapboxgl from 'mapbox-gl';

const slrAPI = new SeaLevelRiseAPI({
  apiKey: process.env.WIA_API_KEY
});

// Initialize map
const map = new mapboxgl.Map({
  container: 'map',
  style: 'mapbox://styles/mapbox/dark-v10',
  center: [-80.1918, 25.7617],
  zoom: 11
});

// Subscribe to real-time sea level updates
const ws = slrAPI.streamSeaLevel({
  stations: ['NOAA-8638610', 'NOAA-8723214'],
  onUpdate: (data) => {
    updateSeaLevelDisplay(data);
    updateFloodRiskLayer(data);
  }
});

// Update flood risk visualization
async function updateFloodRiskLayer(seaLevelData) {
  const currentLevel = seaLevelData.water_level;

  // Get properties at risk at current level
  const atRisk = await slrAPI.getPropertiesAtRisk({
    region: 'miami-dade',
    water_level: currentLevel,
    include_king_tides: true
  });

  // Update map layer
  map.getSource('at-risk-properties').setData({
    type: 'FeatureCollection',
    features: atRisk.features
  });
}
```

---

## 2. Disaster Agency Integration

### 2.1 Emergency Management Systems

**FEMA Integration:**

```python
from wia_ene_055 import SeaLevelRiseClient, AlertSubscription
import fema_api

slr_client = SeaLevelRiseClient(api_key=os.getenv('WIA_API_KEY'))

# Subscribe to high-severity alerts
alert_sub = AlertSubscription(
    regions=['miami-dade', 'broward', 'palm-beach'],
    severity=['HIGH', 'EXTREME'],
    callback=handle_emergency_alert
)

def handle_emergency_alert(alert):
    """Forward critical alerts to FEMA systems"""

    if alert['severity'] == 'EXTREME':
        # Trigger emergency response
        fema_api.create_incident(
            type='COASTAL_FLOODING',
            severity='MAJOR',
            location=alert['location'],
            description=alert['message'],
            predicted_impact=alert['projected_inundation']
        )

        # Activate evacuation protocols
        if alert['recommended_action'] == 'EVACUATE':
            activate_evacuation_plan(
                zones=alert['affected_zones'],
                timeline=alert['evacuation_window']
            )

        # Alert emergency services
        notify_first_responders(alert)

slr_client.subscribe_alerts(alert_sub)
```

### 2.2 Evacuation Route Planning

**Dynamic Route Optimization:**

```python
import networkx as nx
from datetime import datetime, timedelta

def plan_evacuation_routes(flood_alert):
    """Generate evacuation routes based on flood projections"""

    # Get road network
    road_network = get_city_road_network()

    # Get flood projections
    flood_extent = slr_client.project_flood_extent(
        water_level=flood_alert['predicted_level'],
        timestamp=flood_alert['peak_time'],
        region=flood_alert['location']
    )

    # Remove flooded road segments
    G = nx.Graph()
    for road in road_network:
        if not is_flooded(road, flood_extent):
            G.add_edge(road['start'], road['end'], weight=road['length'])

    # Find evacuation routes from each zone
    evacuation_routes = {}

    for zone in flood_alert['affected_zones']:
        # Find shortest path to nearest shelter
        shelters = get_available_shelters(zone)

        for shelter in shelters:
            try:
                path = nx.shortest_path(
                    G,
                    source=zone['center'],
                    target=shelter['location'],
                    weight='weight'
                )

                evacuation_routes[zone['id']] = {
                    'shelter': shelter,
                    'route': path,
                    'distance_km': calculate_route_distance(path),
                    'estimated_time': calculate_travel_time(path, traffic_conditions)
                }
                break

            except nx.NetworkXNoPath:
                # No route available, try next shelter
                continue

    return evacuation_routes
```

### 2.3 Emergency Alert Broadcasting

**Multi-Channel Alert Distribution:**

```python
from wia_ene_055 import CAPAlertGenerator

def broadcast_emergency_alert(flood_alert):
    """Broadcast alerts via multiple channels"""

    # Generate CAP-compliant alert
    cap_generator = CAPAlertGenerator()
    cap_alert = cap_generator.create_alert(
        event='COASTAL_FLOODING',
        severity=flood_alert['severity'],
        urgency='IMMEDIATE',
        certainty='LIKELY',
        area=flood_alert['affected_area'],
        headline=flood_alert['headline'],
        description=flood_alert['description'],
        instruction=flood_alert['instructions']
    )

    # Broadcast via EAS (Emergency Alert System)
    eas_broadcast(cap_alert)

    # Send SMS to affected residents
    residents = get_residents_in_area(flood_alert['affected_area'])
    for resident in residents:
        send_sms(
            to=resident['phone'],
            message=format_sms_alert(flood_alert)
        )

    # Post to social media
    post_twitter(f"⚠️ FLOOD ALERT: {flood_alert['headline']}")

    # Update mobile apps
    push_notification(
        title="Flood Alert",
        body=flood_alert['headline'],
        data={'alert_id': flood_alert['id']}
    )

    # Display on digital signage
    update_digital_signs(flood_alert['message'])
```

---

## 3. Climate Model Integration

### 3.1 CMIP6 Data Integration

**Import Climate Projections:**

```python
import xarray as xr
from wia_ene_055 import ClimateModelAdapter

# Load CMIP6 model output
ds = xr.open_dataset('cmip6_ssp585_model_output.nc')

# Extract sea level data
sea_level_data = ds['zos']  # Sea surface height above geoid

# Convert to WIA-ENE-055 format
adapter = ClimateModelAdapter()
wia_projections = adapter.from_cmip6(
    dataset=sea_level_data,
    scenario='RCP8.5',
    model='CESM2',
    region='southeast-florida'
)

# Upload to WIA-ENE-055 system
slr_client.upload_projections(wia_projections)
```

### 3.2 NOAA Sea Level Rise Viewer Integration

**Sync with NOAA Data:**

```python
import requests
from datetime import datetime

def sync_noaa_projections():
    """Synchronize with NOAA sea level rise projections"""

    # Fetch NOAA data
    noaa_response = requests.get(
        'https://coast.noaa.gov/slrdata/api/v1/projections',
        params={
            'region': 'southeast_atlantic',
            'scenarios': 'int_low,int,int_high'
        }
    )

    noaa_data = noaa_response.json()

    # Transform to WIA format
    wia_format = []

    for projection in noaa_data['projections']:
        wia_format.append({
            'projection_id': f"NOAA-{projection['id']}",
            'scenario': map_noaa_scenario(projection['scenario']),
            'baseline_year': 2000,
            'location': {
                'latitude': projection['lat'],
                'longitude': projection['lon'],
                'region': projection['region']
            },
            'projections': [
                {
                    'year': proj['year'],
                    'sea_level_rise_m': proj['slr_meters'],
                    'confidence_low': proj['lower_bound'],
                    'confidence_high': proj['upper_bound']
                }
                for proj in projection['timeseries']
            ],
            'data_source': 'NOAA Sea Level Rise Viewer'
        })

    # Upload to WIA system
    slr_client.bulk_upload_projections(wia_format)

def map_noaa_scenario(noaa_scenario):
    """Map NOAA scenarios to RCP scenarios"""
    mapping = {
        'int_low': 'RCP2.6',
        'int': 'RCP4.5',
        'int_high': 'RCP8.5'
    }
    return mapping.get(noaa_scenario, 'RCP4.5')
```

### 3.3 Local Climate Model Downscaling

**Downscale Global Models to Local Regions:**

```python
from wia_ene_055 import DownscalingEngine

downscaler = DownscalingEngine()

# Get global projection
global_projection = slr_client.get_projection(
    region='global',
    scenario='RCP8.5',
    year=2050
)

# Downscale to local region
local_factors = {
    'land_subsidence': 0.002,  # m/year
    'glacial_isostatic_adjustment': -0.0005,  # m/year
    'local_ocean_dynamics': 1.15,  # multiplier
    'storm_surge_amplification': 1.25
}

local_projection = downscaler.downscale(
    global_projection=global_projection,
    region='miami-dade',
    local_factors=local_factors,
    resolution='1km'
)

# Validate against local observations
validation = downscaler.validate(
    projection=local_projection,
    observations=get_local_tide_gauge_data('NOAA-8638610')
)

if validation['rmse'] < 0.05:  # 5cm threshold
    slr_client.upload_projection(local_projection)
```

---

## 4. Port Authority Integration

### 4.1 Maritime Infrastructure Monitoring

**Port Facility Risk Assessment:**

```python
from wia_ene_055 import PortInfrastructureAnalyzer

port_analyzer = PortInfrastructureAnalyzer()

# Get port facilities
facilities = port_authority_db.query("""
    SELECT facility_id, facility_type, elevation_msl, location
    FROM port_facilities
    WHERE port_id = 'PORT-MIAMI'
""")

# Assess risk for each facility
risk_assessments = []

for facility in facilities:
    risk = slr_client.assess_infrastructure_risk(
        facility_type=facility['facility_type'],
        elevation=facility['elevation_msl'],
        location=facility['location'],
        scenarios=['RCP4.5', 'RCP8.5'],
        years=[2030, 2050, 2100]
    )

    risk_assessments.append({
        'facility_id': facility['facility_id'],
        'facility_type': facility['facility_type'],
        'risk_timeline': risk['timeline'],
        'adaptation_required': risk['adaptation_required'],
        'estimated_cost': risk['adaptation_cost'],
        'operational_impact': risk['operational_impact']
    })

# Generate adaptation plan
adaptation_plan = port_analyzer.generate_adaptation_plan(
    facilities=risk_assessments,
    budget=100_000_000,
    priority='minimize_operational_disruption'
)

# Update port master plan
port_authority_db.update_master_plan(adaptation_plan)
```

### 4.2 Shipping Channel Depth Management

**Dynamic Draft Management:**

```python
def manage_shipping_channel_depth():
    """Adjust channel depth recommendations based on sea level"""

    # Get current and projected sea levels
    current_level = slr_client.get_current_sea_level('PORT-MIAMI')
    projections = slr_client.get_projections(
        region='port-miami',
        scenario='RCP8.5',
        years=[2030, 2040, 2050]
    )

    # Calculate required dredging
    current_depth = get_channel_depth('MAIN_CHANNEL')
    required_clearance = 15.0  # meters under keel

    for projection in projections:
        future_level = current_level['water_level'] + projection['sea_level_rise_m']

        # Account for largest vessels
        max_vessel_draft = 14.5  # Post-Panamax

        required_depth = future_level + max_vessel_draft + required_clearance

        dredging_required = required_depth - current_depth

        if dredging_required > 0:
            schedule_dredging(
                channel='MAIN_CHANNEL',
                target_depth=required_depth,
                completion_year=projection['year'],
                estimated_cost=calculate_dredging_cost(dredging_required)
            )
```

### 4.3 Berth Elevation Planning

**Upgrade Berth Infrastructure:**

```python
def plan_berth_upgrades():
    """Plan berth elevation upgrades based on sea level rise"""

    berths = get_port_berths('PORT-MIAMI')

    for berth in berths:
        # Get current and future water levels
        current_level = slr_client.get_current_sea_level(berth['location'])

        projection_2050 = slr_client.get_projection(
            location=berth['location'],
            scenario='RCP8.5',
            year=2050
        )

        future_level = current_level['water_level'] + projection_2050['sea_level_rise_m']

        # Calculate required berth elevation
        # Berth should be 2m above high water mark
        high_water_mark = future_level + 0.5  # Spring tide
        required_elevation = high_water_mark + 2.0

        current_elevation = berth['elevation_msl']
        elevation_increase = required_elevation - current_elevation

        if elevation_increase > 0.5:  # 50cm threshold
            create_upgrade_project(
                berth_id=berth['id'],
                current_elevation=current_elevation,
                target_elevation=required_elevation,
                estimated_cost=calculate_elevation_cost(berth, elevation_increase),
                priority=calculate_priority(berth),
                completion_deadline=2045  # 5-year buffer before 2050
            )
```

---

## 5. Insurance Sector Integration

### 5.1 Risk Premium Calculation

**Flood Risk Underwriting:**

```python
from wia_ene_055 import InsuranceRiskCalculator

risk_calc = InsuranceRiskCalculator()

def calculate_flood_insurance_premium(property_data):
    """Calculate flood insurance premium based on SLR risk"""

    # Get WIA-ENE-055 risk assessment
    slr_risk = slr_client.assess_flood_risk(
        latitude=property_data['latitude'],
        longitude=property_data['longitude'],
        elevation_msl=property_data['elevation'],
        scenario='RCP8.5',
        years=[2025, 2030, 2040, 2050]
    )

    # Calculate expected annual loss
    eal = risk_calc.calculate_expected_annual_loss(
        property_value=property_data['value'],
        flood_probability=slr_risk['annual_flood_chance'],
        damage_function=get_depth_damage_function(property_data['type'])
    )

    # Apply insurance formula
    premium = eal * 1.5  # 50% margin
    premium += administrative_costs
    premium *= catastrophe_loading_factor

    # Adjust for mitigation measures
    if property_data['has_flood_barriers']:
        premium *= 0.85

    if property_data['elevation'] > slr_risk['flood_threshold'] + 1.0:
        premium *= 0.70

    return {
        'annual_premium': premium,
        'expected_annual_loss': eal,
        'risk_level': slr_risk['risk_level'],
        'flood_probability': slr_risk['flood_probability']
    }
```

### 5.2 Portfolio Risk Assessment

**Assess Coastal Property Portfolio:**

```python
def assess_portfolio_slr_risk(properties):
    """Assess sea level rise exposure across property portfolio"""

    # Batch risk assessment
    locations = [
        {
            'id': prop['id'],
            'latitude': prop['latitude'],
            'longitude': prop['longitude'],
            'elevation_msl': prop['elevation']
        }
        for prop in properties
    ]

    batch_risk = slr_client.batch_flood_risk(
        locations=locations,
        scenario='RCP8.5',
        year=2050
    )

    # Calculate portfolio metrics
    total_value = sum(p['value'] for p in properties)
    value_at_risk = 0
    high_risk_count = 0

    for prop_id, risk in batch_risk['results'].items():
        property_value = next(p['value'] for p in properties if p['id'] == prop_id)

        if risk['risk_level'] in ['HIGH', 'EXTREME']:
            value_at_risk += property_value
            high_risk_count += 1

    return {
        'total_properties': len(properties),
        'total_value': total_value,
        'value_at_risk': value_at_risk,
        'value_at_risk_percent': (value_at_risk / total_value) * 100,
        'high_risk_properties': high_risk_count,
        'recommended_action': 'DIVEST' if value_at_risk / total_value > 0.3 else 'MONITOR'
    }
```

---

## 6. Conservation & Ecosystem Integration

### 6.1 Habitat Migration Planning

**Coastal Ecosystem Adaptation:**

```python
from wia_ene_055 import EcosystemAnalyzer

eco_analyzer = EcosystemAnalyzer()

def plan_habitat_migration(ecosystem_type, current_location):
    """Plan ecosystem migration in response to sea level rise"""

    # Get sea level projections
    projections = slr_client.get_projections(
        location=current_location,
        scenario='RCP8.5',
        years=range(2025, 2101, 5)
    )

    # Model habitat suitability over time
    suitability_timeline = []

    for projection in projections:
        water_level = projection['sea_level_rise_m']

        # Calculate habitat extent
        suitable_area = calculate_suitable_area(
            ecosystem_type=ecosystem_type,
            water_level=water_level,
            salinity=model_salinity(water_level),
            elevation_range=get_elevation_range(ecosystem_type)
        )

        suitability_timeline.append({
            'year': projection['year'],
            'suitable_area_km2': suitable_area,
            'migration_direction': determine_migration_direction(suitable_area)
        })

    # Identify migration corridors
    corridors = eco_analyzer.identify_migration_corridors(
        from_location=current_location,
        to_locations=find_future_suitable_areas(suitability_timeline),
        barriers=get_development_barriers()
    )

    # Generate conservation plan
    return {
        'current_habitat_viable_until': find_viability_threshold(suitability_timeline),
        'migration_corridors': corridors,
        'land_acquisition_needed': calculate_land_needs(corridors),
        'restoration_sites': identify_restoration_sites(corridors),
        'estimated_cost': estimate_migration_cost(corridors)
    }
```

### 6.2 Wetland Restoration

**Optimize Wetland Creation:**

```python
def optimize_wetland_restoration(budget, region):
    """Identify optimal locations for wetland restoration"""

    # Get elevation data
    elevation_data = slr_client.get_elevation_profile(region)

    # Get sea level projections
    slr_2050 = slr_client.get_projection(
        region=region,
        scenario='RCP4.5',
        year=2050
    )

    future_water_level = slr_2050['sea_level_rise_m']

    # Find areas that will be in tidal zone
    candidate_sites = []

    for point in elevation_data['elevation_points']:
        elevation = point['elevation_msl']

        # Optimal wetland elevation: current high tide to future high tide
        if 0.5 <= elevation <= future_water_level + 1.0:
            # Check land availability
            parcel = get_parcel_info(point['latitude'], point['longitude'])

            if parcel['available_for_conservation']:
                candidate_sites.append({
                    'location': {
                        'lat': point['latitude'],
                        'lon': point['longitude']
                    },
                    'elevation': elevation,
                    'area_hectares': parcel['area'],
                    'acquisition_cost': parcel['price'],
                    'ecosystem_value': calculate_ecosystem_value(parcel)
                })

    # Optimize site selection within budget
    selected_sites = optimize_site_selection(
        sites=candidate_sites,
        budget=budget,
        objective='maximize_ecosystem_value'
    )

    return {
        'selected_sites': selected_sites,
        'total_area_hectares': sum(s['area_hectares'] for s in selected_sites),
        'total_cost': sum(s['acquisition_cost'] for s in selected_sites),
        'projected_carbon_sequestration': calculate_carbon_benefit(selected_sites),
        'storm_surge_protection': calculate_surge_protection(selected_sites)
    }
```

---

## 7. Cross-Standard Integration

### 7.1 Integration with WIA-CLIMATE Standards

**Combined Climate Response:**

```python
from wia_ene_055 import SeaLevelRiseClient
from wia_climate import ClimateActionPlatform

slr_client = SeaLevelRiseClient(api_key=os.getenv('WIA_ENE055_KEY'))
climate_client = ClimateActionPlatform(api_key=os.getenv('WIA_CLIMATE_KEY'))

# Get sea level rise projections
slr_data = slr_client.get_projections(
    region='coastal-city',
    scenario='RCP8.5',
    year=2050
)

# Link to emissions reduction goals
emissions_target = climate_client.get_emissions_target('coastal-city')

# Model benefit of emissions reduction on sea level
slr_rcp45 = slr_client.get_projections(
    region='coastal-city',
    scenario='RCP4.5',  # Lower emissions scenario
    year=2050
)

sea_level_benefit = slr_data['sea_level_rise_m'] - slr_rcp45['sea_level_rise_m']

# Calculate avoided damages
avoided_damages = calculate_avoided_flood_damages(sea_level_benefit)

# Report integrated climate action
climate_client.report_co_benefit(
    action='emissions_reduction',
    co_benefit='reduced_sea_level_rise',
    quantified_benefit=avoided_damages
)
```

---

## 8. Mobile Application Integration

### 8.1 React Native Integration

**Mobile App Example:**

```javascript
import React, { useState, useEffect } from 'react';
import { SeaLevelRiseSDK } from '@wia/ene-055-mobile';
import MapView, { Polygon } from 'react-native-maps';

const SeaLevelApp = () => {
  const [floodZones, setFloodZones] = useState([]);
  const [userLocation, setUserLocation] = useState(null);
  const [riskLevel, setRiskLevel] = useState(null);

  const slrSDK = new SeaLevelRiseSDK({
    apiKey: process.env.WIA_API_KEY
  });

  useEffect(() => {
    // Get user location
    navigator.geolocation.getCurrentPosition((position) => {
      setUserLocation({
        latitude: position.coords.latitude,
        longitude: position.coords.longitude
      });

      // Assess risk at user location
      slrSDK.assessRisk({
        latitude: position.coords.latitude,
        longitude: position.coords.longitude,
        scenario: 'RCP8.5',
        year: 2050
      }).then(risk => {
        setRiskLevel(risk.risk_level);
      });
    });

    // Load flood zones
    slrSDK.getFloodZones({
      region: 'miami-dade',
      scenario: 'RCP8.5',
      year: 2050
    }).then(zones => {
      setFloodZones(zones.features);
    });
  }, []);

  return (
    <MapView
      initialRegion={{
        latitude: 25.7617,
        longitude: -80.1918,
        latitudeDelta: 0.5,
        longitudeDelta: 0.5
      }}
    >
      {floodZones.map((zone, index) => (
        <Polygon
          key={index}
          coordinates={zone.geometry.coordinates[0].map(coord => ({
            latitude: coord[1],
            longitude: coord[0]
          }))}
          fillColor={getRiskColor(zone.properties.risk_level)}
          strokeColor="#000"
        />
      ))}
    </MapView>
  );
};
```

---

## 9. Testing & Validation

### 9.1 Integration Testing

**Test Suite Example:**

```python
import unittest
from wia_ene_055 import SeaLevelRiseClient

class TestCityIntegration(unittest.TestCase):

    def setUp(self):
        self.client = SeaLevelRiseClient(
            api_key=os.getenv('TEST_API_KEY'),
            base_url='https://sandbox-api.wia.global/ene-055/v1'
        )

    def test_flood_risk_assessment(self):
        """Test flood risk assessment integration"""
        risk = self.client.assess_flood_risk(
            latitude=25.7617,
            longitude=-80.1918,
            elevation_msl=1.5,
            scenario='RCP8.5',
            year=2050
        )

        self.assertIn('risk_level', risk)
        self.assertIn(risk['risk_level'], ['LOW', 'MODERATE', 'HIGH', 'EXTREME'])
        self.assertIsInstance(risk['flood_probability'], float)
        self.assertTrue(0 <= risk['flood_probability'] <= 1)

    def test_batch_assessment(self):
        """Test batch risk assessment"""
        locations = [
            {'id': '1', 'latitude': 25.7617, 'longitude': -80.1918, 'elevation_msl': 1.5},
            {'id': '2', 'latitude': 25.7850, 'longitude': -80.1298, 'elevation_msl': 1.2}
        ]

        results = self.client.batch_flood_risk(
            locations=locations,
            scenario='RCP8.5',
            year=2050
        )

        self.assertEqual(len(results['results']), 2)
        self.assertIn('summary', results)

if __name__ == '__main__':
    unittest.main()
```

---

## 10. Best Practices

### 10.1 Performance Optimization

- **Cache frequently accessed projections** (update monthly)
- **Use batch APIs** for multiple location assessments
- **Implement pagination** for large datasets
- **Use WebSocket streaming** for real-time monitoring
- **Compress geospatial data** using GeoParquet or optimized GeoJSON

### 10.2 Error Handling

```python
from wia_ene_055.exceptions import (
    StationNotFoundError,
    RateLimitExceededError,
    InvalidLocationError
)

try:
    risk = slr_client.assess_flood_risk(...)
except StationNotFoundError:
    # Fall back to regional projection
    risk = slr_client.get_regional_projection(...)
except RateLimitExceededError:
    # Implement exponential backoff
    time.sleep(60)
    risk = slr_client.assess_flood_risk(...)
except InvalidLocationError as e:
    logger.error(f"Invalid location: {e}")
    raise
```

### 10.3 Security Considerations

- **Never commit API keys** to version control
- **Use environment variables** for credentials
- **Implement rate limiting** on client side
- **Validate all input data** before sending to API
- **Use HTTPS only** for all communications

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
