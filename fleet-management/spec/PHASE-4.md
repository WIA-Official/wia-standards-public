# WIA-AUTO-024: Fleet Management Standard
## PHASE 4 - Integration Guidelines

**Version:** 1.0.0  
**Status:** Active  
**Category:** Automotive / Fleet Management  
**Last Updated:** January 2025

---

## Overview

Phase 4 provides comprehensive guidelines for integrating fleet management systems with enterprise resource planning (ERP) platforms, customer relationship management (CRM) systems, telematics devices, mobile applications, and third-party services. These integration patterns ensure seamless data flow and operational efficiency.

**弘益人間 (Benefit All Humanity)** - Integrated systems eliminate silos, reduce manual work, and enable data-driven decision making across organizations.

---

## 1. ERP System Integration

### 1.1 Common ERP Platforms

- SAP
- Oracle ERP Cloud
- Microsoft Dynamics 365
- NetSuite
- Infor
- Epicor

### 1.2 Integration Points

**Fleet Acquisition & Disposal:**
- Vehicle purchase orders → ERP procurement
- Asset registration → ERP fixed assets
- Depreciation tracking → ERP accounting
- Vehicle disposal → ERP asset retirement

**Maintenance Management:**
- Work orders → ERP work order management
- Parts inventory → ERP inventory management
- Service invoices → ERP accounts payable
- Warranty claims → ERP claims management

**Fuel Management:**
- Fuel purchases → ERP expense management
- Fuel card transactions → ERP AP automation
- Cost allocation → ERP cost accounting

**Driver Payroll:**
- Hours worked → ERP time & attendance
- Mileage allowances → ERP expense claims
- Performance bonuses → ERP payroll

### 1.3 Integration Architecture

```
Fleet Management System
        ↓ (API)
Integration Middleware (ESB)
        ↓ (API/File)
ERP System
```

**Recommended Middleware:**
- MuleSoft
- Dell Boomi
- Apache Camel
- Microsoft Azure Logic Apps

### 1.4 Data Synchronization

**Real-Time (API-based):**
- Critical events (accidents, breakdowns)
- Work order updates
- Driver status changes

**Batch (Scheduled):**
- Daily trip summaries
- Fuel consumption reports
- Maintenance cost summaries
- Monthly performance metrics

**Example Integration Flow:**
```python
def sync_maintenance_to_erp(maintenance_record):
    # Transform fleet data to ERP format
    work_order = {
        'work_order_id': maintenance_record['maintenanceId'],
        'asset_id': maintenance_record['vehicleId'],
        'type': 'FLEET_MAINTENANCE',
        'scheduled_date': maintenance_record['scheduledDate'],
        'cost': maintenance_record['totalCost'],
        'parts': transform_parts(maintenance_record['parts']),
        'labor': transform_labor(maintenance_record['labor'])
    }
    
    # Call ERP API
    response = erp_client.create_work_order(work_order)
    
    # Update fleet system with ERP reference
    update_maintenance_record(
        maintenance_record['maintenanceId'],
        erp_work_order_id=response['id']
    )
```

---

## 2. CRM Integration

### 2.1 Common CRM Platforms

- Salesforce
- HubSpot
- Microsoft Dynamics 365 CRM
- Zoho CRM
- Pipedrive

### 2.2 Integration Use Cases

**Customer Communication:**
- Delivery notifications → CRM timeline
- ETA updates → CRM customer portal
- Service completion → CRM case management

**Service Scheduling:**
- Customer service requests → Fleet dispatch
- Scheduled appointments → Route planning
- Technician assignment → Driver scheduling

**Performance Tracking:**
- On-time delivery metrics → CRM dashboards
- Customer feedback → Driver performance
- Service quality scores → Fleet KPIs

### 2.3 Example: Salesforce Integration

**Apex Trigger for Service Case Creation:**
```apex
trigger FleetServiceRequest on Case (after insert) {
    for (Case c : Trigger.new) {
        if (c.Type == 'Field Service') {
            // Call fleet management API
            HttpRequest req = new HttpRequest();
            req.setEndpoint('https://api.fleet.com/v1/service-requests');
            req.setMethod('POST');
            req.setHeader('Authorization', 'Bearer ' + getFleetApiToken());
            req.setBody(JSON.serialize(new Map<String, Object>{
                'customerId' => c.AccountId,
                'caseId' => c.Id,
                'serviceType' => c.Subject,
                'location' => c.Address__c,
                'timeWindow' => new Map<String, String>{
                    'start' => c.Scheduled_Start__c,
                    'end' => c.Scheduled_End__c
                }
            }));
            
            Http http = new Http();
            HttpResponse res = http.send(req);
            
            // Update case with fleet reference
            c.Fleet_Service_Id__c = ((Map<String, Object>)JSON.deserializeUntyped(res.getBody())).get('serviceId');
            update c;
        }
    }
}
```

---

## 3. Telematics Device Integration

### 3.1 Common Telematics Providers

- Geotab
- Verizon Connect
- Samsara
- Teletrac Navman
- Omnitracs
- Trimble

### 3.2 Device SDK Integration

**Example: Geotab MyGeotab API**
```python
from mygeotab import API

def fetch_vehicle_data(credentials, vehicle_id):
    api = API(
        username=credentials['username'],
        password=credentials['password'],
        database=credentials['database']
    )
    
    # Authenticate
    api.authenticate()
    
    # Get device data
    device = api.get('Device', id=vehicle_id)
    
    # Get latest GPS data
    gps_data = api.get('LogRecord', search={
        'deviceSearch': {'id': vehicle_id},
        'fromDate': datetime.now() - timedelta(hours=1)
    })
    
    # Get diagnostic data
    diagnostics = api.get('StatusData', search={
        'deviceSearch': {'id': vehicle_id},
        'fromDate': datetime.now() - timedelta(hours=1)
    })
    
    return {
        'device': device,
        'gps': gps_data,
        'diagnostics': diagnostics
    }
```

### 3.3 Data Normalization

Different telematics providers use different data formats. Normalize to WIA-AUTO-024 standard:

```python
def normalize_telematics_data(provider, raw_data):
    if provider == 'geotab':
        return normalize_geotab(raw_data)
    elif provider == 'samsara':
        return normalize_samsara(raw_data)
    elif provider == 'verizon':
        return normalize_verizon(raw_data)
    else:
        raise ValueError(f'Unsupported provider: {provider}')

def normalize_geotab(data):
    return {
        'vehicleId': data['device']['serialNumber'],
        'timestamp': data['gps']['dateTime'],
        'location': {
            'latitude': data['gps']['latitude'],
            'longitude': data['gps']['longitude'],
            'speed': data['gps']['speed']
        },
        'engine': {
            'rpm': find_diagnostic(data, 'EngineSpeed'),
            'temperature': find_diagnostic(data, 'EngineCoolantTemp')
        }
    }
```

---

## 4. Mobile Application Integration

### 4.1 Driver Mobile App

**Key Features:**
- Route navigation
- Job/delivery management
- Electronic proof of delivery (ePOD)
- Hours of service logging
- Vehicle inspection reports
- Communication with dispatch

**SDK Integration:**
```javascript
import { FleetManagementSDK } from '@wia/fleet-mobile-sdk';

const sdk = new FleetManagementSDK({
  apiKey: 'your-api-key',
  environment: 'production'
});

// Driver authentication
await sdk.auth.login({
  driverId: 'DRV-001',
  pin: '1234'
});

// Get assigned routes
const routes = await sdk.routes.getAssigned();

// Update route progress
await sdk.routes.updateStop({
  routeId: 'ROUTE-001',
  stopId: 'STOP-005',
  status: 'completed',
  arrivalTime: new Date(),
  departureTime: new Date(),
  signature: signatureData,
  photos: photoData
});

// Submit HOS logs
await sdk.hos.logDutyStatus({
  status: 'off_duty',
  location: currentLocation,
  odometer: currentOdometer
});
```

### 4.2 Manager Dashboard App

**Key Features:**
- Real-time fleet tracking
- Performance analytics
- Alert management
- Route optimization
- Driver communication

**React Native Example:**
```javascript
import React, { useEffect, useState } from 'react';
import { MapView, Marker } from 'react-native-maps';

function FleetMap({ fleetId }) {
  const [vehicles, setVehicles] = useState([]);
  
  useEffect(() => {
    // Subscribe to vehicle updates
    const subscription = sdk.tracking.subscribeToFleet(fleetId, {
      onUpdate: (vehicleUpdate) => {
        setVehicles(prev => {
          const index = prev.findIndex(v => v.id === vehicleUpdate.vehicleId);
          if (index >= 0) {
            prev[index] = vehicleUpdate;
            return [...prev];
          } else {
            return [...prev, vehicleUpdate];
          }
        });
      }
    });
    
    return () => subscription.unsubscribe();
  }, [fleetId]);
  
  return (
    <MapView>
      {vehicles.map(vehicle => (
        <Marker
          key={vehicle.id}
          coordinate={{
            latitude: vehicle.location.latitude,
            longitude: vehicle.location.longitude
          }}
          title={vehicle.id}
          description={`Speed: ${vehicle.speed} km/h`}
        />
      ))}
    </MapView>
  );
}
```

---

## 5. Third-Party Service Integration

### 5.1 Mapping & Routing Services

**Google Maps Platform:**
```python
import googlemaps

gmaps = googlemaps.Client(key='your-api-key')

# Get route with traffic
directions = gmaps.directions(
    origin=(37.7749, -122.4194),
    destination=(34.0522, -118.2437),
    departure_time='now',
    traffic_model='best_guess'
)

# Geocode address
geocode_result = gmaps.geocode('1600 Amphitheatre Parkway, Mountain View, CA')

# Distance matrix for route optimization
distance_matrix = gmaps.distance_matrix(
    origins=origin_waypoints,
    destinations=destination_waypoints,
    mode='driving',
    departure_time='now'
)
```

### 5.2 Weather Services

**OpenWeatherMap Integration:**
```python
import requests

def get_route_weather(waypoints):
    weather_data = []
    for waypoint in waypoints:
        response = requests.get(
            'https://api.openweathermap.org/data/2.5/weather',
            params={
                'lat': waypoint['latitude'],
                'lon': waypoint['longitude'],
                'appid': 'your-api-key',
                'units': 'metric'
            }
        )
        weather_data.append(response.json())
    return weather_data
```

### 5.3 Payment Processing

**Fuel Card Integration:**
```python
def process_fuel_transaction(transaction):
    # Validate transaction
    if not validate_card(transaction['cardNumber']):
        return {'status': 'declined', 'reason': 'Invalid card'}
    
    # Check vehicle authorization
    vehicle = get_vehicle_by_card(transaction['cardNumber'])
    if not vehicle:
        return {'status': 'declined', 'reason': 'Card not assigned'}
    
    # Process payment
    payment_result = payment_gateway.charge(
        amount=transaction['amount'],
        card=transaction['cardNumber']
    )
    
    # Record in fleet system
    if payment_result['status'] == 'success':
        record_fuel_purchase({
            'vehicleId': vehicle['id'],
            'amount': transaction['amount'],
            'quantity': transaction['quantity'],
            'location': transaction['station'],
            'timestamp': transaction['timestamp']
        })
    
    return payment_result
```

---

## 6. Data Warehouse Integration

### 6.1 ETL Process

**Extract:**
```sql
-- Extract daily trip data
SELECT 
    trip_id,
    vehicle_id,
    driver_id,
    start_time,
    end_time,
    distance,
    fuel_consumed,
    avg_speed
FROM trips
WHERE DATE(start_time) = CURRENT_DATE - INTERVAL '1 day'
```

**Transform:**
```python
def transform_trip_data(raw_trips):
    transformed = []
    for trip in raw_trips:
        transformed.append({
            'trip_key': generate_surrogate_key(trip['trip_id']),
            'date_key': get_date_key(trip['start_time']),
            'vehicle_key': get_vehicle_key(trip['vehicle_id']),
            'driver_key': get_driver_key(trip['driver_id']),
            'distance_km': trip['distance'],
            'duration_minutes': calculate_duration(trip['start_time'], trip['end_time']),
            'fuel_liters': trip['fuel_consumed'],
            'fuel_efficiency_kmpl': trip['distance'] / trip['fuel_consumed'],
            'avg_speed_kmh': trip['avg_speed']
        })
    return transformed
```

**Load:**
```python
def load_to_warehouse(transformed_data):
    warehouse_conn = get_warehouse_connection()
    
    # Bulk insert
    cursor = warehouse_conn.cursor()
    cursor.executemany("""
        INSERT INTO fact_trips (
            trip_key, date_key, vehicle_key, driver_key,
            distance_km, duration_minutes, fuel_liters,
            fuel_efficiency_kmpl, avg_speed_kmh
        ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s)
    """, transformed_data)
    
    warehouse_conn.commit()
```

### 6.2 Real-Time Streaming

**Apache Kafka Integration:**
```python
from kafka import KafkaProducer, KafkaConsumer
import json

# Producer (Fleet Management System)
producer = KafkaProducer(
    bootstrap_servers=['kafka:9092'],
    value_serializer=lambda v: json.dumps(v).encode('utf-8')
)

def publish_vehicle_event(event):
    producer.send('fleet.vehicle.events', value=event)

# Consumer (Data Warehouse)
consumer = KafkaConsumer(
    'fleet.vehicle.events',
    bootstrap_servers=['kafka:9092'],
    value_deserializer=lambda m: json.loads(m.decode('utf-8'))
)

for message in consumer:
    event = message.value
    process_and_store(event)
```

---

## 7. Business Intelligence Integration

### 7.1 Tableau Integration

**Web Data Connector:**
```javascript
(function() {
    var myConnector = tableau.makeConnector();
    
    myConnector.getSchema = function(schemaCallback) {
        var cols = [
            { id: "vehicle_id", dataType: tableau.dataType.string },
            { id: "date", dataType: tableau.dataType.date },
            { id: "distance", dataType: tableau.dataType.float },
            { id: "fuel_consumed", dataType: tableau.dataType.float },
            { id: "efficiency", dataType: tableau.dataType.float }
        ];
        
        var tableSchema = {
            id: "fleetTrips",
            alias: "Fleet Trip Data",
            columns: cols
        };
        
        schemaCallback([tableSchema]);
    };
    
    myConnector.getData = function(table, doneCallback) {
        fetch('https://api.fleet.com/v1/trips')
            .then(response => response.json())
            .then(data => {
                var tableData = data.trips.map(trip => ({
                    vehicle_id: trip.vehicleId,
                    date: trip.date,
                    distance: trip.distance,
                    fuel_consumed: trip.fuelConsumed,
                    efficiency: trip.distance / trip.fuelConsumed
                }));
                
                table.appendRows(tableData);
                doneCallback();
            });
    };
    
    tableau.registerConnector(myConnector);
})();
```

### 7.2 Power BI Integration

**Custom Connector:**
```powerquery
let
    Source = Web.Contents("https://api.fleet.com/v1/analytics", [
        Headers=[
            #"Authorization"="Bearer " & ApiKey,
            #"Content-Type"="application/json"
        ]
    ]),
    JsonData = Json.Document(Source),
    DataTable = Table.FromRecords(JsonData[data]),
    ExpandedTable = Table.ExpandRecordColumn(DataTable, "metrics", 
        {"distance", "fuelConsumed", "efficiency"})
in
    ExpandedTable
```

---

## 8. Security Best Practices

### 8.1 API Security Checklist

- ✅ Use HTTPS/TLS for all communications
- ✅ Implement OAuth 2.0 for authentication
- ✅ Use API keys for device authentication
- ✅ Implement rate limiting
- ✅ Validate and sanitize all inputs
- ✅ Use parameterized queries to prevent SQL injection
- ✅ Implement CORS policies
- ✅ Log all API access
- ✅ Encrypt sensitive data at rest and in transit
- ✅ Implement IP whitelisting for critical endpoints
- ✅ Use webhook signatures for verification
- ✅ Rotate credentials regularly

### 8.2 Data Privacy Compliance

- **GDPR:** Right to access, right to be forgotten, data portability
- **CCPA:** Data disclosure, opt-out mechanisms
- **Industry-specific:** DOT regulations, driver privacy laws

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*  
*© 2025 SmileStory Inc. / WIA*  
*MIT License*
