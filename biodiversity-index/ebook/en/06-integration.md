# Chapter 6: Integration Systems

## Phase 4: GIS, Conservation Planning, and Policy Reporting

### Connecting Biodiversity Data to Decision-Making Systems

---

## Overview

Phase 4 of the WIA Biodiversity Index Standard ensures that standardized biodiversity data flows seamlessly into decision-making systems. This chapter covers integration with GIS platforms, conservation planning tools, policy reporting frameworks, and environmental management systems.

---

## GIS Platform Integration

### ArcGIS Integration

The WIA ArcGIS Toolbox provides native tools for biodiversity analysis within the Esri ecosystem.

**System Requirements:**
- ArcGIS Pro 2.9+ or ArcMap 10.8+
- Python 3.7+ with arcpy
- WIA Biodiversity SDK installed
- Valid WIA API credentials

**Installation:**
```python
# From ArcGIS Pro Python environment
pip install wia-biodiversity-arcgis

# Or download toolbox from GitHub
# https://github.com/WIA-Official/arcgis-biodiversity-tools
```

**Available Tools:**

| Tool | Purpose | Inputs | Outputs |
|------|---------|--------|---------|
| Biodiversity Hotspots | Calculate and map diversity hotspots | Region polygon, index type | Feature class with hotspots |
| Species Distribution | Generate species distribution maps | Species name, occurrences | Raster probability surface |
| Temporal Trends | Visualize diversity trends | Dataset, time range | Charts and trend layers |
| Protected Area Assessment | Compare diversity in/out PAs | PA boundaries, occurrences | Assessment report |
| Habitat Suitability | Model species habitat preferences | Environmental layers, occurrences | Suitability raster |
| Connectivity Analysis | Identify wildlife corridors | Resistance surface, source patches | Corridor network |

**Example: Biodiversity Hotspot Analysis**

```python
import arcpy
from wia_biodiversity_arcgis import BiodiversityTools

# Initialize toolbox
tools = BiodiversityTools(api_key='your_api_key')

# Define analysis parameters
params = {
    'region': 'Amazon_Basin',
    'index_type': 'Shannon',
    'cell_size_km': 10,
    'threshold_percentile': 90,
    'output_fc': r'C:\GIS\Projects\Amazon\hotspots.gdb\biodiversity_hotspots'
}

# Run analysis
result = tools.identify_hotspots(**params)

print(f"Identified {result.hotspot_count} hotspots")
print(f"Total area: {result.total_area_km2:,.0f} km²")
print(f"Mean Shannon index: {result.mean_shannon:.2f}")

# Add to current map
aprx = arcpy.mp.ArcGISProject("CURRENT")
m = aprx.activeMap
m.addDataFromPath(result.output_path)
```

**Species Distribution Modeling:**

```python
from wia_biodiversity_arcgis import SDMTools

# Load occurrence data
occurrences = tools.load_occurrences(
    species='Panthera tigris',
    country='IN',
    quality_flag='validated'
)

# Define environmental predictors
predictors = [
    r'C:\GIS\Data\bioclim\bio01.tif',  # Annual mean temperature
    r'C:\GIS\Data\bioclim\bio12.tif',  # Annual precipitation
    r'C:\GIS\Data\landcover\forest_cover.tif',
    r'C:\GIS\Data\dem\elevation.tif'
]

# Run MaxEnt model
sdm = SDMTools()
model_result = sdm.run_maxent(
    occurrences=occurrences,
    predictors=predictors,
    output_dir=r'C:\GIS\Projects\Tiger_SDM',
    n_replicates=10,
    test_percentage=25
)

print(f"Model AUC: {model_result.auc:.3f}")
print(f"Suitable habitat area: {model_result.suitable_area_km2:,.0f} km²")
```

### QGIS Plugin

The WIA Biodiversity Browser plugin provides QGIS users with direct access to WIA data and analysis tools.

**Installation:**
1. Open QGIS Plugin Manager
2. Add repository: `https://plugins.qgis.org/plugins/wia-biodiversity`
3. Search for "WIA Biodiversity Browser"
4. Install and restart QGIS

**Features:**
- Query WIA API directly from QGIS interface
- Load occurrences as vector layers
- Calculate diversity indices for selected polygons
- Export to standard formats (GeoPackage, Shapefile, GeoJSON)
- Interactive species distribution viewer
- Real-time endangered species alerts

**Python API:**

```python
from qgis.core import QgsVectorLayer, QgsProject, QgsFeature
from wia_qgis import WIABiodiversity

# Initialize connection
wia = WIABiodiversity(api_key='your_api_key')

# Load occurrences as layer
layer = wia.load_occurrences(
    species='Panthera tigris',
    year=2025,
    country='IN',
    layer_name='Tiger Occurrences 2025'
)

# Add to project
QgsProject.instance().addMapLayer(layer)

# Calculate diversity for region
region_layer = QgsProject.instance().mapLayersByName('Study_Area')[0]

diversity_results = wia.calculate_diversity(
    occurrences_layer=layer,
    region_layer=region_layer,
    indices=['shannon', 'simpson', 'species_richness']
)

# Add results as attributes
for feature in region_layer.getFeatures():
    feature['shannon_index'] = diversity_results[feature.id()]['shannon']
    feature['species_richness'] = diversity_results[feature.id()]['richness']
    region_layer.updateFeature(feature)
```

### Google Earth Engine Integration

For large-scale analysis, WIA data can be integrated with Google Earth Engine.

```javascript
// Load WIA occurrence data as Earth Engine asset
var tigerOccurrences = ee.FeatureCollection('users/wia_official/occurrences/panthera_tigris_2025');

// Define study region
var india = ee.FeatureCollection('USDOS/LSIB_SIMPLE/2017')
    .filter(ee.Filter.eq('country_na', 'India'));

// Load environmental data
var forestCover = ee.Image('UMD/hansen/global_forest_change_2023_v1_11')
    .select('treecover2000');

// Calculate occurrence density
var occurrenceDensity = tigerOccurrences
    .reduceToImage(['occurrence_id'], ee.Reducer.count())
    .reproject({crs: 'EPSG:4326', scale: 10000});

// Correlation with forest cover
var correlation = forestCover.addBands(occurrenceDensity)
    .reduceRegion({
        reducer: ee.Reducer.pearsonsCorrelation(),
        geometry: india,
        scale: 10000,
        maxPixels: 1e9
    });

print('Forest-Tiger correlation:', correlation);
```

---

## Conservation Planning Integration

### Marxan Integration

Marxan is the leading software for systematic conservation planning. WIA data integrates seamlessly to define conservation targets.

**WIA-to-Marxan Converter:**

```python
from wia_biodiversity import BiodiversityAPI, MarxanConverter

# Initialize
client = BiodiversityAPI(api_key='your_api_key')
converter = MarxanConverter()

# Define planning region
region = client.spatial.define_region(
    polygon=[[-60.5, -3.5], [-60.0, -3.5], [-60.0, -3.0], [-60.5, -3.0], [-60.5, -3.5]],
    planning_unit_size_km2=100
)

# Load species data from WIA
species_data = client.species.list(
    region=region,
    iucn_status=['CR', 'EN', 'VU'],
    min_occurrences=5
)

# Generate Marxan input files
converter.generate_marxan_files(
    region=region,
    species=species_data,
    output_dir='marxan_inputs/',
    species_target=0.30,  # 30% representation target
    cost_metric='area',
    boundary_penalty=0.5
)

# Creates: spec.dat, puvspr.dat, pu.dat, bound.dat
```

**Generated Marxan Files:**

**spec.dat (Species targets):**
```
id	name	target	spf
1	Panthera_tigris	30	1000
2	Elephas_maximus	30	800
3	Rhinoceros_unicornis	30	1200
4	Prionailurus_bengalensis	20	400
```

**puvspr.dat (Planning unit vs species):**
```
species	pu	amount
1	101	0.15
1	102	0.23
1	105	0.08
2	101	0.42
```

**Running Marxan Analysis:**

```python
from wia_conservation import MarxanRunner

runner = MarxanRunner(
    input_dir='marxan_inputs/',
    output_dir='marxan_outputs/',
    num_runs=100,
    num_iterations=1000000
)

results = runner.run()

# Analyze solutions
best_solution = results.best_solution
selection_frequency = results.selection_frequency

print(f"Best solution cost: {best_solution.cost:,.0f}")
print(f"Number of planning units: {best_solution.num_selected}")
print(f"All targets met: {best_solution.all_targets_met}")
```

### Zonation Integration

Zonation performs spatial conservation prioritization using WIA species distribution data.

```python
from wia_biodiversity import ZonationConverter

converter = ZonationConverter()

# Define species list with conservation weights
species_list = [
    {'name': 'Panthera tigris', 'weight': 1.0, 'iucn': 'EN'},
    {'name': 'Elephas maximus', 'weight': 0.9, 'iucn': 'EN'},
    {'name': 'Rhinoceros unicornis', 'weight': 1.2, 'iucn': 'VU'},
    {'name': 'Prionailurus viverrinus', 'weight': 0.8, 'iucn': 'VU'}
]

# Generate species distribution rasters from occurrence data
converter.create_species_layers(
    species_list=species_list,
    resolution_m=1000,
    output_dir='zonation_inputs/',
    format='geotiff',
    sdm_method='maxent',  # Generate SDM from occurrences
    mask_layer='study_area.shp'
)

# Generate Zonation settings file
converter.create_settings_file(
    species_weights='iucn_based',  # Weight by IUCN status
    analysis_type='core_area_zonation',
    removal_rule='basic_core_area',
    warp_factor=1.0,
    edge_removal=1,
    output_file='zonation_settings.dat'
)
```

### Circuitscape Connectivity Analysis

Model wildlife movement corridors using WIA habitat data.

```python
from wia_conservation import ConnectivityAnalysis

# Load habitat suitability from WIA SDM
analysis = ConnectivityAnalysis(api_key='your_api_key')

# Define source patches (protected areas with tiger populations)
source_patches = analysis.load_protected_areas(
    region='IN',
    species='Panthera tigris',
    min_population=10
)

# Create resistance surface from land cover
resistance = analysis.create_resistance_surface(
    land_cover_source='esri_land_cover_2023',
    resistance_values={
        'forest': 1,
        'shrubland': 3,
        'grassland': 5,
        'cropland': 50,
        'urban': 1000,
        'water': 500
    }
)

# Run Circuitscape
corridors = analysis.run_circuitscape(
    sources=source_patches,
    resistance=resistance,
    output_dir='corridors/',
    mode='pairwise'
)

# Identify priority corridors
priority_corridors = corridors.filter(current_density > corridors.percentile(90))
print(f"Priority corridors identified: {len(priority_corridors)}")
```

---

## Policy Reporting Integration

### CBD (Convention on Biological Diversity)

**Automated Kunming-Montreal Framework Reporting:**

```python
from wia_biodiversity import ReportingFramework

reporter = ReportingFramework(api_key='your_api_key')

# Generate Target 3 report (30x30 Protected Areas)
target_3 = reporter.cbd.target_3_report(
    country='BR',
    reference_year=2022,
    target_year=2030
)

print(f"Current terrestrial PA coverage: {target_3.terrestrial_coverage:.1f}%")
print(f"Current marine PA coverage: {target_3.marine_coverage:.1f}%")
print(f"Gap to 30% terrestrial: {target_3.terrestrial_gap:.1f}%")
print(f"Recommended new PA hectares: {target_3.recommended_new_ha:,.0f}")

# Generate Target 4 report (Species Recovery)
target_4 = reporter.cbd.target_4_report(
    country='BR',
    species_group='all_assessed'
)

# Export as official PDF
reporter.export(
    [target_3, target_4],
    format='pdf',
    template='cbd_official',
    output_file='brazil_cbd_report_2025.pdf'
)
```

**Available CBD Targets:**

| Target | Description | WIA Indicators |
|--------|-------------|----------------|
| Target 3 | 30% protection by 2030 | PA coverage, species in PAs |
| Target 4 | Species recovery actions | Threatened species trends |
| Target 6 | Invasive species reduction | Invasive detections, spread |
| Target 7 | Pollution reduction | Water quality indices |
| Target 8 | Climate adaptation | Range shift detection |

### IPBES Assessment Support

Generate data for Intergovernmental Science-Policy Platform on Biodiversity assessments.

```python
# Nature's Contributions to People (NCP) Assessment
ncp_report = reporter.ipbes.ncp_assessment(
    region='Southeast_Asia',
    ncp_categories=[
        'food_feed',
        'materials_companionship',
        'pollination_seed_dispersal',
        'regulation_climate',
        'regulation_water'
    ]
)

# Drivers of Change Analysis
drivers = reporter.ipbes.drivers_analysis(
    region='Southeast_Asia',
    time_period=('2000-01-01', '2025-12-31'),
    drivers=['land_use_change', 'climate_change', 'overexploitation', 'invasive_species']
)

# Scenario Modeling Support
scenarios = reporter.ipbes.scenario_projections(
    region='Southeast_Asia',
    scenarios=['business_as_usual', 'sustainable_development', 'nature_positive'],
    indicators=['species_richness', 'ecosystem_integrity', 'red_list_index'],
    projection_years=[2030, 2050]
)
```

### National Reporting

**Automated National Biodiversity Report:**

```python
# Generate comprehensive national report
national_report = reporter.generate_national_report(
    country='KE',
    year=2025,
    sections=[
        'executive_summary',
        'state_and_trends',
        'threats_and_pressures',
        'conservation_actions',
        'policy_integration',
        'data_gaps'
    ],
    language='en',
    include_maps=True,
    include_charts=True
)

# Export options
national_report.export_pdf('kenya_nbsap_2025.pdf')
national_report.export_web('kenya_biodiversity_portal/')
national_report.export_data('kenya_indicators.json')
```

---

## Environmental Impact Assessment

### Baseline Assessment Module

```python
from wia_biodiversity import EIAModule

eia = EIAModule(api_key='your_api_key')

# Define project area and impact zones
project_footprint = {
    "type": "Polygon",
    "coordinates": [[[lon, lat] for lon, lat in project_boundary]]
}

impact_zones = {
    "direct_impact": project_footprint,
    "indirect_500m": eia.spatial.buffer(project_footprint, 500),
    "regional_5km": eia.spatial.buffer(project_footprint, 5000),
    "cumulative_25km": eia.spatial.buffer(project_footprint, 25000)
}

# Run baseline assessment
baseline = eia.baseline_assessment(
    zones=impact_zones,
    time_period=('2020-01-01', '2025-12-31'),
    taxa=['birds', 'mammals', 'amphibians', 'reptiles', 'plants'],
    data_sources=['wia', 'gbif', 'iucn', 'local_records']
)

# Baseline report contents
print(f"Species documented: {baseline.total_species}")
print(f"Threatened species: {baseline.threatened_species}")
print(f"Endemic species: {baseline.endemic_species}")
print(f"Critical habitats identified: {baseline.critical_habitat_count}")
```

**Baseline Report Components:**

| Section | Contents |
|---------|----------|
| Species inventory | Complete list with conservation status |
| Habitat assessment | Types, condition, fragmentation |
| Critical habitats | IFC PS6 screening results |
| Seasonal patterns | Migration, breeding activity |
| Data gaps | Taxa/areas with limited data |
| Survey recommendations | Additional surveys needed |

### Impact Prediction

```python
# Define project impacts
project_impacts = {
    'habitat_loss': {
        'tropical_forest': 35,  # hectares lost
        'wetland': 12,
        'grassland': 8
    },
    'noise_disturbance_radius_m': 800,
    'lighting_disturbance': True,
    'water_abstraction_l_day': 50000,
    'discharge_quality': 'treated_secondary',
    'construction_duration_months': 24
}

# Predict biodiversity impacts
impact_prediction = eia.predict_impacts(
    baseline=baseline,
    impacts=project_impacts,
    models=['habitat_loss', 'disturbance_buffer', 'population_viability']
)

# Results
print(f"Species at risk of local extinction: {impact_prediction.local_extinction_risk}")
print(f"Habitat loss significance: {impact_prediction.habitat_significance}")
print(f"Predicted species richness change: {impact_prediction.richness_change:.1f}%")
```

### Mitigation Recommendations

```python
# Generate mitigation hierarchy recommendations
mitigation = eia.recommend_mitigation(
    impacts=impact_prediction,
    objectives=[
        'no_net_loss',
        'threatened_species_protection',
        'ecosystem_function_maintenance'
    ],
    constraints={
        'budget_usd': 5000000,
        'available_offset_area_ha': 500
    }
)

# Mitigation measures by hierarchy
for measure in mitigation.measures:
    print(f"{measure.hierarchy_level}: {measure.description}")
    print(f"  - Effectiveness: {measure.effectiveness:.0%}")
    print(f"  - Cost: ${measure.cost:,.0f}")
    print(f"  - Species benefited: {measure.species_count}")
```

---

## Real-Time Alert Systems

### Endangered Species Detection Alerts

```python
from wia_biodiversity import AlertSystem

alerts = AlertSystem(api_key='your_api_key')

# Subscribe to critically endangered detections
subscription = alerts.subscribe(
    event_type='species_detection',
    filters={
        'iucn_status': ['CR', 'EN'],
        'regions': ['Southeast_Asia', 'South_Asia'],
        'detection_methods': ['camera_trap', 'edna', 'acoustic'],
        'confidence_threshold': 0.90
    },
    notification_channels=['email', 'webhook', 'sms'],
    webhook_url='https://myapp.org/biodiversity-alerts'
)

# Webhook handler example
@app.route('/biodiversity-alerts', methods=['POST'])
def handle_alert():
    data = request.json

    if data['species']['iucn_status'] == 'CR':
        # Critical species - immediate response
        patrol_system.dispatch_patrol(
            location=data['location'],
            priority='high',
            species=data['species']['scientific_name']
        )

        # Notify stakeholders
        notification_service.send_critical_alert(
            recipients=stakeholder_list,
            message=f"Critically endangered {data['species']['common_name']} detected!"
        )

    return {'status': 'processed'}
```

### Threshold Monitoring

```python
# Set up diversity threshold monitoring
threshold_monitor = alerts.threshold_monitor(
    metric='shannon_diversity',
    region='yellowstone_zone_a',
    calculation_frequency='monthly',
    baseline_period=('2020-01-01', '2024-12-31'),
    threshold_type='percent_change',
    threshold_value=-10,  # Alert if 10% decline
    consecutive_violations=2  # Must violate 2 consecutive periods
)

# Anomaly detection
anomaly_detector = alerts.anomaly_detector(
    metrics=['species_richness', 'occurrence_rate', 'detection_probability'],
    region='protected_area_network',
    detection_method='isolation_forest',
    sensitivity='high'
)
```

---

## Dashboard Integration

### Protected Area Dashboard

```javascript
// React component for PA biodiversity dashboard
import { useState, useEffect } from 'react';
import { WIAClient } from '@wia/biodiversity-sdk';
import { Map, HeatmapLayer } from 'mapbox-gl';

function ProtectedAreaDashboard({ paId }) {
    const [data, setData] = useState(null);
    const client = new WIAClient({ apiKey: process.env.WIA_API_KEY });

    useEffect(() => {
        async function loadData() {
            const pa = await client.protectedAreas.get(paId);
            const diversity = await client.indices.calculate({
                region: pa.geometry,
                indices: ['shannon', 'simpson', 'richness'],
                temporalResolution: 'monthly',
                years: 5
            });
            const species = await client.species.list({
                region: pa.geometry,
                includeOccurrences: true
            });

            setData({ pa, diversity, species });
        }
        loadData();
    }, [paId]);

    return (
        <div className="dashboard">
            <header>
                <h1>{data?.pa.name} Biodiversity Dashboard</h1>
                <p>IUCN Category: {data?.pa.iucn_category}</p>
            </header>

            <div className="metrics-grid">
                <MetricCard
                    title="Species Richness"
                    value={data?.diversity.richness.observed}
                    trend={data?.diversity.richness.trend}
                />
                <MetricCard
                    title="Shannon Index"
                    value={data?.diversity.shannon.value.toFixed(2)}
                    trend={data?.diversity.shannon.trend}
                />
                <MetricCard
                    title="Threatened Species"
                    value={data?.species.filter(s => ['CR','EN','VU'].includes(s.iucn)).length}
                />
            </div>

            <div className="map-container">
                <Map center={data?.pa.centroid} zoom={10}>
                    <HeatmapLayer
                        data={data?.species.flatMap(s => s.occurrences)}
                        property="observation_date"
                    />
                </Map>
            </div>

            <DiversityTrendChart data={data?.diversity.timeSeries} />
            <SpeciesTable species={data?.species} />
        </div>
    );
}
```

---

## Key Takeaways

1. **GIS integration** through ArcGIS toolbox, QGIS plugin, and Google Earth Engine
2. **Conservation planning** with Marxan, Zonation, and Circuitscape connectivity analysis
3. **Policy reporting** automation for CBD, IPBES, and national biodiversity strategies
4. **EIA support** including baseline assessment, impact prediction, and mitigation
5. **Real-time alerts** for endangered species detections and diversity threshold violations

## Review Questions

1. What tools are available in the WIA ArcGIS Toolbox?
2. How does WIA data integrate with Marxan for conservation planning?
3. What CBD targets can be automatically reported using the WIA framework?
4. Describe the components of a WIA-supported EIA baseline assessment.
5. How can real-time alerts be configured for endangered species detections?

---

**Next Chapter Preview:** Chapter 7 explores the mathematical foundations of diversity indices, including Shannon, Simpson, and richness estimators.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity · Preserve All Life
