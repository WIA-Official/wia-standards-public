# Chapter 7: System Integration

## Learning Objectives

After completing this chapter, you will be able to:

1. Integrate WIA data with GIS platforms (ArcGIS, QGIS)
2. Connect to global biodiversity databases (GBIF, iNaturalist)
3. Deploy WIA systems on cloud platforms (AWS, Azure, Google Cloud)
4. Link with decision support tools (Marxan, InVEST)
5. Publish datasets with DOIs for scientific citation

---

## 7.1 GIS Platform Integration

### 7.1.1 QGIS Integration

**WIA QGIS Plugin:**

```python
# Install WIA plugin
# QGIS Plugins > Manage and Install Plugins > Search "WIA Ecosystem"

# Load WIA data layer
from qgis.core import QgsVectorLayer

# Connect to WIA API
wia_layer = QgsVectorLayer(
    "url='https://api.example.org/v1/observations?format=geojson&bbox=-125,40,-110,50' \
     type=geojson",
    "Eagle Observations",
    "ogr"
)

QgsProject.instance().addMapLayer(wia_layer)

# Style by validation status
from qgis.core import QgsSymbol, QgsCategorizedSymbolRenderer

categories = []
colors = {
    'expert_verified': '#2ecc71',  # Green
    'validated': '#3498db',         # Blue
    'questionable': '#f39c12',      # Orange
    'invalid': '#e74c3c'            # Red
}

for status, color in colors.items():
    symbol = QgsSymbol.defaultSymbol(wia_layer.geometryType())
    symbol.setColor(QColor(color))
    category = QgsRendererCategory(status, symbol, status.replace('_', ' ').title())
    categories.append(category)

renderer = QgsCategorizedSymbolRenderer('validation_status', categories)
wia_layer.setRenderer(renderer)
wia_layer.triggerRepaint()
```

**Processing Algorithms:**

```python
# Temporal density heatmap
processing.run("wia:temporal_density", {
    'INPUT': wia_layer,
    'TIME_FIELD': 'timestamp',
    'INTERVAL': '1 month',
    'OUTPUT': 'memory:density'
})

# Species richness grid
processing.run("wia:species_richness", {
    'INPUT': wia_layer,
    'TAXON_FIELD': 'scientific_name',
    'CELL_SIZE': 1000,  # meters
    'OUTPUT': 'richness.gpkg'
})
```

### 7.1.2 ArcGIS Integration

**ArcGIS REST Feature Service:**

```javascript
// Add WIA layer to web map
require([
  "esri/Map",
  "esri/views/MapView",
  "esri/layers/GeoJSONLayer"
], function(Map, MapView, GeoJSONLayer) {

  const wiaLayer = new GeoJSONLayer({
    url: "https://api.example.org/v1/observations?format=geojson",
    copyright: "WIA Ecosystem Monitoring",
    popupTemplate: {
      title: "{scientific_name}",
      content: `
        <b>Common Name:</b> {common_name}<br>
        <b>Date:</b> {timestamp}<br>
        <b>Observer:</b> {observer.name}<br>
        <b>Abundance:</b> {abundance}<br>
        <b>Status:</b> {validation_status}
      `
    },
    renderer: {
      type: "simple",
      symbol: {
        type: "simple-marker",
        size: 8,
        color: "blue",
        outline: { color: "white", width: 1 }
      }
    }
  });

  const map = new Map({
    basemap: "topo-vector",
    layers: [wiaLayer]
  });

  const view = new MapView({
    container: "viewDiv",
    map: map,
    center: [-122.4, 47.6],
    zoom: 10
  });
});
```

**ArcGIS Field Maps (Mobile Data Collection):**

```json
{
  "name": "Species Observation",
  "fields": [
    {
      "name": "scientific_name",
      "alias": "Scientific Name",
      "type": "esriFieldTypeString",
      "domain": {
        "type": "codedValue",
        "name": "CommonSpecies",
        "codedValues": [
          {"name": "Haliaeetus leucocephalus", "code": "HALL"},
          {"name": "Ursus arctos", "code": "URSAR"}
        ]
      }
    },
    {
      "name": "abundance",
      "alias": "Count",
      "type": "esriFieldTypeInteger"
    },
    {
      "name": "photo",
      "alias": "Photo",
      "type": "esriFieldTypeBlob"
    }
  ]
}
```

### 7.1.3 OGC Web Services

**Web Feature Service (WFS):**

```xml
<!-- GetCapabilities request -->
GET https://api.example.org/ogc/wfs?
  service=WFS&
  version=2.0.0&
  request=GetCapabilities

<!-- GetFeature request -->
GET https://api.example.org/ogc/wfs?
  service=WFS&
  version=2.0.0&
  request=GetFeature&
  typeName=wia:observations&
  bbox=-125,40,-110,50&
  outputFormat=application/json
```

**Web Map Service (WMS):**

```xml
<!-- GetMap request for visualization -->
GET https://api.example.org/ogc/wms?
  service=WMS&
  version=1.3.0&
  request=GetMap&
  layers=wia:species_density&
  bbox=-125,40,-110,50&
  width=800&
  height=600&
  crs=EPSG:4326&
  format=image/png
```

---

## 7.2 Conservation Database Integration

### 7.2.1 GBIF Integration

**Export to Darwin Core Archive:**

```python
from wia import Client
from dwca import DarwinCoreArchive

client = Client(api_key='YOUR_KEY')
observations = client.get_observations(
    start_date='2025-01-01',
    end_date='2025-12-31'
)

# Map WIA to Darwin Core
dwc_records = []
for obs in observations:
    dwc_records.append({
        'occurrenceID': obs['record_id'],
        'basisOfRecord': 'HumanObservation',
        'eventDate': obs['timestamp'],
        'decimalLatitude': obs['location']['latitude'],
        'decimalLongitude': obs['location']['longitude'],
        'scientificName': obs['taxon']['scientific_name'],
        'kingdom': obs['taxon']['kingdom'],
        'phylum': obs['taxon']['phylum'],
        'class': obs['taxon']['class'],
        'order': obs['taxon']['order'],
        'family': obs['taxon']['family'],
        'genus': obs['taxon']['genus'],
        'specificEpithet': obs['taxon']['species'],
        'individualCount': obs['abundance'],
        'samplingProtocol': obs['detection_method'],
        'recordedBy': obs['observer']['name'],
        'identificationQualifier': obs['quality']['validation_status'],
        'coordinateUncertaintyInMeters': obs['location'].get('precision'),
    })

# Create Darwin Core Archive
archive = DarwinCoreArchive()
archive.add_occurrence_core(dwc_records)
archive.add_eml_metadata({
    'title': 'Pacific Northwest Bird Observations 2025',
    'abstract': '...',
    'creators': [{'name': 'Jane Smith', 'email': 'jane@example.org'}]
})
archive.write('gbif_export.zip')

# Upload to GBIF IPT (Integrated Publishing Toolkit)
# https://www.gbif.org/ipt
```

### 7.2.2 iNaturalist Integration

**Bidirectional Sync:**

```python
import inaturalist

# Import research-grade observations from iNaturalist
observations = inaturalist.get_observations(
    taxon_id=3,  # Birds
    quality_grade='research',
    geo=True,
    bbox=(-125, 40, -110, 50),
    d1='2025-01-01',
    d2='2025-12-31'
)

# Convert to WIA format
wia_observations = []
for obs in observations:
    wia_obs = {
        'wia_version': '1.0',
        'schema_type': 'species-observation',
        'record_id': f'inat-{obs["id"]}',
        'timestamp': obs['observed_on'],
        'location': {
            'latitude': obs['latitude'],
            'longitude': obs['longitude'],
            'precision': obs['positional_accuracy']
        },
        'taxon': {
            'scientific_name': obs['taxon']['name'],
            'common_name': obs['taxon']['preferred_common_name'],
            'taxon_authority': 'iNaturalist Taxonomy'
        },
        'detection_method': 'visual_survey',
        'occurrence_status': 'present',
        'observer': {
            'id': f'inat-user-{obs["user"]["id"]}',
            'name': obs['user']['name']
        },
        'quality': {
            'validation_status': 'expert_verified',
            'confidence_level': 1.0 if obs['quality_grade'] == 'research' else 0.7
        }
    }
    wia_observations.append(wia_obs)

# Submit to WIA database
wia_client.submit_observations(wia_observations)

# Export WIA observations to iNaturalist
# (requires API write access and user consent)
for wia_obs in my_wia_observations:
    inaturalist.create_observation({
        'species_guess': wia_obs['taxon']['scientific_name'],
        'observed_on_string': wia_obs['timestamp'],
        'latitude': wia_obs['location']['latitude'],
        'longitude': wia_obs['location']['longitude'],
        'description': f"Imported from WIA monitoring program"
    })
```

### 7.2.3 eBird Integration

**Import eBird Checklists:**

```r
library(auk)
library(wiaR)

# Download eBird data for region
ebd <- auk_ebd("ebd_US-WA_202501_202512_rel2025.txt") %>%
  auk_bbox(c(-125, 40, -110, 50)) %>%
  auk_complete() %>%  # Only complete checklists
  auk_filter("ebd_filtered.txt")

# Read filtered data
observations <- read_ebd("ebd_filtered.txt")

# Convert to WIA format
wia_obs <- observations %>%
  mutate(
    wia_version = "1.0",
    schema_type = "species-observation",
    record_id = paste0("ebird-", checklist_id, "-", species_code),
    timestamp = observation_date,
    latitude = latitude,
    longitude = longitude,
    scientific_name = scientific_name,
    common_name = common_name,
    abundance = observation_count,
    detection_method = "visual_survey",
    protocol = protocol_type,
    effort_duration = duration_minutes,
    effort_distance = effort_distance_km
  )

# Upload to WIA system
wia_client <- wia_connect(api_key = Sys.getenv("WIA_API_KEY"))
wia_submit_observations(wia_client, wia_obs)
```

---

## 7.3 Cloud Platform Integration

### 7.3.1 AWS Deployment

**Architecture:**

```
AWS WIA Ecosystem Monitoring Stack:
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  ┌──────────────┐         ┌──────────────┐         ┌─────────────┐ │
│  │ IoT Core     │────────►│ Lambda       │────────►│ S3          │ │
│  │ (MQTT)       │         │ (Validation) │         │ (Raw data)  │ │
│  └──────────────┘         └──────────────┘         └─────────────┘ │
│         │                         │                                │
│         │                         ▼                                │
│         │                 ┌──────────────┐                         │
│         │                 │ Aurora       │                         │
│         │                 │ PostgreSQL   │                         │
│         │                 │ + PostGIS    │                         │
│         │                 └──────────────┘                         │
│         │                         │                                │
│         ▼                         ▼                                │
│  ┌──────────────┐         ┌──────────────┐                         │
│  │ SageMaker    │         │ API Gateway  │                         │
│  │ (Species ID) │         │ (REST API)   │                         │
│  └──────────────┘         └──────────────┘                         │
│                                   │                                │
│                                   ▼                                │
│                           ┌──────────────┐                         │
│                           │ CloudFront   │                         │
│                           │ (CDN)        │                         │
│                           └──────────────┘                         │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Terraform Deployment:**

```hcl
# RDS PostgreSQL with PostGIS
resource "aws_db_instance" "wia_db" {
  identifier        = "wia-ecosystem-db"
  engine            = "postgres"
  engine_version    = "15.3"
  instance_class    = "db.t3.medium"
  allocated_storage = 100

  db_name  = "ecosystem"
  username = var.db_username
  password = var.db_password

  vpc_security_group_ids = [aws_security_group.db_sg.id]
  db_subnet_group_name   = aws_db_subnet_group.main.name

  backup_retention_period = 7
  multi_az               = true

  tags = {
    Name = "WIA Ecosystem Monitoring DB"
  }
}

# Lambda for data validation
resource "aws_lambda_function" "validate_observation" {
  filename      = "validate.zip"
  function_name = "wia_validate_observation"
  role          = aws_iam_role.lambda_exec.arn
  handler       = "index.handler"
  runtime       = "python3.11"

  environment {
    variables = {
      DB_HOST = aws_db_instance.wia_db.endpoint
      DB_NAME = "ecosystem"
    }
  }
}

# API Gateway
resource "aws_apigatewayv2_api" "wia_api" {
  name          = "wia-ecosystem-api"
  protocol_type = "HTTP"

  cors_configuration {
    allow_origins = ["*"]
    allow_methods = ["GET", "POST", "PUT", "DELETE"]
    allow_headers = ["*"]
  }
}
```

### 7.3.2 Google Earth Engine Integration

**Extract Remote Sensing at Observation Points:**

```javascript
// Load WIA observations as FeatureCollection
var observations = ee.FeatureCollection('projects/wia/eagle_observations');

// Load Landsat imagery
var landsat = ee.ImageCollection('LANDSAT/LC08/C02/T1_L2')
  .filterBounds(observations)
  .filterDate('2025-01-01', '2025-12-31');

// Calculate NDVI
var addNDVI = function(image) {
  var ndvi = image.normalizedDifference(['SR_B5', 'SR_B4']).rename('NDVI');
  return image.addBands(ndvi);
};

var withNDVI = landsat.map(addNDVI);

// Extract NDVI at each observation point
var extractValues = function(feature) {
  var date = ee.Date(feature.get('timestamp'));

  var image = withNDVI
    .filterDate(date.advance(-7, 'day'), date.advance(7, 'day'))
    .first();

  var ndvi = image.select('NDVI').reduceRegion({
    reducer: ee.Reducer.mean(),
    geometry: feature.geometry(),
    scale: 30
  }).get('NDVI');

  return feature.set('ndvi', ndvi);
};

var enriched = observations.map(extractValues);

// Export enriched dataset
Export.table.toDrive({
  collection: enriched,
  description: 'eagle_observations_with_ndvi',
  fileFormat: 'CSV'
});
```

---

## 7.4 Decision Support Integration

### 7.4.1 Marxan (Conservation Planning)

**Workflow:**

```r
library(wiaR)
library(sf)
library(prioritizr)

# 1. Load WIA species observations
wia_client <- wia_connect(api_key = Sys.getenv("WIA_API_KEY"))
obs <- get_observations(wia_client, bbox = c(-125, 40, -110, 50))
obs_sf <- wia_to_sf(obs)

# 2. Create planning units (hexagons)
study_area <- st_bbox(obs_sf) %>% st_as_sfc()
pu <- st_make_grid(study_area, cellsize = 0.1, square = FALSE) %>%
  st_sf() %>%
  mutate(id = row_number(), cost = 1)

# 3. Calculate species richness per planning unit
species_by_pu <- obs_sf %>%
  st_join(pu) %>%
  group_by(id, scientific_name) %>%
  summarize(presence = 1, .groups = 'drop') %>%
  pivot_wider(names_from = scientific_name, values_from = presence, values_fill = 0)

# 4. Set conservation targets (20% of current distribution)
targets <- species_by_pu %>%
  st_drop_geometry() %>%
  summarize(across(-id, sum)) %>%
  pivot_longer(everything()) %>%
  mutate(target = value * 0.2)

# 5. Run Marxan-style optimization
prob <- problem(pu, species_by_pu %>% select(-id), cost_column = "cost") %>%
  add_min_set_objective() %>%
  add_relative_targets(0.2) %>%
  add_binary_decisions() %>%
  add_default_solver()

solution <- solve(prob)

# 6. Map solution
ggplot(solution) +
  geom_sf(aes(fill = factor(solution_1))) +
  scale_fill_manual(values = c("white", "darkgreen"),
                    labels = c("Not selected", "Selected")) +
  labs(fill = "Conservation\nPriority") +
  theme_minimal()
```

### 7.4.2 InVEST (Ecosystem Services)

**Habitat Quality Model Calibration:**

```python
from wia import Client
import natcap.invest.habitat_quality
import geopandas as gpd

# 1. Load WIA species observations
client = Client(api_key='YOUR_KEY')
obs = client.get_observations(
    taxon='Ursus arctos',  # Grizzly bear
    bbox=(-125, 40, -110, 50)
)
gdf = client.to_geodataframe(obs)

# 2. Create species density raster
from rasterio import features
from scipy.ndimage import gaussian_filter

# Convert points to density surface
density = create_kernel_density(gdf, cell_size=1000, bandwidth=5000)

# 3. Run InVEST Habitat Quality with WIA data as validation
args = {
    'workspace_dir': './invest_output',
    'lulc_cur_path': 'landcover.tif',
    'threats_table_path': 'threats.csv',
    'sensitivity_table_path': 'sensitivity.csv',
    'half_saturation_constant': 0.5,
    'results_suffix': 'grizzly'
}

natcap.invest.habitat_quality.execute(args)

# 4. Compare InVEST output with WIA observations
invest_quality = rasterio.open('./invest_output/quality_grizzly.tif')
predicted_quality = extract_raster_values(invest_quality, gdf)

# Correlation between predicted quality and observed abundance
correlation = pearsonr(predicted_quality, gdf['abundance'])
print(f"Model validation: r = {correlation.statistic:.3f}, p = {correlation.pvalue:.4f}")
```

---

## 7.5 Data Publication and Citation

### 7.5.1 DOI Assignment

**Workflow:**

```python
import datacite

# 1. Prepare dataset metadata
metadata = {
    'identifier': {'identifier': '10.1234/example', 'identifierType': 'DOI'},
    'creators': [
        {
            'name': 'Smith, Jane',
            'nameIdentifiers': [
                {'nameIdentifier': '0000-0002-1825-0097', 'nameIdentifierScheme': 'ORCID'}
            ],
            'affiliation': ['University of Washington']
        }
    ],
    'titles': [{'title': 'Green River Watershed Biodiversity Monitoring 2020-2025'}],
    'publisher': 'WIA Ecosystem Monitoring Repository',
    'publicationYear': '2025',
    'resourceType': {'resourceTypeGeneral': 'Dataset'},
    'descriptions': [
        {
            'description': 'Long-term monitoring of aquatic and terrestrial biodiversity...',
            'descriptionType': 'Abstract'
        }
    ],
    'geoLocations': [
        {
            'geoLocationBox': {
                'westBoundLongitude': -122.5,
                'eastBoundLongitude': -121.8,
                'southBoundLatitude': 47.3,
                'northBoundLatitude': 47.8
            }
        }
    ]
}

# 2. Mint DOI via DataCite
client = datacite.DataCiteClient(username='YOUR_REPO', password='PASSWORD')
doi = client.mint_doi(metadata)

# 3. Update dataset record with DOI
wia_client.update_dataset_metadata(
    dataset_id='green-river-2020-2025',
    doi=doi
)

# 4. Generate citation
citation = f"""선행 연구. Green River Watershed Biodiversity
Monitoring 2020-2025 [Dataset]. WIA Ecosystem Monitoring Repository.
https://doi.org/{doi}"""
```

### 7.5.2 Repository Integration

**Automated Deposition:**

```python
# DataONE
from d1_client import mnclient

mn_client = mnclient.MemberNodeClient('https://mn.example.org/mn')

# Package dataset
science_metadata = generate_eml(wia_dataset)
data_object = create_data_package(
    data_files=['observations.csv', 'sensors.csv'],
    metadata=science_metadata
)

# Upload to DataONE member node
pid = mn_client.create(
    pid='urn:uuid:' + str(uuid.uuid4()),
    obj=data_object,
    sysmeta=system_metadata
)

# Zenodo
from zenodo_client import Zenodo

zenodo = Zenodo(access_token='YOUR_TOKEN')
deposition = zenodo.create_deposition(
    title='Green River Watershed Monitoring',
    creators=[{'name': 'Smith, Jane', 'orcid': '0000-0002-1825-0097'}],
    description='Long-term biodiversity monitoring...',
    upload_type='dataset',
    access_right='open',
    license='cc-by-4.0'
)

# Upload files
zenodo.upload_file(deposition['id'], 'observations.csv')
zenodo.upload_file(deposition['id'], 'metadata.xml')

# Publish
zenodo.publish_deposition(deposition['id'])
```

---

## 7.6 Review Questions

### Question 1
Design a workflow to automatically sync new WIA observations to both GBIF and iNaturalist. How would you handle taxonomic name differences between platforms?

### Question 2
You're deploying a WIA system on AWS for a national monitoring program with 1000 sensors sending data every 5 minutes. Estimate monthly costs for IoT Core, RDS, S3, and Lambda.

### Question 3
Explain how to use WIA observation data to calibrate and validate an InVEST habitat quality model. What metrics would you use to assess model performance?

### Question 4
A researcher wants to extract climate variables from ERA5 reanalysis data at each of 10,000 bird observation points. Should this be done in Google Earth Engine, locally, or another approach?

### Question 5
Design a citation system for individual WIA observations (not just datasets). How would you generate unique, persistent identifiers and encourage proper attribution?

---

## 7.7 Key Takeaways

| Integration | Tools | Purpose |
|-------------|-------|---------|
| **GIS** | ArcGIS, QGIS, OGC services | Spatial analysis and visualization |
| **Biodiversity** | GBIF, iNaturalist, eBird | Global data sharing and discovery |
| **Cloud** | AWS, Azure, GEE | Scalable infrastructure and analysis |
| **Decision Support** | Marxan, InVEST, SMART | Conservation planning and management |
| **Publication** | DataCite, Zenodo, DataONE | Long-term archival and citation |

### Integration Patterns
- **Export**: WIA → GeoJSON → GIS platforms
- **Sync**: Bidirectional with iNaturalist, eBird
- **Enrichment**: Extract remote sensing at observation points
- **Validation**: WIA observations validate model predictions
- **Publication**: Automated deposition to repositories

### Next Chapter Preview

Chapter 8 provides implementation guidance, including deployment roadmaps, infrastructure requirements, testing procedures, and certification pathways for WIA-compliant systems.

---

© 2025 WIA Standards Committee. 弘익人間 (홍익인간) - Benefit All Humanity
