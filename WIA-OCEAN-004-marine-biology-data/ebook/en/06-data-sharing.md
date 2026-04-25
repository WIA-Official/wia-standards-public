# Chapter 6: Data Sharing and Interoperability

## Making Marine Biology Data Accessible to All

Marine biology data achieves its full value only when shared openly and integrated across sources. This chapter explores repositories, APIs, data standards, and best practices that enable researchers worldwide to discover, access, and combine marine biodiversity and environmental data.

### The Global Marine Data Infrastructure

Marine biology depends on a distributed network of data repositories and services:

```typescript
interface MarineDataRepository {
  name: string;
  organization: string;
  scope: "biodiversity" | "environmental" | "genomic" | "imagery" | "multi_domain";
  coverage: "global" | "regional" | "national";

  dataTypes: string[];
  recordCount: number;
  updateFrequency: "real_time" | "daily" | "monthly" | "irregular";

  access: {
    webPortal: string;           // URL
    api: APISpecification;
    bulkDownload: boolean;
    dataFormats: string[];       // ["CSV", "NetCDF", "JSON", "Darwin Core Archive"]
  };

  standards: {
    metadata: string[];          // ["ISO 19115", "Dublin Core", "FGDC"]
    data: string[];              // ["Darwin Core", "CF-NetCDF", "OBIS schema"]
    vocabularies: string[];      // ["WoRMS", "NERC P01", "BODC"]
  };

  licensing: {
    defaultLicense: string;      // "CC0", "CC-BY", "CC-BY-NC"
    commercialUseAllowed: boolean;
    attributionRequired: boolean;
  };

  services: {
    search: boolean;
    visualization: boolean;
    analysis: boolean;
    doi_minting: boolean;
  };
}

interface APISpecification {
  type: "REST" | "GraphQL" | "OGC_WFS" | "OGC_WCS" | "OPeNDAP";
  baseURL: string;
  version: string;
  authentication: "none" | "api_key" | "oauth";
  rateLimits: {
    requestsPerMinute: number;
    requestsPerDay: number;
  };
  documentation: string;         // URL to API docs
}
```

### Key Marine Data Repositories

#### OBIS (Ocean Biodiversity Information System)

```typescript
const obisRepository: MarineDataRepository = {
  name: "OBIS",
  organization: "IOC-UNESCO",
  scope: "biodiversity",
  coverage: "global",

  dataTypes: ["species_occurrences", "abundance", "biomass", "traits"],
  recordCount: 100000000,        // 100 million+ occurrence records
  updateFrequency: "daily",

  access: {
    webPortal: "https://obis.org",
    api: {
      type: "REST",
      baseURL: "https://api.obis.org/v3",
      version: "3.0",
      authentication: "none",
      rateLimits: {
        requestsPerMinute: 60,
        requestsPerDay: 10000
      },
      documentation: "https://api.obis.org/v3/documentation"
    },
    bulkDownload: true,
    dataFormats: ["CSV", "JSON", "Darwin Core Archive"]
  },

  standards: {
    metadata: ["ISO 19115", "EML"],
    data: ["Darwin Core", "OBIS schema"],
    vocabularies: ["WoRMS", "NERC P01"]
  },

  licensing: {
    defaultLicense: "CC0",
    commercialUseAllowed: true,
    attributionRequired: false   // But encouraged
  },

  services: {
    search: true,
    visualization: true,
    analysis: true,
    doi_minting: false
  }
};
```

#### OBIS API Client Implementation

```typescript
class OBISClient {
  private baseURL = "https://api.obis.org/v3";

  // Search for species occurrences
  async searchOccurrences(params: {
    scientificname?: string;
    aphiaID?: number;
    geometry?: string;           // WKT polygon
    startdate?: string;          // ISO 8601
    enddate?: string;
    mindepth?: number;
    maxdepth?: number;
    fields?: string[];
    size?: number;               // Records per page (max 10000)
    after?: string;              // Pagination cursor
  }): Promise<{
    total: number;
    results: OBISOccurrence[];
    after?: string;              // Next page cursor
  }> {
    const queryParams = new URLSearchParams();

    Object.entries(params).forEach(([key, value]) => {
      if (value !== undefined) {
        queryParams.append(key, String(value));
      }
    });

    const response = await fetch(`${this.baseURL}/occurrence?${queryParams}`);
    return response.json();
  }

  // Get species checklist for area
  async getChecklist(params: {
    geometry: string;            // WKT polygon or POINT
    taxonid?: number;            // Filter by higher taxon
    mindepth?: number;
    maxdepth?: number;
  }): Promise<{
    total: number;
    results: {
      taxonID: number;
      scientificName: string;
      taxonRank: string;
      records: number;
    }[];
  }> {
    const queryParams = new URLSearchParams();
    Object.entries(params).forEach(([key, value]) => {
      if (value !== undefined) {
        queryParams.append(key, String(value));
      }
    });

    const response = await fetch(`${this.baseURL}/checklist?${queryParams}`);
    return response.json();
  }

  // Get occurrence grid for mapping
  async getGrid(params: {
    geometry: string;
    gridsize: number;            // Cell size in meters
    taxonid?: number;
  }): Promise<{
    type: "FeatureCollection";
    features: {
      type: "Feature";
      geometry: {
        type: "Polygon";
        coordinates: number[][][];
      };
      properties: {
        records: number;
        species: number;
      };
    }[];
  }> {
    const queryParams = new URLSearchParams();
    Object.entries(params).forEach(([key, value]) => {
      queryParams.append(key, String(value));
    });

    const response = await fetch(`${this.baseURL}/occurrence/grid?${queryParams}`);
    return response.json();
  }

  // Get statistics
  async getStatistics(params: {
    taxonid?: number;
    geometry?: string;
  }): Promise<{
    records: number;
    species: number;
    datasets: number;
    minDate: string;
    maxDate: string;
    minDepth: number;
    maxDepth: number;
  }> {
    const queryParams = new URLSearchParams();
    Object.entries(params).forEach(([key, value]) => {
      if (value !== undefined) {
        queryParams.append(key, String(value));
      }
    });

    const response = await fetch(`${this.baseURL}/statistics?${queryParams}`);
    return response.json();
  }
}

interface OBISOccurrence {
  occurrenceID: string;
  scientificName: string;
  scientificNameID: string;      // WoRMS LSID
  taxonRank: string;
  kingdom: string;
  phylum: string;
  class: string;
  order: string;
  family: string;
  genus: string;

  decimalLatitude: number;
  decimalLongitude: number;
  coordinateUncertaintyInMeters?: number;
  minimumDepthInMeters?: number;
  maximumDepthInMeters?: number;

  eventDate: string;             // ISO 8601
  year: number;
  month?: number;
  day?: number;

  basisOfRecord: string;         // "HumanObservation", "PreservedSpecimen", etc.
  individualCount?: number;
  organismQuantity?: number;
  organismQuantityType?: string; // "individuals", "biomass", etc.

  institutionCode: string;
  collectionCode?: string;
  catalogNumber?: string;
  recordedBy?: string;
  identifiedBy?: string;

  datasetID: string;
  datasetName: string;

  flags: string[];               // Quality flags
  absence: boolean;              // true for absence records
}

// Example usage
const obis = new OBISClient();

// Find all coral grouper observations in Hawaii
const hawaiiGroupers = await obis.searchOccurrences({
  scientificname: "Plectropomus leopardus",
  geometry: "POLYGON((-160 18, -154 18, -154 22, -160 22, -160 18))",
  size: 5000
});

console.log(`Found ${hawaiiGroupers.total} records`);

// Get species checklist for coral reef area
const checklist = await obis.getChecklist({
  geometry: "POINT(-156.5 20.8)",
  mindepth: 0,
  maxdepth: 30
});

console.log(`${checklist.results.length} species found in area`);

// Get gridded occurrence data for mapping
const grid = await obis.getGrid({
  geometry: "POLYGON((-160 18, -154 18, -154 22, -160 22, -160 18))",
  gridsize: 10000,               // 10km cells
  taxonid: 125732                // Serranidae (grouper family)
});
```

### Other Major Repositories

#### GenBank/ENA/DDBJ (INSDC)

```typescript
const genBankRepository: MarineDataRepository = {
  name: "GenBank",
  organization: "NCBI (National Center for Biotechnology Information)",
  scope: "genomic",
  coverage: "global",

  dataTypes: ["DNA_sequences", "RNA_sequences", "protein_sequences", "genomes"],
  recordCount: 250000000,        // 250+ million sequences
  updateFrequency: "daily",

  access: {
    webPortal: "https://www.ncbi.nlm.nih.gov/genbank/",
    api: {
      type: "REST",
      baseURL: "https://eutils.ncbi.nlm.nih.gov/entrez/eutils",
      version: "2.0",
      authentication: "api_key",
      rateLimits: {
        requestsPerMinute: 10,   // 3 without API key
        requestsPerDay: 100000
      },
      documentation: "https://www.ncbi.nlm.nih.gov/books/NBK25501/"
    },
    bulkDownload: true,
    dataFormats: ["GenBank", "FASTA", "XML", "JSON"]
  },

  standards: {
    metadata: ["MIxS", "BioSample", "BioProject"],
    data: ["GenBank format", "INSDC feature table"],
    vocabularies: ["NCBI Taxonomy", "Sequence Ontology"]
  },

  licensing: {
    defaultLicense: "Public Domain",
    commercialUseAllowed: true,
    attributionRequired: false
  },

  services: {
    search: true,
    visualization: true,
    analysis: true,
    doi_minting: false
  }
};

class NCBIClient {
  private baseURL = "https://eutils.ncbi.nlm.nih.gov/entrez/eutils";
  private apiKey: string;

  constructor(apiKey: string) {
    this.apiKey = apiKey;
  }

  // Search for sequences
  async searchSequences(params: {
    organism?: string;
    gene?: string;
    keyword?: string;
    retmax?: number;
  }): Promise<{
    count: number;
    idList: string[];            // GenBank accession numbers
  }> {
    const query = this.buildQuery(params);
    const url = `${this.baseURL}/esearch.fcgi?db=nuccore&term=${encodeURIComponent(query)}&retmode=json&retmax=${params.retmax || 100}&api_key=${this.apiKey}`;

    const response = await fetch(url);
    const data = await response.json();

    return {
      count: parseInt(data.esearchresult.count),
      idList: data.esearchresult.idlist
    };
  }

  // Fetch sequence records
  async fetchSequences(ids: string[]): Promise<{
    accession: string;
    sequence: string;
    organism: string;
    gene: string;
    length: number;
  }[]> {
    const url = `${this.baseURL}/efetch.fcgi?db=nuccore&id=${ids.join(',')}&rettype=fasta&retmode=text&api_key=${this.apiKey}`;

    const response = await fetch(url);
    const fasta = await response.text();

    return this.parseFASTA(fasta);
  }

  private buildQuery(params: any): string {
    const parts: string[] = [];

    if (params.organism) {
      parts.push(`${params.organism}[Organism]`);
    }
    if (params.gene) {
      parts.push(`${params.gene}[Gene Name]`);
    }
    if (params.keyword) {
      parts.push(params.keyword);
    }

    return parts.join(' AND ');
  }

  private parseFASTA(fasta: string): any[] {
    // Simplified FASTA parser
    const records: any[] = [];
    const entries = fasta.split('>').filter(e => e.trim());

    entries.forEach(entry => {
      const lines = entry.split('\n');
      const header = lines[0];
      const sequence = lines.slice(1).join('').replace(/\s/g, '');

      const accMatch = header.match(/^(\S+)/);
      const accession = accMatch ? accMatch[1] : '';

      records.push({
        accession,
        sequence,
        organism: '',              // Would parse from header
        gene: '',
        length: sequence.length
      });
    });

    return records;
  }
}
```

#### Environmental Data Repositories

```typescript
const ergodapRepository: MarineDataRepository = {
  name: "ERDDAP",
  organization: "NOAA / Multiple institutions",
  scope: "environmental",
  coverage: "global",

  dataTypes: ["CTD", "satellite", "model", "buoy", "glider"],
  recordCount: 0,                // Variable (serves many datasets)
  updateFrequency: "real_time",

  access: {
    webPortal: "https://coastwatch.pfeg.noaa.gov/erddap/index.html",
    api: {
      type: "OPeNDAP",
      baseURL: "https://coastwatch.pfeg.noaa.gov/erddap",
      version: "2.23",
      authentication: "none",
      rateLimits: {
        requestsPerMinute: 60,
        requestsPerDay: 100000
      },
      documentation: "https://coastwatch.pfeg.noaa.gov/erddap/rest.html"
    },
    bulkDownload: true,
    dataFormats: ["NetCDF", "CSV", "JSON", "GeoJSON", "KML", "ODV"]
  },

  standards: {
    metadata: ["ISO 19115", "ACDD"],
    data: ["CF-NetCDF"],
    vocabularies: ["NERC P01", "GCMD"]
  },

  licensing: {
    defaultLicense: "varies",
    commercialUseAllowed: true,
    attributionRequired: true
  },

  services: {
    search: true,
    visualization: true,
    analysis: true,
    doi_minting: false
  }
};

class ERDDAPClient {
  private baseURL: string;

  constructor(serverURL: string = "https://coastwatch.pfeg.noaa.gov/erddap") {
    this.baseURL = serverURL;
  }

  // Search for datasets
  async searchDatasets(searchTerm: string): Promise<{
    datasetID: string;
    title: string;
    summary: string;
    institution: string;
  }[]> {
    const url = `${this.baseURL}/search/index.json?page=1&itemsPerPage=1000&searchFor=${encodeURIComponent(searchTerm)}`;

    const response = await fetch(url);
    const data = await response.json();

    // Parse ERDDAP search response
    const table = data.table;
    const datasets: any[] = [];

    // Column indices
    const idIdx = table.columnNames.indexOf('Dataset ID');
    const titleIdx = table.columnNames.indexOf('Title');
    const summaryIdx = table.columnNames.indexOf('Summary');
    const instIdx = table.columnNames.indexOf('Institution');

    table.rows.forEach((row: any[]) => {
      datasets.push({
        datasetID: row[idIdx],
        title: row[titleIdx],
        summary: row[summaryIdx],
        institution: row[instIdx]
      });
    });

    return datasets;
  }

  // Get data in various formats
  async getData(params: {
    datasetID: string;
    variables?: string[];
    constraints?: Record<string, [number, number]>;  // min, max
    format?: "json" | "csv" | "nc" | "geojson";
  }): Promise<any> {
    const { datasetID, variables, constraints, format = "json" } = params;

    let url = `${this.baseURL}/griddap/${datasetID}.${format}?`;

    // Add variables
    if (variables) {
      url += variables.join(',');
    }

    // Add constraints (e.g., time[min:max], latitude[min:max])
    if (constraints) {
      const constraintStr = Object.entries(constraints)
        .map(([var_name, [min, max]]) => `&${var_name}>=${min}&${var_name}<=${max}`)
        .join('');
      url += constraintStr;
    }

    const response = await fetch(url);

    if (format === "json") {
      return response.json();
    } else if (format === "csv") {
      return response.text();
    } else {
      return response.blob();
    }
  }
}

// Example: Get satellite chlorophyll data
const erddap = new ERDDAPClient();

const datasets = await erddap.searchDatasets("chlorophyll satellite");
console.log(`Found ${datasets.length} chlorophyll datasets`);

const chlorophyllData = await erddap.getData({
  datasetID: "erdMH1chla1day",   // MODIS Aqua 1-day chlorophyll
  variables: ["chlorophyll"],
  constraints: {
    time: [Date.parse("2024-01-01"), Date.parse("2024-01-31")],
    latitude: [20, 22],
    longitude: [-158, -156]
  },
  format: "json"
});
```

### Data Integration Workflows

```typescript
interface DataIntegrationPipeline {
  sources: {
    repository: string;
    dataType: string;
    queryParameters: any;
  }[];

  transformations: {
    step: string;
    operation: "harmonize_taxonomy" | "unit_conversion" | "temporal_align" | "spatial_join";
    parameters: any;
  }[];

  output: {
    format: string;
    destination: string;
    license: string;
  };
}

class MarineDataIntegrator {
  private obis: OBISClient;
  private ncbi: NCBIClient;
  private erddap: ERDDAPClient;

  constructor(ncbiApiKey: string) {
    this.obis = new OBISClient();
    this.ncbi = new NCBIClient(ncbiApiKey);
    this.erddap = new ERDDAPClient();
  }

  // Integrate species occurrences with environmental data
  async integrateOccurrencesEnvironment(params: {
    species: string;
    region: string;              // WKT polygon
    startDate: Date;
    endDate: Date;
  }): Promise<{
    occurrenceID: string;
    scientificName: string;
    latitude: number;
    longitude: number;
    date: Date;
    temperature?: number;
    salinity?: number;
    chlorophyll?: number;
  }[]> {
    // 1. Get species occurrences from OBIS
    const occurrences = await this.obis.searchOccurrences({
      scientificname: params.species,
      geometry: params.region,
      startdate: params.startDate.toISOString(),
      enddate: params.endDate.toISOString(),
      size: 10000
    });

    // 2. For each occurrence, fetch environmental data
    const integrated = await Promise.all(
      occurrences.results.map(async (occ) => {
        // Query ERDDAP for environmental data at occurrence location/time
        const env = await this.getEnvironmentalData({
          latitude: occ.decimalLatitude,
          longitude: occ.decimalLongitude,
          date: new Date(occ.eventDate)
        });

        return {
          occurrenceID: occ.occurrenceID,
          scientificName: occ.scientificName,
          latitude: occ.decimalLatitude,
          longitude: occ.decimalLongitude,
          date: new Date(occ.eventDate),
          temperature: env?.temperature,
          salinity: env?.salinity,
          chlorophyll: env?.chlorophyll
        };
      })
    );

    return integrated;
  }

  private async getEnvironmentalData(params: {
    latitude: number;
    longitude: number;
    date: Date;
  }): Promise<{
    temperature?: number;
    salinity?: number;
    chlorophyll?: number;
  } | null> {
    try {
      // Query environmental data (simplified)
      // In reality, would query specific ERDDAP datasets

      return {
        temperature: 25.5,        // Placeholder
        salinity: 35.2,
        chlorophyll: 0.8
      };
    } catch (error) {
      return null;
    }
  }
}
```

### Best Practices for Data Sharing

**1. Use Persistent Identifiers**
Assign DOIs to datasets for permanent citation.

**2. Comprehensive Metadata**
Include who, what, where, when, why, and how.

**3. Open Licenses**
Use CC0 or CC-BY for maximum reusability.

**4. Standard Formats**
Darwin Core for biodiversity, CF-NetCDF for gridded data.

**5. Quality Documentation**
Document QC procedures, known issues, limitations.

**6. Versioning**
Track dataset versions; never delete old versions.

**7. APIs for Programmatic Access**
Enable automated data discovery and retrieval.

**8. Interoperability**
Link to authoritative taxonomies (WoRMS), vocabularies (NERC).

### Philosophy: Data as a Public Good

Marine biology data represents a global commons, freely shared following 弘益人間. The OBIS database alone contains 100 million+ observations contributed by thousands of researchers worldwide, all freely accessible without restrictions.

When you share data openly, you:
- Enable discoveries you never imagined
- Maximize return on research investment
- Accelerate scientific progress globally
- Support conservation and sustainable management
- Honor the contributions of all who helped

Data sharing is not just best practice - it's our responsibility to humanity and the ocean.

---

**Next Chapter:** Conservation Applications - using marine biology data to protect endangered species and ecosystems.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
