# Chapter 6: Census Data System Integration

## Comprehensive Integration Patterns for Census Infrastructure

### 6.1 Integration Architecture Overview

The WIA-CENSUS-DATA standard defines comprehensive integration patterns for connecting census systems with administrative data sources, geographic information systems, and external statistical platforms. This chapter provides detailed guidance on achieving seamless data interoperability.

```typescript
// Census Integration Architecture
interface CensusIntegrationArchitecture {
  version: '1.0.0';

  integrationLayers: {
    dataIntegration: {
      description: 'Combining data from multiple sources';
      patterns: ['ETL', 'ELT', 'CDC', 'Virtual Integration'];
      technologies: ['Apache Spark', 'Talend', 'Informatica'];
    };
    applicationIntegration: {
      description: 'Connecting census applications';
      patterns: ['API Gateway', 'Message Queue', 'Service Mesh'];
      technologies: ['Kong', 'RabbitMQ', 'Istio'];
    };
    processIntegration: {
      description: 'Orchestrating census workflows';
      patterns: ['Workflow Engine', 'Event-Driven', 'Choreography'];
      technologies: ['Apache Airflow', 'Temporal', 'Camunda'];
    };
  };

  integrationDomains: {
    administrativeData: {
      sources: ['Civil registry', 'Tax records', 'Social security', 'Education'];
      frequency: 'Continuous to annual';
      challenges: ['Privacy', 'Quality', 'Linkage'];
    };
    geospatial: {
      sources: ['Address registers', 'Cadastral', 'Satellite imagery'];
      frequency: 'Monthly to annual';
      challenges: ['Coordinate systems', 'Temporal alignment'];
    };
    statisticalSystems: {
      sources: ['Survey systems', 'Business registers', 'Economic statistics'];
      frequency: 'Monthly to annual';
      challenges: ['Harmonization', 'Confidentiality'];
    };
  };
}

// Integration Hub Implementation
class CensusIntegrationHub {
  private connectors: Map<string, DataConnector>;
  private transformers: Map<string, DataTransformer>;
  private orchestrator: WorkflowOrchestrator;

  constructor(config: IntegrationConfig) {
    this.connectors = this.initializeConnectors(config.sources);
    this.transformers = this.initializeTransformers(config.transformations);
    this.orchestrator = new WorkflowOrchestrator(config.workflows);
  }

  async integrateSource(
    sourceId: string,
    options: IntegrationOptions
  ): Promise<IntegrationResult> {
    const connector = this.connectors.get(sourceId);
    if (!connector) {
      throw new Error(`Unknown source: ${sourceId}`);
    }

    // Extract data from source
    const rawData = await connector.extract(options.extractionParams);

    // Transform data
    const transformer = this.transformers.get(sourceId);
    const transformedData = await transformer?.transform(rawData, {
      targetSchema: options.targetSchema,
      mappings: options.mappings
    });

    // Load to target
    const loadResult = await this.loadData(
      transformedData || rawData,
      options.target
    );

    return {
      sourceId,
      recordsExtracted: rawData.recordCount,
      recordsLoaded: loadResult.recordCount,
      errors: loadResult.errors,
      warnings: loadResult.warnings,
      completedAt: new Date().toISOString()
    };
  }

  async runIntegrationPipeline(
    pipelineId: string
  ): Promise<PipelineResult> {
    const pipeline = await this.orchestrator.getPipeline(pipelineId);

    const execution = await this.orchestrator.startExecution(pipeline);

    // Monitor execution
    return new Promise((resolve, reject) => {
      execution.on('completed', (result) => resolve(result));
      execution.on('failed', (error) => reject(error));
    });
  }
}
```

### 6.2 Administrative Data Integration

```typescript
// Administrative Data Integration Framework
interface AdministrativeDataIntegration {
  sourceTypes: {
    civilRegistry: {
      dataElements: ['Births', 'Deaths', 'Marriages', 'Divorces'];
      updateFrequency: 'Continuous';
      linkageKey: 'Personal identifier';
      qualityConsiderations: [
        'Registration coverage',
        'Timeliness of registration',
        'Data completeness'
      ];
    };
    taxRecords: {
      dataElements: ['Income', 'Employment', 'Address'];
      updateFrequency: 'Annual';
      linkageKey: 'Tax identifier';
      qualityConsiderations: [
        'Coverage of tax filers',
        'Income accuracy',
        'Address currency'
      ];
    };
    socialSecurity: {
      dataElements: ['Benefits', 'Contributions', 'Employment history'];
      updateFrequency: 'Monthly';
      linkageKey: 'Social security number';
      qualityConsiderations: [
        'Coverage of workforce',
        'Informal sector gaps'
      ];
    };
    educationRecords: {
      dataElements: ['Enrollment', 'Attainment', 'Institution'];
      updateFrequency: 'Annual';
      linkageKey: 'Student identifier or personal ID';
      qualityConsiderations: [
        'Private school coverage',
        'Historical records'
      ];
    };
    healthRecords: {
      dataElements: ['Service utilization', 'Vaccinations', 'Diagnoses'];
      updateFrequency: 'Continuous';
      linkageKey: 'Patient identifier';
      confidentiality: 'HIGHEST';
    };
  };
}

// Administrative Data Connector
class AdministrativeDataConnector implements DataConnector {
  private sourceConfig: AdminSourceConfig;
  private linkageService: RecordLinkageService;
  private qualityService: DataQualityService;

  constructor(config: AdminSourceConfig) {
    this.sourceConfig = config;
    this.linkageService = new RecordLinkageService(config.linkageParams);
    this.qualityService = new DataQualityService(config.qualityRules);
  }

  async extract(params: ExtractionParams): Promise<ExtractedData> {
    // Connect to administrative source
    const connection = await this.connect();

    try {
      // Build extraction query
      const query = this.buildExtractionQuery(params);

      // Execute extraction
      const rawRecords = await connection.query(query);

      // Apply quality checks
      const qualityResults = await this.qualityService.assess(rawRecords);

      // Transform to standard format
      const standardizedRecords = await this.standardize(rawRecords);

      return {
        records: standardizedRecords,
        recordCount: standardizedRecords.length,
        qualityMetrics: qualityResults,
        extractionTimestamp: new Date().toISOString()
      };
    } finally {
      await connection.close();
    }
  }

  async linkToBase(
    adminRecords: AdminRecord[],
    basePopulation: PopulationRecord[]
  ): Promise<LinkageResult> {
    // Prepare records for linkage
    const preparedAdmin = this.prepareForLinkage(adminRecords);
    const preparedBase = this.prepareForLinkage(basePopulation);

    // Perform deterministic linkage first
    const deterministicMatches = await this.linkageService.deterministicLink(
      preparedAdmin,
      preparedBase,
      this.sourceConfig.deterministicKeys
    );

    // Probabilistic linkage for unmatched
    const unmatched = preparedAdmin.filter(
      r => !deterministicMatches.has(r.id)
    );

    const probabilisticMatches = await this.linkageService.probabilisticLink(
      unmatched,
      preparedBase,
      this.sourceConfig.probabilisticConfig
    );

    // Combine results
    const allMatches = new Map([
      ...deterministicMatches,
      ...probabilisticMatches
    ]);

    return {
      totalRecords: adminRecords.length,
      matched: allMatches.size,
      unmatched: adminRecords.length - allMatches.size,
      matchRate: allMatches.size / adminRecords.length,
      matches: Array.from(allMatches.entries()).map(([adminId, baseId]) => ({
        adminRecordId: adminId,
        baseRecordId: baseId,
        linkageType: deterministicMatches.has(adminId) ? 'DETERMINISTIC' : 'PROBABILISTIC'
      }))
    };
  }

  private prepareForLinkage(records: any[]): LinkageRecord[] {
    return records.map(record => ({
      id: record.id,
      standardizedName: this.standardizeName(record.name),
      standardizedAddress: this.standardizeAddress(record.address),
      dateOfBirth: record.dateOfBirth,
      gender: record.gender,
      phoneticName: this.generatePhonetic(record.name)
    }));
  }

  private standardizeName(name: string): string {
    // Remove titles, standardize case, handle special characters
    return name
      .toLowerCase()
      .replace(/^(mr|mrs|ms|dr|prof)\.?\s+/i, '')
      .replace(/[^\w\s]/g, '')
      .trim();
  }
}

// Record Linkage Service
class RecordLinkageService {
  private blockingStrategy: BlockingStrategy;
  private matchingAlgorithm: MatchingAlgorithm;
  private thresholds: LinkageThresholds;

  async probabilisticLink(
    sourceRecords: LinkageRecord[],
    targetRecords: LinkageRecord[],
    config: ProbabilisticConfig
  ): Promise<Map<string, string>> {
    const matches = new Map<string, string>();

    // Create blocking groups
    const blocks = await this.blockingStrategy.createBlocks(
      sourceRecords,
      targetRecords,
      config.blockingVariables
    );

    // Compare within blocks
    for (const block of blocks) {
      for (const sourceRecord of block.sourceRecords) {
        let bestMatch: { targetId: string; score: number } | null = null;

        for (const targetRecord of block.targetRecords) {
          const score = await this.matchingAlgorithm.compare(
            sourceRecord,
            targetRecord,
            config.comparisonFields
          );

          if (score > this.thresholds.match) {
            if (!bestMatch || score > bestMatch.score) {
              bestMatch = { targetId: targetRecord.id, score };
            }
          }
        }

        if (bestMatch) {
          matches.set(sourceRecord.id, bestMatch.targetId);
        }
      }
    }

    return matches;
  }
}

// Matching Algorithm Implementation
class FellegiSunterMatcher implements MatchingAlgorithm {
  private mWeights: Map<string, number>; // Agreement weights
  private uWeights: Map<string, number>; // Disagreement weights

  async compare(
    record1: LinkageRecord,
    record2: LinkageRecord,
    fields: ComparisonField[]
  ): Promise<number> {
    let totalWeight = 0;

    for (const field of fields) {
      const value1 = record1[field.name];
      const value2 = record2[field.name];

      const similarity = await this.calculateSimilarity(
        value1,
        value2,
        field.comparator
      );

      // Calculate weight contribution
      const mWeight = this.mWeights.get(field.name) || 0.9;
      const uWeight = this.uWeights.get(field.name) || 0.1;

      if (similarity > field.threshold) {
        // Agreement
        totalWeight += Math.log2(mWeight / uWeight);
      } else {
        // Disagreement
        totalWeight += Math.log2((1 - mWeight) / (1 - uWeight));
      }
    }

    // Convert to probability
    return 1 / (1 + Math.pow(2, -totalWeight));
  }

  private async calculateSimilarity(
    value1: string,
    value2: string,
    comparator: string
  ): Promise<number> {
    switch (comparator) {
      case 'EXACT':
        return value1 === value2 ? 1 : 0;

      case 'JARO_WINKLER':
        return this.jaroWinkler(value1, value2);

      case 'LEVENSHTEIN':
        const maxLen = Math.max(value1.length, value2.length);
        const distance = this.levenshtein(value1, value2);
        return 1 - distance / maxLen;

      case 'SOUNDEX':
        return this.soundex(value1) === this.soundex(value2) ? 1 : 0;

      default:
        return value1 === value2 ? 1 : 0;
    }
  }

  private jaroWinkler(s1: string, s2: string): number {
    // Jaro-Winkler string similarity
    const jaro = this.jaroSimilarity(s1, s2);
    const prefixLength = this.commonPrefixLength(s1, s2, 4);
    return jaro + prefixLength * 0.1 * (1 - jaro);
  }
}
```

### 6.3 Geographic Information System Integration

```typescript
// GIS Integration Framework
interface GISIntegration {
  integrationPoints: {
    addressGeocoding: {
      description: 'Convert addresses to coordinates and census geographies';
      inputFormat: 'Structured or unstructured address';
      outputFormat: 'Coordinates + geographic codes';
      quality: 'Match rate, positional accuracy';
    };
    boundaryServices: {
      description: 'Access census geographic boundaries';
      formats: ['GeoJSON', 'TopoJSON', 'Shapefile', 'GeoPackage'];
      operations: ['Point-in-polygon', 'Intersection', 'Buffer'];
    };
    spatialAnalysis: {
      description: 'Perform spatial analysis on census data';
      capabilities: ['Clustering', 'Interpolation', 'Network analysis'];
    };
    mapping: {
      description: 'Visualize census data on maps';
      styles: ['Choropleth', 'Dot density', 'Cartogram'];
    };
  };
}

// GIS Integration Service
class GISIntegrationService {
  private geocoder: GeocodingService;
  private boundaryStore: BoundaryStore;
  private spatialEngine: SpatialEngine;

  async geocodeAndAssign(
    addresses: Address[]
  ): Promise<GeocodingResult[]> {
    const results: GeocodingResult[] = [];

    for (const address of addresses) {
      // Geocode address
      const geocodeResult = await this.geocoder.geocode(address);

      if (geocodeResult.matched) {
        // Assign to census geographies
        const geographies = await this.assignToGeographies(
          geocodeResult.coordinates
        );

        results.push({
          inputAddress: address,
          matchStatus: 'MATCHED',
          coordinates: geocodeResult.coordinates,
          matchScore: geocodeResult.score,
          matchedAddress: geocodeResult.matchedAddress,
          geographies
        });
      } else {
        results.push({
          inputAddress: address,
          matchStatus: 'UNMATCHED',
          suggestions: geocodeResult.suggestions
        });
      }
    }

    return results;
  }

  private async assignToGeographies(
    coordinates: Coordinates
  ): Promise<GeographyAssignment> {
    const point = {
      type: 'Point',
      coordinates: [coordinates.longitude, coordinates.latitude]
    };

    // Query boundaries at all levels
    const assignments: { [level: string]: GeographyInfo } = {};

    const levels = ['nation', 'state', 'county', 'tract', 'blockGroup', 'block'];

    for (const level of levels) {
      const boundary = await this.boundaryStore.findContaining(point, level);

      if (boundary) {
        assignments[level] = {
          code: boundary.geoCode,
          name: boundary.name
        };
      }
    }

    return {
      coordinates,
      assignments,
      timestamp: new Date().toISOString()
    };
  }

  async performSpatialJoin(
    censusData: CensusDataset,
    externalLayer: GeoJSONFeatureCollection,
    joinType: 'INTERSECTS' | 'WITHIN' | 'CONTAINS'
  ): Promise<SpatialJoinResult> {
    const results: JoinedRecord[] = [];

    for (const censusRecord of censusData.records) {
      const censusGeometry = await this.getGeometry(censusRecord.geoCode);

      const matchingFeatures = externalLayer.features.filter(feature => {
        switch (joinType) {
          case 'INTERSECTS':
            return this.spatialEngine.intersects(
              censusGeometry,
              feature.geometry
            );
          case 'WITHIN':
            return this.spatialEngine.within(
              censusGeometry,
              feature.geometry
            );
          case 'CONTAINS':
            return this.spatialEngine.contains(
              censusGeometry,
              feature.geometry
            );
        }
      });

      results.push({
        censusRecord,
        matchedFeatures: matchingFeatures,
        matchCount: matchingFeatures.length
      });
    }

    return {
      totalRecords: censusData.records.length,
      matchedRecords: results.filter(r => r.matchCount > 0).length,
      results
    };
  }

  async aggregateToCustomGeography(
    censusData: CensusDataset,
    customBoundaries: GeoJSONFeatureCollection,
    aggregationMethod: AggregationMethod
  ): Promise<AggregatedDataset> {
    const aggregatedData: AggregatedRecord[] = [];

    for (const boundary of customBoundaries.features) {
      // Find census units that intersect with custom boundary
      const intersectingUnits = await this.findIntersectingUnits(
        boundary.geometry,
        censusData.geographicLevel
      );

      // Calculate area-weighted or population-weighted values
      const aggregatedValues = await this.aggregateValues(
        censusData,
        intersectingUnits,
        aggregationMethod
      );

      aggregatedData.push({
        customGeoId: boundary.properties.id,
        customGeoName: boundary.properties.name,
        values: aggregatedValues,
        sourceUnits: intersectingUnits.length,
        quality: this.assessAggregationQuality(intersectingUnits)
      });
    }

    return {
      customGeography: customBoundaries.properties?.name,
      records: aggregatedData,
      methodology: aggregationMethod,
      qualityNotes: this.generateQualityNotes(aggregatedData)
    };
  }
}

// Spatial Engine Implementation
class SpatialEngine {
  private turf: typeof import('@turf/turf');

  intersects(geom1: GeoJSON.Geometry, geom2: GeoJSON.Geometry): boolean {
    return this.turf.booleanIntersects(geom1, geom2);
  }

  within(geom1: GeoJSON.Geometry, geom2: GeoJSON.Geometry): boolean {
    return this.turf.booleanWithin(geom1, geom2);
  }

  contains(geom1: GeoJSON.Geometry, geom2: GeoJSON.Geometry): boolean {
    return this.turf.booleanContains(geom1, geom2);
  }

  calculateOverlapArea(
    geom1: GeoJSON.Geometry,
    geom2: GeoJSON.Geometry
  ): number {
    const intersection = this.turf.intersect(
      this.turf.feature(geom1),
      this.turf.feature(geom2)
    );

    if (!intersection) return 0;

    return this.turf.area(intersection);
  }

  buffer(
    geometry: GeoJSON.Geometry,
    distance: number,
    units: 'meters' | 'kilometers'
  ): GeoJSON.Geometry {
    return this.turf.buffer(
      this.turf.feature(geometry),
      distance,
      { units }
    ).geometry;
  }

  centroid(geometry: GeoJSON.Geometry): GeoJSON.Point {
    return this.turf.centroid(this.turf.feature(geometry)).geometry;
  }
}
```

### 6.4 Statistical System Integration

```typescript
// Statistical System Integration Framework
interface StatisticalSystemIntegration {
  systems: {
    nationalAccounts: {
      dataExchange: ['GDP components', 'Sector accounts'];
      integration: 'Population denominators, employment data';
      frequency: 'Quarterly, annual';
    };
    laborForceStatistics: {
      dataExchange: ['Employment', 'Unemployment', 'Participation'];
      integration: 'Benchmark to census counts';
      frequency: 'Monthly';
    };
    householdSurveys: {
      dataExchange: ['Income', 'Expenditure', 'Living conditions'];
      integration: 'Sampling frame from census';
      frequency: 'Annual to multi-year';
    };
    businessRegister: {
      dataExchange: ['Enterprise counts', 'Employment'];
      integration: 'Employer identification for job holders';
      frequency: 'Continuous';
    };
  };

  harmonizationChallenges: {
    conceptualDifferences: 'Definition variations across systems';
    temporalMisalignment: 'Different reference periods';
    coverageDifferences: 'Different populations covered';
    classificationDifferences: 'Industry, occupation coding';
  };
}

// Statistical System Integration Service
class StatisticalSystemIntegrationService {
  private sdmxConnector: SDMXConnector;
  private harmonizationEngine: HarmonizationEngine;
  private validationService: CrossSystemValidationService;

  async importFromSDMX(
    dataflowId: string,
    filters: SDMXQueryFilter
  ): Promise<ImportResult> {
    // Connect to SDMX data source
    const connection = await this.sdmxConnector.connect();

    try {
      // Get data structure definition
      const dsd = await connection.getDataStructure(dataflowId);

      // Build and execute query
      const query = this.buildSDMXQuery(dataflowId, filters, dsd);
      const sdmxData = await connection.query(query);

      // Transform to internal format
      const internalData = await this.transformSDMXData(sdmxData, dsd);

      // Validate import
      const validation = await this.validationService.validateImport(
        internalData,
        dataflowId
      );

      return {
        dataflowId,
        recordsImported: internalData.length,
        referenceDate: filters.timePeriod,
        validation,
        importTimestamp: new Date().toISOString()
      };
    } finally {
      await connection.close();
    }
  }

  async exportToSDMX(
    censusData: CensusDataset,
    dataflowId: string
  ): Promise<SDMXDataMessage> {
    // Get target data structure
    const dsd = await this.sdmxConnector.getDataStructure(dataflowId);

    // Map census data to SDMX structure
    const mappedData = await this.mapToSDMX(censusData, dsd);

    // Build SDMX message
    const message: SDMXDataMessage = {
      header: {
        id: crypto.randomUUID(),
        prepared: new Date().toISOString(),
        sender: { id: 'CENSUS_BUREAU' },
        structure: {
          structureID: dsd.id,
          dimensionAtObservation: 'TIME_PERIOD'
        }
      },
      dataSets: [{
        dataSetID: `DS_${dataflowId}_${Date.now()}`,
        action: 'Replace',
        series: mappedData
      }]
    };

    return message;
  }

  async createSamplingFrame(
    surveySpecification: SurveySpec
  ): Promise<SamplingFrame> {
    // Extract population from census
    const population = await this.extractTargetPopulation(
      surveySpecification.universe
    );

    // Apply stratification
    const stratifiedFrame = await this.stratifyPopulation(
      population,
      surveySpecification.stratificationVariables
    );

    // Calculate design weights
    const framedPopulation = await this.calculateFrameWeights(
      stratifiedFrame,
      surveySpecification.designType
    );

    return {
      frameId: crypto.randomUUID(),
      createdAt: new Date().toISOString(),
      universe: surveySpecification.universe,
      totalUnits: framedPopulation.length,
      strata: this.summarizeStrata(framedPopulation),
      validityPeriod: surveySpecification.frameValidityPeriod
    };
  }

  async benchmarkSurvey(
    surveyData: SurveyDataset,
    censusBenchmarks: CensusBenchmarks
  ): Promise<BenchmarkedDataset> {
    const benchmarkedData: BenchmarkedRecord[] = [];

    // Calculate calibration weights
    for (const domain of censusBenchmarks.domains) {
      const surveyTotals = this.calculateSurveyTotals(surveyData, domain);
      const censusTotals = censusBenchmarks.totals[domain.id];

      // Calculate adjustment factors
      const adjustmentFactor = censusTotals / surveyTotals;

      // Apply to survey records
      for (const record of surveyData.records) {
        if (this.matchesDomain(record, domain)) {
          benchmarkedData.push({
            ...record,
            originalWeight: record.weight,
            calibratedWeight: record.weight * adjustmentFactor,
            benchmarkDomain: domain.id
          });
        }
      }
    }

    return {
      ...surveyData,
      records: benchmarkedData,
      benchmarkingApplied: true,
      benchmarkSource: censusBenchmarks.source,
      benchmarkDate: censusBenchmarks.referenceDate
    };
  }
}

// SDMX Connector Implementation
class SDMXConnector {
  private baseUrl: string;
  private httpClient: HttpClient;

  async getDataStructure(dataflowId: string): Promise<SDMXDataStructure> {
    const url = `${this.baseUrl}/datastructure/${dataflowId}`;
    const response = await this.httpClient.get(url, {
      headers: { 'Accept': 'application/vnd.sdmx.structure+json' }
    });

    return this.parseDataStructure(response.data);
  }

  async query(query: SDMXQuery): Promise<SDMXDataset> {
    const url = this.buildQueryUrl(query);
    const response = await this.httpClient.get(url, {
      headers: { 'Accept': 'application/vnd.sdmx.data+json' }
    });

    return this.parseDataset(response.data);
  }

  private buildQueryUrl(query: SDMXQuery): string {
    let url = `${this.baseUrl}/data/${query.dataflow}`;

    // Add dimension filters
    const dimFilters = query.dimensions
      .map(d => d.values.join('+'))
      .join('.');
    url += `/${dimFilters}`;

    // Add query parameters
    const params = new URLSearchParams();
    if (query.startPeriod) params.set('startPeriod', query.startPeriod);
    if (query.endPeriod) params.set('endPeriod', query.endPeriod);
    if (query.detail) params.set('detail', query.detail);

    return `${url}?${params.toString()}`;
  }
}
```

### 6.5 Legacy System Integration

```typescript
// Legacy System Integration Patterns
interface LegacySystemIntegration {
  commonScenarios: {
    mainframeData: {
      challenge: 'Extract data from COBOL/mainframe systems';
      patterns: ['File-based export', 'DB2 connector', 'Screen scraping'];
      modernization: 'Gradual migration with abstraction layer';
    };
    proprietaryFormats: {
      challenge: 'Handle non-standard data formats';
      patterns: ['Custom parsers', 'Format converters', 'ETL tools'];
      standardization: 'Convert to open formats';
    };
    batchProcessing: {
      challenge: 'Integrate with batch-oriented systems';
      patterns: ['Scheduled jobs', 'File polling', 'Event triggers'];
      modernization: 'Add real-time capability layer';
    };
  };
}

// Legacy Integration Adapter
class LegacySystemAdapter {
  private connectionPool: ConnectionPool;
  private formatConverter: FormatConverter;
  private scheduledTasks: ScheduledTaskManager;

  async setupMainframeConnection(
    config: MainframeConfig
  ): Promise<MainframeConnection> {
    return {
      connection: await this.connectionPool.getConnection(config),
      executeQuery: async (query: string) => {
        return this.executeMainframeQuery(query, config);
      },
      extractTable: async (tableName: string, options: ExtractionOptions) => {
        return this.extractMainframeTable(tableName, options, config);
      }
    };
  }

  private async executeMainframeQuery(
    query: string,
    config: MainframeConfig
  ): Promise<QueryResult> {
    const connection = await this.connectionPool.getConnection(config);

    try {
      // Convert SQL to mainframe-compatible format if needed
      const convertedQuery = this.convertQuery(query, config.dialect);

      // Execute with timeout
      const result = await Promise.race([
        connection.execute(convertedQuery),
        this.createTimeout(config.queryTimeout)
      ]);

      // Convert result to standard format
      return this.formatConverter.convertResult(
        result,
        config.outputFormat
      );
    } finally {
      await this.connectionPool.releaseConnection(connection);
    }
  }

  async setupFileWatcher(
    config: FileWatchConfig
  ): Promise<FileWatcher> {
    const watcher = new FileWatcher(config.directory, {
      patterns: config.filePatterns,
      pollInterval: config.pollInterval
    });

    watcher.on('newFile', async (filePath) => {
      // Process new file
      const data = await this.processLegacyFile(filePath, config);

      // Transform to standard format
      const standardData = await this.formatConverter.convert(
        data,
        config.sourceFormat,
        'JSON'
      );

      // Load to modern system
      await this.loadToModernSystem(standardData, config.target);

      // Archive original file
      await this.archiveFile(filePath, config.archiveDir);
    });

    return watcher;
  }

  async processLegacyFile(
    filePath: string,
    config: FileProcessConfig
  ): Promise<any[]> {
    const fileContent = await fs.readFile(filePath);

    switch (config.sourceFormat) {
      case 'FIXED_WIDTH':
        return this.parseFixedWidth(fileContent, config.fieldSpec);

      case 'EBCDIC':
        const asciiContent = this.convertEBCDIC(fileContent);
        return this.parseFixedWidth(asciiContent, config.fieldSpec);

      case 'PACKED_DECIMAL':
        return this.parsePackedDecimal(fileContent, config.fieldSpec);

      default:
        throw new Error(`Unsupported format: ${config.sourceFormat}`);
    }
  }

  private parseFixedWidth(
    content: Buffer,
    fieldSpec: FieldSpec[]
  ): any[] {
    const records: any[] = [];
    const lines = content.toString().split('\n');

    for (const line of lines) {
      if (line.trim() === '') continue;

      const record: any = {};

      for (const field of fieldSpec) {
        const value = line.substring(field.start, field.start + field.length);
        record[field.name] = this.parseFieldValue(value, field.type);
      }

      records.push(record);
    }

    return records;
  }

  private convertEBCDIC(buffer: Buffer): Buffer {
    // EBCDIC to ASCII conversion table
    const conversionTable = this.getEBCDICTable();
    const result = Buffer.alloc(buffer.length);

    for (let i = 0; i < buffer.length; i++) {
      result[i] = conversionTable[buffer[i]] || 0x20;
    }

    return result;
  }
}

// Format Converter
class FormatConverter {
  async convert(
    data: any,
    sourceFormat: string,
    targetFormat: string
  ): Promise<any> {
    // Parse source format
    const parsed = await this.parse(data, sourceFormat);

    // Convert to target format
    return this.serialize(parsed, targetFormat);
  }

  private async parse(data: any, format: string): Promise<any> {
    switch (format) {
      case 'CSV':
        return this.parseCSV(data);
      case 'XML':
        return this.parseXML(data);
      case 'FIXED_WIDTH':
        return data; // Already parsed
      default:
        return data;
    }
  }

  private serialize(data: any, format: string): any {
    switch (format) {
      case 'JSON':
        return JSON.stringify(data, null, 2);
      case 'CSV':
        return this.serializeCSV(data);
      case 'PARQUET':
        return this.serializeParquet(data);
      default:
        return data;
    }
  }
}
```

### 6.6 Integration Testing and Monitoring

```typescript
// Integration Testing Framework
class IntegrationTestFramework {
  private testRunner: TestRunner;
  private mockServices: MockServiceManager;
  private assertionLibrary: AssertionLibrary;

  async runIntegrationTests(
    testSuite: IntegrationTestSuite
  ): Promise<TestResults> {
    const results: TestResult[] = [];

    // Setup test environment
    await this.setupTestEnvironment(testSuite.setup);

    for (const test of testSuite.tests) {
      try {
        // Setup mocks if needed
        if (test.mocks) {
          await this.mockServices.setup(test.mocks);
        }

        // Run test
        const result = await this.testRunner.run(test);

        // Verify assertions
        for (const assertion of test.assertions) {
          const assertionResult = await this.assertionLibrary.verify(
            assertion,
            result
          );

          if (!assertionResult.passed) {
            throw new AssertionError(assertion, assertionResult);
          }
        }

        results.push({
          testId: test.id,
          status: 'PASSED',
          duration: result.duration
        });

      } catch (error) {
        results.push({
          testId: test.id,
          status: 'FAILED',
          error: (error as Error).message,
          stackTrace: (error as Error).stack
        });
      } finally {
        // Cleanup mocks
        await this.mockServices.cleanup();
      }
    }

    // Teardown test environment
    await this.teardownTestEnvironment();

    return {
      suite: testSuite.name,
      total: results.length,
      passed: results.filter(r => r.status === 'PASSED').length,
      failed: results.filter(r => r.status === 'FAILED').length,
      results
    };
  }
}

// Integration Monitoring Service
class IntegrationMonitoringService {
  private metricsCollector: MetricsCollector;
  private alertManager: AlertManager;
  private dashboardService: DashboardService;

  async monitorIntegration(
    integrationId: string
  ): Promise<MonitoringDashboard> {
    // Collect metrics
    const metrics = await this.metricsCollector.collect(integrationId);

    // Check thresholds
    const alerts = await this.checkThresholds(metrics);

    // Update dashboard
    await this.dashboardService.update(integrationId, metrics);

    // Send alerts if needed
    for (const alert of alerts) {
      await this.alertManager.send(alert);
    }

    return {
      integrationId,
      status: alerts.length > 0 ? 'DEGRADED' : 'HEALTHY',
      metrics,
      alerts,
      lastUpdated: new Date().toISOString()
    };
  }

  private async checkThresholds(
    metrics: IntegrationMetrics
  ): Promise<Alert[]> {
    const alerts: Alert[] = [];

    if (metrics.errorRate > 0.05) {
      alerts.push({
        level: 'WARNING',
        message: `Error rate ${(metrics.errorRate * 100).toFixed(2)}% exceeds threshold`,
        metric: 'errorRate'
      });
    }

    if (metrics.latencyP99 > 5000) {
      alerts.push({
        level: 'WARNING',
        message: `P99 latency ${metrics.latencyP99}ms exceeds threshold`,
        metric: 'latencyP99'
      });
    }

    if (metrics.throughput < metrics.expectedThroughput * 0.8) {
      alerts.push({
        level: 'ERROR',
        message: `Throughput ${metrics.throughput} below expected`,
        metric: 'throughput'
      });
    }

    return alerts;
  }
}
```

---

**WIA-CENSUS-DATA System Integration**
**Version**: 1.0.0
**Last Updated**: 2025
**License**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - Benefit All Humanity
