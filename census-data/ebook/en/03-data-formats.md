# Chapter 3: Census Data Formats and Schema Specifications

## Comprehensive Data Modeling for Population Statistics

### 3.1 Core Data Model Architecture

The WIA-CENSUS-DATA standard defines a comprehensive data model that supports the full lifecycle of census data from collection through dissemination. This chapter provides detailed specifications for all data structures.

```typescript
// Core Census Data Model
interface CensusDataModel {
  version: '1.0.0';
  modelType: 'RELATIONAL_WITH_HIERARCHICAL';

  coreEntities: {
    person: PersonEntity;
    household: HouseholdEntity;
    dwelling: DwellingEntity;
    geographicUnit: GeographicEntity;
    enumeration: EnumerationEntity;
  };

  relationships: {
    personToHousehold: 'MANY_TO_ONE';
    householdToDwelling: 'MANY_TO_ONE';
    dwellingToGeography: 'MANY_TO_ONE';
    personToEnumeration: 'MANY_TO_ONE';
  };

  temporalModel: {
    referenceDate: 'Census day point-in-time';
    historicalTracking: 'Version-based with effective dates';
    longitudinalLinking: 'Persistent person identifiers';
  };
}

// Person Entity Definition
interface PersonEntity {
  identification: {
    personId: {
      type: 'UUID';
      description: 'Unique person identifier within census';
      persistence: 'Single census cycle';
      format: 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx';
    };
    linkedPersonId: {
      type: 'ENCRYPTED_HASH';
      description: 'Cross-census linkage key';
      encryption: 'One-way hash with salt';
      purpose: 'Longitudinal analysis only';
    };
  };

  demographics: {
    age: {
      type: 'INTEGER';
      range: [0, 125];
      derivedFrom: 'dateOfBirth';
      asOfDate: 'censusReferenceDate';
    };
    dateOfBirth: {
      type: 'DATE';
      format: 'YYYY-MM-DD';
      confidentiality: 'RESTRICTED';
      publishedAs: 'ageGroups';
    };
    sex: {
      type: 'ENUM';
      values: ['MALE', 'FEMALE', 'OTHER', 'NOT_STATED'];
      standard: 'UN Recommendations for Census';
    };
    genderIdentity: {
      type: 'ENUM';
      values: ['MAN', 'WOMAN', 'NON_BINARY', 'OTHER', 'NOT_STATED'];
      optional: true;
      introducedYear: 2021;
    };
  };

  familyRelationships: {
    relationshipToReferencePerson: {
      type: 'ENUM';
      values: [
        'REFERENCE_PERSON',
        'SPOUSE_PARTNER',
        'CHILD',
        'PARENT',
        'SIBLING',
        'GRANDCHILD',
        'GRANDPARENT',
        'OTHER_RELATIVE',
        'NON_RELATIVE',
        'LODGER',
        'NOT_STATED'
      ];
    };
    maritalStatus: {
      type: 'ENUM';
      values: [
        'NEVER_MARRIED',
        'MARRIED',
        'SEPARATED',
        'DIVORCED',
        'WIDOWED',
        'REGISTERED_PARTNERSHIP',
        'NOT_STATED'
      ];
    };
  };

  birthplaceAndCitizenship: {
    countryOfBirth: {
      type: 'ISO3166_ALPHA3';
      description: 'Country of birth code';
    };
    citizenship: {
      type: 'ARRAY<ISO3166_ALPHA3>';
      description: 'Current citizenship(s)';
    };
    yearOfImmigration: {
      type: 'INTEGER';
      range: [1900, 'currentYear'];
      conditional: 'foreignBorn';
    };
  };

  ethnicityCulture: {
    ethnicGroup: {
      type: 'ARRAY<CODE>';
      codeList: 'NATIONAL_ETHNICITY_CODES';
      multiple: true;
      maxSelections: 5;
    };
    indigenousStatus: {
      type: 'BOOLEAN';
      nationalDefinition: true;
    };
    languagesSpoken: {
      type: 'ARRAY<ISO639_3>';
      description: 'Languages spoken at home';
    };
    religion: {
      type: 'CODE';
      codeList: 'RELIGION_CODES';
      optional: true;
      sensitivityLevel: 'HIGH';
    };
  };

  education: {
    highestQualification: {
      type: 'CODE';
      codeList: 'ISCED_2011';
      levels: [0, 1, 2, 3, 4, 5, 6, 7, 8];
    };
    fieldOfStudy: {
      type: 'CODE';
      codeList: 'ISCED_F_2013';
      conditional: 'hasPostSecondary';
    };
    currentlyStudying: {
      type: 'BOOLEAN';
    };
    institutionType: {
      type: 'ENUM';
      values: ['PUBLIC', 'PRIVATE', 'NOT_STUDYING'];
    };
  };

  economicActivity: {
    laborForceStatus: {
      type: 'ENUM';
      values: [
        'EMPLOYED',
        'UNEMPLOYED_SEEKING',
        'UNEMPLOYED_NOT_SEEKING',
        'NOT_IN_LABOR_FORCE',
        'NOT_APPLICABLE'
      ];
      referenceWeek: 'Week before census';
    };
    occupation: {
      type: 'CODE';
      codeList: 'ISCO_08';
      digits: 4;
      conditional: 'employed';
    };
    industry: {
      type: 'CODE';
      codeList: 'ISIC_REV4';
      digits: 4;
      conditional: 'employed';
    };
    employmentStatus: {
      type: 'ENUM';
      values: [
        'EMPLOYEE',
        'EMPLOYER',
        'OWN_ACCOUNT_WORKER',
        'CONTRIBUTING_FAMILY_WORKER',
        'NOT_APPLICABLE'
      ];
    };
    hoursWorked: {
      type: 'INTEGER';
      range: [0, 168];
      description: 'Hours worked in reference week';
    };
    workplaceLocation: {
      type: 'GEOGRAPHIC_CODE';
      level: 'locality';
    };
  };

  disability: {
    functionalDifficulty: {
      type: 'OBJECT';
      domains: {
        seeing: DifficultyLevel;
        hearing: DifficultyLevel;
        walking: DifficultyLevel;
        cognition: DifficultyLevel;
        selfCare: DifficultyLevel;
        communication: DifficultyLevel;
      };
      standard: 'Washington Group Short Set';
    };
  };

  metadata: {
    responseMode: {
      type: 'ENUM';
      values: ['INTERNET', 'PAPER', 'PHONE', 'FACE_TO_FACE', 'ADMIN_DATA'];
    };
    responseQuality: {
      imputationFlags: 'ARRAY<VARIABLE_NAME>';
      editFlags: 'ARRAY<VARIABLE_NAME>';
      qualityScore: 'FLOAT [0-1]';
    };
    processingHistory: {
      type: 'ARRAY<ProcessingEvent>';
    };
  };
}

type DifficultyLevel = 'NO_DIFFICULTY' | 'SOME_DIFFICULTY' | 'A_LOT_OF_DIFFICULTY' | 'CANNOT_DO' | 'NOT_STATED';
```

### 3.2 Household and Dwelling Schemas

```typescript
// Household Entity Definition
interface HouseholdEntity {
  identification: {
    householdId: {
      type: 'UUID';
      description: 'Unique household identifier';
    };
    dwellingId: {
      type: 'UUID';
      foreignKey: 'dwelling.dwellingId';
    };
    householdSequence: {
      type: 'INTEGER';
      description: 'Sequence number within dwelling';
      range: [1, 99];
    };
  };

  composition: {
    householdType: {
      type: 'ENUM';
      values: [
        'ONE_PERSON',
        'COUPLE_NO_CHILDREN',
        'COUPLE_WITH_CHILDREN',
        'LONE_PARENT',
        'MULTI_FAMILY',
        'NON_FAMILY_GROUP',
        'OTHER'
      ];
    };
    householdSize: {
      type: 'INTEGER';
      range: [1, 99];
      derived: true;
    };
    familyNuclei: {
      type: 'ARRAY<FamilyNucleus>';
      description: 'Family units within household';
    };
    numberOfGenerations: {
      type: 'INTEGER';
      range: [1, 5];
      derived: true;
    };
  };

  economicCharacteristics: {
    totalIncome: {
      type: 'MONETARY';
      currency: 'LOCAL_CURRENCY';
      period: 'ANNUAL';
      confidentialityTreatment: 'RANGES';
    };
    incomeSource: {
      type: 'ARRAY<ENUM>';
      values: [
        'EMPLOYMENT',
        'SELF_EMPLOYMENT',
        'INVESTMENT',
        'PENSION',
        'SOCIAL_BENEFITS',
        'OTHER'
      ];
    };
    tenureStatus: {
      type: 'ENUM';
      values: [
        'OWNER_OUTRIGHT',
        'OWNER_MORTGAGE',
        'RENTER_PRIVATE',
        'RENTER_SOCIAL',
        'RENT_FREE',
        'OTHER'
      ];
    };
    rentAmount: {
      type: 'MONETARY';
      conditional: 'isRenter';
      period: 'MONTHLY';
    };
  };

  vehicles: {
    numberOfVehicles: {
      type: 'INTEGER';
      range: [0, 20];
    };
    vehicleTypes: {
      type: 'ARRAY<ENUM>';
      values: ['CAR', 'MOTORCYCLE', 'BICYCLE', 'NONE'];
    };
  };

  technology: {
    internetAccess: {
      type: 'BOOLEAN';
    };
    internetType: {
      type: 'ARRAY<ENUM>';
      values: ['BROADBAND', 'MOBILE', 'DIAL_UP', 'NONE'];
    };
    devices: {
      type: 'ARRAY<ENUM>';
      values: ['COMPUTER', 'TABLET', 'SMARTPHONE', 'NONE'];
    };
  };
}

// Family Nucleus Structure
interface FamilyNucleus {
  nucleusId: string;
  nucleusType: 'COUPLE_NO_CHILDREN' | 'COUPLE_WITH_CHILDREN' | 'LONE_PARENT';
  referencePersonId: string;
  partnerId?: string;
  childrenIds: string[];
}

// Dwelling Entity Definition
interface DwellingEntity {
  identification: {
    dwellingId: {
      type: 'UUID';
      description: 'Unique dwelling identifier';
    };
    addressId: {
      type: 'UUID';
      foreignKey: 'address.addressId';
    };
    geographicCode: {
      type: 'HIERARCHICAL_CODE';
      levels: ['nation', 'region', 'district', 'locality', 'block'];
    };
  };

  physicalCharacteristics: {
    dwellingType: {
      type: 'ENUM';
      values: [
        'DETACHED_HOUSE',
        'SEMI_DETACHED',
        'TOWNHOUSE',
        'APARTMENT_LOW_RISE',
        'APARTMENT_HIGH_RISE',
        'MOBILE_HOME',
        'COLLECTIVE_LIVING',
        'OTHER'
      ];
    };
    numberOfRooms: {
      type: 'INTEGER';
      range: [1, 99];
    };
    numberOfBedrooms: {
      type: 'INTEGER';
      range: [0, 50];
    };
    floorArea: {
      type: 'FLOAT';
      unit: 'SQUARE_METERS';
      range: [5, 10000];
    };
    yearBuilt: {
      type: 'INTEGER';
      range: [1500, 'currentYear'];
      publishedAs: 'periodRanges';
    };
    numberOfFloors: {
      type: 'INTEGER';
      range: [1, 200];
    };
  };

  utilities: {
    waterSupply: {
      type: 'ENUM';
      values: ['PIPED_INSIDE', 'PIPED_OUTSIDE', 'WELL', 'OTHER', 'NONE'];
    };
    sewerConnection: {
      type: 'ENUM';
      values: ['PUBLIC_SEWER', 'SEPTIC', 'OTHER', 'NONE'];
    };
    heatingType: {
      type: 'ENUM';
      values: ['CENTRAL', 'ELECTRIC', 'GAS', 'OIL', 'WOOD', 'NONE'];
    };
    coolingType: {
      type: 'ENUM';
      values: ['CENTRAL_AIR', 'ROOM_UNITS', 'EVAPORATIVE', 'NONE'];
    };
    electricitySource: {
      type: 'ENUM';
      values: ['GRID', 'SOLAR', 'GENERATOR', 'NONE'];
    };
  };

  condition: {
    overallCondition: {
      type: 'ENUM';
      values: ['EXCELLENT', 'GOOD', 'FAIR', 'POOR', 'DILAPIDATED'];
    };
    repairsNeeded: {
      type: 'ARRAY<ENUM>';
      values: ['ROOF', 'WALLS', 'FLOORS', 'PLUMBING', 'ELECTRICAL', 'NONE'];
    };
  };

  value: {
    estimatedValue: {
      type: 'MONETARY';
      conditional: 'ownerOccupied';
      confidentialityTreatment: 'RANGES';
    };
    monthlyRent: {
      type: 'MONETARY';
      conditional: 'rented';
    };
  };

  occupancyStatus: {
    status: {
      type: 'ENUM';
      values: [
        'OCCUPIED_USUAL_RESIDENTS',
        'OCCUPIED_TEMPORARY',
        'VACANT_FOR_SALE',
        'VACANT_FOR_RENT',
        'VACANT_SEASONAL',
        'VACANT_OTHER',
        'UNDER_CONSTRUCTION'
      ];
    };
    numberOfHouseholds: {
      type: 'INTEGER';
      range: [0, 99];
    };
    numberOfUsualResidents: {
      type: 'INTEGER';
      range: [0, 999];
    };
  };
}
```

### 3.3 Geographic Data Structures

```typescript
// Geographic Hierarchy Definition
interface GeographicHierarchy {
  version: '1.0.0';
  referenceYear: number;

  levels: {
    nation: {
      level: 0;
      codeFormat: 'ISO3166_ALPHA3';
      example: 'USA';
      count: 1;
    };
    region: {
      level: 1;
      codeFormat: 'ALPHA_NUMERIC_3';
      example: 'NE1';
      description: 'Major administrative division';
      typicalCount: '4-20';
    };
    state: {
      level: 2;
      codeFormat: 'ISO3166_2';
      example: 'US-CA';
      typicalCount: '10-100';
    };
    county: {
      level: 3;
      codeFormat: 'NUMERIC_5';
      example: '06037';
      typicalCount: '100-5000';
    };
    municipality: {
      level: 4;
      codeFormat: 'NUMERIC_7';
      description: 'City, town, village';
    };
    tract: {
      level: 5;
      codeFormat: 'NUMERIC_11';
      description: 'Census tract';
      typicalPopulation: '1200-8000';
    };
    blockGroup: {
      level: 6;
      codeFormat: 'NUMERIC_12';
      typicalPopulation: '600-3000';
    };
    block: {
      level: 7;
      codeFormat: 'NUMERIC_15';
      description: 'Smallest tabulation unit';
      typicalPopulation: '0-600';
    };
  };

  alternativeGeographies: {
    statisticalAreas: {
      metropolitanArea: 'Functional urban regions';
      urbanRuralClassification: 'Settlement type';
      economicRegion: 'Labor market areas';
    };
    gridBased: {
      standardGrid: '1km x 1km cells';
      fineGrid: '100m x 100m cells';
      coordinateSystem: 'WGS84';
    };
    postalGeography: {
      postalCode: 'Mail delivery areas';
      postalDistrict: 'Postal administration';
    };
  };
}

// Geographic Unit Schema
interface GeographicUnit {
  identification: {
    geoCode: string;
    geoLevel: number;
    geoName: string;
    geoNameAlternate: string[];
    parentGeoCode: string;
  };

  boundary: {
    type: 'GeoJSON';
    geometry: {
      type: 'Polygon' | 'MultiPolygon';
      coordinates: number[][][];
    };
    simplificationLevel: 'FULL' | 'STANDARD' | 'SIMPLIFIED';
    coordinateSystem: 'EPSG:4326';
  };

  centroid: {
    latitude: number;
    longitude: number;
    type: 'GEOMETRIC' | 'POPULATION_WEIGHTED';
  };

  characteristics: {
    landArea: {
      value: number;
      unit: 'SQUARE_KILOMETERS';
    };
    waterArea: {
      value: number;
      unit: 'SQUARE_KILOMETERS';
    };
    urbanRuralClass: {
      type: 'ENUM';
      values: ['URBAN_CORE', 'URBAN_FRINGE', 'PERI_URBAN', 'RURAL', 'REMOTE'];
    };
    coastalStatus: boolean;
    elevation: {
      min: number;
      max: number;
      mean: number;
      unit: 'METERS';
    };
  };

  hierarchy: {
    ancestors: GeographicReference[];
    children: GeographicReference[];
    siblings: GeographicReference[];
  };

  temporalValidity: {
    effectiveDate: string;
    expirationDate: string | null;
    changeType: 'NEW' | 'MODIFIED' | 'MERGED' | 'SPLIT' | 'UNCHANGED';
    predecessorCodes: string[];
    successorCodes: string[];
  };
}

interface GeographicReference {
  geoCode: string;
  geoLevel: number;
  geoName: string;
}

// Geographic Concordance
class GeographicConcordance {
  // Cross-walk between geographic vintages
  static buildConcordance(
    sourceYear: number,
    targetYear: number
  ): ConcordanceTable {
    return {
      sourceVintage: sourceYear,
      targetVintage: targetYear,
      mappings: [],
      interpolationMethod: 'AREA_WEIGHTED' // or POPULATION_WEIGHTED
    };
  }
}

interface ConcordanceTable {
  sourceVintage: number;
  targetVintage: number;
  mappings: ConcordanceMapping[];
  interpolationMethod: string;
}

interface ConcordanceMapping {
  sourceGeoCode: string;
  targetGeoCode: string;
  overlapPercentage: number;
  populationWeight: number;
  relationshipType: 'ONE_TO_ONE' | 'ONE_TO_MANY' | 'MANY_TO_ONE' | 'MANY_TO_MANY';
}
```

### 3.4 Classification Systems

```typescript
// Standard Classification Codes
interface ClassificationSystems {
  demographic: {
    age: {
      singleYear: '0-125';
      fiveYearGroups: ['0-4', '5-9', '10-14', /* ... */ '85+'];
      lifeCycleGroups: [
        '0-14 (Children)',
        '15-24 (Youth)',
        '25-54 (Prime Working Age)',
        '55-64 (Older Workers)',
        '65+ (Seniors)'
      ];
    };
  };

  occupation: {
    standard: 'ISCO-08';
    structure: {
      majorGroup: 'Single digit (1-9, 0)';
      subMajorGroup: 'Two digits';
      minorGroup: 'Three digits';
      unitGroup: 'Four digits';
    };
    majorGroups: [
      { code: '1', title: 'Managers' },
      { code: '2', title: 'Professionals' },
      { code: '3', title: 'Technicians and Associate Professionals' },
      { code: '4', title: 'Clerical Support Workers' },
      { code: '5', title: 'Service and Sales Workers' },
      { code: '6', title: 'Skilled Agricultural Workers' },
      { code: '7', title: 'Craft and Related Trades Workers' },
      { code: '8', title: 'Plant and Machine Operators' },
      { code: '9', title: 'Elementary Occupations' },
      { code: '0', title: 'Armed Forces Occupations' }
    ];
  };

  industry: {
    standard: 'ISIC Rev.4';
    structure: {
      section: 'Single letter (A-U)';
      division: 'Two digits';
      group: 'Three digits';
      class: 'Four digits';
    };
    sections: [
      { code: 'A', title: 'Agriculture, Forestry and Fishing' },
      { code: 'B', title: 'Mining and Quarrying' },
      { code: 'C', title: 'Manufacturing' },
      // ... additional sections
    ];
  };

  education: {
    standard: 'ISCED 2011';
    levels: [
      { code: 0, title: 'Early Childhood Education' },
      { code: 1, title: 'Primary Education' },
      { code: 2, title: 'Lower Secondary Education' },
      { code: 3, title: 'Upper Secondary Education' },
      { code: 4, title: 'Post-secondary Non-tertiary' },
      { code: 5, title: 'Short-cycle Tertiary' },
      { code: 6, title: 'Bachelor or Equivalent' },
      { code: 7, title: 'Master or Equivalent' },
      { code: 8, title: 'Doctoral or Equivalent' }
    ];
    fieldsStandard: 'ISCED-F 2013';
  };

  ethnicity: {
    note: 'Nationally defined, no international standard';
    harmonizationAttempts: [
      'UN ethnic/national groups',
      'Regional standards (EU, etc.)'
    ];
    sensitivityLevel: 'HIGH';
    multipleResponseAllowed: true;
  };

  religion: {
    note: 'Nationally defined';
    internationalGuidance: 'UN Principles and Recommendations';
    commonCategories: [
      'Christian (with denominations)',
      'Muslim',
      'Hindu',
      'Buddhist',
      'Jewish',
      'Other',
      'No religion',
      'Not stated'
    ];
    optionalQuestion: true;
  };
}

// Classification Code Validation
class ClassificationValidator {
  static validateISCO08(code: string): ValidationResult {
    const pattern = /^[0-9]{1,4}$/;
    if (!pattern.test(code)) {
      return { valid: false, error: 'Invalid ISCO-08 format' };
    }

    const majorGroup = code[0];
    if (!['0', '1', '2', '3', '4', '5', '6', '7', '8', '9'].includes(majorGroup)) {
      return { valid: false, error: 'Invalid major group' };
    }

    return { valid: true };
  }

  static validateISIC4(code: string): ValidationResult {
    const sectionPattern = /^[A-U]$/;
    const numericPattern = /^[0-9]{2,4}$/;

    if (sectionPattern.test(code) || numericPattern.test(code)) {
      return { valid: true };
    }

    return { valid: false, error: 'Invalid ISIC Rev.4 format' };
  }

  static validateISCED2011(level: number): ValidationResult {
    if (level >= 0 && level <= 8) {
      return { valid: true };
    }
    return { valid: false, error: 'ISCED level must be 0-8' };
  }
}

interface ValidationResult {
  valid: boolean;
  error?: string;
}
```

### 3.5 Data Exchange Formats

```typescript
// Data Exchange Format Specifications
interface DataExchangeFormats {
  microdata: {
    formats: {
      csv: {
        description: 'Comma-separated values';
        encoding: 'UTF-8';
        delimiter: ',';
        quoteChar: '"';
        lineTerminator: '\n';
        headerRow: true;
        missingValueCodes: ['', 'NA', '.'];
      };
      parquet: {
        description: 'Apache Parquet columnar format';
        compression: 'SNAPPY' | 'GZIP' | 'ZSTD';
        rowGroupSize: 128 * 1024 * 1024;
        useCase: 'Large datasets, analytics';
      };
      sdmxMl: {
        description: 'Statistical Data and Metadata Exchange';
        standard: 'SDMX 3.0';
        useCase: 'International exchange';
      };
    };
  };

  aggregateData: {
    formats: {
      jsonStat: {
        description: 'JSON-stat 2.0';
        structure: 'Hypercube with metadata';
        example: {
          class: 'dataset',
          dimension: {},
          value: [],
          status: []
        };
      };
      sdmxJson: {
        description: 'SDMX-JSON';
        standard: 'SDMX 3.0';
        useCase: 'API responses';
      };
      csvW: {
        description: 'CSV on the Web';
        metadataFormat: 'JSON-LD';
        w3cStandard: true;
      };
    };
  };

  geographic: {
    formats: {
      geoJson: {
        description: 'Geographic JSON';
        standard: 'RFC 7946';
        coordinateSystem: 'WGS84';
      };
      topoJson: {
        description: 'Topology-encoded GeoJSON';
        compression: 'Shared arc topology';
        useCase: 'Web mapping';
      };
      geoPackage: {
        description: 'OGC GeoPackage';
        format: 'SQLite container';
        useCase: 'Desktop GIS';
      };
      shapeFile: {
        description: 'ESRI Shapefile';
        status: 'Legacy but widely used';
        limitation: 'File size, attribute names';
      };
    };
  };
}

// SDMX Data Structure Definition
interface SDMXDataStructure {
  id: string;
  agencyId: string;
  version: string;

  dimensions: SDMXDimension[];
  measures: SDMXMeasure[];
  attributes: SDMXAttribute[];

  groups: SDMXGroup[];
}

interface SDMXDimension {
  id: string;
  position: number;
  conceptIdentity: string;
  localRepresentation: {
    enumeration: string; // Reference to codelist
  };
}

interface SDMXMeasure {
  id: string;
  conceptIdentity: string;
  localRepresentation: {
    format: {
      dataType: 'Integer' | 'Decimal' | 'String';
    };
  };
}

interface SDMXAttribute {
  id: string;
  conceptIdentity: string;
  assignmentStatus: 'Mandatory' | 'Conditional';
  attachmentLevel: 'DataSet' | 'Series' | 'Observation';
}

// JSON-stat 2.0 Dataset
interface JSONStatDataset {
  class: 'dataset';
  version: '2.0';
  label: string;
  source: string;
  updated: string;

  id: string[];
  size: number[];
  dimension: {
    [key: string]: JSONStatDimension;
  };
  value: (number | null)[];
  status?: {
    [index: string]: string;
  };

  link?: {
    alternate?: JSONStatLink[];
  };
}

interface JSONStatDimension {
  label: string;
  category: {
    index: { [key: string]: number } | string[];
    label: { [key: string]: string };
    unit?: { [key: string]: JSONStatUnit };
  };
}

interface JSONStatUnit {
  label: string;
  decimals: number;
  position: 'start' | 'end';
}

interface JSONStatLink {
  type: string;
  href: string;
}
```

### 3.6 Metadata Standards

```typescript
// Census Metadata Framework
interface CensusMetadataFramework {
  standards: {
    primary: 'SDMX';
    supplementary: ['Dublin Core', 'DDI', 'ISO 19115'];
  };

  metadataLevels: {
    structural: {
      description: 'Data structure definitions';
      includes: [
        'Concept schemes',
        'Codelists',
        'Data structure definitions',
        'Dataflows'
      ];
    };
    reference: {
      description: 'Descriptive metadata';
      includes: [
        'Methodology documentation',
        'Quality reports',
        'Processing descriptions',
        'Revision policies'
      ];
    };
    process: {
      description: 'Production metadata';
      includes: [
        'Collection procedures',
        'Processing steps',
        'Quality controls',
        'Release schedule'
      ];
    };
  };
}

// Reference Metadata Structure
interface ReferenceMetadata {
  identification: {
    title: string;
    subtitle?: string;
    abstract: string;
    keywords: string[];
    topicCategories: string[];
  };

  pointOfContact: {
    organizationName: string;
    contactEmail: string;
    contactPhone?: string;
    role: 'producer' | 'distributor' | 'custodian';
  };

  temporalCoverage: {
    referenceDate: string;
    publicationDate: string;
    revisionDate?: string;
    updateFrequency: string;
  };

  spatialCoverage: {
    geographicDescription: string;
    boundingBox: {
      westLongitude: number;
      eastLongitude: number;
      southLatitude: number;
      northLatitude: number;
    };
    lowestGeographicLevel: string;
  };

  quality: {
    accuracy: {
      samplingError?: string;
      nonSamplingError: string;
      coverageRate: number;
      responseRate: number;
    };
    comparability: {
      temporal: string;
      geographic: string;
      withOtherSources: string;
    };
    coherence: string;
    timeliness: {
      collectionToPublication: string;
    };
  };

  methodology: {
    universe: string;
    unitOfAnalysis: string;
    samplingProcedure?: string;
    dataCollectionMode: string[];
    dataCollectionPeriod: string;
    estimationMethod?: string;
    imputationMethod?: string;
  };

  accessibility: {
    dataDistributor: string;
    accessConditions: string;
    dataFormat: string[];
    apiEndpoint?: string;
    downloadUrl?: string;
    citationRequirement: string;
  };

  confidentiality: {
    disclosureAvoidanceApplied: boolean;
    methods: string[];
    suppressionThreshold?: number;
    perturbationApplied?: boolean;
  };
}
```

---

**WIA-CENSUS-DATA Data Formats**
**Version**: 1.0.0
**Last Updated**: 2025
**License**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - Benefit All Humanity
