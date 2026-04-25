# 제3장: 인구조사 데이터 형식 및 스키마 사양

## 인구 통계를 위한 포괄적인 데이터 모델링

### 3.1 핵심 데이터 모델 아키텍처

WIA-CENSUS-DATA 표준은 수집부터 배포까지 인구조사 데이터의 전체 생명주기를 지원하는 포괄적인 데이터 모델을 정의합니다. 이 장에서는 모든 데이터 구조에 대한 상세한 사양을 제공합니다.

```typescript
// 핵심 인구조사 데이터 모델
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
    referenceDate: '인구조사일 시점';
    historicalTracking: '유효 날짜가 있는 버전 기반';
    longitudinalLinking: '영구 개인 식별자';
  };
}

// 개인 엔티티 정의
interface PersonEntity {
  identification: {
    personId: {
      type: 'UUID';
      description: '인구조사 내 고유 개인 식별자';
      persistence: '단일 인구조사 주기';
      format: 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx';
    };
    linkedPersonId: {
      type: 'ENCRYPTED_HASH';
      description: '인구조사 간 연계 키';
      encryption: '솔트가 있는 단방향 해시';
      purpose: '종단적 분석 전용';
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
      standard: 'UN 인구조사 권고사항';
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
      description: '출생 국가 코드';
    };
    citizenship: {
      type: 'ARRAY<ISO3166_ALPHA3>';
      description: '현재 시민권';
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
    languagesSpoken: {
      type: 'ARRAY<ISO639_3>';
      description: '가정에서 사용하는 언어';
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
      referenceWeek: '인구조사 전 주';
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
    hoursWorked: {
      type: 'INTEGER';
      range: [0, 168];
      description: '기준 주간 근로 시간';
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
}

type DifficultyLevel = 'NO_DIFFICULTY' | 'SOME_DIFFICULTY' | 'A_LOT_OF_DIFFICULTY' | 'CANNOT_DO' | 'NOT_STATED';
```

### 3.2 가구 및 주거 스키마

```typescript
// 가구 엔티티 정의
interface HouseholdEntity {
  identification: {
    householdId: {
      type: 'UUID';
      description: '고유 가구 식별자';
    };
    dwellingId: {
      type: 'UUID';
      foreignKey: 'dwelling.dwellingId';
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
      description: '가구 내 가족 단위';
    };
  };

  economicCharacteristics: {
    totalIncome: {
      type: 'MONETARY';
      currency: 'LOCAL_CURRENCY';
      period: 'ANNUAL';
      confidentialityTreatment: 'RANGES';
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

// 주거 엔티티 정의
interface DwellingEntity {
  identification: {
    dwellingId: {
      type: 'UUID';
      description: '고유 주거 식별자';
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
  };

  utilities: {
    waterSupply: {
      type: 'ENUM';
      values: ['PIPED_INSIDE', 'PIPED_OUTSIDE', 'WELL', 'OTHER', 'NONE'];
    };
    heatingType: {
      type: 'ENUM';
      values: ['CENTRAL', 'ELECTRIC', 'GAS', 'OIL', 'WOOD', 'NONE'];
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
  };
}
```

### 3.3 지리적 데이터 구조

```typescript
// 지리적 계층 정의
interface GeographicHierarchy {
  version: '1.0.0';
  referenceYear: number;

  levels: {
    nation: {
      level: 0;
      codeFormat: 'ISO3166_ALPHA3';
      example: 'KOR';
      count: 1;
    };
    region: {
      level: 1;
      codeFormat: 'ALPHA_NUMERIC_3';
      example: '서울';
      description: '주요 행정 구역';
      typicalCount: '17개 시도';
    };
    district: {
      level: 2;
      codeFormat: 'NUMERIC_5';
      example: '11010';
      typicalCount: '250개 시군구';
    };
    subdistrict: {
      level: 3;
      codeFormat: 'NUMERIC_8';
      description: '읍면동';
    };
    block: {
      level: 4;
      codeFormat: 'NUMERIC_10';
      description: '최소 집계 단위';
    };
  };

  alternativeGeographies: {
    statisticalAreas: {
      metropolitanArea: '기능적 도시 지역';
      urbanRuralClassification: '정주 유형';
      economicRegion: '노동 시장 지역';
    };
    gridBased: {
      standardGrid: '1km x 1km 셀';
      fineGrid: '100m x 100m 셀';
      coordinateSystem: 'WGS84';
    };
  };
}

// 지리적 단위 스키마
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
    urbanRuralClass: {
      type: 'ENUM';
      values: ['URBAN_CORE', 'URBAN_FRINGE', 'PERI_URBAN', 'RURAL', 'REMOTE'];
    };
  };

  temporalValidity: {
    effectiveDate: string;
    expirationDate: string | null;
    changeType: 'NEW' | 'MODIFIED' | 'MERGED' | 'SPLIT' | 'UNCHANGED';
  };
}
```

### 3.4 분류 시스템

```typescript
// 표준 분류 코드
interface ClassificationSystems {
  demographic: {
    age: {
      singleYear: '0-125';
      fiveYearGroups: ['0-4', '5-9', '10-14', /* ... */ '85+'];
      lifeCycleGroups: [
        '0-14 (아동)',
        '15-24 (청년)',
        '25-54 (핵심 노동 연령)',
        '55-64 (고령 근로자)',
        '65+ (고령자)'
      ];
    };
  };

  occupation: {
    standard: 'ISCO-08';
    structure: {
      majorGroup: '단일 자릿수 (1-9, 0)';
      subMajorGroup: '두 자릿수';
      minorGroup: '세 자릿수';
      unitGroup: '네 자릿수';
    };
    majorGroups: [
      { code: '1', title: '관리자' },
      { code: '2', title: '전문가' },
      { code: '3', title: '기술자 및 준전문가' },
      { code: '4', title: '사무 종사자' },
      { code: '5', title: '서비스 및 판매 종사자' },
      { code: '6', title: '농림어업 숙련 종사자' },
      { code: '7', title: '기능원 및 관련 기능 종사자' },
      { code: '8', title: '장치, 기계 조작 및 조립 종사자' },
      { code: '9', title: '단순노무 종사자' },
      { code: '0', title: '군인' }
    ];
  };

  industry: {
    standard: 'ISIC Rev.4';
    structure: {
      section: '단일 문자 (A-U)';
      division: '두 자릿수';
      group: '세 자릿수';
      class: '네 자릿수';
    };
  };

  education: {
    standard: 'ISCED 2011';
    levels: [
      { code: 0, title: '유아 교육' },
      { code: 1, title: '초등 교육' },
      { code: 2, title: '전기 중등 교육' },
      { code: 3, title: '후기 중등 교육' },
      { code: 4, title: '중등후 비고등 교육' },
      { code: 5, title: '단기 고등 교육' },
      { code: 6, title: '학사 또는 동등' },
      { code: 7, title: '석사 또는 동등' },
      { code: 8, title: '박사 또는 동등' }
    ];
  };
}
```

### 3.5 데이터 교환 형식

```typescript
// 데이터 교환 형식 사양
interface DataExchangeFormats {
  microdata: {
    formats: {
      csv: {
        description: '쉼표로 구분된 값';
        encoding: 'UTF-8';
        delimiter: ',';
        headerRow: true;
        missingValueCodes: ['', 'NA', '.'];
      };
      parquet: {
        description: 'Apache Parquet 열 형식';
        compression: 'SNAPPY' | 'GZIP' | 'ZSTD';
        useCase: '대규모 데이터셋, 분석';
      };
      sdmxMl: {
        description: '통계 데이터 및 메타데이터 교환';
        standard: 'SDMX 3.0';
        useCase: '국제 교환';
      };
    };
  };

  aggregateData: {
    formats: {
      jsonStat: {
        description: 'JSON-stat 2.0';
        structure: '메타데이터가 있는 하이퍼큐브';
      };
      sdmxJson: {
        description: 'SDMX-JSON';
        standard: 'SDMX 3.0';
        useCase: 'API 응답';
      };
    };
  };

  geographic: {
    formats: {
      geoJson: {
        description: '지리적 JSON';
        standard: 'RFC 7946';
        coordinateSystem: 'WGS84';
      };
      topoJson: {
        description: '토폴로지 인코딩 GeoJSON';
        compression: '공유 아크 토폴로지';
        useCase: '웹 매핑';
      };
      geoPackage: {
        description: 'OGC GeoPackage';
        format: 'SQLite 컨테이너';
        useCase: '데스크톱 GIS';
      };
    };
  };
}

// JSON-stat 2.0 데이터셋
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
}

interface JSONStatDimension {
  label: string;
  category: {
    index: { [key: string]: number } | string[];
    label: { [key: string]: string };
  };
}
```

### 3.6 메타데이터 표준

```typescript
// 인구조사 메타데이터 프레임워크
interface CensusMetadataFramework {
  standards: {
    primary: 'SDMX';
    supplementary: ['Dublin Core', 'DDI', 'ISO 19115'];
  };

  metadataLevels: {
    structural: {
      description: '데이터 구조 정의';
      includes: [
        '개념 체계',
        '코드 목록',
        '데이터 구조 정의',
        '데이터플로우'
      ];
    };
    reference: {
      description: '설명적 메타데이터';
      includes: [
        '방법론 문서',
        '품질 보고서',
        '처리 설명',
        '개정 정책'
      ];
    };
    process: {
      description: '생산 메타데이터';
      includes: [
        '수집 절차',
        '처리 단계',
        '품질 관리',
        '공개 일정'
      ];
    };
  };
}

// 참조 메타데이터 구조
interface ReferenceMetadata {
  identification: {
    title: string;
    abstract: string;
    keywords: string[];
  };

  temporalCoverage: {
    referenceDate: string;
    publicationDate: string;
    updateFrequency: string;
  };

  spatialCoverage: {
    geographicDescription: string;
    lowestGeographicLevel: string;
  };

  quality: {
    accuracy: {
      coverageRate: number;
      responseRate: number;
    };
    comparability: {
      temporal: string;
      geographic: string;
    };
    timeliness: {
      collectionToPublication: string;
    };
  };

  methodology: {
    universe: string;
    dataCollectionMode: string[];
    estimationMethod?: string;
    imputationMethod?: string;
  };

  accessibility: {
    dataFormat: string[];
    apiEndpoint?: string;
    citationRequirement: string;
  };

  confidentiality: {
    disclosureAvoidanceApplied: boolean;
    methods: string[];
    suppressionThreshold?: number;
  };
}
```

---

**WIA-CENSUS-DATA 데이터 형식**
**버전**: 1.0.0
**최종 업데이트**: 2025
**라이선스**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
