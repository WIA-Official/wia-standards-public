/**
 * WIA-ENE-024: Upcycling Standard - TypeScript Type Definitions
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Core Enums
// ============================================================================

/**
 * 재료 카테고리
 */
export enum MaterialCategory {
  TEXTILE = 'textile',           // 섬유
  PLASTIC = 'plastic',           // 플라스틱
  METAL = 'metal',               // 금속
  WOOD = 'wood',                 // 목재
  GLASS = 'glass',               // 유리
  PAPER = 'paper',               // 종이
  E_WASTE = 'e_waste',           // 전자폐기물
  CONSTRUCTION = 'construction', // 건축자재
  RUBBER = 'rubber',             // 고무
  CERAMIC = 'ceramic',           // 도자기
  LEATHER = 'leather',           // 가죽
  MIXED = 'mixed'                // 혼합
}

/**
 * 재료 상태 등급
 */
export enum MaterialGrade {
  A = 'A', // Excellent - 거의 손상 없음 (95-100%)
  B = 'B', // Good - 경미한 손상 (75-94%)
  C = 'C', // Fair - 중간 손상 (50-74%)
  D = 'D', // Poor - 심각한 손상 (25-49%)
  F = 'F'  // Failed - 업사이클 불가 (0-24%)
}

/**
 * 프로젝트 상태
 */
export enum ProjectStatus {
  PLANNING = 'planning',         // 기획 중
  SOURCING = 'sourcing',         // 재료 수집 중
  IN_PROGRESS = 'in_progress',   // 제작 중
  QA = 'qa',                     // 품질 검사 중
  COMPLETED = 'completed',       // 완료
  SOLD = 'sold',                 // 판매됨
  CANCELLED = 'cancelled'        // 취소됨
}

/**
 * 제품 카테고리
 */
export enum ProductCategory {
  FURNITURE = 'furniture',       // 가구
  BAG = 'bag',                   // 가방
  CLOTHING = 'clothing',         // 의류
  ACCESSORY = 'accessory',       // 액세서리
  HOME_DECOR = 'home_decor',     // 홈데코
  ART = 'art',                   // 예술작품
  PLANTER = 'planter',           // 화분
  LIGHTING = 'lighting',         // 조명
  TOY = 'toy',                   // 장난감
  STATIONERY = 'stationery',     // 문구류
  OTHER = 'other'                // 기타
}

/**
 * QA 결과
 */
export enum QAResult {
  PASS = 'pass',                 // 통과
  CONDITIONAL_PASS = 'conditional_pass', // 조건부 통과
  FAIL = 'fail',                 // 실패
  PENDING = 'pending'            // 검사 대기
}

/**
 * 인증 레벨
 */
export enum CertificationLevel {
  BASIC = 'basic',               // 레벨 1: 기본 인증
  PREMIUM = 'premium',           // 레벨 2: 프리미엄 인증
  EXCELLENCE = 'excellence'      // 레벨 3: 엑셀런스 인증
}

// ============================================================================
// Core Interfaces
// ============================================================================

/**
 * 원료 재료 정보
 */
export interface SourceMaterial {
  materialType: MaterialCategory;
  subType?: string;              // 세부 분류 (예: denim, PET, oak)
  originalProduct?: string;      // 원래 제품 (예: jeans, bottle)
  brand?: string;                // 브랜드
  quantity: number;              // 수량
  unit: string;                  // 단위 (pieces, kg, liters 등)
  weight: number;                // 무게
  weightUnit: string;            // 무게 단위 (kg, g, lbs)
  condition: MaterialGrade;      // 상태 등급
  sourceChannel: string;         // 수집 경로
  acquisitionCost: number;       // 획득 비용
  currency: string;              // 화폐 단위
  acquisitionDate?: string;      // 획득 날짜 (ISO 8601)
  location?: GeoLocation;        // 수집 위치
  photos?: string[];             // 사진 URL
}

/**
 * 위치 정보
 */
export interface GeoLocation {
  lat: number;                   // 위도
  lon: number;                   // 경도
  address?: string;              // 주소
  city?: string;                 // 도시
  country?: string;              // 국가
}

/**
 * 변환 프로세스
 */
export interface Transformation {
  designId: string;              // 디자인 ID
  designName: string;            // 디자인 이름
  designer: string;              // 디자이너
  processes: ProcessStep[];      // 프로세스 단계
  totalLaborHours: number;       // 총 작업 시간
  additionalMaterials?: AdditionalMaterial[]; // 추가 자재
  tools?: string[];              // 사용 도구
  techniques?: string[];         // 사용 기법
}

/**
 * 프로세스 단계
 */
export interface ProcessStep {
  step: number;                  // 단계 번호
  name: string;                  // 단계 이름
  description?: string;          // 설명
  duration: number;              // 소요 시간
  durationUnit: string;          // 시간 단위 (hours, days, minutes)
  energyUsed?: number;           // 사용 에너지
  energyUnit?: string;           // 에너지 단위 (kWh, MJ)
  waterUsed?: number;            // 사용 물
  waterUnit?: string;            // 물 단위 (liters, gallons)
  emissions?: number;            // CO2 배출량 (kg)
  worker?: string;               // 작업자
  notes?: string;                // 메모
  photos?: string[];             // 프로세스 사진
}

/**
 * 추가 자재
 */
export interface AdditionalMaterial {
  material: string;              // 자재명
  quantity: number;              // 수량
  unit?: string;                 // 단위
  cost: number;                  // 비용
  currency: string;              // 화폐 단위
  isRecycled?: boolean;          // 재활용 자재 여부
  supplier?: string;             // 공급업체
}

/**
 * 최종 제품 정보
 */
export interface OutputProduct {
  productType: ProductCategory;  // 제품 카테고리
  subType?: string;              // 세부 분류
  productName: string;           // 제품명
  sku?: string;                  // 재고 관리 코드
  quantity: number;              // 생산 수량
  dimensions?: Dimensions;       // 크기
  weight?: number;               // 무게
  weightUnit?: string;           // 무게 단위
  color?: string[];              // 색상
  features?: string[];           // 특징
  uniqueId?: string[];           // 각 제품 고유 ID
  description?: string;          // 설명
  photos?: string[];             // 제품 사진
}

/**
 * 크기 정보
 */
export interface Dimensions {
  width?: number;                // 폭
  height?: number;               // 높이
  depth?: number;                // 깊이
  diameter?: number;             // 지름
  unit: string;                  // 단위 (cm, inch, mm)
}

/**
 * 가치 지표
 */
export interface ValueMetrics {
  sourceValue: number;           // 원료 가치
  additionalCosts: number;       // 추가 비용 (부자재 등)
  laborCost: number;             // 인건비
  overheadCost?: number;         // 간접비
  totalCost: number;             // 총 비용
  sellingPrice: number;          // 판매가
  sellingPricePerUnit?: number;  // 개당 판매가
  currency: string;              // 화폐 단위
  valueMultiplier: number;       // 가치 배율
  profitMargin: number;          // 이익률
  profitMarginUnit: string;      // 이익률 단위 (percent, ratio)
  marketComparison?: number;     // 시장 대비 가격 (유사 제품 대비 %)
}

/**
 * 환경 영향
 */
export interface EnvironmentalImpact {
  wasteReduced: number;          // 폐기물 감소량
  wasteUnit: string;             // 폐기물 단위
  co2Avoided: number;            // CO2 저감량
  co2Unit: string;               // CO2 단위
  waterSaved?: number;           // 물 절약량
  waterUnit?: string;            // 물 단위
  energySaved?: number;          // 에너지 절약량
  energyUnit?: string;           // 에너지 단위
  calculationMethod: string;     // 계산 방법 (예: LCA_standard)
  certifications?: string[];     // 환경 인증
  carbonFootprint?: number;      // 탄소 발자국 (kg CO2e)
  lifeExpectancy?: number;       // 예상 사용 수명 (years)
}

/**
 * 품질 보증
 */
export interface QualityAssurance {
  inspectionDate: string;        // 검사 날짜 (ISO 8601)
  inspector: string;             // 검사자
  qcResult: QAResult;            // QC 결과
  durabilityTest?: QAResult;     // 내구성 테스트
  safetyTest?: QAResult;         // 안전성 테스트
  chemicalTest?: ChemicalTest;   // 화학물질 테스트
  defects?: Defect[];            // 결함 목록
  warranty?: string;             // 보증 기간
  notes?: string;                // 메모
}

/**
 * 화학물질 테스트
 */
export interface ChemicalTest {
  lead?: number;                 // 납 (ppm)
  cadmium?: number;              // 카드뮴 (ppm)
  mercury?: number;              // 수은 (ppm)
  voc?: number;                  // VOC (μg/m³)
  formaldehyde?: number;         // 폼알데히드 (ppm)
  testDate: string;              // 테스트 날짜
  laboratory?: string;           // 검사 기관
  certified?: boolean;           // 인증 통과 여부
}

/**
 * 결함 정보
 */
export interface Defect {
  type: string;                  // 결함 유형
  severity: 'minor' | 'major' | 'critical'; // 심각도
  description: string;           // 설명
  location?: string;             // 위치
  fixable: boolean;              // 수리 가능 여부
  fixCost?: number;              // 수리 비용
}

/**
 * 메타데이터
 */
export interface ProjectMetadata {
  creator: string;               // 제작자
  creatorId?: string;            // 제작자 ID
  location?: string;             // 제작 위치
  tags?: string[];               // 태그
  storyText?: string;            // 스토리텔링
  photos?: string[];             // 사진
  videos?: string[];             // 동영상
  socialMedia?: {                // 소셜미디어
    instagram?: string;
    facebook?: string;
    youtube?: string;
  };
  collaborators?: string[];      // 협업자
  sponsors?: string[];           // 후원자
}

/**
 * 업사이클링 프로젝트 (전체)
 */
export interface UpcyclingProject {
  projectId: string;             // 프로젝트 ID
  projectName: string;           // 프로젝트명
  status: ProjectStatus;         // 상태
  createdAt: string;             // 생성일 (ISO 8601)
  updatedAt?: string;            // 수정일 (ISO 8601)
  completedAt?: string;          // 완료일 (ISO 8601)

  sourceMaterial: SourceMaterial;      // 원료 정보
  transformation: Transformation;      // 변환 프로세스
  outputProduct: OutputProduct;        // 최종 제품
  valueMetrics: ValueMetrics;          // 가치 지표
  environmentalImpact: EnvironmentalImpact; // 환경 영향
  qualityAssurance?: QualityAssurance; // 품질 보증
  metadata: ProjectMetadata;           // 메타데이터
}

// ============================================================================
// API Request/Response Types
// ============================================================================

/**
 * 프로젝트 생성 요청
 */
export interface CreateProjectRequest {
  projectName: string;
  sourceMaterial: SourceMaterial;
  outputProduct: Partial<OutputProduct>;
  metadata?: Partial<ProjectMetadata>;
}

/**
 * 프로젝트 생성 응답
 */
export interface CreateProjectResponse {
  success: boolean;
  data?: UpcyclingProject;
  error?: ErrorResponse;
}

/**
 * 프로젝트 목록 조회 요청
 */
export interface ListProjectsRequest {
  status?: ProjectStatus;
  materialType?: MaterialCategory;
  productType?: ProductCategory;
  creatorId?: string;
  limit?: number;
  offset?: number;
  sortBy?: 'createdAt' | 'completedAt' | 'valueMultiplier';
  sortOrder?: 'asc' | 'desc';
}

/**
 * 프로젝트 목록 조회 응답
 */
export interface ListProjectsResponse {
  success: boolean;
  data?: {
    projects: UpcyclingProject[];
    total: number;
    limit: number;
    offset: number;
  };
  error?: ErrorResponse;
}

/**
 * 프로젝트 상세 조회 응답
 */
export interface GetProjectResponse {
  success: boolean;
  data?: UpcyclingProject;
  error?: ErrorResponse;
}

/**
 * 프로젝트 업데이트 요청
 */
export interface UpdateProjectRequest {
  projectName?: string;
  status?: ProjectStatus;
  transformation?: Partial<Transformation>;
  outputProduct?: Partial<OutputProduct>;
  valueMetrics?: Partial<ValueMetrics>;
  environmentalImpact?: Partial<EnvironmentalImpact>;
  qualityAssurance?: Partial<QualityAssurance>;
  metadata?: Partial<ProjectMetadata>;
}

/**
 * 프로젝트 업데이트 응답
 */
export interface UpdateProjectResponse {
  success: boolean;
  data?: UpcyclingProject;
  error?: ErrorResponse;
}

/**
 * 환경 영향 계산 요청
 */
export interface CalculateImpactRequest {
  sourceMaterial: Pick<SourceMaterial, 'materialType' | 'weight' | 'weightUnit'>;
  processes: Pick<ProcessStep, 'energyUsed' | 'waterUsed'>[];
  productionMethod?: 'manual' | 'semi_auto' | 'automated';
}

/**
 * 환경 영향 계산 응답
 */
export interface CalculateImpactResponse {
  success: boolean;
  data?: EnvironmentalImpact;
  error?: ErrorResponse;
}

/**
 * 인증서 정보
 */
export interface Certificate {
  certificateId: string;         // 인증서 ID
  projectId: string;             // 프로젝트 ID
  level: CertificationLevel;     // 인증 레벨
  issuedAt: string;              // 발급일 (ISO 8601)
  validUntil?: string;           // 유효기한 (ISO 8601)
  issuer: string;                // 발급 기관
  verificationUrl?: string;      // 검증 URL
  qrCode?: string;               // QR 코드 (base64 or URL)
  blockchainHash?: string;       // 블록체인 해시
}

/**
 * 인증서 조회 응답
 */
export interface GetCertificateResponse {
  success: boolean;
  data?: Certificate;
  error?: ErrorResponse;
}

/**
 * 블록체인 기록
 */
export interface BlockchainRecord {
  blockchainId: string;          // 블록체인 ID
  standardId: string;            // 표준 ID (WIA-ENE-024)
  productId: string;             // 제품 ID
  timeline: BlockchainEvent[];   // 타임라인
  verification: {
    merkleRoot: string;          // 머클 루트
    signature: string;           // 서명
  };
}

/**
 * 블록체인 이벤트
 */
export interface BlockchainEvent {
  timestamp: string;             // 타임스탬프 (ISO 8601)
  event: string;                 // 이벤트 유형
  location?: string;             // 위치
  actor: string;                 // 행위자
  data?: Record<string, any>;    // 추가 데이터
  hash: string;                  // 해시
}

/**
 * 통계 정보
 */
export interface Statistics {
  totalProjects: number;         // 총 프로젝트 수
  completedProjects: number;     // 완료된 프로젝트 수
  totalWasteReduced: number;     // 총 폐기물 감소량 (kg)
  totalCO2Avoided: number;       // 총 CO2 저감량 (kg)
  averageValueMultiplier: number; // 평균 가치 배율
  topMaterials: Array<{          // 상위 재료
    materialType: MaterialCategory;
    count: number;
  }>;
  topProducts: Array<{           // 상위 제품
    productType: ProductCategory;
    count: number;
  }>;
}

/**
 * 통계 조회 응답
 */
export interface GetStatisticsResponse {
  success: boolean;
  data?: Statistics;
  error?: ErrorResponse;
}

/**
 * 에러 응답
 */
export interface ErrorResponse {
  code: string;                  // 에러 코드
  message: string;               // 에러 메시지
  details?: any;                 // 상세 정보
}

// ============================================================================
// Client Configuration
// ============================================================================

/**
 * 클라이언트 설정
 */
export interface ClientConfig {
  apiKey?: string;               // API 키
  endpoint: string;              // API 엔드포인트
  timeout?: number;              // 타임아웃 (ms)
  version?: string;              // API 버전
  locale?: 'en' | 'ko';          // 언어
}

// ============================================================================
// Helper Types
// ============================================================================

/**
 * API 응답 래퍼
 */
export type ApiResponse<T> = {
  success: true;
  data: T;
} | {
  success: false;
  error: ErrorResponse;
};

/**
 * 페이지네이션
 */
export interface Pagination {
  limit: number;
  offset: number;
  total: number;
}

/**
 * 정렬 옵션
 */
export interface SortOption<T> {
  field: keyof T;
  order: 'asc' | 'desc';
}
