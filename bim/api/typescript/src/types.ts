/**
 * WIA BIM Standard - Type Definitions
 *
 * @packageDocumentation
 * @module wia-bim
 */

export enum BIMLevel {
  Level0 = 'level_0',
  Level1 = 'level_1',
  Level2 = 'level_2',
  Level3 = 'level_3'
}

export enum IFCVersion {
  IFC2x3 = 'ifc2x3',
  IFC4 = 'ifc4',
  IFC4x3 = 'ifc4x3'
}

export enum ElementCategory {
  Architectural = 'architectural',
  Structural = 'structural',
  MEP = 'mep',
  Site = 'site',
  Furniture = 'furniture',
  Equipment = 'equipment'
}

export enum ProjectPhase {
  Concept = 'concept',
  SchematicDesign = 'schematic_design',
  DesignDevelopment = 'design_development',
  ConstructionDocuments = 'construction_documents',
  Construction = 'construction',
  Operation = 'operation',
  Renovation = 'renovation',
  Demolition = 'demolition'
}

export enum LOD {
  LOD100 = 100,
  LOD200 = 200,
  LOD300 = 300,
  LOD350 = 350,
  LOD400 = 400,
  LOD500 = 500
}

export interface Vector3D {
  x: number;
  y: number;
  z: number;
}

export interface BoundingBox {
  min: Vector3D;
  max: Vector3D;
}

export interface Transformation {
  origin: Vector3D;
  rotation: Vector3D;
  scale: Vector3D;
}

export interface BIMProject {
  id: string;
  name: string;
  description?: string;
  phase: ProjectPhase;
  bimLevel: BIMLevel;
  ifcVersion: IFCVersion;
  location: ProjectLocation;
  discipline: string[];
  models: BIMModel[];
  createdAt: Date;
  updatedAt: Date;
}

export interface ProjectLocation {
  address: string;
  city: string;
  country: string;
  latitude?: number;
  longitude?: number;
  elevation?: number;
}

export interface BIMModel {
  id: string;
  name: string;
  discipline: string;
  lod: LOD;
  version: string;
  filePath: string;
  fileSize: number;
  format: 'ifc' | 'rvt' | 'nwd' | 'dwg' | 'glb';
  elements: BIMElement[];
  metadata: Record<string, unknown>;
  lastModified: Date;
}

export interface BIMElement {
  id: string;
  globalId: string;
  ifcType: string;
  name: string;
  category: ElementCategory;
  lod: LOD;
  geometry: ElementGeometry;
  properties: PropertySet[];
  materials: MaterialInfo[];
  classification?: Classification;
  relationships: ElementRelationship[];
}

export interface ElementGeometry {
  boundingBox: BoundingBox;
  transformation: Transformation;
  volume?: number;
  area?: number;
  length?: number;
  meshReference?: string;
}

export interface PropertySet {
  name: string;
  properties: Property[];
}

export interface Property {
  name: string;
  value: unknown;
  type: 'string' | 'number' | 'boolean' | 'date' | 'length' | 'area' | 'volume';
  unit?: string;
}

export interface MaterialInfo {
  name: string;
  category: string;
  fraction?: number;
  cost?: number;
  properties?: Record<string, unknown>;
}

export interface Classification {
  system: string;
  code: string;
  description?: string;
}

export interface ElementRelationship {
  type: 'contains' | 'contained_in' | 'connected_to' | 'adjacent_to' | 'intersects';
  targetId: string;
  metadata?: Record<string, unknown>;
}

export interface ClashDetection {
  id: string;
  modelA: string;
  modelB: string;
  clashes: Clash[];
  timestamp: Date;
  status: 'pending' | 'in_progress' | 'completed';
}

export interface Clash {
  id: string;
  elementA: string;
  elementB: string;
  clashPoint: Vector3D;
  distance: number;
  severity: 'critical' | 'major' | 'minor';
  status: 'new' | 'active' | 'resolved' | 'ignored';
  assignedTo?: string;
}

export interface QuantityTakeoff {
  id: string;
  modelId: string;
  items: QuantityItem[];
  totalCost?: number;
  currency?: string;
  timestamp: Date;
}

export interface QuantityItem {
  elementId: string;
  ifcType: string;
  name: string;
  quantity: number;
  unit: string;
  unitCost?: number;
  totalCost?: number;
}

export interface COBieData {
  facility: COBieFacility;
  floors: COBieFloor[];
  spaces: COBieSpace[];
  zones: COBieZone[];
  types: COBieType[];
  components: COBieComponent[];
}

export interface COBieFacility {
  name: string;
  category: string;
  projectName: string;
  siteName: string;
  linearUnits: string;
  areaUnits: string;
  volumeUnits: string;
}

export interface COBieFloor {
  name: string;
  elevation: number;
  height: number;
}

export interface COBieSpace {
  name: string;
  floorName: string;
  category: string;
  area: number;
}

export interface COBieZone {
  name: string;
  category: string;
  spaces: string[];
}

export interface COBieType {
  name: string;
  category: string;
  manufacturer?: string;
  modelNumber?: string;
}

export interface COBieComponent {
  name: string;
  typeName: string;
  spaceName: string;
  serialNumber?: string;
  installationDate?: Date;
}

export interface BIMConfig {
  serverUrl: string;
  apiKey?: string;
  defaultIFCVersion: IFCVersion;
  defaultLOD: LOD;
  enableClashDetection: boolean;
  enableQuantityTakeoff: boolean;
}

export enum CertificationLevel {
  Bronze = 'bronze',
  Silver = 'silver',
  Gold = 'gold'
}

export interface ComplianceReport {
  standard: 'WIA-BIM';
  testDate: string;
  config: BIMConfig;
  targetLevel: CertificationLevel;
  tests: TestResult[];
  passed: boolean;
  achievedLevel?: CertificationLevel;
}

export interface TestResult {
  testName: string;
  passed: boolean;
  notes?: string;
}
