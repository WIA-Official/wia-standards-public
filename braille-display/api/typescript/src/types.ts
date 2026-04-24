/**
 * WIA Braille Display Standard - Type Definitions
 * @packageDocumentation
 * @module wia-braille-display
 */

export enum BrailleGrade {
  Grade1 = 'grade1',
  Grade2 = 'grade2',
  Grade3 = 'grade3',
  Computer = 'computer'
}

export enum BrailleCode {
  UEB = 'ueb',
  EBAE = 'ebae',
  French = 'french',
  German = 'german',
  Korean = 'korean',
  Japanese = 'japanese',
  Chinese = 'chinese',
  Nemeth = 'nemeth',
  Music = 'music'
}

export enum DisplayType {
  SingleLine = 'single_line',
  MultiLine = 'multi_line',
  TactileGraphics = 'tactile_graphics',
  Hybrid = 'hybrid'
}

export enum ConnectionType {
  USB = 'usb',
  Bluetooth = 'bluetooth',
  Serial = 'serial',
  WiFi = 'wifi'
}

export interface BrailleCell {
  dots: boolean[];
  character?: string;
  unicode?: number;
}

export interface DisplayInfo {
  id: string;
  name: string;
  manufacturer: string;
  model: string;
  type: DisplayType;
  cellCount: number;
  rowCount: number;
  dotCount: 6 | 8;
  hasCursor: boolean;
  hasButtons: boolean;
  buttonCount: number;
  connection: ConnectionType;
  firmwareVersion: string;
}

export interface DisplayState {
  deviceId: string;
  connected: boolean;
  cells: BrailleCell[];
  cursorPosition: number;
  activeRow: number;
  buttonStates: boolean[];
  batteryLevel?: number;
  lastUpdate: Date;
}

export interface TextContent {
  text: string;
  brailleGrade: BrailleGrade;
  brailleCode: BrailleCode;
  cursorPosition?: number;
  selectionStart?: number;
  selectionEnd?: number;
}

export interface TranslationResult {
  brailleCells: BrailleCell[];
  brailleString: string;
  characterMapping: CharacterMapping[];
}

export interface CharacterMapping {
  textIndex: number;
  brailleIndex: number;
  textLength: number;
  brailleLength: number;
}

export interface TactileGraphic {
  id: string;
  name: string;
  width: number;
  height: number;
  data: boolean[][];
  description: string;
  metadata?: Record<string, unknown>;
}

export interface InputEvent {
  type: 'button' | 'cursor' | 'gesture' | 'braille_input';
  deviceId: string;
  timestamp: Date;
  data: ButtonEvent | CursorEvent | GestureEvent | BrailleInputEvent;
}

export interface ButtonEvent {
  button: number;
  action: 'press' | 'release' | 'hold';
}

export interface CursorEvent {
  position: number;
  action: 'route' | 'cut';
}

export interface GestureEvent {
  gesture: string;
  fingers: number;
  direction?: string;
}

export interface BrailleInputEvent {
  dots: boolean[];
  character?: string;
}

export interface NavigationContext {
  documentType: string;
  totalLength: number;
  currentPosition: number;
  headingLevel?: number;
  listLevel?: number;
  tableInfo?: TableInfo;
}

export interface TableInfo {
  rows: number;
  columns: number;
  currentRow: number;
  currentColumn: number;
  hasHeader: boolean;
}

export interface BrailleDisplayConfig {
  defaultGrade: BrailleGrade;
  defaultCode: BrailleCode;
  autoScroll: boolean;
  scrollSpeed: number;
  cursorBlinkRate: number;
  audioFeedback: boolean;
  hapticFeedback: boolean;
}

export enum CertificationLevel {
  Bronze = 'bronze',
  Silver = 'silver',
  Gold = 'gold'
}

export interface ComplianceReport {
  standard: 'WIA-BRAILLE-DISPLAY';
  testDate: string;
  config: BrailleDisplayConfig;
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
