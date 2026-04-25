/**
 * WIA Data Visualization Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Visualization Types
// ============================================================================

export interface WIADataVisualization {
  standard: 'WIA-DATA-VISUALIZATION';
  version: string;
  visualization: VisualizationMetadata;
  dataSource: DataSourceConfig;
  chart: ChartConfiguration;
  styling: StylingConfiguration;
  interactions?: InteractionConfiguration;
  accessibility?: AccessibilityConfiguration;
  extensions?: Record<string, unknown>;
}

export interface VisualizationMetadata {
  id: string;
  name: string;
  description?: string;
  type: VisualizationType;
  owner: string;
  createdAt: string;
  updatedAt?: string;
  version?: string;
  tags?: string[];
}

export type VisualizationType =
  | 'bar'
  | 'line'
  | 'area'
  | 'pie'
  | 'donut'
  | 'scatter'
  | 'bubble'
  | 'heatmap'
  | 'treemap'
  | 'sunburst'
  | 'sankey'
  | 'chord'
  | 'gauge'
  | 'funnel'
  | 'map'
  | 'network'
  | 'table'
  | 'kpi'
  | 'custom';

// ============================================================================
// Data Source Types
// ============================================================================

export interface DataSourceConfig {
  type: DataSourceType;
  connection: ConnectionConfig;
  query?: QueryConfig;
  transform?: TransformConfig[];
  refresh?: RefreshConfig;
}

export type DataSourceType =
  | 'api'
  | 'database'
  | 'file'
  | 'stream'
  | 'inline';

export interface ConnectionConfig {
  url?: string;
  method?: 'GET' | 'POST';
  headers?: Record<string, string>;
  authentication?: AuthenticationConfig;
  timeout?: number;
}

export interface AuthenticationConfig {
  type: 'none' | 'basic' | 'bearer' | 'api-key' | 'oauth2';
  credentials?: Record<string, string>;
}

export interface QueryConfig {
  type: 'sql' | 'graphql' | 'json-path' | 'custom';
  expression: string;
  parameters?: Record<string, unknown>;
}

export interface TransformConfig {
  type: TransformType;
  config: Record<string, unknown>;
}

export type TransformType =
  | 'filter'
  | 'sort'
  | 'aggregate'
  | 'pivot'
  | 'join'
  | 'calculate'
  | 'format'
  | 'rename';

export interface RefreshConfig {
  type: 'manual' | 'interval' | 'realtime';
  interval?: number;
  onEvent?: string[];
}

// ============================================================================
// Chart Configuration Types
// ============================================================================

export interface ChartConfiguration {
  dimensions: DimensionConfig[];
  measures: MeasureConfig[];
  layout: LayoutConfig;
  axes?: AxesConfig;
  legend?: LegendConfig;
  tooltip?: TooltipConfig;
  annotations?: AnnotationConfig[];
}

export interface DimensionConfig {
  field: string;
  label?: string;
  type: DimensionType;
  format?: FormatConfig;
  sort?: SortConfig;
  binning?: BinningConfig;
}

export type DimensionType =
  | 'categorical'
  | 'temporal'
  | 'geographical'
  | 'hierarchical';

export interface MeasureConfig {
  field: string;
  label?: string;
  aggregation: AggregationType;
  format?: FormatConfig;
  color?: ColorConfig;
  threshold?: ThresholdConfig[];
}

export type AggregationType =
  | 'sum'
  | 'avg'
  | 'min'
  | 'max'
  | 'count'
  | 'distinct-count'
  | 'median'
  | 'percentile'
  | 'variance'
  | 'stddev'
  | 'custom';

export interface FormatConfig {
  type: 'number' | 'currency' | 'percentage' | 'date' | 'custom';
  pattern?: string;
  locale?: string;
  decimals?: number;
  prefix?: string;
  suffix?: string;
}

export interface SortConfig {
  direction: 'asc' | 'desc';
  by?: 'value' | 'label' | 'custom';
  customOrder?: string[];
}

export interface BinningConfig {
  type: 'auto' | 'fixed' | 'custom';
  count?: number;
  size?: number;
  boundaries?: number[];
}

export interface ColorConfig {
  type: 'solid' | 'gradient' | 'categorical' | 'sequential' | 'diverging';
  value?: string;
  palette?: string[];
  scale?: ColorScale;
}

export interface ColorScale {
  domain: number[];
  range: string[];
  interpolation?: 'linear' | 'step' | 'basis';
}

export interface ThresholdConfig {
  value: number;
  color: string;
  label?: string;
  operator: 'lt' | 'lte' | 'eq' | 'gte' | 'gt';
}

export interface LayoutConfig {
  width: SizeConfig;
  height: SizeConfig;
  padding: PaddingConfig;
  orientation?: 'horizontal' | 'vertical';
  stacking?: 'none' | 'stacked' | 'percent';
}

export interface SizeConfig {
  type: 'fixed' | 'responsive' | 'auto';
  value?: number;
  min?: number;
  max?: number;
}

export interface PaddingConfig {
  top: number;
  right: number;
  bottom: number;
  left: number;
}

export interface AxesConfig {
  x?: AxisConfig;
  y?: AxisConfig;
  y2?: AxisConfig;
}

export interface AxisConfig {
  show: boolean;
  label?: string;
  position?: 'top' | 'bottom' | 'left' | 'right';
  scale?: ScaleConfig;
  ticks?: TickConfig;
  grid?: GridConfig;
}

export interface ScaleConfig {
  type: 'linear' | 'log' | 'pow' | 'sqrt' | 'time' | 'ordinal' | 'band';
  domain?: [number, number] | string[];
  nice?: boolean;
  zero?: boolean;
}

export interface TickConfig {
  count?: number;
  format?: FormatConfig;
  rotation?: number;
  values?: (string | number)[];
}

export interface GridConfig {
  show: boolean;
  style?: 'solid' | 'dashed' | 'dotted';
  color?: string;
  opacity?: number;
}

export interface LegendConfig {
  show: boolean;
  position: 'top' | 'bottom' | 'left' | 'right';
  orientation?: 'horizontal' | 'vertical';
  interactive?: boolean;
  maxItems?: number;
}

export interface TooltipConfig {
  show: boolean;
  format?: TooltipFormat;
  customTemplate?: string;
  position?: 'auto' | 'fixed';
}

export interface TooltipFormat {
  title?: string;
  fields: TooltipField[];
}

export interface TooltipField {
  field: string;
  label?: string;
  format?: FormatConfig;
}

export interface AnnotationConfig {
  type: 'line' | 'area' | 'point' | 'text';
  position: AnnotationPosition;
  label?: string;
  style?: AnnotationStyle;
}

export interface AnnotationPosition {
  x?: number | string;
  y?: number | string;
  x2?: number | string;
  y2?: number | string;
}

export interface AnnotationStyle {
  color?: string;
  opacity?: number;
  lineWidth?: number;
  lineDash?: number[];
  fontSize?: number;
}

// ============================================================================
// Styling Types
// ============================================================================

export interface StylingConfiguration {
  theme: ThemeConfig;
  colors: ColorPaletteConfig;
  typography: TypographyConfig;
  borders?: BorderConfig;
  shadows?: ShadowConfig;
}

export interface ThemeConfig {
  name: string;
  mode: 'light' | 'dark' | 'auto';
  base?: string;
  overrides?: Record<string, unknown>;
}

export interface ColorPaletteConfig {
  primary: string;
  secondary: string;
  background: string;
  text: string;
  accent: string[];
  semantic?: SemanticColors;
}

export interface SemanticColors {
  success: string;
  warning: string;
  error: string;
  info: string;
}

export interface TypographyConfig {
  fontFamily: string;
  fontSize: FontSizeConfig;
  fontWeight?: FontWeightConfig;
}

export interface FontSizeConfig {
  title: number;
  label: number;
  body: number;
  caption: number;
}

export interface FontWeightConfig {
  normal: number;
  bold: number;
}

export interface BorderConfig {
  radius: number;
  width: number;
  color: string;
}

export interface ShadowConfig {
  enabled: boolean;
  color: string;
  blur: number;
  offset: { x: number; y: number };
}

// ============================================================================
// Interaction Types
// ============================================================================

export interface InteractionConfiguration {
  selection?: SelectionConfig;
  zoom?: ZoomConfig;
  pan?: PanConfig;
  drill?: DrillConfig;
  filter?: FilterConfig;
  crossFilter?: CrossFilterConfig;
}

export interface SelectionConfig {
  enabled: boolean;
  type: 'single' | 'multi' | 'brush';
  highlightOnHover?: boolean;
}

export interface ZoomConfig {
  enabled: boolean;
  type: 'wheel' | 'pinch' | 'both';
  minScale?: number;
  maxScale?: number;
}

export interface PanConfig {
  enabled: boolean;
  type: 'drag' | 'scroll';
}

export interface DrillConfig {
  enabled: boolean;
  hierarchy: string[];
  direction: 'down' | 'up' | 'both';
}

export interface FilterConfig {
  enabled: boolean;
  controls: FilterControl[];
}

export interface FilterControl {
  field: string;
  type: 'dropdown' | 'slider' | 'date-picker' | 'search';
  label?: string;
  defaultValue?: unknown;
}

export interface CrossFilterConfig {
  enabled: boolean;
  targets: string[];
  mode: 'highlight' | 'filter';
}

// ============================================================================
// Accessibility Types
// ============================================================================

export interface AccessibilityConfiguration {
  enabled: boolean;
  title?: string;
  description?: string;
  keyboardNavigation?: boolean;
  screenReader?: ScreenReaderConfig;
  highContrast?: boolean;
  alternativeFormats?: AlternativeFormat[];
}

export interface ScreenReaderConfig {
  announcements?: boolean;
  dataTable?: boolean;
  summary?: string;
}

export interface AlternativeFormat {
  type: 'table' | 'text' | 'audio';
  content?: string;
  url?: string;
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface VisualizationResponse {
  id: string;
  name: string;
  type: VisualizationType;
  thumbnailUrl?: string;
  createdAt: string;
  updatedAt?: string;
  links: {
    self: string;
    embed?: string;
    export?: string;
  };
}

export interface RenderRequest {
  visualizationId: string;
  format: 'svg' | 'png' | 'pdf';
  width?: number;
  height?: number;
  quality?: number;
  filters?: Record<string, unknown>;
}

export interface RenderResponse {
  url: string;
  expiresAt: string;
  format: string;
  size: { width: number; height: number };
}

// ============================================================================
// Utility Types
// ============================================================================

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
  timestamp: string;
  requestId?: string;
}

export interface ValidationResult {
  valid: boolean;
  errors?: ValidationError[];
}

export interface ValidationError {
  path: string;
  message: string;
  value?: unknown;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    total: number;
    limit: number;
    offset: number;
    hasMore: boolean;
  };
  links: {
    first?: string;
    prev?: string;
    self: string;
    next?: string;
    last?: string;
  };
}
