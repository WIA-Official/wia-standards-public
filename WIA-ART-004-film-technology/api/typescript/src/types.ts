/**
 * WIA-ART-004: Film Technology - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Arts Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

export interface FilmTechnologyMetadata {
  id: string;
  title: string;
  artist: string;
  created: Date;
  format: string;
  license: string;
}

export interface FilmTechnologyConfig {
  apiKey?: string;
  endpoint?: string;
  version?: string;
}

export interface ValidationResult {
  isValid: boolean;
  errors: string[];
  warnings: string[];
}

export interface ExportOptions {
  format: string;
  quality?: number;
  embedMetadata?: boolean;
}
