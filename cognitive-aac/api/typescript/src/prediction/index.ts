/**
 * WIA Cognitive AAC - Prediction Module Exports
 * 홍익인간 - 널리 인간을 이롭게 하라
 */

// Types
export * from './types';

// Core modules
export { PredictionEngine } from './PredictionEngine';
export { PatternLearner } from './PatternLearner';
export { ContextDetector, type ContextDetectorConfig, type DetectedContext } from './ContextDetector';
export { PrivacyManager, type DataExport, type StorageQuota, type PrivacyAuditLog } from './PrivacyManager';

// Default export
export { PredictionEngine as default } from './PredictionEngine';
