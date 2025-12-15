/**
 * WIA Bionic Eye API
 *
 * @packageDocumentation
 */

export * from './types';

export const VERSION = '1.0.0';

/**
 * WIA Bionic Eye API
 *
 * A comprehensive API for visual prosthesis systems including:
 * - Image capture and processing
 * - Electrode array management
 * - Stimulation pattern generation
 * - Phosphene mapping
 * - Safety systems
 *
 * ## Quick Start
 *
 * ```typescript
 * import { StimulationController, MappingStrategy } from '@wia/bionic-eye';
 *
 * const controller = new StimulationController(electrodeArray);
 * controller.setStrategy(MappingStrategy.SCOREBOARD);
 * controller.stimulateFrame(processedVisual);
 * ```
 */
