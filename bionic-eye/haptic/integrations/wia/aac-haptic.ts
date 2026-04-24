/**
 * WIA Haptic Standard - AAC (Augmentative and Alternative Communication) Haptic Integration
 *
 * Provides haptic feedback for AAC systems including:
 * - Symbol-based communication boards
 * - Scanning interfaces
 * - Switch access
 * - Text-to-speech confirmation
 */

import { IHapticDevice } from '../../api/typescript/src/device';
import { HapticPattern, WaveformType } from '../../api/typescript/src/types';

/**
 * AAC symbol categories
 */
export type AACSymbolCategory =
  | 'core_word'      // High-frequency words (I, you, want, go, etc.)
  | 'person'         // People (mom, dad, teacher)
  | 'action'         // Verbs
  | 'descriptor'     // Adjectives
  | 'object'         // Nouns
  | 'place'          // Locations
  | 'feeling'        // Emotions
  | 'question'       // Question words
  | 'social'         // Greetings, manners
  | 'time'           // Time words
  | 'navigation'     // UI navigation
  | 'delete'         // Backspace, clear
  | 'speak';         // Speak message

/**
 * AAC symbol representation
 */
export interface AACSymbol {
  id: string;
  label: string;
  category: AACSymbolCategory;
  imageUrl?: string;
  audioUrl?: string;
  row?: number;
  column?: number;
}

/**
 * Scanning mode types
 */
export type ScanMode =
  | 'row_column'     // Scan rows, then columns
  | 'linear'         // Scan each item sequentially
  | 'block'          // Scan groups of items
  | 'directed';      // User-directed scanning

/**
 * AAC event types
 */
export interface AACEvent {
  type: AACEventType;
  symbol?: AACSymbol;
  message?: string;
  scanPosition?: { row: number; column: number };
  timestamp: number;
}

export type AACEventType =
  | 'symbol_hover'      // Scanning over symbol
  | 'symbol_select'     // Symbol selected
  | 'symbol_delete'     // Symbol removed from message
  | 'message_speak'     // Message spoken
  | 'message_clear'     // Message cleared
  | 'row_scan'          // Row scanning active
  | 'column_scan'       // Column scanning active
  | 'block_scan'        // Block scanning
  | 'scan_start'        // Scanning started
  | 'scan_pause'        // Scanning paused
  | 'switch_press'      // Switch pressed
  | 'page_change';      // Page/board changed

/**
 * AAC haptic configuration
 */
export interface AACHapticConfig {
  // Enable scanning haptics
  scanningEnabled: boolean;

  // Enable symbol selection haptics
  selectionEnabled: boolean;

  // Enable category-based haptic differentiation
  categoryHapticsEnabled: boolean;

  // Scanning tick rate (ms)
  scanTickInterval: number;

  // Intensity levels
  intensity: {
    scan: number;
    hover: number;
    select: number;
    speak: number;
  };
}

/**
 * Default AAC haptic configuration
 */
export const DEFAULT_AAC_CONFIG: AACHapticConfig = {
  scanningEnabled: true,
  selectionEnabled: true,
  categoryHapticsEnabled: true,
  scanTickInterval: 1000,
  intensity: {
    scan: 0.3,
    hover: 0.4,
    select: 0.7,
    speak: 0.8,
  },
};

/**
 * AAC haptic patterns
 */
export const AAC_PATTERNS = {
  // Row scan tick
  ROW_SCAN: {
    id: 'aac.scan.row',
    name: 'Row Scan',
    description: 'Row scanning indicator',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 120, intensity: 0.3, duration: 50 },
    ],
    totalDuration: 50,
  } as HapticPattern,

  // Column scan tick
  COLUMN_SCAN: {
    id: 'aac.scan.column',
    name: 'Column Scan',
    description: 'Column scanning indicator',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 180, intensity: 0.4, duration: 50 },
    ],
    totalDuration: 50,
  } as HapticPattern,

  // Symbol hover
  SYMBOL_HOVER: {
    id: 'aac.symbol.hover',
    name: 'Symbol Hover',
    description: 'Hovering over symbol',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 150, intensity: 0.3, duration: 40 },
    ],
    totalDuration: 40,
  } as HapticPattern,

  // Symbol selected
  SYMBOL_SELECT: {
    id: 'aac.symbol.select',
    name: 'Symbol Selected',
    description: 'Symbol added to message',
    primitives: [
      { waveform: WaveformType.Square, frequency: 200, intensity: 0.7, duration: 80 },
      { waveform: WaveformType.Square, frequency: 200, intensity: 0.5, duration: 60, delay: 50 },
    ],
    totalDuration: 190,
  } as HapticPattern,

  // Delete/backspace
  SYMBOL_DELETE: {
    id: 'aac.symbol.delete',
    name: 'Symbol Deleted',
    description: 'Symbol removed from message',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 100, intensity: 0.5, duration: 100 },
    ],
    totalDuration: 100,
  } as HapticPattern,

  // Message spoken
  MESSAGE_SPEAK: {
    id: 'aac.message.speak',
    name: 'Message Spoken',
    description: 'Message sent to TTS',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 180, intensity: 0.7, duration: 100 },
      { waveform: WaveformType.Sine, frequency: 220, intensity: 0.8, duration: 100, delay: 50 },
      { waveform: WaveformType.Sine, frequency: 260, intensity: 0.9, duration: 150, delay: 50 },
    ],
    totalDuration: 450,
  } as HapticPattern,

  // Message cleared
  MESSAGE_CLEAR: {
    id: 'aac.message.clear',
    name: 'Message Cleared',
    description: 'Message bar cleared',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 150, intensity: 0.4, duration: 150 },
      { waveform: WaveformType.Sine, frequency: 100, intensity: 0.3, duration: 150, delay: 50 },
    ],
    totalDuration: 350,
  } as HapticPattern,

  // Switch press confirmation
  SWITCH_PRESS: {
    id: 'aac.switch.press',
    name: 'Switch Press',
    description: 'Switch input registered',
    primitives: [
      { waveform: WaveformType.Square, frequency: 180, intensity: 0.6, duration: 50 },
    ],
    totalDuration: 50,
  } as HapticPattern,

  // Page change
  PAGE_CHANGE: {
    id: 'aac.page.change',
    name: 'Page Changed',
    description: 'Board/page navigation',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 150, intensity: 0.5, duration: 100 },
      { waveform: WaveformType.Sine, frequency: 175, intensity: 0.6, duration: 100, delay: 50 },
    ],
    totalDuration: 250,
  } as HapticPattern,

  // Scan start
  SCAN_START: {
    id: 'aac.scan.start',
    name: 'Scanning Started',
    description: 'Auto-scan beginning',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 120, intensity: 0.4, duration: 100 },
      { waveform: WaveformType.Sine, frequency: 160, intensity: 0.5, duration: 100, delay: 50 },
    ],
    totalDuration: 250,
  } as HapticPattern,

  // Scan pause
  SCAN_PAUSE: {
    id: 'aac.scan.pause',
    name: 'Scanning Paused',
    description: 'Auto-scan paused',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 160, intensity: 0.4, duration: 100 },
      { waveform: WaveformType.Sine, frequency: 120, intensity: 0.3, duration: 100, delay: 50 },
    ],
    totalDuration: 250,
  } as HapticPattern,
};

/**
 * Category-specific haptic patterns
 */
export const CATEGORY_PATTERNS: Record<AACSymbolCategory, HapticPattern> = {
  core_word: {
    id: 'aac.category.core',
    name: 'Core Word',
    primitives: [{ waveform: WaveformType.Sine, frequency: 180, intensity: 0.5, duration: 60 }],
    totalDuration: 60,
  },
  person: {
    id: 'aac.category.person',
    name: 'Person',
    primitives: [{ waveform: WaveformType.Sine, frequency: 150, intensity: 0.4, duration: 80 }],
    totalDuration: 80,
  },
  action: {
    id: 'aac.category.action',
    name: 'Action',
    primitives: [
      { waveform: WaveformType.Square, frequency: 160, intensity: 0.5, duration: 40 },
      { waveform: WaveformType.Square, frequency: 160, intensity: 0.4, duration: 40, delay: 20 },
    ],
    totalDuration: 100,
  },
  descriptor: {
    id: 'aac.category.descriptor',
    name: 'Descriptor',
    primitives: [{ waveform: WaveformType.Triangle, frequency: 140, intensity: 0.4, duration: 80 }],
    totalDuration: 80,
  },
  object: {
    id: 'aac.category.object',
    name: 'Object',
    primitives: [{ waveform: WaveformType.Sine, frequency: 130, intensity: 0.4, duration: 70 }],
    totalDuration: 70,
  },
  place: {
    id: 'aac.category.place',
    name: 'Place',
    primitives: [{ waveform: WaveformType.Sine, frequency: 120, intensity: 0.5, duration: 100 }],
    totalDuration: 100,
  },
  feeling: {
    id: 'aac.category.feeling',
    name: 'Feeling',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 100, intensity: 0.3, duration: 50 },
      { waveform: WaveformType.Sine, frequency: 150, intensity: 0.5, duration: 50, delay: 30 },
    ],
    totalDuration: 130,
  },
  question: {
    id: 'aac.category.question',
    name: 'Question',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 140, intensity: 0.4, duration: 60 },
      { waveform: WaveformType.Sine, frequency: 180, intensity: 0.5, duration: 60, delay: 30 },
    ],
    totalDuration: 150,
  },
  social: {
    id: 'aac.category.social',
    name: 'Social',
    primitives: [{ waveform: WaveformType.Sine, frequency: 160, intensity: 0.5, duration: 80 }],
    totalDuration: 80,
  },
  time: {
    id: 'aac.category.time',
    name: 'Time',
    primitives: [
      { waveform: WaveformType.Square, frequency: 120, intensity: 0.3, duration: 30 },
      { waveform: WaveformType.Square, frequency: 120, intensity: 0.3, duration: 30, delay: 30 },
    ],
    totalDuration: 90,
  },
  navigation: {
    id: 'aac.category.navigation',
    name: 'Navigation',
    primitives: [{ waveform: WaveformType.Sine, frequency: 170, intensity: 0.4, duration: 50 }],
    totalDuration: 50,
  },
  delete: {
    id: 'aac.category.delete',
    name: 'Delete',
    primitives: [{ waveform: WaveformType.Sine, frequency: 100, intensity: 0.5, duration: 80 }],
    totalDuration: 80,
  },
  speak: {
    id: 'aac.category.speak',
    name: 'Speak',
    primitives: [{ waveform: WaveformType.Sine, frequency: 200, intensity: 0.7, duration: 100 }],
    totalDuration: 100,
  },
};

/**
 * AAC Haptic Integration
 *
 * Provides haptic feedback for AAC communication systems.
 *
 * @example
 * ```typescript
 * const device = await HapticDeviceManager.connect('bluetooth', deviceId);
 * const aacHaptic = new AACHaptic(device);
 *
 * // Connect to AAC app events
 * aacApp.on('event', (event) => {
 *   aacHaptic.onAACEvent(event);
 * });
 * ```
 */
export class AACHaptic {
  private device: IHapticDevice;
  private config: AACHapticConfig;
  private isActive: boolean = false;
  private scanMode: ScanMode = 'row_column';
  private currentScanPosition: { row: number; column: number } = { row: 0, column: 0 };

  constructor(
    device: IHapticDevice,
    config?: Partial<AACHapticConfig>
  ) {
    this.device = device;
    this.config = { ...DEFAULT_AAC_CONFIG, ...config };
  }

  /**
   * Start AAC haptic feedback
   */
  start(): void {
    this.isActive = true;
  }

  /**
   * Stop AAC haptic feedback
   */
  stop(): void {
    this.isActive = false;
  }

  /**
   * Set scanning mode
   */
  setScanMode(mode: ScanMode): void {
    this.scanMode = mode;
  }

  /**
   * Handle AAC event
   */
  async onAACEvent(event: AACEvent): Promise<void> {
    if (!this.isActive) return;

    switch (event.type) {
      case 'symbol_hover':
        await this.handleSymbolHover(event);
        break;
      case 'symbol_select':
        await this.handleSymbolSelect(event);
        break;
      case 'symbol_delete':
        await this.handleSymbolDelete();
        break;
      case 'message_speak':
        await this.handleMessageSpeak();
        break;
      case 'message_clear':
        await this.handleMessageClear();
        break;
      case 'row_scan':
        await this.handleRowScan(event);
        break;
      case 'column_scan':
        await this.handleColumnScan(event);
        break;
      case 'scan_start':
        await this.handleScanStart();
        break;
      case 'scan_pause':
        await this.handleScanPause();
        break;
      case 'switch_press':
        await this.handleSwitchPress();
        break;
      case 'page_change':
        await this.handlePageChange();
        break;
    }
  }

  /**
   * Handle symbol hover
   */
  private async handleSymbolHover(event: AACEvent): Promise<void> {
    if (!event.symbol) return;

    if (this.config.categoryHapticsEnabled) {
      const pattern = CATEGORY_PATTERNS[event.symbol.category];
      if (pattern) {
        await this.playPattern(pattern);
        return;
      }
    }

    await this.playPattern(AAC_PATTERNS.SYMBOL_HOVER);
  }

  /**
   * Handle symbol selection
   */
  private async handleSymbolSelect(event: AACEvent): Promise<void> {
    if (!this.config.selectionEnabled) return;
    await this.playPattern(AAC_PATTERNS.SYMBOL_SELECT);
  }

  /**
   * Handle symbol deletion
   */
  private async handleSymbolDelete(): Promise<void> {
    await this.playPattern(AAC_PATTERNS.SYMBOL_DELETE);
  }

  /**
   * Handle message speak
   */
  private async handleMessageSpeak(): Promise<void> {
    await this.playPattern(AAC_PATTERNS.MESSAGE_SPEAK);
  }

  /**
   * Handle message clear
   */
  private async handleMessageClear(): Promise<void> {
    await this.playPattern(AAC_PATTERNS.MESSAGE_CLEAR);
  }

  /**
   * Handle row scan
   */
  private async handleRowScan(event: AACEvent): Promise<void> {
    if (!this.config.scanningEnabled) return;

    if (event.scanPosition) {
      this.currentScanPosition = event.scanPosition;
    }

    await this.playPattern(AAC_PATTERNS.ROW_SCAN);
  }

  /**
   * Handle column scan
   */
  private async handleColumnScan(event: AACEvent): Promise<void> {
    if (!this.config.scanningEnabled) return;

    if (event.scanPosition) {
      this.currentScanPosition = event.scanPosition;
    }

    await this.playPattern(AAC_PATTERNS.COLUMN_SCAN);
  }

  /**
   * Handle scan start
   */
  private async handleScanStart(): Promise<void> {
    if (!this.config.scanningEnabled) return;
    await this.playPattern(AAC_PATTERNS.SCAN_START);
  }

  /**
   * Handle scan pause
   */
  private async handleScanPause(): Promise<void> {
    if (!this.config.scanningEnabled) return;
    await this.playPattern(AAC_PATTERNS.SCAN_PAUSE);
  }

  /**
   * Handle switch press
   */
  private async handleSwitchPress(): Promise<void> {
    await this.playPattern(AAC_PATTERNS.SWITCH_PRESS);
  }

  /**
   * Handle page change
   */
  private async handlePageChange(): Promise<void> {
    await this.playPattern(AAC_PATTERNS.PAGE_CHANGE);
  }

  /**
   * Create scan progress pattern
   */
  createScanProgressPattern(row: number, totalRows: number): HapticPattern {
    const progress = row / totalRows;
    const frequency = 100 + Math.round(progress * 100);

    return {
      id: `aac.scan.row.${row}`,
      name: `Row ${row} Scan`,
      primitives: [
        { waveform: WaveformType.Sine, frequency, intensity: 0.4, duration: 50 },
      ],
      totalDuration: 50,
    };
  }

  /**
   * Play haptic pattern
   */
  private async playPattern(pattern: HapticPattern): Promise<void> {
    try {
      await this.device.playPattern(pattern);
    } catch (error) {
      console.error('Failed to play haptic pattern:', error);
    }
  }
}

export default AACHaptic;
