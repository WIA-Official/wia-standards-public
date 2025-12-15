/**
 * WIA Eye Gaze Standard - AAC Integration
 *
 * Integration with Augmentative and Alternative Communication (AAC) systems.
 * Enables gaze-based symbol selection, scanning, and text generation.
 *
 * 弘益人間 - 널리 인간을 이롭게
 *
 * @module @anthropics/wia-eye-gaze/integrations/aac
 */

import type {
  GazePoint,
  GazeTarget,
  GazeEvent,
  DwellController,
} from '../api/typescript/src/types';

/**
 * AAC Grid representation
 */
export interface AACGrid {
  /** Grid identifier */
  id: string;
  /** Grid name */
  name: string;
  /** Number of rows */
  rows: number;
  /** Number of columns */
  columns: number;
  /** Grid cells */
  cells: AACCell[];
  /** Scanning pattern */
  scanPattern?: ScanPattern;
}

/**
 * AAC Cell (button) in a grid
 */
export interface AACCell {
  /** Cell identifier */
  id: string;
  /** Cell position (row, column) */
  position: { row: number; column: number };
  /** Symbol to display */
  symbol?: AACSymbol;
  /** Text label */
  label: string;
  /** Action when selected */
  action: AACAction;
  /** Is this cell navigable? */
  navigable: boolean;
  /** Background color */
  backgroundColor?: string;
  /** Gaze target bounds */
  bounds?: TargetBounds;
}

/**
 * AAC Symbol representation
 */
export interface AACSymbol {
  /** Symbol ID (e.g., PCS, SymbolStix ID) */
  id: string;
  /** Symbol library/system */
  library: SymbolLibrary;
  /** Image URL or base64 data */
  imageUrl?: string;
  /** Alt text for accessibility */
  altText: string;
}

/**
 * Symbol library types
 */
export type SymbolLibrary =
  | 'PCS' // Picture Communication Symbols
  | 'SymbolStix'
  | 'ARASAAC'
  | 'Blissymbols'
  | 'Widgit'
  | 'Mulberry'
  | 'OpenSymbols'
  | 'Custom';

/**
 * AAC Action types
 */
export type AACAction =
  | { type: 'speak'; text: string; voice?: VoiceSettings }
  | { type: 'navigate'; gridId: string }
  | { type: 'back' }
  | { type: 'home' }
  | { type: 'clear' }
  | { type: 'delete' }
  | { type: 'space' }
  | { type: 'append'; text: string }
  | { type: 'custom'; handler: string; params?: Record<string, unknown> };

/**
 * Voice settings for speech synthesis
 */
export interface VoiceSettings {
  /** Voice URI/identifier */
  voiceURI?: string;
  /** Speech rate (0.1-10) */
  rate?: number;
  /** Pitch (0-2) */
  pitch?: number;
  /** Volume (0-1) */
  volume?: number;
  /** Language code */
  lang?: string;
}

/**
 * Scanning patterns for switch access
 */
export type ScanPattern =
  | 'row-column' // Scan rows, then columns
  | 'column-row' // Scan columns, then rows
  | 'linear' // Scan cell by cell
  | 'quadrant' // Divide into quadrants
  | 'block'; // Block scanning

/**
 * Target bounds for gaze detection
 */
export interface TargetBounds {
  x: number;
  y: number;
  width: number;
  height: number;
}

/**
 * Gaze-based AAC selection result
 */
export interface SelectionResult {
  /** Selected cell */
  cell: AACCell;
  /** Selection method */
  method: 'dwell' | 'blink' | 'switch' | 'gaze-gesture';
  /** Dwell duration (if dwell selection) */
  dwellDuration?: number;
  /** Confidence score */
  confidence: number;
  /** Timestamp */
  timestamp: number;
}

/**
 * Message buffer for sentence composition
 */
export interface MessageBuffer {
  /** Buffered symbols/words */
  items: BufferItem[];
  /** Current sentence text */
  text: string;
  /** Word predictions */
  predictions?: string[];
}

export interface BufferItem {
  type: 'symbol' | 'word' | 'letter';
  content: string;
  symbolId?: string;
  timestamp: number;
}

/**
 * AAC Integration callback types
 */
export type SelectionCallback = (result: SelectionResult) => void;
export type ScanCallback = (highlighted: AACCell[]) => void;
export type MessageCallback = (buffer: MessageBuffer) => void;

/**
 * Gaze to AAC Integration
 *
 * Provides gaze-based control for AAC applications including
 * symbol selection, scanning, and message composition.
 */
export class GazeToAAC {
  private grid: AACGrid | null = null;
  private dwellController: DwellController | null = null;
  private messageBuffer: MessageBuffer = { items: [], text: '' };
  private selectionCallbacks: SelectionCallback[] = [];
  private scanCallbacks: ScanCallback[] = [];
  private messageCallbacks: MessageCallback[] = [];
  private isScanning = false;
  private scanIndex = 0;
  private scanTimer: ReturnType<typeof setInterval> | null = null;

  /**
   * Configuration
   */
  private config = {
    dwellTime: 800, // ms
    scanInterval: 1500, // ms
    blinkSelectionEnabled: false,
    gazeTolerance: 0.05, // Normalized units
    wordPrediction: true,
    auditoryFeedback: true,
    visualFeedback: true,
  };

  constructor(options?: Partial<typeof GazeToAAC.prototype.config>) {
    if (options) {
      this.config = { ...this.config, ...options };
    }
  }

  /**
   * Load an AAC grid
   */
  loadGrid(grid: AACGrid): void {
    this.grid = grid;
    this.registerGazeTargets();
  }

  /**
   * Register all grid cells as gaze targets
   */
  private registerGazeTargets(): void {
    if (!this.grid || !this.dwellController) return;

    for (const cell of this.grid.cells) {
      if (cell.navigable && cell.bounds) {
        this.dwellController.registerTarget({
          elementId: cell.id,
          bounds: cell.bounds,
          label: cell.label,
          dwellTime: this.config.dwellTime,
          metadata: { cell },
        });
      }
    }
  }

  /**
   * Set the dwell controller for gaze-based selection
   */
  setDwellController(controller: DwellController): void {
    this.dwellController = controller;

    // Listen for dwell complete events
    controller.onDwellComplete((event) => {
      const cell = event.target.metadata?.cell as AACCell | undefined;
      if (cell) {
        this.handleSelection(cell, 'dwell', event.duration);
      }
    });
  }

  /**
   * Select a symbol by ID
   */
  selectSymbol(symbolId: string): void {
    if (!this.grid) return;

    const cell = this.grid.cells.find(
      (c) => c.symbol?.id === symbolId || c.id === symbolId
    );

    if (cell) {
      this.handleSelection(cell, 'gaze-gesture');
    }
  }

  /**
   * Handle cell selection
   */
  private handleSelection(
    cell: AACCell,
    method: SelectionResult['method'],
    dwellDuration?: number
  ): void {
    // Execute action
    this.executeAction(cell.action, cell);

    // Create result
    const result: SelectionResult = {
      cell,
      method,
      dwellDuration,
      confidence: 0.95,
      timestamp: Date.now(),
    };

    // Notify callbacks
    for (const callback of this.selectionCallbacks) {
      callback(result);
    }

    // Auditory feedback
    if (this.config.auditoryFeedback) {
      this.playSelectionSound();
    }
  }

  /**
   * Execute AAC action
   */
  private executeAction(action: AACAction, cell: AACCell): void {
    switch (action.type) {
      case 'speak':
        this.speak(action.text, action.voice);
        break;

      case 'append':
        this.appendToBuffer({
          type: cell.symbol ? 'symbol' : 'word',
          content: action.text,
          symbolId: cell.symbol?.id,
          timestamp: Date.now(),
        });
        break;

      case 'navigate':
        this.navigateToGrid(action.gridId);
        break;

      case 'back':
        this.navigateBack();
        break;

      case 'home':
        this.navigateHome();
        break;

      case 'clear':
        this.clearBuffer();
        break;

      case 'delete':
        this.deleteLastItem();
        break;

      case 'space':
        this.appendToBuffer({
          type: 'word',
          content: ' ',
          timestamp: Date.now(),
        });
        break;

      case 'custom':
        this.executeCustomAction(action.handler, action.params);
        break;
    }
  }

  /**
   * Speak text using speech synthesis
   */
  speak(text: string, voice?: VoiceSettings): void {
    if (typeof speechSynthesis === 'undefined') {
      console.warn('Speech synthesis not available');
      return;
    }

    const utterance = new SpeechSynthesisUtterance(text);

    if (voice) {
      if (voice.voiceURI) {
        const voices = speechSynthesis.getVoices();
        const selectedVoice = voices.find((v) => v.voiceURI === voice.voiceURI);
        if (selectedVoice) utterance.voice = selectedVoice;
      }
      if (voice.rate !== undefined) utterance.rate = voice.rate;
      if (voice.pitch !== undefined) utterance.pitch = voice.pitch;
      if (voice.volume !== undefined) utterance.volume = voice.volume;
      if (voice.lang) utterance.lang = voice.lang;
    }

    speechSynthesis.speak(utterance);
  }

  /**
   * Start gaze-based scanning
   */
  startGazeScan(grid?: AACGrid): void {
    if (grid) this.loadGrid(grid);
    if (!this.grid) return;

    this.isScanning = true;
    this.scanIndex = 0;

    const scanPattern = this.grid.scanPattern || 'row-column';

    this.scanTimer = setInterval(() => {
      if (!this.grid) return;

      const highlighted = this.getHighlightedCells(scanPattern);
      for (const callback of this.scanCallbacks) {
        callback(highlighted);
      }

      this.scanIndex++;
    }, this.config.scanInterval);
  }

  /**
   * Stop scanning
   */
  stopGazeScan(): void {
    this.isScanning = false;
    if (this.scanTimer) {
      clearInterval(this.scanTimer);
      this.scanTimer = null;
    }
  }

  /**
   * Get currently highlighted cells based on scan pattern
   */
  private getHighlightedCells(pattern: ScanPattern): AACCell[] {
    if (!this.grid) return [];

    const navigableCells = this.grid.cells.filter((c) => c.navigable);

    switch (pattern) {
      case 'row-column': {
        const rowIndex = this.scanIndex % this.grid.rows;
        return navigableCells.filter((c) => c.position.row === rowIndex);
      }

      case 'linear': {
        const cellIndex = this.scanIndex % navigableCells.length;
        return [navigableCells[cellIndex]];
      }

      default:
        return [];
    }
  }

  /**
   * Convert gaze targets to text
   */
  gazeToText(targets: GazeTarget[]): string {
    return targets
      .map((t) => {
        const cell = this.grid?.cells.find((c) => c.id === t.elementId);
        return cell?.label || '';
      })
      .join(' ')
      .trim();
  }

  /**
   * Append item to message buffer
   */
  private appendToBuffer(item: BufferItem): void {
    this.messageBuffer.items.push(item);
    this.messageBuffer.text = this.messageBuffer.items
      .map((i) => i.content)
      .join('');

    // Update predictions if enabled
    if (this.config.wordPrediction) {
      this.updatePredictions();
    }

    this.notifyMessageCallbacks();
  }

  /**
   * Clear message buffer
   */
  clearBuffer(): void {
    this.messageBuffer = { items: [], text: '' };
    this.notifyMessageCallbacks();
  }

  /**
   * Delete last item from buffer
   */
  deleteLastItem(): void {
    this.messageBuffer.items.pop();
    this.messageBuffer.text = this.messageBuffer.items
      .map((i) => i.content)
      .join('');
    this.notifyMessageCallbacks();
  }

  /**
   * Update word predictions (simplified)
   */
  private updatePredictions(): void {
    // In a real implementation, this would use a language model
    // For now, we provide basic predictions
    const lastWord = this.messageBuffer.text.split(' ').pop() || '';

    if (lastWord.length >= 2) {
      // Simplified prediction based on common words
      const commonWords = ['the', 'and', 'is', 'it', 'to', 'I', 'want', 'need', 'help', 'yes', 'no'];
      this.messageBuffer.predictions = commonWords
        .filter((w) => w.toLowerCase().startsWith(lastWord.toLowerCase()))
        .slice(0, 3);
    } else {
      this.messageBuffer.predictions = [];
    }
  }

  /**
   * Get current message buffer
   */
  getMessageBuffer(): MessageBuffer {
    return { ...this.messageBuffer };
  }

  /**
   * Speak the current message buffer
   */
  speakBuffer(voice?: VoiceSettings): void {
    if (this.messageBuffer.text) {
      this.speak(this.messageBuffer.text, voice);
    }
  }

  // Event handlers
  onSelection(callback: SelectionCallback): void {
    this.selectionCallbacks.push(callback);
  }

  onScan(callback: ScanCallback): void {
    this.scanCallbacks.push(callback);
  }

  onMessage(callback: MessageCallback): void {
    this.messageCallbacks.push(callback);
  }

  private notifyMessageCallbacks(): void {
    for (const callback of this.messageCallbacks) {
      callback(this.getMessageBuffer());
    }
  }

  // Navigation methods (would be implemented with actual navigation stack)
  private navigationStack: string[] = [];
  private homeGridId: string = '';

  setHomeGrid(gridId: string): void {
    this.homeGridId = gridId;
  }

  private navigateToGrid(gridId: string): void {
    if (this.grid) {
      this.navigationStack.push(this.grid.id);
    }
    // Load new grid (would fetch from storage/server)
    console.log(`Navigate to grid: ${gridId}`);
  }

  private navigateBack(): void {
    const previousGridId = this.navigationStack.pop();
    if (previousGridId) {
      console.log(`Navigate back to: ${previousGridId}`);
    }
  }

  private navigateHome(): void {
    this.navigationStack = [];
    if (this.homeGridId) {
      console.log(`Navigate home: ${this.homeGridId}`);
    }
  }

  private executeCustomAction(handler: string, params?: Record<string, unknown>): void {
    console.log(`Custom action: ${handler}`, params);
  }

  private playSelectionSound(): void {
    // In a real implementation, this would play an audio cue
    console.log('Selection sound played');
  }

  /**
   * Cleanup
   */
  dispose(): void {
    this.stopGazeScan();
    this.selectionCallbacks = [];
    this.scanCallbacks = [];
    this.messageCallbacks = [];
  }
}

/**
 * Create AAC grid from JSON
 */
export function createAACGrid(json: Record<string, unknown>): AACGrid {
  // Validate and transform JSON to AACGrid
  return json as unknown as AACGrid;
}

/**
 * Standard AAC grids/vocabularies
 */
export const StandardGrids = {
  /** Quick phrases grid */
  QUICK_PHRASES: 'wia-aac-quick-phrases',
  /** Core vocabulary grid */
  CORE_VOCABULARY: 'wia-aac-core-vocabulary',
  /** QWERTY keyboard grid */
  KEYBOARD_QWERTY: 'wia-aac-keyboard-qwerty',
  /** ABC keyboard grid */
  KEYBOARD_ABC: 'wia-aac-keyboard-abc',
  /** Yes/No grid */
  YES_NO: 'wia-aac-yes-no',
  /** Emotions grid */
  EMOTIONS: 'wia-aac-emotions',
  /** Basic needs grid */
  BASIC_NEEDS: 'wia-aac-basic-needs',
};

export default GazeToAAC;
