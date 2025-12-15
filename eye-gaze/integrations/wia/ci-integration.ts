/**
 * WIA Eye Gaze Standard - CI (Cochlear Implant) Integration
 *
 * Integration with Cochlear Implant enhancement systems.
 * Enables gaze-directed audio source enhancement (cocktail party effect).
 *
 * 弘益人間 - 널리 인간을 이롭게
 *
 * @module @anthropics/wia-eye-gaze/integrations/ci
 */

import type { GazePoint, Vector2D, Vector3D } from '../api/typescript/src/types';

/**
 * 3D Gaze direction from eye tracking
 */
export interface GazeDirection {
  /** Gaze origin in 3D space (mm) */
  origin: Vector3D;
  /** Gaze direction unit vector */
  direction: Vector3D;
  /** Timestamp */
  timestamp: number;
  /** Confidence */
  confidence: number;
}

/**
 * Audio source in the environment
 */
export interface AudioSource {
  /** Source identifier */
  id: string;
  /** Source type */
  type: 'person' | 'speaker' | 'device' | 'ambient' | 'unknown';
  /** 3D position relative to user (mm) */
  position: Vector3D;
  /** Angular position (azimuth, elevation) */
  angularPosition: {
    azimuth: number; // -180 to 180 degrees
    elevation: number; // -90 to 90 degrees
  };
  /** Current volume/loudness (dB SPL) */
  loudness: number;
  /** Is speech detected? */
  isSpeech: boolean;
  /** Speaker identification */
  speakerId?: string;
  /** Label/name */
  label?: string;
}

/**
 * Audio enhancement settings for CI
 */
export interface AudioEnhancement {
  /** Source to enhance */
  sourceId: string;
  /** Gain boost (dB) */
  gainBoost: number;
  /** Noise reduction level for other sources (0-1) */
  noiseReduction: number;
  /** Beamforming strength (0-1) */
  beamforming: number;
  /** Directional focus angle (degrees) */
  focusAngle: number;
}

/**
 * CI Device capabilities
 */
export interface CICapabilities {
  /** Supports directional microphones */
  directionalMic: boolean;
  /** Number of channels/electrodes */
  channels: number;
  /** Supports beamforming */
  beamforming: boolean;
  /** Supports noise reduction */
  noiseReduction: boolean;
  /** Supports streaming audio */
  audioStreaming: boolean;
  /** Maximum gain boost (dB) */
  maxGainBoost: number;
  /** Bilateral (both ears) */
  bilateral: boolean;
  /** Device vendor */
  vendor: CIVendor;
}

/**
 * CI Device vendors
 */
export type CIVendor =
  | 'Cochlear'
  | 'Advanced Bionics'
  | 'MED-EL'
  | 'Oticon Medical'
  | 'Unknown';

/**
 * Gaze-audio alignment result
 */
export interface GazeAudioAlignment {
  /** Is user looking at an audio source? */
  aligned: boolean;
  /** Best matching source */
  source?: AudioSource;
  /** Alignment angle (degrees) */
  alignmentAngle: number;
  /** Alignment score (0-1) */
  alignmentScore: number;
  /** Dwell time on source (ms) */
  dwellTime: number;
}

/**
 * Cocktail party effect configuration
 */
export interface CocktailPartyConfig {
  /** Enable automatic source tracking */
  autoTrack: boolean;
  /** Minimum dwell time to enhance (ms) */
  dwellThreshold: number;
  /** Maximum gain boost (dB) */
  maxBoost: number;
  /** Transition smoothing time (ms) */
  transitionTime: number;
  /** Angular tolerance for alignment (degrees) */
  alignmentTolerance: number;
  /** Suppress non-gazed sources */
  suppressOthers: boolean;
  /** Suppression amount (dB) */
  suppressionAmount: number;
}

/**
 * Enhancement event
 */
export interface EnhancementEvent {
  type: 'started' | 'updated' | 'stopped';
  source?: AudioSource;
  enhancement?: AudioEnhancement;
  timestamp: number;
}

/**
 * Gaze to CI Integration
 *
 * Provides gaze-directed audio enhancement for cochlear implant users.
 * Implements the "cocktail party effect" where the audio source the user
 * is looking at is enhanced while others are suppressed.
 */
export class GazeToCI {
  /** Audio sources in the environment */
  private audioSources: Map<string, AudioSource> = new Map();

  /** Current gaze direction */
  private currentGaze: GazeDirection | null = null;

  /** Active enhancement */
  private activeEnhancement: AudioEnhancement | null = null;

  /** Dwell tracking */
  private gazeStartTime: number = 0;
  private lastAlignedSource: string | null = null;

  /** CI device capabilities */
  private capabilities: CICapabilities | null = null;

  /** Configuration */
  private config: CocktailPartyConfig = {
    autoTrack: true,
    dwellThreshold: 500, // ms
    maxBoost: 12, // dB
    transitionTime: 200, // ms
    alignmentTolerance: 15, // degrees
    suppressOthers: true,
    suppressionAmount: 6, // dB
  };

  /** Enhancement callbacks */
  private callbacks: ((event: EnhancementEvent) => void)[] = [];

  constructor(options?: Partial<CocktailPartyConfig>) {
    if (options) {
      this.config = { ...this.config, ...options };
    }
  }

  /**
   * Set CI device capabilities
   */
  setCapabilities(capabilities: CICapabilities): void {
    this.capabilities = capabilities;

    // Adjust config based on capabilities
    if (!capabilities.beamforming) {
      this.config.alignmentTolerance = 30; // Wider tolerance without beamforming
    }
    if (capabilities.maxGainBoost < this.config.maxBoost) {
      this.config.maxBoost = capabilities.maxGainBoost;
    }
  }

  /**
   * Register an audio source in the environment
   */
  registerAudioSource(source: AudioSource): void {
    this.audioSources.set(source.id, source);
  }

  /**
   * Update audio source position/properties
   */
  updateAudioSource(sourceId: string, updates: Partial<AudioSource>): void {
    const source = this.audioSources.get(sourceId);
    if (source) {
      this.audioSources.set(sourceId, { ...source, ...updates });
    }
  }

  /**
   * Remove audio source
   */
  removeAudioSource(sourceId: string): void {
    this.audioSources.delete(sourceId);
    if (this.activeEnhancement?.sourceId === sourceId) {
      this.stopEnhancement();
    }
  }

  /**
   * Enhance audio source that user is looking at
   *
   * This is the main function for the cocktail party effect.
   * It calculates which audio source aligns with the user's gaze
   * and applies enhancement to that source.
   */
  enhanceGazedAudioSource(gazeDirection: Vector3D): AudioEnhancement | null {
    // Convert to full gaze direction
    const gaze: GazeDirection = {
      origin: { x: 0, y: 0, z: 0 }, // User's head position
      direction: normalizeVector(gazeDirection),
      timestamp: Date.now(),
      confidence: 1.0,
    };

    this.currentGaze = gaze;

    // Find best aligned audio source
    const alignment = this.findAlignedSource(gaze);

    if (!alignment.aligned || !alignment.source) {
      // No aligned source - stop enhancement if dwell ended
      if (this.activeEnhancement) {
        this.stopEnhancement();
      }
      return null;
    }

    // Check dwell time
    if (alignment.source.id !== this.lastAlignedSource) {
      this.gazeStartTime = Date.now();
      this.lastAlignedSource = alignment.source.id;
    }

    const dwellTime = Date.now() - this.gazeStartTime;

    if (dwellTime < this.config.dwellThreshold) {
      // Not dwelling long enough yet
      return this.activeEnhancement;
    }

    // Calculate enhancement parameters
    const enhancement = this.calculateEnhancement(alignment);

    // Apply enhancement
    if (this.shouldUpdateEnhancement(enhancement)) {
      this.applyEnhancement(enhancement);
    }

    return enhancement;
  }

  /**
   * Find audio source aligned with gaze direction
   */
  private findAlignedSource(gaze: GazeDirection): GazeAudioAlignment {
    let bestAlignment: GazeAudioAlignment = {
      aligned: false,
      alignmentAngle: 180,
      alignmentScore: 0,
      dwellTime: 0,
    };

    for (const source of this.audioSources.values()) {
      // Calculate angle between gaze and source direction
      const sourceDirection = normalizeVector(source.position);
      const angle = angleBetweenVectors(gaze.direction, sourceDirection);

      if (angle < this.config.alignmentTolerance && angle < bestAlignment.alignmentAngle) {
        const score = 1 - angle / this.config.alignmentTolerance;
        const dwellTime =
          source.id === this.lastAlignedSource
            ? Date.now() - this.gazeStartTime
            : 0;

        bestAlignment = {
          aligned: true,
          source,
          alignmentAngle: angle,
          alignmentScore: score,
          dwellTime,
        };
      }
    }

    return bestAlignment;
  }

  /**
   * Calculate enhancement parameters
   */
  private calculateEnhancement(alignment: GazeAudioAlignment): AudioEnhancement {
    if (!alignment.source) {
      throw new Error('No source for enhancement');
    }

    // Boost increases with alignment score and dwell time
    const dwellFactor = Math.min(alignment.dwellTime / 2000, 1.0);
    const gainBoost = this.config.maxBoost * alignment.alignmentScore * dwellFactor;

    // Beamforming strength based on alignment
    const beamforming = alignment.alignmentScore;

    // Calculate focus angle from source position
    const focusAngle = Math.atan2(
      alignment.source.position.x,
      alignment.source.position.z
    ) * (180 / Math.PI);

    return {
      sourceId: alignment.source.id,
      gainBoost,
      noiseReduction: this.config.suppressOthers ? 0.7 : 0.3,
      beamforming,
      focusAngle,
    };
  }

  /**
   * Check if enhancement needs updating
   */
  private shouldUpdateEnhancement(newEnhancement: AudioEnhancement): boolean {
    if (!this.activeEnhancement) return true;
    if (this.activeEnhancement.sourceId !== newEnhancement.sourceId) return true;
    if (Math.abs(this.activeEnhancement.gainBoost - newEnhancement.gainBoost) > 1) return true;
    return false;
  }

  /**
   * Apply enhancement to CI device
   */
  private applyEnhancement(enhancement: AudioEnhancement): void {
    const isNew = !this.activeEnhancement ||
                  this.activeEnhancement.sourceId !== enhancement.sourceId;

    this.activeEnhancement = enhancement;

    // Suppress other sources if enabled
    if (this.config.suppressOthers) {
      for (const source of this.audioSources.values()) {
        if (source.id !== enhancement.sourceId) {
          this.suppressSource(source.id, this.config.suppressionAmount);
        }
      }
    }

    // Notify callbacks
    const source = this.audioSources.get(enhancement.sourceId);
    this.emitEvent({
      type: isNew ? 'started' : 'updated',
      source,
      enhancement,
      timestamp: Date.now(),
    });

    // In a real implementation, this would communicate with the CI device
    console.log(`Enhancement applied: ${enhancement.sourceId}, +${enhancement.gainBoost.toFixed(1)}dB`);
  }

  /**
   * Suppress an audio source
   */
  private suppressSource(sourceId: string, amount: number): void {
    // In a real implementation, this would communicate with the CI device
    console.log(`Suppressing source: ${sourceId}, -${amount}dB`);
  }

  /**
   * Stop current enhancement
   */
  private stopEnhancement(): void {
    if (this.activeEnhancement) {
      const source = this.audioSources.get(this.activeEnhancement.sourceId);
      this.emitEvent({
        type: 'stopped',
        source,
        enhancement: this.activeEnhancement,
        timestamp: Date.now(),
      });

      this.activeEnhancement = null;
      this.lastAlignedSource = null;
      console.log('Enhancement stopped');
    }
  }

  /**
   * Get current enhancement status
   */
  getCurrentEnhancement(): AudioEnhancement | null {
    return this.activeEnhancement;
  }

  /**
   * Get all registered audio sources
   */
  getAudioSources(): AudioSource[] {
    return Array.from(this.audioSources.values());
  }

  /**
   * Register enhancement event callback
   */
  onEnhancement(callback: (event: EnhancementEvent) => void): void {
    this.callbacks.push(callback);
  }

  private emitEvent(event: EnhancementEvent): void {
    for (const callback of this.callbacks) {
      callback(event);
    }
  }

  /**
   * Update configuration
   */
  configure(options: Partial<CocktailPartyConfig>): void {
    this.config = { ...this.config, ...options };
  }

  /**
   * Get current configuration
   */
  getConfig(): CocktailPartyConfig {
    return { ...this.config };
  }

  /**
   * Cleanup
   */
  dispose(): void {
    this.stopEnhancement();
    this.audioSources.clear();
    this.callbacks = [];
    this.currentGaze = null;
  }
}

// Vector utility functions

function normalizeVector(v: Vector3D): Vector3D {
  const magnitude = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  if (magnitude === 0) return { x: 0, y: 0, z: 1 };
  return {
    x: v.x / magnitude,
    y: v.y / magnitude,
    z: v.z / magnitude,
  };
}

function angleBetweenVectors(a: Vector3D, b: Vector3D): number {
  const dot = a.x * b.x + a.y * b.y + a.z * b.z;
  const angleRad = Math.acos(Math.max(-1, Math.min(1, dot)));
  return angleRad * (180 / Math.PI);
}

/**
 * CI Device presets
 */
export const CIDevicePresets: Record<string, CICapabilities> = {
  'cochlear-nucleus-7': {
    directionalMic: true,
    channels: 22,
    beamforming: true,
    noiseReduction: true,
    audioStreaming: true,
    maxGainBoost: 15,
    bilateral: false,
    vendor: 'Cochlear',
  },
  'advanced-bionics-marvel': {
    directionalMic: true,
    channels: 16,
    beamforming: true,
    noiseReduction: true,
    audioStreaming: true,
    maxGainBoost: 12,
    bilateral: false,
    vendor: 'Advanced Bionics',
  },
  'medel-rondo-3': {
    directionalMic: true,
    channels: 12,
    beamforming: true,
    noiseReduction: true,
    audioStreaming: true,
    maxGainBoost: 10,
    bilateral: false,
    vendor: 'MED-EL',
  },
};

export default GazeToCI;
