/**
 * WIA BCI Neurofeedback Adapter
 *
 * Real-time brain activity visualization.
 */

import { BaseOutputAdapter } from './IOutputAdapter';
import {
  OutputType,
  OutputContent,
  OutputOptions,
  VisualizationMode,
  ChannelData,
  CursorPosition,
  NeurofeedbackTheme,
  OutputError,
  OutputErrorCode,
  ClassificationContent,
} from './types';
import { BandPowers } from '../types/signal';

/**
 * Neurofeedback Adapter interface
 */
export interface INeurofeedbackAdapter {
  setVisualization(mode: VisualizationMode): void;
  getVisualization(): VisualizationMode;
  updateBandPowers(powers: BandPowers): void;
  updateTopography(channels: ChannelData[]): void;
  updateClassification(result: ClassificationContent): void;
  updateCursor(position: CursorPosition): void;
  setCanvas(canvas: HTMLCanvasElement): void;
  setTheme(theme: NeurofeedbackTheme): void;
}

/**
 * Default theme
 */
const DEFAULT_THEME: NeurofeedbackTheme = {
  background: '#1a1a2e',
  foreground: '#eee',
  positive: '#4ade80',
  negative: '#f87171',
  neutral: '#94a3b8',
};

/**
 * Band colors
 */
const BAND_COLORS = {
  delta: '#6366f1', // Indigo
  theta: '#8b5cf6', // Purple
  alpha: '#22c55e', // Green
  beta: '#eab308',  // Yellow
  gamma: '#ef4444', // Red
};

/**
 * Canvas-based Neurofeedback Adapter
 */
export class CanvasNeurofeedbackAdapter extends BaseOutputAdapter implements INeurofeedbackAdapter {
  readonly type: OutputType = 'neurofeedback';
  readonly name = 'Canvas Neurofeedback';

  private canvas: HTMLCanvasElement | null = null;
  private ctx: CanvasRenderingContext2D | null = null;
  private mode: VisualizationMode = 'band_powers';
  private theme: NeurofeedbackTheme = DEFAULT_THEME;
  private animationId: number | null = null;

  // Current data
  private bandPowers: BandPowers | null = null;
  private channels: ChannelData[] = [];
  private classification: ClassificationContent | null = null;
  private cursor: CursorPosition = { x: 0, y: 0 };

  // History for time series
  private powerHistory: BandPowers[] = [];
  private maxHistory = 100;

  async initialize(options?: OutputOptions): Promise<void> {
    if (options?.theme) {
      this.theme = { ...this.theme, ...(options.theme as Partial<NeurofeedbackTheme>) };
    }
    if (options?.mode) {
      this.mode = options.mode as VisualizationMode;
    }
    this.ready = true;
    this.emit('ready', {});
  }

  setCanvas(canvas: HTMLCanvasElement): void {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    if (!this.ctx) {
      throw new OutputError(
        OutputErrorCode.NEURO_CANVAS_NOT_SET,
        'Failed to get canvas context',
        'neurofeedback'
      );
    }
    this.startRenderLoop();
  }

  setVisualization(mode: VisualizationMode): void {
    this.mode = mode;
  }

  getVisualization(): VisualizationMode {
    return this.mode;
  }

  setTheme(theme: NeurofeedbackTheme): void {
    this.theme = theme;
  }

  updateBandPowers(powers: BandPowers): void {
    this.bandPowers = powers;
    this.powerHistory.push(powers);
    if (this.powerHistory.length > this.maxHistory) {
      this.powerHistory.shift();
    }
  }

  updateTopography(channels: ChannelData[]): void {
    this.channels = channels;
  }

  updateClassification(result: ClassificationContent): void {
    this.classification = result;
  }

  updateCursor(position: CursorPosition): void {
    this.cursor = position;
  }

  async output(content: OutputContent): Promise<void> {
    if (content.type === 'signal' && content.signal) {
      this.updateBandPowers(content.signal.bandPowers);
      this.updateTopography(content.signal.channels);
    } else if (content.type === 'classification' && content.classification) {
      this.updateClassification(content.classification);
    }
  }

  private startRenderLoop(): void {
    const render = () => {
      this.render();
      this.animationId = requestAnimationFrame(render);
    };
    render();
  }

  private render(): void {
    if (!this.ctx || !this.canvas) return;

    const { width, height } = this.canvas;
    const ctx = this.ctx;

    // Clear
    ctx.fillStyle = this.theme.background;
    ctx.fillRect(0, 0, width, height);

    switch (this.mode) {
      case 'band_powers':
        this.renderBandPowers(ctx, width, height);
        break;
      case 'classification':
        this.renderClassification(ctx, width, height);
        break;
      case 'cursor':
        this.renderCursor(ctx, width, height);
        break;
      case 'time_series':
        this.renderTimeSeries(ctx, width, height);
        break;
      case 'combined':
        this.renderCombined(ctx, width, height);
        break;
      default:
        this.renderBandPowers(ctx, width, height);
    }
  }

  private renderBandPowers(ctx: CanvasRenderingContext2D, width: number, height: number): void {
    if (!this.bandPowers) return;

    const bands = ['delta', 'theta', 'alpha', 'beta', 'gamma'] as const;
    const barWidth = (width - 60) / bands.length - 10;
    const maxHeight = height - 80;

    // Title
    ctx.fillStyle = this.theme.foreground;
    ctx.font = 'bold 16px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('Band Powers', width / 2, 30);

    bands.forEach((band, i) => {
      const value = this.bandPowers![band];
      const normalizedValue = Math.min(1, value / 50); // Normalize to 0-1
      const barHeight = normalizedValue * maxHeight;
      const x = 40 + i * (barWidth + 10);
      const y = height - 40 - barHeight;

      // Bar
      ctx.fillStyle = BAND_COLORS[band];
      ctx.fillRect(x, y, barWidth, barHeight);

      // Label
      ctx.fillStyle = this.theme.foreground;
      ctx.font = '12px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(band.charAt(0).toUpperCase() + band.slice(1), x + barWidth / 2, height - 20);

      // Value
      ctx.fillText(value.toFixed(1), x + barWidth / 2, y - 5);
    });
  }

  private renderClassification(ctx: CanvasRenderingContext2D, width: number, height: number): void {
    if (!this.classification) {
      ctx.fillStyle = this.theme.neutral;
      ctx.font = '24px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('Waiting for classification...', width / 2, height / 2);
      return;
    }

    const { className, confidence } = this.classification;

    // Class name
    ctx.fillStyle = this.theme.foreground;
    ctx.font = 'bold 32px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(className.toUpperCase(), width / 2, height / 2 - 20);

    // Confidence bar
    const barWidth = width * 0.6;
    const barHeight = 30;
    const barX = (width - barWidth) / 2;
    const barY = height / 2 + 20;

    // Background
    ctx.fillStyle = this.theme.neutral;
    ctx.fillRect(barX, barY, barWidth, barHeight);

    // Fill
    const color = confidence > 0.7 ? this.theme.positive :
                  confidence > 0.4 ? this.theme.neutral : this.theme.negative;
    ctx.fillStyle = color;
    ctx.fillRect(barX, barY, barWidth * confidence, barHeight);

    // Border
    ctx.strokeStyle = this.theme.foreground;
    ctx.strokeRect(barX, barY, barWidth, barHeight);

    // Percentage
    ctx.fillStyle = this.theme.foreground;
    ctx.font = '16px sans-serif';
    ctx.fillText(`${Math.round(confidence * 100)}%`, width / 2, barY + barHeight + 25);
  }

  private renderCursor(ctx: CanvasRenderingContext2D, width: number, height: number): void {
    const centerX = width / 2;
    const centerY = height / 2;
    const maxRadius = Math.min(width, height) * 0.4;

    // Grid
    ctx.strokeStyle = this.theme.neutral;
    ctx.lineWidth = 1;

    // Horizontal line
    ctx.beginPath();
    ctx.moveTo(centerX - maxRadius, centerY);
    ctx.lineTo(centerX + maxRadius, centerY);
    ctx.stroke();

    // Vertical line
    ctx.beginPath();
    ctx.moveTo(centerX, centerY - maxRadius);
    ctx.lineTo(centerX, centerY + maxRadius);
    ctx.stroke();

    // Circle
    ctx.beginPath();
    ctx.arc(centerX, centerY, maxRadius, 0, Math.PI * 2);
    ctx.stroke();

    // Cursor
    const cursorX = centerX + this.cursor.x * maxRadius;
    const cursorY = centerY - this.cursor.y * maxRadius; // Invert Y

    ctx.fillStyle = this.cursor.click ? this.theme.positive : this.theme.foreground;
    ctx.beginPath();
    ctx.arc(cursorX, cursorY, 15, 0, Math.PI * 2);
    ctx.fill();
  }

  private renderTimeSeries(ctx: CanvasRenderingContext2D, width: number, height: number): void {
    if (this.powerHistory.length < 2) return;

    const bands = ['alpha', 'beta'] as const; // Show only alpha and beta for simplicity
    const graphHeight = (height - 60) / bands.length;

    bands.forEach((band, bandIdx) => {
      const y0 = 40 + bandIdx * graphHeight;

      // Label
      ctx.fillStyle = BAND_COLORS[band];
      ctx.font = 'bold 14px sans-serif';
      ctx.textAlign = 'left';
      ctx.fillText(band.toUpperCase(), 10, y0 + 15);

      // Draw line
      ctx.strokeStyle = BAND_COLORS[band];
      ctx.lineWidth = 2;
      ctx.beginPath();

      const xStep = (width - 40) / (this.maxHistory - 1);
      this.powerHistory.forEach((powers, i) => {
        const value = powers[band];
        const normalizedValue = Math.min(1, value / 50);
        const x = 40 + i * xStep;
        const y = y0 + graphHeight - normalizedValue * (graphHeight - 20) - 10;

        if (i === 0) {
          ctx.moveTo(x, y);
        } else {
          ctx.lineTo(x, y);
        }
      });
      ctx.stroke();
    });
  }

  private renderCombined(ctx: CanvasRenderingContext2D, width: number, height: number): void {
    // Split canvas into sections
    const halfHeight = height / 2;

    // Top: Band powers (smaller)
    ctx.save();
    ctx.beginPath();
    ctx.rect(0, 0, width, halfHeight);
    ctx.clip();
    this.renderBandPowers(ctx, width, halfHeight);
    ctx.restore();

    // Bottom: Classification
    ctx.save();
    ctx.translate(0, halfHeight);
    ctx.beginPath();
    ctx.rect(0, 0, width, halfHeight);
    ctx.clip();
    this.renderClassification(ctx, width, halfHeight);
    ctx.restore();

    // Divider
    ctx.strokeStyle = this.theme.neutral;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, halfHeight);
    ctx.lineTo(width, halfHeight);
    ctx.stroke();
  }

  async dispose(): Promise<void> {
    if (this.animationId) {
      cancelAnimationFrame(this.animationId);
      this.animationId = null;
    }
    this.canvas = null;
    this.ctx = null;
    await super.dispose();
  }
}
