/**
 * WIA BCI Error Classes
 * @module wia-bci/core/BciError
 */

/**
 * Error Codes
 */
export const ErrorCodes = {
  CONNECTION_FAILED: 'E001',
  DEVICE_NOT_FOUND: 'E002',
  PERMISSION_DENIED: 'E003',
  STREAM_ERROR: 'E004',
  INVALID_CONFIG: 'E005',
  ADAPTER_ERROR: 'E006',
  TIMEOUT: 'E007',
  DISCONNECTED: 'E008',
  NOT_CONNECTED: 'E009',
  ALREADY_CONNECTED: 'E010',
  INVALID_STATE: 'E011',
} as const;

export type ErrorCode = (typeof ErrorCodes)[keyof typeof ErrorCodes];

/**
 * BCI Error Class
 */
export class BciError extends Error {
  public readonly code: ErrorCode;
  public readonly recoverable: boolean;
  public readonly details?: unknown;

  constructor(
    code: ErrorCode,
    message: string,
    options?: {
      recoverable?: boolean;
      details?: unknown;
      cause?: Error;
    }
  ) {
    super(message, { cause: options?.cause });
    this.name = 'BciError';
    this.code = code;
    this.recoverable = options?.recoverable ?? false;
    this.details = options?.details;

    // Maintains proper stack trace for where error was thrown
    if (Error.captureStackTrace) {
      Error.captureStackTrace(this, BciError);
    }
  }

  /**
   * Convert to ErrorEvent format
   */
  toEvent() {
    return {
      code: this.code,
      message: this.message,
      details: this.details,
      recoverable: this.recoverable,
    };
  }

  /**
   * Create ConnectionFailed error
   */
  static connectionFailed(message: string, details?: unknown): BciError {
    return new BciError(ErrorCodes.CONNECTION_FAILED, message, {
      recoverable: true,
      details,
    });
  }

  /**
   * Create DeviceNotFound error
   */
  static deviceNotFound(message = 'Device not found'): BciError {
    return new BciError(ErrorCodes.DEVICE_NOT_FOUND, message, {
      recoverable: false,
    });
  }

  /**
   * Create NotConnected error
   */
  static notConnected(): BciError {
    return new BciError(ErrorCodes.NOT_CONNECTED, 'Not connected to any device', {
      recoverable: false,
    });
  }

  /**
   * Create AlreadyConnected error
   */
  static alreadyConnected(): BciError {
    return new BciError(ErrorCodes.ALREADY_CONNECTED, 'Already connected to a device', {
      recoverable: false,
    });
  }

  /**
   * Create InvalidConfig error
   */
  static invalidConfig(message: string, details?: unknown): BciError {
    return new BciError(ErrorCodes.INVALID_CONFIG, message, {
      recoverable: false,
      details,
    });
  }

  /**
   * Create Timeout error
   */
  static timeout(message = 'Operation timed out'): BciError {
    return new BciError(ErrorCodes.TIMEOUT, message, {
      recoverable: true,
    });
  }

  /**
   * Create StreamError
   */
  static streamError(message: string, details?: unknown): BciError {
    return new BciError(ErrorCodes.STREAM_ERROR, message, {
      recoverable: true,
      details,
    });
  }
}
