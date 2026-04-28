/**
 * WIA-IND-019: Ticketing System SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive tools for digital ticketing systems including:
 * - Ticket creation and management
 * - QR code and barcode generation
 * - Dynamic pricing algorithms
 * - Fraud prevention and validation
 * - Transfer and resale operations
 * - Multi-venue support
 * - Season pass management
 * - Access control integration
 */

import {
  Ticket,
  Event,
  Venue,
  Seat,
  TicketHolder,
  Pricing,
  Validity,
  SecurityData,
  ValidationRequest,
  ValidationResult,
  TicketTransfer,
  TransferResult,
  ResaleListing,
  ListingResult,
  PurchaseResult,
  SeasonPass,
  DynamicPricingParams,
  DynamicPricingResult,
  QRCodeParams,
  CapacityStatus,
  AnalyticsResult,
  AnalyticsMetric,
  BotScore,
  TicketingErrorCode,
  TicketingError,
} from './types';

import QRCode from 'qrcode';
import { SignJWT, jwtVerify } from 'jose';
import { ethers } from 'ethers';
import { randomBytes, createHash, createCipheriv, createDecipheriv } from 'crypto';

// ============================================================================
// Constants
// ============================================================================

const VERSION = '1.0.0';
const TOTP_INTERVAL = 30; // seconds
const SEAT_LOCK_DURATION = 600; // 10 minutes
const QR_ERROR_CORRECTION: 'H' = 'H'; // High (30% recovery)

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Generate unique ticket ID
 */
function generateTicketId(eventCode: string, section: string): string {
  const year = new Date().getFullYear();
  const sequence = Math.floor(Math.random() * 10000)
    .toString()
    .padStart(4, '0');
  return `TKT-${year}-${eventCode}-${section}-${sequence}`;
}

/**
 * Generate Time-based One-Time Password
 */
function generateTOTP(secret: string, timestamp: number = Date.now(), interval: number = TOTP_INTERVAL): string {
  const counter = Math.floor(timestamp / 1000 / interval);
  const hash = createHash('sha256')
    .update(`${secret}${counter}`)
    .digest('hex');
  const code = parseInt(hash.substring(0, 6), 16) % 1000000;
  return code.toString().padStart(6, '0');
}

/**
 * Verify TOTP code
 */
function verifyTOTP(
  secret: string,
  code: string,
  timestamp: number = Date.now(),
  interval: number = TOTP_INTERVAL,
  window: number = 1
): boolean {
  for (let i = -window; i <= window; i++) {
    const adjustedTime = timestamp + i * interval * 1000;
    const expectedCode = generateTOTP(secret, adjustedTime, interval);
    if (expectedCode === code) {
      return true;
    }
  }
  return false;
}

/**
 * Calculate geofence distance in meters
 */
function calculateDistance(
  lat1: number,
  lon1: number,
  lat2: number,
  lon2: number
): number {
  const R = 6371000; // Earth radius in meters
  const φ1 = (lat1 * Math.PI) / 180;
  const φ2 = (lat2 * Math.PI) / 180;
  const Δφ = ((lat2 - lat1) * Math.PI) / 180;
  const Δλ = ((lon2 - lon1) * Math.PI) / 180;

  const a =
    Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
    Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) * Math.sin(Δλ / 2);
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

  return R * c;
}

/**
 * Encrypt data using AES-256-GCM
 */
function encrypt(data: string, key: Buffer): string {
  const iv = randomBytes(16);
  const cipher = createCipheriv('aes-256-gcm', key, iv);

  let encrypted = cipher.update(data, 'utf8', 'hex');
  encrypted += cipher.final('hex');

  const authTag = cipher.getAuthTag();

  return JSON.stringify({
    iv: iv.toString('hex'),
    encrypted,
    authTag: authTag.toString('hex'),
  });
}

/**
 * Decrypt data using AES-256-GCM
 */
function decrypt(encryptedData: string, key: Buffer): string {
  const { iv, encrypted, authTag } = JSON.parse(encryptedData);

  const decipher = createDecipheriv(
    'aes-256-gcm',
    key,
    Buffer.from(iv, 'hex')
  );
  decipher.setAuthTag(Buffer.from(authTag, 'hex'));

  let decrypted = decipher.update(encrypted, 'hex', 'utf8');
  decrypted += decipher.final('utf8');

  return decrypted;
}

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-IND-019 Ticketing System SDK
 */
export class TicketingSDK {
  private version = VERSION;
  private apiKey?: string;
  private environment: 'development' | 'production';
  private encryptionKey: Buffer;

  constructor(config?: {
    apiKey?: string;
    environment?: 'development' | 'production';
    encryptionKey?: string;
  }) {
    this.apiKey = config?.apiKey;
    this.environment = config?.environment || 'development';
    this.encryptionKey = config?.encryptionKey
      ? Buffer.from(config.encryptionKey, 'hex')
      : randomBytes(32);
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  // ========================================================================
  // Ticket Creation
  // ========================================================================

  /**
   * Create a new ticket
   */
  async createTicket(params: {
    eventId: string;
    eventName: string;
    venue: Venue;
    seat?: Seat;
    holder: TicketHolder;
    basePrice: number;
    currency: string;
    validity: Omit<Validity, 'issueDate'>;
    transferable?: boolean;
    resellable?: boolean;
  }): Promise<Ticket> {
    const {
      eventId,
      eventName,
      venue,
      seat,
      holder,
      basePrice,
      currency,
      validity,
      transferable = true,
      resellable = true,
    } = params;

    // Generate ticket ID
    const sectionCode = seat?.section || 'GA';
    const eventCode = eventId.split('-').pop() || '000';
    const ticketId = generateTicketId(eventCode, sectionCode);

    // Generate TOTP secret
    const totpSecret = randomBytes(20).toString('base32');

    // Calculate pricing
    const taxes = basePrice * 0.1; // 10% tax
    const fees = 5.0; // $5 service fee
    const finalPrice = basePrice + taxes + fees;

    const pricing: Pricing = {
      originalPrice: basePrice,
      finalPrice,
      currency,
      taxes,
      fees,
      discounts: [],
    };

    // Generate QR code
    const qrCode = await this.generateQRCode({
      ticketId,
      eventId,
      holderEmail: holder.email || '',
      expiryDate: validity.expiryDate,
      totpSecret,
      format: 'base64',
      size: 512,
      errorCorrection: QR_ERROR_CORRECTION,
    });

    // Generate signature
    const signature = await this.signTicket({
      ticketId,
      eventId,
      holder: holder.email || holder.name,
    });

    const ticket: Ticket = {
      ticketId,
      eventId,
      eventName,
      venue,
      seat,
      holder,
      pricing,
      validity: {
        ...validity,
        issueDate: new Date().toISOString(),
      },
      security: {
        qrCode,
        signature,
        totp: {
          secret: encrypt(totpSecret, this.encryptionKey),
          interval: TOTP_INTERVAL,
        },
      },
      status: 'active',
      transferable,
      resellable,
      checkInStatus: 'not-checked-in',
      metadata: {
        issuer: 'WIA-IND-019-SDK',
        platform: 'WIA-IND-019',
        version: this.version,
      },
    };

    return ticket;
  }

  // ========================================================================
  // QR Code Generation
  // ========================================================================

  /**
   * Generate QR code for ticket
   */
  async generateQRCode(params: QRCodeParams): Promise<string> {
    const {
      ticketId,
      eventId,
      holderEmail,
      expiryDate,
      totpSecret,
      format,
      size = 512,
      errorCorrection = 'H',
    } = params;

    // Create JWT payload
    const now = Math.floor(Date.now() / 1000);
    const exp = Math.floor(new Date(expiryDate).getTime() / 1000);

    const payload = {
      tid: ticketId,
      eid: eventId,
      sub: holderEmail,
      iat: now,
      exp,
      totp: totpSecret ? generateTOTP(totpSecret) : undefined,
      ver: this.version,
    };

    // Sign JWT (using placeholder secret in this example)
    const secret = new TextEncoder().encode('wia-ind-019-secret-key');
    const jwt = await new SignJWT(payload)
      .setProtectedHeader({ alg: 'HS256', typ: 'JWT' })
      .setIssuedAt()
      .setExpirationTime(exp)
      .sign(secret);

    // Generate QR code
    const qrOptions: QRCode.QRCodeToDataURLOptions = {
      errorCorrectionLevel: errorCorrection,
      width: size,
      margin: 2,
      color: {
        dark: '#000000',
        light: '#FFFFFF',
      },
    };

    if (format === 'base64') {
      return await QRCode.toDataURL(jwt, qrOptions);
    } else if (format === 'svg') {
      return await QRCode.toString(jwt, { type: 'svg', ...qrOptions });
    } else if (format === 'terminal') {
      return await QRCode.toString(jwt, { type: 'terminal' });
    } else {
      // PNG buffer as base64
      const buffer = await QRCode.toBuffer(jwt, qrOptions);
      return buffer.toString('base64');
    }
  }

  /**
   * Verify QR code signature
   */
  async verifyQRSignature(qrCode: string): Promise<boolean> {
    try {
      const secret = new TextEncoder().encode('wia-ind-019-secret-key');
      const { payload } = await jwtVerify(qrCode, secret);

      // Check expiration
      if (payload.exp && payload.exp < Date.now() / 1000) {
        return false;
      }

      return true;
    } catch {
      return false;
    }
  }

  // ========================================================================
  // Dynamic Pricing
  // ========================================================================

  /**
   * Calculate dynamic price
   */
  calculateDynamicPrice(params: DynamicPricingParams): DynamicPricingResult {
    const {
      basePrice,
      demandFactor = 1.0,
      daysUntilEvent,
      capacityRemaining,
      marketConditions = 'normal',
    } = params;

    // Time decay factor
    let timeDecay = 1.0;
    if (daysUntilEvent > 90) timeDecay = 0.6;
    else if (daysUntilEvent > 60) timeDecay = 0.8;
    else if (daysUntilEvent > 30) timeDecay = 1.0;
    else if (daysUntilEvent > 7) timeDecay = 1.1;
    else if (daysUntilEvent > 1) timeDecay = 1.2;
    else timeDecay = 1.5;

    // Scarcity factor
    let scarcityFactor = 1.0;
    if (capacityRemaining > 0.7) scarcityFactor = 0.7;
    else if (capacityRemaining > 0.5) scarcityFactor = 0.9;
    else if (capacityRemaining > 0.25) scarcityFactor = 1.0;
    else if (capacityRemaining > 0.1) scarcityFactor = 1.5;
    else scarcityFactor = 2.5;

    // Market multiplier
    let marketMultiplier = 1.0;
    if (marketConditions === 'low') marketMultiplier = 0.8;
    else if (marketConditions === 'normal') marketMultiplier = 1.0;
    else if (marketConditions === 'high') marketMultiplier = 1.2;
    else if (marketConditions === 'very-high') marketMultiplier = 1.3;

    // Calculate final price
    const calculatedPrice =
      basePrice * demandFactor * timeDecay * scarcityFactor * marketMultiplier;

    // Apply constraints (50% - 250% of base)
    const minPrice = basePrice * 0.5;
    const maxPrice = basePrice * 2.5;
    const finalPrice = Math.max(minPrice, Math.min(maxPrice, calculatedPrice));

    return {
      calculatedPrice: finalPrice,
      basePrice,
      demandFactor,
      timeDecayFactor: timeDecay,
      scarcityFactor,
      marketMultiplier,
      breakdown: {
        demandAdjustment: basePrice * (demandFactor - 1),
        timeAdjustment: basePrice * (timeDecay - 1),
        scarcityAdjustment: basePrice * (scarcityFactor - 1),
        marketAdjustment: basePrice * (marketMultiplier - 1),
      },
    };
  }

  // ========================================================================
  // Ticket Validation
  // ========================================================================

  /**
   * Validate ticket at entry
   */
  async validateTicket(request: ValidationRequest): Promise<ValidationResult> {
    const {
      ticketId,
      validationData,
      location,
      timestamp,
      validationMethod,
    } = request;

    // Simulated ticket lookup (in production, this would query a database)
    const ticket = await this.getTicket(ticketId);

    const warnings: string[] = [];
    const errors: string[] = [];

    // Check if ticket exists
    if (!ticket) {
      return {
        isValid: false,
        ticketId,
        holder: { name: 'Unknown', verified: false },
        warnings,
        errors: ['Ticket not found'],
        action: 'deny',
        accessLevel: 'standard',
      };
    }

    // Check status
    if (ticket.status === 'used') {
      errors.push('Ticket already used');
    } else if (ticket.status === 'cancelled') {
      errors.push('Ticket cancelled');
    } else if (ticket.status === 'expired') {
      errors.push('Ticket expired');
    }

    // Check expiry
    const now = new Date(timestamp);
    const expiry = new Date(ticket.validity.expiryDate);
    if (now > expiry) {
      errors.push('Ticket expired');
    }

    // Verify QR code
    if (validationMethod === 'qr') {
      const qrValid = await this.verifyQRSignature(validationData);
      if (!qrValid) {
        errors.push('Invalid QR code');
      }
    }

    // Check geofence (if venue has coordinates)
    if (location.coordinates && ticket.venue.location.coordinates) {
      const distance = calculateDistance(
        location.coordinates.lat,
        location.coordinates.lon,
        ticket.venue.location.coordinates.lat,
        ticket.venue.location.coordinates.lon
      );

      if (distance > 100) {
        // 100 meter radius
        warnings.push('Location outside venue geofence');
      }
    }

    // Determine result
    const isValid = errors.length === 0;
    const action = isValid
      ? 'allow'
      : warnings.length > 0 && errors.length === 0
      ? 'manual-check'
      : 'deny';

    const accessLevel = ticket.seat?.type === 'VIP' ? 'vip' : ticket.seat?.accessible ? 'accessible' : 'standard';

    return {
      isValid,
      ticketId,
      holder: {
        name: ticket.holder.name,
        verified: ticket.holder.verified,
      },
      seat: ticket.seat,
      warnings,
      errors,
      action,
      accessLevel,
      checkInTime: isValid ? now : undefined,
    };
  }

  // ========================================================================
  // Ticket Transfer
  // ========================================================================

  /**
   * Transfer ticket to another person
   */
  async transferTicket(transfer: TicketTransfer): Promise<TransferResult> {
    const { ticketId, fromEmail, toEmail, requireApproval } = transfer;

    // Get ticket
    const ticket = await this.getTicket(ticketId);

    if (!ticket) {
      throw new TicketingError(
        TicketingErrorCode.INVALID_TICKET,
        'Ticket not found'
      );
    }

    // Verify ownership
    if (ticket.holder.email !== fromEmail) {
      throw new TicketingError(
        TicketingErrorCode.INVALID_TICKET,
        'Not ticket owner'
      );
    }

    // Check transferable
    if (!ticket.transferable) {
      throw new TicketingError(
        TicketingErrorCode.NOT_TRANSFERABLE,
        'Ticket is non-transferable'
      );
    }

    // Create transfer ID
    const transferId = `TXF-${Date.now()}-${randomBytes(4).toString('hex')}`;

    // In production, this would create a transfer request in database
    const status = requireApproval ? 'pending' : 'auto-approved';

    if (!requireApproval) {
      // Execute transfer immediately
      ticket.holder.email = toEmail;
      ticket.status = 'transferred';
      // Regenerate QR code
      ticket.security.qrCode = await this.generateQRCode({
        ticketId: ticket.ticketId,
        eventId: ticket.eventId,
        holderEmail: toEmail,
        expiryDate: ticket.validity.expiryDate,
        format: 'base64',
      });
    }

    return {
      success: true,
      transferId,
      status,
    };
  }

  // ========================================================================
  // Resale Marketplace
  // ========================================================================

  /**
   * List ticket for resale
   */
  async listForResale(listing: ResaleListing): Promise<ListingResult> {
    const { ticketId, askingPrice, sellerId } = listing;

    const ticket = await this.getTicket(ticketId);

    if (!ticket) {
      throw new TicketingError(
        TicketingErrorCode.INVALID_TICKET,
        'Ticket not found'
      );
    }

    if (!ticket.resellable) {
      throw new TicketingError(
        TicketingErrorCode.NOT_RESELLABLE,
        'Ticket is non-resellable'
      );
    }

    // Check price constraints (max 120% of face value)
    if (askingPrice > ticket.pricing.finalPrice * 1.2) {
      throw new TicketingError(
        TicketingErrorCode.PRICE_VIOLATION,
        'Asking price exceeds maximum markup of 120%'
      );
    }

    // Create listing ID
    const listingId = `LST-${Date.now()}-${randomBytes(4).toString('hex')}`;

    return {
      success: true,
      listingId,
      url: `https://marketplace.wia.com/listing/${listingId}`,
    };
  }

  // ========================================================================
  // Helper Methods
  // ========================================================================

  /**
   * Get ticket by ID (simulated)
   */
  private async getTicket(ticketId: string): Promise<Ticket | null> {
    // In production, this would query a database
    // For now, return null as a placeholder
    return null;
  }

  /**
   * Sign ticket data
   */
  private async signTicket(data: Record<string, unknown>): Promise<string> {
    const hash = createHash('sha256')
      .update(JSON.stringify(data))
      .digest('hex');
    return hash;
  }

  /**
   * Check seat availability
   */
  async checkSeatAvailability(
    eventId: string,
    section: string
  ): Promise<CapacityStatus> {
    // Simulated capacity check
    return {
      total: 1000,
      sold: 675,
      reserved: 50,
      available: 275,
      blocked: 0,
      utilization: 0.675,
    };
  }

  /**
   * Get analytics for event
   */
  async getAnalytics(
    eventId: string,
    metrics: AnalyticsMetric[]
  ): Promise<AnalyticsResult> {
    // Simulated analytics
    return {
      eventId,
      totalTicketsSold: 15432,
      revenue: 2314800.0,
      currentAttendance: 12890,
      peakHour: '21:00',
      demographics: {
        age: {
          '18-24': 25,
          '25-34': 35,
          '35-44': 20,
          '45-54': 12,
          '55+': 8,
        },
        gender: {
          male: 48,
          female: 50,
          other: 2,
        },
        location: {
          local: 60,
          regional: 30,
          international: 10,
        },
      },
      conversionRate: 0.68,
    };
  }

  /**
   * Detect bot activity
   */
  async detectBot(userAgent: string, fingerprint: string): Promise<BotScore> {
    // Simulated bot detection
    // In production, this would use ML models

    const signals = {
      userAgent: userAgent.includes('bot') ? 1.0 : 0.0,
      ipReputation: 0.5,
      browserFingerprint: fingerprint,
      mouseMovements: 0.7,
      keystrokeDynamics: 0.6,
      pageNavigation: 'normal',
      requestSpeed: 0.3,
      formFillTime: 0.4,
    };

    const probability =
      (signals.userAgent +
        signals.ipReputation +
        signals.mouseMovements +
        signals.keystrokeDynamics +
        signals.requestSpeed +
        signals.formFillTime) /
      6;

    const action: 'allow' | 'challenge' | 'block' =
      probability > 0.7 ? 'block' : probability > 0.4 ? 'challenge' : 'allow';

    return {
      probability,
      signals,
      action,
    };
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Create a new ticketing SDK instance
 */
export function createTicketingSDK(config?: {
  apiKey?: string;
  environment?: 'development' | 'production';
  encryptionKey?: string;
}): TicketingSDK {
  return new TicketingSDK(config);
}

/**
 * Calculate dynamic price (standalone)
 */
export function calculateDynamicPrice(
  params: DynamicPricingParams
): DynamicPricingResult {
  const sdk = new TicketingSDK();
  return sdk.calculateDynamicPrice(params);
}

/**
 * Generate TOTP code (standalone)
 */
export function generateTOTPCode(
  secret: string,
  timestamp?: number
): string {
  return generateTOTP(secret, timestamp);
}

/**
 * Verify TOTP code (standalone)
 */
export function verifyTOTPCode(
  secret: string,
  code: string,
  timestamp?: number
): boolean {
  return verifyTOTP(secret, code, timestamp);
}

// ============================================================================
// Export All
// ============================================================================

export * from './types';

export {
  TicketingSDK,
  generateTicketId,
  generateTOTP,
  verifyTOTP,
  calculateDistance,
};

/**
 * Default export
 */
export default TicketingSDK;

/**
 * 弘익人間 (홍익인간) · Benefit All Humanity
 */
