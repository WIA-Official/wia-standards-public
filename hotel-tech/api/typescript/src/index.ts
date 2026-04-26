/**
 * WIA-IND-016: Hotel Tech SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry Standards Group
 *
 * 弘익人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive tools for hotel technology including:
 * - Property Management System (PMS) integration
 * - Reservation management
 * - Smart room controls
 * - Revenue management
 * - Channel manager connectivity
 * - Guest services automation
 */

import {
  PropertyConfig,
  Reservation,
  ReservationStatus,
  GuestProfile,
  Room,
  RoomStatus,
  HousekeepingStatus,
  MobileKey,
  DynamicPricingResult,
  ChannelSyncResult,
  GuestFeedback,
  ConciergeRequest,
  Recommendation,
  SmartRoomControls,
  HOTEL_CONSTANTS,
  HotelErrorCode,
  HotelTechError,
  RateCode,
  PaymentMethod,
  BookingSource,
  OccupancyForecast,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-IND-016 Hotel Tech SDK
 */
export class HotelTechSDK {
  private version = '1.0.0';
  private properties: Map<string, PropertyConfig> = new Map();
  private reservations: Map<string, Reservation> = new Map();
  private rooms: Map<string, Room> = new Map();

  constructor(private config?: { propertyId?: string; pmsProvider?: string; apiKey?: string }) {
    // SDK initialization
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  // ============================================================================
  // Property Management
  // ============================================================================

  /**
   * Register a hotel property
   *
   * @param property - Property configuration
   * @returns Registered property
   */
  registerProperty(property: PropertyConfig): PropertyConfig {
    if (!property.propertyId) {
      throw new HotelTechError(
        HotelErrorCode.PMS_CONNECTION_ERROR,
        'Property ID is required'
      );
    }

    this.properties.set(property.propertyId, property);
    return property;
  }

  /**
   * Get property by ID
   *
   * @param propertyId - Property ID
   * @returns Property configuration
   */
  getProperty(propertyId: string): PropertyConfig | undefined {
    return this.properties.get(propertyId);
  }

  // ============================================================================
  // Reservation Management
  // ============================================================================

  /**
   * Create a new reservation
   *
   * @param params - Reservation parameters
   * @returns Created reservation
   */
  async createReservation(params: {
    propertyId?: string;
    guestName: string;
    email: string;
    phone?: string;
    checkIn: string;
    checkOut: string;
    roomType: string;
    rateCode?: RateCode;
    adults: number;
    children?: number;
    specialRequests?: string;
    source?: string;
  }): Promise<Reservation> {
    const propertyId = params.propertyId || this.config?.propertyId || 'default';

    // Validate dates
    const checkIn = new Date(params.checkIn);
    const checkOut = new Date(params.checkOut);
    const nights = Math.ceil((checkOut.getTime() - checkIn.getTime()) / (1000 * 60 * 60 * 24));

    if (nights <= 0) {
      throw new HotelTechError(
        HotelErrorCode.INVALID_DATES,
        'Check-out date must be after check-in date'
      );
    }

    // Generate confirmation number
    const confirmationNumber = this.generateConfirmationNumber();

    // Calculate rates
    const roomRate = this.calculateRoomRate(params.roomType, params.rateCode || 'BAR');
    const totalAmount = roomRate * nights;
    const taxesAndFees = totalAmount * 0.15; // 15% taxes

    // Create guest profile
    const guest: GuestProfile = {
      guestId: `guest-${Date.now()}`,
      firstName: params.guestName.split(' ')[0],
      lastName: params.guestName.split(' ').slice(1).join(' ') || params.guestName,
      email: params.email,
      phone: params.phone || '',
      createdAt: new Date(),
    };

    // Create reservation
    const reservation: Reservation = {
      confirmationNumber,
      propertyId,
      guest,
      checkIn: params.checkIn,
      checkOut: params.checkOut,
      nights,
      roomType: params.roomType,
      rateCode: params.rateCode || 'BAR',
      roomRate,
      adults: params.adults,
      children: params.children || 0,
      specialRequests: params.specialRequests,
      status: 'confirmed',
      source: {
        channel: 'direct',
        source: params.source || 'website',
      },
      totalAmount: totalAmount + taxesAndFees,
      taxesAndFees,
      cancellationPolicy: {
        type: 'flexible',
        freeCancellationUntil: new Date(checkIn.getTime() - 24 * 60 * 60 * 1000).toISOString(),
      },
      createdAt: new Date(),
      updatedAt: new Date(),
    };

    this.reservations.set(confirmationNumber, reservation);
    return reservation;
  }

  /**
   * Get reservation by confirmation number
   *
   * @param confirmationNumber - Confirmation number
   * @returns Reservation or undefined
   */
  getReservation(confirmationNumber: string): Reservation | undefined {
    return this.reservations.get(confirmationNumber);
  }

  /**
   * Update reservation status
   *
   * @param confirmationNumber - Confirmation number
   * @param status - New status
   * @returns Updated reservation
   */
  updateReservationStatus(
    confirmationNumber: string,
    status: ReservationStatus
  ): Reservation {
    const reservation = this.reservations.get(confirmationNumber);
    if (!reservation) {
      throw new HotelTechError(
        HotelErrorCode.RESERVATION_NOT_FOUND,
        `Reservation ${confirmationNumber} not found`
      );
    }

    reservation.status = status;
    reservation.updatedAt = new Date();

    return reservation;
  }

  /**
   * Cancel reservation
   *
   * @param confirmationNumber - Confirmation number
   * @returns Cancellation result
   */
  async cancelReservation(confirmationNumber: string): Promise<{
    success: boolean;
    refundAmount: number;
    penalty: number;
  }> {
    const reservation = this.reservations.get(confirmationNumber);
    if (!reservation) {
      throw new HotelTechError(
        HotelErrorCode.RESERVATION_NOT_FOUND,
        `Reservation ${confirmationNumber} not found`
      );
    }

    // Check cancellation policy
    const now = new Date();
    const checkIn = new Date(reservation.checkIn);
    const freeCancellationDate = reservation.cancellationPolicy.freeCancellationUntil
      ? new Date(reservation.cancellationPolicy.freeCancellationUntil)
      : new Date(checkIn.getTime() - 24 * 60 * 60 * 1000);

    let penalty = 0;
    if (now > freeCancellationDate) {
      // Apply cancellation penalty
      penalty = reservation.totalAmount * (reservation.cancellationPolicy.penaltyPercentage || 50) / 100;
    }

    const refundAmount = reservation.totalAmount - penalty;

    // Update status
    reservation.status = 'cancelled';
    reservation.updatedAt = new Date();

    return {
      success: true,
      refundAmount,
      penalty,
    };
  }

  // ============================================================================
  // Room Management
  // ============================================================================

  /**
   * Search room availability
   *
   * @param params - Search parameters
   * @returns Available rooms
   */
  async searchAvailability(params: {
    propertyId?: string;
    checkIn: string;
    checkOut: string;
    adults: number;
    children?: number;
    roomType?: string;
  }): Promise<Array<{ type: string; available: number; rate: number }>> {
    const propertyId = params.propertyId || this.config?.propertyId || 'default';
    const property = this.properties.get(propertyId);

    if (!property) {
      throw new HotelTechError(
        HotelErrorCode.PMS_CONNECTION_ERROR,
        `Property ${propertyId} not found`
      );
    }

    // Simulate availability search
    const results = property.roomTypes.map((roomType) => ({
      type: roomType.code,
      available: Math.floor(roomType.quantity * 0.7), // 70% available
      rate: roomType.baseRate,
    }));

    return results;
  }

  /**
   * Update room status
   *
   * @param params - Room status update
   * @returns Updated room
   */
  async updateRoomStatus(params: {
    propertyId?: string;
    roomNumber: string;
    status: RoomStatus;
    inspectedBy?: string;
    timestamp?: Date;
  }): Promise<Room> {
    const propertyId = params.propertyId || this.config?.propertyId || 'default';
    const roomKey = `${propertyId}-${params.roomNumber}`;

    let room = this.rooms.get(roomKey);
    if (!room) {
      // Create new room entry
      room = {
        roomNumber: params.roomNumber,
        propertyId,
        roomType: 'standard',
        floor: parseInt(params.roomNumber.charAt(0)) || 1,
        status: params.status,
        housekeepingStatus: {
          status: params.status === 'clean' || params.status === 'available' ? 'clean' : 'dirty',
          completedAt: params.timestamp || new Date(),
          inspectedBy: params.inspectedBy,
        },
      };
    } else {
      room.status = params.status;
      room.housekeepingStatus.status = params.status === 'clean' || params.status === 'available' ? 'clean' : 'dirty';
      room.housekeepingStatus.completedAt = params.timestamp || new Date();
      room.housekeepingStatus.inspectedBy = params.inspectedBy;
    }

    this.rooms.set(roomKey, room);
    return room;
  }

  /**
   * Get room by number
   *
   * @param propertyId - Property ID
   * @param roomNumber - Room number
   * @returns Room information
   */
  getRoom(propertyId: string, roomNumber: string): Room | undefined {
    return this.rooms.get(`${propertyId}-${roomNumber}`);
  }

  // ============================================================================
  // Guest Check-in/Check-out
  // ============================================================================

  /**
   * Guest check-in
   *
   * @param confirmationNumber - Confirmation number
   * @param roomNumber - Assigned room number
   * @returns Check-in result
   */
  async checkIn(confirmationNumber: string, roomNumber: string): Promise<{
    success: boolean;
    reservation: Reservation;
    mobileKey?: MobileKey;
  }> {
    const reservation = this.reservations.get(confirmationNumber);
    if (!reservation) {
      throw new HotelTechError(
        HotelErrorCode.RESERVATION_NOT_FOUND,
        `Reservation ${confirmationNumber} not found`
      );
    }

    // Assign room
    reservation.roomNumber = roomNumber;
    reservation.status = 'checked-in';
    reservation.updatedAt = new Date();

    // Update room status
    await this.updateRoomStatus({
      propertyId: reservation.propertyId,
      roomNumber,
      status: 'occupied',
    });

    // Generate mobile key
    const mobileKey = await this.generateMobileKey({
      guestId: reservation.guest.guestId,
      roomNumber,
      validFrom: new Date(),
      validUntil: new Date(reservation.checkOut),
    });

    return {
      success: true,
      reservation,
      mobileKey,
    };
  }

  /**
   * Guest check-out
   *
   * @param confirmationNumber - Confirmation number
   * @returns Check-out result with folio
   */
  async checkOut(confirmationNumber: string): Promise<{
    success: boolean;
    totalAmount: number;
    breakdown: Array<{ description: string; amount: number }>;
  }> {
    const reservation = this.reservations.get(confirmationNumber);
    if (!reservation) {
      throw new HotelTechError(
        HotelErrorCode.RESERVATION_NOT_FOUND,
        `Reservation ${confirmationNumber} not found`
      );
    }

    // Update reservation status
    reservation.status = 'checked-out';
    reservation.updatedAt = new Date();

    // Update room status
    if (reservation.roomNumber) {
      await this.updateRoomStatus({
        propertyId: reservation.propertyId,
        roomNumber: reservation.roomNumber,
        status: 'dirty',
      });
    }

    // Generate folio breakdown
    const breakdown = [
      { description: `Room charges (${reservation.nights} nights)`, amount: reservation.roomRate * reservation.nights },
      { description: 'Taxes and fees', amount: reservation.taxesAndFees },
    ];

    // Add additional services
    if (reservation.services) {
      reservation.services.forEach((service) => {
        breakdown.push({ description: service.name, amount: service.total });
      });
    }

    const totalAmount = breakdown.reduce((sum, item) => sum + item.amount, 0);

    return {
      success: true,
      totalAmount,
      breakdown,
    };
  }

  // ============================================================================
  // Mobile Key Management
  // ============================================================================

  /**
   * Generate mobile key for guest
   *
   * @param params - Key parameters
   * @returns Generated mobile key
   */
  async generateMobileKey(params: {
    guestId: string;
    roomNumber: string;
    validFrom: Date;
    validUntil: Date;
  }): Promise<MobileKey> {
    const keyId = `key-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;

    const mobileKey: MobileKey = {
      keyId,
      guestId: params.guestId,
      roomNumber: params.roomNumber,
      validFrom: params.validFrom,
      validUntil: params.validUntil,
      keyType: 'mobile',
      accessLevel: {
        room: true,
        elevator: true,
        pool: true,
        gym: true,
      },
      status: 'active',
      usageLog: [],
    };

    return mobileKey;
  }

  // ============================================================================
  // Revenue Management & Dynamic Pricing
  // ============================================================================

  /**
   * Generate dynamic pricing recommendation
   *
   * @param params - Pricing parameters
   * @returns Dynamic pricing result
   */
  async generateDynamicRate(params: {
    propertyId?: string;
    roomType: string;
    checkIn: string;
    nights: number;
    occupancy?: number;
    events?: string[];
    competitors?: boolean;
  }): Promise<DynamicPricingResult> {
    const propertyId = params.propertyId || this.config?.propertyId || 'default';
    const baseRate = this.calculateRoomRate(params.roomType, 'BAR');

    // Calculate demand-based pricing
    const occupancyForecast = params.occupancy || 0.75;
    let demandLevel: 'low' | 'medium' | 'high' | 'very-high' = 'medium';

    if (occupancyForecast < 0.5) demandLevel = 'low';
    else if (occupancyForecast < 0.7) demandLevel = 'medium';
    else if (occupancyForecast < 0.9) demandLevel = 'high';
    else demandLevel = 'very-high';

    // Apply pricing factors
    const factors = [];
    let priceMultiplier = 1.0;

    // Occupancy factor
    if (demandLevel === 'low') {
      priceMultiplier *= 0.85;
      factors.push({ name: 'Low occupancy discount', impact: -15, description: 'Reduced rates for low demand' });
    } else if (demandLevel === 'high') {
      priceMultiplier *= 1.15;
      factors.push({ name: 'High demand premium', impact: 15, description: 'Increased rates for high demand' });
    } else if (demandLevel === 'very-high') {
      priceMultiplier *= 1.30;
      factors.push({ name: 'Peak demand premium', impact: 30, description: 'Maximum rates for peak demand' });
    }

    // Event-based pricing
    if (params.events && params.events.length > 0) {
      priceMultiplier *= 1.20;
      factors.push({ name: 'Special events', impact: 20, description: `Events: ${params.events.join(', ')}` });
    }

    // Day of week factor
    const checkInDate = new Date(params.checkIn);
    const dayOfWeek = checkInDate.getDay();
    if (dayOfWeek === 5 || dayOfWeek === 6) {
      priceMultiplier *= 1.10;
      factors.push({ name: 'Weekend premium', impact: 10, description: 'Higher rates for Friday-Saturday' });
    }

    const recommendedRate = Math.round(baseRate * priceMultiplier);

    // Competitor rates (simulated)
    const competitorRates = params.competitors
      ? [recommendedRate * 0.95, recommendedRate * 1.05, recommendedRate * 0.98]
      : undefined;

    return {
      roomType: params.roomType,
      date: params.checkIn,
      baseRate,
      recommendedRate,
      occupancyForecast,
      demandLevel,
      factors,
      competitorRates,
      confidence: 0.85,
    };
  }

  /**
   * Forecast occupancy
   *
   * @param params - Forecast parameters
   * @returns Occupancy forecast
   */
  async forecastOccupancy(params: {
    propertyId?: string;
    date: string;
  }): Promise<OccupancyForecast> {
    // Simulate occupancy forecast
    const baseOccupancy = 0.70;
    const randomVariation = (Math.random() - 0.5) * 0.2;
    const forecastedOccupancy = Math.max(0.3, Math.min(0.95, baseOccupancy + randomVariation));

    return {
      date: params.date,
      forecastedOccupancy,
      confidence: 0.80,
      factors: ['Historical data', 'Booking pace', 'Market trends'],
    };
  }

  // ============================================================================
  // Channel Manager
  // ============================================================================

  /**
   * Sync rates and inventory with channels
   *
   * @param params - Sync parameters
   * @returns Sync results
   */
  async syncChannels(params: {
    propertyId?: string;
    updateRates?: boolean;
    updateInventory?: boolean;
    channels?: string[];
  }): Promise<ChannelSyncResult[]> {
    const propertyId = params.propertyId || this.config?.propertyId || 'default';
    const channels = params.channels || ['booking.com', 'expedia', 'airbnb'];

    const results: ChannelSyncResult[] = channels.map((channelId) => ({
      channelId,
      timestamp: new Date(),
      success: true,
      roomsUpdated: params.updateInventory ? 50 : 0,
      ratesUpdated: params.updateRates ? 100 : 0,
    }));

    return results;
  }

  // ============================================================================
  // Smart Room Controls
  // ============================================================================

  /**
   * Get smart room controls
   *
   * @param roomNumber - Room number
   * @returns Smart room controls
   */
  async getSmartRoomControls(roomNumber: string): Promise<SmartRoomControls> {
    return {
      roomNumber,
      climate: {
        mode: 'auto',
        targetTemperature: 22,
        currentTemperature: 21.5,
        humidity: 45,
        fanSpeed: 'auto',
      },
      lighting: {
        scenes: [
          { name: 'Welcome', sceneId: 'welcome', settings: {} },
          { name: 'Reading', sceneId: 'reading', settings: {} },
          { name: 'Sleep', sceneId: 'sleep', settings: {} },
        ],
        currentScene: 'welcome',
      },
      curtains: {
        position: 100,
        automated: true,
      },
      entertainment: {
        tvPower: false,
        volume: 20,
        streamingServices: ['Netflix', 'YouTube', 'Amazon Prime'],
      },
      voiceAssistant: {
        enabled: true,
        type: 'alexa',
        language: 'en-US',
      },
    };
  }

  /**
   * Update smart room controls
   *
   * @param roomNumber - Room number
   * @param controls - Control updates
   * @returns Updated controls
   */
  async updateSmartRoomControls(
    roomNumber: string,
    controls: Partial<SmartRoomControls>
  ): Promise<SmartRoomControls> {
    const current = await this.getSmartRoomControls(roomNumber);
    return { ...current, ...controls };
  }

  // ============================================================================
  // Guest Feedback & Reviews
  // ============================================================================

  /**
   * Submit guest feedback
   *
   * @param feedback - Guest feedback
   * @returns Submitted feedback
   */
  async submitFeedback(feedback: Omit<GuestFeedback, 'feedbackId' | 'submittedAt'>): Promise<GuestFeedback> {
    const completeFeedback: GuestFeedback = {
      ...feedback,
      feedbackId: `feedback-${Date.now()}`,
      submittedAt: new Date(),
    };

    return completeFeedback;
  }

  // ============================================================================
  // Concierge Services
  // ============================================================================

  /**
   * Create concierge request
   *
   * @param request - Request details
   * @returns Created request
   */
  async createConciergeRequest(
    request: Omit<ConciergeRequest, 'requestId' | 'status' | 'createdAt'>
  ): Promise<ConciergeRequest> {
    const completeRequest: ConciergeRequest = {
      ...request,
      requestId: `req-${Date.now()}`,
      status: 'pending',
      createdAt: new Date(),
    };

    return completeRequest;
  }

  /**
   * Get local recommendations
   *
   * @param params - Recommendation parameters
   * @returns List of recommendations
   */
  async getRecommendations(params: {
    propertyId?: string;
    type?: 'restaurant' | 'attraction' | 'activity' | 'event';
    maxDistance?: number;
  }): Promise<Recommendation[]> {
    // Simulate recommendations
    const recommendations: Recommendation[] = [
      {
        recommendationId: 'rec-1',
        type: 'restaurant',
        name: 'The Gourmet Bistro',
        description: 'Fine dining with local cuisine',
        category: 'Fine Dining',
        distance: 500,
        rating: 4.5,
        priceLevel: 3,
        address: {
          street: '123 Main St',
          city: 'City',
          state: 'State',
          postalCode: '12345',
          country: 'Country',
        },
      },
    ];

    return recommendations;
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Generate confirmation number
   */
  private generateConfirmationNumber(): string {
    const chars = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789';
    let result = '';
    for (let i = 0; i < 6; i++) {
      result += chars.charAt(Math.floor(Math.random() * chars.length));
    }
    return result;
  }

  /**
   * Calculate room rate
   */
  private calculateRoomRate(roomType: string, rateCode: RateCode): number {
    // Base rates by room type
    const baseRates: Record<string, number> = {
      'standard': 150,
      'deluxe': 250,
      'suite': 450,
      'deluxe-king': 280,
      'deluxe-queen': 260,
      'suite-junior': 380,
      'suite-executive': 520,
      'suite-presidential': 1200,
    };

    const baseRate = baseRates[roomType] || 200;

    // Rate code adjustments
    const rateMultipliers: Record<RateCode, number> = {
      'BAR': 1.0,
      'CORP': 0.9,
      'GOV': 0.85,
      'AAA': 0.92,
      'PROMO': 0.8,
      'PKG': 1.1,
      'GROUP': 0.88,
      'LRA': 0.75,
    };

    return Math.round(baseRate * (rateMultipliers[rateCode] || 1.0));
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Create a reservation (standalone function)
 */
export async function createReservation(params: {
  propertyId?: string;
  guestName: string;
  email: string;
  checkIn: string;
  checkOut: string;
  roomType: string;
  adults: number;
}): Promise<Reservation> {
  const sdk = new HotelTechSDK();
  return sdk.createReservation(params);
}

/**
 * Update room status (standalone function)
 */
export async function updateRoomStatus(params: {
  roomNumber: string;
  status: RoomStatus;
  inspectedBy?: string;
}): Promise<Room> {
  const sdk = new HotelTechSDK();
  return sdk.updateRoomStatus(params);
}

/**
 * Generate dynamic rate (standalone function)
 */
export async function generateDynamicRate(params: {
  roomType: string;
  checkIn: string;
  nights: number;
  occupancy?: number;
}): Promise<DynamicPricingResult> {
  const sdk = new HotelTechSDK();
  return sdk.generateDynamicRate(params);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { HotelTechSDK };
export default HotelTechSDK;

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
