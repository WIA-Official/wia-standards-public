/**
 * WIA-IND-015: Travel Tech SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

import type {
  TravelTechConfig,
  FlightSearchRequest,
  FlightSearchResult,
  HotelSearchRequest,
  HotelSearchResult,
  TransportSearchRequest,
  TransportOption,
  DocumentVerificationRequest,
  DocumentVerificationResult,
  CurrencyConversionRequest,
  CurrencyConversionResult,
  TravelItinerary,
  ItineraryItem,
  InsurancePolicy,
  LoyaltyAccount,
  TravelAlert,
  Traveler,
  SpecialRequirements,
  ApiResult,
  MoneyAmount,
  AirportCode,
  DateString,
  CurrencyCode,
} from './types';

export * from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-IND-015 Travel Tech SDK
 *
 * Comprehensive travel technology integration for flights, hotels,
 * transportation, documents, and travel services.
 */
export class TravelTechSDK {
  private config: TravelTechConfig;
  private baseUrl: string;

  constructor(config: TravelTechConfig) {
    this.config = config;
    this.baseUrl = config.baseUrl || this.getDefaultBaseUrl();
  }

  private getDefaultBaseUrl(): string {
    switch (this.config.environment) {
      case 'production':
        return 'https://api.wiastandards.com/ind-015/v1';
      case 'sandbox':
        return 'https://sandbox-api.wiastandards.com/ind-015/v1';
      case 'development':
        return 'http://localhost:3000/ind-015/v1';
      default:
        return 'https://api.wiastandards.com/ind-015/v1';
    }
  }

  // ==========================================================================
  // Flight Services
  // ==========================================================================

  /**
   * Search for flights
   */
  async searchFlights(
    request: FlightSearchRequest
  ): Promise<ApiResult<FlightSearchResult>> {
    return this.makeRequest<FlightSearchResult>('/flights/search', 'POST', request);
  }

  /**
   * Get flight details by ID
   */
  async getFlightDetails(flightId: string): Promise<ApiResult<any>> {
    return this.makeRequest(`/flights/${flightId}`, 'GET');
  }

  /**
   * Book a flight
   */
  async bookFlight(
    flightId: string,
    travelers: Traveler[],
    payment: any
  ): Promise<ApiResult<any>> {
    return this.makeRequest('/flights/book', 'POST', {
      flightId,
      travelers,
      payment,
    });
  }

  /**
   * Cancel flight booking
   */
  async cancelFlight(bookingReference: string): Promise<ApiResult<any>> {
    return this.makeRequest(`/flights/bookings/${bookingReference}/cancel`, 'POST');
  }

  /**
   * Check flight status
   */
  async checkFlightStatus(flightNumber: string, date: DateString): Promise<ApiResult<any>> {
    return this.makeRequest('/flights/status', 'GET', { flightNumber, date });
  }

  // ==========================================================================
  // Hotel Services
  // ==========================================================================

  /**
   * Search for hotels
   */
  async searchHotels(
    request: HotelSearchRequest
  ): Promise<ApiResult<HotelSearchResult>> {
    return this.makeRequest<HotelSearchResult>('/hotels/search', 'POST', request);
  }

  /**
   * Get hotel details
   */
  async getHotelDetails(hotelId: string): Promise<ApiResult<any>> {
    return this.makeRequest(`/hotels/${hotelId}`, 'GET');
  }

  /**
   * Book a hotel
   */
  async bookHotel(
    hotelId: string,
    roomId: string,
    travelers: Traveler[],
    payment: any
  ): Promise<ApiResult<any>> {
    return this.makeRequest('/hotels/book', 'POST', {
      hotelId,
      roomId,
      travelers,
      payment,
    });
  }

  /**
   * Cancel hotel booking
   */
  async cancelHotel(bookingReference: string): Promise<ApiResult<any>> {
    return this.makeRequest(`/hotels/bookings/${bookingReference}/cancel`, 'POST');
  }

  // ==========================================================================
  // Transportation Services
  // ==========================================================================

  /**
   * Search for transportation options
   */
  async searchTransportation(
    request: TransportSearchRequest
  ): Promise<ApiResult<TransportOption[]>> {
    return this.makeRequest<TransportOption[]>('/transport/search', 'POST', request);
  }

  /**
   * Book transportation
   */
  async bookTransportation(
    optionId: string,
    travelers: Traveler[],
    payment: any
  ): Promise<ApiResult<any>> {
    return this.makeRequest('/transport/book', 'POST', {
      optionId,
      travelers,
      payment,
    });
  }

  // ==========================================================================
  // Travel Documents
  // ==========================================================================

  /**
   * Verify travel documents and requirements
   */
  async checkTravelDocuments(
    request: DocumentVerificationRequest
  ): Promise<ApiResult<DocumentVerificationResult>> {
    return this.makeRequest<DocumentVerificationResult>(
      '/documents/verify',
      'POST',
      request
    );
  }

  /**
   * Get visa requirements for destination
   */
  async getVisaRequirements(
    passport: string,
    destination: string
  ): Promise<ApiResult<any>> {
    return this.makeRequest('/documents/visa-requirements', 'GET', {
      passport,
      destination,
    });
  }

  /**
   * Get vaccine requirements for destination
   */
  async getVaccineRequirements(destination: string): Promise<ApiResult<any>> {
    return this.makeRequest('/documents/vaccine-requirements', 'GET', {
      destination,
    });
  }

  // ==========================================================================
  // Currency and Payment
  // ==========================================================================

  /**
   * Convert currency
   */
  async convertCurrency(
    request: CurrencyConversionRequest
  ): Promise<ApiResult<CurrencyConversionResult>> {
    return this.makeRequest<CurrencyConversionResult>(
      '/currency/convert',
      'POST',
      request
    );
  }

  /**
   * Get exchange rates
   */
  async getExchangeRates(base: CurrencyCode): Promise<ApiResult<any>> {
    return this.makeRequest('/currency/rates', 'GET', { base });
  }

  /**
   * Process payment
   */
  async processPayment(payment: any): Promise<ApiResult<any>> {
    return this.makeRequest('/payments/process', 'POST', payment);
  }

  // ==========================================================================
  // Itinerary Management
  // ==========================================================================

  /**
   * Create a new itinerary
   */
  async createItinerary(itinerary: Partial<TravelItinerary>): Promise<ApiResult<TravelItinerary>> {
    return this.makeRequest<TravelItinerary>('/itineraries', 'POST', itinerary);
  }

  /**
   * Get itinerary by ID
   */
  async getItinerary(itineraryId: string): Promise<ApiResult<TravelItinerary>> {
    return this.makeRequest<TravelItinerary>(`/itineraries/${itineraryId}`, 'GET');
  }

  /**
   * Update itinerary
   */
  async updateItinerary(
    itineraryId: string,
    updates: Partial<TravelItinerary>
  ): Promise<ApiResult<TravelItinerary>> {
    return this.makeRequest<TravelItinerary>(
      `/itineraries/${itineraryId}`,
      'PATCH',
      updates
    );
  }

  /**
   * Add item to itinerary
   */
  async addItineraryItem(
    itineraryId: string,
    item: ItineraryItem
  ): Promise<ApiResult<TravelItinerary>> {
    return this.makeRequest<TravelItinerary>(
      `/itineraries/${itineraryId}/items`,
      'POST',
      item
    );
  }

  /**
   * Delete itinerary
   */
  async deleteItinerary(itineraryId: string): Promise<ApiResult<void>> {
    return this.makeRequest<void>(`/itineraries/${itineraryId}`, 'DELETE');
  }

  // ==========================================================================
  // Insurance Services
  // ==========================================================================

  /**
   * Search for travel insurance policies
   */
  async searchInsurance(criteria: any): Promise<ApiResult<InsurancePolicy[]>> {
    return this.makeRequest<InsurancePolicy[]>('/insurance/search', 'POST', criteria);
  }

  /**
   * Purchase insurance policy
   */
  async purchaseInsurance(policyId: string, travelers: Traveler[]): Promise<ApiResult<any>> {
    return this.makeRequest('/insurance/purchase', 'POST', {
      policyId,
      travelers,
    });
  }

  /**
   * File insurance claim
   */
  async fileInsuranceClaim(claim: any): Promise<ApiResult<any>> {
    return this.makeRequest('/insurance/claims', 'POST', claim);
  }

  // ==========================================================================
  // Loyalty Programs
  // ==========================================================================

  /**
   * Get loyalty account details
   */
  async getLoyaltyAccount(accountId: string): Promise<ApiResult<LoyaltyAccount>> {
    return this.makeRequest<LoyaltyAccount>(`/loyalty/accounts/${accountId}`, 'GET');
  }

  /**
   * Get loyalty balance
   */
  async getLoyaltyBalance(programId: string, memberNumber: string): Promise<ApiResult<any>> {
    return this.makeRequest('/loyalty/balance', 'GET', {
      programId,
      memberNumber,
    });
  }

  /**
   * Redeem loyalty points
   */
  async redeemLoyaltyPoints(
    accountId: string,
    points: number,
    redemptionType: string
  ): Promise<ApiResult<any>> {
    return this.makeRequest('/loyalty/redeem', 'POST', {
      accountId,
      points,
      redemptionType,
    });
  }

  // ==========================================================================
  // Alerts and Notifications
  // ==========================================================================

  /**
   * Get travel alerts for destination
   */
  async getTravelAlerts(destination: string): Promise<ApiResult<TravelAlert[]>> {
    return this.makeRequest<TravelAlert[]>('/alerts', 'GET', { destination });
  }

  /**
   * Subscribe to alerts for booking
   */
  async subscribeToAlerts(
    bookingReference: string,
    channels: string[]
  ): Promise<ApiResult<any>> {
    return this.makeRequest('/alerts/subscribe', 'POST', {
      bookingReference,
      channels,
    });
  }

  // ==========================================================================
  // Accessibility Services
  // ==========================================================================

  /**
   * Find accessible travel options
   */
  async findAccessibleOptions(
    destination: string,
    requirements: SpecialRequirements
  ): Promise<ApiResult<any>> {
    return this.makeRequest('/accessibility/search', 'POST', {
      destination,
      requirements,
    });
  }

  /**
   * Request special assistance
   */
  async requestAssistance(
    bookingReference: string,
    assistance: SpecialRequirements
  ): Promise<ApiResult<any>> {
    return this.makeRequest('/accessibility/request', 'POST', {
      bookingReference,
      assistance,
    });
  }

  // ==========================================================================
  // Package Deals
  // ==========================================================================

  /**
   * Search for travel packages (flight + hotel)
   */
  async searchPackages(criteria: any): Promise<ApiResult<any>> {
    return this.makeRequest('/packages/search', 'POST', criteria);
  }

  /**
   * Book a travel package
   */
  async bookPackage(
    packageId: string,
    travelers: Traveler[],
    payment: any
  ): Promise<ApiResult<any>> {
    return this.makeRequest('/packages/book', 'POST', {
      packageId,
      travelers,
      payment,
    });
  }

  // ==========================================================================
  // Utility Methods
  // ==========================================================================

  /**
   * Make HTTP request to API
   */
  private async makeRequest<T>(
    endpoint: string,
    method: string = 'GET',
    data?: any
  ): Promise<ApiResult<T>> {
    const url = `${this.baseUrl}${endpoint}`;
    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Version': '1.0.0',
    };

    if (this.config.defaultLanguage) {
      headers['Accept-Language'] = this.config.defaultLanguage;
    }

    try {
      const options: RequestInit = {
        method,
        headers,
      };

      if (data && (method === 'POST' || method === 'PATCH' || method === 'PUT')) {
        options.body = JSON.stringify(data);
      } else if (data && method === 'GET') {
        const params = new URLSearchParams(data);
        const fullUrl = `${url}?${params}`;
        const response = await fetch(fullUrl, options);
        return this.handleResponse<T>(response);
      }

      const response = await fetch(url, options);
      return this.handleResponse<T>(response);
    } catch (error) {
      return {
        success: false,
        error: {
          code: 'NETWORK_ERROR',
          message: error instanceof Error ? error.message : 'Unknown error',
        },
      };
    }
  }

  /**
   * Handle API response
   */
  private async handleResponse<T>(response: Response): Promise<ApiResult<T>> {
    const contentType = response.headers.get('content-type');
    const isJson = contentType?.includes('application/json');

    if (!response.ok) {
      const errorBody = isJson ? await response.json() : await response.text();
      return {
        success: false,
        error: {
          code: `HTTP_${response.status}`,
          message: response.statusText,
          details: errorBody,
        },
      };
    }

    if (isJson) {
      const data = await response.json();
      return {
        success: true,
        data: data as T,
      };
    }

    return {
      success: true,
      data: null as T,
    };
  }
}

// ============================================================================
// Standalone Helper Functions
// ============================================================================

/**
 * Search for flights (standalone function)
 */
export async function searchFlights(
  request: FlightSearchRequest,
  apiKey: string,
  environment: 'production' | 'sandbox' | 'development' = 'production'
): Promise<ApiResult<FlightSearchResult>> {
  const sdk = new TravelTechSDK({ apiKey, environment });
  return sdk.searchFlights(request);
}

/**
 * Search for hotels (standalone function)
 */
export async function searchHotels(
  request: HotelSearchRequest,
  apiKey: string,
  environment: 'production' | 'sandbox' | 'development' = 'production'
): Promise<ApiResult<HotelSearchResult>> {
  const sdk = new TravelTechSDK({ apiKey, environment });
  return sdk.searchHotels(request);
}

/**
 * Check travel documents (standalone function)
 */
export async function checkTravelDocuments(
  request: DocumentVerificationRequest,
  apiKey: string,
  environment: 'production' | 'sandbox' | 'development' = 'production'
): Promise<ApiResult<DocumentVerificationResult>> {
  const sdk = new TravelTechSDK({ apiKey, environment });
  return sdk.checkTravelDocuments(request);
}

/**
 * Convert currency (standalone function)
 */
export async function convertCurrency(
  request: CurrencyConversionRequest,
  apiKey: string,
  environment: 'production' | 'sandbox' | 'development' = 'production'
): Promise<ApiResult<CurrencyConversionResult>> {
  const sdk = new TravelTechSDK({ apiKey, environment });
  return sdk.convertCurrency(request);
}

/**
 * Create itinerary (standalone function)
 */
export async function createItinerary(
  itinerary: Partial<TravelItinerary>,
  apiKey: string,
  environment: 'production' | 'sandbox' | 'development' = 'production'
): Promise<ApiResult<TravelItinerary>> {
  const sdk = new TravelTechSDK({ apiKey, environment });
  return sdk.createItinerary(itinerary);
}

/**
 * Find accessible options (standalone function)
 */
export async function findAccessibleOptions(
  destination: string,
  requirements: SpecialRequirements,
  apiKey: string,
  environment: 'production' | 'sandbox' | 'development' = 'production'
): Promise<ApiResult<any>> {
  const sdk = new TravelTechSDK({ apiKey, environment });
  return sdk.findAccessibleOptions(destination, requirements);
}

// ============================================================================
// Calculation Utilities
// ============================================================================

/**
 * Calculate total travel duration (in hours)
 */
export function calculateTravelDuration(
  departureTime: string,
  arrivalTime: string
): number {
  const departure = new Date(departureTime);
  const arrival = new Date(arrivalTime);
  const durationMs = arrival.getTime() - departure.getTime();
  return durationMs / (1000 * 60 * 60); // Convert to hours
}

/**
 * Calculate number of nights between dates
 */
export function calculateNights(checkIn: DateString, checkOut: DateString): number {
  const start = new Date(checkIn);
  const end = new Date(checkOut);
  const diffTime = end.getTime() - start.getTime();
  return Math.ceil(diffTime / (1000 * 60 * 60 * 24));
}

/**
 * Format money amount
 */
export function formatMoney(amount: MoneyAmount): string {
  const formatter = new Intl.NumberFormat('en-US', {
    style: 'currency',
    currency: amount.currency,
  });
  return formatter.format(amount.amount);
}

/**
 * Calculate price per night
 */
export function calculatePricePerNight(
  totalPrice: MoneyAmount,
  checkIn: DateString,
  checkOut: DateString
): MoneyAmount {
  const nights = calculateNights(checkIn, checkOut);
  return {
    amount: totalPrice.amount / nights,
    currency: totalPrice.currency,
  };
}

/**
 * Validate passport expiry (must be valid 6 months beyond travel)
 */
export function validatePassportExpiry(
  expiryDate: DateString,
  travelDate: DateString
): { valid: boolean; message?: string } {
  const expiry = new Date(expiryDate);
  const travel = new Date(travelDate);
  const sixMonthsAfterTravel = new Date(travel);
  sixMonthsAfterTravel.setMonth(sixMonthsAfterTravel.getMonth() + 6);

  if (expiry < travel) {
    return {
      valid: false,
      message: 'Passport has expired',
    };
  }

  if (expiry < sixMonthsAfterTravel) {
    return {
      valid: false,
      message: 'Passport must be valid for at least 6 months beyond travel date',
    };
  }

  return { valid: true };
}

/**
 * Calculate carbon footprint for flight (rough estimate in kg CO2)
 */
export function calculateFlightCarbon(distanceKm: number, passengers: number = 1): number {
  // Average: 0.115 kg CO2 per passenger per km
  const CO2_PER_KM = 0.115;
  return distanceKm * passengers * CO2_PER_KM;
}

/**
 * Get airport timezone (simplified - would use real timezone database)
 */
export function getAirportTimezone(airportCode: AirportCode): string {
  // This is a simplified version. In production, use a proper timezone database
  const timezones: Record<string, string> = {
    JFK: 'America/New_York',
    LAX: 'America/Los_Angeles',
    LHR: 'Europe/London',
    CDG: 'Europe/Paris',
    NRT: 'Asia/Tokyo',
    SYD: 'Australia/Sydney',
  };
  return timezones[airportCode] || 'UTC';
}

/**
 * Calculate layover time between flights (in minutes)
 */
export function calculateLayover(
  flight1Arrival: string,
  flight2Departure: string
): number {
  const arrival = new Date(flight1Arrival);
  const departure = new Date(flight2Departure);
  const diffMs = departure.getTime() - arrival.getTime();
  return Math.floor(diffMs / (1000 * 60)); // Convert to minutes
}

/**
 * Check if layover is sufficient (minimum 45 min domestic, 90 min international)
 */
export function isSufficientLayover(
  layoverMinutes: number,
  isInternational: boolean = false
): { sufficient: boolean; message?: string } {
  const minRequired = isInternational ? 90 : 45;

  if (layoverMinutes < minRequired) {
    return {
      sufficient: false,
      message: `Layover of ${layoverMinutes} minutes is less than recommended ${minRequired} minutes`,
    };
  }

  return { sufficient: true };
}

// Export default
export default TravelTechSDK;

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
