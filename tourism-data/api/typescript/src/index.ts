/**
 * WIA-IND-017: Tourism Data SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

import type {
  Attraction,
  AttractionSearchParams,
  Destination,
  POI,
  POISearchParams,
  CrowdDensity,
  VisitorStatistics,
  AccessibilityInfo,
  CulturalHeritageSite,
  LocalExperience,
  SeasonalityData,
  SafetyInfo,
  GeoCoordinates,
  PaginatedResponse,
  Result,
  AsyncResult,
} from './types';

import {
  TourismDataError,
  TourismErrorCode,
} from './types';

// Re-export all types
export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

/**
 * SDK configuration options
 */
export interface TourismDataSDKConfig {
  /** API key for authentication */
  apiKey?: string;

  /** Base API URL */
  baseUrl?: string;

  /** Default language */
  language?: string;

  /** Units system */
  units?: 'metric' | 'imperial';

  /** Request timeout (ms) */
  timeout?: number;

  /** Enable caching */
  cache?: boolean;

  /** Cache TTL (seconds) */
  cacheTTL?: number;
}

/**
 * Default configuration
 */
const DEFAULT_CONFIG: Required<TourismDataSDKConfig> = {
  apiKey: '',
  baseUrl: 'https://api.wiastandards.com/tourism/v1',
  language: 'en',
  units: 'metric',
  timeout: 30000,
  cache: true,
  cacheTTL: 3600,
};

// ============================================================================
// Tourism Data SDK
// ============================================================================

/**
 * Main Tourism Data SDK class
 */
export class TourismDataSDK {
  private config: Required<TourismDataSDKConfig>;
  private cache: Map<string, { data: unknown; expires: number }>;

  constructor(config: TourismDataSDKConfig = {}) {
    this.config = { ...DEFAULT_CONFIG, ...config };
    this.cache = new Map();
  }

  // ==========================================================================
  // Attraction Methods
  // ==========================================================================

  /**
   * Search for tourist attractions
   */
  async searchAttractions(
    params: AttractionSearchParams
  ): AsyncResult<PaginatedResponse<Attraction>> {
    try {
      this.validateSearchParams(params);

      const cacheKey = `attractions:${JSON.stringify(params)}`;
      const cached = this.getFromCache<PaginatedResponse<Attraction>>(cacheKey);
      if (cached) {
        return { success: true, data: cached };
      }

      // Simulate API call (in production, this would be a real HTTP request)
      const results = this.mockAttractionSearch(params);

      this.setCache(cacheKey, results);
      return { success: true, data: results };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error : new Error('Unknown error'),
      };
    }
  }

  /**
   * Get attraction by ID
   */
  async getAttraction(id: string): AsyncResult<Attraction> {
    try {
      if (!id) {
        throw new TourismDataError(
          TourismErrorCode.ATTRACTION_NOT_FOUND,
          'Attraction ID is required'
        );
      }

      const cacheKey = `attraction:${id}`;
      const cached = this.getFromCache<Attraction>(cacheKey);
      if (cached) {
        return { success: true, data: cached };
      }

      // Mock data
      const attraction = this.mockAttraction(id);

      this.setCache(cacheKey, attraction);
      return { success: true, data: attraction };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error : new Error('Unknown error'),
      };
    }
  }

  // ==========================================================================
  // Destination Methods
  // ==========================================================================

  /**
   * Get destination information
   */
  async getDestination(params: {
    id: string;
    include?: ('attractions' | 'weather' | 'events' | 'safety')[];
  }): AsyncResult<Destination> {
    try {
      if (!params.id) {
        throw new TourismDataError(
          TourismErrorCode.DESTINATION_NOT_FOUND,
          'Destination ID is required'
        );
      }

      const cacheKey = `destination:${params.id}:${params.include?.join(',')}`;
      const cached = this.getFromCache<Destination>(cacheKey);
      if (cached) {
        return { success: true, data: cached };
      }

      const destination = this.mockDestination(params.id);

      this.setCache(cacheKey, destination);
      return { success: true, data: destination };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error : new Error('Unknown error'),
      };
    }
  }

  /**
   * Search destinations
   */
  async searchDestinations(query: string): AsyncResult<Destination[]> {
    try {
      if (!query || query.length < 2) {
        throw new TourismDataError(
          TourismErrorCode.INVALID_SEARCH_PARAMS,
          'Query must be at least 2 characters'
        );
      }

      const results = this.mockDestinationSearch(query);
      return { success: true, data: results };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error : new Error('Unknown error'),
      };
    }
  }

  // ==========================================================================
  // POI Methods
  // ==========================================================================

  /**
   * Search points of interest
   */
  searchPOI(params: POISearchParams): Result<POI[]> {
    try {
      this.validatePOISearchParams(params);

      const results = this.mockPOISearch(params);
      return { success: true, data: results };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error : new Error('Unknown error'),
      };
    }
  }

  /**
   * Get POI by ID
   */
  async getPOI(id: string): AsyncResult<POI> {
    try {
      if (!id) {
        throw new TourismDataError(
          TourismErrorCode.INVALID_SEARCH_PARAMS,
          'POI ID is required'
        );
      }

      const poi = this.mockPOI(id);
      return { success: true, data: poi };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error : new Error('Unknown error'),
      };
    }
  }

  // ==========================================================================
  // Crowd Density Methods
  // ==========================================================================

  /**
   * Get real-time crowd density
   */
  async getCrowdDensity(params: {
    attractionId: string;
    realtime?: boolean;
  }): AsyncResult<CrowdDensity> {
    try {
      if (!params.attractionId) {
        throw new TourismDataError(
          TourismErrorCode.ATTRACTION_NOT_FOUND,
          'Attraction ID is required'
        );
      }

      const crowdData = this.mockCrowdDensity(params.attractionId);
      return { success: true, data: crowdData };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error : new Error('Unknown error'),
      };
    }
  }

  // ==========================================================================
  // Visitor Statistics Methods
  // ==========================================================================

  /**
   * Get visitor statistics
   */
  async getVisitorStats(params: {
    attractionId: string;
    period: string | { start: Date; end: Date };
    granularity?: 'hourly' | 'daily' | 'weekly' | 'monthly' | 'yearly';
    metrics?: ('visitors' | 'revenue' | 'satisfaction')[];
  }): AsyncResult<VisitorStatistics> {
    try {
      if (!params.attractionId) {
        throw new TourismDataError(
          TourismErrorCode.ATTRACTION_NOT_FOUND,
          'Attraction ID is required'
        );
      }

      const stats = this.mockVisitorStats(params);
      return { success: true, data: stats };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error : new Error('Unknown error'),
      };
    }
  }

  // ==========================================================================
  // Accessibility Methods
  // ==========================================================================

  /**
   * Get accessibility information
   */
  async getAccessibilityInfo(attractionId: string): AsyncResult<AccessibilityInfo> {
    try {
      if (!attractionId) {
        throw new TourismDataError(
          TourismErrorCode.ATTRACTION_NOT_FOUND,
          'Attraction ID is required'
        );
      }

      const accessibility = this.mockAccessibility(attractionId);
      return { success: true, data: accessibility };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error : new Error('Unknown error'),
      };
    }
  }

  // ==========================================================================
  // Cultural Heritage Methods
  // ==========================================================================

  /**
   * Get cultural heritage sites
   */
  async getCulturalHeritage(params: {
    country?: string;
    unescoOnly?: boolean;
    category?: 'cultural' | 'natural' | 'mixed';
  }): AsyncResult<CulturalHeritageSite[]> {
    try {
      const sites = this.mockCulturalHeritage(params);
      return { success: true, data: sites };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error : new Error('Unknown error'),
      };
    }
  }

  // ==========================================================================
  // Local Experiences Methods
  // ==========================================================================

  /**
   * Search local experiences
   */
  async searchExperiences(params: {
    location: GeoCoordinates;
    radius?: number;
    type?: string;
    language?: string;
  }): AsyncResult<LocalExperience[]> {
    try {
      const experiences = this.mockLocalExperiences(params);
      return { success: true, data: experiences };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error : new Error('Unknown error'),
      };
    }
  }

  // ==========================================================================
  // Seasonality Methods
  // ==========================================================================

  /**
   * Get seasonality data
   */
  async getSeasonality(destinationId: string): AsyncResult<SeasonalityData> {
    try {
      if (!destinationId) {
        throw new TourismDataError(
          TourismErrorCode.DESTINATION_NOT_FOUND,
          'Destination ID is required'
        );
      }

      const seasonality = this.mockSeasonality(destinationId);
      return { success: true, data: seasonality };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error : new Error('Unknown error'),
      };
    }
  }

  // ==========================================================================
  // Safety Methods
  // ==========================================================================

  /**
   * Get safety information
   */
  async getSafetyInfo(destinationId: string): AsyncResult<SafetyInfo> {
    try {
      if (!destinationId) {
        throw new TourismDataError(
          TourismErrorCode.DESTINATION_NOT_FOUND,
          'Destination ID is required'
        );
      }

      const safety = this.mockSafetyInfo(destinationId);
      return { success: true, data: safety };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error : new Error('Unknown error'),
      };
    }
  }

  // ==========================================================================
  // Utility Methods
  // ==========================================================================

  /**
   * Calculate distance between two coordinates (Haversine formula)
   */
  static calculateDistance(
    coord1: GeoCoordinates,
    coord2: GeoCoordinates
  ): number {
    const R = 6371000; // Earth's radius in meters
    const φ1 = (coord1.lat * Math.PI) / 180;
    const φ2 = (coord2.lat * Math.PI) / 180;
    const Δφ = ((coord2.lat - coord1.lat) * Math.PI) / 180;
    const Δλ = ((coord2.lng - coord1.lng) * Math.PI) / 180;

    const a =
      Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
      Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) * Math.sin(Δλ / 2);

    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    return R * c;
  }

  /**
   * Validate coordinates
   */
  static validateCoordinates(coord: GeoCoordinates): boolean {
    return (
      coord.lat >= -90 &&
      coord.lat <= 90 &&
      coord.lng >= -180 &&
      coord.lng <= 180
    );
  }

  // ==========================================================================
  // Private Helper Methods
  // ==========================================================================

  private validateSearchParams(params: AttractionSearchParams): void {
    if (params.location && !TourismDataSDK.validateCoordinates(params.location)) {
      throw new TourismDataError(
        TourismErrorCode.INVALID_LOCATION,
        'Invalid coordinates'
      );
    }

    if (params.radius && (params.radius < 0 || params.radius > 50000)) {
      throw new TourismDataError(
        TourismErrorCode.INVALID_SEARCH_PARAMS,
        'Radius must be between 0 and 50000 meters'
      );
    }
  }

  private validatePOISearchParams(params: POISearchParams): void {
    if (params.location && !TourismDataSDK.validateCoordinates(params.location)) {
      throw new TourismDataError(
        TourismErrorCode.INVALID_LOCATION,
        'Invalid coordinates'
      );
    }
  }

  private getFromCache<T>(key: string): T | null {
    if (!this.config.cache) return null;

    const cached = this.cache.get(key);
    if (!cached) return null;

    if (Date.now() > cached.expires) {
      this.cache.delete(key);
      return null;
    }

    return cached.data as T;
  }

  private setCache(key: string, data: unknown): void {
    if (!this.config.cache) return;

    this.cache.set(key, {
      data,
      expires: Date.now() + this.config.cacheTTL * 1000,
    });
  }

  // ==========================================================================
  // Mock Data Methods (for demonstration)
  // ==========================================================================

  private mockAttractionSearch(
    params: AttractionSearchParams
  ): PaginatedResponse<Attraction> {
    const mockAttractions: Attraction[] = [
      this.mockAttraction('eiffel-tower'),
      this.mockAttraction('louvre-museum'),
    ];

    return {
      results: mockAttractions.slice(0, params.limit || 10),
      pagination: {
        total: mockAttractions.length,
        limit: params.limit || 10,
        offset: params.offset || 0,
        hasMore: false,
      },
    };
  }

  private mockAttraction(id: string): Attraction {
    return {
      id,
      name: 'Eiffel Tower',
      names: {
        en: 'Eiffel Tower',
        fr: 'Tour Eiffel',
        ja: 'エッフェル塔',
      },
      category: 'historical',
      type: 'monument',
      description: {
        en: 'Iconic iron lattice tower on the Champ de Mars in Paris',
        fr: 'Tour en treillis de fer emblématique sur le Champ de Mars à Paris',
      },
      location: {
        coordinates: { lat: 48.8584, lng: 2.2945 },
        address: {
          street: 'Champ de Mars, 5 Av. Anatole France',
          city: 'Paris',
          postalCode: '75007',
          country: 'FR',
        },
      },
      ratings: {
        overall: 4.7,
        count: 125000,
        breakdown: {
          5: 85000,
          4: 30000,
          3: 8000,
          2: 1500,
          1: 500,
        },
      },
      updatedAt: new Date(),
      quality: 'gold',
    };
  }

  private mockDestination(id: string): Destination {
    return {
      id,
      name: 'Paris',
      names: {
        en: 'Paris',
        fr: 'Paris',
        ja: 'パリ',
      },
      type: 'city',
      description: {
        en: 'The City of Light, capital of France',
        fr: 'La Ville Lumière, capitale de la France',
      },
      location: {
        coordinates: { lat: 48.8566, lng: 2.3522 },
      },
      tagline: {
        en: 'The City of Light',
        fr: 'La Ville Lumière',
      },
      population: 2161000,
      currency: {
        code: 'EUR',
        name: 'Euro',
        symbol: '€',
      },
      languages: ['fr', 'en'],
      bestSeasons: ['spring', 'fall'],
      updatedAt: new Date(),
    };
  }

  private mockDestinationSearch(query: string): Destination[] {
    return [this.mockDestination('paris-france')];
  }

  private mockPOISearch(params: POISearchParams): POI[] {
    return [
      {
        id: 'poi-001',
        name: 'Le Jules Verne',
        type: 'restaurant',
        location: {
          coordinates: { lat: 48.8584, lng: 2.2945 },
          address: {
            city: 'Paris',
            country: 'FR',
          },
        },
        rating: 4.5,
        priceRange: '$$$$',
        openNow: true,
      },
    ];
  }

  private mockPOI(id: string): POI {
    return {
      id,
      name: 'Sample POI',
      type: 'restaurant',
      location: {
        coordinates: { lat: 48.8566, lng: 2.3522 },
        address: { city: 'Paris', country: 'FR' },
      },
    };
  }

  private mockCrowdDensity(attractionId: string): CrowdDensity {
    return {
      current: 2500,
      capacity: 3000,
      percentage: 83,
      level: 'high',
      estimatedWaitTime: 45,
      timestamp: new Date(),
      forecast: [
        {
          time: new Date(Date.now() + 3600000),
          expectedLevel: 'very-high',
        },
      ],
    };
  }

  private mockVisitorStats(params: unknown): VisitorStatistics {
    return {
      id: 'eiffel-tower',
      period: {
        start: new Date('2024-01-01'),
        end: new Date('2024-12-31'),
      },
      granularity: 'monthly',
      totalVisitors: 7000000,
      dataPoints: [],
    };
  }

  private mockAccessibility(attractionId: string): AccessibilityInfo {
    return {
      wheelchair: true,
      elevator: true,
      accessibleParking: true,
      accessibleRestrooms: true,
      audioGuide: ['en', 'fr', 'es', 'de', 'it', 'ja', 'zh'],
      braille: true,
      serviceAnimals: true,
      rating: 4.5,
    };
  }

  private mockCulturalHeritage(params: unknown): CulturalHeritageSite[] {
    return [];
  }

  private mockLocalExperiences(params: unknown): LocalExperience[] {
    return [];
  }

  private mockSeasonality(destinationId: string): SeasonalityData {
    return {
      id: destinationId,
      seasons: [
        {
          name: 'spring',
          months: [3, 4, 5],
          visitorVolume: 0.7,
          priceLevel: 0.8,
          crowdLevel: 'moderate',
        },
        {
          name: 'summer',
          months: [6, 7, 8],
          visitorVolume: 1.0,
          priceLevel: 1.0,
          crowdLevel: 'very-high',
        },
      ],
    };
  }

  private mockSafetyInfo(destinationId: string): SafetyInfo {
    return {
      overall: 4.0,
      crimeLevel: 'low',
      emergency: {
        police: '17',
        ambulance: '15',
        fire: '18',
      },
    };
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Search attractions (standalone function)
 */
export async function searchAttractions(
  params: AttractionSearchParams
): AsyncResult<PaginatedResponse<Attraction>> {
  const sdk = new TourismDataSDK();
  return sdk.searchAttractions(params);
}

/**
 * Get destination information (standalone function)
 */
export async function getDestinationInfo(params: {
  id: string;
  include?: ('attractions' | 'weather' | 'events' | 'safety')[];
}): AsyncResult<Destination> {
  const sdk = new TourismDataSDK();
  return sdk.getDestination(params);
}

/**
 * Get crowd density (standalone function)
 */
export async function getCrowdDensity(params: {
  attractionId: string;
  realtime?: boolean;
}): AsyncResult<CrowdDensity> {
  const sdk = new TourismDataSDK();
  return sdk.getCrowdDensity(params);
}

/**
 * Get visitor statistics (standalone function)
 */
export async function getVisitorStats(params: {
  attractionId: string;
  period: string | { start: Date; end: Date };
  granularity?: 'hourly' | 'daily' | 'weekly' | 'monthly' | 'yearly';
  metrics?: ('visitors' | 'revenue' | 'satisfaction')[];
}): AsyncResult<VisitorStatistics> {
  const sdk = new TourismDataSDK();
  return sdk.getVisitorStats(params);
}

/**
 * Search POI (standalone function)
 */
export function searchPOI(params: POISearchParams): Result<POI[]> {
  const sdk = new TourismDataSDK();
  return sdk.searchPOI(params);
}

/**
 * Get accessibility info (standalone function)
 */
export async function getAccessibilityInfo(
  attractionId: string
): AsyncResult<AccessibilityInfo> {
  const sdk = new TourismDataSDK();
  return sdk.getAccessibilityInfo(attractionId);
}

/**
 * Get cultural heritage sites (standalone function)
 */
export async function getCulturalHeritage(params: {
  country?: string;
  unescoOnly?: boolean;
  category?: 'cultural' | 'natural' | 'mixed';
}): AsyncResult<CulturalHeritageSite[]> {
  const sdk = new TourismDataSDK();
  return sdk.getCulturalHeritage(params);
}

/**
 * Calculate distance between coordinates
 */
export function calculateDistance(
  coord1: GeoCoordinates,
  coord2: GeoCoordinates
): number {
  return TourismDataSDK.calculateDistance(coord1, coord2);
}

// ============================================================================
// 弘益人間 (홍익인간) · Benefit All Humanity
// ============================================================================
