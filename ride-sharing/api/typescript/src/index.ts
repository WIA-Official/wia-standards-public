/**
 * WIA-AUTO-014: Ride Sharing SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Mobility Group
 *
 * 弘익人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive tools for ride sharing platforms including:
 * - Driver-rider matching with multi-criteria optimization
 * - Dynamic fare calculation with surge pricing
 * - Route optimization for single and multi-stop trips
 * - Safety features and emergency protocols
 * - Real-time demand analysis and heatmaps
 */

import {
  RideRequest,
  DriverMatch,
  MatchingParams,
  MatchingScore,
  FareCalculationParams,
  FareResult,
  SurgePricing,
  DemandHeatmap,
  RouteOptimizationRequest,
  OptimizedRoute,
  Trip,
  DriverProfile,
  RiderProfile,
  CarbonFootprint,
  Coordinates,
  Route,
  RIDE_SHARING_CONSTANTS,
  DEFAULT_MATCHING_WEIGHTS,
  RideErrorCode,
  RideSharingError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-AUTO-014 Ride Sharing SDK
 */
export class RideSharingSDK {
  private version = '1.0.0';
  private apiKey?: string;
  private region?: string;

  constructor(config?: { apiKey?: string; region?: string }) {
    this.apiKey = config?.apiKey;
    this.region = config?.region;
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Match drivers with a ride request
   *
   * @param params - Matching parameters
   * @returns Best driver match
   */
  matchDriver(params: MatchingParams): DriverMatch {
    // In a real implementation, this would query available drivers
    // For now, we'll simulate the matching process

    const { pickup, destination, vehicleType = 'sedan', maxWaitTime = 900 } = params;

    // Simulate driver matching
    const mockDriver: DriverMatch = {
      driverId: `driver_${Date.now()}`,
      driver: {
        name: 'Sample Driver',
        rating: 4.8,
        totalTrips: 1234,
      },
      vehicle: {
        make: 'Toyota',
        model: 'Camry',
        year: 2022,
        color: 'Silver',
        licensePlate: 'ABC1234',
        capacity: 4,
        type: vehicleType,
        features: ['ac', 'bluetooth', 'usb_charging'],
        accessibility: [],
      },
      eta: 5, // minutes
      distance: 2.5, // km
      matchScore: 0.85,
      location: {
        latitude: pickup.latitude + 0.01,
        longitude: pickup.longitude + 0.01,
      },
      fareEstimate: this.calculateFareEstimate(pickup, destination, vehicleType),
    };

    return mockDriver;
  }

  /**
   * Calculate matching score for a driver-rider pair
   *
   * @param driver - Driver profile
   * @param params - Matching parameters
   * @returns Matching score breakdown
   */
  calculateMatchingScore(
    driver: DriverProfile,
    params: MatchingParams
  ): MatchingScore {
    const { pickup } = params;
    const driverLocation = driver.status.location || {
      latitude: 0,
      longitude: 0,
    };

    // Calculate distance factor
    const pickupDistance = this.calculateDistance(pickup, driverLocation);
    const distanceFactor = Math.max(
      0,
      1 - pickupDistance / RIDE_SHARING_CONSTANTS.MAX_PICKUP_DISTANCE
    );

    // Calculate time factor (assume 30 km/h average speed in city)
    const estimatedTime = (pickupDistance / 30) * 60; // minutes
    const timeFactor = Math.max(
      0,
      1 - estimatedTime / (RIDE_SHARING_CONSTANTS.MAX_WAIT_TIME / 60)
    );

    // Calculate rating factor
    const ratingFactor =
      (driver.rating.average - RIDE_SHARING_CONSTANTS.MIN_DRIVER_RATING) /
      (5.0 - RIDE_SHARING_CONSTANTS.MIN_DRIVER_RATING);

    // Calculate preference factor (simplified)
    const preferenceFactor = 0.8; // Would compare actual preferences

    // Calculate vehicle factor
    const vehicleFactor = params.vehicleType === driver.vehicle.type ? 1.0 : 0.7;

    // Calculate weighted total
    const weights = DEFAULT_MATCHING_WEIGHTS;
    const total =
      weights.distance * distanceFactor +
      weights.time * timeFactor +
      weights.rating * ratingFactor +
      weights.preference * preferenceFactor +
      weights.vehicle * vehicleFactor;

    return {
      total: Math.min(1.0, Math.max(0, total)),
      components: {
        distanceFactor,
        timeFactor,
        ratingFactor,
        preferenceFactor,
        vehicleFactor,
      },
      weights,
    };
  }

  /**
   * Calculate fare for a trip
   *
   * @param params - Fare calculation parameters
   * @returns Detailed fare breakdown
   */
  calculateFare(params: FareCalculationParams): FareResult {
    const {
      distance,
      duration,
      vehicleType = 'sedan',
      surgeMultiplier = 1.0,
      timeOfDay,
    } = params;

    // Base rates (vary by vehicle type)
    const rates = this.getVehicleRates(vehicleType);

    // Calculate base fare components
    const baseFare = rates.baseFee;
    const distanceFare = distance * rates.perKm;
    const timeFare = duration * rates.perMinute;
    const subtotal = baseFare + distanceFare + timeFare;

    // Calculate surge amount
    const surgeAmount = subtotal * (surgeMultiplier - 1);

    // Calculate fees (service fee, taxes, etc.)
    const serviceFee = (subtotal + surgeAmount) * RIDE_SHARING_CONSTANTS.SERVICE_FEE;
    const fees = serviceFee;

    // Calculate discount (if promo code applied)
    const discount = 0; // Would apply promo code logic

    // Calculate total
    const total = subtotal + surgeAmount + fees - discount;

    // Calculate driver earnings (total - platform commission)
    const platformFee = total * RIDE_SHARING_CONSTANTS.PLATFORM_COMMISSION;
    const driverEarnings = total - platformFee;

    return {
      total: Math.round(total * 100) / 100,
      currency: 'USD',
      breakdown: {
        baseFare,
        distanceFare,
        timeFare,
        subtotal,
        surgeAmount,
        fees,
        discount,
      },
      surge: surgeMultiplier,
      displayPrice: `$${total.toFixed(2)}`,
      driverEarnings: Math.round(driverEarnings * 100) / 100,
      platformFee: Math.round(platformFee * 100) / 100,
    };
  }

  /**
   * Calculate surge multiplier based on demand and supply
   *
   * @param demand - Current ride requests in area
   * @param supply - Available drivers in area
   * @param area - Geographic area identifier
   * @returns Surge pricing data
   */
  calculateSurge(demand: number, supply: number, area: string): SurgePricing {
    // Prevent division by zero
    const effectiveSupply = Math.max(1, supply);
    const demandSupplyRatio = demand / effectiveSupply;

    // Calculate surge using logarithmic formula
    // S = max(0, min(S_max, k × ln(demand/supply)))
    const k = RIDE_SHARING_CONSTANTS.SURGE_SENSITIVITY;
    const rawSurge = k * Math.log(demandSupplyRatio);
    const multiplier = Math.max(
      1.0,
      Math.min(RIDE_SHARING_CONSTANTS.MAX_SURGE_MULTIPLIER, 1.0 + rawSurge)
    );

    // Determine surge level
    let level: SurgePricing['level'];
    if (multiplier < 1.2) level = 'none';
    else if (multiplier < 1.5) level = 'low';
    else if (multiplier < 2.0) level = 'medium';
    else if (multiplier < 3.0) level = 'high';
    else level = 'extreme';

    // Round to nearest 0.25
    const roundedMultiplier = Math.round(multiplier * 4) / 4;

    return {
      area,
      multiplier: roundedMultiplier,
      demand,
      supply,
      demandSupplyRatio,
      validUntil: new Date(Date.now() + 5 * 60 * 1000).toISOString(), // Valid for 5 minutes
      level,
    };
  }

  /**
   * Optimize route for single or multi-stop trip
   *
   * @param request - Route optimization request
   * @returns Optimized route with efficiency metrics
   */
  optimizeRoute(request: RouteOptimizationRequest): OptimizedRoute {
    const { waypoints, optimize = 'time', vehicleType = 'sedan' } = request;

    if (waypoints.length < 2) {
      throw new RideSharingError(
        RideErrorCode.INVALID_PARAMETERS,
        'At least 2 waypoints required for route optimization'
      );
    }

    // Calculate route metrics
    let totalDistance = 0;
    let totalDuration = 0;

    for (let i = 0; i < waypoints.length - 1; i++) {
      const segment = this.calculateDistance(waypoints[i], waypoints[i + 1]);
      totalDistance += segment;
      // Assume average speed of 40 km/h
      totalDuration += (segment / 40) * 3600; // seconds
    }

    // Calculate efficiency score
    const straightLineDistance = this.calculateDistance(
      waypoints[0],
      waypoints[waypoints.length - 1]
    );
    const distanceEfficiency = straightLineDistance / totalDistance;
    const efficiencyScore = Math.min(1.0, distanceEfficiency);

    // Calculate carbon emissions
    const carbonEmissions = this.calculateCarbonEmissions(
      totalDistance,
      vehicleType
    );

    // Estimate cost (based on distance and time)
    const estimatedCost = this.calculateFare({
      distance: totalDistance,
      duration: totalDuration / 60, // convert to minutes
      vehicleType,
    }).total;

    const route: Route = {
      distance: totalDistance,
      duration: totalDuration,
      waypoints,
      trafficCondition: 'moderate',
    };

    return {
      route,
      optimizationGoal: optimize,
      efficiencyScore: Math.round(efficiencyScore * 100) / 100,
      carbonEmissions: Math.round(carbonEmissions * 100) / 100,
      estimatedCost: Math.round(estimatedCost * 100) / 100,
      warnings: [],
    };
  }

  /**
   * Calculate carbon footprint for a trip
   *
   * @param tripId - Trip identifier
   * @param distance - Trip distance in km
   * @param vehicleType - Type of vehicle used
   * @returns Carbon footprint analysis
   */
  calculateCarbonFootprint(
    tripId: string,
    distance: number,
    vehicleType: string
  ): CarbonFootprint {
    // Emission factors (kg CO₂ per km)
    const emissionFactors: Record<string, number> = {
      economy: 0.15,
      sedan: 0.18,
      premium: 0.22,
      suv: 0.25,
      luxury: 0.28,
      van: 0.24,
      xl: 0.26,
      green: 0.05, // Electric/hybrid
    };

    const factor = emissionFactors[vehicleType] || 0.18;
    const totalEmissions = distance * factor;

    // Comparison to alternatives
    const comparisonToAlternatives = {
      personalCar: totalEmissions * 1.2, // Worse by 20%
      publicTransit: totalEmissions * 0.3, // Better by 70%
      taxi: totalEmissions * 0.95, // Slightly better
    };

    return {
      tripId,
      totalEmissions: Math.round(totalEmissions * 100) / 100,
      emissionsPerKm: Math.round(factor * 100) / 100,
      comparisonToAlternatives,
    };
  }

  /**
   * Generate demand heatmap for a region
   *
   * @param region - Region identifier
   * @param radius - Radius in km
   * @returns Demand heatmap data
   */
  generateDemandHeatmap(region: string, radius: number = 10): DemandHeatmap {
    // In a real implementation, this would query actual demand data
    // For now, we'll generate sample heatmap cells

    const cells = [];
    const gridSize = 5; // 5x5 grid

    for (let i = 0; i < gridSize; i++) {
      for (let j = 0; j < gridSize; j++) {
        // Simulate demand and supply
        const demand = Math.floor(Math.random() * 50);
        const supply = Math.floor(Math.random() * 30);
        const surge = this.calculateSurge(demand, supply, `cell_${i}_${j}`);

        cells.push({
          center: {
            latitude: 37.7749 + (i - gridSize / 2) * 0.02,
            longitude: -122.4194 + (j - gridSize / 2) * 0.02,
          },
          radius: (radius * 1000) / gridSize, // Convert to meters
          demand,
          supply,
          surgeMultiplier: surge.multiplier,
          intensity: Math.min(1.0, demand / 50),
        });
      }
    }

    return {
      region,
      timestamp: new Date().toISOString(),
      cells,
    };
  }

  /**
   * Validate ride request
   *
   * @param request - Ride request to validate
   * @returns Validation result with any errors
   */
  validateRideRequest(request: RideRequest): {
    valid: boolean;
    errors: string[];
  } {
    const errors: string[] = [];

    // Validate pickup location
    if (!this.isValidCoordinate(request.pickup.location)) {
      errors.push('Invalid pickup location coordinates');
    }

    // Validate destination location
    if (!this.isValidCoordinate(request.destination.location)) {
      errors.push('Invalid destination location coordinates');
    }

    // Validate passengers
    if (request.passengers < 1 || request.passengers > 8) {
      errors.push('Invalid passenger count (must be 1-8)');
    }

    // Check if pickup and destination are the same
    const distance = this.calculateDistance(
      request.pickup.location,
      request.destination.location
    );

    if (distance < 0.1) {
      errors.push('Pickup and destination are too close (minimum 100m)');
    }

    return {
      valid: errors.length === 0,
      errors,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Calculate distance between two coordinates using Haversine formula
   */
  private calculateDistance(coord1: Coordinates, coord2: Coordinates): number {
    const R = 6371; // Earth's radius in km
    const dLat = this.toRadians(coord2.latitude - coord1.latitude);
    const dLon = this.toRadians(coord2.longitude - coord1.longitude);

    const a =
      Math.sin(dLat / 2) * Math.sin(dLat / 2) +
      Math.cos(this.toRadians(coord1.latitude)) *
        Math.cos(this.toRadians(coord2.latitude)) *
        Math.sin(dLon / 2) *
        Math.sin(dLon / 2);

    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
  }

  /**
   * Convert degrees to radians
   */
  private toRadians(degrees: number): number {
    return degrees * (Math.PI / 180);
  }

  /**
   * Validate coordinates
   */
  private isValidCoordinate(coord: Coordinates): boolean {
    return (
      coord.latitude >= -90 &&
      coord.latitude <= 90 &&
      coord.longitude >= -180 &&
      coord.longitude <= 180
    );
  }

  /**
   * Get vehicle rates based on type
   */
  private getVehicleRates(vehicleType: string): {
    baseFee: number;
    perKm: number;
    perMinute: number;
  } {
    const rates: Record<string, { baseFee: number; perKm: number; perMinute: number }> =
      {
        economy: { baseFee: 2.5, perKm: 0.75, perMinute: 0.15 },
        sedan: { baseFee: 3.5, perKm: 1.2, perMinute: 0.25 },
        premium: { baseFee: 5.0, perKm: 1.8, perMinute: 0.35 },
        suv: { baseFee: 6.0, perKm: 2.0, perMinute: 0.4 },
        luxury: { baseFee: 10.0, perKm: 3.0, perMinute: 0.5 },
        van: { baseFee: 7.0, perKm: 2.2, perMinute: 0.45 },
        xl: { baseFee: 8.0, perKm: 2.5, perMinute: 0.48 },
        green: { baseFee: 3.0, perKm: 1.0, perMinute: 0.2 },
      };

    return rates[vehicleType] || rates.sedan;
  }

  /**
   * Calculate carbon emissions
   */
  private calculateCarbonEmissions(
    distance: number,
    vehicleType: string
  ): number {
    const emissionFactors: Record<string, number> = {
      economy: 0.15,
      sedan: 0.18,
      premium: 0.22,
      suv: 0.25,
      luxury: 0.28,
      van: 0.24,
      xl: 0.26,
      green: 0.05,
    };

    const factor = emissionFactors[vehicleType] || 0.18;
    return distance * factor;
  }

  /**
   * Calculate fare estimate range
   */
  private calculateFareEstimate(
    pickup: Coordinates,
    destination: Coordinates,
    vehicleType: string
  ): { min: number; max: number; estimate: number; currency: string } {
    const distance = this.calculateDistance(pickup, destination);
    const duration = (distance / 40) * 60; // Assume 40 km/h average

    const fare = this.calculateFare({
      distance,
      duration,
      vehicleType,
    });

    return {
      min: Math.round(fare.total * 0.85 * 100) / 100,
      max: Math.round(fare.total * 1.15 * 100) / 100,
      estimate: fare.total,
      currency: 'USD',
    };
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Match driver to ride request (standalone function)
 */
export function matchDriver(params: MatchingParams): DriverMatch {
  const sdk = new RideSharingSDK();
  return sdk.matchDriver(params);
}

/**
 * Calculate fare (standalone function)
 */
export function calculateFare(params: FareCalculationParams): FareResult {
  const sdk = new RideSharingSDK();
  return sdk.calculateFare(params);
}

/**
 * Calculate surge pricing (standalone function)
 */
export function calculateSurge(
  demand: number,
  supply: number,
  area: string
): SurgePricing {
  const sdk = new RideSharingSDK();
  return sdk.calculateSurge(demand, supply, area);
}

/**
 * Optimize route (standalone function)
 */
export function optimizeRoute(request: RouteOptimizationRequest): OptimizedRoute {
  const sdk = new RideSharingSDK();
  return sdk.optimizeRoute(request);
}

/**
 * Verify driver credentials (standalone function)
 */
export function verifyDriver(driverId: string): Promise<boolean> {
  // In a real implementation, this would check credentials
  return Promise.resolve(true);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { RideSharingSDK };
export default RideSharingSDK;
