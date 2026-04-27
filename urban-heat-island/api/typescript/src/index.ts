/**
 * WIA Urban Heat Island Standard - TypeScript SDK
 *
 * SDK for urban temperature management and heat mitigation
 *
 * @module @wia/urban-heat-island
 * @version 1.0.0
 */

import {
  Coordinate,
  TemperatureReading,
  TemperatureMap,
  TemperatureGridCell,
  HeatHotspot,
  HotspotDetectionConfig,
  HotspotSeverity,
  SurfaceMaterial,
  SurfaceMaterialType,
  AlbedoData,
  VegetationIndex,
  VegetationCoverage,
  CoolingStrategy,
  CoolingStrategyType,
  StrategyEvaluation,
  MitigationPlan,
  MitigationPhase,
  ThermalComfortInput,
  ThermalComfortResult,
  ThermalComfortIndex,
  TemperatureTrend,
  UHIIntensity,
  HeatEvent,
  HeatEventType,
  HeatEventHandler,
  UHIConfig,
} from './types';

/**
 * WIA Urban Heat Island Management System
 *
 * Comprehensive system for monitoring, analyzing, and mitigating urban heat islands
 */
export class WIAUrbanHeatIsland {
  private config: UHIConfig;
  private temperatureData: TemperatureReading[] = [];
  private hotspots: Map<string, HeatHotspot> = new Map();
  private strategies: Map<string, CoolingStrategy> = new Map();
  private mitigationPlans: Map<string, MitigationPlan> = new Map();
  private eventHandlers: Map<HeatEventType, HeatEventHandler[]> = new Map();
  private monitoringInterval?: NodeJS.Timeout;

  constructor(config: UHIConfig) {
    this.config = config;
    this.initializeEventHandlers();
  }

  // ============================================================================
  // Temperature Monitoring
  // ============================================================================

  /**
   * Record a temperature reading
   */
  public recordTemperature(reading: TemperatureReading): void {
    this.temperatureData.push(reading);

    // Check for threshold violations
    if (this.exceedsThreshold(reading)) {
      this.emitEvent({
        id: this.generateId(),
        type: 'temperature-threshold-exceeded',
        severity: 'warning',
        location: reading.location,
        data: { reading },
        timestamp: new Date(),
        message: `Temperature ${reading.value}°${reading.unit} exceeds threshold at location`,
      });
    }
  }

  /**
   * Generate temperature map for a region
   */
  public async generateTemperatureMap(
    bounds: TemperatureMap['bounds'],
    resolution: number = 100
  ): Promise<TemperatureMap> {
    const grid: TemperatureGridCell[][] = [];

    // Calculate grid dimensions
    const latStep = resolution / 111000; // approximately meters to degrees
    const lonStep = resolution / (111000 * Math.cos(bounds.north * Math.PI / 180));

    for (let lat = bounds.south; lat <= bounds.north; lat += latStep) {
      const row: TemperatureGridCell[] = [];
      for (let lon = bounds.west; lon <= bounds.east; lon += lonStep) {
        const coordinate: Coordinate = { latitude: lat, longitude: lon };
        const temperature = await this.getTemperatureAtLocation(coordinate);
        const surfaceMaterial = await this.getSurfaceMaterial(coordinate);
        const albedo = this.calculateAlbedo(surfaceMaterial);
        const vegetationIndex = await this.getVegetationIndex(coordinate);

        row.push({
          coordinate,
          temperature,
          surfaceMaterial,
          albedo,
          vegetationIndex: vegetationIndex.value,
          buildingDensity: await this.getBuildingDensity(coordinate),
        });
      }
      grid.push(row);
    }

    return {
      id: this.generateId(),
      name: `Temperature Map ${new Date().toISOString()}`,
      bounds,
      resolution,
      grid,
      timestamp: new Date(),
      metadata: {},
    };
  }

  /**
   * Get temperature readings within a time range
   */
  public getTemperatureHistory(
    location: Coordinate,
    startDate: Date,
    endDate: Date
  ): TemperatureReading[] {
    return this.temperatureData.filter(reading => {
      const matchesLocation = this.distanceBetween(reading.location, location) < 100; // 100m radius
      const matchesTime = reading.timestamp >= startDate && reading.timestamp <= endDate;
      return matchesLocation && matchesTime;
    });
  }

  // ============================================================================
  // Hotspot Detection
  // ============================================================================

  /**
   * Detect heat hotspots in the current temperature data
   */
  public async detectHotspots(config?: Partial<HotspotDetectionConfig>): Promise<HeatHotspot[]> {
    const detectionConfig = { ...this.config.detection, ...config };
    const hotspots: HeatHotspot[] = [];

    // Group temperature readings by proximity
    const clusters = this.clusterTemperatureReadings(detectionConfig.clusteringDistance);

    for (const cluster of clusters) {
      const avgTemp = this.calculateAverageTemperature(cluster);
      const maxTemp = Math.max(...cluster.map(r => r.value));

      if (avgTemp > detectionConfig.temperatureThreshold) {
        const severity = this.calculateHotspotSeverity(avgTemp, detectionConfig.severityThresholds);
        const location = this.calculateCentroid(cluster.map(r => r.location));
        const area = this.calculateClusterArea(cluster);

        const hotspot: HeatHotspot = {
          id: this.generateId(),
          location,
          area,
          averageTemperature: avgTemp,
          peakTemperature: maxTemp,
          severity,
          affectedPopulation: await this.estimateAffectedPopulation(location, area),
          contributingFactors: await this.analyzeContributingFactors(location),
          detectedAt: new Date(),
        };

        hotspots.push(hotspot);
        this.hotspots.set(hotspot.id, hotspot);

        this.emitEvent({
          id: this.generateId(),
          type: 'hotspot-detected',
          severity: severity === 'extreme' ? 'critical' : 'warning',
          location,
          data: { hotspot },
          timestamp: new Date(),
          message: `Heat hotspot detected with ${severity} severity`,
        });
      }
    }

    return hotspots;
  }

  /**
   * Get all detected hotspots
   */
  public getHotspots(): HeatHotspot[] {
    return Array.from(this.hotspots.values());
  }

  /**
   * Get hotspots by severity
   */
  public getHotspotsBySeverity(severity: HotspotSeverity): HeatHotspot[] {
    return this.getHotspots().filter(h => h.severity === severity);
  }

  // ============================================================================
  // Mitigation Planning
  // ============================================================================

  /**
   * Create a mitigation plan for specific hotspots
   */
  public async createMitigationPlan(
    name: string,
    hotspotIds: string[],
    budget: number
  ): Promise<MitigationPlan> {
    const targetHotspots = hotspotIds
      .map(id => this.hotspots.get(id))
      .filter(h => h !== undefined) as HeatHotspot[];

    if (targetHotspots.length === 0) {
      throw new Error('No valid hotspots found for mitigation plan');
    }

    // Generate and evaluate strategies for each hotspot
    const evaluatedStrategies: StrategyEvaluation[] = [];

    for (const hotspot of targetHotspots) {
      const strategies = await this.generateCoolingStrategies(hotspot);
      for (const strategy of strategies) {
        const evaluation = await this.evaluateStrategy(strategy, hotspot);
        evaluatedStrategies.push(evaluation);
      }
    }

    // Select best strategies within budget
    const selectedStrategies = this.selectOptimalStrategies(
      evaluatedStrategies,
      budget
    );

    // Create implementation phases
    const phases = this.createImplementationPhases(selectedStrategies);

    const plan: MitigationPlan = {
      id: this.generateId(),
      name,
      targetHotspots: hotspotIds,
      strategies: selectedStrategies.map(e => e.strategy),
      totalCost: selectedStrategies.reduce((sum, e) => sum + e.strategy.implementationCost, 0),
      estimatedCoolingEffect: selectedStrategies.reduce(
        (sum, e) => sum + e.strategy.estimatedCoolingEffect,
        0
      ),
      timeline: {
        startDate: new Date(),
        phases,
      },
      priorityLevel: this.calculatePlanPriority(targetHotspots),
    };

    this.mitigationPlans.set(plan.id, plan);

    return plan;
  }

  /**
   * Get all mitigation plans
   */
  public getMitigationPlans(): MitigationPlan[] {
    return Array.from(this.mitigationPlans.values());
  }

  // ============================================================================
  // Cooling Strategy Evaluation
  // ============================================================================

  /**
   * Evaluate a cooling strategy
   */
  public async evaluateStrategy(
    strategy: CoolingStrategy,
    hotspot: HeatHotspot
  ): Promise<StrategyEvaluation> {
    const effectivenessScore = this.calculateEffectiveness(strategy, hotspot);
    const costEffectiveness = strategy.estimatedCoolingEffect / strategy.implementationCost;
    const implementationFeasibility = await this.assessFeasibility(strategy);
    const communityImpact = await this.assessCommunityImpact(strategy);
    const environmentalBenefit = this.assessEnvironmentalBenefit(strategy);

    const overallScore = (
      effectivenessScore * 0.3 +
      (costEffectiveness * 100) * 0.25 +
      implementationFeasibility * 100 * 0.2 +
      communityImpact * 100 * 0.15 +
      environmentalBenefit * 100 * 0.1
    );

    let recommendationLevel: StrategyEvaluation['recommendationLevel'];
    if (overallScore >= 80) recommendationLevel = 'critical';
    else if (overallScore >= 60) recommendationLevel = 'high';
    else if (overallScore >= 40) recommendationLevel = 'medium';
    else recommendationLevel = 'low';

    return {
      strategy,
      effectivenessScore,
      costEffectiveness,
      implementationFeasibility,
      communityImpact,
      environmentalBenefit,
      recommendationLevel,
    };
  }

  /**
   * Register a new cooling strategy
   */
  public registerStrategy(strategy: CoolingStrategy): void {
    this.strategies.set(strategy.id, strategy);
  }

  /**
   * Get all registered strategies
   */
  public getStrategies(): CoolingStrategy[] {
    return Array.from(this.strategies.values());
  }

  // ============================================================================
  // Thermal Comfort Calculation
  // ============================================================================

  /**
   * Calculate thermal comfort index
   */
  public calculateThermalComfort(
    input: ThermalComfortInput,
    indexType: ThermalComfortIndex = 'UTCI'
  ): ThermalComfortResult {
    let value: number;
    let stressLevel: ThermalComfortResult['stressLevel'];

    switch (indexType) {
      case 'UTCI':
        value = this.calculateUTCI(input);
        stressLevel = this.getUTCIStressLevel(value);
        break;
      case 'PET':
        value = this.calculatePET(input);
        stressLevel = this.getPETStressLevel(value);
        break;
      case 'PMV':
        value = this.calculatePMV(input);
        stressLevel = this.getPMVStressLevel(value);
        break;
      case 'SET':
        value = this.calculateSET(input);
        stressLevel = this.getSETStressLevel(value);
        break;
    }

    const recommendation = this.generateThermalComfortRecommendation(stressLevel);

    return {
      index: indexType,
      value,
      stressLevel,
      recommendation,
      location: input.location,
      calculatedAt: new Date(),
    };
  }

  // ============================================================================
  // Analytics and Trends
  // ============================================================================

  /**
   * Analyze temperature trends over time
   */
  public analyzeTemperatureTrend(
    location: Coordinate,
    startDate: Date,
    endDate: Date
  ): TemperatureTrend {
    const readings = this.getTemperatureHistory(location, startDate, endDate);

    if (readings.length === 0) {
      throw new Error('No temperature data available for trend analysis');
    }

    const temperatures = readings.map(r => r.value);
    const avgTemp = temperatures.reduce((a, b) => a + b, 0) / temperatures.length;
    const maxTemp = Math.max(...temperatures);
    const minTemp = Math.min(...temperatures);

    // Simple linear regression for trend
    const rateOfChange = this.calculateRateOfChange(readings);
    let trend: TemperatureTrend['trend'];
    if (rateOfChange > 0.1) trend = 'increasing';
    else if (rateOfChange < -0.1) trend = 'decreasing';
    else trend = 'stable';

    return {
      location,
      period: { start: startDate, end: endDate },
      averageTemperature: avgTemp,
      maxTemperature: maxTemp,
      minTemperature: minTemp,
      trend,
      rateOfChange,
    };
  }

  /**
   * Calculate urban heat island intensity
   */
  public async calculateUHIIntensity(
    urbanLocation: Coordinate,
    ruralReferenceLocation: Coordinate
  ): Promise<UHIIntensity> {
    const urbanTemp = await this.getTemperatureAtLocation(urbanLocation);
    const ruralTemp = await this.getTemperatureAtLocation(ruralReferenceLocation);

    return {
      urbanTemperature: urbanTemp.value,
      ruralReferenceTemperature: ruralTemp.value,
      intensity: urbanTemp.value - ruralTemp.value,
      timestamp: new Date(),
      affectedArea: await this.estimateAffectedArea(urbanLocation),
    };
  }

  // ============================================================================
  // Event Handling
  // ============================================================================

  /**
   * Register an event handler
   */
  public on(eventType: HeatEventType, handler: HeatEventHandler): void {
    if (!this.eventHandlers.has(eventType)) {
      this.eventHandlers.set(eventType, []);
    }
    this.eventHandlers.get(eventType)!.push(handler);
  }

  /**
   * Remove an event handler
   */
  public off(eventType: HeatEventType, handler: HeatEventHandler): void {
    const handlers = this.eventHandlers.get(eventType);
    if (handlers) {
      const index = handlers.indexOf(handler);
      if (index > -1) {
        handlers.splice(index, 1);
      }
    }
  }

  /**
   * Emit an event
   */
  private emitEvent(event: HeatEvent): void {
    const handlers = this.eventHandlers.get(event.type);
    if (handlers) {
      handlers.forEach(handler => {
        try {
          handler(event);
        } catch (error) {
          console.error(`Error in event handler for ${event.type}:`, error);
        }
      });
    }
  }

  // ============================================================================
  // Monitoring Control
  // ============================================================================

  /**
   * Start continuous monitoring
   */
  public startMonitoring(): void {
    if (this.monitoringInterval) {
      return; // Already monitoring
    }

    this.monitoringInterval = setInterval(() => {
      this.performMonitoringCycle();
    }, this.config.monitoring.updateInterval * 1000);
  }

  /**
   * Stop continuous monitoring
   */
  public stopMonitoring(): void {
    if (this.monitoringInterval) {
      clearInterval(this.monitoringInterval);
      this.monitoringInterval = undefined;
    }
  }

  // ============================================================================
  // Private Helper Methods
  // ============================================================================

  private initializeEventHandlers(): void {
    // Initialize empty handler arrays for all event types
    const eventTypes: HeatEventType[] = [
      'hotspot-detected',
      'temperature-threshold-exceeded',
      'strategy-completed',
      'mitigation-milestone',
      'alert-issued',
    ];

    eventTypes.forEach(type => {
      this.eventHandlers.set(type, []);
    });
  }

  private async performMonitoringCycle(): Promise<void> {
    // Detect new hotspots
    await this.detectHotspots();

    // Clean up old data if needed
    if (this.config.analysis.enableTrendAnalysis) {
      this.cleanupOldData();
    }
  }

  private cleanupOldData(): void {
    const retentionPeriod = this.config.analysis.historicalDataRetention * 24 * 60 * 60 * 1000;
    const cutoffDate = new Date(Date.now() - retentionPeriod);

    this.temperatureData = this.temperatureData.filter(
      reading => reading.timestamp > cutoffDate
    );
  }

  private generateId(): string {
    return `uhi_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  private exceedsThreshold(reading: TemperatureReading): boolean {
    return reading.value > this.config.detection.temperatureThreshold;
  }

  private async getTemperatureAtLocation(location: Coordinate): Promise<TemperatureReading> {
    // Find nearest temperature reading or interpolate
    const nearbyReadings = this.temperatureData.filter(r =>
      this.distanceBetween(r.location, location) < 1000
    );

    if (nearbyReadings.length === 0) {
      // Return estimated temperature
      return {
        value: 25, // Default temperature
        unit: this.config.monitoring.temperatureUnit,
        timestamp: new Date(),
        location,
        source: 'interpolated',
        reliability: 0.5,
      };
    }

    // Use inverse distance weighting for interpolation
    return this.interpolateTemperature(location, nearbyReadings);
  }

  private interpolateTemperature(
    location: Coordinate,
    readings: TemperatureReading[]
  ): TemperatureReading {
    let weightedSum = 0;
    let weightSum = 0;

    readings.forEach(reading => {
      const distance = this.distanceBetween(location, reading.location);
      const weight = 1 / (distance + 1); // Avoid division by zero
      weightedSum += reading.value * weight;
      weightSum += weight;
    });

    return {
      value: weightedSum / weightSum,
      unit: this.config.monitoring.temperatureUnit,
      timestamp: new Date(),
      location,
      source: 'interpolated',
      reliability: Math.min(readings.length / 10, 1),
    };
  }

  private distanceBetween(coord1: Coordinate, coord2: Coordinate): number {
    // Haversine formula for calculating distance between coordinates
    const R = 6371000; // Earth's radius in meters
    const φ1 = coord1.latitude * Math.PI / 180;
    const φ2 = coord2.latitude * Math.PI / 180;
    const Δφ = (coord2.latitude - coord1.latitude) * Math.PI / 180;
    const Δλ = (coord2.longitude - coord1.longitude) * Math.PI / 180;

    const a = Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
              Math.cos(φ1) * Math.cos(φ2) *
              Math.sin(Δλ / 2) * Math.sin(Δλ / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    return R * c;
  }

  private async getSurfaceMaterial(location: Coordinate): Promise<SurfaceMaterial> {
    // Placeholder - would typically query a surface material database
    return {
      type: 'concrete',
      albedo: 0.3,
      emissivity: 0.9,
      thermalCapacity: 880,
      coverage: 100,
    };
  }

  private calculateAlbedo(material: SurfaceMaterial): number {
    return material.albedo;
  }

  private async getVegetationIndex(location: Coordinate): Promise<VegetationIndex> {
    // Placeholder - would typically query satellite data
    return {
      type: 'NDVI',
      value: 0.5,
      location,
      timestamp: new Date(),
      source: 'satellite',
    };
  }

  private async getBuildingDensity(location: Coordinate): Promise<number> {
    // Placeholder - would typically query GIS data
    return 0.6; // 60% building coverage
  }

  private clusterTemperatureReadings(distance: number): TemperatureReading[][] {
    // Simple clustering algorithm
    const clusters: TemperatureReading[][] = [];
    const used = new Set<number>();

    this.temperatureData.forEach((reading, index) => {
      if (used.has(index)) return;

      const cluster: TemperatureReading[] = [reading];
      used.add(index);

      this.temperatureData.forEach((other, otherIndex) => {
        if (used.has(otherIndex)) return;
        if (this.distanceBetween(reading.location, other.location) < distance) {
          cluster.push(other);
          used.add(otherIndex);
        }
      });

      if (cluster.length > 0) {
        clusters.push(cluster);
      }
    });

    return clusters;
  }

  private calculateAverageTemperature(readings: TemperatureReading[]): number {
    const sum = readings.reduce((acc, r) => acc + r.value, 0);
    return sum / readings.length;
  }

  private calculateHotspotSeverity(
    temperature: number,
    thresholds: HotspotDetectionConfig['severityThresholds']
  ): HotspotSeverity {
    if (temperature >= thresholds.extreme) return 'extreme';
    if (temperature >= thresholds.high) return 'high';
    if (temperature >= thresholds.moderate) return 'moderate';
    return 'low';
  }

  private calculateCentroid(locations: Coordinate[]): Coordinate {
    const sumLat = locations.reduce((sum, loc) => sum + loc.latitude, 0);
    const sumLon = locations.reduce((sum, loc) => sum + loc.longitude, 0);

    return {
      latitude: sumLat / locations.length,
      longitude: sumLon / locations.length,
    };
  }

  private calculateClusterArea(readings: TemperatureReading[]): number {
    // Simplified area calculation
    if (readings.length < 3) return 1000; // Default 1000 m²

    const locs = readings.map(r => r.location);
    const minLat = Math.min(...locs.map(l => l.latitude));
    const maxLat = Math.max(...locs.map(l => l.latitude));
    const minLon = Math.min(...locs.map(l => l.longitude));
    const maxLon = Math.max(...locs.map(l => l.longitude));

    const width = this.distanceBetween(
      { latitude: minLat, longitude: minLon },
      { latitude: minLat, longitude: maxLon }
    );
    const height = this.distanceBetween(
      { latitude: minLat, longitude: minLon },
      { latitude: maxLat, longitude: minLon }
    );

    return width * height;
  }

  private async estimateAffectedPopulation(location: Coordinate, area: number): Promise<number> {
    // Placeholder - would typically query population density data
    const densityPerSqKm = 5000; // People per sq km
    const areaSqKm = area / 1000000;
    return Math.round(densityPerSqKm * areaSqKm);
  }

  private async analyzeContributingFactors(location: Coordinate) {
    // Placeholder implementation
    return [
      { type: 'surface-material' as const, contribution: 0.4, description: 'High albedo materials' },
      { type: 'lack-of-vegetation' as const, contribution: 0.3, description: 'Low vegetation coverage' },
      { type: 'building-density' as const, contribution: 0.3, description: 'High building density' },
    ];
  }

  private async generateCoolingStrategies(hotspot: HeatHotspot): Promise<CoolingStrategy[]> {
    // Generate appropriate strategies based on hotspot characteristics
    const strategies: CoolingStrategy[] = [];
    const baseId = this.generateId();

    strategies.push({
      id: `${baseId}_green`,
      name: 'Green Infrastructure',
      type: 'green-infrastructure',
      description: 'Install green roofs, vertical gardens, and pocket parks',
      targetArea: { location: hotspot.location, radius: 500 },
      estimatedCoolingEffect: 2.5,
      implementationCost: 500000,
      maintenanceCost: 25000,
      lifespan: 20,
      cobenefits: ['Air quality', 'Stormwater management', 'Biodiversity'],
    });

    return strategies;
  }

  private selectOptimalStrategies(
    evaluations: StrategyEvaluation[],
    budget: number
  ): StrategyEvaluation[] {
    // Sort by recommendation level and cost-effectiveness
    const sorted = evaluations.sort((a, b) => {
      if (a.recommendationLevel !== b.recommendationLevel) {
        const levels = { critical: 4, high: 3, medium: 2, low: 1 };
        return levels[b.recommendationLevel] - levels[a.recommendationLevel];
      }
      return b.costEffectiveness - a.costEffectiveness;
    });

    // Select strategies within budget
    const selected: StrategyEvaluation[] = [];
    let remainingBudget = budget;

    for (const evaluation of sorted) {
      if (evaluation.strategy.implementationCost <= remainingBudget) {
        selected.push(evaluation);
        remainingBudget -= evaluation.strategy.implementationCost;
      }
    }

    return selected;
  }

  private createImplementationPhases(strategies: StrategyEvaluation[]): MitigationPhase[] {
    // Group strategies into logical phases
    return [
      {
        name: 'Phase 1: Planning and Design',
        duration: 3,
        strategies: strategies.slice(0, Math.ceil(strategies.length / 3)).map(e => e.strategy.id),
        cost: strategies.slice(0, Math.ceil(strategies.length / 3))
          .reduce((sum, e) => sum + e.strategy.implementationCost, 0) * 0.3,
        expectedOutcome: 'Completed designs and permits',
      },
      {
        name: 'Phase 2: Implementation',
        duration: 12,
        strategies: strategies.map(e => e.strategy.id),
        cost: strategies.reduce((sum, e) => sum + e.strategy.implementationCost, 0) * 0.6,
        expectedOutcome: 'All strategies implemented',
      },
      {
        name: 'Phase 3: Monitoring and Optimization',
        duration: 6,
        strategies: strategies.map(e => e.strategy.id),
        cost: strategies.reduce((sum, e) => sum + e.strategy.implementationCost, 0) * 0.1,
        expectedOutcome: 'Performance verified and optimized',
      },
    ];
  }

  private calculatePlanPriority(hotspots: HeatHotspot[]): number {
    const avgSeverity = hotspots.reduce((sum, h) => {
      const severityScore = { low: 1, moderate: 2, high: 3, extreme: 4 };
      return sum + severityScore[h.severity];
    }, 0) / hotspots.length;

    return Math.min(Math.ceil(avgSeverity), 5);
  }

  private calculateEffectiveness(strategy: CoolingStrategy, hotspot: HeatHotspot): number {
    // Score from 0-100 based on how well strategy addresses hotspot
    let score = 50; // Base score

    if (strategy.estimatedCoolingEffect >= hotspot.averageTemperature - 25) {
      score += 30;
    } else if (strategy.estimatedCoolingEffect >= hotspot.averageTemperature - 30) {
      score += 20;
    }

    // Consider hotspot severity
    const severityBonus = { low: 0, moderate: 10, high: 15, extreme: 20 };
    score += severityBonus[hotspot.severity];

    return Math.min(score, 100);
  }

  private async assessFeasibility(strategy: CoolingStrategy): Promise<number> {
    // Return feasibility score 0-1
    return 0.75; // Placeholder
  }

  private async assessCommunityImpact(strategy: CoolingStrategy): Promise<number> {
    // Return impact score 0-1
    return strategy.cobenefits.length * 0.15;
  }

  private assessEnvironmentalBenefit(strategy: CoolingStrategy): number {
    // Return benefit score 0-1
    const greenStrategies = ['green-infrastructure', 'urban-forest', 'water-feature'];
    return greenStrategies.includes(strategy.type) ? 0.9 : 0.6;
  }

  private calculateUTCI(input: ThermalComfortInput): number {
    // Simplified UTCI calculation
    const { airTemperature, relativeHumidity, windSpeed, solarRadiation } = input;

    const vaporPressure = (relativeHumidity / 100) * 6.11 * Math.exp(
      (17.27 * airTemperature) / (237.7 + airTemperature)
    );

    return airTemperature +
           0.6 * vaporPressure -
           0.4 * windSpeed +
           0.0015 * solarRadiation;
  }

  private getUTCIStressLevel(utci: number): ThermalComfortResult['stressLevel'] {
    if (utci > 46) return 'extreme-stress';
    if (utci > 38) return 'strong-stress';
    if (utci > 32) return 'moderate-stress';
    if (utci > 26) return 'slight-stress';
    return 'no-stress';
  }

  private calculatePET(input: ThermalComfortInput): number {
    return input.airTemperature + (input.relativeHumidity / 100) * 5;
  }

  private getPETStressLevel(pet: number): ThermalComfortResult['stressLevel'] {
    if (pet > 41) return 'extreme-stress';
    if (pet > 35) return 'strong-stress';
    if (pet > 29) return 'moderate-stress';
    if (pet > 23) return 'slight-stress';
    return 'no-stress';
  }

  private calculatePMV(input: ThermalComfortInput): number {
    // Simplified PMV
    return (input.airTemperature - 22) / 10;
  }

  private getPMVStressLevel(pmv: number): ThermalComfortResult['stressLevel'] {
    if (Math.abs(pmv) > 3) return 'extreme-stress';
    if (Math.abs(pmv) > 2) return 'strong-stress';
    if (Math.abs(pmv) > 1) return 'moderate-stress';
    if (Math.abs(pmv) > 0.5) return 'slight-stress';
    return 'no-stress';
  }

  private calculateSET(input: ThermalComfortInput): number {
    return input.airTemperature + (input.relativeHumidity / 100) * 3;
  }

  private getSETStressLevel(set: number): ThermalComfortResult['stressLevel'] {
    if (set > 37) return 'extreme-stress';
    if (set > 30) return 'strong-stress';
    if (set > 26) return 'moderate-stress';
    if (set > 22) return 'slight-stress';
    return 'no-stress';
  }

  private generateThermalComfortRecommendation(
    stressLevel: ThermalComfortResult['stressLevel']
  ): string {
    const recommendations = {
      'no-stress': 'Comfortable conditions. No special precautions needed.',
      'slight-stress': 'Slightly uncomfortable. Consider light clothing and hydration.',
      'moderate-stress': 'Moderately uncomfortable. Seek shade and stay hydrated.',
      'strong-stress': 'Strong heat stress. Limit outdoor activities and stay in air-conditioned spaces.',
      'extreme-stress': 'Extreme heat stress. Avoid outdoor exposure. Health risk for vulnerable populations.',
    };

    return recommendations[stressLevel];
  }

  private calculateRateOfChange(readings: TemperatureReading[]): number {
    if (readings.length < 2) return 0;

    // Sort by timestamp
    const sorted = readings.sort((a, b) => a.timestamp.getTime() - b.timestamp.getTime());

    const firstTemp = sorted[0].value;
    const lastTemp = sorted[sorted.length - 1].value;
    const timeDiff = sorted[sorted.length - 1].timestamp.getTime() - sorted[0].timestamp.getTime();
    const yearsDiff = timeDiff / (1000 * 60 * 60 * 24 * 365);

    return (lastTemp - firstTemp) / yearsDiff;
  }

  private async estimateAffectedArea(location: Coordinate): Promise<number> {
    // Placeholder - return estimated area in square kilometers
    return 25; // 25 sq km
  }
}

// ============================================================================
// Factory Function
// ============================================================================

/**
 * Create a new WIA Urban Heat Island instance
 */
export function createUrbanHeatIsland(config: UHIConfig): WIAUrbanHeatIsland {
  return new WIAUrbanHeatIsland(config);
}

// ============================================================================
// Exports
// ============================================================================

export * from './types';
export default WIAUrbanHeatIsland;
