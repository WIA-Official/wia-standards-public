/**
 * WIA-SLEEP SDK - Sleep Analyzer Service
 *
 * Service for analyzing sleep architecture, quality metrics, and patterns.
 */

import {
  SleepRecord,
  SleepArchitecture,
  SleepArchitectureSummary,
  SleepStagePercentages,
  SleepCycle,
  SleepQualityMetrics,
  SleepStage,
  Hypnogram,
  Percentage
} from '../types/SleepRecord';

/**
 * Reference values for sleep metrics
 */
export interface SleepReferenceValues {
  /** Target sleep efficiency */
  sleepEfficiency: { min: Percentage; optimal: Percentage };
  /** Target sleep onset latency */
  sleepOnsetLatency: { max: number; optimal: number };
  /** Target WASO */
  wakeAfterSleepOnset: { max: number; optimal: number };
  /** Target stage percentages */
  stagePercentages: {
    n1: { min: Percentage; max: Percentage };
    n2: { min: Percentage; max: Percentage };
    n3: { min: Percentage; max: Percentage };
    rem: { min: Percentage; max: Percentage };
  };
}

/**
 * Default reference values for adults
 */
const ADULT_REFERENCE_VALUES: SleepReferenceValues = {
  sleepEfficiency: { min: 85, optimal: 90 },
  sleepOnsetLatency: { max: 30, optimal: 15 },
  wakeAfterSleepOnset: { max: 30, optimal: 20 },
  stagePercentages: {
    n1: { min: 2, max: 10 },
    n2: { min: 45, max: 55 },
    n3: { min: 15, max: 25 },
    rem: { min: 20, max: 25 }
  }
};

/**
 * Sleep quality assessment
 */
export interface SleepQualityAssessment {
  /** Overall quality score (0-100) */
  overallScore: number;
  /** Component scores */
  components: {
    efficiency: number;
    continuity: number;
    architecture: number;
    timing: number;
  };
  /** Quality level */
  qualityLevel: 'excellent' | 'good' | 'fair' | 'poor';
  /** Areas for improvement */
  concerns: string[];
  /** Positive aspects */
  strengths: string[];
}

/**
 * Sleep trend data point
 */
export interface SleepTrendPoint {
  date: string;
  totalSleepTime_min: number;
  sleepEfficiency_pct: Percentage;
  sleepQualityScore: number;
}

/**
 * Sleep pattern analysis
 */
export interface SleepPatternAnalysis {
  /** Average metrics over period */
  averages: {
    totalSleepTime_min: number;
    sleepEfficiency_pct: Percentage;
    sleepOnsetLatency_min: number;
    waso_min: number;
  };
  /** Variability metrics */
  variability: {
    bedtimeVariability_min: number;
    wakeTimeVariability_min: number;
    durationVariability_min: number;
  };
  /** Trend direction */
  trends: {
    efficiency: 'improving' | 'stable' | 'declining';
    duration: 'increasing' | 'stable' | 'decreasing';
  };
  /** Sleep debt estimation */
  sleepDebt: {
    current_hours: number;
    weekly_hours: number;
    recoveryNeeded: boolean;
  };
}

/**
 * Sleep analyzer service
 */
export class SleepAnalyzer {
  private referenceValues: SleepReferenceValues;

  constructor(referenceValues: SleepReferenceValues = ADULT_REFERENCE_VALUES) {
    this.referenceValues = referenceValues;
  }

  /**
   * Calculate sleep quality metrics from architecture
   */
  calculateQualityMetrics(architecture: SleepArchitecture): SleepQualityMetrics {
    const { summary, stagePercentages, microstructure } = architecture;

    // Sleep Quality Index (composite score)
    let sleepQualityIndex = 50; // Base score

    // Efficiency contribution (max 20 points)
    const efficiencyScore =
      Math.min(20, (summary.sleepEfficiency_pct / 100) * 20);
    sleepQualityIndex += efficiencyScore;

    // Latency contribution (max 10 points)
    const latencyScore = Math.max(
      0,
      10 - summary.sleepOnsetLatency_min / 3
    );
    sleepQualityIndex += latencyScore;

    // WASO contribution (max 10 points)
    const wasoScore = Math.max(
      0,
      10 - summary.wakeAfterSleepOnset_min / 6
    );
    sleepQualityIndex += wasoScore;

    // Stage architecture contribution (max 10 points)
    let architectureScore = 10;
    if (stagePercentages) {
      if (stagePercentages.n3_sws_pct < 15) architectureScore -= 3;
      if (stagePercentages.rem_pct < 20) architectureScore -= 3;
      if (stagePercentages.n1_pct > 10) architectureScore -= 2;
    }
    sleepQualityIndex += Math.max(0, architectureScore);

    sleepQualityIndex = Math.max(0, Math.min(100, sleepQualityIndex));

    // Restoration score based on SWS
    let restorationScore = 50;
    if (stagePercentages) {
      restorationScore =
        Math.min(100, (stagePercentages.n3_sws_pct / 20) * 100);
    }
    if (microstructure?.slowWaves) {
      restorationScore =
        Math.min(100, restorationScore + microstructure.slowWaves.density_per_min * 5);
    }

    // Cognitive recovery score based on REM and spindles
    let cognitiveRecoveryScore = 50;
    if (stagePercentages) {
      cognitiveRecoveryScore =
        Math.min(100, (stagePercentages.rem_pct / 22) * 100);
    }
    if (microstructure?.spindles) {
      cognitiveRecoveryScore =
        Math.min(100, cognitiveRecoveryScore + microstructure.spindles.density_per_min * 10);
    }

    // Sleep fragmentation
    const sleepFragmentation = summary.arousals_per_hour || 0;

    return {
      sleepQualityIndex,
      restorationScore,
      cognitiveRecoveryScore,
      sleepFragmentation
    };
  }

  /**
   * Assess overall sleep quality
   */
  assessQuality(record: SleepRecord): SleepQualityAssessment {
    const { sleepArchitecture } = record;
    const { summary, stagePercentages } = sleepArchitecture;
    const qualityMetrics =
      sleepArchitecture.qualityMetrics ||
      this.calculateQualityMetrics(sleepArchitecture);

    const concerns: string[] = [];
    const strengths: string[] = [];

    // Evaluate efficiency
    let efficiencyScore = (summary.sleepEfficiency_pct / 100) * 100;
    if (summary.sleepEfficiency_pct >= 90) {
      strengths.push('Excellent sleep efficiency');
    } else if (summary.sleepEfficiency_pct < 85) {
      concerns.push('Low sleep efficiency - too much time awake in bed');
    }

    // Evaluate latency
    let continuityScore = 100;
    if (summary.sleepOnsetLatency_min <= 15) {
      strengths.push('Quick sleep onset');
    } else if (summary.sleepOnsetLatency_min > 30) {
      concerns.push('Prolonged sleep onset latency');
      continuityScore -= 20;
    }

    // Evaluate WASO
    if (summary.wakeAfterSleepOnset_min <= 20) {
      strengths.push('Good sleep continuity');
    } else if (summary.wakeAfterSleepOnset_min > 45) {
      concerns.push('Fragmented sleep with frequent/long awakenings');
      continuityScore -= 30;
    }

    // Evaluate architecture
    let architectureScore = 80;
    if (stagePercentages) {
      if (stagePercentages.n3_sws_pct >= 15) {
        strengths.push('Adequate deep sleep');
      } else {
        concerns.push('Insufficient deep sleep (N3/SWS)');
        architectureScore -= 20;
      }

      if (stagePercentages.rem_pct >= 20) {
        strengths.push('Adequate REM sleep');
      } else {
        concerns.push('Insufficient REM sleep');
        architectureScore -= 20;
      }

      if (stagePercentages.n1_pct > 15) {
        concerns.push('Elevated light sleep (N1) percentage');
        architectureScore -= 10;
      }
    }

    // Timing score (simplified - would need schedule data for full assessment)
    const timingScore = 75;

    // Calculate overall score
    const overallScore =
      efficiencyScore * 0.3 +
      continuityScore * 0.25 +
      architectureScore * 0.25 +
      timingScore * 0.2;

    let qualityLevel: 'excellent' | 'good' | 'fair' | 'poor';
    if (overallScore >= 85) {
      qualityLevel = 'excellent';
    } else if (overallScore >= 70) {
      qualityLevel = 'good';
    } else if (overallScore >= 50) {
      qualityLevel = 'fair';
    } else {
      qualityLevel = 'poor';
    }

    return {
      overallScore: Math.round(overallScore),
      components: {
        efficiency: Math.round(efficiencyScore),
        continuity: Math.round(continuityScore),
        architecture: Math.round(architectureScore),
        timing: Math.round(timingScore)
      },
      qualityLevel,
      concerns,
      strengths
    };
  }

  /**
   * Detect sleep cycles from hypnogram
   */
  detectSleepCycles(hypnogram: Hypnogram): SleepCycle[] {
    const cycles: SleepCycle[] = [];
    const { stages, epochDuration_sec, startTime } = hypnogram;

    let cycleStart = 0;
    let inREM = false;
    let remStart = -1;
    let cycleNumber = 1;

    for (let i = 0; i < stages.length; i++) {
      const stage = stages[i];

      if (stage === SleepStage.REM && !inREM) {
        inREM = true;
        remStart = i;
      } else if (stage !== SleepStage.REM && inREM) {
        // REM ended - cycle complete
        inREM = false;

        const cycleEndEpoch = i;
        const startEpochTime = new Date(
          new Date(startTime).getTime() + cycleStart * epochDuration_sec * 1000
        );
        const endEpochTime = new Date(
          new Date(startTime).getTime() + cycleEndEpoch * epochDuration_sec * 1000
        );

        const duration_min =
          ((cycleEndEpoch - cycleStart) * epochDuration_sec) / 60;
        const remDuration_min =
          ((cycleEndEpoch - remStart) * epochDuration_sec) / 60;
        const nremDuration_min = duration_min - remDuration_min;

        // Count N3 epochs in this cycle
        let n3Epochs = 0;
        for (let j = cycleStart; j < cycleEndEpoch; j++) {
          if (stages[j] === SleepStage.N3) n3Epochs++;
        }
        const n3Duration_min = (n3Epochs * epochDuration_sec) / 60;

        cycles.push({
          cycleNumber,
          startTime: startEpochTime.toTimeString().slice(0, 8),
          endTime: endEpochTime.toTimeString().slice(0, 8),
          duration_min,
          nremDuration_min,
          remDuration_min,
          n3Duration_min,
          complete: true
        });

        cycleNumber++;
        cycleStart = i;
      }
    }

    // Handle incomplete final cycle
    if (cycleStart < stages.length - 1) {
      const duration_min =
        ((stages.length - cycleStart) * epochDuration_sec) / 60;
      if (duration_min > 30) {
        // Only count if substantial
        cycles.push({
          cycleNumber,
          startTime: new Date(
            new Date(startTime).getTime() + cycleStart * epochDuration_sec * 1000
          )
            .toTimeString()
            .slice(0, 8),
          endTime: new Date(
            new Date(startTime).getTime() + stages.length * epochDuration_sec * 1000
          )
            .toTimeString()
            .slice(0, 8),
          duration_min,
          nremDuration_min: duration_min,
          remDuration_min: 0,
          n3Duration_min: 0,
          complete: false
        });
      }
    }

    return cycles;
  }

  /**
   * Calculate stage percentages from hypnogram
   */
  calculateStagePercentages(hypnogram: Hypnogram): SleepStagePercentages {
    const { stages } = hypnogram;
    const totalSleepEpochs = stages.filter((s) => s !== SleepStage.WAKE).length;

    if (totalSleepEpochs === 0) {
      return { n1_pct: 0, n2_pct: 0, n3_sws_pct: 0, rem_pct: 0 };
    }

    const counts = {
      wake: 0,
      n1: 0,
      n2: 0,
      n3: 0,
      rem: 0
    };

    for (const stage of stages) {
      switch (stage) {
        case SleepStage.WAKE:
          counts.wake++;
          break;
        case SleepStage.N1:
          counts.n1++;
          break;
        case SleepStage.N2:
          counts.n2++;
          break;
        case SleepStage.N3:
          counts.n3++;
          break;
        case SleepStage.REM:
          counts.rem++;
          break;
      }
    }

    return {
      wake_pct: (counts.wake / stages.length) * 100,
      n1_pct: (counts.n1 / totalSleepEpochs) * 100,
      n2_pct: (counts.n2 / totalSleepEpochs) * 100,
      n3_sws_pct: (counts.n3 / totalSleepEpochs) * 100,
      rem_pct: (counts.rem / totalSleepEpochs) * 100
    };
  }

  /**
   * Analyze sleep patterns over multiple records
   */
  analyzePatterns(
    records: SleepRecord[],
    targetSleepNeed_hours: number = 8
  ): SleepPatternAnalysis {
    if (records.length === 0) {
      throw new Error('No records provided for pattern analysis');
    }

    // Calculate averages
    const totalTST = records.reduce(
      (sum, r) => sum + r.sleepArchitecture.summary.totalSleepTime_min,
      0
    );
    const totalEfficiency = records.reduce(
      (sum, r) => sum + r.sleepArchitecture.summary.sleepEfficiency_pct,
      0
    );
    const totalSOL = records.reduce(
      (sum, r) => sum + r.sleepArchitecture.summary.sleepOnsetLatency_min,
      0
    );
    const totalWASO = records.reduce(
      (sum, r) => sum + r.sleepArchitecture.summary.wakeAfterSleepOnset_min,
      0
    );

    const n = records.length;
    const avgTST = totalTST / n;
    const avgEfficiency = totalEfficiency / n;

    // Calculate variability (simplified)
    const tstValues = records.map(
      (r) => r.sleepArchitecture.summary.totalSleepTime_min
    );
    const tstStdDev = this.calculateStdDev(tstValues);

    // Calculate sleep debt
    const targetSleepMin = targetSleepNeed_hours * 60;
    const dailyDeficit = targetSleepMin - avgTST;
    const weeklyDebt = Math.max(0, dailyDeficit * 7) / 60;

    // Determine trends (simplified - comparing first and last thirds)
    const thirdSize = Math.floor(n / 3);
    let efficiencyTrend: 'improving' | 'stable' | 'declining' = 'stable';
    let durationTrend: 'increasing' | 'stable' | 'decreasing' = 'stable';

    if (n >= 6) {
      const firstThirdEff =
        records
          .slice(0, thirdSize)
          .reduce((sum, r) => sum + r.sleepArchitecture.summary.sleepEfficiency_pct, 0) /
        thirdSize;
      const lastThirdEff =
        records
          .slice(-thirdSize)
          .reduce((sum, r) => sum + r.sleepArchitecture.summary.sleepEfficiency_pct, 0) /
        thirdSize;

      if (lastThirdEff - firstThirdEff > 3) efficiencyTrend = 'improving';
      else if (firstThirdEff - lastThirdEff > 3) efficiencyTrend = 'declining';
    }

    return {
      averages: {
        totalSleepTime_min: Math.round(avgTST),
        sleepEfficiency_pct: Math.round(avgEfficiency * 10) / 10,
        sleepOnsetLatency_min: Math.round(totalSOL / n),
        waso_min: Math.round(totalWASO / n)
      },
      variability: {
        bedtimeVariability_min: 0, // Would need timestamp data
        wakeTimeVariability_min: 0,
        durationVariability_min: Math.round(tstStdDev)
      },
      trends: {
        efficiency: efficiencyTrend,
        duration: durationTrend
      },
      sleepDebt: {
        current_hours: Math.max(0, dailyDeficit / 60),
        weekly_hours: weeklyDebt,
        recoveryNeeded: weeklyDebt > 5
      }
    };
  }

  private calculateStdDev(values: number[]): number {
    const mean = values.reduce((sum, v) => sum + v, 0) / values.length;
    const squaredDiffs = values.map((v) => Math.pow(v - mean, 2));
    return Math.sqrt(squaredDiffs.reduce((sum, d) => sum + d, 0) / values.length);
  }
}

export default SleepAnalyzer;
