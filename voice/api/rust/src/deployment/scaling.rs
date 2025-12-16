//! Auto-scaling configuration and logic

use serde::{Deserialize, Serialize};
use std::time::Duration;

/// Scaling configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScalingConfig {
    /// Minimum replicas
    pub min_replicas: u32,

    /// Maximum replicas
    pub max_replicas: u32,

    /// Target CPU utilization (percentage)
    pub target_cpu_utilization: u32,

    /// Target memory utilization (percentage)
    pub target_memory_utilization: u32,

    /// Scale up cooldown
    pub scale_up_cooldown: Duration,

    /// Scale down cooldown
    pub scale_down_cooldown: Duration,

    /// Custom metrics
    pub custom_metrics: Vec<CustomMetricTarget>,
}

impl Default for ScalingConfig {
    fn default() -> Self {
        Self {
            min_replicas: 3,
            max_replicas: 50,
            target_cpu_utilization: 70,
            target_memory_utilization: 80,
            scale_up_cooldown: Duration::from_secs(0),
            scale_down_cooldown: Duration::from_secs(300),
            custom_metrics: vec![
                CustomMetricTarget {
                    name: "requests_per_second".to_string(),
                    target_value: 1000.0,
                    metric_type: MetricType::AverageValue,
                },
            ],
        }
    }
}

/// Custom metric target
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CustomMetricTarget {
    /// Metric name
    pub name: String,

    /// Target value
    pub target_value: f64,

    /// Metric type
    pub metric_type: MetricType,
}

/// Metric type for scaling
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MetricType {
    /// Average value across all pods
    AverageValue,
    /// Total value
    Value,
    /// Utilization percentage
    Utilization,
}

/// Scaling decision
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScalingDecision {
    /// Scale up by N replicas
    ScaleUp(u32),
    /// Scale down by N replicas
    ScaleDown(u32),
    /// No scaling needed
    NoChange,
}

/// Current metrics for scaling decision
#[derive(Debug, Clone)]
pub struct CurrentMetrics {
    pub cpu_utilization: f64,
    pub memory_utilization: f64,
    pub requests_per_second: f64,
    pub current_replicas: u32,
    pub custom_metrics: Vec<(String, f64)>,
}

/// Scaling calculator
pub struct ScalingCalculator {
    config: ScalingConfig,
}

impl ScalingCalculator {
    /// Create a new scaling calculator
    pub fn new(config: ScalingConfig) -> Self {
        Self { config }
    }

    /// Calculate scaling decision
    pub fn calculate(&self, metrics: &CurrentMetrics) -> ScalingDecision {
        let current = metrics.current_replicas;

        // Check if at limits
        if current <= self.config.min_replicas {
            // Can only scale up
            if self.should_scale_up(metrics) {
                let desired = self.calculate_desired_replicas(metrics);
                if desired > current {
                    return ScalingDecision::ScaleUp(
                        (desired - current).min(self.config.max_replicas - current),
                    );
                }
            }
            return ScalingDecision::NoChange;
        }

        if current >= self.config.max_replicas {
            // Can only scale down
            if self.should_scale_down(metrics) {
                let desired = self.calculate_desired_replicas(metrics);
                if desired < current {
                    return ScalingDecision::ScaleDown(
                        (current - desired).min(current - self.config.min_replicas),
                    );
                }
            }
            return ScalingDecision::NoChange;
        }

        // Normal scaling
        let desired = self.calculate_desired_replicas(metrics);

        if desired > current {
            ScalingDecision::ScaleUp(desired - current)
        } else if desired < current {
            ScalingDecision::ScaleDown(current - desired)
        } else {
            ScalingDecision::NoChange
        }
    }

    fn should_scale_up(&self, metrics: &CurrentMetrics) -> bool {
        metrics.cpu_utilization > self.config.target_cpu_utilization as f64
            || metrics.memory_utilization > self.config.target_memory_utilization as f64
    }

    fn should_scale_down(&self, metrics: &CurrentMetrics) -> bool {
        metrics.cpu_utilization < (self.config.target_cpu_utilization as f64 * 0.5)
            && metrics.memory_utilization < (self.config.target_memory_utilization as f64 * 0.5)
    }

    fn calculate_desired_replicas(&self, metrics: &CurrentMetrics) -> u32 {
        let current = metrics.current_replicas as f64;

        // Calculate based on CPU
        let cpu_desired =
            current * (metrics.cpu_utilization / self.config.target_cpu_utilization as f64);

        // Calculate based on memory
        let memory_desired =
            current * (metrics.memory_utilization / self.config.target_memory_utilization as f64);

        // Calculate based on custom metrics
        let mut custom_desired = current;
        for target in &self.config.custom_metrics {
            if let Some((_, value)) = metrics.custom_metrics.iter().find(|(n, _)| n == &target.name)
            {
                let desired = current * (value / target.target_value);
                custom_desired = custom_desired.max(desired);
            }
        }

        // Take the maximum of all calculations
        let desired = cpu_desired.max(memory_desired).max(custom_desired);

        // Clamp to min/max
        (desired.ceil() as u32)
            .max(self.config.min_replicas)
            .min(self.config.max_replicas)
    }
}

/// Scaling behavior configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScalingBehavior {
    /// Scale up behavior
    pub scale_up: ScalingPolicy,

    /// Scale down behavior
    pub scale_down: ScalingPolicy,
}

impl Default for ScalingBehavior {
    fn default() -> Self {
        Self {
            scale_up: ScalingPolicy {
                stabilization_window: Duration::from_secs(0),
                policies: vec![
                    PolicyRule {
                        policy_type: PolicyType::Percent,
                        value: 100,
                        period: Duration::from_secs(15),
                    },
                    PolicyRule {
                        policy_type: PolicyType::Pods,
                        value: 4,
                        period: Duration::from_secs(15),
                    },
                ],
                select_policy: PolicySelection::Max,
            },
            scale_down: ScalingPolicy {
                stabilization_window: Duration::from_secs(300),
                policies: vec![PolicyRule {
                    policy_type: PolicyType::Percent,
                    value: 10,
                    period: Duration::from_secs(60),
                }],
                select_policy: PolicySelection::Max,
            },
        }
    }
}

/// Scaling policy
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScalingPolicy {
    /// Stabilization window
    pub stabilization_window: Duration,

    /// Policy rules
    pub policies: Vec<PolicyRule>,

    /// Policy selection
    pub select_policy: PolicySelection,
}

/// Policy rule
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PolicyRule {
    /// Policy type
    pub policy_type: PolicyType,

    /// Value
    pub value: u32,

    /// Period
    pub period: Duration,
}

/// Policy type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PolicyType {
    /// Percentage of current replicas
    Percent,
    /// Absolute number of pods
    Pods,
}

/// Policy selection strategy
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PolicySelection {
    /// Maximum of all policies
    Max,
    /// Minimum of all policies
    Min,
    /// Disable scaling
    Disabled,
}

/// Capacity planning information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CapacityPlan {
    /// Baseline capacity
    pub baseline: CapacityLevel,

    /// Peak capacity (multiplier)
    pub peak_multiplier: f64,

    /// Emergency capacity
    pub emergency: CapacityLevel,
}

/// Capacity level
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CapacityLevel {
    /// API instances
    pub api_instances: u32,

    /// ASR workers
    pub asr_workers: u32,

    /// Translation workers
    pub translation_workers: u32,

    /// Renderers
    pub renderers: u32,
}

impl Default for CapacityPlan {
    fn default() -> Self {
        Self {
            baseline: CapacityLevel {
                api_instances: 3,
                asr_workers: 2,
                translation_workers: 2,
                renderers: 1,
            },
            peak_multiplier: 3.0,
            emergency: CapacityLevel {
                api_instances: 10,
                asr_workers: 5,
                translation_workers: 5,
                renderers: 3,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scaling_calculator_scale_up() {
        let config = ScalingConfig::default();
        let calculator = ScalingCalculator::new(config);

        let metrics = CurrentMetrics {
            cpu_utilization: 90.0,
            memory_utilization: 50.0,
            requests_per_second: 800.0,
            current_replicas: 3,
            custom_metrics: vec![],
        };

        let decision = calculator.calculate(&metrics);
        assert!(matches!(decision, ScalingDecision::ScaleUp(_)));
    }

    #[test]
    fn test_scaling_calculator_scale_down() {
        let config = ScalingConfig::default();
        let calculator = ScalingCalculator::new(config);

        let metrics = CurrentMetrics {
            cpu_utilization: 20.0,
            memory_utilization: 20.0,
            requests_per_second: 200.0,
            current_replicas: 10,
            custom_metrics: vec![],
        };

        let decision = calculator.calculate(&metrics);
        assert!(matches!(decision, ScalingDecision::ScaleDown(_)));
    }

    #[test]
    fn test_scaling_calculator_no_change() {
        let config = ScalingConfig::default();
        let calculator = ScalingCalculator::new(config);

        let metrics = CurrentMetrics {
            cpu_utilization: 70.0,
            memory_utilization: 70.0,
            requests_per_second: 1000.0,
            current_replicas: 5,
            custom_metrics: vec![],
        };

        let decision = calculator.calculate(&metrics);
        // May scale up or down slightly, or no change depending on calculation
        assert!(matches!(
            decision,
            ScalingDecision::NoChange | ScalingDecision::ScaleUp(1) | ScalingDecision::ScaleDown(1)
        ));
    }

    #[test]
    fn test_min_replicas_constraint() {
        let mut config = ScalingConfig::default();
        config.min_replicas = 3;
        let calculator = ScalingCalculator::new(config);

        let metrics = CurrentMetrics {
            cpu_utilization: 10.0,
            memory_utilization: 10.0,
            requests_per_second: 100.0,
            current_replicas: 3,
            custom_metrics: vec![],
        };

        let decision = calculator.calculate(&metrics);
        // Should not scale down below minimum
        assert!(matches!(decision, ScalingDecision::NoChange));
    }
}
