//! Result analysis and visualization

use crate::state::QuantumState;
use crate::types::ExecutionResult;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Result analyzer for quantum execution results
pub struct ResultAnalyzer {
    results: Vec<ExecutionResult>,
}

impl ResultAnalyzer {
    /// Create a new analyzer
    pub fn new() -> Self {
        Self {
            results: Vec::new(),
        }
    }

    /// Add a result
    pub fn add(&mut self, result: ExecutionResult) {
        self.results.push(result);
    }

    /// Add multiple results
    pub fn add_all(&mut self, results: Vec<ExecutionResult>) {
        self.results.extend(results);
    }

    /// Get aggregated counts
    pub fn counts(&self) -> HashMap<String, usize> {
        let mut total_counts = HashMap::new();

        for result in &self.results {
            for (bitstring, &count) in &result.counts {
                *total_counts.entry(bitstring.clone()).or_insert(0) += count;
            }
        }

        total_counts
    }

    /// Get probabilities
    pub fn probabilities(&self) -> HashMap<String, f64> {
        let counts = self.counts();
        let total: usize = counts.values().sum();

        if total == 0 {
            return HashMap::new();
        }

        counts
            .into_iter()
            .map(|(k, v)| (k, v as f64 / total as f64))
            .collect()
    }

    /// Calculate Shannon entropy
    pub fn entropy(&self) -> f64 {
        let probs = self.probabilities();
        let mut entropy = 0.0;

        for &p in probs.values() {
            if p > 0.0 {
                entropy -= p * p.log2();
            }
        }

        entropy
    }

    /// Get most probable state
    pub fn most_probable(&self) -> Option<(String, f64)> {
        let probs = self.probabilities();
        probs
            .into_iter()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
    }

    /// Get top k states
    pub fn top_k(&self, k: usize) -> Vec<(String, f64)> {
        let mut probs: Vec<_> = self.probabilities().into_iter().collect();
        probs.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        probs.into_iter().take(k).collect()
    }

    /// Calculate expectation value of Z operator
    pub fn z_expectation(&self) -> f64 {
        let probs = self.probabilities();
        let mut expectation = 0.0;

        for (bitstring, &prob) in &probs {
            let ones = bitstring.chars().filter(|&c| c == '1').count();
            let parity = if ones % 2 == 0 { 1.0 } else { -1.0 };
            expectation += parity * prob;
        }

        expectation
    }

    /// Calculate variance
    pub fn variance(&self) -> f64 {
        let probs = self.probabilities();
        let mean = self.z_expectation();
        let mut variance = 0.0;

        for (bitstring, &prob) in &probs {
            let ones = bitstring.chars().filter(|&c| c == '1').count();
            let value = if ones % 2 == 0 { 1.0 } else { -1.0 };
            variance += prob * (value - mean).powi(2);
        }

        variance
    }

    /// Calculate standard deviation
    pub fn std_dev(&self) -> f64 {
        self.variance().sqrt()
    }

    /// Calculate confidence interval
    pub fn confidence_interval(&self, confidence: f64) -> (f64, f64) {
        let mean = self.z_expectation();
        let std = self.std_dev();
        let total_shots: usize = self.results.iter()
            .flat_map(|r| r.counts.values())
            .sum();

        // Z-score for confidence level (simplified)
        let z = match confidence {
            c if c >= 0.99 => 2.576,
            c if c >= 0.95 => 1.96,
            c if c >= 0.90 => 1.645,
            _ => 1.0,
        };

        let margin = z * std / (total_shots as f64).sqrt();
        (mean - margin, mean + margin)
    }

    /// Get summary statistics
    pub fn summary(&self) -> AnalysisSummary {
        let counts = self.counts();
        let probs = self.probabilities();
        let total_shots: usize = counts.values().sum();
        let num_unique = counts.len();

        AnalysisSummary {
            total_shots,
            unique_states: num_unique,
            most_probable: self.most_probable(),
            entropy: self.entropy(),
            z_expectation: self.z_expectation(),
            variance: self.variance(),
        }
    }
}

impl Default for ResultAnalyzer {
    fn default() -> Self {
        Self::new()
    }
}

/// Analysis summary
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnalysisSummary {
    /// Total number of shots
    pub total_shots: usize,
    /// Number of unique states observed
    pub unique_states: usize,
    /// Most probable state and its probability
    pub most_probable: Option<(String, f64)>,
    /// Shannon entropy
    pub entropy: f64,
    /// Z operator expectation value
    pub z_expectation: f64,
    /// Variance
    pub variance: f64,
}

/// Circuit visualizer
pub struct Visualizer {
    config: VisualizerConfig,
}

/// Visualizer configuration
#[derive(Debug, Clone)]
pub struct VisualizerConfig {
    /// Output width
    pub width: usize,
    /// Color scheme
    pub colors: bool,
    /// Show gate labels
    pub show_labels: bool,
}

impl Default for VisualizerConfig {
    fn default() -> Self {
        Self {
            width: 80,
            colors: false,
            show_labels: true,
        }
    }
}

impl Visualizer {
    /// Create a new visualizer
    pub fn new() -> Self {
        Self {
            config: VisualizerConfig::default(),
        }
    }

    /// Create with configuration
    pub fn with_config(config: VisualizerConfig) -> Self {
        Self { config }
    }

    /// Draw histogram of counts
    pub fn histogram(&self, counts: &HashMap<String, usize>) -> String {
        let max_count = counts.values().max().copied().unwrap_or(1);
        let total: usize = counts.values().sum();
        let bar_width = self.config.width.saturating_sub(30);

        let mut lines = Vec::new();
        lines.push(format!("Measurement Results (total: {})", total));
        lines.push("─".repeat(self.config.width));

        let mut sorted: Vec<_> = counts.iter().collect();
        sorted.sort_by(|a, b| b.1.cmp(a.1));

        for (state, &count) in sorted.iter().take(16) {
            let bar_len = (count as f64 / max_count as f64 * bar_width as f64) as usize;
            let bar = "█".repeat(bar_len);
            let prob = count as f64 / total as f64 * 100.0;
            lines.push(format!(
                "{:>8} │{:<width$}│ {:>5} ({:>5.1}%)",
                state,
                bar,
                count,
                prob,
                width = bar_width
            ));
        }

        if sorted.len() > 16 {
            lines.push(format!("... and {} more states", sorted.len() - 16));
        }

        lines.join("\n")
    }

    /// Draw state probability distribution
    pub fn probability_chart(&self, probs: &HashMap<String, f64>) -> String {
        let mut lines = Vec::new();
        lines.push("Probability Distribution".to_string());
        lines.push("─".repeat(self.config.width));

        let mut sorted: Vec<_> = probs.iter().collect();
        sorted.sort_by(|a, b| b.1.partial_cmp(a.1).unwrap());

        let bar_width = self.config.width.saturating_sub(25);

        for (state, &prob) in sorted.iter().take(10) {
            let bar_len = (prob * bar_width as f64) as usize;
            let bar = "▓".repeat(bar_len);
            lines.push(format!(
                "{:>8} │{:<width$}│ {:>6.2}%",
                state,
                bar,
                prob * 100.0,
                width = bar_width
            ));
        }

        lines.join("\n")
    }

    /// Draw Bloch sphere (ASCII representation)
    pub fn bloch_sphere(&self, state: &QuantumState, qubit: usize) -> String {
        if state.num_qubits <= qubit {
            return "Invalid qubit index".to_string();
        }

        // Simplified Bloch sphere representation
        let amp_0 = state.amplitude(0);
        let amp_1 = state.amplitude(1);

        let theta = 2.0 * amp_1.norm().acos();
        let phi = amp_1.arg() - amp_0.arg();

        let x = theta.sin() * phi.cos();
        let y = theta.sin() * phi.sin();
        let z = theta.cos();

        let mut lines = Vec::new();
        lines.push(format!("Bloch Sphere (qubit {})", qubit));
        lines.push("─".repeat(30));
        lines.push(format!("θ = {:.4} rad", theta));
        lines.push(format!("φ = {:.4} rad", phi));
        lines.push("─".repeat(30));
        lines.push(format!("X = {:+.4}", x));
        lines.push(format!("Y = {:+.4}", y));
        lines.push(format!("Z = {:+.4}", z));

        lines.join("\n")
    }

    /// Generate summary text
    pub fn summary_text(&self, summary: &AnalysisSummary) -> String {
        let mut lines = Vec::new();
        lines.push("Analysis Summary".to_string());
        lines.push("═".repeat(40));
        lines.push(format!("Total shots:      {}", summary.total_shots));
        lines.push(format!("Unique states:    {}", summary.unique_states));
        if let Some((state, prob)) = &summary.most_probable {
            lines.push(format!("Most probable:    {} ({:.2}%)", state, prob * 100.0));
        }
        lines.push(format!("Entropy:          {:.4} bits", summary.entropy));
        lines.push(format!("⟨Z⟩ expectation:  {:+.4}", summary.z_expectation));
        lines.push(format!("Variance:         {:.4}", summary.variance));
        lines.push("═".repeat(40));

        lines.join("\n")
    }
}

impl Default for Visualizer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_analyzer() {
        let mut analyzer = ResultAnalyzer::new();

        let mut counts = HashMap::new();
        counts.insert("00".to_string(), 500);
        counts.insert("11".to_string(), 500);

        analyzer.add(ExecutionResult {
            counts,
            ..Default::default()
        });

        let probs = analyzer.probabilities();
        assert!((probs["00"] - 0.5).abs() < 0.01);
        assert!((probs["11"] - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_visualizer() {
        let mut counts = HashMap::new();
        counts.insert("00".to_string(), 512);
        counts.insert("11".to_string(), 512);

        let viz = Visualizer::new();
        let histogram = viz.histogram(&counts);

        assert!(histogram.contains("00"));
        assert!(histogram.contains("11"));
    }
}
