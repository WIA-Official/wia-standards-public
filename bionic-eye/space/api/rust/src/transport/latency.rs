//! Latency simulation for deep space communication

use serde::{Deserialize, Serialize};

/// Deep space targets
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DeepSpaceTarget {
    /// Low Earth Orbit (400 km)
    Leo,
    /// Geostationary Earth Orbit (35,786 km)
    Geo,
    /// Moon (384,400 km)
    Moon,
    /// Mars at closest approach (~55 million km)
    MarsOpposition,
    /// Mars at conjunction (~400 million km)
    MarsConjunction,
    /// Jupiter (~600-900 million km)
    Jupiter,
    /// Saturn (~1.2-1.7 billion km)
    Saturn,
    /// Neptune (~4.3-4.7 billion km)
    Neptune,
    /// Voyager 1 (~24 billion km)
    Voyager1,
    /// Alpha Centauri (4.37 light-years)
    AlphaCentauri,
}

impl DeepSpaceTarget {
    /// Get one-way light delay in milliseconds
    pub fn light_delay_ms(&self) -> u64 {
        match self {
            DeepSpaceTarget::Leo => 2,              // ~2ms
            DeepSpaceTarget::Geo => 120,            // ~120ms
            DeepSpaceTarget::Moon => 1_300,         // ~1.3s
            DeepSpaceTarget::MarsOpposition => 180_000,    // ~3 min
            DeepSpaceTarget::MarsConjunction => 1_320_000, // ~22 min
            DeepSpaceTarget::Jupiter => 2_700_000,  // ~45 min
            DeepSpaceTarget::Saturn => 4_800_000,   // ~80 min
            DeepSpaceTarget::Neptune => 14_400_000, // ~4 hours
            DeepSpaceTarget::Voyager1 => 79_200_000, // ~22 hours
            DeepSpaceTarget::AlphaCentauri => 137_797_920_000_000, // ~4.37 years
        }
    }

    /// Get round-trip delay in milliseconds
    pub fn round_trip_delay_ms(&self) -> u64 {
        self.light_delay_ms() * 2
    }

    /// Get human-readable delay string
    pub fn delay_string(&self) -> String {
        let ms = self.light_delay_ms();
        if ms < 1000 {
            format!("{}ms", ms)
        } else if ms < 60_000 {
            format!("{:.1}s", ms as f64 / 1000.0)
        } else if ms < 3_600_000 {
            format!("{:.1}min", ms as f64 / 60_000.0)
        } else if ms < 86_400_000 {
            format!("{:.1}h", ms as f64 / 3_600_000.0)
        } else if ms < 31_536_000_000 {
            format!("{:.1}d", ms as f64 / 86_400_000.0)
        } else {
            format!("{:.2}y", ms as f64 / 31_536_000_000.0)
        }
    }
}

/// Latency configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LatencyConfig {
    /// Base one-way delay in milliseconds
    pub base_delay_ms: u64,

    /// Jitter as fraction of base delay (0.0 - 1.0)
    pub jitter_factor: f64,

    /// Packet loss probability (0.0 - 1.0)
    pub packet_loss: f64,

    /// Signal degradation probability (0.0 - 1.0)
    pub signal_degradation: f64,
}

impl LatencyConfig {
    /// Create config for specific target
    pub fn for_target(target: DeepSpaceTarget) -> Self {
        let base_delay_ms = target.light_delay_ms();

        // Jitter increases with distance
        let jitter_factor = match target {
            DeepSpaceTarget::Leo | DeepSpaceTarget::Geo => 0.01,
            DeepSpaceTarget::Moon => 0.02,
            DeepSpaceTarget::MarsOpposition | DeepSpaceTarget::MarsConjunction => 0.05,
            DeepSpaceTarget::Jupiter | DeepSpaceTarget::Saturn => 0.08,
            _ => 0.1,
        };

        // Packet loss increases with distance
        let packet_loss = match target {
            DeepSpaceTarget::Leo | DeepSpaceTarget::Geo => 0.0001,
            DeepSpaceTarget::Moon => 0.001,
            DeepSpaceTarget::MarsOpposition => 0.005,
            DeepSpaceTarget::MarsConjunction => 0.01, // Solar conjunction causes issues
            _ => 0.02,
        };

        Self {
            base_delay_ms,
            jitter_factor,
            packet_loss,
            signal_degradation: packet_loss * 2.0,
        }
    }

    /// Create custom config
    pub fn custom(base_delay_ms: u64, jitter_factor: f64) -> Self {
        Self {
            base_delay_ms,
            jitter_factor,
            packet_loss: 0.0,
            signal_degradation: 0.0,
        }
    }

    /// LEO config
    pub fn leo() -> Self {
        Self::for_target(DeepSpaceTarget::Leo)
    }

    /// GEO config
    pub fn geo() -> Self {
        Self::for_target(DeepSpaceTarget::Geo)
    }

    /// Moon config
    pub fn moon() -> Self {
        Self::for_target(DeepSpaceTarget::Moon)
    }

    /// Mars at closest approach
    pub fn mars_opposition() -> Self {
        Self::for_target(DeepSpaceTarget::MarsOpposition)
    }

    /// Mars at solar conjunction
    pub fn mars_conjunction() -> Self {
        Self::for_target(DeepSpaceTarget::MarsConjunction)
    }

    /// No latency (for local testing)
    pub fn none() -> Self {
        Self {
            base_delay_ms: 0,
            jitter_factor: 0.0,
            packet_loss: 0.0,
            signal_degradation: 0.0,
        }
    }

    /// Calculate actual delay with jitter
    pub fn calculate_delay(&self) -> u64 {
        if self.base_delay_ms == 0 {
            return 0;
        }

        let jitter_range = (self.base_delay_ms as f64 * self.jitter_factor) as i64;
        if jitter_range == 0 {
            return self.base_delay_ms;
        }

        // Simple pseudo-random jitter
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let random = (now % (jitter_range as u128 * 2 + 1)) as i64 - jitter_range;

        (self.base_delay_ms as i64 + random).max(0) as u64
    }

    /// Check if packet should be lost
    pub fn should_lose_packet(&self) -> bool {
        if self.packet_loss <= 0.0 {
            return false;
        }

        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let random = (now % 10000) as f64 / 10000.0;

        random < self.packet_loss
    }
}

impl Default for LatencyConfig {
    fn default() -> Self {
        Self::none()
    }
}

/// Latency simulator
pub struct LatencySimulator {
    config: LatencyConfig,
    stats: LatencyStats,
}

/// Latency statistics
#[derive(Debug, Clone, Default)]
pub struct LatencyStats {
    pub total_messages: u64,
    pub lost_messages: u64,
    pub degraded_messages: u64,
    pub total_delay_ms: u64,
    pub min_delay_ms: u64,
    pub max_delay_ms: u64,
}

impl LatencyStats {
    pub fn average_delay_ms(&self) -> f64 {
        if self.total_messages == 0 {
            0.0
        } else {
            self.total_delay_ms as f64 / self.total_messages as f64
        }
    }

    pub fn loss_rate(&self) -> f64 {
        if self.total_messages == 0 {
            0.0
        } else {
            self.lost_messages as f64 / self.total_messages as f64
        }
    }
}

impl LatencySimulator {
    /// Create new simulator
    pub fn new(config: LatencyConfig) -> Self {
        Self {
            config,
            stats: LatencyStats::default(),
        }
    }

    /// Apply simulated delay
    pub async fn apply_delay(&self) {
        let delay_ms = self.config.calculate_delay();
        if delay_ms > 0 {
            tokio::time::sleep(std::time::Duration::from_millis(delay_ms)).await;
        }
    }

    /// Simulate message transmission
    pub async fn simulate_transmission(&mut self) -> TransmissionResult {
        let delay_ms = self.config.calculate_delay();

        // Update stats
        self.stats.total_messages += 1;
        self.stats.total_delay_ms += delay_ms;

        if self.stats.min_delay_ms == 0 || delay_ms < self.stats.min_delay_ms {
            self.stats.min_delay_ms = delay_ms;
        }
        if delay_ms > self.stats.max_delay_ms {
            self.stats.max_delay_ms = delay_ms;
        }

        // Check for packet loss
        if self.config.should_lose_packet() {
            self.stats.lost_messages += 1;
            return TransmissionResult::Lost;
        }

        // Apply delay
        if delay_ms > 0 {
            tokio::time::sleep(std::time::Duration::from_millis(delay_ms)).await;
        }

        // Check for signal degradation
        if self.config.signal_degradation > 0.0 {
            let now = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos();
            let random = (now % 10000) as f64 / 10000.0;
            if random < self.config.signal_degradation {
                self.stats.degraded_messages += 1;
                return TransmissionResult::Degraded { delay_ms };
            }
        }

        TransmissionResult::Success { delay_ms }
    }

    /// Get statistics
    pub fn stats(&self) -> &LatencyStats {
        &self.stats
    }

    /// Reset statistics
    pub fn reset_stats(&mut self) {
        self.stats = LatencyStats::default();
    }
}

/// Transmission result
#[derive(Debug, Clone)]
pub enum TransmissionResult {
    Success { delay_ms: u64 },
    Degraded { delay_ms: u64 },
    Lost,
}

impl TransmissionResult {
    pub fn is_success(&self) -> bool {
        matches!(self, TransmissionResult::Success { .. })
    }

    pub fn delay_ms(&self) -> Option<u64> {
        match self {
            TransmissionResult::Success { delay_ms } => Some(*delay_ms),
            TransmissionResult::Degraded { delay_ms } => Some(*delay_ms),
            TransmissionResult::Lost => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_deep_space_delays() {
        assert!(DeepSpaceTarget::Leo.light_delay_ms() < DeepSpaceTarget::Moon.light_delay_ms());
        assert!(
            DeepSpaceTarget::Moon.light_delay_ms() < DeepSpaceTarget::MarsOpposition.light_delay_ms()
        );
        assert!(
            DeepSpaceTarget::MarsOpposition.light_delay_ms()
                < DeepSpaceTarget::MarsConjunction.light_delay_ms()
        );
    }

    #[test]
    fn test_round_trip_delay() {
        let target = DeepSpaceTarget::Moon;
        assert_eq!(target.round_trip_delay_ms(), target.light_delay_ms() * 2);
    }

    #[test]
    fn test_delay_string() {
        assert_eq!(DeepSpaceTarget::Leo.delay_string(), "2ms");
        assert!(DeepSpaceTarget::Moon.delay_string().contains('s'));
        assert!(DeepSpaceTarget::MarsOpposition.delay_string().contains("min"));
    }

    #[test]
    fn test_latency_config_presets() {
        let leo = LatencyConfig::leo();
        let mars = LatencyConfig::mars_opposition();

        assert!(leo.base_delay_ms < mars.base_delay_ms);
        assert!(leo.packet_loss < mars.packet_loss);
    }

    #[test]
    fn test_calculate_delay() {
        let config = LatencyConfig::custom(100, 0.0);
        assert_eq!(config.calculate_delay(), 100);

        let config_with_jitter = LatencyConfig::custom(100, 0.1);
        let delay = config_with_jitter.calculate_delay();
        assert!(delay >= 90 && delay <= 110);
    }

    #[test]
    fn test_latency_config_none() {
        let config = LatencyConfig::none();
        assert_eq!(config.base_delay_ms, 0);
        assert_eq!(config.calculate_delay(), 0);
    }

    #[tokio::test]
    async fn test_latency_simulator() {
        let config = LatencyConfig::custom(10, 0.0);
        let mut simulator = LatencySimulator::new(config);

        let result = simulator.simulate_transmission().await;
        assert!(result.is_success());
        assert_eq!(simulator.stats().total_messages, 1);
    }

    #[test]
    fn test_latency_stats() {
        let mut stats = LatencyStats::default();
        stats.total_messages = 100;
        stats.lost_messages = 5;
        stats.total_delay_ms = 10000;

        assert_eq!(stats.average_delay_ms(), 100.0);
        assert_eq!(stats.loss_rate(), 0.05);
    }

    #[test]
    fn test_transmission_result() {
        let success = TransmissionResult::Success { delay_ms: 100 };
        assert!(success.is_success());
        assert_eq!(success.delay_ms(), Some(100));

        let lost = TransmissionResult::Lost;
        assert!(!lost.is_success());
        assert!(lost.delay_ms().is_none());
    }
}
