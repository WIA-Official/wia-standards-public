//! Spatial scene encoding for haptic feedback
//!
//! Encodes complex scenes with multiple obstacles into haptic patterns.

use super::{
    ActuatorActivation, SpatialDirection, Vector3,
    direction::{DirectionEncoder, DirectionEncoderConfig},
    distance::{DistanceEncoder, DistanceEncoderConfig, DistanceEncodingResult, ObstacleType, DistanceZone},
};

/// Obstacle in the scene
#[derive(Debug, Clone)]
pub struct Obstacle {
    /// Unique identifier
    pub id: u32,
    /// Obstacle type
    pub obstacle_type: ObstacleType,
    /// Direction from user (azimuth in degrees)
    pub direction: f32,
    /// Elevation angle (-90 to +90)
    pub elevation: f32,
    /// Distance in meters
    pub distance: f32,
    /// Angular width (degrees)
    pub width: Option<f32>,
    /// Height (meters)
    pub height: Option<f32>,
    /// Movement velocity
    pub velocity: Option<Vector3>,
    /// Is the obstacle moving
    pub is_moving: bool,
    /// Detection confidence (0-1)
    pub confidence: f32,
    /// Computed priority (0-1)
    pub priority: f32,
    /// Last seen timestamp (ms)
    pub last_seen: u64,
}

impl Obstacle {
    /// Create a new obstacle
    pub fn new(
        id: u32,
        obstacle_type: ObstacleType,
        direction: f32,
        distance: f32,
    ) -> Self {
        Self {
            id,
            obstacle_type,
            direction,
            elevation: 0.0,
            distance,
            width: None,
            height: None,
            velocity: None,
            is_moving: false,
            confidence: 1.0,
            priority: 0.0,
            last_seen: 0,
        }
    }

    /// Set elevation
    pub fn with_elevation(mut self, elevation: f32) -> Self {
        self.elevation = elevation;
        self
    }

    /// Set as moving with velocity
    pub fn with_velocity(mut self, velocity: Vector3) -> Self {
        self.velocity = Some(velocity);
        self.is_moving = true;
        self
    }

    /// Get spatial direction
    pub fn spatial_direction(&self) -> SpatialDirection {
        SpatialDirection::new(self.direction, self.elevation)
    }
}

/// Spatial scene containing multiple obstacles
#[derive(Debug, Clone, Default)]
pub struct SpatialScene {
    /// Timestamp of scene capture
    pub timestamp: u64,
    /// All detected obstacles
    pub obstacles: Vec<Obstacle>,
    /// User's heading (degrees)
    pub user_heading: f32,
    /// Navigation destination (if any)
    pub destination: Option<SpatialDirection>,
}

impl SpatialScene {
    /// Create a new empty scene
    pub fn new(timestamp: u64) -> Self {
        Self {
            timestamp,
            obstacles: Vec::new(),
            user_heading: 0.0,
            destination: None,
        }
    }

    /// Add an obstacle to the scene
    pub fn add_obstacle(&mut self, obstacle: Obstacle) {
        self.obstacles.push(obstacle);
    }

    /// Get obstacles sorted by distance
    pub fn by_distance(&self) -> Vec<&Obstacle> {
        let mut sorted: Vec<_> = self.obstacles.iter().collect();
        sorted.sort_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap());
        sorted
    }

    /// Get obstacles sorted by priority
    pub fn by_priority(&self) -> Vec<&Obstacle> {
        let mut sorted: Vec<_> = self.obstacles.iter().collect();
        sorted.sort_by(|a, b| b.priority.partial_cmp(&a.priority).unwrap());
        sorted
    }

    /// Get nearest obstacle
    pub fn nearest(&self) -> Option<&Obstacle> {
        self.obstacles.iter().min_by(|a, b| {
            a.distance.partial_cmp(&b.distance).unwrap()
        })
    }

    /// Get most critical obstacle (highest priority)
    pub fn most_critical(&self) -> Option<&Obstacle> {
        self.obstacles.iter().max_by(|a, b| {
            a.priority.partial_cmp(&b.priority).unwrap()
        })
    }

    /// Get obstacles in a direction range
    pub fn in_direction_range(&self, min_az: f32, max_az: f32) -> Vec<&Obstacle> {
        self.obstacles.iter().filter(|o| {
            let az = super::normalize_angle(o.direction);
            let min = super::normalize_angle(min_az);
            let max = super::normalize_angle(max_az);
            if min <= max {
                az >= min && az <= max
            } else {
                az >= min || az <= max
            }
        }).collect()
    }

    /// Calculate priorities for all obstacles
    pub fn calculate_priorities(&mut self) {
        let has_destination = self.destination.is_some();

        for obstacle in &mut self.obstacles {
            let mut priority = 0.0;

            // Distance factor (0.3 weight)
            let distance_factor = (1.0 - (obstacle.distance / 10.0).min(1.0)).powf(2.0);
            priority += distance_factor * 0.3;

            // Type factor (0.2 weight)
            priority += obstacle.obstacle_type.priority() * 0.2;

            // Approaching factor (0.25 weight)
            if let Some(velocity) = &obstacle.velocity {
                let direction_vec = obstacle.spatial_direction().to_unit_vector();
                let approach_speed = -velocity.dot(&direction_vec);
                if approach_speed > 0.0 {
                    priority += (approach_speed / 5.0).min(1.0) * 0.25;
                }
            }

            // In-path factor (0.15 weight)
            if has_destination {
                let dest = self.destination.as_ref().unwrap();
                let deviation = super::angle_difference(obstacle.direction, dest.azimuth).abs();
                if deviation < 30.0 {
                    priority += 0.15;
                }
            }

            // Novelty factor (0.1 weight)
            // Simplified: recently detected obstacles get bonus
            // In real implementation, would compare against previous timestamps

            obstacle.priority = priority.min(1.0);
        }
    }
}

/// Scene encoding strategy
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncodingStrategy {
    /// Time division: objects shown sequentially
    TimeDivision,
    /// Spatial division: different actuators for regions
    SpatialDivision,
    /// Frequency division: different frequencies for objects
    FrequencyDivision,
    /// Hybrid: continuous critical + TDM overview
    Hybrid,
}

/// Scene encoder configuration
#[derive(Debug, Clone)]
pub struct SceneEncoderConfig {
    /// Encoding strategy
    pub strategy: EncodingStrategy,
    /// Maximum objects to encode
    pub max_objects: usize,
    /// Time slot duration (ms) for TDM
    pub slot_duration: u32,
    /// Gap between slots (ms)
    pub gap_duration: u32,
    /// Cycle duration (ms)
    pub cycle_duration: u32,
    /// Direction encoder config
    pub direction_config: DirectionEncoderConfig,
    /// Distance encoder config
    pub distance_config: DistanceEncoderConfig,
}

impl Default for SceneEncoderConfig {
    fn default() -> Self {
        Self {
            strategy: EncodingStrategy::Hybrid,
            max_objects: 5,
            slot_duration: 150,
            gap_duration: 50,
            cycle_duration: 1000,
            direction_config: DirectionEncoderConfig::default(),
            distance_config: DistanceEncoderConfig::default(),
        }
    }
}

/// Encoded haptic output for a single obstacle
#[derive(Debug, Clone)]
pub struct ObstacleEncoding {
    /// Obstacle ID
    pub obstacle_id: u32,
    /// Actuator activations for direction
    pub activations: Vec<ActuatorActivation>,
    /// Distance encoding
    pub distance: DistanceEncodingResult,
    /// Start time in cycle (ms)
    pub start_time: u32,
    /// Duration (ms)
    pub duration: u32,
}

/// Encoded scene output
#[derive(Debug, Clone)]
pub struct SceneEncoding {
    /// Individual obstacle encodings
    pub obstacles: Vec<ObstacleEncoding>,
    /// Total cycle duration
    pub cycle_duration: u32,
    /// Should loop
    pub should_loop: bool,
    /// Primary (most critical) obstacle
    pub primary: Option<ObstacleEncoding>,
}

/// Scene encoder
pub struct SceneEncoder {
    config: SceneEncoderConfig,
    direction_encoder: DirectionEncoder,
    distance_encoder: DistanceEncoder,
}

impl SceneEncoder {
    /// Create a new scene encoder
    pub fn new(config: SceneEncoderConfig) -> Self {
        let direction_encoder = DirectionEncoder::new(config.direction_config.clone());
        let distance_encoder = DistanceEncoder::new(config.distance_config.clone());

        Self {
            config,
            direction_encoder,
            distance_encoder,
        }
    }

    /// Create encoder with default configuration
    pub fn default_encoder() -> Self {
        Self::new(SceneEncoderConfig::default())
    }

    /// Encode a spatial scene
    pub fn encode(&self, scene: &SpatialScene) -> SceneEncoding {
        match self.config.strategy {
            EncodingStrategy::TimeDivision => self.encode_tdm(scene),
            EncodingStrategy::SpatialDivision => self.encode_sdm(scene),
            EncodingStrategy::FrequencyDivision => self.encode_fdm(scene),
            EncodingStrategy::Hybrid => self.encode_hybrid(scene),
        }
    }

    /// Time Division Multiplexing encoding
    fn encode_tdm(&self, scene: &SpatialScene) -> SceneEncoding {
        let obstacles: Vec<_> = scene.by_priority()
            .into_iter()
            .take(self.config.max_objects)
            .collect();

        if obstacles.is_empty() {
            return SceneEncoding {
                obstacles: vec![],
                cycle_duration: self.config.cycle_duration,
                should_loop: true,
                primary: None,
            };
        }

        let slot_count = obstacles.len();
        let actual_slot = self.config.cycle_duration / slot_count as u32 - self.config.gap_duration;

        let encoded: Vec<_> = obstacles.iter().enumerate().map(|(i, obstacle)| {
            let direction = obstacle.spatial_direction();
            let activations = self.direction_encoder.encode(&direction);
            let distance = self.distance_encoder.encode_with_type(
                obstacle.distance,
                obstacle.obstacle_type,
            );

            ObstacleEncoding {
                obstacle_id: obstacle.id,
                activations,
                distance,
                start_time: i as u32 * (actual_slot + self.config.gap_duration),
                duration: actual_slot,
            }
        }).collect();

        let primary = encoded.first().cloned();

        SceneEncoding {
            obstacles: encoded,
            cycle_duration: self.config.cycle_duration,
            should_loop: true,
            primary,
        }
    }

    /// Spatial Division Multiplexing encoding
    fn encode_sdm(&self, scene: &SpatialScene) -> SceneEncoding {
        // Define 4 quadrants
        let quadrants = [
            (315.0, 45.0),   // Front
            (45.0, 135.0),   // Right
            (135.0, 225.0),  // Back
            (225.0, 315.0),  // Left
        ];

        let mut encoded = Vec::new();

        for (min_az, max_az) in quadrants {
            let in_quadrant = scene.in_direction_range(min_az, max_az);

            if let Some(nearest) = in_quadrant.iter()
                .min_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap())
            {
                let direction = nearest.spatial_direction();
                let activations = self.direction_encoder.encode(&direction);
                let distance = self.distance_encoder.encode_with_type(
                    nearest.distance,
                    nearest.obstacle_type,
                );

                encoded.push(ObstacleEncoding {
                    obstacle_id: nearest.id,
                    activations,
                    distance,
                    start_time: 0,  // All simultaneous in SDM
                    duration: self.config.cycle_duration,
                });
            }
        }

        let primary = scene.most_critical().map(|o| {
            let direction = o.spatial_direction();
            ObstacleEncoding {
                obstacle_id: o.id,
                activations: self.direction_encoder.encode(&direction),
                distance: self.distance_encoder.encode_with_type(o.distance, o.obstacle_type),
                start_time: 0,
                duration: self.config.cycle_duration,
            }
        });

        SceneEncoding {
            obstacles: encoded,
            cycle_duration: self.config.cycle_duration,
            should_loop: true,
            primary,
        }
    }

    /// Frequency Division Multiplexing encoding
    fn encode_fdm(&self, scene: &SpatialScene) -> SceneEncoding {
        // Group by priority/distance zone and assign frequencies
        let obstacles: Vec<_> = scene.by_priority()
            .into_iter()
            .take(self.config.max_objects)
            .collect();

        let encoded: Vec<_> = obstacles.iter().enumerate().map(|(i, obstacle)| {
            let direction = obstacle.spatial_direction();
            let activations = self.direction_encoder.encode(&direction);
            let mut distance = self.distance_encoder.encode_with_type(
                obstacle.distance,
                obstacle.obstacle_type,
            );

            // Assign frequency based on position in priority list
            // Higher priority = higher frequency
            let base_freq = 200 - (i as u16 * 40);
            distance.frequency = base_freq.max(60);

            ObstacleEncoding {
                obstacle_id: obstacle.id,
                activations,
                distance,
                start_time: 0,
                duration: self.config.cycle_duration,
            }
        }).collect();

        let primary = encoded.first().cloned();

        SceneEncoding {
            obstacles: encoded,
            cycle_duration: self.config.cycle_duration,
            should_loop: true,
            primary,
        }
    }

    /// Hybrid encoding (continuous critical + TDM overview)
    fn encode_hybrid(&self, scene: &SpatialScene) -> SceneEncoding {
        // Find critical obstacle (nearest in critical zone)
        let critical = scene.obstacles.iter()
            .filter(|o| o.distance < 2.0)  // Within 2m
            .min_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap());

        let primary = critical.map(|o| {
            let direction = o.spatial_direction();
            ObstacleEncoding {
                obstacle_id: o.id,
                activations: self.direction_encoder.encode(&direction),
                distance: self.distance_encoder.encode_with_type(o.distance, o.obstacle_type),
                start_time: 0,
                duration: self.config.cycle_duration,
            }
        });

        // TDM for remaining obstacles
        let critical_id = critical.map(|o| o.id);
        let others: Vec<_> = scene.obstacles.iter()
            .filter(|o| Some(o.id) != critical_id)
            .collect();

        let slot_count = others.len().min(3);  // Max 3 for overview
        let slot_duration = if slot_count > 0 {
            self.config.cycle_duration / (slot_count as u32 + 1)
        } else {
            self.config.cycle_duration
        };

        let mut encoded: Vec<_> = others.iter()
            .take(slot_count)
            .enumerate()
            .map(|(i, obstacle)| {
                let direction = obstacle.spatial_direction();
                let activations = self.direction_encoder.encode(&direction);
                let mut distance = self.distance_encoder.encode_with_type(
                    obstacle.distance,
                    obstacle.obstacle_type,
                );
                // Reduce intensity for overview
                distance.intensity *= 0.5;

                ObstacleEncoding {
                    obstacle_id: obstacle.id,
                    activations,
                    distance,
                    start_time: (i as u32 + 1) * slot_duration,
                    duration: slot_duration - self.config.gap_duration,
                }
            })
            .collect();

        // Add primary at the beginning if exists
        if let Some(ref p) = primary {
            encoded.insert(0, p.clone());
        }

        SceneEncoding {
            obstacles: encoded,
            cycle_duration: self.config.cycle_duration,
            should_loop: true,
            primary,
        }
    }

    /// Check if scene has any critical obstacles
    pub fn has_critical(&self, scene: &SpatialScene) -> bool {
        scene.obstacles.iter().any(|o| {
            self.distance_encoder.get_zone(o.distance) == DistanceZone::Critical
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_scene() -> SpatialScene {
        let mut scene = SpatialScene::new(0);
        scene.add_obstacle(Obstacle::new(1, ObstacleType::Wall, 0.0, 3.0));
        scene.add_obstacle(Obstacle::new(2, ObstacleType::Person, 45.0, 2.0));
        scene.add_obstacle(Obstacle::new(3, ObstacleType::Drop, 180.0, 1.0));
        scene.calculate_priorities();
        scene
    }

    #[test]
    fn test_scene_creation() {
        let scene = create_test_scene();
        assert_eq!(scene.obstacles.len(), 3);
    }

    #[test]
    fn test_scene_nearest() {
        let scene = create_test_scene();
        let nearest = scene.nearest().unwrap();
        assert_eq!(nearest.id, 3);  // Drop at 1m is nearest
    }

    #[test]
    fn test_scene_priority() {
        let scene = create_test_scene();
        let critical = scene.most_critical().unwrap();
        // Drop should have highest priority (dangerous type + close)
        assert_eq!(critical.obstacle_type, ObstacleType::Drop);
    }

    #[test]
    fn test_tdm_encoding() {
        let scene = create_test_scene();
        let encoder = SceneEncoder::new(SceneEncoderConfig {
            strategy: EncodingStrategy::TimeDivision,
            ..Default::default()
        });

        let encoding = encoder.encode(&scene);
        assert_eq!(encoding.obstacles.len(), 3);
        assert!(encoding.should_loop);
    }

    #[test]
    fn test_hybrid_encoding() {
        let scene = create_test_scene();
        let encoder = SceneEncoder::default_encoder();

        let encoding = encoder.encode(&scene);
        // Should have primary (the Drop obstacle)
        assert!(encoding.primary.is_some());
    }
}
