//! Sensor fusion and obstacle detection for WIA Smart Wheelchair

use super::types::*;
use super::lidar::LaserScan;

/*******************************************************************************
 * Odometry
 ******************************************************************************/

/// Wheel encoder data
#[derive(Debug, Clone)]
pub struct WheelEncoderData {
    pub header: Header,
    pub left_position: f64,   // radians
    pub right_position: f64,  // radians
    pub left_velocity: f64,   // rad/s
    pub right_velocity: f64,  // rad/s
}

/// Odometry output
#[derive(Debug, Clone)]
pub struct Odometry {
    pub header: Header,
    pub child_frame_id: String,
    pub pose: Pose2D,
    pub velocity: Velocity2D,
    pub covariance: CovarianceMatrix6x6,
}

/// 2D velocity
#[derive(Debug, Clone, Copy, Default)]
pub struct Velocity2D {
    pub linear: f64,   // m/s
    pub angular: f64,  // rad/s
}

impl Odometry {
    pub fn new(header: Header) -> Self {
        Self {
            header,
            child_frame_id: "base_link".to_string(),
            pose: Pose2D::default(),
            velocity: Velocity2D::default(),
            covariance: diagonal_covariance([0.01, 0.01, 0.0, 0.0, 0.0, 0.01]),
        }
    }
}

/// Differential drive odometry calculator
pub struct DifferentialDriveOdometry {
    pose: Pose2D,
    velocity: Velocity2D,
    last_left_pos: Option<f64>,
    last_right_pos: Option<f64>,
    last_time: Option<u64>,
    wheel_base: f64,
    wheel_radius: f64,
}

impl DifferentialDriveOdometry {
    pub fn new(wheel_base: f64, wheel_radius: f64) -> Self {
        Self {
            pose: Pose2D::default(),
            velocity: Velocity2D::default(),
            last_left_pos: None,
            last_right_pos: None,
            last_time: None,
            wheel_base,
            wheel_radius,
        }
    }

    /// Update odometry from encoder readings
    pub fn update(&mut self, encoders: &WheelEncoderData, timestamp_ms: u64) -> Odometry {
        if self.last_left_pos.is_none() || self.last_right_pos.is_none() {
            self.last_left_pos = Some(encoders.left_position);
            self.last_right_pos = Some(encoders.right_position);
            self.last_time = Some(timestamp_ms);
            return self.create_odometry(&encoders.header);
        }

        let dt = (timestamp_ms - self.last_time.unwrap()) as f64 / 1000.0;
        if dt <= 0.0 {
            return self.create_odometry(&encoders.header);
        }

        // Calculate wheel displacements
        let left_delta = encoders.left_position - self.last_left_pos.unwrap();
        let right_delta = encoders.right_position - self.last_right_pos.unwrap();

        // Convert to linear displacements
        let left_dist = left_delta * self.wheel_radius;
        let right_dist = right_delta * self.wheel_radius;

        // Calculate robot displacement
        let linear_dist = (left_dist + right_dist) / 2.0;
        let angular_dist = (right_dist - left_dist) / self.wheel_base;

        // Update pose
        if angular_dist.abs() < 1e-6 {
            // Straight line motion
            self.pose.x += linear_dist * self.pose.theta.cos();
            self.pose.y += linear_dist * self.pose.theta.sin();
        } else {
            // Arc motion
            let radius = linear_dist / angular_dist;
            self.pose.x += radius * ((self.pose.theta + angular_dist).sin() - self.pose.theta.sin());
            self.pose.y += radius * (self.pose.theta.cos() - (self.pose.theta + angular_dist).cos());
            self.pose.theta += angular_dist;
        }

        // Normalize theta
        self.pose.theta = normalize_angle(self.pose.theta);

        // Calculate velocities
        self.velocity.linear = linear_dist / dt;
        self.velocity.angular = angular_dist / dt;

        // Store for next iteration
        self.last_left_pos = Some(encoders.left_position);
        self.last_right_pos = Some(encoders.right_position);
        self.last_time = Some(timestamp_ms);

        self.create_odometry(&encoders.header)
    }

    fn create_odometry(&self, header: &Header) -> Odometry {
        let mut odom = Odometry::new(Header::new("odom"));
        odom.header.seq = header.seq;
        odom.pose = self.pose;
        odom.velocity = self.velocity;
        odom
    }

    pub fn reset(&mut self, pose: Option<Pose2D>) {
        self.pose = pose.unwrap_or_default();
        self.velocity = Velocity2D::default();
        self.last_left_pos = None;
        self.last_right_pos = None;
        self.last_time = None;
    }

    pub fn get_pose(&self) -> Pose2D {
        self.pose
    }

    pub fn get_velocity(&self) -> Velocity2D {
        self.velocity
    }
}

/*******************************************************************************
 * Obstacle Detection
 ******************************************************************************/

/// Obstacle type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObstacleType {
    Static,
    Dynamic,
    Unknown,
}

impl Default for ObstacleType {
    fn default() -> Self {
        Self::Unknown
    }
}

/// Detected obstacle
#[derive(Debug, Clone)]
pub struct Obstacle {
    pub id: u32,
    pub obstacle_type: ObstacleType,
    pub position: Vector2D,
    pub velocity: Option<Vector2D>,
    pub size: ObstacleSize,
    pub confidence: f32,
    pub last_seen: u64,
}

/// Obstacle size
#[derive(Debug, Clone, Copy, Default)]
pub struct ObstacleSize {
    pub width: f32,
    pub depth: f32,
}

/// Zone state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZoneState {
    Clear,
    Warning,
    Danger,
}

impl Default for ZoneState {
    fn default() -> Self {
        Self::Clear
    }
}

/// Zone status for all directions
#[derive(Debug, Clone, Copy, Default)]
pub struct ZoneStatus {
    pub front: ZoneState,
    pub front_left: ZoneState,
    pub front_right: ZoneState,
    pub left: ZoneState,
    pub right: ZoneState,
    pub rear: ZoneState,
}

/// Obstacle detector configuration
#[derive(Debug, Clone)]
pub struct ObstacleDetectorConfig {
    pub detection_range: f32,     // meters
    pub safety_margin: f32,       // meters
    pub update_rate: f32,         // Hz
    pub danger_distance: f32,     // meters
    pub warning_distance: f32,    // meters
    pub cluster_tolerance: f32,   // meters
    pub min_cluster_size: usize,
    pub tracking_timeout: u64,    // ms
    pub max_tracked_obstacles: usize,
}

impl Default for ObstacleDetectorConfig {
    fn default() -> Self {
        Self {
            detection_range: 5.0,
            safety_margin: 0.3,
            update_rate: 20.0,
            danger_distance: 0.5,
            warning_distance: 1.5,
            cluster_tolerance: 0.1,
            min_cluster_size: 3,
            tracking_timeout: 500,
            max_tracked_obstacles: 50,
        }
    }
}

/// Simple obstacle detector
pub struct ObstacleDetector {
    config: ObstacleDetectorConfig,
    obstacles: Vec<Obstacle>,
    zone_status: ZoneStatus,
    next_obstacle_id: u32,
}

impl ObstacleDetector {
    pub fn new(config: ObstacleDetectorConfig) -> Self {
        Self {
            config,
            obstacles: Vec::new(),
            zone_status: ZoneStatus::default(),
            next_obstacle_id: 1,
        }
    }

    /// Update with laser scan
    pub fn update_scan(&mut self, scan: &LaserScan, timestamp_ms: u64) {
        // Extract points from scan
        let mut points = Vec::new();

        for (i, &range) in scan.ranges.iter().enumerate() {
            if !range.is_finite() || range < scan.range_min || range > scan.range_max {
                continue;
            }
            if range > self.config.detection_range {
                continue;
            }

            let angle = scan.angle_min + i as f32 * scan.angle_increment;
            points.push(Vector2D {
                x: (range * angle.cos()) as f64,
                y: (range * angle.sin()) as f64,
            });
        }

        // Simple clustering
        let clusters = self.cluster_points(&points);

        // Convert clusters to obstacles
        self.obstacles = clusters.into_iter()
            .map(|cluster| self.cluster_to_obstacle(&cluster, timestamp_ms))
            .collect();

        // Update zone status
        self.update_zone_status(scan);
    }

    fn cluster_points(&self, points: &[Vector2D]) -> Vec<Vec<Vector2D>> {
        let mut clusters = Vec::new();
        let mut visited = vec![false; points.len()];

        for i in 0..points.len() {
            if visited[i] {
                continue;
            }

            let mut cluster = vec![points[i]];
            visited[i] = true;

            for j in (i + 1)..points.len() {
                if visited[j] {
                    continue;
                }

                let dist = self.distance(&points[i], &points[j]);
                if dist < self.config.cluster_tolerance as f64 {
                    cluster.push(points[j]);
                    visited[j] = true;
                }
            }

            if cluster.len() >= self.config.min_cluster_size {
                clusters.push(cluster);
            }

            if clusters.len() >= self.config.max_tracked_obstacles {
                break;
            }
        }

        clusters
    }

    fn cluster_to_obstacle(&mut self, cluster: &[Vector2D], timestamp: u64) -> Obstacle {
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut min_x = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_y = f64::NEG_INFINITY;

        for p in cluster {
            sum_x += p.x;
            sum_y += p.y;
            min_x = min_x.min(p.x);
            max_x = max_x.max(p.x);
            min_y = min_y.min(p.y);
            max_y = max_y.max(p.y);
        }

        let id = self.next_obstacle_id;
        self.next_obstacle_id += 1;

        Obstacle {
            id,
            obstacle_type: ObstacleType::Unknown,
            position: Vector2D {
                x: sum_x / cluster.len() as f64,
                y: sum_y / cluster.len() as f64,
            },
            velocity: None,
            size: ObstacleSize {
                width: (max_y - min_y) as f32 + self.config.safety_margin,
                depth: (max_x - min_x) as f32 + self.config.safety_margin,
            },
            confidence: (cluster.len() as f32 / 20.0).min(1.0),
            last_seen: timestamp,
        }
    }

    fn update_zone_status(&mut self, scan: &LaserScan) {
        use std::f32::consts::PI;

        // Define zone angles (in radians)
        let zones: [(fn(&mut ZoneStatus) -> &mut ZoneState, f32, f32); 6] = [
            (|z| &mut z.front, -PI / 6.0, PI / 6.0),
            (|z| &mut z.front_left, PI / 6.0, PI / 2.0),
            (|z| &mut z.front_right, -PI / 2.0, -PI / 6.0),
            (|z| &mut z.left, PI / 2.0, PI * 5.0 / 6.0),
            (|z| &mut z.right, -PI * 5.0 / 6.0, -PI / 2.0),
            (|z| &mut z.rear, PI * 5.0 / 6.0, PI),
        ];

        for (accessor, min_angle, max_angle) in zones {
            let mut min_dist = f32::INFINITY;

            for (i, &range) in scan.ranges.iter().enumerate() {
                if !range.is_finite() || range < scan.range_min || range > scan.range_max {
                    continue;
                }

                let angle = scan.angle_min + i as f32 * scan.angle_increment;
                let in_zone = (angle >= min_angle && angle <= max_angle)
                    || (min_angle > max_angle && (angle >= min_angle || angle <= max_angle));

                if in_zone {
                    min_dist = min_dist.min(range);
                }
            }

            let state = if min_dist <= self.config.danger_distance {
                ZoneState::Danger
            } else if min_dist <= self.config.warning_distance {
                ZoneState::Warning
            } else {
                ZoneState::Clear
            };

            *accessor(&mut self.zone_status) = state;
        }
    }

    fn distance(&self, a: &Vector2D, b: &Vector2D) -> f64 {
        ((a.x - b.x).powi(2) + (a.y - b.y).powi(2)).sqrt()
    }

    /// Get all detected obstacles
    pub fn get_obstacles(&self) -> &[Obstacle] {
        &self.obstacles
    }

    /// Get zone status
    pub fn get_zone_status(&self) -> ZoneStatus {
        self.zone_status
    }

    /// Check if path is clear
    pub fn is_path_clear(&self, direction: f64, distance: f64) -> bool {
        let half_width = 0.4;  // Robot half-width

        for obs in &self.obstacles {
            let obs_angle = obs.position.y.atan2(obs.position.x);
            let obs_dist = (obs.position.x.powi(2) + obs.position.y.powi(2)).sqrt();

            let angle_diff = normalize_angle(obs_angle - direction).abs();
            if angle_diff < std::f64::consts::FRAC_PI_4 && obs_dist < distance {
                let lateral_dist = obs_dist * angle_diff.sin();
                if lateral_dist < half_width + obs.size.width as f64 / 2.0 {
                    return false;
                }
            }
        }

        true
    }

    /// Get closest obstacle in direction
    pub fn get_closest_obstacle(&self, direction: f64, fov: f64) -> Option<&Obstacle> {
        let mut closest: Option<&Obstacle> = None;
        let mut min_dist = f64::INFINITY;

        for obs in &self.obstacles {
            let obs_angle = obs.position.y.atan2(obs.position.x);
            let angle_diff = normalize_angle(obs_angle - direction).abs();

            if angle_diff <= fov / 2.0 {
                let dist = (obs.position.x.powi(2) + obs.position.y.powi(2)).sqrt();
                if dist < min_dist {
                    min_dist = dist;
                    closest = Some(obs);
                }
            }
        }

        closest
    }
}

/// Transform pose by delta
pub fn transform_pose(pose: &Pose2D, delta: &Pose2D) -> Pose2D {
    let cos_theta = pose.theta.cos();
    let sin_theta = pose.theta.sin();

    Pose2D {
        x: pose.x + delta.x * cos_theta - delta.y * sin_theta,
        y: pose.y + delta.x * sin_theta + delta.y * cos_theta,
        theta: normalize_angle(pose.theta + delta.theta),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_differential_drive_odometry() {
        let mut odom = DifferentialDriveOdometry::new(0.5, 0.1);

        let encoders = WheelEncoderData {
            header: Header::new("encoder"),
            left_position: 0.0,
            right_position: 0.0,
            left_velocity: 0.0,
            right_velocity: 0.0,
        };

        odom.update(&encoders, 0);

        // Move forward
        let encoders = WheelEncoderData {
            header: Header::new("encoder"),
            left_position: 10.0,
            right_position: 10.0,
            left_velocity: 1.0,
            right_velocity: 1.0,
        };

        let result = odom.update(&encoders, 1000);
        assert!(result.pose.x > 0.0);
        assert!(result.pose.y.abs() < 0.01);
    }

    #[test]
    fn test_obstacle_detector() {
        let config = ObstacleDetectorConfig::default();
        let mut detector = ObstacleDetector::new(config);

        let scan = LaserScan {
            header: Header::new("lidar"),
            angle_min: -std::f32::consts::PI,
            angle_max: std::f32::consts::PI,
            angle_increment: 0.01,
            time_increment: 0.0,
            scan_time: 0.1,
            range_min: 0.1,
            range_max: 10.0,
            ranges: vec![1.0; 628],
            intensities: None,
        };

        detector.update_scan(&scan, 0);
        assert!(detector.get_obstacles().len() > 0);
    }
}
