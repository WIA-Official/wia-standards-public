//! LiDAR sensor interface for WIA Smart Wheelchair

use super::types::*;

/// Supported LiDAR models
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LidarModel {
    RplidarA1,
    RplidarA2,
    RplidarA3,
    YdlidarX4,
    YdlidarG4,
    HokuyoUst10lx,
    SickTim551,
    Generic,
}

impl Default for LidarModel {
    fn default() -> Self {
        Self::Generic
    }
}

/// LiDAR configuration
#[derive(Debug, Clone)]
pub struct LidarConfig {
    pub model: LidarModel,
    pub scan_rate: f32,           // Hz (5-20)
    pub angular_resolution: f32,  // degrees
    pub max_range: f32,           // meters
    pub min_range: f32,           // meters
    pub frame_id: String,
    pub invert_scan: bool,
    pub angle_offset: f32,        // degrees
}

impl Default for LidarConfig {
    fn default() -> Self {
        Self {
            model: LidarModel::Generic,
            scan_rate: 10.0,
            angular_resolution: 1.0,
            max_range: 12.0,
            min_range: 0.1,
            frame_id: "lidar_link".to_string(),
            invert_scan: false,
            angle_offset: 0.0,
        }
    }
}

impl LidarConfig {
    /// Get preset configuration for a specific model
    pub fn from_model(model: LidarModel) -> Self {
        let mut config = Self::default();
        config.model = model;

        match model {
            LidarModel::RplidarA1 => {
                config.scan_rate = 5.5;
                config.angular_resolution = 1.0;
                config.max_range = 12.0;
                config.min_range = 0.15;
            }
            LidarModel::RplidarA2 => {
                config.scan_rate = 10.0;
                config.angular_resolution = 0.9;
                config.max_range = 18.0;
                config.min_range = 0.2;
            }
            LidarModel::RplidarA3 => {
                config.scan_rate = 15.0;
                config.angular_resolution = 0.225;
                config.max_range = 25.0;
                config.min_range = 0.2;
            }
            LidarModel::YdlidarX4 => {
                config.scan_rate = 7.0;
                config.angular_resolution = 0.5;
                config.max_range = 10.0;
                config.min_range = 0.12;
            }
            LidarModel::YdlidarG4 => {
                config.scan_rate = 9.0;
                config.angular_resolution = 0.28;
                config.max_range = 16.0;
                config.min_range = 0.28;
            }
            LidarModel::HokuyoUst10lx => {
                config.scan_rate = 40.0;
                config.angular_resolution = 0.25;
                config.max_range = 10.0;
                config.min_range = 0.06;
            }
            LidarModel::SickTim551 => {
                config.scan_rate = 15.0;
                config.angular_resolution = 0.33;
                config.max_range = 10.0;
                config.min_range = 0.05;
            }
            LidarModel::Generic => {}
        }

        config
    }
}

/// Laser scan data
#[derive(Debug, Clone)]
pub struct LaserScan {
    pub header: Header,
    pub angle_min: f32,           // radians
    pub angle_max: f32,           // radians
    pub angle_increment: f32,     // radians
    pub time_increment: f32,      // seconds
    pub scan_time: f32,           // seconds
    pub range_min: f32,           // meters
    pub range_max: f32,           // meters
    pub ranges: Vec<f32>,         // meters
    pub intensities: Option<Vec<f32>>,
}

impl LaserScan {
    /// Create a new laser scan
    pub fn new(header: Header, ranges: Vec<f32>) -> Self {
        let num_readings = ranges.len();
        let angle_range = std::f32::consts::PI * 2.0;

        Self {
            header,
            angle_min: -std::f32::consts::PI,
            angle_max: std::f32::consts::PI,
            angle_increment: angle_range / num_readings as f32,
            time_increment: 0.0,
            scan_time: 0.1,
            range_min: 0.1,
            range_max: 12.0,
            ranges,
            intensities: None,
        }
    }

    /// Get the number of valid readings
    pub fn valid_count(&self) -> usize {
        self.ranges.iter()
            .filter(|&r| r.is_finite() && *r >= self.range_min && *r <= self.range_max)
            .count()
    }

    /// Get angle for a given index
    pub fn angle_at(&self, index: usize) -> f32 {
        self.angle_min + (index as f32) * self.angle_increment
    }

    /// Convert to point cloud
    pub fn to_point_cloud(&self) -> Vec<Vector2D> {
        let mut points = Vec::with_capacity(self.valid_count());

        for (i, &range) in self.ranges.iter().enumerate() {
            if !range.is_finite() || range < self.range_min || range > self.range_max {
                continue;
            }

            let angle = self.angle_at(i);
            points.push(Vector2D {
                x: (range * angle.cos()) as f64,
                y: (range * angle.sin()) as f64,
            });
        }

        points
    }

    /// Filter by range
    pub fn filter_by_range(&self, min: f32, max: f32) -> Self {
        let mut filtered = self.clone();
        for range in &mut filtered.ranges {
            if *range < min || *range > max {
                *range = f32::INFINITY;
            }
        }
        filtered
    }

    /// Filter by angle
    pub fn filter_by_angle(&self, min_angle: f32, max_angle: f32) -> Self {
        let start_idx = ((min_angle - self.angle_min) / self.angle_increment).max(0.0) as usize;
        let end_idx = ((max_angle - self.angle_min) / self.angle_increment).min(self.ranges.len() as f32) as usize;

        let mut filtered = self.clone();
        filtered.ranges = self.ranges[start_idx..end_idx].to_vec();
        filtered.angle_min = self.angle_min + start_idx as f32 * self.angle_increment;
        filtered.angle_max = self.angle_min + end_idx as f32 * self.angle_increment;

        if let Some(ref intensities) = self.intensities {
            filtered.intensities = Some(intensities[start_idx..end_idx].to_vec());
        }

        filtered
    }

    /// Downsample scan
    pub fn downsample(&self, factor: usize) -> Self {
        let mut filtered = self.clone();
        filtered.ranges = self.ranges.iter()
            .step_by(factor)
            .copied()
            .collect();
        filtered.angle_increment *= factor as f32;
        filtered.time_increment *= factor as f32;

        if let Some(ref intensities) = self.intensities {
            filtered.intensities = Some(
                intensities.iter().step_by(factor).copied().collect()
            );
        }

        filtered
    }
}

/// Scan statistics
#[derive(Debug, Clone, Default)]
pub struct ScanStatistics {
    pub valid_points: usize,
    pub invalid_points: usize,
    pub average_range: f32,
    pub min_range: f32,
    pub max_range: f32,
    pub scan_duration_ms: f32,
}

impl ScanStatistics {
    pub fn from_scan(scan: &LaserScan) -> Self {
        let mut valid = 0;
        let mut sum = 0.0f32;
        let mut min = f32::INFINITY;
        let mut max = f32::NEG_INFINITY;

        for &range in &scan.ranges {
            if range.is_finite() && range >= scan.range_min && range <= scan.range_max {
                valid += 1;
                sum += range;
                min = min.min(range);
                max = max.max(range);
            }
        }

        Self {
            valid_points: valid,
            invalid_points: scan.ranges.len() - valid,
            average_range: if valid > 0 { sum / valid as f32 } else { 0.0 },
            min_range: if min.is_finite() { min } else { 0.0 },
            max_range: if max.is_finite() { max } else { 0.0 },
            scan_duration_ms: scan.scan_time * 1000.0,
        }
    }
}

/// LiDAR interface trait
pub trait LidarInterface: Send + Sync {
    /// Get current configuration
    fn config(&self) -> &LidarConfig;

    /// Get current status
    fn status(&self) -> SensorStatus;

    /// Initialize the LiDAR
    fn initialize(&mut self) -> Result<(), String>;

    /// Start scanning
    fn start(&mut self) -> Result<(), String>;

    /// Stop scanning
    fn stop(&mut self) -> Result<(), String>;

    /// Get latest scan
    fn get_latest_scan(&self) -> Option<LaserScan>;

    /// Get scan statistics
    fn get_statistics(&self) -> ScanStatistics;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_laser_scan_to_point_cloud() {
        let ranges = vec![1.0, 1.0, 1.0, 1.0];
        let mut scan = LaserScan::new(Header::new("test"), ranges);
        scan.angle_min = 0.0;
        scan.angle_max = std::f32::consts::PI;
        scan.angle_increment = std::f32::consts::FRAC_PI_2;

        let points = scan.to_point_cloud();
        assert_eq!(points.len(), 4);
    }

    #[test]
    fn test_scan_statistics() {
        let ranges = vec![1.0, 2.0, f32::INFINITY, 3.0];
        let scan = LaserScan::new(Header::new("test"), ranges);

        let stats = ScanStatistics::from_scan(&scan);
        assert_eq!(stats.valid_points, 3);
        assert_eq!(stats.invalid_points, 1);
        assert!((stats.average_range - 2.0).abs() < 0.01);
    }
}
