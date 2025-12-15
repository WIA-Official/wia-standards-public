//! IMU sensor interface for WIA Smart Wheelchair

use super::types::*;

/// Supported IMU models
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImuModel {
    Mpu6050,
    Mpu9250,
    Bno055,
    Bno085,
    Icm20948,
    Lsm6ds3,
    Generic,
}

impl Default for ImuModel {
    fn default() -> Self {
        Self::Generic
    }
}

/// Accelerometer range (±g)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccelRange {
    G2 = 2,
    G4 = 4,
    G8 = 8,
    G16 = 16,
}

impl Default for AccelRange {
    fn default() -> Self {
        Self::G8
    }
}

/// Gyroscope range (±dps)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GyroRange {
    Dps250 = 250,
    Dps500 = 500,
    Dps1000 = 1000,
    Dps2000 = 2000,
}

impl Default for GyroRange {
    fn default() -> Self {
        Self::Dps1000
    }
}

/// IMU configuration
#[derive(Debug, Clone)]
pub struct ImuConfig {
    pub model: ImuModel,
    pub accel_range: AccelRange,
    pub gyro_range: GyroRange,
    pub sample_rate: u32,           // Hz
    pub enable_magnetometer: bool,
    pub enable_temperature: bool,
    pub frame_id: String,
}

impl Default for ImuConfig {
    fn default() -> Self {
        Self {
            model: ImuModel::Generic,
            accel_range: AccelRange::G8,
            gyro_range: GyroRange::Dps1000,
            sample_rate: 100,
            enable_magnetometer: false,
            enable_temperature: true,
            frame_id: "imu_link".to_string(),
        }
    }
}

impl ImuConfig {
    /// Get preset configuration for a specific model
    pub fn from_model(model: ImuModel) -> Self {
        let mut config = Self::default();
        config.model = model;

        match model {
            ImuModel::Mpu6050 => {
                config.sample_rate = 200;
                config.enable_magnetometer = false;
            }
            ImuModel::Mpu9250 => {
                config.sample_rate = 200;
                config.enable_magnetometer = true;
            }
            ImuModel::Bno055 => {
                config.accel_range = AccelRange::G4;
                config.gyro_range = GyroRange::Dps2000;
                config.sample_rate = 100;
                config.enable_magnetometer = true;
            }
            ImuModel::Bno085 => {
                config.sample_rate = 400;
                config.enable_magnetometer = true;
            }
            ImuModel::Icm20948 => {
                config.sample_rate = 225;
                config.enable_magnetometer = true;
            }
            ImuModel::Lsm6ds3 => {
                config.sample_rate = 416;
                config.enable_magnetometer = false;
            }
            ImuModel::Generic => {}
        }

        config
    }
}

/// IMU calibration data
#[derive(Debug, Clone, Default)]
pub struct ImuCalibration {
    pub accel_bias: Vector3D,
    pub accel_scale: Vector3D,
    pub gyro_bias: Vector3D,
    pub mag_bias: Option<Vector3D>,
    pub mag_scale: Option<Vector3D>,
    pub temperature: Option<f32>,
}

impl ImuCalibration {
    pub fn new() -> Self {
        Self {
            accel_scale: Vector3D::new(1.0, 1.0, 1.0),
            ..Default::default()
        }
    }
}

/// Raw IMU data
#[derive(Debug, Clone)]
pub struct ImuRawData {
    pub header: Header,
    pub accel_raw: Vector3D,
    pub gyro_raw: Vector3D,
    pub mag_raw: Option<Vector3D>,
    pub temp_raw: Option<i16>,
}

/// Processed IMU data
#[derive(Debug, Clone)]
pub struct ImuData {
    pub header: Header,
    pub accel: Vector3D,              // m/s²
    pub gyro: Vector3D,               // rad/s
    pub mag: Option<Vector3D>,        // μT
    pub orientation: Option<Quaternion>,
    pub linear_accel: Option<Vector3D>,  // gravity-compensated
    pub temperature: Option<f32>,     // °C
}

impl ImuData {
    pub fn new(header: Header) -> Self {
        Self {
            header,
            accel: Vector3D::zero(),
            gyro: Vector3D::zero(),
            mag: None,
            orientation: None,
            linear_accel: None,
            temperature: None,
        }
    }
}

/// Full IMU message (ROS-compatible)
#[derive(Debug, Clone)]
pub struct ImuMessage {
    pub header: Header,
    pub orientation: Quaternion,
    pub orientation_covariance: [f64; 9],
    pub angular_velocity: Vector3D,
    pub angular_velocity_covariance: [f64; 9],
    pub linear_acceleration: Vector3D,
    pub linear_acceleration_covariance: [f64; 9],
}

/// Calibration status (for sensors like BNO055)
#[derive(Debug, Clone, Copy, Default)]
pub struct CalibrationStatus {
    pub system: u8,   // 0-3
    pub gyro: u8,     // 0-3
    pub accel: u8,    // 0-3
    pub mag: Option<u8>,  // 0-3
}

impl CalibrationStatus {
    pub fn is_fully_calibrated(&self) -> bool {
        self.system == 3 && self.gyro == 3 && self.accel == 3
            && self.mag.map_or(true, |m| m == 3)
    }
}

/// IMU interface trait
pub trait ImuInterface: Send + Sync {
    /// Get current configuration
    fn config(&self) -> &ImuConfig;

    /// Get current status
    fn status(&self) -> SensorStatus;

    /// Initialize IMU
    fn initialize(&mut self) -> Result<(), String>;

    /// Start data acquisition
    fn start(&mut self) -> Result<(), String>;

    /// Stop data acquisition
    fn stop(&mut self) -> Result<(), String>;

    /// Get latest data
    fn get_latest_data(&self) -> Option<ImuData>;

    /// Get raw data
    fn get_raw_data(&self) -> Option<ImuRawData>;

    /// Get calibration status
    fn get_calibration_status(&self) -> CalibrationStatus;

    /// Run self-test
    fn self_test(&mut self) -> Result<bool, String>;

    /// Calibrate IMU
    fn calibrate(&mut self) -> Result<ImuCalibration, String>;

    /// Apply calibration
    fn apply_calibration(&mut self, cal: &ImuCalibration);
}

/// Apply calibration to raw IMU data
pub fn apply_calibration(raw: &ImuRawData, cal: &ImuCalibration) -> ImuData {
    let accel = Vector3D {
        x: (raw.accel_raw.x - cal.accel_bias.x) * cal.accel_scale.x,
        y: (raw.accel_raw.y - cal.accel_bias.y) * cal.accel_scale.y,
        z: (raw.accel_raw.z - cal.accel_bias.z) * cal.accel_scale.z,
    };

    let gyro = Vector3D {
        x: raw.gyro_raw.x - cal.gyro_bias.x,
        y: raw.gyro_raw.y - cal.gyro_bias.y,
        z: raw.gyro_raw.z - cal.gyro_bias.z,
    };

    let mag = raw.mag_raw.as_ref().and_then(|m| {
        cal.mag_bias.as_ref().and_then(|bias| {
            cal.mag_scale.as_ref().map(|scale| {
                Vector3D {
                    x: (m.x - bias.x) * scale.x,
                    y: (m.y - bias.y) * scale.y,
                    z: (m.z - bias.z) * scale.z,
                }
            })
        })
    });

    ImuData {
        header: raw.header.clone(),
        accel,
        gyro,
        mag,
        orientation: None,
        linear_accel: None,
        temperature: None,
    }
}

/// Calculate roll and pitch from accelerometer
pub fn accel_to_roll_pitch(accel: &Vector3D) -> (f64, f64) {
    let roll = accel.y.atan2(accel.z);
    let pitch = (-accel.x).atan2((accel.y * accel.y + accel.z * accel.z).sqrt());
    (roll, pitch)
}

/// Calculate yaw from magnetometer (with tilt compensation)
pub fn mag_to_yaw(mag: &Vector3D, roll: f64, pitch: f64) -> f64 {
    let cos_roll = roll.cos();
    let sin_roll = roll.sin();
    let cos_pitch = pitch.cos();
    let sin_pitch = pitch.sin();

    // Tilt-compensated magnetic field
    let mag_x = mag.x * cos_pitch + mag.y * sin_roll * sin_pitch + mag.z * cos_roll * sin_pitch;
    let mag_y = mag.y * cos_roll - mag.z * sin_roll;

    (-mag_y).atan2(mag_x)
}

/// Remove gravity from acceleration using orientation
pub fn remove_gravity(accel: &Vector3D, orientation: &Quaternion, gravity: f64) -> Vector3D {
    let q = orientation;

    // Rotate gravity vector by inverse of orientation
    let gx = 2.0 * (q.x * q.z - q.w * q.y) * gravity;
    let gy = 2.0 * (q.w * q.x + q.y * q.z) * gravity;
    let gz = (q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z) * gravity;

    Vector3D {
        x: accel.x - gx,
        y: accel.y - gy,
        z: accel.z - gz,
    }
}

/// Simple complementary filter for orientation
pub struct ComplementaryFilter {
    orientation: Quaternion,
    last_timestamp: Option<u64>,
    alpha: f64,
}

impl ComplementaryFilter {
    pub fn new(alpha: f64) -> Self {
        Self {
            orientation: Quaternion::identity(),
            last_timestamp: None,
            alpha,
        }
    }

    pub fn update(&mut self, imu: &ImuData, timestamp_ms: u64) -> Quaternion {
        let dt = match self.last_timestamp {
            Some(last) if timestamp_ms > last => (timestamp_ms - last) as f64 / 1000.0,
            _ => {
                self.last_timestamp = Some(timestamp_ms);
                self.initialize_from_accel(&imu.accel, imu.mag.as_ref());
                return self.orientation;
            }
        };

        if dt <= 0.0 || dt > 0.5 {
            self.last_timestamp = Some(timestamp_ms);
            self.initialize_from_accel(&imu.accel, imu.mag.as_ref());
            return self.orientation;
        }

        self.last_timestamp = Some(timestamp_ms);

        // Integrate gyroscope
        let gyro_quat = self.integrate_gyro(&imu.gyro, dt);

        // Get orientation from accelerometer/magnetometer
        let (roll, pitch) = accel_to_roll_pitch(&imu.accel);
        let yaw = imu.mag.as_ref()
            .map(|m| mag_to_yaw(m, roll, pitch))
            .unwrap_or_else(|| self.orientation.to_euler().2);
        let accel_quat = Quaternion::from_euler(roll, pitch, yaw);

        // Complementary filter
        self.orientation = slerp(&accel_quat, &gyro_quat, self.alpha);
        self.orientation
    }

    fn initialize_from_accel(&mut self, accel: &Vector3D, mag: Option<&Vector3D>) {
        let (roll, pitch) = accel_to_roll_pitch(accel);
        let yaw = mag.map(|m| mag_to_yaw(m, roll, pitch)).unwrap_or(0.0);
        self.orientation = Quaternion::from_euler(roll, pitch, yaw);
    }

    fn integrate_gyro(&self, gyro: &Vector3D, dt: f64) -> Quaternion {
        let half_dt = dt * 0.5;
        let dq = Quaternion {
            w: 1.0,
            x: gyro.x * half_dt,
            y: gyro.y * half_dt,
            z: gyro.z * half_dt,
        };

        multiply_quaternion(&self.orientation, &dq).normalize()
    }

    pub fn reset(&mut self) {
        self.orientation = Quaternion::identity();
        self.last_timestamp = None;
    }
}

fn multiply_quaternion(a: &Quaternion, b: &Quaternion) -> Quaternion {
    Quaternion {
        w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    }
}

fn slerp(a: &Quaternion, b: &Quaternion, t: f64) -> Quaternion {
    let mut dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;

    let b = if dot < 0.0 {
        dot = -dot;
        Quaternion { x: -b.x, y: -b.y, z: -b.z, w: -b.w }
    } else {
        *b
    };

    if dot > 0.9995 {
        // Linear interpolation
        return Quaternion {
            x: a.x + t * (b.x - a.x),
            y: a.y + t * (b.y - a.y),
            z: a.z + t * (b.z - a.z),
            w: a.w + t * (b.w - a.w),
        }.normalize();
    }

    let theta = dot.acos();
    let sin_theta = theta.sin();
    let wa = ((1.0 - t) * theta).sin() / sin_theta;
    let wb = (t * theta).sin() / sin_theta;

    Quaternion {
        x: wa * a.x + wb * b.x,
        y: wa * a.y + wb * b.y,
        z: wa * a.z + wb * b.z,
        w: wa * a.w + wb * b.w,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_accel_to_roll_pitch() {
        // Accelerometer pointing straight down (gravity)
        let accel = Vector3D::new(0.0, 0.0, 9.81);
        let (roll, pitch) = accel_to_roll_pitch(&accel);

        assert!(roll.abs() < 0.01);
        assert!(pitch.abs() < 0.01);
    }

    #[test]
    fn test_complementary_filter() {
        let mut filter = ComplementaryFilter::new(0.98);

        let imu = ImuData {
            header: Header::new("imu"),
            accel: Vector3D::new(0.0, 0.0, 9.81),
            gyro: Vector3D::zero(),
            mag: None,
            orientation: None,
            linear_accel: None,
            temperature: None,
        };

        let q = filter.update(&imu, 0);
        assert!((q.w - 1.0).abs() < 0.1);  // Should be close to identity
    }
}
