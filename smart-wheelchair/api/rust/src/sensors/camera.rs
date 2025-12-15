//! Camera sensor interface for WIA Smart Wheelchair

use super::types::*;

/// Camera type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CameraType {
    Rgb,
    Depth,
    Rgbd,
    Stereo,
    Thermal,
}

impl Default for CameraType {
    fn default() -> Self {
        Self::Rgb
    }
}

/// Image format
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImageFormat {
    Rgb8,
    Bgr8,
    Rgba8,
    Bgra8,
    Mono8,
    Mono16,
    Depth16,
    Depth32f,
}

impl Default for ImageFormat {
    fn default() -> Self {
        Self::Rgb8
    }
}

impl ImageFormat {
    /// Get bytes per pixel
    pub fn bytes_per_pixel(&self) -> usize {
        match self {
            Self::Mono8 => 1,
            Self::Mono16 | Self::Depth16 => 2,
            Self::Rgb8 | Self::Bgr8 => 3,
            Self::Rgba8 | Self::Bgra8 | Self::Depth32f => 4,
        }
    }
}

/// Camera resolution
#[derive(Debug, Clone, Copy, Default)]
pub struct Resolution {
    pub width: u32,
    pub height: u32,
}

impl Resolution {
    pub fn new(width: u32, height: u32) -> Self {
        Self { width, height }
    }

    pub fn pixel_count(&self) -> usize {
        (self.width * self.height) as usize
    }
}

/// Field of view
#[derive(Debug, Clone, Copy, Default)]
pub struct FieldOfView {
    pub horizontal: f32,  // degrees
    pub vertical: f32,    // degrees
}

/// Camera intrinsic parameters
#[derive(Debug, Clone, Copy)]
pub struct CameraIntrinsics {
    pub fx: f64,  // Focal length x (pixels)
    pub fy: f64,  // Focal length y (pixels)
    pub cx: f64,  // Principal point x (pixels)
    pub cy: f64,  // Principal point y (pixels)
    pub k1: f64,  // Radial distortion
    pub k2: f64,
    pub k3: f64,
    pub p1: f64,  // Tangential distortion
    pub p2: f64,
}

impl Default for CameraIntrinsics {
    fn default() -> Self {
        Self {
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
            k1: 0.0,
            k2: 0.0,
            k3: 0.0,
            p1: 0.0,
            p2: 0.0,
        }
    }
}

impl CameraIntrinsics {
    /// Project 3D point to pixel coordinates
    pub fn project(&self, point: &Vector3D) -> Option<(f64, f64)> {
        if point.z <= 0.0 {
            return None;
        }

        let u = (point.x * self.fx / point.z) + self.cx;
        let v = (point.y * self.fy / point.z) + self.cy;

        Some((u, v))
    }

    /// Deproject pixel to 3D ray (unit vector)
    pub fn deproject(&self, u: f64, v: f64) -> Vector3D {
        Vector3D {
            x: (u - self.cx) / self.fx,
            y: (v - self.cy) / self.fy,
            z: 1.0,
        }
    }

    /// Deproject pixel with depth to 3D point
    pub fn deproject_with_depth(&self, u: f64, v: f64, depth: f64) -> Vector3D {
        Vector3D {
            x: (u - self.cx) * depth / self.fx,
            y: (v - self.cy) * depth / self.fy,
            z: depth,
        }
    }
}

/// Camera configuration
#[derive(Debug, Clone)]
pub struct CameraConfig {
    pub camera_type: CameraType,
    pub resolution: Resolution,
    pub fps: u32,
    pub fov: FieldOfView,
    pub format: ImageFormat,
    pub frame_id: String,
    pub auto_exposure: bool,
    pub auto_white_balance: bool,
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            camera_type: CameraType::Rgb,
            resolution: Resolution::new(640, 480),
            fps: 30,
            fov: FieldOfView { horizontal: 60.0, vertical: 45.0 },
            format: ImageFormat::Rgb8,
            frame_id: "camera_link".to_string(),
            auto_exposure: true,
            auto_white_balance: true,
        }
    }
}

/// Camera frame (image)
#[derive(Debug, Clone)]
pub struct CameraFrame {
    pub header: Header,
    pub format: ImageFormat,
    pub width: u32,
    pub height: u32,
    pub step: u32,  // bytes per row
    pub data: Vec<u8>,
}

impl CameraFrame {
    pub fn new(header: Header, width: u32, height: u32, format: ImageFormat) -> Self {
        let bpp = format.bytes_per_pixel();
        let step = width * bpp as u32;
        let data = vec![0u8; (height * step) as usize];

        Self {
            header,
            format,
            width,
            height,
            step,
            data,
        }
    }

    /// Get pixel value at (x, y)
    pub fn get_pixel(&self, x: u32, y: u32) -> Option<&[u8]> {
        if x >= self.width || y >= self.height {
            return None;
        }

        let bpp = self.format.bytes_per_pixel();
        let offset = (y * self.step + x * bpp as u32) as usize;
        Some(&self.data[offset..offset + bpp])
    }
}

/// Depth image
#[derive(Debug, Clone)]
pub struct DepthImage {
    pub header: Header,
    pub width: u32,
    pub height: u32,
    pub data: Vec<u16>,       // millimeters
    pub min_depth: f32,       // meters
    pub max_depth: f32,       // meters
}

impl DepthImage {
    pub fn new(header: Header, width: u32, height: u32) -> Self {
        Self {
            header,
            width,
            height,
            data: vec![0u16; (width * height) as usize],
            min_depth: 0.1,
            max_depth: 10.0,
        }
    }

    /// Get depth at pixel (x, y) in meters
    pub fn get_depth(&self, x: u32, y: u32) -> Option<f32> {
        if x >= self.width || y >= self.height {
            return None;
        }

        let idx = (y * self.width + x) as usize;
        let depth_mm = self.data[idx];

        if depth_mm == 0 {
            None
        } else {
            Some(depth_mm as f32 / 1000.0)
        }
    }

    /// Convert to point cloud
    pub fn to_point_cloud(&self, intrinsics: &CameraIntrinsics) -> Vec<Vector3D> {
        let mut points = Vec::new();

        for v in 0..self.height {
            for u in 0..self.width {
                if let Some(depth) = self.get_depth(u, v) {
                    if depth >= self.min_depth && depth <= self.max_depth {
                        let point = intrinsics.deproject_with_depth(
                            u as f64,
                            v as f64,
                            depth as f64,
                        );
                        points.push(point);
                    }
                }
            }
        }

        points
    }
}

/// RGBD frame (combined color and depth)
#[derive(Debug, Clone)]
pub struct RgbdFrame {
    pub header: Header,
    pub color: CameraFrame,
    pub depth: DepthImage,
    pub aligned: bool,
}

/// Stereo image pair
#[derive(Debug, Clone)]
pub struct StereoFrame {
    pub header: Header,
    pub left: CameraFrame,
    pub right: CameraFrame,
    pub baseline: f32,  // meters
}

/// Camera interface trait
pub trait CameraInterface: Send + Sync {
    /// Get current configuration
    fn config(&self) -> &CameraConfig;

    /// Get current status
    fn status(&self) -> SensorStatus;

    /// Get camera intrinsics
    fn intrinsics(&self) -> &CameraIntrinsics;

    /// Initialize camera
    fn initialize(&mut self) -> Result<(), String>;

    /// Start streaming
    fn start(&mut self) -> Result<(), String>;

    /// Stop streaming
    fn stop(&mut self) -> Result<(), String>;

    /// Get latest frame
    fn get_latest_frame(&self) -> Option<CameraFrame>;

    /// Capture single frame
    fn capture(&mut self) -> Result<CameraFrame, String>;
}

/// Depth camera interface trait
pub trait DepthCameraInterface: CameraInterface {
    /// Get depth image
    fn get_depth_image(&self) -> Option<DepthImage>;

    /// Convert depth to point cloud
    fn to_point_cloud(&self, depth: &DepthImage) -> Vec<Vector3D>;

    /// Set depth range
    fn set_depth_range(&mut self, min: f32, max: f32) -> Result<(), String>;
}

/// RGBD camera interface trait
pub trait RgbdCameraInterface: DepthCameraInterface {
    /// Get aligned RGBD frame
    fn get_rgbd_frame(&self) -> Option<RgbdFrame>;

    /// Enable/disable depth-color alignment
    fn set_alignment(&mut self, enabled: bool) -> Result<(), String>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_camera_intrinsics_project() {
        let intrinsics = CameraIntrinsics {
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
            ..Default::default()
        };

        let point = Vector3D::new(0.0, 0.0, 1.0);
        let (u, v) = intrinsics.project(&point).unwrap();

        assert!((u - 320.0).abs() < 0.01);
        assert!((v - 240.0).abs() < 0.01);
    }

    #[test]
    fn test_camera_intrinsics_deproject() {
        let intrinsics = CameraIntrinsics {
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
            ..Default::default()
        };

        let point = intrinsics.deproject_with_depth(320.0, 240.0, 1.0);
        assert!((point.x).abs() < 0.01);
        assert!((point.y).abs() < 0.01);
        assert!((point.z - 1.0).abs() < 0.01);
    }
}
