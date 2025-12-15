/**
 * @file camera.ts
 * @description WIA Smart Wheelchair Camera Interface
 * @version 1.0.0
 */

import {
  Header,
  Vector3D,
  PointCloud,
  BaseSensorConfig,
  SensorStatus,
  SensorCallback,
  Disposable,
} from './types';

/**
 * Camera type
 */
export type CameraType = 'rgb' | 'depth' | 'rgbd' | 'stereo' | 'thermal';

/**
 * Image format
 */
export type ImageFormat =
  | 'rgb8'
  | 'bgr8'
  | 'rgba8'
  | 'bgra8'
  | 'mono8'
  | 'mono16'
  | 'depth16'
  | 'depth32f';

/**
 * Camera configuration
 */
export interface CameraConfig extends BaseSensorConfig {
  type: CameraType;
  resolution: Resolution;
  fps: number;
  fov: FOV;
  format: ImageFormat;
  autoExposure: boolean;
  autoWhiteBalance: boolean;
}

/**
 * Resolution
 */
export interface Resolution {
  width: number;
  height: number;
}

/**
 * Field of View
 */
export interface FOV {
  horizontal: number;       // degrees
  vertical: number;         // degrees
  diagonal?: number;        // degrees
}

/**
 * Camera intrinsic parameters
 */
export interface CameraIntrinsics {
  fx: number;               // Focal length x (pixels)
  fy: number;               // Focal length y (pixels)
  cx: number;               // Principal point x (pixels)
  cy: number;               // Principal point y (pixels)
  k1?: number;              // Radial distortion k1
  k2?: number;              // Radial distortion k2
  k3?: number;              // Radial distortion k3
  p1?: number;              // Tangential distortion p1
  p2?: number;              // Tangential distortion p2
}

/**
 * Camera frame (RGB image)
 */
export interface CameraFrame {
  header: Header;
  format: ImageFormat;
  width: number;
  height: number;
  step: number;             // bytes per row
  data: Uint8Array;
  encoding?: string;
}

/**
 * Depth image
 */
export interface DepthImage {
  header: Header;
  width: number;
  height: number;
  step: number;
  data: Uint16Array | Float32Array;  // mm (uint16) or meters (float32)
  minDepth: number;         // meters
  maxDepth: number;         // meters
  unit: 'mm' | 'meters';
}

/**
 * RGBD frame (combined)
 */
export interface RGBDFrame {
  header: Header;
  color: CameraFrame;
  depth: DepthImage;
  aligned: boolean;         // Whether depth is aligned to color
}

/**
 * Stereo image pair
 */
export interface StereoFrame {
  header: Header;
  left: CameraFrame;
  right: CameraFrame;
  baseline: number;         // meters, distance between cameras
}

/**
 * Camera presets
 */
export const CAMERA_PRESETS: Record<string, Partial<CameraConfig>> = {
  realsense_d435: {
    type: 'rgbd',
    resolution: { width: 640, height: 480 },
    fps: 30,
    fov: { horizontal: 87, vertical: 58 },
  },
  realsense_d455: {
    type: 'rgbd',
    resolution: { width: 1280, height: 720 },
    fps: 30,
    fov: { horizontal: 87, vertical: 58 },
  },
  oak_d: {
    type: 'rgbd',
    resolution: { width: 1280, height: 720 },
    fps: 30,
    fov: { horizontal: 71.86, vertical: 56.74 },
  },
  zed2: {
    type: 'stereo',
    resolution: { width: 1280, height: 720 },
    fps: 30,
    fov: { horizontal: 110, vertical: 70 },
  },
  usb_webcam: {
    type: 'rgb',
    resolution: { width: 640, height: 480 },
    fps: 30,
    fov: { horizontal: 60, vertical: 45 },
  },
};

/**
 * Camera Interface
 */
export interface CameraInterface extends Disposable {
  /** Current configuration */
  readonly config: CameraConfig;

  /** Current status */
  readonly status: SensorStatus;

  /** Camera intrinsics */
  readonly intrinsics: CameraIntrinsics;

  /** Initialize camera */
  initialize(): Promise<void>;

  /** Start streaming */
  start(): Promise<void>;

  /** Stop streaming */
  stop(): Promise<void>;

  /** Register frame callback */
  onFrame(callback: SensorCallback<CameraFrame>): void;

  /** Remove frame callback */
  offFrame(callback: SensorCallback<CameraFrame>): void;

  /** Get latest frame */
  getLatestFrame(): CameraFrame | null;

  /** Capture single frame */
  capture(): Promise<CameraFrame>;

  /** Set exposure (if supported) */
  setExposure?(value: number): Promise<void>;

  /** Set gain (if supported) */
  setGain?(value: number): Promise<void>;
}

/**
 * Depth Camera Interface (extends CameraInterface)
 */
export interface DepthCameraInterface extends CameraInterface {
  /** Get depth image */
  getDepthImage(): DepthImage | null;

  /** Register depth callback */
  onDepth(callback: SensorCallback<DepthImage>): void;

  /** Remove depth callback */
  offDepth(callback: SensorCallback<DepthImage>): void;

  /** Convert depth to point cloud */
  toPointCloud(depth: DepthImage, color?: CameraFrame): PointCloud;

  /** Set depth range */
  setDepthRange(min: number, max: number): Promise<void>;
}

/**
 * RGBD Camera Interface
 */
export interface RGBDCameraInterface extends DepthCameraInterface {
  /** Get aligned RGBD frame */
  getRGBDFrame(): RGBDFrame | null;

  /** Register RGBD callback */
  onRGBD(callback: SensorCallback<RGBDFrame>): void;

  /** Remove RGBD callback */
  offRGBD(callback: SensorCallback<RGBDFrame>): void;

  /** Enable/disable depth-color alignment */
  setAlignment(enabled: boolean): Promise<void>;
}

/**
 * Convert depth image to point cloud
 */
export function depthToPointCloud(
  depth: DepthImage,
  intrinsics: CameraIntrinsics,
  color?: CameraFrame
): PointCloud {
  const points: Vector3D[] = [];
  const colors: { r: number; g: number; b: number }[] = [];

  const depthScale = depth.unit === 'mm' ? 0.001 : 1.0;

  for (let v = 0; v < depth.height; v++) {
    for (let u = 0; u < depth.width; u++) {
      const idx = v * depth.width + u;
      const d = (depth.data[idx] as number) * depthScale;

      if (d <= 0 || d < depth.minDepth || d > depth.maxDepth) {
        continue;
      }

      // Deproject pixel to 3D point
      const x = (u - intrinsics.cx) * d / intrinsics.fx;
      const y = (v - intrinsics.cy) * d / intrinsics.fy;
      const z = d;

      points.push({ x, y, z });

      // Add color if available
      if (color && color.format === 'rgb8') {
        const colorIdx = (v * color.width + u) * 3;
        colors.push({
          r: color.data[colorIdx],
          g: color.data[colorIdx + 1],
          b: color.data[colorIdx + 2],
        });
      }
    }
  }

  return {
    header: depth.header,
    points,
    colors: colors.length > 0 ? colors : undefined,
  };
}

/**
 * Project 3D point to pixel coordinates
 */
export function projectPoint(
  point: Vector3D,
  intrinsics: CameraIntrinsics
): { u: number; v: number } | null {
  if (point.z <= 0) {
    return null;
  }

  const u = (point.x * intrinsics.fx / point.z) + intrinsics.cx;
  const v = (point.y * intrinsics.fy / point.z) + intrinsics.cy;

  return { u, v };
}

/**
 * Deproject pixel to 3D ray
 */
export function deprojectPixel(
  u: number,
  v: number,
  intrinsics: CameraIntrinsics
): Vector3D {
  return {
    x: (u - intrinsics.cx) / intrinsics.fx,
    y: (v - intrinsics.cy) / intrinsics.fy,
    z: 1.0,
  };
}

/**
 * Apply undistortion to pixel coordinates
 */
export function undistortPoint(
  u: number,
  v: number,
  intrinsics: CameraIntrinsics
): { u: number; v: number } {
  const k1 = intrinsics.k1 ?? 0;
  const k2 = intrinsics.k2 ?? 0;
  const k3 = intrinsics.k3 ?? 0;
  const p1 = intrinsics.p1 ?? 0;
  const p2 = intrinsics.p2 ?? 0;

  // Normalize coordinates
  const x = (u - intrinsics.cx) / intrinsics.fx;
  const y = (v - intrinsics.cy) / intrinsics.fy;

  const r2 = x * x + y * y;
  const r4 = r2 * r2;
  const r6 = r4 * r2;

  // Radial distortion
  const radial = 1 + k1 * r2 + k2 * r4 + k3 * r6;

  // Tangential distortion
  const xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
  const yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;

  // Back to pixel coordinates
  return {
    u: xd * intrinsics.fx + intrinsics.cx,
    v: yd * intrinsics.fy + intrinsics.cy,
  };
}

/**
 * Resize image (simple nearest neighbor)
 */
export function resizeImage(
  frame: CameraFrame,
  newWidth: number,
  newHeight: number
): CameraFrame {
  const channels = frame.format.includes('rgba') || frame.format.includes('bgra') ? 4 :
                   frame.format.includes('rgb') || frame.format.includes('bgr') ? 3 : 1;

  const newData = new Uint8Array(newWidth * newHeight * channels);
  const xRatio = frame.width / newWidth;
  const yRatio = frame.height / newHeight;

  for (let y = 0; y < newHeight; y++) {
    for (let x = 0; x < newWidth; x++) {
      const srcX = Math.floor(x * xRatio);
      const srcY = Math.floor(y * yRatio);
      const srcIdx = (srcY * frame.width + srcX) * channels;
      const dstIdx = (y * newWidth + x) * channels;

      for (let c = 0; c < channels; c++) {
        newData[dstIdx + c] = frame.data[srcIdx + c];
      }
    }
  }

  return {
    ...frame,
    width: newWidth,
    height: newHeight,
    step: newWidth * channels,
    data: newData,
  };
}
