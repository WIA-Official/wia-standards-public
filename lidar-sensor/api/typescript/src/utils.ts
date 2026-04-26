/**
 * Utility Functions
 */

import { Point3D, PointCloud, CalibrationParameters } from './types';

/**
 * Transform point from sensor frame to vehicle frame
 */
export function transformPoint(
  point: Point3D,
  calibration: CalibrationParameters
): Point3D {
  const { translation, rotation } = calibration.extrinsic;

  // Apply rotation (simplified - assumes rotation matrix)
  let rotatedX = point.x;
  let rotatedY = point.y;
  let rotatedZ = point.z;

  if (Array.isArray(rotation) && Array.isArray(rotation[0])) {
    const R = rotation as number[][];
    rotatedX = R[0][0] * point.x + R[0][1] * point.y + R[0][2] * point.z;
    rotatedY = R[1][0] * point.x + R[1][1] * point.y + R[1][2] * point.z;
    rotatedZ = R[2][0] * point.x + R[2][1] * point.y + R[2][2] * point.z;
  }

  // Apply translation
  return {
    x: rotatedX + translation[0],
    y: rotatedY + translation[1],
    z: rotatedZ + translation[2],
    intensity: point.intensity,
    timestamp: point.timestamp,
    ring: point.ring,
  };
}

/**
 * Convert point cloud to vehicle frame
 */
export function transformPointCloud(
  cloud: PointCloud,
  calibration: CalibrationParameters
): PointCloud {
  const transformedPoints = cloud.points.map((point) =>
    transformPoint(point, calibration)
  );

  return {
    ...cloud,
    points: transformedPoints,
    frameId: 'vehicle',
  };
}

/**
 * Compute statistics for point cloud
 */
export interface PointCloudStatistics {
  pointCount: number;
  bounds: {
    minX: number;
    maxX: number;
    minY: number;
    maxY: number;
    minZ: number;
    maxZ: number;
  };
  centroid: Point3D;
  averageIntensity: number;
}

export function computeStatistics(cloud: PointCloud): PointCloudStatistics {
  if (cloud.pointCount === 0) {
    throw new Error('Cannot compute statistics for empty point cloud');
  }

  let sumX = 0, sumY = 0, sumZ = 0, sumIntensity = 0;
  let minX = Infinity, maxX = -Infinity;
  let minY = Infinity, maxY = -Infinity;
  let minZ = Infinity, maxZ = -Infinity;

  cloud.points.forEach((point) => {
    sumX += point.x;
    sumY += point.y;
    sumZ += point.z;
    sumIntensity += point.intensity;

    minX = Math.min(minX, point.x);
    maxX = Math.max(maxX, point.x);
    minY = Math.min(minY, point.y);
    maxY = Math.max(maxY, point.y);
    minZ = Math.min(minZ, point.z);
    maxZ = Math.max(maxZ, point.z);
  });

  const n = cloud.pointCount;

  return {
    pointCount: n,
    bounds: { minX, maxX, minY, maxY, minZ, maxZ },
    centroid: {
      x: sumX / n,
      y: sumY / n,
      z: sumZ / n,
      intensity: sumIntensity / n,
    },
    averageIntensity: sumIntensity / n,
  };
}

/**
 * Save point cloud to PCD format (ASCII)
 */
export function exportToPCD(cloud: PointCloud): string {
  let pcd = '# .PCD v0.7 - Point Cloud Data file format\n';
  pcd += 'VERSION 0.7\n';
  pcd += 'FIELDS x y z intensity\n';
  pcd += 'SIZE 4 4 4 4\n';
  pcd += 'TYPE F F F F\n';
  pcd += 'COUNT 1 1 1 1\n';
  pcd += `WIDTH ${cloud.pointCount}\n`;
  pcd += 'HEIGHT 1\n';
  pcd += 'VIEWPOINT 0 0 0 1 0 0 0\n';
  pcd += `POINTS ${cloud.pointCount}\n`;
  pcd += 'DATA ascii\n';

  cloud.points.forEach((point) => {
    pcd += `${point.x} ${point.y} ${point.z} ${point.intensity}\n`;
  });

  return pcd;
}
