/**
 * Point Cloud Processing Utilities
 */

import { PointCloud, Point3D, DetectedObject, BoundingBox3D } from './types';

/**
 * Point Cloud Processor
 * 
 * Provides utilities for processing, filtering, and analyzing point clouds.
 */
export class PointCloudProcessor {
  /**
   * Filter points by distance range
   */
  static filterByRange(
    cloud: PointCloud,
    minRange: number,
    maxRange: number
  ): PointCloud {
    const filteredPoints = cloud.points.filter((point) => {
      const range = Math.sqrt(point.x ** 2 + point.y ** 2 + point.z ** 2);
      return range >= minRange && range <= maxRange;
    });

    return {
      ...cloud,
      points: filteredPoints,
      pointCount: filteredPoints.length,
    };
  }

  /**
   * Filter points by bounding box
   */
  static filterByBoundingBox(
    cloud: PointCloud,
    bbox: BoundingBox3D
  ): PointCloud {
    const { center, dimensions } = bbox;
    const halfWidth = dimensions.width / 2;
    const halfHeight = dimensions.height / 2;
    const halfDepth = dimensions.depth / 2;

    const filteredPoints = cloud.points.filter((point) => {
      const dx = Math.abs(point.x - center.x);
      const dy = Math.abs(point.y - center.y);
      const dz = Math.abs(point.z - center.z);
      return dx <= halfWidth && dy <= halfHeight && dz <= halfDepth;
    });

    return {
      ...cloud,
      points: filteredPoints,
      pointCount: filteredPoints.length,
    };
  }

  /**
   * Downsample point cloud using voxel grid
   */
  static voxelGridDownsample(
    cloud: PointCloud,
    voxelSize: number
  ): PointCloud {
    const voxelMap = new Map<string, Point3D[]>();

    // Group points into voxels
    cloud.points.forEach((point) => {
      const vx = Math.floor(point.x / voxelSize);
      const vy = Math.floor(point.y / voxelSize);
      const vz = Math.floor(point.z / voxelSize);
      const key = `${vx},${vy},${vz}`;

      if (!voxelMap.has(key)) {
        voxelMap.set(key, []);
      }
      voxelMap.get(key)!.push(point);
    });

    // Compute centroid of each voxel
    const downsampledPoints: Point3D[] = [];
    voxelMap.forEach((voxelPoints) => {
      const centroid = this.computeCentroid(voxelPoints);
      downsampledPoints.push(centroid);
    });

    return {
      ...cloud,
      points: downsampledPoints,
      pointCount: downsampledPoints.length,
    };
  }

  /**
   * Estimate ground plane using RANSAC
   */
  static estimateGroundPlane(
    cloud: PointCloud,
    iterations: number = 100,
    threshold: number = 0.1
  ): { normal: [number, number, number]; d: number } {
    // Simplified RANSAC for ground plane estimation
    // Plane equation: ax + by + cz + d = 0
    let bestInliers = 0;
    let bestPlane = { normal: [0, 1, 0] as [number, number, number], d: 0 };

    for (let i = 0; i < iterations; i++) {
      // Sample 3 random points
      const samples = this.randomSample(cloud.points, 3);
      if (samples.length < 3) continue;

      // Compute plane from 3 points
      const plane = this.planeFrom3Points(samples[0], samples[1], samples[2]);
      if (!plane) continue;

      // Count inliers
      let inliers = 0;
      cloud.points.forEach((point) => {
        const distance = this.pointToPlaneDistance(point, plane);
        if (distance < threshold) inliers++;
      });

      if (inliers > bestInliers) {
        bestInliers = inliers;
        bestPlane = plane;
      }
    }

    return bestPlane;
  }

  /**
   * Euclidean clustering
   */
  static euclideanClustering(
    cloud: PointCloud,
    clusterTolerance: number,
    minClusterSize: number,
    maxClusterSize: number
  ): PointCloud[] {
    // Simplified clustering - full implementation would use KD-tree
    const clusters: PointCloud[] = [];
    const visited = new Set<number>();

    cloud.points.forEach((point, index) => {
      if (visited.has(index)) return;

      const cluster: Point3D[] = [];
      const queue: number[] = [index];

      while (queue.length > 0) {
        const currentIndex = queue.shift()!;
        if (visited.has(currentIndex)) continue;

        visited.add(currentIndex);
        const currentPoint = cloud.points[currentIndex];
        cluster.push(currentPoint);

        // Find neighbors
        cloud.points.forEach((neighbor, neighborIndex) => {
          if (visited.has(neighborIndex)) return;
          const distance = this.euclideanDistance(currentPoint, neighbor);
          if (distance < clusterTolerance) {
            queue.push(neighborIndex);
          }
        });
      }

      if (cluster.length >= minClusterSize && cluster.length <= maxClusterSize) {
        clusters.push({
          ...cloud,
          points: cluster,
          pointCount: cluster.length,
        });
      }
    });

    return clusters;
  }

  // Helper methods

  private static computeCentroid(points: Point3D[]): Point3D {
    const sum = points.reduce(
      (acc, p) => ({
        x: acc.x + p.x,
        y: acc.y + p.y,
        z: acc.z + p.z,
        intensity: acc.intensity + p.intensity,
      }),
      { x: 0, y: 0, z: 0, intensity: 0 }
    );

    const n = points.length;
    return {
      x: sum.x / n,
      y: sum.y / n,
      z: sum.z / n,
      intensity: sum.intensity / n,
    };
  }

  private static randomSample<T>(array: T[], n: number): T[] {
    const shuffled = [...array].sort(() => 0.5 - Math.random());
    return shuffled.slice(0, n);
  }

  private static planeFrom3Points(
    p1: Point3D,
    p2: Point3D,
    p3: Point3D
  ): { normal: [number, number, number]; d: number } | null {
    // Compute two vectors in the plane
    const v1 = { x: p2.x - p1.x, y: p2.y - p1.y, z: p2.z - p1.z };
    const v2 = { x: p3.x - p1.x, y: p3.y - p1.y, z: p3.z - p1.z };

    // Cross product gives normal
    const normal: [number, number, number] = [
      v1.y * v2.z - v1.z * v2.y,
      v1.z * v2.x - v1.x * v2.z,
      v1.x * v2.y - v1.y * v2.x,
    ];

    const length = Math.sqrt(normal[0] ** 2 + normal[1] ** 2 + normal[2] ** 2);
    if (length < 1e-6) return null;

    // Normalize
    normal[0] /= length;
    normal[1] /= length;
    normal[2] /= length;

    // Compute d
    const d = -(normal[0] * p1.x + normal[1] * p1.y + normal[2] * p1.z);

    return { normal, d };
  }

  private static pointToPlaneDistance(
    point: Point3D,
    plane: { normal: [number, number, number]; d: number }
  ): number {
    const { normal, d } = plane;
    return Math.abs(normal[0] * point.x + normal[1] * point.y + normal[2] * point.z + d);
  }

  private static euclideanDistance(p1: Point3D, p2: Point3D): number {
    return Math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2);
  }
}
