//! Simulation world definition

use crate::types::{Environment, Position3D};
use std::collections::HashMap;

/// Simulation world bounds
#[derive(Debug, Clone)]
pub struct WorldBounds {
    pub min: Position3D,
    pub max: Position3D,
}

impl WorldBounds {
    pub fn new(min: Position3D, max: Position3D) -> Self {
        Self { min, max }
    }

    /// Create a cubic world centered at origin
    pub fn cubic(half_size: f64) -> Self {
        Self {
            min: Position3D::new(-half_size, -half_size, -half_size),
            max: Position3D::new(half_size, half_size, half_size),
        }
    }

    /// Check if position is within bounds
    pub fn contains(&self, pos: &Position3D) -> bool {
        pos.x >= self.min.x && pos.x <= self.max.x &&
        pos.y >= self.min.y && pos.y <= self.max.y &&
        pos.z >= self.min.z && pos.z <= self.max.z
    }

    /// Get world volume in nm³
    pub fn volume(&self) -> f64 {
        (self.max.x - self.min.x) *
        (self.max.y - self.min.y) *
        (self.max.z - self.min.z)
    }

    /// Get center of world
    pub fn center(&self) -> Position3D {
        Position3D::new(
            (self.min.x + self.max.x) / 2.0,
            (self.min.y + self.max.y) / 2.0,
            (self.min.z + self.max.z) / 2.0,
        )
    }
}

impl Default for WorldBounds {
    fn default() -> Self {
        Self::cubic(10_000.0) // 20 μm cube
    }
}

/// Simulation world configuration
#[derive(Debug, Clone)]
pub struct SimulationWorld {
    pub bounds: WorldBounds,
    pub environment: Environment,
    pub time_step_ns: f64,
    pub obstacles: Vec<Obstacle>,
    pub regions: HashMap<String, Region>,
}

impl SimulationWorld {
    pub fn new() -> Self {
        Self {
            bounds: WorldBounds::default(),
            environment: Environment::physiological(),
            time_step_ns: 1000.0, // 1 μs default timestep
            obstacles: Vec::new(),
            regions: HashMap::new(),
        }
    }

    /// Create a bloodstream simulation environment
    pub fn bloodstream() -> Self {
        let mut world = Self::new();
        world.environment = Environment::physiological();
        world.bounds = WorldBounds::cubic(50_000.0); // 100 μm cube

        // Add some obstacles (simplified red blood cells)
        for i in 0..10 {
            world.obstacles.push(Obstacle::sphere(
                Position3D::new(
                    (i as f64 - 5.0) * 8000.0,
                    0.0,
                    0.0,
                ),
                3500.0, // RBC diameter ~7μm
            ));
        }

        world
    }

    /// Create an intracellular simulation environment
    pub fn intracellular() -> Self {
        let mut world = Self::new();
        world.environment = Environment::intracellular();
        world.bounds = WorldBounds::cubic(5_000.0); // 10 μm cube (cell interior)

        // Define nucleus region
        world.regions.insert(
            "nucleus".to_string(),
            Region::sphere(Position3D::origin(), 2500.0),
        );

        world
    }

    /// Add an obstacle
    pub fn add_obstacle(&mut self, obstacle: Obstacle) {
        self.obstacles.push(obstacle);
    }

    /// Add a region
    pub fn add_region(&mut self, name: impl Into<String>, region: Region) {
        self.regions.insert(name.into(), region);
    }

    /// Check if position collides with any obstacle
    pub fn check_collision(&self, pos: &Position3D, radius: f64) -> Option<&Obstacle> {
        self.obstacles.iter().find(|o| o.intersects(pos, radius))
    }

    /// Get region at position
    pub fn get_region_at(&self, pos: &Position3D) -> Option<&str> {
        for (name, region) in &self.regions {
            if region.contains(pos) {
                return Some(name);
            }
        }
        None
    }
}

impl Default for SimulationWorld {
    fn default() -> Self {
        Self::new()
    }
}

/// Obstacle in simulation world
#[derive(Debug, Clone)]
pub struct Obstacle {
    pub shape: ObstacleShape,
    pub position: Position3D,
    pub properties: ObstacleProperties,
}

impl Obstacle {
    /// Create a spherical obstacle
    pub fn sphere(center: Position3D, radius: f64) -> Self {
        Self {
            shape: ObstacleShape::Sphere { radius },
            position: center,
            properties: ObstacleProperties::default(),
        }
    }

    /// Create a box obstacle
    pub fn box_shape(center: Position3D, half_extents: Position3D) -> Self {
        Self {
            shape: ObstacleShape::Box { half_extents },
            position: center,
            properties: ObstacleProperties::default(),
        }
    }

    /// Check if this obstacle intersects with a sphere at given position
    pub fn intersects(&self, pos: &Position3D, radius: f64) -> bool {
        match &self.shape {
            ObstacleShape::Sphere { radius: obs_radius } => {
                let dist = self.position.distance_to(pos);
                dist < (radius + obs_radius)
            }
            ObstacleShape::Box { half_extents } => {
                // Simple AABB check
                let dx = (pos.x - self.position.x).abs();
                let dy = (pos.y - self.position.y).abs();
                let dz = (pos.z - self.position.z).abs();

                dx < (half_extents.x + radius) &&
                dy < (half_extents.y + radius) &&
                dz < (half_extents.z + radius)
            }
            ObstacleShape::Cylinder { radius: cyl_radius, height } => {
                let dx = pos.x - self.position.x;
                let dy = pos.y - self.position.y;
                let dz = (pos.z - self.position.z).abs();

                let horizontal_dist = (dx * dx + dy * dy).sqrt();
                horizontal_dist < (cyl_radius + radius) && dz < (height / 2.0 + radius)
            }
        }
    }
}

/// Shape of an obstacle
#[derive(Debug, Clone)]
pub enum ObstacleShape {
    Sphere { radius: f64 },
    Box { half_extents: Position3D },
    Cylinder { radius: f64, height: f64 },
}

/// Properties of an obstacle
#[derive(Debug, Clone)]
pub struct ObstacleProperties {
    pub permeable: bool,
    pub permeability: f64, // 0.0 = solid, 1.0 = fully permeable
    pub label: Option<String>,
}

impl Default for ObstacleProperties {
    fn default() -> Self {
        Self {
            permeable: false,
            permeability: 0.0,
            label: None,
        }
    }
}

/// Named region in simulation world
#[derive(Debug, Clone)]
pub struct Region {
    pub shape: RegionShape,
    pub position: Position3D,
    pub local_environment: Option<Environment>,
}

impl Region {
    /// Create a spherical region
    pub fn sphere(center: Position3D, radius: f64) -> Self {
        Self {
            shape: RegionShape::Sphere { radius },
            position: center,
            local_environment: None,
        }
    }

    /// Create a box region
    pub fn box_shape(center: Position3D, half_extents: Position3D) -> Self {
        Self {
            shape: RegionShape::Box { half_extents },
            position: center,
            local_environment: None,
        }
    }

    /// Set local environment for this region
    pub fn with_environment(mut self, env: Environment) -> Self {
        self.local_environment = Some(env);
        self
    }

    /// Check if position is within this region
    pub fn contains(&self, pos: &Position3D) -> bool {
        match &self.shape {
            RegionShape::Sphere { radius } => {
                self.position.distance_to(pos) <= *radius
            }
            RegionShape::Box { half_extents } => {
                let dx = (pos.x - self.position.x).abs();
                let dy = (pos.y - self.position.y).abs();
                let dz = (pos.z - self.position.z).abs();

                dx <= half_extents.x &&
                dy <= half_extents.y &&
                dz <= half_extents.z
            }
        }
    }
}

/// Shape of a region
#[derive(Debug, Clone)]
pub enum RegionShape {
    Sphere { radius: f64 },
    Box { half_extents: Position3D },
}
