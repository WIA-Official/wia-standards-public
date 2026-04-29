//! Message routing for nanoscale networks

use crate::error::{NanoError, NanoResult};
use crate::types::Position3D;
use super::{ProtocolMessage, ChannelConfig, QoS};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

/// Message router for nanoscale communication
pub struct MessageRouter {
    id: String,
    routes: Arc<RwLock<HashMap<String, RouteEntry>>>,
    neighbors: Arc<RwLock<HashMap<String, NeighborInfo>>>,
    routing_table: Arc<RwLock<Vec<RoutingEntry>>>,
    config: RouterConfig,
    stats: Arc<RwLock<RouterStats>>,
}

impl MessageRouter {
    pub fn new(id: impl Into<String>, config: RouterConfig) -> Self {
        Self {
            id: id.into(),
            routes: Arc::new(RwLock::new(HashMap::new())),
            neighbors: Arc::new(RwLock::new(HashMap::new())),
            routing_table: Arc::new(RwLock::new(Vec::new())),
            config,
            stats: Arc::new(RwLock::new(RouterStats::default())),
        }
    }

    pub fn id(&self) -> &str {
        &self.id
    }

    /// Add a direct route to a destination
    pub async fn add_route(&self, dest_id: String, route: RouteEntry) {
        let mut routes = self.routes.write().await;
        routes.insert(dest_id, route);
    }

    /// Remove a route
    pub async fn remove_route(&self, dest_id: &str) {
        let mut routes = self.routes.write().await;
        routes.remove(dest_id);
    }

    /// Register a neighbor
    pub async fn add_neighbor(&self, neighbor: NeighborInfo) {
        let mut neighbors = self.neighbors.write().await;
        neighbors.insert(neighbor.id.clone(), neighbor);
    }

    /// Remove a neighbor
    pub async fn remove_neighbor(&self, neighbor_id: &str) {
        let mut neighbors = self.neighbors.write().await;
        neighbors.remove(neighbor_id);
    }

    /// Route a message to its destination
    pub async fn route(&self, message: ProtocolMessage) -> NanoResult<RoutingDecision> {
        let mut stats = self.stats.write().await;
        stats.messages_routed += 1;

        // Check TTL
        if message.header.ttl == 0 {
            stats.messages_dropped += 1;
            return Err(NanoError::Communication("TTL expired".into()));
        }

        // Determine destination
        let dest_id = match &message.header.dest_id {
            Some(id) => id.clone(),
            None => {
                // Broadcast message
                if message.header.flags.broadcast {
                    return Ok(RoutingDecision::Broadcast);
                }
                return Err(NanoError::Communication("No destination specified".into()));
            }
        };

        // Check if destination is us
        if dest_id == self.id {
            return Ok(RoutingDecision::Deliver);
        }

        // Look up direct route
        let routes = self.routes.read().await;
        if let Some(route) = routes.get(&dest_id) {
            return Ok(RoutingDecision::Forward {
                next_hop: route.next_hop.clone(),
                metric: route.metric,
            });
        }

        // Look for neighbor
        let neighbors = self.neighbors.read().await;
        if neighbors.contains_key(&dest_id) {
            return Ok(RoutingDecision::Forward {
                next_hop: dest_id,
                metric: 1,
            });
        }

        // Use routing table
        let table = self.routing_table.read().await;
        for entry in table.iter() {
            if entry.destination == dest_id {
                return Ok(RoutingDecision::Forward {
                    next_hop: entry.next_hop.clone(),
                    metric: entry.metric,
                });
            }
        }

        // No route found
        stats.messages_dropped += 1;
        Err(NanoError::NotFound(format!("No route to {}", dest_id)))
    }

    /// Get all neighbors
    pub async fn get_neighbors(&self) -> Vec<NeighborInfo> {
        let neighbors = self.neighbors.read().await;
        neighbors.values().cloned().collect()
    }

    /// Update routing table (distance-vector style)
    pub async fn update_routing_table(&self, updates: Vec<RoutingUpdate>) {
        let mut table = self.routing_table.write().await;

        for update in updates {
            let new_metric = update.metric + 1; // Add hop cost

            // Find existing entry
            if let Some(entry) = table.iter_mut().find(|e| e.destination == update.destination) {
                // Update if better route
                if new_metric < entry.metric {
                    entry.next_hop = update.via.clone();
                    entry.metric = new_metric;
                    entry.last_updated = chrono::Utc::now().timestamp() as u64;
                }
            } else {
                // Add new entry
                table.push(RoutingEntry {
                    destination: update.destination,
                    next_hop: update.via,
                    metric: new_metric,
                    last_updated: chrono::Utc::now().timestamp() as u64,
                });
            }
        }

        // Remove stale entries (older than 60 seconds)
        let now = chrono::Utc::now().timestamp() as u64;
        table.retain(|e| now - e.last_updated < 60);
    }

    /// Get router statistics
    pub async fn stats(&self) -> RouterStats {
        self.stats.read().await.clone()
    }
}

/// Router configuration
#[derive(Debug, Clone)]
pub struct RouterConfig {
    pub max_routes: usize,
    pub max_neighbors: usize,
    pub route_timeout_s: u64,
    pub default_qos: QoS,
}

impl Default for RouterConfig {
    fn default() -> Self {
        Self {
            max_routes: 1000,
            max_neighbors: 100,
            route_timeout_s: 60,
            default_qos: QoS::BestEffort,
        }
    }
}

/// Route entry
#[derive(Debug, Clone)]
pub struct RouteEntry {
    pub next_hop: String,
    pub metric: u32,
    pub qos: QoS,
    pub expires_at: Option<u64>,
}

/// Neighbor information
#[derive(Debug, Clone)]
pub struct NeighborInfo {
    pub id: String,
    pub position: Option<Position3D>,
    pub distance_nm: Option<f64>,
    pub link_quality: f64,
    pub last_seen: u64,
    pub capabilities: Vec<String>,
}

/// Routing table entry
#[derive(Debug, Clone)]
pub struct RoutingEntry {
    pub destination: String,
    pub next_hop: String,
    pub metric: u32,
    pub last_updated: u64,
}

/// Routing update from neighbor
#[derive(Debug, Clone)]
pub struct RoutingUpdate {
    pub destination: String,
    pub via: String,
    pub metric: u32,
}

/// Routing decision
#[derive(Debug, Clone)]
pub enum RoutingDecision {
    /// Deliver locally
    Deliver,
    /// Forward to next hop
    Forward { next_hop: String, metric: u32 },
    /// Broadcast to all neighbors
    Broadcast,
    /// Drop the message
    Drop { reason: String },
}

/// Router statistics
#[derive(Debug, Clone, Default)]
pub struct RouterStats {
    pub messages_routed: u64,
    pub messages_delivered: u64,
    pub messages_forwarded: u64,
    pub messages_dropped: u64,
    pub active_routes: usize,
    pub active_neighbors: usize,
}

/// Geographic routing support
pub struct GeoRouter {
    inner: MessageRouter,
    position: Position3D,
    communication_range: f64,
}

impl GeoRouter {
    pub fn new(id: impl Into<String>, position: Position3D, range_nm: f64) -> Self {
        Self {
            inner: MessageRouter::new(id, RouterConfig::default()),
            position,
            communication_range: range_nm,
        }
    }

    pub fn position(&self) -> Position3D {
        self.position
    }

    pub fn set_position(&mut self, pos: Position3D) {
        self.position = pos;
    }

    /// Find best neighbor to forward to based on geography
    pub async fn geo_route(&self, target_position: Position3D) -> Option<String> {
        let neighbors = self.inner.get_neighbors().await;

        let mut best_neighbor: Option<&NeighborInfo> = None;
        let mut best_distance = f64::MAX;

        for neighbor in &neighbors {
            if let Some(pos) = neighbor.position {
                let dist = pos.distance_to(&target_position);
                if dist < best_distance && dist < self.communication_range {
                    best_distance = dist;
                    best_neighbor = Some(neighbor);
                }
            }
        }

        best_neighbor.map(|n| n.id.clone())
    }

    /// Check if a position is in communication range
    pub fn in_range(&self, position: Position3D) -> bool {
        self.position.distance_to(&position) <= self.communication_range
    }
}
