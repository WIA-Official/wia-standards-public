//! Quorum Sensing Protocol Implementation
//!
//! Implements bacterial-inspired quorum sensing for nanoscale swarm coordination.
//! When the local concentration of signaling molecules exceeds a threshold,
//! collective behavior is triggered.

use crate::error::{NanoError, NanoResult};
use crate::types::Position3D;
use crate::transport::{DiffusionConfig, DiffusionTransport};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

/// Signaling molecule types for quorum sensing
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SignalMolecule {
    /// Acyl-homoserine lactone (Gram-negative bacteria)
    AHL,
    /// Autoinducer peptide (Gram-positive bacteria)
    AIP,
    /// Autoinducer-2 (universal)
    AI2,
    /// Custom signaling molecule
    Custom,
}

impl SignalMolecule {
    /// Get diffusion coefficient for the molecule in water (m²/s)
    pub fn diffusion_coefficient(&self) -> f64 {
        match self {
            Self::AHL => 4.9e-10,  // ~300 Da
            Self::AIP => 1e-10,    // ~1000 Da (peptide)
            Self::AI2 => 6e-10,    // ~200 Da
            Self::Custom => 5e-10, // Default
        }
    }

    /// Get molecular weight in Daltons
    pub fn molecular_weight(&self) -> f64 {
        match self {
            Self::AHL => 300.0,
            Self::AIP => 1000.0,
            Self::AI2 => 200.0,
            Self::Custom => 500.0,
        }
    }
}

/// Collective behavior types triggered by quorum sensing
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CollectiveBehavior {
    /// Form a biofilm-like structure
    BiofilmFormation,
    /// Attack target collectively
    TargetAttack,
    /// Release cargo simultaneously
    CargoRelease,
    /// Emit bioluminescence
    Bioluminescence,
    /// Produce enzymes
    EnzymeProduction,
    /// Disperse from formation
    Disperse,
    /// Custom behavior
    Custom(String),
}

/// Configuration for quorum sensing
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuorumSensingConfig {
    /// Signaling molecule type
    pub molecule: SignalMolecule,
    /// Signal production rate (molecules/second)
    pub production_rate: f64,
    /// Threshold concentration for activation (nanomolar)
    pub threshold_nm: f64,
    /// Maximum sensing range (nanometers)
    pub sensing_range_nm: f64,
    /// Signal degradation rate (per second)
    pub degradation_rate: f64,
    /// Collective behavior to trigger
    pub behavior: CollectiveBehavior,
    /// Minimum time above threshold before activation (seconds)
    pub activation_delay_s: f64,
}

impl Default for QuorumSensingConfig {
    fn default() -> Self {
        Self {
            molecule: SignalMolecule::AHL,
            production_rate: 1000.0,     // 1000 molecules/s
            threshold_nm: 100.0,          // 100 nM
            sensing_range_nm: 5000.0,     // 5 μm
            degradation_rate: 0.01,       // 1% per second
            behavior: CollectiveBehavior::TargetAttack,
            activation_delay_s: 1.0,
        }
    }
}

/// State of a quorum sensing node
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuorumNode {
    /// Unique node identifier
    pub id: String,
    /// Node position
    pub position: Position3D,
    /// Current local signal concentration (nanomolar)
    pub local_concentration_nm: f64,
    /// Whether the node is activated
    pub activated: bool,
    /// Time spent above threshold
    pub time_above_threshold_s: f64,
    /// Signal production enabled
    pub producing: bool,
}

impl QuorumNode {
    /// Create a new quorum sensing node
    pub fn new(id: impl Into<String>, position: Position3D) -> Self {
        Self {
            id: id.into(),
            position,
            local_concentration_nm: 0.0,
            activated: false,
            time_above_threshold_s: 0.0,
            producing: true,
        }
    }

    /// Check if this node should be activated
    pub fn should_activate(&self, threshold: f64, delay: f64) -> bool {
        self.local_concentration_nm >= threshold && self.time_above_threshold_s >= delay
    }
}

/// Quorum sensing network coordinator
pub struct QuorumSensingNetwork {
    config: QuorumSensingConfig,
    nodes: Arc<RwLock<HashMap<String, QuorumNode>>>,
    transport: DiffusionTransport,
    current_time_s: f64,
}

impl QuorumSensingNetwork {
    /// Create a new quorum sensing network
    pub fn new(config: QuorumSensingConfig) -> Self {
        let diffusion_config = DiffusionConfig {
            carrier_molecule: format!("{:?}", config.molecule),
            diffusion_coefficient_m2_per_s: config.molecule.diffusion_coefficient(),
            medium_viscosity_pa_s: 0.001,
            temperature_k: 310.0,
            detection_threshold: 0.001,
            max_range_nm: config.sensing_range_nm,
        };

        Self {
            config,
            nodes: Arc::new(RwLock::new(HashMap::new())),
            transport: DiffusionTransport::new(diffusion_config),
            current_time_s: 0.0,
        }
    }

    /// Add a node to the network
    pub async fn add_node(&self, node: QuorumNode) {
        let mut nodes = self.nodes.write().await;
        nodes.insert(node.id.clone(), node);
    }

    /// Remove a node from the network
    pub async fn remove_node(&self, id: &str) -> Option<QuorumNode> {
        let mut nodes = self.nodes.write().await;
        nodes.remove(id)
    }

    /// Get node count
    pub async fn node_count(&self) -> usize {
        self.nodes.read().await.len()
    }

    /// Get all nodes
    pub async fn get_nodes(&self) -> Vec<QuorumNode> {
        self.nodes.read().await.values().cloned().collect()
    }

    /// Get a specific node
    pub async fn get_node(&self, id: &str) -> Option<QuorumNode> {
        self.nodes.read().await.get(id).cloned()
    }

    /// Update the network state
    pub async fn update(&mut self, delta_time_s: f64) -> NanoResult<QuorumUpdateResult> {
        self.current_time_s += delta_time_s;
        let mut nodes = self.nodes.write().await;

        // Calculate concentration contributions from all producing nodes
        let producing_nodes: Vec<_> = nodes
            .values()
            .filter(|n| n.producing)
            .map(|n| (n.id.clone(), n.position.clone()))
            .collect();

        let mut newly_activated = Vec::new();

        // Update each node's local concentration
        for node in nodes.values_mut() {
            let mut total_concentration = 0.0;

            // Sum contributions from all other producing nodes
            for (other_id, other_pos) in &producing_nodes {
                if other_id == &node.id {
                    // Self contribution (local production)
                    total_concentration += self.config.production_rate * delta_time_s * 0.001;
                } else {
                    // Contribution from other nodes based on distance
                    let distance = node.position.distance_to(other_pos);
                    if distance <= self.config.sensing_range_nm {
                        let signal_strength = self.calculate_signal_contribution(distance, delta_time_s);
                        total_concentration += signal_strength;
                    }
                }
            }

            // Apply degradation
            node.local_concentration_nm *= 1.0 - self.config.degradation_rate * delta_time_s;

            // Add new signal
            node.local_concentration_nm += total_concentration;

            // Check threshold
            if node.local_concentration_nm >= self.config.threshold_nm {
                node.time_above_threshold_s += delta_time_s;

                // Check for activation
                if !node.activated && node.should_activate(
                    self.config.threshold_nm,
                    self.config.activation_delay_s,
                ) {
                    node.activated = true;
                    newly_activated.push(node.id.clone());
                }
            } else {
                // Reset timer if below threshold
                node.time_above_threshold_s = 0.0;
            }
        }

        // Check if quorum is reached
        let total_nodes = nodes.len();
        let activated_nodes = nodes.values().filter(|n| n.activated).count();
        let quorum_reached = activated_nodes as f64 / total_nodes as f64 >= 0.5;

        Ok(QuorumUpdateResult {
            total_nodes,
            activated_nodes,
            newly_activated,
            quorum_reached,
            current_time_s: self.current_time_s,
        })
    }

    /// Calculate signal contribution from a source at given distance
    fn calculate_signal_contribution(&self, distance_nm: f64, time_s: f64) -> f64 {
        // Simplified concentration model based on diffusion
        let d = self.config.molecule.diffusion_coefficient();
        let r = distance_nm * 1e-9; // Convert to meters

        if time_s <= 0.0 || r <= 0.0 {
            return 0.0;
        }

        // Steady-state concentration from point source: C = Q / (4πDr)
        let q = self.config.production_rate; // molecules/s
        let concentration = q / (4.0 * std::f64::consts::PI * d * r);

        // Convert to nanomolar (rough approximation)
        concentration * 1e-6
    }

    /// Check if quorum has been reached
    pub async fn is_quorum_reached(&self) -> bool {
        let nodes = self.nodes.read().await;
        let total = nodes.len();
        if total == 0 {
            return false;
        }

        let activated = nodes.values().filter(|n| n.activated).count();
        activated as f64 / total as f64 >= 0.5
    }

    /// Get the current network status
    pub async fn get_status(&self) -> QuorumNetworkStatus {
        let nodes = self.nodes.read().await;
        let total_nodes = nodes.len();
        let activated_nodes = nodes.values().filter(|n| n.activated).count();
        let producing_nodes = nodes.values().filter(|n| n.producing).count();

        let avg_concentration = if total_nodes > 0 {
            nodes.values().map(|n| n.local_concentration_nm).sum::<f64>() / total_nodes as f64
        } else {
            0.0
        };

        let max_concentration = nodes
            .values()
            .map(|n| n.local_concentration_nm)
            .fold(0.0, f64::max);

        QuorumNetworkStatus {
            total_nodes,
            activated_nodes,
            producing_nodes,
            avg_concentration_nm: avg_concentration,
            max_concentration_nm: max_concentration,
            threshold_nm: self.config.threshold_nm,
            quorum_reached: activated_nodes as f64 / total_nodes.max(1) as f64 >= 0.5,
            behavior: self.config.behavior.clone(),
        }
    }

    /// Trigger collective behavior if quorum is reached
    pub async fn trigger_behavior(&self) -> NanoResult<Option<CollectiveBehavior>> {
        if self.is_quorum_reached().await {
            Ok(Some(self.config.behavior.clone()))
        } else {
            Ok(None)
        }
    }

    /// Reset all nodes to initial state
    pub async fn reset(&mut self) {
        let mut nodes = self.nodes.write().await;
        for node in nodes.values_mut() {
            node.local_concentration_nm = 0.0;
            node.activated = false;
            node.time_above_threshold_s = 0.0;
        }
        self.current_time_s = 0.0;
    }
}

/// Result of a quorum sensing network update
#[derive(Debug, Clone)]
pub struct QuorumUpdateResult {
    /// Total number of nodes
    pub total_nodes: usize,
    /// Number of activated nodes
    pub activated_nodes: usize,
    /// IDs of newly activated nodes
    pub newly_activated: Vec<String>,
    /// Whether quorum has been reached (>50% activated)
    pub quorum_reached: bool,
    /// Current simulation time
    pub current_time_s: f64,
}

/// Status of the quorum sensing network
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuorumNetworkStatus {
    /// Total nodes in network
    pub total_nodes: usize,
    /// Activated nodes
    pub activated_nodes: usize,
    /// Nodes producing signal
    pub producing_nodes: usize,
    /// Average signal concentration
    pub avg_concentration_nm: f64,
    /// Maximum signal concentration
    pub max_concentration_nm: f64,
    /// Activation threshold
    pub threshold_nm: f64,
    /// Whether quorum is reached
    pub quorum_reached: bool,
    /// Behavior to trigger
    pub behavior: CollectiveBehavior,
}

/// Builder for quorum sensing simulation scenarios
pub struct QuorumSensingBuilder {
    config: QuorumSensingConfig,
    initial_nodes: Vec<QuorumNode>,
}

impl QuorumSensingBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            config: QuorumSensingConfig::default(),
            initial_nodes: Vec::new(),
        }
    }

    /// Set the signaling molecule
    pub fn molecule(mut self, molecule: SignalMolecule) -> Self {
        self.config.molecule = molecule;
        self
    }

    /// Set the activation threshold
    pub fn threshold(mut self, threshold_nm: f64) -> Self {
        self.config.threshold_nm = threshold_nm;
        self
    }

    /// Set the production rate
    pub fn production_rate(mut self, rate: f64) -> Self {
        self.config.production_rate = rate;
        self
    }

    /// Set the collective behavior
    pub fn behavior(mut self, behavior: CollectiveBehavior) -> Self {
        self.config.behavior = behavior;
        self
    }

    /// Add nodes in a grid pattern
    pub fn grid_nodes(mut self, count_x: usize, count_y: usize, spacing_nm: f64) -> Self {
        for i in 0..count_x {
            for j in 0..count_y {
                let id = format!("node_{}_{}", i, j);
                let pos = Position3D::new(i as f64 * spacing_nm, j as f64 * spacing_nm, 0.0);
                self.initial_nodes.push(QuorumNode::new(id, pos));
            }
        }
        self
    }

    /// Add nodes in a random cluster
    pub fn cluster_nodes(mut self, count: usize, center: Position3D, radius_nm: f64) -> Self {
        use std::f64::consts::PI;

        for i in 0..count {
            let angle = 2.0 * PI * (i as f64) / (count as f64);
            let r = radius_nm * (i as f64 / count as f64).sqrt(); // Fill disk evenly
            let pos = Position3D::new(
                center.x + r * angle.cos(),
                center.y + r * angle.sin(),
                center.z,
            );
            self.initial_nodes.push(QuorumNode::new(format!("node_{}", i), pos));
        }
        self
    }

    /// Build the quorum sensing network
    pub async fn build(self) -> QuorumSensingNetwork {
        let network = QuorumSensingNetwork::new(self.config);

        for node in self.initial_nodes {
            network.add_node(node).await;
        }

        network
    }
}

impl Default for QuorumSensingBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_quorum_sensing_basic() {
        let mut network = QuorumSensingBuilder::new()
            .threshold(10.0)
            .production_rate(1000.0)
            .cluster_nodes(10, Position3D::new(0.0, 0.0, 0.0), 100.0)
            .build()
            .await;

        assert_eq!(network.node_count().await, 10);
        assert!(!network.is_quorum_reached().await);

        // Run simulation for a while
        for _ in 0..100 {
            let result = network.update(0.1).await.unwrap();
            if result.quorum_reached {
                break;
            }
        }

        let status = network.get_status().await;
        assert!(status.max_concentration_nm > 0.0);
    }

    #[tokio::test]
    async fn test_node_activation() {
        let config = QuorumSensingConfig {
            threshold_nm: 1.0,
            activation_delay_s: 0.0, // Instant activation
            ..Default::default()
        };

        let mut network = QuorumSensingNetwork::new(config);

        // Add closely spaced nodes
        for i in 0..5 {
            network.add_node(QuorumNode::new(
                format!("node_{}", i),
                Position3D::new(i as f64 * 10.0, 0.0, 0.0),
            )).await;
        }

        // Update multiple times
        for _ in 0..50 {
            network.update(0.1).await.unwrap();
        }

        let status = network.get_status().await;
        assert!(status.activated_nodes > 0);
    }

    #[test]
    fn test_signal_molecule_properties() {
        assert!(SignalMolecule::AHL.diffusion_coefficient() > 0.0);
        assert!(SignalMolecule::AHL.molecular_weight() > 0.0);
    }

    #[test]
    fn test_quorum_node_activation() {
        let mut node = QuorumNode::new("test", Position3D::new(0.0, 0.0, 0.0));

        // Below threshold
        node.local_concentration_nm = 50.0;
        node.time_above_threshold_s = 2.0;
        assert!(!node.should_activate(100.0, 1.0));

        // Above threshold but not enough time
        node.local_concentration_nm = 150.0;
        node.time_above_threshold_s = 0.5;
        assert!(!node.should_activate(100.0, 1.0));

        // Above threshold with enough time
        node.time_above_threshold_s = 1.5;
        assert!(node.should_activate(100.0, 1.0));
    }
}
