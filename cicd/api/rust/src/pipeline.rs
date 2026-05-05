//! Pipeline model. Mirrors `schemas/pipeline.schema.json`.

use std::collections::{BTreeMap, BTreeSet};

/// Five canonical DAG node identifiers.
pub const PIPELINE_NODES: [&str; 5] = ["SOURCE", "BUILD", "TEST_GATE", "SECURITY_GATE", "CD_HANDOFF"];

/// Recognised pipeline triggers.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Trigger { /// push to a tracked branch
    Push,
    /// pull request open / sync
    Pr,
    /// tag publish
    Tag,
    /// manual dispatch
    Manual,
    /// schedule / cron
    Schedule
}

impl Trigger {
    /// Stringify into the spec ENUM value.
    pub fn as_str(&self) -> &'static str {
        match self {
            Trigger::Push => "PUSH",
            Trigger::Pr => "PR",
            Trigger::Tag => "TAG",
            Trigger::Manual => "MANUAL",
            Trigger::Schedule => "SCHEDULE",
        }
    }
}

/// Recognised security gates.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum SecurityGate { /// static application security testing
    Sast,
    /// software composition analysis
    Sca,
    /// dynamic application security testing
    Dast,
    /// secrets scan
    Secrets,
    /// container image scan
    Container,
    /// infrastructure as code scan
    Iac
}

impl SecurityGate {
    /// Stringify.
    pub fn as_str(&self) -> &'static str {
        match self {
            SecurityGate::Sast => "SAST",
            SecurityGate::Sca => "SCA",
            SecurityGate::Dast => "DAST",
            SecurityGate::Secrets => "SECRETS",
            SecurityGate::Container => "CONTAINER",
            SecurityGate::Iac => "IAC",
        }
    }
}

/// Policy mode.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PolicyMode { /// observe only
    Audit,
    /// block on policy fail
    Enforce
}

/// Deployment strategy.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DeployStrategy { /// rolling update
    Rolling,
    /// blue-green
    BlueGreen,
    /// canary
    Canary,
    /// progressive (canary + analysis)
    Progressive
}

/// Functional class of a pipeline node.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NodeType { /// SCM source
    Source,
    /// build / package
    Build,
    /// test gate
    Test,
    /// security gate
    Security,
    /// continuous delivery handoff
    Deliver
}

/// A single pipeline node.
#[derive(Debug, Clone)]
pub struct Node {
    /// Canonical identifier (member of `PIPELINE_NODES`).
    pub id:        String,
    /// Functional class.
    pub kind:      NodeType,
    /// IDs of upstream nodes that must complete before this one.
    pub needs:     Vec<String>,
    /// Security gates evaluated by this node (Security only).
    pub gates:     Vec<SecurityGate>,
    /// Policy mode (Security only).
    pub policy:    Option<PolicyMode>,
    /// Deploy strategy (Deliver only).
    pub strategy:  Option<DeployStrategy>,
    /// Traffic curve percentages (Deliver only).
    pub traffic_curve: Vec<u8>,
}

impl Node {
    /// Construct a node with no dependencies.
    pub fn new(id: &str, kind: NodeType) -> Self {
        Self { id: id.to_string(), kind, needs: vec![], gates: vec![], policy: None, strategy: None, traffic_curve: vec![] }
    }
    /// Builder: set dependencies.
    pub fn needs(mut self, deps: &[&str]) -> Self {
        self.needs = deps.iter().map(|d| d.to_string()).collect(); self
    }
}

/// A pipeline.
#[derive(Debug, Clone)]
pub struct Pipeline {
    /// Pipeline name.
    pub name:    String,
    /// Trigger.
    pub trigger: Trigger,
    /// Nodes.
    pub nodes:   Vec<Node>,
}

/// Validation error.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ValidationError {
    /// JSON pointer-like path.
    pub path:    String,
    /// Human-readable message.
    pub message: String,
}

/// Validate a pipeline against the WIA-CICD spec ENUMs.
pub fn validate(p: &Pipeline) -> Result<(), Vec<ValidationError>> {
    let mut errs = Vec::new();
    if p.name.is_empty() { errs.push(ValidationError { path: "/name".into(), message: "required".into() }); }
    if p.nodes.is_empty() { errs.push(ValidationError { path: "/nodes".into(), message: "at least one node required".into() }); }
    let valid_ids: BTreeSet<&str> = PIPELINE_NODES.iter().copied().collect();
    for (i, n) in p.nodes.iter().enumerate() {
        if !valid_ids.contains(n.id.as_str()) {
            errs.push(ValidationError { path: format!("/nodes/{}/id", i), message: format!("invalid id {}", n.id) });
        }
    }
    if errs.is_empty() { Ok(()) } else { Err(errs) }
}

/// Topological sort. Returns the linearised execution order or an error on cycle.
pub fn topo_sort(p: &Pipeline) -> Result<Vec<String>, &'static str> {
    let mut in_deg: BTreeMap<&str, usize>     = BTreeMap::new();
    let mut edges:  BTreeMap<&str, Vec<&str>> = BTreeMap::new();
    for n in &p.nodes { in_deg.insert(&n.id, 0); edges.entry(&n.id).or_default(); }
    for n in &p.nodes {
        for prev in &n.needs {
            *in_deg.entry(&n.id).or_insert(0) += 1;
            edges.entry(prev.as_str()).or_default().push(&n.id);
        }
    }
    let mut ready: Vec<&str> = in_deg.iter().filter(|(_, &d)| d == 0).map(|(k, _)| *k).collect();
    ready.sort();
    let mut order: Vec<String> = Vec::new();
    while let Some(cur) = ready.first().copied() {
        ready.remove(0);
        order.push(cur.to_string());
        if let Some(nexts) = edges.get(cur) {
            for next in nexts.clone() {
                let entry = in_deg.entry(next).or_insert(1);
                *entry = entry.saturating_sub(1);
                if *entry == 0 { ready.push(next); ready.sort(); }
            }
        }
    }
    if order.len() != p.nodes.len() { return Err("cycle detected"); }
    Ok(order)
}
