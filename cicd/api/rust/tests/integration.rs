//! Integration tests for `wia-cicd`.
//!
//! These tests treat the SDK as an external user would: only the public
//! re-exports from `lib.rs` are touched.

use wia_cicd::{
    compose_cache_hit, emit_cyclonedx, emit_spdx_tag_value, error_budget_burn, evaluate_dora,
    meets_slsa_target, topo_sort, validate, DeployStrategy, DoraMetrics, Node, NodeType, Pipeline,
    PolicyMode, SbomComponent, SbomMeta, SecurityGate, SlsaLevel, Trigger,
};

fn elite_pipeline() -> Pipeline {
    Pipeline {
        name:    "wia-cicd-reference".to_string(),
        trigger: Trigger::Push,
        nodes: vec![
            Node::new("SOURCE",         NodeType::Source),
            Node::new("BUILD",          NodeType::Build).needs(&["SOURCE"]),
            Node::new("TEST_GATE",      NodeType::Test).needs(&["BUILD"]),
            Node {
                gates:    vec![SecurityGate::Sast, SecurityGate::Sca, SecurityGate::Secrets],
                policy:   Some(PolicyMode::Enforce),
                ..Node::new("SECURITY_GATE", NodeType::Security).needs(&["BUILD"])
            },
            Node {
                strategy:      Some(DeployStrategy::Canary),
                traffic_curve: vec![5, 25, 50, 100],
                ..Node::new("CD_HANDOFF", NodeType::Deliver).needs(&["TEST_GATE", "SECURITY_GATE"])
            },
        ],
    }
}

#[test]
fn validates_reference_pipeline() {
    assert!(validate(&elite_pipeline()).is_ok());
}

#[test]
fn rejects_unknown_node_id() {
    let mut p = elite_pipeline();
    p.nodes[0].id = "FOO".to_string();
    let errs = validate(&p).unwrap_err();
    assert!(errs.iter().any(|e| e.path == "/nodes/0/id"));
}

#[test]
fn topo_sort_respects_dependencies() {
    let order = topo_sort(&elite_pipeline()).expect("acyclic");
    let pos = |id: &str| order.iter().position(|x| x == id).unwrap();
    assert!(pos("SOURCE")    < pos("BUILD"));
    assert!(pos("BUILD")     < pos("TEST_GATE"));
    assert!(pos("BUILD")     < pos("SECURITY_GATE"));
    assert!(pos("TEST_GATE") < pos("CD_HANDOFF"));
    assert!(pos("SECURITY_GATE") < pos("CD_HANDOFF"));
}

#[test]
fn topo_sort_detects_cycle() {
    let mut p = elite_pipeline();
    // Inject SOURCE -> SOURCE self-loop.
    p.nodes[0].needs = vec!["SOURCE".to_string()];
    assert!(topo_sort(&p).is_err());
}

#[test]
fn elite_metrics_pass_all_axes() {
    let m = DoraMetrics {
        lead_time_hr: 0.4,
        deploy_freq_per_day: 12.0,
        failure_rate: 0.02,
        mttr_min: 3.0,
        rework_rate: 0.03,
        fast_feedback_p95_min: Some(8.0),
        security_scan_p95_min: Some(12.0),
        prod_deploy_p95_min: Some(45.0),
    };
    let v = evaluate_dora(&m);
    assert!(v.elite);
    assert!(v.failures.is_empty());
    assert!((v.score - 1.0).abs() < 1e-9);
}

#[test]
fn slow_lead_time_blocks_elite() {
    let m = DoraMetrics { lead_time_hr: 6.0, ..Default::default() };
    let v = evaluate_dora(&m);
    assert!(!v.elite);
    assert!(v.failures.iter().any(|k| *k == "lead_time_hr"));
}

#[test]
fn cache_hit_composition_bounds() {
    assert!((compose_cache_hit(0.0, 0.0, 0.0)).abs() < 1e-9);
    assert!((compose_cache_hit(1.0, 0.0, 0.0) - 1.0).abs() < 1e-9);
    let mid = compose_cache_hit(0.8, 0.6, 0.4);
    assert!(mid > 0.95 && mid <= 1.0);
}

#[test]
fn error_budget_burn_against_three_nines() {
    let burn = error_budget_burn(0.015, 0.999);
    assert!(burn > 14.0 && burn < 16.0);
}

#[test]
fn cyclonedx_contains_components() {
    let meta = SbomMeta {
        serial_number: "urn:uuid:0".into(),
        timestamp:     "2026-05-05T20:00:00Z".into(),
        tool_vendor:   "WIA".into(),
        tool_name:     "wia-cicd".into(),
        tool_version:  "1.0.0".into(),
    };
    let comps = vec![SbomComponent {
        kind:     "library",
        name:     "serde".into(),
        version:  "1.0".into(),
        purl:     Some("pkg:cargo/serde@1.0".into()),
        licenses: vec!["MIT".into()],
    }];
    let json = emit_cyclonedx(&meta, &comps);
    assert!(json.contains("\"bomFormat\":\"CycloneDX\""));
    assert!(json.contains("\"specVersion\":\"1.5\""));
    assert!(json.contains("pkg:cargo/serde@1.0"));
}

#[test]
fn spdx_tag_value_well_formed() {
    let meta = SbomMeta {
        serial_number: "urn:uuid:1".into(),
        timestamp:     "2026-05-05T20:00:00Z".into(),
        tool_vendor:   "WIA".into(),
        tool_name:     "wia-cicd".into(),
        tool_version:  "1.0.0".into(),
    };
    let comps = vec![SbomComponent {
        kind:     "application",
        name:     "demo".into(),
        version:  "0.1.0".into(),
        purl:     None,
        licenses: vec!["Apache-2.0".into()],
    }];
    let doc = emit_spdx_tag_value(&meta, &comps);
    assert!(doc.starts_with("SPDXVersion: SPDX-2.3"));
    assert!(doc.contains("PackageName: demo"));
    assert!(doc.contains("Apache-2.0"));
}

#[test]
fn slsa_l3_signed_is_target() {
    assert!(meets_slsa_target(&SlsaLevel::L3, true));
    assert!(!meets_slsa_target(&SlsaLevel::L3, false));
    assert!(!meets_slsa_target(&SlsaLevel::L2, true));
}
