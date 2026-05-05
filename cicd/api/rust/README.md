# wia-cicd (Rust)

Reference SDK for the WIA-CICD open standard initiative.

```toml
[dependencies]
wia-cicd = "1.0.0"
```

```rust
use wia_cicd::{Pipeline, Trigger, Node, NodeType, validate, topo_sort};

let p = Pipeline {
    name:    "sample".into(),
    trigger: Trigger::Push,
    nodes:   vec![
        Node::new("SOURCE",        NodeType::Source),
        Node::new("BUILD",         NodeType::Build).needs(&["SOURCE"]),
        Node::new("TEST_GATE",     NodeType::Test).needs(&["BUILD"]),
        Node::new("SECURITY_GATE", NodeType::Security).needs(&["TEST_GATE"]),
        Node::new("CD_HANDOFF",    NodeType::Deliver).needs(&["SECURITY_GATE"]),
    ],
};
validate(&p).unwrap();
let order = topo_sort(&p).unwrap();
assert_eq!(order, vec!["SOURCE","BUILD","TEST_GATE","SECURITY_GATE","CD_HANDOFF"]);
```

弘益人間 — Benefit All Humanity · MIT License
