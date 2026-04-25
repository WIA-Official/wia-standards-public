# WIA-AI-013: AI Bias Detection Standard
## Phase 4: Integration Specification

**Version:** 1.0
**Status:** Final
**Last Updated:** 2025-01-08
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This specification defines how to integrate AI bias detection into existing ML pipelines, CI/CD workflows, and production systems following 弘益人間 principles.

---

## 2. ML Pipeline Integration

### 2.1 Training Pipeline Integration

```python
# Example: Integrate bias detection into training pipeline

from wia_ai_013 import BiasDetector, FairnessMetrics

# During training
detector = BiasDetector(
    protected_attributes=['gender', 'race'],
    fairness_criteria={
        'demographic_parity_ratio': 0.8,
        'equalized_odds_disparity': 0.1
    }
)

# Pre-training: Check data bias
data_report = detector.analyze_dataset(
    data=training_data,
    labels=labels,
    protected_attrs=protected_attributes
)

if data_report.status == 'fail':
    # Apply mitigation
    mitigated_data = detector.mitigate_data_bias(
        data=training_data,
        strategy='reweighting'
    )
    training_data = mitigated_data

# Train model
model = train_model(training_data, labels)

# Post-training: Check model bias
model_report = detector.analyze_model(
    model=model,
    test_data=test_data,
    test_labels=test_labels,
    protected_attrs=test_protected_attrs
)

if model_report.status != 'pass':
    raise BiasViolationError(f"Model fails fairness criteria: {model_report.violations}")

# Save model with bias report
model.save_with_metadata({
    'bias_report': model_report.to_json(),
    'wia_ai_013_compliant': True
})
```

### 2.2 Evaluation Pipeline Integration

```python
# Automated fairness evaluation

def evaluate_with_fairness(model, test_data, protected_attrs):
    """Evaluate model with standard metrics + fairness"""

    # Standard evaluation
    accuracy = model.evaluate(test_data)

    # Fairness evaluation
    detector = BiasDetector()
    fairness_report = detector.analyze_model(
        model=model,
        test_data=test_data,
        protected_attrs=protected_attrs
    )

    return {
        'accuracy': accuracy,
        'fairness_metrics': fairness_report.metrics,
        'fairness_status': fairness_report.status,
        'recommendations': fairness_report.recommendations
    }
```

---

## 3. CI/CD Integration

### 3.1 Pre-Deployment Checks

```yaml
# .gitlab-ci.yml / .github/workflows/ml-pipeline.yml

bias_check:
  stage: test
  script:
    - pip install wia-ai-013-sdk
    - python bias_check.py --model $MODEL_PATH --data $TEST_DATA
  artifacts:
    reports:
      wia_ai_013: bias_report.json
  rules:
    - if: '$CI_PIPELINE_SOURCE == "merge_request"'

deploy:
  stage: deploy
  script:
    - deploy_model.sh
  dependencies:
    - bias_check
  only:
    - main
  when: manual  # Require manual approval if bias detected
```

### 3.2 Bias Check Script

```python
# bias_check.py

import sys
import argparse
from wia_ai_013 import BiasDetector

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True)
    parser.add_argument('--data', required=True)
    parser.add_argument('--threshold', default=0.8, type=float)
    args = parser.parse_args()

    detector = BiasDetector()
    report = detector.analyze_model_from_file(
        model_path=args.model,
        data_path=args.data
    )

    # Save report
    report.save('bias_report.json')

    # Check compliance
    if report.demographic_parity_ratio < args.threshold:
        print(f"❌ FAIL: Demographic parity ratio {report.demographic_parity_ratio} < {args.threshold}")
        sys.exit(1)

    print(f"✅ PASS: All fairness checks passed")
    sys.exit(0)

if __name__ == '__main__':
    main()
```

---

## 4. Production Monitoring Integration

### 4.1 Model Serving Integration

```python
# Flask/FastAPI integration example

from flask import Flask, request, jsonify
from wia_ai_013 import ProductionMonitor

app = Flask(__name__)
monitor = ProductionMonitor(
    model_id='credit-model-v2',
    protected_attributes=['gender', 'race'],
    alert_webhook='https://alerts.example.com/bias'
)

@app.route('/predict', methods=['POST'])
def predict():
    data = request.json

    # Make prediction
    prediction = model.predict(data['features'])

    # Log for bias monitoring
    monitor.log_prediction(
        prediction_id=generate_id(),
        input_features=data['features'],
        prediction=prediction,
        protected_attrs=data.get('protected_attributes')
    )

    return jsonify({
        'prediction': prediction,
        'prediction_id': monitor.last_prediction_id
    })

@app.route('/feedback', methods=['POST'])
def feedback():
    data = request.json

    # Log actual outcome
    monitor.log_outcome(
        prediction_id=data['prediction_id'],
        actual_outcome=data['actual_outcome']
    )

    return jsonify({'status': 'recorded'})

# Periodic fairness check
@app.before_first_request
def setup_monitoring():
    monitor.start_periodic_evaluation(interval_minutes=60)
```

### 4.2 Kubernetes Sidecar Pattern

```yaml
# k8s-deployment.yml

apiVersion: apps/v1
kind: Deployment
metadata:
  name: ml-model
spec:
  template:
    spec:
      containers:
      - name: model-server
        image: myorg/model:v2
        ports:
        - containerPort: 8080

      - name: bias-monitor
        image: wia/bias-monitor:latest
        env:
        - name: MODEL_ENDPOINT
          value: "http://localhost:8080"
        - name: PROTECTED_ATTRIBUTES
          value: "gender,race"
        - name: ALERT_WEBHOOK
          value: "https://alerts.example.com/bias"
        ports:
        - containerPort: 9090
```

---

## 5. Data Pipeline Integration

### 5.1 Apache Airflow DAG

```python
# airflow_dag.py

from airflow import DAG
from airflow.operators.python import PythonOperator
from wia_ai_013 import BiasDetector
from datetime import datetime, timedelta

def check_data_bias(**context):
    detector = BiasDetector()
    data = load_data_from_warehouse()

    report = detector.analyze_dataset(data)

    if report.status == 'fail':
        raise AirflowException(f"Data bias detected: {report.violations}")

    context['task_instance'].xcom_push(key='bias_report', value=report.to_dict())

def train_model(**context):
    bias_report = context['task_instance'].xcom_pull(key='bias_report')

    if bias_report['status'] == 'warning':
        # Apply mitigation
        data = apply_bias_mitigation(load_data())
    else:
        data = load_data()

    model = train(data)
    return model

with DAG(
    'ml_training_with_bias_check',
    start_date=datetime(2025, 1, 1),
    schedule_interval=timedelta(days=1)
) as dag:

    check_bias = PythonOperator(
        task_id='check_data_bias',
        python_callable=check_data_bias
    )

    train = PythonOperator(
        task_id='train_model',
        python_callable=train_model
    )

    check_bias >> train
```

### 5.2 Apache Spark Integration

```python
# Spark MLlib integration

from pyspark.ml import Pipeline
from wia_ai_013.spark import BiasDetectorTransformer

# Add bias detection to Spark pipeline
pipeline = Pipeline(stages=[
    vectorizer,
    scaler,
    BiasDetectorTransformer(
        protectedAttributeCols=['gender', 'race'],
        fairnessCriteria={'demographic_parity_ratio': 0.8}
    ),
    classifier
])

model = pipeline.fit(training_data)
```

---

## 6. Monitoring Platform Integration

### 6.1 Prometheus Integration

```python
# Export fairness metrics to Prometheus

from prometheus_client import Gauge, Counter
from wia_ai_013 import BiasMonitor

# Define metrics
demographic_parity = Gauge(
    'model_demographic_parity_ratio',
    'Demographic parity ratio',
    ['model_id', 'protected_attribute']
)

bias_alerts = Counter(
    'bias_alerts_total',
    'Total bias alerts triggered',
    ['model_id', 'severity']
)

# Monitor and export
monitor = BiasMonitor(model_id='credit-model')

def export_metrics():
    metrics = monitor.get_current_metrics()

    for attr, value in metrics.demographic_parity.by_attribute.items():
        demographic_parity.labels(
            model_id='credit-model',
            protected_attribute=attr
        ).set(value)
```

### 6.2 Grafana Dashboard

```json
{
  "dashboard": {
    "title": "AI Fairness Monitoring",
    "panels": [
      {
        "title": "Demographic Parity Over Time",
        "targets": [
          {
            "expr": "model_demographic_parity_ratio{model_id=\"credit-model\"}",
            "legendFormat": "{{protected_attribute}}"
          }
        ],
        "alert": {
          "conditions": [
            {
              "evaluator": {
                "params": [0.8],
                "type": "lt"
              },
              "operator": {"type": "and"},
              "query": {"params": ["A", "5m", "now"]},
              "reducer": {"type": "avg"},
              "type": "query"
            }
          ],
          "name": "Demographic Parity Violation"
        }
      }
    ]
  }
}
```

---

## 7. MLOps Platform Integration

### 7.1 MLflow Integration

```python
# MLflow experiment tracking with fairness

import mlflow
from wia_ai_013 import BiasDetector

with mlflow.start_run():
    # Train model
    model = train_model(data)

    # Log model
    mlflow.sklearn.log_model(model, "model")

    # Bias detection
    detector = BiasDetector()
    report = detector.analyze_model(model, test_data)

    # Log fairness metrics
    mlflow.log_metrics({
        'demographic_parity': report.demographic_parity_ratio,
        'equalized_odds': report.equalized_odds_disparity,
        'accuracy': report.accuracy
    })

    # Log full bias report
    mlflow.log_artifact('bias_report.json')

    # Tag with compliance status
    mlflow.set_tag('wia_ai_013_compliant', report.status == 'pass')
```

### 7.2 Kubeflow Integration

```python
# Kubeflow pipeline component

from kfp import dsl
from kfp.components import create_component_from_func

def bias_check_component(
    model_path: str,
    data_path: str,
    threshold: float = 0.8
) -> str:
    from wia_ai_013 import BiasDetector

    detector = BiasDetector()
    report = detector.analyze_from_paths(model_path, data_path)

    if report.demographic_parity_ratio < threshold:
        raise ValueError(f"Bias check failed: {report.violations}")

    return report.to_json()

bias_check_op = create_component_from_func(
    bias_check_component,
    base_image='python:3.9',
    packages_to_install=['wia-ai-013-sdk']
)

@dsl.pipeline(name='ML Pipeline with Bias Check')
def ml_pipeline():
    train_task = train_op()
    bias_task = bias_check_op(
        model_path=train_task.outputs['model'],
        data_path='/data/test.csv'
    )
    deploy_task = deploy_op(model_path=bias_task.outputs['model'])
```

---

## 8. Language-Specific SDKs

### 8.1 Python SDK

```python
from wia_ai_013 import BiasDetector, FairnessMetrics

detector = BiasDetector(config='config.yaml')
report = detector.analyze_model(model, test_data)
```

### 8.2 JavaScript/TypeScript SDK

```typescript
import { BiasDetector } from '@wia/ai-013';

const detector = new BiasDetector({
  protectedAttributes: ['gender', 'race'],
  fairnessCriteria: {
    demographicParityRatio: 0.8
  }
});

const report = await detector.analyzeModel(model, testData);
```

### 8.3 Java SDK

```java
import org.wia.ai013.BiasDetector;
import org.wia.ai013.BiasReport;

BiasDetector detector = new BiasDetector.Builder()
    .withProtectedAttributes("gender", "race")
    .withFairnessCriteria(FairnessCriteria.DEMOGRAPHIC_PARITY, 0.8)
    .build();

BiasReport report = detector.analyzeModel(model, testData);
```

---

## 9. Compliance Checklist

### 9.1 Integration Requirements

- [ ] Bias checks integrated into training pipeline
- [ ] Automated testing before deployment
- [ ] Production monitoring enabled
- [ ] Alert system configured
- [ ] Audit logging implemented
- [ ] Documentation updated
- [ ] Team trained on tools

### 9.2 弘益人間 Integration Principles

- Transparency: All bias checks visible to stakeholders
- Accessibility: Monitoring dashboards available to authorized users
- Accountability: Clear ownership of bias issues
- Continuous Improvement: Regular review and updates
- Privacy: Protect individual data in logs

---

## 10. Best Practices

### 10.1 Gradual Rollout

1. **Pilot**: Test on non-critical model
2. **Monitor**: Observe for false positives
3. **Tune**: Adjust thresholds based on context
4. **Expand**: Roll out to additional models
5. **Automate**: Enforce in CI/CD

### 10.2 Team Collaboration

- **Data Scientists**: Implement bias checks in notebooks
- **ML Engineers**: Integrate into pipelines
- **DevOps**: Monitor production systems
- **Compliance**: Review audit logs
- **Product**: Define fairness requirements

---

**Document Version:** 1.0
**Effective Date:** 2025-01-08
**Maintained By:** WIA Standards Committee
**License:** CC BY 4.0

弘益人間 · Benefit All Humanity

---

## Annex A — Conformance Tier Matrix

WIA conformance for ai-bias-detection is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/ai-bias-detection/api/` — TypeScript SDK skeleton
- `wia-standards/standards/ai-bias-detection/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/ai-bias-detection/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---
