# WIA-AI-012: Federated Learning - Phase 3: Protocol Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01-09

## Overview

This specification defines the federated learning protocols, coordination mechanisms, and security procedures.

## Philosophy

弘益人間 (Hongik Ingan) - Standardized protocols enable trustworthy, privacy-preserving collaboration that benefits all participants.

## 1. Training Round Protocol

### 1.1 Round Lifecycle

```
┌─────────────┐
│ INITIALIZED │
└──────┬──────┘
       │ server.initializeRound()
       ▼
┌─────────────┐
│  SELECTING  │ ← Client selection phase
└──────┬──────┘
       │ server.inviteClients()
       ▼
┌─────────────┐
│   TRAINING  │ ← Clients train locally
└──────┬──────┘
       │ clients.submitUpdates()
       ▼
┌─────────────┐
│ AGGREGATING │ ← Server aggregates updates
└──────┬──────┘
       │ server.aggregate()
       ▼
┌─────────────┐
│  COMPLETED  │ ← New global model distributed
└─────────────┘
```

### 1.2 Round Initialization

**Server Procedure:**
```python
def initialize_round(self, round_number):
    # 1. Validate prerequisites
    assert self.global_model is not None
    assert self.can_start_new_round()

    # 2. Create round metadata
    round_info = {
        'round_number': round_number,
        'global_model_version': self.global_model.version,
        'started_at': now(),
        'deadline': now() + config.round_duration,
        'min_clients': config.min_clients,
        'target_clients': config.target_clients
    }

    # 3. Persist round state
    self.db.create_round(round_info)

    # 4. Transition to SELECTING state
    self.state = RoundState.SELECTING

    return round_info
```

### 1.3 Client Selection Protocol

```python
def select_clients(self, round_number, strategy='random'):
    # 1. Query available clients
    available_clients = self.registry.get_eligible_clients({
        'last_seen': within_24_hours(),
        'battery': above(0.3),
        'network': 'wifi'
    })

    # 2. Apply selection strategy
    if strategy == 'random':
        selected = random.sample(available_clients, k=target_clients)
    elif strategy == 'fair':
        selected = self.fair_selector.select(available_clients)
    elif strategy == 'diverse':
        selected = self.diversity_selector.select(available_clients)

    # 3. Send invitations
    for client in selected:
        self.send_invitation(client, round_number)

    # 4. Track invited clients
    self.db.record_invitations(round_number, selected)

    return selected
```

### 1.4 Client Training Protocol

**Client Procedure:**
```python
def participate_in_round(self, invitation):
    # 1. Validate invitation
    assert self.verify_invitation_signature(invitation)
    assert invitation.deadline > now()
    assert self.meets_requirements(invitation.requirements)

    # 2. Download global model
    global_model = self.download_model(invitation.model_url)

    # 3. Train locally
    try:
        local_model = self.train_locally(
            global_model,
            epochs=invitation.config.local_epochs,
            batch_size=invitation.config.batch_size
        )

        # 4. Compute update
        model_update = local_model - global_model

        # 5. Apply privacy mechanisms
        private_update = self.apply_privacy(model_update)

        # 6. Compress update
        compressed_update = self.compress(private_update)

        # 7. Submit to server
        receipt = self.submit_update(compressed_update, invitation.round_number)

        return receipt

    except Exception as e:
        self.report_failure(invitation.round_number, error=e)
        raise
```

### 1.5 Update Submission Protocol

```python
def submit_update(self, client_id, round_number, update_data):
    # 1. Validate timing
    round_info = self.db.get_round(round_number)
    if now() > round_info.deadline:
        raise DeadlineExpiredError()

    # 2. Validate client authorization
    if client_id not in round_info.invited_clients:
        raise UnauthorizedClientError()

    # 3. Verify signature
    assert self.crypto.verify_signature(
        update_data.signature,
        update_data.payload,
        client_id
    )

    # 4. Validate update format
    assert self.validator.validate_update(update_data)

    # 5. Store update
    update_id = self.storage.save_update(
        round_number,
        client_id,
        update_data
    )

    # 6. Check if ready to aggregate
    submitted_count = self.db.count_submitted_updates(round_number)
    if submitted_count >= round_info.min_clients:
        self.trigger_aggregation(round_number)

    return {'update_id': update_id, 'status': 'accepted'}
```

### 1.6 Aggregation Protocol

```python
def aggregate_updates(self, round_number):
    # 1. Collect all updates
    updates = self.db.get_round_updates(round_number)

    # 2. Anomaly detection
    trusted_updates = self.anomaly_detector.filter(updates)

    # 3. Apply aggregation algorithm
    if self.config.aggregation == 'fedavg':
        new_global = self.fedavg(trusted_updates)
    elif self.config.aggregation == 'krum':
        new_global = self.krum(trusted_updates)
    elif self.config.aggregation == 'median':
        new_global = self.coordinate_median(trusted_updates)

    # 4. Add server-side differential privacy
    private_global = self.add_central_dp_noise(new_global)

    # 5. Evaluate new model
    metrics = self.evaluate(private_global)

    # 6. Accept or reject
    if self.should_accept(metrics):
        self.global_model = private_global
        self.publish_model(private_global)

    # 7. Complete round
    self.db.complete_round(round_number, metrics)

    return new_global
```

## 2. Secure Aggregation Protocol

### 2.1 Bonawitz et al. Secure Aggregation

**Phase 1: Setup**
```python
def setup_secure_aggregation(clients):
    # Each client generates key pairs
    for client in clients:
        private_key, public_key = generate_keypair()
        client.store_keys(private_key, public_key)
        server.register_public_key(client.id, public_key)
```

**Phase 2: Share Keys**
```python
def share_keys_phase(clients):
    # Clients establish pairwise shared secrets
    for client_i in clients:
        for client_j in clients:
            if client_i.id < client_j.id:
                shared_secret = dh_exchange(
                    client_i.private_key,
                    client_j.public_key
                )
                client_i.pairwise_secrets[client_j.id] = shared_secret
```

**Phase 3: Masked Input Collection**
```python
def collect_masked_inputs(clients):
    masked_inputs = []

    for client in clients:
        # Generate mask from pairwise secrets
        mask = client.generate_mask_from_secrets()

        # Add mask to update
        masked_update = client.update + mask

        # Submit masked update
        masked_inputs.append(masked_update)

    return masked_inputs
```

**Phase 4: Unmasking**
```python
def unmask_aggregate(masked_inputs):
    # Sum all masked inputs (masks cancel out)
    aggregate = sum(masked_inputs)

    # Masks sum to zero, leaving true aggregate
    # aggregate = Σ(update_i + mask_i) = Σupdate_i

    return aggregate
```

## 3. Differential Privacy Protocol

### 3.1 Local Differential Privacy

```python
def apply_local_dp(gradient, epsilon, delta):
    # 1. Clip gradient
    clipped = clip_by_norm(gradient, max_norm=1.0)

    # 2. Calculate noise scale
    sensitivity = max_norm
    sigma = sensitivity * sqrt(2 * log(1.25 / delta)) / epsilon

    # 3. Add Gaussian noise
    noise = gaussian_noise(shape=gradient.shape, stddev=sigma)
    private_gradient = clipped + noise

    return private_gradient
```

### 3.2 Privacy Accounting

```python
class PrivacyAccountant:
    def __init__(self, epsilon_total, delta_total):
        self.epsilon_budget = epsilon_total
        self.delta_budget = delta_total
        self.epsilon_spent = 0
        self.delta_spent = 0

    def check_budget(self, epsilon_round, delta_round):
        """Check if sufficient budget remains"""
        return (
            self.epsilon_spent + epsilon_round <= self.epsilon_budget and
            self.delta_spent + delta_round <= self.delta_budget
        )

    def spend(self, epsilon_round, delta_round):
        """Deduct from privacy budget"""
        if not self.check_budget(epsilon_round, delta_round):
            raise PrivacyBudgetExceededError()

        self.epsilon_spent += epsilon_round
        self.delta_spent += delta_round
```

## 4. Communication Protocol

### 4.1 Chunked Upload for Large Models

```python
def upload_large_update(update_data, chunk_size=1MB):
    total_size = len(update_data)
    num_chunks = ceil(total_size / chunk_size)

    # Initiate upload
    upload_id = server.initiate_upload(total_size, num_chunks)

    # Upload chunks
    for i, chunk in enumerate(split_into_chunks(update_data, chunk_size)):
        server.upload_chunk(upload_id, chunk_index=i, data=chunk)

        # Report progress
        progress = (i + 1) / num_chunks
        server.update_progress(upload_id, progress)

    # Finalize
    server.finalize_upload(upload_id)
```

### 4.2 Resume Support

```python
def resumable_download(model_url):
    # Check if partial download exists
    partial_path = cache_dir / 'partial_model.bin'
    bytes_downloaded = partial_path.size() if partial_path.exists() else 0

    # Request remaining bytes
    response = requests.get(
        model_url,
        headers={'Range': f'bytes={bytes_downloaded}-'}
    )

    # Append to partial file
    with open(partial_path, 'ab') as f:
        f.write(response.content)

    # Verify complete
    if partial_path.size() == expected_size:
        rename(partial_path, 'model.bin')
```

## 5. Failure Handling

### 5.1 Client Dropout

```python
def handle_client_dropout(round_number, client_id):
    # 1. Mark client as dropped
    db.mark_client_dropped(round_number, client_id)

    # 2. Check if still have minimum clients
    remaining = db.count_active_clients(round_number)

    if remaining < config.min_clients:
        # 3. Select backup clients
        backups = select_backup_clients(
            needed=config.min_clients - remaining
        )

        # 4. Send late invitations
        for backup in backups:
            send_invitation(backup, round_number, urgent=True)

    # 5. Update round deadline if needed
    if remaining < config.target_clients * 0.5:
        extend_deadline(round_number, additional_time=30min)
```

### 5.2 Server Failure Recovery

```python
def recover_from_failure():
    # 1. Load latest checkpoint
    checkpoint = load_latest_checkpoint()

    # 2. Restore state
    self.global_model = checkpoint.model
    self.round_number = checkpoint.round
    self.state = checkpoint.state

    # 3. Resume round if in progress
    if self.state == RoundState.TRAINING:
        # Check deadline
        if now() < checkpoint.deadline:
            # Resume collection
            wait_for_updates(checkpoint.round)
        else:
            # Deadline passed, aggregate what we have
            aggregate_updates(checkpoint.round)
```

## 6. Quality Assurance

### 6.1 Update Validation

```python
def validate_update(update):
    checks = [
        check_format(update),           # Correct structure
        check_dimensions(update),        # Matching model shape
        check_magnitude(update),         # Reasonable values
        check_staleness(update),         # Not too old
        check_signature(update),         # Cryptographic auth
        check_privacy_budget(update),    # DP parameters
    ]

    return all(checks)
```

### 6.2 Model Validation

```python
def validate_new_model(new_model, old_model):
    # 1. Sanity checks
    assert new_model.shape == old_model.shape
    assert not has_nan_or_inf(new_model)

    # 2. Performance check
    new_accuracy = evaluate(new_model)
    old_accuracy = evaluate(old_model)

    # Accept if improved or not degraded significantly
    return new_accuracy >= old_accuracy * 0.95
```

---

**Copyright © 2025 SmileStory Inc. / World Certification Industry Association**
**弘益人間 (Hongik Ingan) · Benefit All Humanity**

**License:** CC BY 4.0
