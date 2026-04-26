# WIA-AI-016 Phase 3: Protocol Specification

> 弘益人間 (홍익인간) · Benefit All Humanity

## Overview

Phase 3 defines communication protocols, coordination algorithms, and consensus mechanisms for WIA-AI-016 multi-agent systems.

## FIPA Interaction Protocols

### Request Protocol

```
Initiator          Participant
    |                   |
    |---REQUEST-------->|
    |                   |
    |<--AGREE/REFUSE----|
    |                   |
    |<--INFORM/FAILURE--|
    |                   |
```

**Steps:**
1. Initiator sends REQUEST
2. Participant responds with AGREE or REFUSE
3. If AGREE, participant sends INFORM (success) or FAILURE

### Contract Net Protocol

```
Manager           Contractors
    |                  |
    |---CFP----------->| (broadcast)
    |                  |
    |<--PROPOSE--------|
    |<--REFUSE---------|
    |                  |
    |---ACCEPT-------->| (to winner)
    |---REJECT-------->| (to others)
    |                  |
    |<--INFORM/FAILURE-|
```

**Steps:**
1. Manager broadcasts CFP (Call for Proposals)
2. Contractors respond with PROPOSE or REFUSE
3. Manager evaluates and sends ACCEPT to winner, REJECT to others
4. Winner executes and reports result

### Query Protocol

```
Initiator          Participant
    |                   |
    |---QUERY-IF------->|
    |                   |
    |<--INFORM----------|
    |                   |
```

### Subscribe Protocol

```
Subscriber         Publisher
    |                   |
    |---SUBSCRIBE------>|
    |                   |
    |<--AGREE-----------|
    |                   |
    |<--INFORM----------| (periodic updates)
    |<--INFORM----------|
    |                   |
    |---CANCEL--------->|
```

## Negotiation Protocols

### Monotonic Concession

```typescript
interface NegotiationRound {
    proposal: number;
    concession: number;
    minAcceptable: number;
}

// Each round, agents make concessions toward agreement
while (rounds < maxRounds && !agreement) {
    myProposal = makeProposal();
    theirProposal = await receiveProposal();

    if (isAcceptable(theirProposal)) {
        agreement = formAgreement(myProposal, theirProposal);
        break;
    }

    makeConcession(theirProposal);
    rounds++;
}
```

### Multi-Issue Negotiation

Issues are negotiated in parallel with trade-offs:

```json
{
  "issues": [
    {"name": "price", "value": 100, "importance": 0.6},
    {"name": "delivery", "value": 7, "importance": 0.3},
    {"name": "quality", "value": 0.9, "importance": 0.1}
  ],
  "utilityFunction": "weighted_sum"
}
```

## Consensus Algorithms

### Raft Consensus

**Roles:**
- Leader: Coordinates consensus
- Follower: Accepts leader commands
- Candidate: Competes for leadership

**Process:**
1. **Leader Election**: Followers timeout → become candidates → request votes
2. **Log Replication**: Leader receives commands → replicates to followers
3. **Commitment**: When majority ack → leader commits → followers commit

```
Term 1          Term 2
Leader          Follower becomes Candidate
  |                    |
  |-- heartbeat ------>|
  |<-- ack ------------|
  |                    |
  [Leader fails]       |
                       | (election timeout)
                       |-- requestVote -->
                       |<-- voteGranted ---
                       | (becomes leader)
```

### PBFT (Practical Byzantine Fault Tolerance)

Tolerates up to f Byzantine (malicious) faults with 3f+1 nodes.

**Phases:**
1. **Pre-Prepare**: Primary broadcasts request
2. **Prepare**: Replicas broadcast prepare messages
3. **Commit**: After receiving 2f prepares, broadcast commit
4. **Reply**: After 2f+1 commits, execute and reply

## Coordination Patterns

### Blackboard System

Shared data space for indirect coordination:

```
Agents write to/read from shared blackboard

Agent1 --[write]--> Blackboard <--[read]-- Agent2
Agent3 --[read]-->      |
                        |
                    [Triggers]
                        |
                    Agent4 activated
```

### Publish-Subscribe

```
Publisher              Broker              Subscribers
    |                    |                     |
    |--publish(topic)--->|                     |
    |                    |--notify(topic)----->|
    |                    |--notify(topic)----->|
```

### Master-Slave

```
Master
  |
  |-- delegates tasks -->
  |                       Slave1
  |                       Slave2
  |<-- results ----------Slave3
```

## Resource Allocation

### Auction Mechanisms

**English Auction (Ascending)**
```
price = startPrice
while (no winner) {
    bids = collectBids(currentPrice);
    if (bids.length > 0) {
        currentPrice += increment;
    } else {
        winner = highestBidder;
    }
}
```

**Dutch Auction (Descending)**
```
price = startPrice
while (price > 0 && no acceptance) {
    if (anyoneAccepts(currentPrice)) {
        winner = firstAccepter;
        break;
    }
    currentPrice -= decrement;
}
```

**Vickrey (Second-Price Sealed-Bid)**
```
bids = collectSealedBids();
sortedBids = sort(bids, descending);
winner = sortedBids[0].bidder;
price = sortedBids[1].amount; // Pay second-highest
```

## Load Balancing Protocols

### Round Robin
```
nextAgent = agents[currentIndex];
currentIndex = (currentIndex + 1) % agents.length;
```

### Least Loaded
```
nextAgent = agents.reduce((min, agent) =>
    agent.load < min.load ? agent : min
);
```

### Weighted Round Robin
```
weights = [agent.capacity for agent in agents];
nextAgent = weightedSelect(agents, weights);
```

## Security Protocols

### Challenge-Response Authentication

```
Client                Server
  |                      |
  |--1. connect--------->|
  |<-2. challenge--------|
  |                      |
  |--3. sign(challenge)->|
  |                      |
  |<-4. token/reject-----|
```

### Secure Message Exchange

```
Sender                  Receiver
  |                        |
  |--1. encrypt(message)-->|
  |     + signature        |
  |                        |
  |                    2. verify signature
  |                    3. decrypt message
```

## Performance Optimizations

### Message Batching

```
batch = []
timer = startTimer(maxDelay)

onMessage(msg) {
    batch.push(msg);
    if (batch.length >= maxBatchSize) {
        sendBatch(batch);
        batch = [];
        timer.reset();
    }
}

onTimer() {
    if (batch.length > 0) {
        sendBatch(batch);
        batch = [];
    }
}
```

### Lazy Evaluation

```
// Don't compute until needed
result = createLazyComputation(() => expensiveFunction());

// Only computed when accessed
value = result.getValue();
```

## Error Handling

### Retry with Exponential Backoff

```
maxRetries = 5
baseDelay = 1000

for (attempt = 0; attempt < maxRetries; attempt++) {
    try {
        return await operation();
    } catch (error) {
        delay = baseDelay * Math.pow(2, attempt);
        await sleep(delay);
    }
}
throw new Error('Max retries exceeded');
```

### Circuit Breaker

```
State: CLOSED | OPEN | HALF_OPEN

CLOSED:  Normal operation, count failures
         → OPEN after threshold failures

OPEN:    Reject requests immediately
         → HALF_OPEN after timeout

HALF_OPEN: Try limited requests
           → CLOSED if success
           → OPEN if failure
```

## Monitoring Protocol

### Health Check

**Endpoint**: `GET /health`

```json
{
  "status": "healthy | degraded | unhealthy",
  "timestamp": "2025-12-25T10:00:00Z",
  "uptime": 86400,
  "metrics": {
    "tasksCompleted": 1000,
    "avgResponseTime": 250,
    "errorRate": 0.01
  }
}
```

### Heartbeat

Periodic status updates (every 30s):

```json
{
  "type": "heartbeat",
  "agentId": "agent-001",
  "status": "active",
  "load": 0.75,
  "timestamp": "2025-12-25T10:00:00Z"
}
```

## Task Lifecycle State Machine

Every accepted task MUST progress through one of the following terminal states. State transitions MUST be reflected in the SSE stream defined in Phase 2 §Streaming.

```
                  +----------+
                  | submitted|
                  +-----+----+
                        |
                        v
                  +-----+----+
                  | accepted |
                  +--+----+--+
                     |    |
            (assign) |    | (refuse)
                     v    v
              +------+--+ +-------+
              | working | | failed|
              +--+---+--+ +-------+
                 |   |
        (need)   |   | (await peer)
                 v   v
        +-------+--+ +----------+
        | inputs   | | awaiting |
        |  needed  | |  agent   |
        +----+-----+ +----+-----+
             |            |
             +------+-----+
                    v
              +-----+-----+
              | completed |
              +-----------+
```

| State | Description | Allowed transitions |
|-------|-------------|---------------------|
| `submitted` | Task accepted by manager, not yet assigned | `accepted`, `rejected` |
| `accepted` | Manager has chosen at least one contractor | `working`, `failed` |
| `working` | Contractor executing | `inputs-needed`, `awaiting-agent`, `completed`, `failed`, `cancelled` |
| `inputs-needed` | Task is paused awaiting client clarification | `working`, `cancelled` |
| `awaiting-agent` | Sub-task delegated to another agent | `working`, `failed` |
| `completed` | Result delivered, idempotent reads available | terminal |
| `failed` | Permanent failure with diagnostic | terminal |
| `cancelled` | Cancelled by client or supervisor | terminal |

Each transition MUST be reported as an event whose `from` and `to` fields are members of the table above. Receivers MUST treat unknown transitions as `failed`.

## JSON-RPC 2.0 Envelope

Bilateral agent-to-agent invocation reuses JSON-RPC 2.0. Beyond the MCP-aligned `tools/*` methods, WIA-AI-016 reserves the namespace `wia/agent/*`:

| Method | Direction | Description |
|--------|-----------|-------------|
| `wia/agent/describe` | client → server | Returns the Agent Card |
| `wia/agent/ping` | client → server | Liveness probe |
| `wia/agent/cancel` | client → server | Cancels a running task |
| `wia/agent/tasks/get` | client → server | Reads task state |
| `wia/agent/tasks/stream` | client → server | Subscribes to lifecycle events |
| `wia/agent/notify` | server → client | Server-pushed event |

Notifications (no `id`) MUST be used for all server-pushed events; clients MUST tolerate unknown notifications without erroring.

## Trust Establishment

Pairs of agents that have not previously interacted MUST run the trust handshake before exchanging anything beyond `wia/agent/describe`:

```
Initiator                       Responder
   |                                |
   |--TLS 1.3 ClientHello---------->|
   |<-TLS 1.3 ServerHello-----------|
   |   (cert chain incl. SPIFFE ID) |
   |                                |
   |--POST /trust/initiate--------->|
   |   { nonce, agentCard }         |
   |<-200 OK { challenge }----------|
   |                                |
   |--POST /trust/prove------------>|
   |   { sig(challenge, privKey) }  |
   |<-200 OK { token, ttl }---------|
```

Tokens MUST be JWTs whose `iss` matches the responder's Agent Card `provider.url`, `aud` matches the initiator agent ID, and `exp` is ≤ 1 hour in the future. Verification MUST follow RFC 7515 (JWS) and RFC 7519 (JWT). Mutual TLS endpoints MAY rely on SPIFFE Verifiable Identity Documents (SVIDs) instead of JWTs.

## Rate Limit Signaling

When an agent must throttle a peer it MUST respond per RFC 6585 §4 with HTTP 429 and the headers:

```
Retry-After: 30
WIA-RateLimit-Limit: 100
WIA-RateLimit-Remaining: 0
WIA-RateLimit-Reset: 1735128030
WIA-RateLimit-Policy: "100;w=60"
```

Clients MUST honour `Retry-After` and SHOULD apply jitter (full-jitter exponential backoff) before retrying.

## Conflict Resolution

When two agents independently mutate replicated state (e.g. shared blackboard tuples), conflicts MUST be reconciled with the following precedence:

1. **Vector clocks**: each replica maintains a per-writer counter. The merged value is the pair-wise max of vectors.
2. **Last-writer-wins fallback**: if vectors are concurrent (`A ‖ B`), the writer with the lexicographically smallest `agentId` wins.
3. **Application override**: applications MAY register a `merge(a, b) -> c` callback that supersedes step 2.

Reconciliation events MUST be logged with `event.type = "conflict-resolved"` and include both originals plus the merged value.

## Saga-Style Multi-Agent Transactions

For workflows that span multiple agents WIA-AI-016 RECOMMENDS the saga pattern: each step has an inverse compensation that is invoked on rollback.

```json
{
  "sagaId": "saga-001",
  "steps": [
    {"agent": "agent-payment", "action": "charge", "compensate": "refund"},
    {"agent": "agent-inventory", "action": "reserve", "compensate": "release"},
    {"agent": "agent-shipping", "action": "dispatch", "compensate": "recall"}
  ],
  "policy": "abort-on-first-failure"
}
```

Coordinators MUST persist the saga log before invoking each forward step so that crash recovery can resume compensation. Compensations MUST be idempotent.

## Failure Detection

Agents that maintain peer liveness views MUST use a phi-accrual style failure detector. Recommended parameters: window size 1000 samples, threshold φ = 8, sampling interval 1 s. Suspect peers MUST be moved to `degraded` for 30 s before being declared `offline`; this prevents flapping under transient packet loss.

## Time Synchronisation

All timestamps in protocol events MUST be UTC formatted per ISO 8601 with millisecond precision and the `Z` designator. Clocks SHOULD be disciplined via NTP (RFC 5905) or PTP (IEEE 1588) with drift below 250 ms across the agent fleet. Receivers MUST tolerate clock skew up to 5 minutes for tokens (`exp`/`nbf`) per RFC 7519.

## Conformance Test Vectors

Phase 3 ships a vector file at `cli/test-vectors/phase-3.json` whose entries take the form:

```json
{
  "name": "request-protocol/happy-path",
  "trace": [
    {"from": "initiator", "to": "participant", "performative": "request", "content": {"action": "ping"}},
    {"from": "participant", "to": "initiator", "performative": "agree"},
    {"from": "participant", "to": "initiator", "performative": "inform", "content": {"result": "pong"}}
  ],
  "expected": {"final": "completed"}
}
```

Implementations MUST pass every vector with no warnings before claiming WIA-AI-016 Phase 3 conformance.

## Normative References

- JSON-RPC 2.0 — JSON-RPC Working Group
- IETF RFC 8446 — TLS 1.3
- IETF RFC 7515 — JSON Web Signature (JWS)
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6585 — Additional HTTP Status Codes
- IETF RFC 5905 — Network Time Protocol Version 4
- IEEE 1588-2019 — Precision Time Protocol
- ISO 8601:2019 — Date and time representations
- SPIFFE — Secure Production Identity Framework for Everyone

---

**WIA-AI-016 Phase 3 Specification v1.0**
© 2025 SmileStory Inc. / World Certification Industry Association
弘益人間 (홍익인간) · Benefit All Humanity
