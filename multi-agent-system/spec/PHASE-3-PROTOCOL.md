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

---

**WIA-AI-016 Phase 3 Specification v1.0**
© 2025 SmileStory Inc. / World Certification Industry Association
弘益人間 (홍익인간) · Benefit All Humanity
