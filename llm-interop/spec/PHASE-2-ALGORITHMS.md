# WIA-LLM-INTEROP: Phase 2 - Algorithms Specification

**Version**: 1.0.0
**Date**: 2025-12-16
**Status**: Draft
**Philosophy**: 홍익인간 (弘益人間) - Benefit All Humanity

---

## 1. Overview

This document defines the core algorithms for WIA-LLM-INTEROP:

| Algorithm | Purpose |
|-----------|---------|
| Capability Matching | Find AI with required capabilities |
| Capability Negotiation | Exchange capabilities during handshake |
| Message Routing | Route messages to appropriate AI |
| Task Decomposition | Break complex tasks into subtasks |
| Consensus | Reach agreement among multiple AIs |
| Trust Scoring | Evaluate AI reliability |
| Load Balancing | Distribute work efficiently |
| Circuit Breaking | Handle failures gracefully |

---

## 2. Capability Matching Algorithm

### 2.1 Match Score Calculation

```
Algorithm: CalculateCapabilityMatch
Input: query (CapabilityQuery), candidate (CapabilityDocument)
Output: score (0.0 - 1.0), eligible (boolean)

1. Initialize scores:
   domain_score = 0.0
   language_score = 0.0
   level_score = 0.0
   tool_score = 0.0

2. Check hard requirements (must pass all):
   IF query.min_level AND candidate.level < query.min_level:
       RETURN (0.0, false)

   FOR EACH required_domain IN query.required_domains:
       IF NOT candidate.domains.contains(required_domain):
           RETURN (0.0, false)

   FOR EACH required_tool IN query.required_tools:
       IF NOT candidate.tools.contains(required_tool):
           RETURN (0.0, false)

3. Calculate domain score:
   FOR EACH required_domain IN query.required_domains:
       candidate_domain = candidate.domains.find(required_domain)
       domain_score += candidate_domain.proficiency
   domain_score /= query.required_domains.length

4. Calculate language score:
   FOR EACH required_lang IN query.required_languages:
       candidate_lang = candidate.languages.find(required_lang)
       IF candidate_lang:
           language_score += candidate_lang.proficiency
       ELSE:
           language_score += 0.0
   language_score /= query.required_languages.length

5. Calculate level score:
   level_score = min(candidate.level / 4.0, 1.0)

6. Calculate tool score:
   matched_tools = 0
   FOR EACH required_tool IN query.required_tools:
       IF candidate.tools.contains(required_tool):
           matched_tools += 1
   tool_score = matched_tools / query.required_tools.length

7. Weighted final score:
   weights = {
       domain: 0.40,
       language: 0.20,
       level: 0.25,
       tool: 0.15
   }

   final_score = (
       domain_score * weights.domain +
       language_score * weights.language +
       level_score * weights.level +
       tool_score * weights.tool
   )

8. RETURN (final_score, true)
```

### 2.2 Best Match Selection

```
Algorithm: SelectBestMatch
Input: query (CapabilityQuery), candidates (CapabilityDocument[])
Output: best_candidate, score

1. scored_candidates = []

2. FOR EACH candidate IN candidates:
   (score, eligible) = CalculateCapabilityMatch(query, candidate)
   IF eligible:
       scored_candidates.append((candidate, score))

3. IF scored_candidates.isEmpty():
   RETURN (null, 0.0)

4. Sort scored_candidates by score DESC

5. RETURN scored_candidates[0]
```

### 2.3 Multi-Select for Federation

```
Algorithm: SelectFederationMembers
Input:
    task (TaskDefinition),
    candidates (CapabilityDocument[]),
    max_members (integer)
Output: selected_members[]

1. Decompose task into capability requirements:
   requirements = DecomposeTaskRequirements(task)

2. selected = []

3. FOR EACH requirement IN requirements:
   IF selected.length >= max_members:
       BREAK

   # Find best candidate for this requirement
   remaining_candidates = candidates.exclude(selected)
   (best, score) = SelectBestMatch(requirement, remaining_candidates)

   IF best AND score > 0.5:
       selected.append({
           candidate: best,
           role: requirement.suggested_role,
           score: score
       })

4. RETURN selected
```

---

## 3. Capability Negotiation Algorithm

### 3.1 Handshake Protocol

```
Algorithm: CapabilityHandshake
Participants: initiator (AI), responder (AI)
Output: negotiated_session

State Machine:
    INIT -> CAPABILITY_EXCHANGE -> NEGOTIATION -> AUTH -> COMPLETE

1. INIT Phase:
   initiator sends HandshakePayload:
       phase: "init"
       supported_versions: ["1.0"]
       preferred_encoding: "utf-8"

   responder validates, sends ACK

2. CAPABILITY_EXCHANGE Phase:
   initiator sends:
       phase: "capability_exchange"
       capability: initiator.capability_document

   responder sends:
       phase: "capability_exchange"
       capability: responder.capability_document

3. NEGOTIATION Phase:
   Both parties calculate intersection:

   negotiated = {
       version: selectVersion(i.versions, r.versions),
       encoding: selectEncoding(i.encoding, r.encoding),
       compression: selectCompression(i.compression, r.compression),
       features: intersect(i.features, r.features)
   }

   IF negotiated is valid:
       both send: phase: "negotiation", negotiated: {...}

4. AUTH Phase (if required):
   Exchange authentication tokens/signatures

5. COMPLETE Phase:
   Both send: phase: "complete"
   Session established
```

### 3.2 Version Selection

```
Algorithm: SelectVersion
Input: versions_a[], versions_b[]
Output: selected_version or null

1. common = intersect(versions_a, versions_b)

2. IF common.isEmpty():
   RETURN null

3. Sort common by semantic version DESC

4. RETURN common[0]  # Highest common version
```

### 3.3 Feature Intersection

```
Algorithm: NegotiateFeatures
Input: features_a (ModalitySupport), features_b (ModalitySupport)
Output: negotiated_features

1. negotiated = {}

2. FOR EACH modality IN ["text", "image", "audio", "code"]:
   a = features_a[modality]
   b = features_b[modality]

   IF a AND b:
       negotiated[modality] = {
           input: a.input AND b.input,
           output: a.output AND b.output,
           max_size: min(a.max_size, b.max_size)
       }

3. RETURN negotiated
```

---

## 4. Message Routing Algorithm

### 4.1 Direct Routing

```
Algorithm: DirectRoute
Input: message (MessageEnvelope)
Output: destination_endpoint

1. IF message.to.model_id:
   # Direct addressing
   endpoint = registry.lookup(message.to.model_id)
   IF endpoint:
       RETURN endpoint
   ELSE:
       THROW E603_NODE_UNAVAILABLE

2. RETURN null  # Not direct routing
```

### 4.2 Capability-Based Routing

```
Algorithm: CapabilityRoute
Input: message (MessageEnvelope)
Output: destination_endpoint

1. query = message.to.capability_query

2. candidates = registry.getActiveNodes()

3. (best_match, score) = SelectBestMatch(query, candidates)

4. IF best_match AND score > 0.5:
   RETURN best_match.endpoint
ELSE:
   THROW E100_CAPABILITY_MISMATCH
```

### 4.3 Load-Balanced Routing

```
Algorithm: LoadBalancedRoute
Input:
    message (MessageEnvelope),
    strategy (LoadBalanceStrategy)
Output: destination_endpoint

Strategies:
    ROUND_ROBIN, LEAST_CONNECTIONS, WEIGHTED, RANDOM

1. candidates = FindEligibleCandidates(message.to)

2. IF candidates.isEmpty():
   THROW E603_NODE_UNAVAILABLE

3. SWITCH strategy:
   CASE ROUND_ROBIN:
       index = (last_index + 1) % candidates.length
       selected = candidates[index]

   CASE LEAST_CONNECTIONS:
       selected = candidates.minBy(c => c.active_connections)

   CASE WEIGHTED:
       # Weight by capability score and availability
       weights = candidates.map(c => {
           score = c.capability_score * c.availability_score
           RETURN (c, score)
       })
       selected = weightedRandomSelect(weights)

   CASE RANDOM:
       selected = randomSelect(candidates)

4. RETURN selected.endpoint
```

### 4.4 Broadcast Routing

```
Algorithm: BroadcastRoute
Input: message (MessageEnvelope)
Output: destinations[]

1. scope = message.to.broadcast

2. SWITCH scope:
   CASE FEDERATION:
       RETURN federation.getAllMembers()

   CASE DOMAIN:
       domain = message.metadata.domain
       RETURN registry.getByDomain(domain)

   CASE ALL:
       RETURN registry.getAllActive()
```

---

## 5. Task Decomposition Algorithm

### 5.1 Automatic Task Decomposition

```
Algorithm: DecomposeTask
Input: task (TaskDefinition)
Output: subtasks[]

1. Analyze task complexity:
   complexity = AnalyzeComplexity(task.original_query)

2. IF complexity < THRESHOLD_SIMPLE:
   RETURN [task]  # No decomposition needed

3. Extract task components:
   components = ExtractComponents(task.original_query)

   Components may include:
   - Multiple questions
   - Multiple domains
   - Sequential dependencies
   - Parallel-executable parts

4. FOR EACH component IN components:
   subtask = {
       task_id: generateUUID(),
       parent_task_id: task.task_id,
       description: component.description,
       required_capability: InferCapability(component),
       dependencies: component.dependencies,
       priority: component.priority
   }
   subtasks.append(subtask)

5. ValidateDependencyGraph(subtasks)

6. RETURN subtasks
```

### 5.2 Complexity Analysis

```
Algorithm: AnalyzeComplexity
Input: query (string)
Output: complexity_score (0.0 - 1.0)

Factors:
1. query_length_factor = min(len(query) / 1000, 1.0) * 0.2

2. domain_count = CountDistinctDomains(query)
   domain_factor = min(domain_count / 3, 1.0) * 0.3

3. question_count = CountQuestions(query)
   question_factor = min(question_count / 5, 1.0) * 0.25

4. dependency_indicators = CountDependencyKeywords(query)
   # Keywords: "then", "after", "based on", "using the result"
   dependency_factor = min(dependency_indicators / 3, 1.0) * 0.25

5. complexity = (
       query_length_factor +
       domain_factor +
       question_factor +
       dependency_factor
   )

6. RETURN complexity
```

### 5.3 Dependency Graph Validation

```
Algorithm: ValidateDependencyGraph
Input: subtasks[]
Output: valid (boolean), errors[]

1. Build directed graph:
   graph = DirectedGraph()
   FOR EACH task IN subtasks:
       graph.addNode(task.task_id)
       FOR EACH dep IN task.dependencies:
           graph.addEdge(dep, task.task_id)

2. Check for cycles:
   IF graph.hasCycle():
       RETURN (false, ["Circular dependency detected"])

3. Check all dependencies exist:
   FOR EACH task IN subtasks:
       FOR EACH dep IN task.dependencies:
           IF NOT subtasks.contains(dep):
               RETURN (false, ["Missing dependency: " + dep])

4. RETURN (true, [])
```

### 5.4 Task Scheduling (Topological Sort)

```
Algorithm: ScheduleTasks
Input: subtasks[]
Output: execution_order[]

1. graph = BuildDependencyGraph(subtasks)

2. in_degree = {} # Count of incoming edges
   FOR EACH task IN subtasks:
       in_degree[task.task_id] = task.dependencies.length

3. queue = []
   FOR EACH task IN subtasks:
       IF in_degree[task.task_id] == 0:
           queue.enqueue(task)

4. execution_order = []
   parallel_batches = []
   current_batch = []

5. WHILE queue is not empty:
   # All tasks in queue can run in parallel
   current_batch = queue.drain()
   parallel_batches.append(current_batch)

   next_queue = []
   FOR EACH completed_task IN current_batch:
       execution_order.append(completed_task)
       FOR EACH dependent IN graph.dependents(completed_task):
           in_degree[dependent] -= 1
           IF in_degree[dependent] == 0:
               next_queue.append(dependent)
   queue = next_queue

6. RETURN {
       linear_order: execution_order,
       parallel_batches: parallel_batches
   }
```

---

## 6. Consensus Algorithms

### 6.1 Majority Vote

```
Algorithm: MajorityConsensus
Input:
    question (ConsensusQuestion),
    votes (ConsensusVote[])
Output: ConsensusResult

1. Count votes:
   vote_counts = {}
   FOR EACH vote IN votes:
       key = vote.vote
       vote_counts[key] = (vote_counts[key] || 0) + 1

2. Find majority:
   total_votes = votes.length
   FOR EACH (option, count) IN vote_counts:
       IF count > total_votes / 2:
           RETURN {
               outcome: option,
               confidence: count / total_votes,
               vote_count: total_votes,
               quorum_reached: true,
               vote_breakdown: vote_counts
           }

3. # No majority - return plurality
   plurality = max(vote_counts, key=count)
   RETURN {
       outcome: plurality.option,
       confidence: plurality.count / total_votes,
       vote_count: total_votes,
       quorum_reached: false,
       vote_breakdown: vote_counts
   }
```

### 6.2 Weighted Vote

```
Algorithm: WeightedConsensus
Input:
    question (ConsensusQuestion),
    votes (ConsensusVote[]),
    weights (Map<model_id, weight>)
Output: ConsensusResult

1. Calculate weighted scores:
   weighted_scores = {}

   FOR EACH vote IN votes:
       voter_weight = weights[vote.voter_id] || 1.0
       confidence_factor = vote.confidence
       effective_weight = voter_weight * confidence_factor

       key = vote.vote
       weighted_scores[key] = (weighted_scores[key] || 0) + effective_weight

2. Total weight:
   total_weight = sum(weighted_scores.values())

3. Find winner:
   winner = max(weighted_scores, key=score)

   RETURN {
       outcome: winner.option,
       confidence: winner.score / total_weight,
       vote_count: votes.length,
       quorum_reached: winner.score > total_weight * 0.5,
       vote_breakdown: weighted_scores
   }
```

### 6.3 Debate Protocol

```
Algorithm: DebateConsensus
Input:
    question (ConsensusQuestion),
    participants (AI[]),
    moderator (AI),
    max_rounds (integer)
Output: ConsensusResult

1. Initial positions:
   positions = {}
   FOR EACH ai IN participants:
       response = ai.generateResponse(question)
       positions[ai.model_id] = {
           position: response.content,
           confidence: response.confidence,
           reasoning: response.reasoning
       }

2. Debate rounds:
   FOR round = 1 TO max_rounds:
       # Each AI can respond to others
       FOR EACH ai IN participants:
           other_positions = positions.exclude(ai.model_id)
           rebuttal = ai.generateRebuttal(question, other_positions)

           IF rebuttal.changed_position:
               positions[ai.model_id] = rebuttal.new_position

       # Check for convergence
       IF AllPositionsAgree(positions):
           BREAK

3. Final decision by moderator:
   final = moderator.decide(question, positions)

   RETURN {
       outcome: final.decision,
       confidence: final.confidence,
       vote_count: participants.length,
       reasoning: final.reasoning,
       debate_transcript: positions
   }
```

### 6.4 RAFT-inspired Consensus (for critical decisions)

```
Algorithm: RaftConsensus
Input:
    proposal (ConsensusQuestion),
    nodes (FederationMember[])
Output: ConsensusResult

Terms:
    - Leader: Node that proposes
    - Follower: Node that votes
    - Candidate: Node seeking leadership

1. Leader Election (if needed):
   IF no current_leader OR leader_timeout:
       candidate = self
       term += 1
       votes_received = 1  # Vote for self

       FOR EACH node IN nodes.exclude(self):
           response = node.requestVote(term, self.model_id)
           IF response.vote_granted:
               votes_received += 1

       IF votes_received > nodes.length / 2:
           current_leader = self

2. Proposal Phase:
   leader sends AppendEntries:
       term: current_term
       proposal: proposal
       leader_id: leader.model_id

3. Commit Phase:
   acknowledgments = 0
   FOR EACH response IN responses:
       IF response.success:
           acknowledgments += 1

   IF acknowledgments > nodes.length / 2:
       # Commit the decision
       RETURN {
           outcome: proposal.outcome,
           confidence: acknowledgments / nodes.length,
           quorum_reached: true
       }

4. RETURN failure if quorum not reached
```

---

## 7. Trust Scoring Algorithm

### 7.1 Calculate Trust Score

```
Algorithm: CalculateTrustScore
Input: ai (ModelIdentity), history (InteractionHistory)
Output: trust_score (0.0 - 1.0)

Factors:
1. Base reputation:
   base_score = GetBaseReputation(ai.provider)
   # Known providers: 0.7, Unknown: 0.3

2. Historical accuracy:
   IF history.total_interactions > 0:
       accuracy = history.successful / history.total_interactions
       accuracy_score = accuracy
   ELSE:
       accuracy_score = 0.5  # No history

3. Response quality:
   IF history.quality_ratings.length > 0:
       quality_score = average(history.quality_ratings)
   ELSE:
       quality_score = 0.5

4. Consistency:
   # How consistent are responses across similar queries
   consistency_score = CalculateConsistency(history)

5. Verification status:
   IF ai.verification.wia_verified:
       verification_bonus = 0.1
   ELSE:
       verification_bonus = 0.0

6. Weighted combination:
   weights = {
       base: 0.20,
       accuracy: 0.35,
       quality: 0.25,
       consistency: 0.20
   }

   trust_score = (
       base_score * weights.base +
       accuracy_score * weights.accuracy +
       quality_score * weights.quality +
       consistency_score * weights.consistency +
       verification_bonus
   )

   trust_score = clamp(trust_score, 0.0, 1.0)

7. RETURN trust_score
```

### 7.2 Malicious AI Detection

```
Algorithm: DetectMaliciousAI
Input: ai (ModelIdentity), history (InteractionHistory)
Output: (is_malicious, confidence, reasons[])

Red Flags:
1. High error rate:
   error_rate = history.errors / history.total_interactions
   IF error_rate > 0.5:
       red_flags.append("High error rate: " + error_rate)

2. Inconsistent responses:
   consistency = CalculateConsistency(history)
   IF consistency < 0.3:
       red_flags.append("Inconsistent responses")

3. Confidence manipulation:
   # Claims high confidence but often wrong
   IF averageConfidence > 0.8 AND accuracyRate < 0.4:
       red_flags.append("Confidence inflation detected")

4. Response time anomalies:
   IF averageResponseTime significantly differs from claimed limits:
       red_flags.append("Response time anomaly")

5. Content pattern analysis:
   IF containsSuspiciousPatterns(history.responses):
       red_flags.append("Suspicious content patterns")

6. Decision:
   IF red_flags.length >= 3:
       RETURN (true, 0.8, red_flags)
   ELIF red_flags.length >= 2:
       RETURN (true, 0.5, red_flags)
   ELSE:
       RETURN (false, 0.0, [])
```

---

## 8. Load Balancing Algorithm

### 8.1 Dynamic Weight Calculation

```
Algorithm: CalculateNodeWeight
Input: node (FederationMember)
Output: weight (0.0 - 1.0)

1. Capacity factor:
   current_load = node.active_requests / node.max_concurrent
   capacity_score = 1.0 - current_load

2. Performance factor:
   avg_response_time = node.metrics.average_response_time_ms
   expected_response_time = 1000  # 1 second baseline
   performance_score = min(expected_response_time / avg_response_time, 1.0)

3. Reliability factor:
   uptime = node.metrics.uptime_percentage
   success_rate = node.metrics.success_rate
   reliability_score = (uptime + success_rate) / 2

4. Freshness factor:
   time_since_seen = now() - node.last_seen
   IF time_since_seen > 60 seconds:
       freshness_penalty = min(time_since_seen / 300, 1.0)
       freshness_score = 1.0 - freshness_penalty
   ELSE:
       freshness_score = 1.0

5. Combined weight:
   weight = (
       capacity_score * 0.35 +
       performance_score * 0.25 +
       reliability_score * 0.30 +
       freshness_score * 0.10
   )

6. RETURN weight
```

### 8.2 Adaptive Load Balancing

```
Algorithm: AdaptiveLoadBalance
Input:
    message (MessageEnvelope),
    nodes (FederationMember[]),
    history (LoadBalanceHistory)
Output: selected_node

1. Calculate current weights:
   weighted_nodes = []
   FOR EACH node IN nodes:
       weight = CalculateNodeWeight(node)
       weighted_nodes.append((node, weight))

2. Apply historical adjustments:
   FOR EACH (node, weight) IN weighted_nodes:
       recent_failures = history.getRecentFailures(node.model_id)
       IF recent_failures > 0:
           penalty = min(recent_failures * 0.1, 0.5)
           weight = weight * (1.0 - penalty)

3. Normalize weights:
   total_weight = sum(w for (_, w) in weighted_nodes)
   normalized = [(n, w/total_weight) for (n, w) in weighted_nodes]

4. Probabilistic selection:
   random_value = random(0.0, 1.0)
   cumulative = 0.0
   FOR EACH (node, weight) IN normalized:
       cumulative += weight
       IF random_value <= cumulative:
           RETURN node

5. Fallback to first node
   RETURN normalized[0].node
```

---

## 9. Circuit Breaker Algorithm

### 9.1 Circuit Breaker State Machine

```
States: CLOSED, OPEN, HALF_OPEN

Algorithm: CircuitBreaker
Input: target (node), config (CircuitBreakerConfig)

State transitions:
    CLOSED --[failures >= threshold]--> OPEN
    OPEN --[timeout elapsed]--> HALF_OPEN
    HALF_OPEN --[success]--> CLOSED
    HALF_OPEN --[failure]--> OPEN
```

### 9.2 Circuit Breaker Implementation

```
Algorithm: ExecuteWithCircuitBreaker
Input:
    target (node),
    request (MessageEnvelope),
    config (CircuitBreakerConfig)
Output: response or error

1. Get circuit state:
   circuit = GetCircuit(target.model_id)

2. Check state:
   SWITCH circuit.state:
       CASE OPEN:
           IF now() - circuit.last_failure < config.timeout_seconds:
               THROW E503_CIRCUIT_OPEN
           ELSE:
               circuit.state = HALF_OPEN

       CASE HALF_OPEN:
           IF circuit.half_open_requests >= config.half_open_requests:
               THROW E503_CIRCUIT_OPEN

3. Execute request:
   TRY:
       response = target.send(request)

       # Success
       IF circuit.state == HALF_OPEN:
           circuit.success_count += 1
           IF circuit.success_count >= config.success_threshold:
               circuit.state = CLOSED
               circuit.failure_count = 0
       ELSE:
           circuit.failure_count = max(0, circuit.failure_count - 1)

       RETURN response

   CATCH error:
       # Failure
       circuit.failure_count += 1
       circuit.last_failure = now()

       IF circuit.state == HALF_OPEN:
           circuit.state = OPEN
       ELIF circuit.failure_count >= config.failure_threshold:
           circuit.state = OPEN

       THROW error
```

---

## 10. Response Aggregation Algorithm

### 10.1 Simple Aggregation

```
Algorithm: AggregateResponses
Input: responses (ResponsePayload[])
Output: aggregated (ResponsePayload)

1. IF responses.length == 1:
   RETURN responses[0]

2. Combine content:
   aggregated_content = ""
   FOR EACH (i, response) IN enumerate(responses):
       aggregated_content += f"[Source {i+1}]: {response.content}\n\n"

3. Average confidence:
   avg_confidence = average(r.confidence for r in responses)

4. Merge sources:
   all_sources = flatten(r.sources for r in responses)
   unique_sources = deduplicate(all_sources)

5. Merge suggestions:
   all_suggestions = flatten(r.suggestions for r in responses)
   ranked_suggestions = rank_by_relevance(all_suggestions)[:5]

6. Aggregate usage:
   total_usage = {
       input_tokens: sum(r.usage.input_tokens for r in responses),
       output_tokens: sum(r.usage.output_tokens for r in responses),
       processing_time_ms: max(r.usage.processing_time_ms for r in responses)
   }

7. RETURN {
       content: aggregated_content,
       confidence: avg_confidence,
       sources: unique_sources,
       suggestions: ranked_suggestions,
       usage: total_usage
   }
```

### 10.2 Intelligent Synthesis

```
Algorithm: SynthesizeResponses
Input:
    responses (ResponsePayload[]),
    synthesizer (AI)
Output: synthesized (ResponsePayload)

1. Prepare synthesis prompt:
   prompt = f"""
   Multiple AI systems have provided the following responses to the same query.
   Please synthesize these into a single, coherent response.

   {format_responses(responses)}

   Provide:
   1. A unified response that captures the key insights
   2. Note any disagreements between sources
   3. Your confidence in the synthesized response
   """

2. Generate synthesis:
   synthesis = synthesizer.generate(prompt)

3. Calculate combined confidence:
   # Weight by individual confidences and agreement
   agreement_score = CalculateAgreementScore(responses)
   synthesized_confidence = synthesis.confidence * agreement_score

4. RETURN {
       content: synthesis.content,
       confidence: synthesized_confidence,
       sources: [...all sources, {title: "AI Synthesis", ...}],
       reasoning: synthesis.reasoning
   }
```

### 10.3 Agreement Score Calculation

```
Algorithm: CalculateAgreementScore
Input: responses (ResponsePayload[])
Output: score (0.0 - 1.0)

1. IF responses.length < 2:
   RETURN 1.0

2. Extract key claims from each response:
   all_claims = []
   FOR EACH response IN responses:
       claims = ExtractClaims(response.content)
       all_claims.append(claims)

3. Calculate pairwise similarity:
   similarities = []
   FOR i = 0 TO responses.length - 2:
       FOR j = i + 1 TO responses.length - 1:
           sim = CosineSimilarity(all_claims[i], all_claims[j])
           similarities.append(sim)

4. Average similarity:
   agreement = average(similarities)

5. RETURN agreement
```

---

## 11. Pseudocode Notation

| Symbol | Meaning |
|--------|---------|
| `FOR EACH x IN collection` | Iterate over collection |
| `IF condition:` | Conditional block |
| `SWITCH x:` | Multi-way branch |
| `RETURN value` | Return from algorithm |
| `THROW error` | Raise exception |
| `TRY: ... CATCH:` | Exception handling |
| `min(a, b)` | Minimum of a and b |
| `max(a, b)` | Maximum of a and b |
| `clamp(x, lo, hi)` | Constrain x between lo and hi |
| `average(collection)` | Mean of values |
| `sum(collection)` | Sum of values |

---

**Document ID**: WIA-LLM-INTEROP-PHASE-2
**Version**: 1.0.0
**Date**: 2025-12-16
**Philosophy**: 홍익인간 (弘益人間) - 알고리즘으로 AI들이 협력합니다
