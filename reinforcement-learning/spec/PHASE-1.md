# WIA-AI-025 Reinforcement Learning - PHASE 1 Specification

## Overview

**Status:** ✅ ACTIVE
**Version:** 1.0.0
**Last Updated:** 2025-01-20
**Maintainer:** WIA Standards Committee

## Philosophy

**弘益人間 (홍익인간) · Benefit All Humanity**

Reinforcement Learning enables agents to learn optimal behavior through interaction, democratizing AI capabilities for all.

## Scope

PHASE 1 focuses on foundational reinforcement learning algorithms and environments suitable for education, research, and simple production applications.

## Core Components

### 1.1 Basic RL Algorithms

#### Q-Learning
- **Type:** Model-free, off-policy, value-based
- **State Space:** Discrete
- **Action Space:** Discrete
- **Complexity:** O(|S| × |A|)

**Algorithm:**
```python
Q(s,a) ← Q(s,a) + α[r + γ max_a' Q(s',a') - Q(s,a)]
```

**Requirements:**
- ✅ Tabular Q-table implementation
- ✅ ε-greedy exploration
- ✅ Learning rate decay
- ✅ Convergence guarantees for finite MDPs

#### SARSA
- **Type:** Model-free, on-policy, value-based
- **State Space:** Discrete
- **Action Space:** Discrete

**Algorithm:**
```python
Q(s,a) ← Q(s,a) + α[r + γ Q(s',a') - Q(s,a)]
```

**Differences from Q-Learning:**
- On-policy: learns value of policy being followed
- More conservative (safer during training)
- Better for stochastic environments

#### Monte Carlo Methods
- **Type:** Model-free
- **Update:** Episode-based
- **Variants:** First-visit MC, Every-visit MC

**Requirements:**
- ✅ Complete episode collection
- ✅ Return calculation with discount factor
- ✅ Incremental mean updates
- ✅ Exploring starts or ε-greedy

### 1.2 Environment Interface

#### OpenAI Gym Compatibility

```python
class Environment:
    def reset(self) -> State:
        """Reset environment to initial state."""
        pass

    def step(self, action: Action) -> Tuple[State, float, bool, dict]:
        """
        Execute action and return:
        - next_state: New state after action
        - reward: Reward received
        - done: Whether episode terminated
        - info: Additional information
        """
        pass

    def render(self, mode='human') -> None:
        """Visualize environment."""
        pass

    @property
    def action_space(self) -> Space:
        """Define action space."""
        pass

    @property
    def observation_space(self) -> Space:
        """Define observation space."""
        pass
```

#### Standard Environments

1. **GridWorld** (4x4, 8x8, 16x16)
   - Discrete states and actions
   - Deterministic or stochastic transitions
   - Customizable rewards and obstacles

2. **FrozenLake**
   - 4x4 or 8x8 grid
   - Slippery surfaces (stochastic)
   - Goal: reach target without falling

3. **CliffWalking**
   - Grid navigation with dangerous cliff
   - Tests safety vs optimality tradeoff
   - Demonstrates Q-Learning vs SARSA difference

4. **CartPole**
   - Balance pole on cart
   - Continuous state, discrete action
   - Classic control problem

### 1.3 Value Functions

#### State Value Function V(s)
```python
V^π(s) = E_π[G_t | S_t = s]
       = E_π[R_{t+1} + γR_{t+2} + γ²R_{t+3} + ... | S_t = s]
```

#### Action Value Function Q(s,a)
```python
Q^π(s,a) = E_π[G_t | S_t = s, A_t = a]
         = E_π[R_{t+1} + γQ^π(S_{t+1}, A_{t+1}) | S_t = s, A_t = a]
```

#### Optimal Value Functions
```python
V*(s) = max_π V^π(s)
Q*(s,a) = max_π Q^π(s,a)
```

### 1.4 Exploration Strategies

#### 1. ε-Greedy
```python
def epsilon_greedy(Q, state, epsilon=0.1):
    if random() < epsilon:
        return random_action()
    else:
        return argmax(Q[state])
```

**Parameters:**
- `epsilon`: Exploration probability (typical: 0.1)
- Decay schedule: `epsilon = max(epsilon_min, epsilon * decay_rate)`

#### 2. Boltzmann Exploration
```python
def boltzmann(Q, state, temperature=1.0):
    exp_values = exp(Q[state] / temperature)
    probs = exp_values / sum(exp_values)
    return sample(probs)
```

#### 3. Upper Confidence Bound (UCB)
```python
def ucb(Q, N, state, c=2.0):
    total_visits = sum(N[state])
    ucb_values = Q[state] + c * sqrt(log(total_visits) / N[state])
    return argmax(ucb_values)
```

## Implementation Requirements

### 1.5 Training Loop

```python
def train_rl_agent(env, agent, episodes=1000):
    """
    Standard RL training loop.

    Args:
        env: Environment instance
        agent: RL agent (Q-Learning, SARSA, etc.)
        episodes: Number of training episodes

    Returns:
        agent: Trained agent
        metrics: Training metrics (rewards, steps, etc.)
    """
    episode_rewards = []
    episode_lengths = []

    for episode in range(episodes):
        state = env.reset()
        total_reward = 0
        steps = 0
        done = False

        while not done:
            # Select action
            action = agent.select_action(state)

            # Execute action
            next_state, reward, done, info = env.step(action)

            # Update agent
            agent.update(state, action, reward, next_state, done)

            state = next_state
            total_reward += reward
            steps += 1

        episode_rewards.append(total_reward)
        episode_lengths.append(steps)

        # Logging
        if (episode + 1) % 100 == 0:
            avg_reward = np.mean(episode_rewards[-100:])
            print(f"Episode {episode+1}, Avg Reward: {avg_reward:.2f}")

    return agent, {
        'rewards': episode_rewards,
        'lengths': episode_lengths
    }
```

### 1.6 Evaluation

```python
def evaluate_agent(env, agent, num_episodes=100):
    """
    Evaluate trained agent.

    Args:
        env: Environment instance
        agent: Trained agent
        num_episodes: Number of evaluation episodes

    Returns:
        metrics: Evaluation metrics
    """
    rewards = []
    success_rate = 0

    for episode in range(num_episodes):
        state = env.reset()
        total_reward = 0
        done = False

        while not done:
            action = agent.select_action(state, greedy=True)
            state, reward, done, info = env.step(action)
            total_reward += reward

        rewards.append(total_reward)
        if info.get('success', False):
            success_rate += 1

    return {
        'mean_reward': np.mean(rewards),
        'std_reward': np.std(rewards),
        'success_rate': success_rate / num_episodes
    }
```

## Performance Benchmarks

### GridWorld 8x8
- **Q-Learning:** ≥95% success rate within 5000 episodes
- **SARSA:** ≥90% success rate within 5000 episodes
- **Monte Carlo:** ≥85% success rate within 10000 episodes

### FrozenLake 8x8
- **Q-Learning:** ≥70% success rate within 20000 episodes
- **SARSA:** ≥65% success rate within 20000 episodes

### CartPole-v1
- **Q-Learning (discretized):** Average reward ≥195 within 500 episodes
- **DQN (PHASE 2):** Average reward ≥195 within 200 episodes

## API Specification

### Agent Interface

```typescript
interface RLAgent {
    // Select action given current state
    selectAction(state: State, greedy?: boolean): Action;

    // Update agent parameters
    update(
        state: State,
        action: Action,
        reward: number,
        nextState: State,
        done: boolean
    ): void;

    // Get current policy
    getPolicy(): Policy;

    // Get value function
    getValueFunction(): ValueFunction;

    // Save/load agent
    save(path: string): void;
    load(path: string): void;
}
```

### Configuration

```typescript
interface QLearningConfig {
    learningRate: number;      // α ∈ (0, 1], default: 0.1
    discountFactor: number;    // γ ∈ [0, 1], default: 0.99
    epsilon: number;           // ε ∈ [0, 1], default: 0.1
    epsilonDecay: number;      // default: 0.995
    epsilonMin: number;        // default: 0.01
}
```

## Testing Requirements

### Unit Tests
- ✅ Q-value updates correct
- ✅ Policy improvement verified
- ✅ Exploration-exploitation balance
- ✅ Convergence to optimal policy in simple environments

### Integration Tests
- ✅ Full training pipeline
- ✅ Evaluation metrics
- ✅ Save/load functionality
- ✅ Multi-environment compatibility

### Performance Tests
- ✅ Training time < 60 seconds for GridWorld 8x8
- ✅ Memory usage < 100MB for tabular methods
- ✅ Inference latency < 1ms

## Documentation Requirements

- ✅ Algorithm descriptions with pseudocode
- ✅ Code examples for each algorithm
- ✅ Environment setup guides
- ✅ Hyperparameter tuning guidelines
- ✅ Common pitfalls and solutions
- ✅ References to seminal papers

## Compliance

Implementations must:
1. Follow OpenAI Gym interface
2. Support standard numpy/torch serialization
3. Provide reproducibility (seed setting)
4. Include type hints (Python 3.7+)
5. Pass all unit and integration tests
6. Meet performance benchmarks

## References

1. Sutton, R. S., & Barto, A. G. (2018). *Reinforcement Learning: An Introduction* (2nd ed.)
2. Watkins, C. J., & Dayan, P. (1992). Q-learning. *Machine Learning*, 8(3-4), 279-292.
3. Rummery, G. A., & Niranjan, M. (1994). On-line Q-learning using connectionist systems.

## Changelog

### v1.0.0 (2025-01-20)
- Initial PHASE 1 specification
- Core algorithms: Q-Learning, SARSA, Monte Carlo
- Standard environments defined
- API specification complete

---

**Next:** [PHASE 2 - Deep Reinforcement Learning](./PHASE-2.md)

## Appendix — Reserved Tokens

Future WIA-AI-025 minor versions add tokens but never remove them. Implementations MUST treat unknown tokens as `unrecognised` rather than fail the envelope.

The `algorithm_family` field uses one of the registered RL algorithm families: `value_based`, `policy_gradient`, `actor_critic`, `model_based`, `inverse_rl`, `imitation_learning`, `multi_agent`, `meta_learning`, `offline_rl`, `safe_rl`. Future minor versions add tokens but never remove them. Implementations MUST treat unknown tokens as `unrecognised` rather than fail the envelope.

## Appendix — Data Format Implementation Notes

### Conformance test suite

A black-box test suite is published at `https://github.com/WIA-Official/wia-reinforcement-learning-conformance` and walks the public surface of this Phase. Hosts publishing `bridge_profile=Full` SHOULD additionally pass the suite extension tests for at least one supported third-party RL framework (Stable-Baselines3, RLlib, CleanRL, Tianshou, Acme).

### Reference container

The `wia/reinforcement-learning-host:1.0.0` container image implements every endpoint specified in this Phase with mock environments and pre-trained checkpoints. The container ships with built-in OpenAI Gymnasium environment compatibility shim suitable for development and testing.

### Operational considerations

Reinforcement-learning infrastructure has three operational concerns that integrators consistently underestimate. First, checkpoint storage scales aggressively with experiment frequency — a typical research lab generates 100 GB of checkpoints per week from active hyperparameter sweeps. Second, environment determinism is a property of the environment + the framework + the GPU driver stack; the standard's environment_descriptor envelope captures all three so reproducibility studies can pin the exact stack. Third, reward hacking detection — agents trained against poorly-specified reward functions exhibit pathological behaviour that the standard's evaluation_run envelope helps surface via the documented `safety_metrics` block.

### Backwards-compatibility promise

Within the 1.x line every endpoint and envelope listed in this Phase MUST remain reachable and MUST continue to honour the documented status codes and content shapes. Hosts MAY add optional query parameters, response fields, new endpoints, or media types. Hosts MUST NOT remove or repurpose existing ones. Breaking changes ride a major version bump and MUST be preceded by a 12-month deprecation window per IETF RFC 8594 and RFC 9745.

---

© 2025 SmileStory Inc. / WIA · 弘益人間 (홍익인간) · Benefit All Humanity
