# WIA-AI-025 Reinforcement Learning - PHASE 2 Specification

## Overview

**Status:** ✅ ACTIVE
**Version:** 1.0.0
**Last Updated:** 2025-01-20
**Prerequisites:** PHASE 1 complete

## Scope

PHASE 2 introduces deep reinforcement learning algorithms that use neural networks as function approximators, enabling RL in high-dimensional state spaces.

## Core Algorithms

### 2.1 Deep Q-Network (DQN)

**Innovations:**
1. Neural network Q-function approximation
2. Experience replay buffer
3. Target network for stable learning

```python
class DQN:
    def __init__(self, state_dim, action_dim):
        self.q_network = QNetwork(state_dim, action_dim)
        self.target_network = QNetwork(state_dim, action_dim)
        self.replay_buffer = ReplayBuffer(capacity=100000)

    def update(self, batch_size=32):
        # Sample from replay buffer
        states, actions, rewards, next_states, dones = \
            self.replay_buffer.sample(batch_size)

        # Compute targets using target network
        with torch.no_grad():
            next_q_values = self.target_network(next_states).max(1)[0]
            targets = rewards + gamma * next_q_values * (1 - dones)

        # Update Q-network
        q_values = self.q_network(states).gather(1, actions)
        loss = F.mse_loss(q_values, targets)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    def update_target_network(self):
        self.target_network.load_state_dict(self.q_network.state_dict())
```

**Requirements:**
- ✅ Replay buffer with priority sampling
- ✅ Target network updated every N steps
- ✅ Gradient clipping
- ✅ Huber loss option
- ✅ Double DQN variant

### 2.2 Policy Gradient Methods

#### REINFORCE
```python
∇_θ J(θ) = E[Σ_t ∇_θ log π_θ(a_t|s_t) G_t]
```

**Requirements:**
- ✅ Monte Carlo return calculation
- ✅ Baseline subtraction for variance reduction
- ✅ Advantage estimation
- ✅ Entropy regularization

#### Proximal Policy Optimization (PPO)
```python
L^CLIP(θ) = E[min(r_t(θ) Â_t, clip(r_t(θ), 1-ε, 1+ε) Â_t)]
```

**Key Features:**
- Clipped surrogate objective
- Multiple epochs per batch
- Generalized Advantage Estimation (GAE)
- Adaptive KL penalty

### 2.3 Actor-Critic Methods

#### Advantage Actor-Critic (A2C)
```python
class A2C:
    def __init__(self, state_dim, action_dim):
        self.actor = PolicyNetwork(state_dim, action_dim)
        self.critic = ValueNetwork(state_dim)

    def update(self, trajectories):
        # Compute advantages
        values = self.critic(states)
        next_values = self.critic(next_states)
        advantages = rewards + gamma * next_values - values

        # Update critic
        critic_loss = advantages.pow(2).mean()

        # Update actor
        log_probs = self.actor.log_prob(states, actions)
        actor_loss = -(log_probs * advantages.detach()).mean()

        # Total loss
        loss = actor_loss + 0.5 * critic_loss - 0.01 * entropy
```

## Network Architectures

### 2.4 Standard Networks

#### MLP for Vector States
```python
class MLPNetwork(nn.Module):
    def __init__(self, input_dim, output_dim, hidden_dims=[256, 256]):
        super().__init__()
        layers = []
        prev_dim = input_dim

        for hidden_dim in hidden_dims:
            layers.extend([
                nn.Linear(prev_dim, hidden_dim),
                nn.ReLU()
            ])
            prev_dim = hidden_dim

        layers.append(nn.Linear(prev_dim, output_dim))
        self.network = nn.Sequential(*layers)
```

#### CNN for Image States
```python
class CNNNetwork(nn.Module):
    def __init__(self, input_channels, num_actions):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(input_channels, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            nn.ReLU()
        )
        self.fc = nn.Sequential(
            nn.Linear(7 * 7 * 64, 512),
            nn.ReLU(),
            nn.Linear(512, num_actions)
        )
```

## Advanced Techniques

### 2.5 Experience Replay

```python
class PrioritizedReplayBuffer:
    def __init__(self, capacity, alpha=0.6):
        self.capacity = capacity
        self.alpha = alpha  # Priority exponent
        self.buffer = []
        self.priorities = []

    def add(self, experience, td_error):
        priority = (abs(td_error) + 1e-6) ** self.alpha
        self.priorities.append(priority)
        self.buffer.append(experience)

    def sample(self, batch_size, beta=0.4):
        # Sample based on priorities
        probs = np.array(self.priorities) / sum(self.priorities)
        indices = np.random.choice(len(self.buffer), batch_size, p=probs)

        # Importance sampling weights
        weights = (len(self.buffer) * probs[indices]) ** (-beta)
        weights /= weights.max()

        return batch, indices, weights
```

### 2.6 Generalized Advantage Estimation (GAE)

```python
def compute_gae(rewards, values, next_values, dones, gamma=0.99, lambda_=0.95):
    advantages = []
    gae = 0

    for t in reversed(range(len(rewards))):
        delta = rewards[t] + gamma * next_values[t] * (1 - dones[t]) - values[t]
        gae = delta + gamma * lambda_ * (1 - dones[t]) * gae
        advantages.insert(0, gae)

    return advantages
```

## Continuous Control

### 2.7 Continuous Action Spaces

```python
class GaussianPolicy(nn.Module):
    def __init__(self, state_dim, action_dim):
        super().__init__()
        self.mean = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.ReLU(),
            nn.Linear(256, action_dim)
        )
        self.log_std = nn.Parameter(torch.zeros(action_dim))

    def forward(self, state):
        mean = self.mean(state)
        std = torch.exp(self.log_std)
        return Normal(mean, std)

    def sample(self, state):
        dist = self.forward(state)
        action = dist.sample()
        log_prob = dist.log_prob(action).sum(dim=-1)
        return action, log_prob
```

## Performance Benchmarks

### Atari Games (DQN)
- **Breakout:** Score ≥ 400 within 10M frames
- **Pong:** Score ≥ 18 within 5M frames
- **Space Invaders:** Score ≥ 1000 within 10M frames

### MuJoCo Continuous Control (PPO)
- **HalfCheetah-v2:** Average return ≥ 2000
- **Hopper-v2:** Average return ≥ 2500
- **Walker2d-v2:** Average return ≥ 2500

## Implementation Requirements

### 2.8 Training Infrastructure

```python
def train_dqn(env, config, max_timesteps=1000000):
    agent = DQN(
        state_dim=env.observation_space.shape,
        action_dim=env.action_space.n,
        **config
    )

    state = env.reset()
    episode_reward = 0
    episode_length = 0

    for timestep in range(max_timesteps):
        # Select action
        if timestep < config.warmup_steps:
            action = env.action_space.sample()
        else:
            action = agent.select_action(state, epsilon=epsilon)

        # Execute action
        next_state, reward, done, info = env.step(action)
        episode_reward += reward
        episode_length += 1

        # Store transition
        agent.replay_buffer.add(state, action, reward, next_state, done)

        # Update agent
        if timestep >= config.warmup_steps:
            agent.update(batch_size=config.batch_size)

        # Update target network
        if timestep % config.target_update_freq == 0:
            agent.update_target_network()

        # Episode end
        if done:
            logger.info(f"Episode reward: {episode_reward}, Length: {episode_length}")
            state = env.reset()
            episode_reward = 0
            episode_length = 0
        else:
            state = next_state
```

## Testing Requirements

### Unit Tests
- ✅ Neural network forward/backward pass
- ✅ Replay buffer operations
- ✅ Target network updates
- ✅ GAE computation correctness

### Integration Tests
- ✅ End-to-end training pipeline
- ✅ Checkpoint save/load
- ✅ Multi-GPU training
- ✅ Distributed training (Ray RLlib)

### Performance Tests
- ✅ Atari benchmark compliance
- ✅ MuJoCo benchmark compliance
- ✅ Training time within 2x reference
- ✅ GPU memory efficiency

## API Specification

```typescript
interface DeepRLAgent {
    network: NeuralNetwork;
    optimizer: Optimizer;
    replayBuffer: ReplayBuffer;

    selectAction(state: Tensor, epsilon?: number): Action;
    update(batchSize: number): TrainingMetrics;
    save(path: string): void;
    load(path: string): void;
}

interface TrainingConfig {
    learningRate: number;
    batchSize: number;
    bufferCapacity: number;
    targetUpdateFreq: number;
    gamma: number;
    epsilon: EpsilonSchedule;
}
```

## Best Practices

1. **Normalization:** Always normalize observations
2. **Reward Clipping:** Clip rewards to [-1, 1] for Atari
3. **Gradient Clipping:** Clip gradients to prevent explosion
4. **Frame Stacking:** Stack 4 frames for partial observability
5. **Action Repeat:** Repeat actions for 4 frames (Atari)

## References

1. 선행 연구. Human-level control through deep reinforcement learning. *Nature*.
2. 선행 연구. Proximal Policy Optimization Algorithms.
3. 선행 연구. Asynchronous Methods for Deep Reinforcement Learning.

---

**Previous:** [PHASE 1 - Tabular Methods](./PHASE-1.md)
**Next:** [PHASE 3 - Advanced Deep RL](./PHASE-3.md)

© 2025 SmileStory Inc. / WIA · 弘益人間 (홍익인간) · Benefit All Humanity
