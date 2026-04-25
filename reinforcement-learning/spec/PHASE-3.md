# WIA-AI-025 Reinforcement Learning - PHASE 3 Specification

## Overview

**Status:** ✅ ACTIVE
**Version:** 1.0.0
**Prerequisites:** PHASE 1, PHASE 2 complete

## Scope

PHASE 3 covers advanced deep RL, model-based methods, multi-agent systems, and offline RL for complex real-world applications.

## Advanced Actor-Critic

### 3.1 Soft Actor-Critic (SAC)

**Maximum Entropy RL:**
```python
J(π) = E[Σ r_t + α H(π(·|s_t))]

where H(π) = -E[log π(a|s)] is entropy
```

**Implementation:**
```python
class SAC:
    def __init__(self, state_dim, action_dim):
        self.actor = GaussianPolicy(state_dim, action_dim)
        self.critic1 = QNetwork(state_dim, action_dim)
        self.critic2 = QNetwork(state_dim, action_dim)
        self.target_critic1 = copy.deepcopy(self.critic1)
        self.target_critic2 = copy.deepcopy(self.critic2)

        # Automatic temperature tuning
        self.target_entropy = -action_dim
        self.log_alpha = torch.zeros(1, requires_grad=True)
```

**Requirements:**
- ✅ Twin Q-functions to reduce overestimation
- ✅ Stochastic policy with entropy bonus
- ✅ Automatic temperature tuning
- ✅ Off-policy learning with replay buffer
- ✅ Soft target updates

### 3.2 Twin Delayed DDPG (TD3)

**Three Key Improvements:**
1. Twin critics with minimum
2. Delayed policy updates
3. Target policy smoothing

```python
# Target policy smoothing
noise = (torch.randn_like(action) * noise_std).clamp(-noise_clip, noise_clip)
next_action = (self.actor_target(next_state) + noise).clamp(-max_action, max_action)

# Twin critics
target_q1 = self.critic1_target(next_state, next_action)
target_q2 = self.critic2_target(next_state, next_action)
target_q = torch.min(target_q1, target_q2)
```

## Model-Based RL

### 3.3 Learned Dynamics Models

```python
class WorldModel:
    def __init__(self, state_dim, action_dim):
        # Dynamics model
        self.dynamics = DynamicsNetwork(state_dim, action_dim)

        # Reward model
        self.reward = RewardNetwork(state_dim, action_dim)

    def predict(self, state, action):
        next_state = self.dynamics(state, action)
        reward = self.reward(state, action)
        return next_state, reward

    def train(self, replay_buffer, batch_size=256):
        states, actions, rewards, next_states = replay_buffer.sample(batch_size)

        # Predict
        pred_next_states, pred_rewards = self.predict(states, actions)

        # Loss
        state_loss = F.mse_loss(pred_next_states, next_states)
        reward_loss = F.mse_loss(pred_rewards, rewards)

        return state_loss + reward_loss
```

### 3.4 Model Predictive Control (MPC)

```python
def mpc_planning(model, state, horizon=10, num_sequences=1000):
    """
    Plan action sequence using learned model.
    """
    best_return = -float('inf')
    best_action = None

    for _ in range(num_sequences):
        # Sample random action sequence
        action_sequence = [sample_action() for _ in range(horizon)]

        # Simulate trajectory
        total_return = 0
        sim_state = state

        for t, action in enumerate(action_sequence):
            sim_state, reward = model.predict(sim_state, action)
            total_return += (gamma ** t) * reward

        # Track best sequence
        if total_return > best_return:
            best_return = total_return
            best_action = action_sequence[0]

    return best_action
```

## Multi-Agent RL

### 3.5 QMIX - Value Decomposition

```python
class QMIXNetwork(nn.Module):
    def __init__(self, n_agents, obs_dim, n_actions, state_dim):
        super().__init__()

        # Individual Q-networks
        self.agent_networks = nn.ModuleList([
            AgentQNetwork(obs_dim, n_actions)
            for _ in range(n_agents)
        ])

        # Mixing network (ensures monotonicity)
        self.hyper_w1 = nn.Linear(state_dim, n_agents * mixing_embed_dim)
        self.hyper_w2 = nn.Linear(state_dim, mixing_embed_dim)

    def forward(self, agent_obs, global_state, actions):
        # Individual Q-values
        agent_qs = [net(obs) for net, obs in zip(self.agent_networks, agent_obs)]
        agent_qs = torch.stack(agent_qs)

        # Mix Q-values (monotonic combination)
        w1 = torch.abs(self.hyper_w1(global_state))
        w2 = torch.abs(self.hyper_w2(global_state))

        # Q_tot = f(Q1, Q2, ..., Qn | global_state)
        q_tot = self.mix(agent_qs, w1, w2)
        return q_tot
```

### 3.6 Multi-Agent PPO

```python
class MAPPO:
    def __init__(self, n_agents, obs_dim, action_dim):
        self.agents = [
            PPOAgent(obs_dim, action_dim)
            for _ in range(n_agents)
        ]

        # Centralized critic sees all observations
        self.centralized_critic = CentralizedCritic(n_agents * obs_dim)

    def update(self, trajectories):
        # Compute advantages using centralized critic
        all_obs = concat([traj.obs for traj in trajectories])
        values = self.centralized_critic(all_obs)

        # Update each agent's policy
        for i, agent in enumerate(self.agents):
            agent.update_policy(
                trajectories[i],
                advantages=advantages[i]
            )
```

## Offline RL

### 3.7 Conservative Q-Learning (CQL)

**Prevent overestimation on out-of-distribution actions:**

```python
def cql_loss(q_network, batch, alpha=1.0):
    states, actions, rewards, next_states, dones = batch

    # Standard Q-learning loss
    q_values = q_network(states, actions)
    with torch.no_grad():
        next_q = q_network(next_states).max(1)[0]
        targets = rewards + gamma * next_q * (1 - dones)
    bellman_loss = F.mse_loss(q_values, targets)

    # Conservative penalty: push down Q-values on unseen actions
    all_q_values = q_network(states)  # Q for all actions
    logsumexp = torch.logsumexp(all_q_values, dim=1)
    conservative_penalty = logsumexp.mean() - q_values.mean()

    return bellman_loss + alpha * conservative_penalty
```

### 3.8 Behavioral Cloning + RL

```python
def bc_rl_training(offline_data, env):
    # Phase 1: Behavioral cloning
    policy = train_behavioral_cloning(offline_data)

    # Phase 2: Fine-tune with online RL
    for episode in range(num_episodes):
        state = env.reset()
        done = False

        while not done:
            action = policy(state)
            next_state, reward, done = env.step(action)

            # Update with online data
            policy.update(state, action, reward, next_state)

            state = next_state
```

## Hierarchical RL

### 3.9 Options Framework

```python
class Option:
    def __init__(self, init_set, policy, termination):
        self.init_set = init_set      # States where option can start
        self.policy = policy           # Low-level policy
        self.termination = termination # When to terminate

    def execute(self, env, state):
        """Execute option until termination."""
        trajectory = []

        while not self.termination(state):
            action = self.policy(state)
            next_state, reward, done = env.step(action)
            trajectory.append((state, action, reward))

            if done:
                break

            state = next_state

        return trajectory

class HierarchicalAgent:
    def __init__(self, high_level_policy, options):
        self.high_policy = high_level_policy  # Selects options
        self.options = options                 # Set of skills

    def act(self, state):
        # High-level: select option
        option = self.high_policy.select_option(state)

        # Low-level: execute option
        trajectory = option.execute(env, state)

        return trajectory
```

## Meta-Learning

### 3.10 Model-Agnostic Meta-Learning (MAML)

```python
def maml_update(tasks, meta_policy, inner_lr=0.01, outer_lr=0.001):
    """
    Learn to learn: adapt quickly to new tasks.
    """
    meta_gradients = []

    for task in tasks:
        # Inner loop: adapt to task
        adapted_policy = copy.deepcopy(meta_policy)

        for _ in range(inner_steps):
            loss = compute_task_loss(adapted_policy, task)
            grads = torch.autograd.grad(loss, adapted_policy.parameters())

            # Gradient descent
            for param, grad in zip(adapted_policy.parameters(), grads):
                param.data -= inner_lr * grad

        # Outer loop: compute meta-gradient
        meta_loss = compute_task_loss(adapted_policy, task, eval=True)
        meta_grads = torch.autograd.grad(meta_loss, meta_policy.parameters())
        meta_gradients.append(meta_grads)

    # Update meta-policy
    for param, grads in zip(meta_policy.parameters(), zip(*meta_gradients)):
        param.data -= outer_lr * torch.stack(grads).mean(0)
```

## Curriculum Learning

### 3.11 Automatic Curriculum

```python
class CurriculumScheduler:
    def __init__(self, task_difficulties):
        self.tasks = task_difficulties
        self.current_task = 0

    def select_task(self, agent_performance):
        """
        Select next task based on agent's current ability.
        """
        # Progress to harder task if mastered current
        if agent_performance > threshold:
            self.current_task = min(self.current_task + 1, len(self.tasks) - 1)

        # Regress if struggling
        elif agent_performance < lower_threshold:
            self.current_task = max(self.current_task - 1, 0)

        return self.tasks[self.current_task]
```

## Performance Requirements

### Benchmarks

**MuJoCo (SAC/TD3):**
- Ant-v2: ≥ 4000 average return
- Humanoid-v2: ≥ 5000 average return

**Multi-Agent (QMIX):**
- SMAC (StarCraft Multi-Agent Challenge): ≥ 80% win rate

**Offline RL (CQL):**
- D4RL MuJoCo: ≥ 60% of expert performance

## API Specification

```typescript
interface AdvancedRLAgent {
    // Model-based planning
    planWithModel(model: DynamicsModel, horizon: number): Action;

    // Multi-agent coordination
    coordinateWithAgents(observations: Observation[]): Action[];

    // Offline learning
    trainOffline(dataset: OfflineDataset): void;

    // Meta-learning
    adapt(newTask: Task, shots: number): void;
}
```

## References

1. 선행 연구. Soft Actor-Critic Algorithms
2. 선행 연구. Addressing Function Approximation Error in Actor-Critic Methods (TD3)
3. 선행 연구. QMIX: Monotonic Value Function Factorisation
4. 선행 연구. Conservative Q-Learning for Offline RL
5. 선행 연구. Model-Agnostic Meta-Learning

---

**Previous:** [PHASE 2 - Deep RL](./PHASE-2.md)
**Next:** [PHASE 4 - Production Systems](./PHASE-4.md)

© 2025 SmileStory Inc. / WIA · 弘益人間 (홍익인간) · Benefit All Humanity
