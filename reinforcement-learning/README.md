# WIA-AI-025: Reinforcement Learning Standard 🎮

**홍익인간 (弘益人間) (홍익인간) · Benefit All Humanity**

## Overview

WIA-AI-025 is a comprehensive reinforcement learning standard developed by the World Certification Industry Association (WIA). This standard provides implementations, specifications, and educational resources for reinforcement learning from basic algorithms to production deployment.

## 🌟 Features

- **📚 Complete Ebook:** 8 chapters covering RL fundamentals to production deployment (English + Korean)
- **🎮 Interactive Simulator:** Hands-on learning with 5 interactive demos
- **📋 4-Phase Specifications:** Progressive standards from tabular methods to production systems
- **💻 TypeScript SDK:** Production-ready RL library with type safety
- **🌐 Multi-Language Support:** English and Korean documentation
- **🎯 Real-World Focus:** Practical implementations and deployment guides

## 📖 Table of Contents

- [Quick Start](#quick-start)
- [Directory Structure](#directory-structure)
- [Ebook Chapters](#ebook-chapters)
- [Specifications](#specifications)
- [API Usage](#api-usage)
- [Simulator](#simulator)
- [Contributing](#contributing)
- [License](#license)

## 🚀 Quick Start

### 1. Explore the Interactive Landing Page

Open `index.html` in your browser:

```bash
cd reinforcement-learning
open index.html  # macOS
# or
xdg-open index.html  # Linux
# or
start index.html  # Windows
```

### 2. Try the Simulator

Navigate to the simulator for hands-on learning:

```bash
open simulator/index.html
```

Features:
- 🎯 Q-Learning Demo
- 🎲 Policy Gradient
- 🌍 Environment Sandbox
- ⭐ Reward Shaping
- 🤝 Multi-Agent RL

### 3. Read the Ebook

**English Version:**
- Chapter 1: [Introduction to Reinforcement Learning](./ebook/en/01-introduction.html)
- Chapter 2: [Markov Decision Process Fundamentals](./ebook/en/02-mdp-fundamentals.html)
- Chapter 3: [Value-Based Methods](./ebook/en/03-value-based-methods.html)
- Chapter 4: [Policy Gradient Methods](./ebook/en/04-policy-gradient.html)
- Chapter 5: [Actor-Critic Methods](./ebook/en/05-actor-critic.html)
- Chapter 6: [Model-Based RL](./ebook/en/06-model-based-rl.html)
- Chapter 7: [Multi-Agent RL](./ebook/en/07-multi-agent-rl.html)
- Chapter 8: [Production Deployment](./ebook/en/08-production-deployment.html)

**Korean Version:**
- 제1장: [강화학습 소개](./ebook/ko/01-introduction.html)
- (Additional chapters available in Korean)

### 4. Use the TypeScript SDK

Install the SDK:

```bash
npm install @wia/rl-sdk
```

Example usage:

```typescript
import { QLearningAgent } from '@wia/rl-sdk';

// Create agent
const agent = new QLearningAgent({
    stateSize: 64,
    actionSize: 4,
    learningRate: 0.1,
    discountFactor: 0.99,
    epsilon: 0.1
});

// Train
const result = agent.train(1000, env);

// Evaluate
const evalResult = agent.evaluate(100, env);
console.log(`Mean Reward: ${evalResult.metrics.meanReward}`);
```

## 📁 Directory Structure

```
reinforcement-learning/
├── index.html                  # Landing page with dark theme & animations
├── simulator/
│   └── index.html             # Interactive 5-tab simulator
├── ebook/
│   ├── en/                    # 8 English chapters (18-25KB each)
│   │   ├── 01-introduction.html
│   │   ├── 02-mdp-fundamentals.html
│   │   ├── 03-value-based-methods.html
│   │   ├── 04-policy-gradient.html
│   │   ├── 05-actor-critic.html
│   │   ├── 06-model-based-rl.html
│   │   ├── 07-multi-agent-rl.html
│   │   └── 08-production-deployment.html
│   └── ko/                    # 8 Korean chapters (15-20KB each)
│       ├── 01-introduction.html
│       └── ...
├── spec/
│   ├── PHASE-1.md             # Tabular methods & basics
│   ├── PHASE-2.md             # Deep RL (DQN, PPO, A2C)
│   ├── PHASE-3.md             # Advanced (SAC, TD3, MARL, Offline RL)
│   └── PHASE-4.md             # Production deployment
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── types.ts       # TypeScript type definitions
│       │   └── index.ts       # Main SDK implementation
│       └── package.json
└── README.md                   # This file
```

## 📚 Ebook Chapters

Each chapter includes:
- ✅ 8-10 major sections with h2 headings
- ✅ Comprehensive code examples
- ✅ Visual diagrams and tables
- ✅ Summary section (요약/Summary)
- ✅ Review questions (복습 문제/Review Questions)
- ✅ Philosophy statement: 홍익인간 (弘益人間) (홍익인간)

### English Chapters (18-25KB each)

1. **Introduction to Reinforcement Learning** - What is RL, exploration vs exploitation, applications
2. **MDP Fundamentals** - Markov property, value functions, Bellman equations, policy iteration
3. **Value-Based Methods** - Q-Learning, SARSA, DQN, Double DQN, Dueling DQN
4. **Policy Gradient** - REINFORCE, PPO, TRPO, continuous control, GAE
5. **Actor-Critic** - A2C, A3C, SAC, TD3, combining value and policy learning
6. **Model-Based RL** - Dyna, learned models, MPC, MCTS, World Models
7. **Multi-Agent RL** - QMIX, MADDPG, communication, emergent behavior
8. **Production Deployment** - Serving, distributed training, safety, monitoring, A/B testing

### Korean Chapters (15-20KB each)

Concise Korean translations with same topics and structure.

## 📋 Specifications

### PHASE 1: Foundation Algorithms
- Q-Learning (tabular)
- SARSA
- Monte Carlo methods
- OpenAI Gym compatibility
- Basic exploration strategies

### PHASE 2: Deep Reinforcement Learning
- Deep Q-Networks (DQN)
- Policy gradient methods (REINFORCE, PPO)
- Actor-Critic (A2C, A3C)
- Experience replay
- Continuous control

### PHASE 3: Advanced Methods
- Soft Actor-Critic (SAC)
- Twin Delayed DDPG (TD3)
- Model-based RL
- Multi-agent systems (QMIX, MADDPG)
- Offline RL (CQL)
- Hierarchical RL
- Meta-learning (MAML)

### PHASE 4: Production Systems
- Model serving (FastAPI, TensorFlow Serving)
- Distributed training (Ray RLlib)
- Safety constraints (CPO)
- Monitoring & observability
- Online learning
- A/B testing
- Model optimization (quantization, distillation)

## 💻 API Usage

### Q-Learning Example

```typescript
import { QLearningAgent, EpsilonGreedyExploration } from '@wia/rl-sdk';

const agent = new QLearningAgent({
    stateSize: 64,
    actionSize: 4,
    learningRate: 0.1,
    discountFactor: 0.99,
    epsilon: 0.1,
    epsilonDecay: 0.995,
    epsilonMin: 0.01
});

// Training
const result = agent.train(1000, environment);
console.log(`Trained in ${result.trainingTime}ms`);

// Evaluation
const evalResult = agent.evaluate(100, environment);
console.log(`Mean Reward: ${evalResult.metrics.meanReward.toFixed(2)}`);
console.log(`Success Rate: ${(evalResult.metrics.successRate * 100).toFixed(1)}%`);

// Save/Load
agent.save('./models/q-learning-agent.json');
agent.load('./models/q-learning-agent.json');
```

### Replay Buffer Example

```typescript
import { SimpleReplayBuffer } from '@wia/rl-sdk';

const buffer = new SimpleReplayBuffer(10000);

// Add transitions
buffer.add({
    state: [0, 1, 2, 3],
    action: 1,
    reward: 10,
    nextState: [0, 2, 3, 4],
    done: false
});

// Sample batch
const batch = buffer.sample(32);
```

### Utility Functions

```typescript
import { computeReturns, computeAdvantages, normalizeArray } from '@wia/rl-sdk';

// Compute Monte Carlo returns
const rewards = [1, 0, 0, 5, 2];
const returns = computeReturns(rewards, 0.99);

// Compute GAE advantages
const advantages = computeAdvantages(
    rewards,
    values,
    nextValues,
    dones,
    0.99,  // gamma
    0.95   // lambda
);

// Normalize
const normalized = normalizeArray(advantages);
```

## 🎮 Simulator Features

The interactive simulator includes 5 tabs:

### 1. Q-Learning Demo 🎯
- Interactive grid world
- Visualize Q-value updates
- Adjust hyperparameters in real-time
- Training progress charts

### 2. Policy Gradient 🎲
- CartPole environment
- REINFORCE algorithm
- PPO comparison
- Policy performance visualization

### 3. Environment Sandbox 🌍
- Build custom environments
- Define state/action spaces
- Configure rewards
- Export as OpenAI Gym code

### 4. Reward Shaping ⭐
- Design reward functions
- Compare shaping strategies
- Sparse vs dense rewards
- Potential-based shaping

### 5. Multi-Agent RL 🤝
- Cooperative navigation
- Competitive scenarios
- Communication protocols
- Emergent behaviors

## 🔧 Development

### Local Development

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/reinforcement-learning

# Install TypeScript SDK dependencies
cd api/typescript
npm install

# Build
npm run build

# Run tests
npm test

# Lint and format
npm run lint
npm run format
```

### Running the Simulator Locally

Simply open the HTML files in a modern browser:

```bash
# Landing page
open index.html

# Simulator
open simulator/index.html

# Ebook
open ebook/en/01-introduction.html
```

## 🤝 Contributing

We welcome contributions! Please follow these guidelines:

1. **Code Style:** Follow TypeScript best practices
2. **Documentation:** Update relevant documentation
3. **Testing:** Add tests for new features
4. **Commit Messages:** Use conventional commits format

```bash
git commit -m "feat: add DQN implementation"
git commit -m "fix: correct Q-learning update rule"
git commit -m "docs: improve API documentation"
```

## 📄 License

MIT License

Copyright © 2025 SmileStory Inc. / WIA (World Certification Industry Association)

## 🌍 Philosophy

**홍익인간 (弘益人間) (홍익인간) · Benefit All Humanity**

This standard embodies the Korean philosophy of "broadly benefiting humanity." Reinforcement learning has immense potential to improve lives through:

- 🏥 Healthcare optimization
- 🚗 Autonomous vehicles
- 🌱 Environmental protection
- 🏭 Industrial efficiency
- 🎓 Personalized education
- 🤖 Assistive robotics

By making RL accessible through open standards, we democratize this powerful technology for the benefit of all.

## 📞 Contact

- **Website:** https://wia.official
- **GitHub:** https://github.com/WIA-Official/wia-standards
- **Issues:** https://github.com/WIA-Official/wia-standards/issues

## 🙏 Acknowledgments

- OpenAI for Gym environment standard
- Sutton & Barto for "Reinforcement Learning: An Introduction"
- DeepMind for DQN and AlphaGo innovations
- Berkeley for Soft Actor-Critic
- OpenAI for PPO and foundational research
- The broader RL research community

---

**Made with ❤️ by SmileStory Inc. / WIA**

**홍익인간 (弘益人間) (홍익인간) · Benefit All Humanity**
