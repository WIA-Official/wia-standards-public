/**
 * WIA-AI-025 Reinforcement Learning TypeScript SDK
 * Type Definitions
 *
 * © 2025 SmileStory Inc. / WIA
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */

// ============================================================================
// Core Types
// ============================================================================

export type State = number[] | number;
export type Action = number[] | number;
export type Reward = number;

export interface Transition {
    state: State;
    action: Action;
    reward: Reward;
    nextState: State;
    done: boolean;
}

export interface Episode {
    transitions: Transition[];
    totalReward: number;
    length: number;
}

// ============================================================================
// Environment Interface
// ============================================================================

export interface Space {
    shape: number[];
    dtype: 'float32' | 'int32';
    low?: number | number[];
    high?: number | number[];
}

export interface DiscreteSpace extends Space {
    n: number;  // Number of discrete actions/states
}

export interface BoxSpace extends Space {
    low: number[];
    high: number[];
}

export interface StepResult {
    observation: State;
    reward: Reward;
    done: boolean;
    info: Record<string, any>;
}

export interface Environment {
    actionSpace: Space;
    observationSpace: Space;

    reset(): State;
    step(action: Action): StepResult;
    render(mode?: 'human' | 'rgb_array'): void;
    close(): void;
    seed(seed: number): void;
}

// ============================================================================
// Agent Interface
// ============================================================================

export interface AgentConfig {
    learningRate: number;
    discountFactor: number;
    epsilon?: number;
    epsilonDecay?: number;
    epsilonMin?: number;
}

export interface Agent {
    selectAction(state: State, greedy?: boolean): Action;
    update(transition: Transition): void;
    train(episodes: number, env: Environment): TrainingResult;
    evaluate(episodes: number, env: Environment): EvaluationResult;
    save(path: string): void;
    load(path: string): void;
}

// ============================================================================
// Q-Learning Agent
// ============================================================================

export interface QLearningConfig extends AgentConfig {
    stateSize: number;
    actionSize: number;
}

export interface QTable {
    [state: string]: number[];  // state -> Q-values for each action
}

export interface QLearningAgent extends Agent {
    qTable: QTable;
    getQValue(state: State, action: Action): number;
    setQValue(state: State, action: Action, value: number): void;
    getBestAction(state: State): Action;
}

// ============================================================================
// Deep RL Agents
// ============================================================================

export interface NetworkConfig {
    inputDim: number;
    outputDim: number;
    hiddenLayers: number[];
    activation: 'relu' | 'tanh' | 'sigmoid';
    optimizer: 'adam' | 'sgd' | 'rmsprop';
}

export interface DQNConfig extends AgentConfig {
    networkConfig: NetworkConfig;
    bufferCapacity: number;
    batchSize: number;
    targetUpdateFreq: number;
    warmupSteps: number;
}

export interface ReplayBuffer {
    capacity: number;
    size: number;

    add(transition: Transition): void;
    sample(batchSize: number): Transition[];
    clear(): void;
}

export interface DQNAgent extends Agent {
    qNetwork: any;  // Neural network
    targetNetwork: any;
    replayBuffer: ReplayBuffer;
    updateTargetNetwork(): void;
}

// ============================================================================
// Policy Gradient Agents
// ============================================================================

export interface PolicyGradientConfig extends AgentConfig {
    networkConfig: NetworkConfig;
    entropyCoefficient: number;
    valueCoefficient?: number;
}

export interface PPOConfig extends PolicyGradientConfig {
    clipEpsilon: number;
    epochs: number;
    batchSize: number;
    lambda: number;  // GAE lambda
}

export interface PolicyGradientAgent extends Agent {
    policyNetwork: any;
    computeReturns(rewards: number[]): number[];
    computeAdvantages(states: State[], rewards: number[]): number[];
}

// ============================================================================
// Actor-Critic Agents
// ============================================================================

export interface ActorCriticConfig extends AgentConfig {
    actorConfig: NetworkConfig;
    criticConfig: NetworkConfig;
}

export interface ActorCriticAgent extends Agent {
    actor: any;  // Policy network
    critic: any;  // Value network
}

export interface SACConfig extends ActorCriticConfig {
    temperature: number;
    targetEntropy: number;
    autoTuneTemperature: boolean;
}

export interface SACAgent extends ActorCriticAgent {
    critic1: any;
    critic2: any;
    targetCritic1: any;
    targetCritic2: any;
    temperature: number;
}

// ============================================================================
// Training and Evaluation
// ============================================================================

export interface TrainingMetrics {
    episode: number;
    reward: number;
    length: number;
    loss?: number;
    epsilon?: number;
    timestamp: number;
}

export interface TrainingResult {
    episodes: Episode[];
    metrics: TrainingMetrics[];
    finalAgent: Agent;
    trainingTime: number;
}

export interface EvaluationMetrics {
    meanReward: number;
    stdReward: number;
    minReward: number;
    maxReward: number;
    successRate: number;
    meanLength: number;
}

export interface EvaluationResult {
    metrics: EvaluationMetrics;
    episodes: Episode[];
}

// ============================================================================
// Model-Based RL
// ============================================================================

export interface DynamicsModel {
    predict(state: State, action: Action): { nextState: State; reward: Reward };
    train(transitions: Transition[]): number;  // Returns loss
}

export interface ModelBasedAgent extends Agent {
    model: DynamicsModel;
    planWithModel(state: State, horizon: number): Action[];
}

// ============================================================================
// Multi-Agent RL
// ============================================================================

export interface MultiAgentEnvironment {
    nAgents: number;
    reset(): State[];
    step(actions: Action[]): {
        observations: State[];
        rewards: Reward[];
        dones: boolean[];
        info: Record<string, any>;
    };
}

export interface MultiAgentSystem {
    agents: Agent[];
    train(episodes: number, env: MultiAgentEnvironment): TrainingResult[];
    evaluate(episodes: number, env: MultiAgentEnvironment): EvaluationResult[];
}

// ============================================================================
// Utility Types
// ============================================================================

export interface ExplorationStrategy {
    selectAction(qValues: number[]): Action;
    updateParameters(episode: number): void;
}

export interface EpsilonGreedy extends ExplorationStrategy {
    epsilon: number;
    epsilonDecay: number;
    epsilonMin: number;
}

export interface BoltzmannExploration extends ExplorationStrategy {
    temperature: number;
    temperatureDecay: number;
}

export interface Logger {
    log(level: 'info' | 'warn' | 'error', message: string, data?: any): void;
    getHistory(): any[];
}

export interface Checkpoint {
    episode: number;
    agent: any;  // Serialized agent
    metrics: TrainingMetrics;
    timestamp: number;
}

// ============================================================================
// Safety and Constraints
// ============================================================================

export interface SafetyConstraint {
    check(state: State, action: Action): boolean;
    project(state: State, action: Action): Action;
}

export interface SafeRLAgent extends Agent {
    constraints: SafetyConstraint[];
    checkSafety(state: State, action: Action): boolean;
    projectToSafeSet(state: State, action: Action): Action;
}

// ============================================================================
// Production Deployment
// ============================================================================

export interface ModelServer {
    load(modelPath: string): void;
    predict(observation: State): Action;
    health(): { status: 'healthy' | 'unhealthy'; details: any };
}

export interface ABTestConfig {
    policies: Record<string, Agent>;
    trafficSplit: Record<string, number>;
}

export interface ABTest {
    selectPolicy(userId: string): { name: string; policy: Agent };
    recordMetric(policyName: string, reward: number): void;
    analyze(): { winner: string; pValue: number; confidence: number };
}

// ============================================================================
// Exports
// ============================================================================

export default {
    // Core types exported above
};
