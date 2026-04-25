/**
 * WIA-AI-025 Reinforcement Learning TypeScript SDK
 * Main Entry Point
 *
 * © 2025 SmileStory Inc. / WIA
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */

import * as Types from './types';

// Export all types
export * from './types';

// ============================================================================
// Q-Learning Agent Implementation
// ============================================================================

export class QLearningAgent implements Types.QLearningAgent {
    public qTable: Types.QTable = {};
    private config: Types.QLearningConfig;

    constructor(config: Types.QLearningConfig) {
        this.config = {
            epsilon: 0.1,
            epsilonDecay: 0.995,
            epsilonMin: 0.01,
            ...config
        };
    }

    private stateKey(state: Types.State): string {
        return Array.isArray(state) ? state.join(',') : state.toString();
    }

    private initializeState(state: Types.State): void {
        const key = this.stateKey(state);
        if (!(key in this.qTable)) {
            this.qTable[key] = new Array(this.config.actionSize).fill(0);
        }
    }

    public getQValue(state: Types.State, action: Types.Action): number {
        this.initializeState(state);
        const key = this.stateKey(state);
        return this.qTable[key][action as number];
    }

    public setQValue(state: Types.State, action: Types.Action, value: number): void {
        this.initializeState(state);
        const key = this.stateKey(state);
        this.qTable[key][action as number] = value;
    }

    public getBestAction(state: Types.State): Types.Action {
        this.initializeState(state);
        const key = this.stateKey(state);
        const qValues = this.qTable[key];
        return qValues.indexOf(Math.max(...qValues));
    }

    public selectAction(state: Types.State, greedy: boolean = false): Types.Action {
        if (!greedy && Math.random() < (this.config.epsilon || 0.1)) {
            // Explore: random action
            return Math.floor(Math.random() * this.config.actionSize);
        } else {
            // Exploit: best action
            return this.getBestAction(state);
        }
    }

    public update(transition: Types.Transition): void {
        const { state, action, reward, nextState, done } = transition;

        // Q-Learning update: Q(s,a) ← Q(s,a) + α[r + γ max Q(s',a') - Q(s,a)]
        const currentQ = this.getQValue(state, action);
        const maxNextQ = done ? 0 : Math.max(...this.qTable[this.stateKey(nextState)] || [0]);

        const target = reward + this.config.discountFactor * maxNextQ;
        const newQ = currentQ + this.config.learningRate * (target - currentQ);

        this.setQValue(state, action, newQ);
    }

    public train(episodes: number, env: Types.Environment): Types.TrainingResult {
        const startTime = Date.now();
        const episodeData: Types.Episode[] = [];
        const metrics: Types.TrainingMetrics[] = [];

        for (let episode = 0; episode < episodes; episode++) {
            let state = env.reset();
            let totalReward = 0;
            const transitions: Types.Transition[] = [];
            let done = false;
            let steps = 0;

            while (!done) {
                const action = this.selectAction(state);
                const result = env.step(action);

                const transition: Types.Transition = {
                    state,
                    action,
                    reward: result.reward,
                    nextState: result.observation,
                    done: result.done
                };

                this.update(transition);
                transitions.push(transition);

                totalReward += result.reward;
                state = result.observation;
                done = result.done;
                steps++;
            }

            // Decay epsilon
            if (this.config.epsilon && this.config.epsilonDecay) {
                this.config.epsilon = Math.max(
                    this.config.epsilonMin || 0.01,
                    this.config.epsilon * this.config.epsilonDecay
                );
            }

            episodeData.push({
                transitions,
                totalReward,
                length: steps
            });

            metrics.push({
                episode,
                reward: totalReward,
                length: steps,
                epsilon: this.config.epsilon,
                timestamp: Date.now()
            });

            if ((episode + 1) % 100 === 0) {
                const avgReward = metrics.slice(-100).reduce((sum, m) => sum + m.reward, 0) / 100;
                console.log(`Episode ${episode + 1}, Avg Reward: ${avgReward.toFixed(2)}, Epsilon: ${this.config.epsilon?.toFixed(3)}`);
            }
        }

        return {
            episodes: episodeData,
            metrics,
            finalAgent: this,
            trainingTime: Date.now() - startTime
        };
    }

    public evaluate(episodes: number, env: Types.Environment): Types.EvaluationResult {
        const episodeData: Types.Episode[] = [];
        const rewards: number[] = [];
        const lengths: number[] = [];
        let successes = 0;

        for (let episode = 0; episode < episodes; episode++) {
            let state = env.reset();
            let totalReward = 0;
            const transitions: Types.Transition[] = [];
            let done = false;
            let steps = 0;

            while (!done) {
                const action = this.selectAction(state, true);  // Greedy
                const result = env.step(action);

                transitions.push({
                    state,
                    action,
                    reward: result.reward,
                    nextState: result.observation,
                    done: result.done
                });

                totalReward += result.reward;
                state = result.observation;
                done = result.done;
                steps++;

                if (result.info.success) successes++;
            }

            episodeData.push({
                transitions,
                totalReward,
                length: steps
            });

            rewards.push(totalReward);
            lengths.push(steps);
        }

        const meanReward = rewards.reduce((a, b) => a + b, 0) / rewards.length;
        const variance = rewards.reduce((sum, r) => sum + Math.pow(r - meanReward, 2), 0) / rewards.length;
        const stdReward = Math.sqrt(variance);

        return {
            metrics: {
                meanReward,
                stdReward,
                minReward: Math.min(...rewards),
                maxReward: Math.max(...rewards),
                successRate: successes / episodes,
                meanLength: lengths.reduce((a, b) => a + b, 0) / lengths.length
            },
            episodes: episodeData
        };
    }

    public save(path: string): void {
        // Implementation would serialize qTable to file
        console.log(`Saving agent to ${path}`);
        // In browser: localStorage, in Node: fs.writeFileSync
    }

    public load(path: string): void {
        // Implementation would deserialize qTable from file
        console.log(`Loading agent from ${path}`);
    }
}

// ============================================================================
// Utility Functions
// ============================================================================

export class EpsilonGreedyExploration implements Types.EpsilonGreedy {
    public epsilon: number;
    public epsilonDecay: number;
    public epsilonMin: number;

    constructor(epsilon = 0.1, epsilonDecay = 0.995, epsilonMin = 0.01) {
        this.epsilon = epsilon;
        this.epsilonDecay = epsilonDecay;
        this.epsilonMin = epsilonMin;
    }

    public selectAction(qValues: number[]): Types.Action {
        if (Math.random() < this.epsilon) {
            return Math.floor(Math.random() * qValues.length);
        } else {
            return qValues.indexOf(Math.max(...qValues));
        }
    }

    public updateParameters(episode: number): void {
        this.epsilon = Math.max(this.epsilonMin, this.epsilon * this.epsilonDecay);
    }
}

export class SimpleReplayBuffer implements Types.ReplayBuffer {
    public capacity: number;
    public size: number = 0;
    private buffer: Types.Transition[] = [];
    private position: number = 0;

    constructor(capacity: number) {
        this.capacity = capacity;
    }

    public add(transition: Types.Transition): void {
        if (this.buffer.length < this.capacity) {
            this.buffer.push(transition);
        } else {
            this.buffer[this.position] = transition;
        }
        this.position = (this.position + 1) % this.capacity;
        this.size = Math.min(this.size + 1, this.capacity);
    }

    public sample(batchSize: number): Types.Transition[] {
        const batch: Types.Transition[] = [];
        for (let i = 0; i < batchSize; i++) {
            const index = Math.floor(Math.random() * this.size);
            batch.push(this.buffer[index]);
        }
        return batch;
    }

    public clear(): void {
        this.buffer = [];
        this.position = 0;
        this.size = 0;
    }
}

// ============================================================================
// Helper Functions
// ============================================================================

export function computeReturns(rewards: number[], gamma: number): number[] {
    const returns: number[] = [];
    let G = 0;

    for (let t = rewards.length - 1; t >= 0; t--) {
        G = rewards[t] + gamma * G;
        returns.unshift(G);
    }

    return returns;
}

export function computeAdvantages(
    rewards: number[],
    values: number[],
    nextValues: number[],
    dones: boolean[],
    gamma: number = 0.99,
    lambda: number = 0.95
): number[] {
    const advantages: number[] = [];
    let gae = 0;

    for (let t = rewards.length - 1; t >= 0; t--) {
        const delta = rewards[t] + gamma * nextValues[t] * (1 - (dones[t] ? 1 : 0)) - values[t];
        gae = delta + gamma * lambda * (1 - (dones[t] ? 1 : 0)) * gae;
        advantages.unshift(gae);
    }

    return advantages;
}

export function normalizeArray(arr: number[]): number[] {
    const mean = arr.reduce((a, b) => a + b, 0) / arr.length;
    const variance = arr.reduce((sum, x) => sum + Math.pow(x - mean, 2), 0) / arr.length;
    const std = Math.sqrt(variance) + 1e-8;

    return arr.map(x => (x - mean) / std);
}

// ============================================================================
// Example Usage
// ============================================================================

export const example = {
    /**
     * Example: Training a Q-Learning agent
     */
    trainQLearning: (env: Types.Environment) => {
        const agent = new QLearningAgent({
            stateSize: 64,
            actionSize: 4,
            learningRate: 0.1,
            discountFactor: 0.99,
            epsilon: 0.1,
            epsilonDecay: 0.995,
            epsilonMin: 0.01
        });

        const result = agent.train(1000, env);
        console.log(`Training completed in ${result.trainingTime}ms`);

        const evalResult = agent.evaluate(100, env);
        console.log(`Mean Reward: ${evalResult.metrics.meanReward.toFixed(2)}`);
        console.log(`Success Rate: ${(evalResult.metrics.successRate * 100).toFixed(1)}%`);

        return { agent, result, evalResult };
    }
};

// ============================================================================
// Default Export
// ============================================================================

export default {
    QLearningAgent,
    EpsilonGreedyExploration,
    SimpleReplayBuffer,
    computeReturns,
    computeAdvantages,
    normalizeArray,
    example
};
