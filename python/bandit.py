import numpy as na
import random
import scipy.stats

class Bandit:
    def __init__(self, narms):
        self.narms = narms
        self.arms = na.zeros(self.narms)
        self.actions = na.array(list(range(self.narms)))
        for i in self.actions:
            self.arms[i] = random.random()



    def sample(self, armnum):
        if random.random() < self.arms[armnum]:
            return 1
        else:
            return -1

def qlearning(bandit, iterations):
    Q = na.zeros(bandit.narms)
    epsilon = 0.1
    alpha = 0.01
    gamma = 0.8
    for iterations in range(iterations):
        if random.random() < epsilon:
            best_ai = random.choice(bandit.actions)
        else:
            best_ai = na.argmax(Q)
        r = bandit.sample(best_ai)
        Q[best_ai] = Q[best_ai] + alpha * (r + gamma * max(Q) - Q[best_ai])

        yield best_ai, r

def thompson_sampling(bandit, iterations):
    S = na.zeros(bandit.narms)
    F = na.zeros(bandit.narms)
    for iterations in range(iterations):
        sampled_params = []
        for a_i in bandit.actions:
            p = scipy.stats.beta.rvs(S[a_i] + 1, F[a_i] + 1)
            sampled_params.append(p)

        best_ai = na.argmax(sampled_params)

        r = bandit.sample(best_ai)

        if r == 1:
            S[best_ai] += 1
        else:
            F[best_ai] += 1
        yield best_ai, r

def ml_bandit(bandit, iterations):
    S = na.zeros(bandit.narms) + 1
    F = na.zeros(bandit.narms) + 1
    epsilon = 0.1

    for iterations in range(iterations):
        sampled_params = []
        for a_i in bandit.actions:
            p = S[a_i] / (S[a_i] + F[a_i])
            sampled_params.append(p)

        if random.random() < epsilon:
            best_ai = random.choice(bandit.actions)
        else:
            best_ai = na.argmax(sampled_params)

        r = bandit.sample(best_ai)

        if r == 1:
            S[best_ai] += 1
        else:
            F[best_ai] += 1
        yield best_ai, r

def random_agent(bandit, iterations):

    for iterations in range(iterations):
        best_ai = random.choice(bandit.actions)
        r = bandit.sample(best_ai)
            
        yield best_ai, r

def explore_then_exploit(bandit, iterations):
    S = na.zeros(bandit.narms)
    F = na.zeros(bandit.narms)

    for iterations in range(iterations):
        if iterations < 10 or len(na.nonzero(S)) == 0:
            best_ai = random.choice(bandit.actions)
            r = bandit.sample(best_ai)
            if r == 1:
                S[best_ai] += 1
            else:
                F[best_ai] += 1
        else:
            probabilities = [S[i] / (S[i] + F[i]) for i in bandit.actions]

            best_ai = na.argmax(probabilities)
            r = bandit.sample(best_ai)

        yield best_ai, r

def policy_search(bandit, iterations):
    policy_return = na.zeros(bandit.narms)
    policy_tries = na.zeros(bandit.narms)
    
    for iteration in range(iterations):
        if iteration <  100:
            ai = iteration % bandit.narms
            r = bandit.sample(ai)
            policy_return[ai] += r
            policy_tries[ai] += 1
            yield ai, r
        else:
            best_ai = na.argmax([policy_return[ai] / policy_tries[ai] for ai in bandit.actions])
            r = bandit.sample(best_ai)
            yield best_ai, r



    

def regularbandit():
    bandit = Bandit(3)
    bandit.arms = [0.2, 0.3, 0.7]
    print bandit.arms
    

    for method in [ml_bandit, explore_then_exploit, qlearning, thompson_sampling, random_agent, policy_search]:
     
    
        total_rewards = []
        for trials in range(30):
            total_reward = 0

            for a_i, r in method(bandit, 2000):
                total_reward += r
            total_rewards.append(total_reward)

        print method.__name__, na.mean(total_reward), "+/-", scipy.stats.sem(total_rewards)


def budgetedbandit():
    bandit = Bandit(3)
    bandit.arms = [0.2, 0.3, 0.7]
    print bandit.arms
    

    for method in [ml_bandit, thompson_sampling, random_agent]:
        total_rewards = []
        for trials in range(100):
            total_reward = 0
            for a_i, r in method(bandit, 50):
                total_reward += r
            total_rewards.append(total_reward)
            
        print method.__name__, na.mean(total_rewards), "+/-", scipy.stats.sem(total_rewards) * 1.96

def main():
    budgetedbandit()

if __name__ == "__main__":
    main()
