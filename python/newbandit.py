import numpy as na
import random
import scipy.stats

class Bandit(object):

    def randomBandit(narms):
        arms = na.random.rand(narms)
        return Bandit(arms)

    def __init__(self, arms):
        self.arms = arms
        self.narms = len(arms)
        self.actions = na.array(list(range(self.narms)))
        self.best_ai = na.argmax(self.arms)
        

    def sample(self, armnum):
        if random.random() < self.arms[armnum]:
            return 1
        else:
            return -1

    def expectedReturn(self, actionDistribution):
        assert len(actionDistribution) == self.narms
        return na.dot(actionDistribution, self.arms)

    def regret(self, actionDistribution):
        return self.arms[self.best_ai] - self.expectedReturn(actionDistribution)

class LoggedBandit(object):
    def __init__(self, narms):
        self.bandit = Bandit(narms)
        self.log = []

    def reset(self):
        self.log = []

    def sample(self, armnum):
        result = self.bandit.sample(armnum)
        self.log.append(armnum, result)
    def regret(self, actionDistribution):
        return self.bandit.regret(actionDistribution)

class Policy(object):
    
    def train(self, bandit):
        raise NotImplementedError()

    def selectAction(self, bandit):
        raise NotImplementedError()

class ThompsonSampling(Policy):
    def __init__(self):
        pass

    def train(self, bandit, max_budget):
        self.S = na.zeros(bandit.narms)
        self.F = na.zeros(bandit.narms)
        for iterations in range(max_budget):
            sampled_params = []
            for a_i in bandit.actions:
                p = scipy.stats.beta.rvs(self.S[a_i] + 1, self.F[a_i] + 1)
                sampled_params.append(p)

            best_ai = na.argmax(sampled_params)

            r = bandit.sample(best_ai)

            if r == 1:
                self.S[best_ai] += 1
            else:
                self.F[best_ai] += 1

        self.marginals = [(s + 1)/(s + f + 2) for (s, f) in zip(self.S, self.F)]
    def bestAction(self):
        return na.argmax(self.marginals)

    def actionDistribution(self):
        #return self.marginals / na.sum(self.marginals)
        delta = na.zeros(len(self.S))
        delta[self.bestAction()] = 1
        return delta
        

def main():
    bandit = Bandit([0.2, 0.3, 0.7])

    print bandit.arms

    thompson_sampling = ThompsonSampling()
    for method in [thompson_sampling]:
        method.train(bandit, 3)
        
        regret = bandit.regret(method.actionDistribution())
        print regret
        


if __name__ == "__main__":
    main()
