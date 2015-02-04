import numpy as na
import random
import scipy.stats
import pylab as mpl

class Bandit(object):

    def randomBandit(narms):
        arms = na.random.rand(narms)
        return Bandit(arms)

    def __init__(self, arms):
        self.arms = arms
        self.narms = len(arms)
        self.actions = na.array(list(range(self.narms)))
        self.best_ai = na.argmax(self.arms)
        self.log = []

    def sample(self, armnum):

        if random.random() < self.arms[armnum]:
            result = 1
        else:
            result = -1
        self.log.append((armnum, result))
        return result

    def reset_log(self):
        self.log = []



    def expected_return(self, actionDistribution):
        assert len(actionDistribution) == self.narms
        return na.dot(actionDistribution, self.arms)

    def regret(self, actionDistribution):
        return self.arms[self.best_ai] - self.expected_return(actionDistribution)


class Policy(object):
    
    def train(self, bandit):
        raise NotImplementedError()

    def selectAction(self, bandit):
        raise NotImplementedError()


class AlgorithmB(Policy):
    def __init__(self):
        self.upperbound = 0.8


    def train(self, bandit, max_budget):
        self.S = na.zeros(bandit.narms) * 0.0
        self.F = na.zeros(bandit.narms) * 0.0
        
        iteration = 0

        for a_i in bandit.actions:
            while True:
                if iteration >= max_budget:
                    return

                r = bandit.sample(a_i)
                iteration += 1
                if r == 1:
                    self.S[a_i] += 1.0
                else:
                    self.F[a_i] += 1.0
                ntrials = self.S[a_i] + self.F[a_i]
                if ntrials == 1:
                    continue
                arm_mean = self.S[a_i] / ntrials
                arm_stderr = na.sqrt(arm_mean * (1 - arm_mean) / ntrials)
                arm_confidence = arm_stderr * 1.96
                if (arm_mean + arm_confidence)  < self.upperbound:
                    break # this arm sucks; next arm
                elif (arm_mean - arm_confidence > self.upperbound):
                    return #this arm is awesome; leave
                else:
                    # continue trying this arm
                    pass

    @property
    def marginals(self):
        return [s/(s + f + 2) for (s, f) in zip(self.S, self.F)]
        
    def bestAction(self):
        return na.argmax(self.marginals)

    def actionDistribution(self):
        #return self.marginals / na.sum(self.marginals)
        delta = na.zeros(len(self.S))
        delta[self.bestAction()] = 1
        return delta
    def __str__(self):
        return "Algorithm B"

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
    def __str__(self):
        return "Thompson Sampling"
        

def main():

    
    easy_bandit = Bandit([0.1, 0.1, 0.9, 0.1, 0.9, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])


    
    figure = mpl.figure()
    bandit = na.zeros(20) + 0.1
    bandit[1] = 0.9  # Thompson
    bandit = Bandit(bandit)
    plotBandit(bandit, figure.gca())
    figure.suptitle("Easy Object")

    figure = mpl.figure()
    bandit = na.zeros(20) + 0.1
    bandit[len(bandit)/2] = 0.9
    bandit = Bandit(bandit)
    plotBandit(bandit, figure.gca())
    figure.suptitle("Hard Object")


    figure = mpl.figure()
    bandit = na.zeros(20) + 0.1
    bandit[-1] = 0.9
    bandit = Bandit(bandit)
    plotBandit(bandit, figure.gca())
    figure.suptitle("Pathological Object")

    mpl.show()


def plotBandit(bandit, axes):

    thompson_sampling = ThompsonSampling()
    algorithmB = AlgorithmB()
    for method in [thompson_sampling, algorithmB]:
        results = []
        for budget in na.arange(0, 110, 10):
            regrets = []
            budgets = []
            for iteration in range(50):
                bandit.reset_log()
                method.train(bandit, budget)
                budgets.append(len(bandit.log))
                if len(bandit.log) > budget:
                    raise ValueError("Someone is cheating:" + `len(bandit.log)` + " budget: " + `budget`)
                regret = bandit.regret(method.actionDistribution())
                regrets.append(regret)

            results.append((budgets, regrets))
        
        X = [na.mean(b) for (b, r) in results]
        Y = [na.mean(r) for (b, r) in results]
        xerr = [scipy.stats.sem(b) * 1.96 for b, r in results]
        yerr = [scipy.stats.sem(r) * 1.96 for b, r in results]
        mpl.errorbar(X, Y, label=str(method), xerr=xerr, yerr=yerr)

    mpl.ylabel("Simple Regret")
    mpl.xlabel("Training Trials")
    mpl.axis((0, 105, -0.1, 1))
    mpl.legend()

if __name__ == "__main__":
    main()
