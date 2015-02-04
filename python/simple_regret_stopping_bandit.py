import numpy as na
import random
import scipy.stats
import pylab as mpl
from scipy.special import betainc

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
    def __init__(self, confidence=95):
        self.upperbound = 0.8
        self.confidence = confidence
        self.confidence_map = {
            1: 1,
            90: 1.645,
            95: 1.96,
            99: 2.575}
        self.confidence_factor = self.confidence_map[confidence]


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

                arm_confidence = arm_stderr * self.confidence_factor
                if (arm_mean + arm_confidence)  < self.upperbound:
                    break # this arm sucks; next arm
                elif (arm_mean - arm_confidence > self.upperbound):
                    return #this arm is awesome; leave
                else:
                    # continue trying this arm
                    pass

    @property
    def marginals(self):
        return [s/(s + f) if (s + f != 0) else 0.5 for (s, f) in zip(self.S, self.F)]
        
    def bestAction(self):
        return na.argmax(self.marginals)

    def actionDistribution(self):
        #return self.marginals / na.sum(self.marginals)
        delta = na.zeros(len(self.S))
        delta[self.bestAction()] = 1
        return delta
    def __str__(self):
        return "Algorithm B: " + str(self.confidence)




class AlgorithmC(Policy):
    def __init__(self, confidence=95):
        self.upperbound = 0.8
        self.confidence = confidence  / 100.0


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
                Pr_mu_less_than_bound = betainc(self.S[a_i] + 1, self.F[a_i] + 1, self.upperbound)
                Pr_mu_greater_than_bound = 1 - Pr_mu_less_than_bound
                if Pr_mu_less_than_bound  >= self.confidence:
                    break # this arm sucks; next arm
                elif Pr_mu_greater_than_bound >= self.confidence:
                    print
                    print "arm", a_i
                    print "arm_mean", arm_mean
                    print "s", self.S
                    print "f", self.F
                    return #this arm is awesome; leave
                else:
                    # continue trying this arm
                    pass

    @property
    def marginals(self):
        return [s/(s + f) if (s + f != 0) else 0.5 for (s, f) in zip(self.S, self.F)]
        
    def bestAction(self):
        return na.argmax(self.marginals)

    def actionDistribution(self):
        #return self.marginals / na.sum(self.marginals)
        delta = na.zeros(len(self.S))
        delta[self.bestAction()] = 1
        return delta
    def __str__(self):
        return "Algorithm C: " + str(self.confidence)



class Stochastic(Policy):
    def __init__(self, n, confidence):
        self.n = n
        self.upperbound = 0.8
        self.confidence = confidence  / 100.0

    def train(self, bandit, max_budget):
        self.S = na.zeros(bandit.narms) * 0.0
        self.F = na.zeros(bandit.narms) * 0.0
        

        for a_i in bandit.actions:
            for i in range(self.n):
                if len(bandit.log) >= max_budget:
                    return
                r = bandit.sample(a_i)
                if r == 1:
                    self.S[a_i] += 1.0
                else:
                    self.F[a_i] += 1.0
            ntrials = self.S[a_i] + self.F[a_i]

            arm_mean = self.S[a_i] / ntrials
            Pr_mu_less_than_bound = betainc(self.S[a_i] + 1, self.F[a_i] + 1, self.upperbound)
            Pr_mu_greater_than_bound = 1 - Pr_mu_less_than_bound
            if Pr_mu_greater_than_bound >= self.confidence:
                print
                print "arm", a_i
                print "arm_mean", arm_mean
                print "s", self.S
                print "f", self.F
                return #this arm is awesome; leave
            else:
                # continue trying this arm
                pass

    @property
    def marginals(self):
        return [s/(s + f) if (s + f != 0) else 0.5 for (s, f) in zip(self.S, self.F)]
        
    def bestAction(self):
        return na.argmax(self.marginals)

    def actionDistribution(self):
        #return self.marginals / na.sum(self.marginals)
        delta = na.zeros(len(self.S))
        delta[self.bestAction()] = 1
        return delta
    def __str__(self):
        return "Stochastic n=%d, %.2f" % (self.n, self.confidence)



class StochasticEarlyStopping(Policy):
    def __init__(self, n, confidence):
        self.n = n
        self.upperbound = 0.8
        self.confidence = confidence  / 100.0

    def train(self, bandit, max_budget):
        self.S = na.zeros(bandit.narms) * 0.0
        self.F = na.zeros(bandit.narms) * 0.0
        

        for a_i in bandit.actions:
            for i in range(self.n):
                if len(bandit.log) >= max_budget:
                    return
                r = bandit.sample(a_i)
                if r == 1:
                    self.S[a_i] += 1.0
                else:
                    self.F[a_i] += 1.0

                Pr_mu_less_than_bound = betainc(self.S[a_i] + 1, self.F[a_i] + 1, self.upperbound)
                ntrials = self.S[a_i] + self.F[a_i]
                arm_mean = self.S[a_i] / ntrials
                Pr_mu_greater_than_bound = 1 - Pr_mu_less_than_bound
                if Pr_mu_less_than_bound >= self.confidence:
                    break
            if Pr_mu_greater_than_bound >= self.confidence:
                print
                print "arm", a_i
                print "arm_mean", arm_mean
                print "s", self.S
                print "f", self.F
                return #this arm is awesome; leave
            else:
                # continue trying this arm
                pass

    @property
    def marginals(self):
        return [s/(s + f) if (s + f != 0) else 0.5 for (s, f) in zip(self.S, self.F)]
        
    def bestAction(self):
        return na.argmax(self.marginals)

    def actionDistribution(self):
        #return self.marginals / na.sum(self.marginals)
        delta = na.zeros(len(self.S))
        delta[self.bestAction()] = 1
        return delta
    def __str__(self):
        return "StochasticEarlyStopping n=%d, %.2f" % (self.n, self.confidence)



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
    #algorithmB = AlgorithmB(confidence=95)
    #algorithmC95 = AlgorithmC(confidence=95)
    #algorithmC99 = AlgorithmC(confidence=99)
    stochastic5 = Stochastic(n=5, confidence=95)
    stochastic2 = Stochastic(n=2, confidence=95)
    stochasticEarlyStopping5 = StochasticEarlyStopping(n=5, confidence=95)

    for method in [thompson_sampling, stochastic5, stochastic2, stochasticEarlyStopping5]:
        results = []
        for budget in na.arange(0, 110, 10):
            regrets = []
            budgets = []
            for iteration in range(25):
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
