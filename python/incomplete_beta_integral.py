from scipy.special import betainc, beta
import matplotlib.pyplot as plt; plt.rc('text', usetex=True)
import pylab as mpl
import numpy as na

def regularized_incomplete_beta(a, b, x):
    return betainc(a, b, x) * beta(a, b)

def plot(S, F):
    a = S + 1
    b = F + 1

    X = na.arange(0, 1, 0.1)
    Y = [regularized_incomplete_beta(a, b, x) for x in X]
    mpl.plot(X, Y, label="S=%d; F=%d" % (S, F))
    mpl.ylabel("$\Pr(\mu <= x)$")
    mpl.xlabel("$x$")
    mpl.legend()

def main():
    plot(0, 0)
    mpl.show()

if __name__ == "__main__":
    main()

