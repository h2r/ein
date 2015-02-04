from scipy.special import betainc, beta
import pylab as mpl
import numpy as na


def plot(S, F):
    a = S + 1
    b = F + 1

    X = na.arange(0, 1, 0.1)
    Y = [betainc(a, b, x) * beta(a, b) for x in X]
    mpl.plot(X, Y, label="S=%d; F=%d" % (S, F))
    mpl.legend()

def main():
    plot(0, 0)
    mpl.show()

if __name__ == "__main__":
    main()

