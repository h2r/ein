from scipy.special import betainc
import matplotlib.pyplot as plt; plt.rc('text', usetex=True)
import pylab as mpl
import numpy as na

def plot(successes, failures):
    a = successes + 1
    b = failures + 1

    X = na.arange(0, 1, 0.01)
    Y = [betainc(a, b, x) for x in X]
    mpl.plot(X, Y, label="successes=%d; failures=%d" % (successes, failures))

    if (successes + failures == 0):
        mu = 0.5
    else:
        mu = float(successes) / (successes + failures)
        
    muX = [mu]
    muY = [0.5]
    mpl.scatter(muX, muY, marker="o", color="k")  # plot mean given successes and failures
    mpl.ylabel("$\Pr(\mu <= x)$")
    mpl.xlabel("$x$")
    mpl.legend(loc="lower right")

def main():
    """
    Smoke test for incomplete beta.
    """
    plot(0, 0)
    plot(5, 5)
    plot(5, 10)
    plot(10, 5)
    mpl.title("Value of the incomplete beta function for different values of $x$")
    mpl.show()

if __name__ == "__main__":
    main()

