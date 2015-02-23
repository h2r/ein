#!/usr/bin/env python
import numpy as na

class Entry:
    def __init__(self, name, prior, training, marginal):
        self.name = name
        
        self.prior = tuple(prior)
        self.prior_rate = float(self.prior[0]) / self.prior[1]

        self.training = tuple(training)
        self.training_rate = float(self.training[0]) / self.training[1]

        self.marginal = tuple(marginal)
        self.marginal_rate = float(self.marginal[0]) / self.marginal[1]

    def __str__(self):
        return (self.name.ljust(20) + "\t\t" + 
                str(self.prior).rjust(10) + "\t\t" + 
                str(self.training).rjust(10) + "\t\t" + 
                str(self.marginal).rjust(10))

    def __repr__(self):
        return "Entry(%s, %s, %s, %s)" % (repr(self.name), 
                                          repr(self.prior),
                                          repr(self.training),
                                          repr(self.marginal))

def main():
    data = []
    for line in open("results.txt"):
        if line.strip() != "":
            stuff = [x.strip() for x in line.split("\t") if x.strip() != ""]
            name = stuff[0]
            numbers = [[int(y) for y in x.split("/")] for x in stuff[1:]]
            assert len(numbers) == 3
            data.append(Entry(name, numbers[0], numbers[1], numbers[2]))
            
    print data
    data.sort(key=lambda x: x.marginal_rate)
    block0 = (0, len(data)/3)
    block1 = (len(data)/3, 2 * len(data)/3)
    block2 = (2* len(data)/3, len(data))


    for i, e in enumerate(data):

        print str(i).ljust(3), e

    print "prior"
    print na.sum([e.prior[0] for e in data]), "/",
    print na.sum([e.prior[1] for e in data])

    print "marginal"
    print na.sum([e.marginal[0] for e in data]), "/",
    print na.sum([e.marginal[1] for e in data])

    
if __name__ == "__main__":
    main()
