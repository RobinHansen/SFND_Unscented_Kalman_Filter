#!/usr/bin/env python3

import sys, matplotlib.pyplot as plt

if len(sys.argv) > 1:
    epsilon = float(sys.argv[2]) if len(sys.argv) > 2 else 7.8
    NIS = [float(e) for e in open(sys.argv[1]).read().split(',') if e != '']
    plt.plot(range(len(NIS)), [epsilon]*len(NIS), label="Epsilon; ꭓ².050")
    plt.plot(range(len(NIS)), NIS, label="NIS")
    plt.xlabel("k")
    plt.ylabel("ε epsilon")
    plt.text(len(NIS),epsilon,f"X².050={epsilon}")
    plt.title(sys.argv[1])
    plt.show()
else:
    print(f"Usage {sys.argv[0]}: csv-file [epsilon]")