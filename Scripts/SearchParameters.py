
import numpy as np
import os

if __name__ == '__main__':
    index = 0
    while index < 100000:
        index += 1

        os.system("../cmake-build-debug/pf_test {0} {1} {2} {3} {4} {5} {6}".format(
            3,
            int(np.random.uniform(2000,7000)),
            float(np.random.uniform(0.1,2.0)),
            float(np.random.uniform(1.0,20.0)),
            int(np.random.uniform(3000,80000)),
            float(np.random.uniform(0.1,3.0)),
            float(np.random.uniform(0.5,20.0)),
            5
        ))
        print("next ...")