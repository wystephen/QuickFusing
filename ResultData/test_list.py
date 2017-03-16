import numpy as np
import os

if __name__ == '__main__':
    # for exam_id,particle_num,t_sigma,eval_sigma in ([3,5],[1,2,3,4],[0.1,0.3,0.5],[0.1,0.5,10]):
    #     print(exam_id,particle_num,t_sigma,eval_sigma)
    for exam_id in [5, 3]:
        for particle_num in [100, 500, 1000, 2000, 2500, 3000, 5000, 8000, 10000, 15000, 20000, 30000, 40000, 50000,
                             60000]:
            for t_sigma in [0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 10.0]:
                for eval_sigma in [1.0, 2.0, 3.0, 4.0, 5.0, 10.0]:
                    for times in range(1, 50):
                        print(exam_id, particle_num, t_sigma, eval_sigma, times)
                        file_name = "/home/steve/locate/RunResult/{0}/{1}-{2}-{3}/{4}-".format(
                            exam_id,
                            particle_num,
                            t_sigma,
                            eval_sigma,
                            times
                        )
                        os.makedirs(file_name + "all")
                        os.system("../cmake-build-debug/pf_test {0} {1} {2} {3} {4} {5} {6} {7} {8}".format(
                            3,
                            int(10000),
                            float(t_sigma),
                            float(eval_sigma),
                            int(particle_num),
                            float(t_sigma),
                            float(eval_sigma),
                            exam_id,
                            file_name
                        ))
