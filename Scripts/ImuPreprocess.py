# -*- coding:utf-8 -*-
# Create by steve in 17-4-14 at 上午11:03


import numpy as np
import scipy as sp

import matplotlib.pyplot as plt

# from OPENSHOE import zupt_test
from OPENSHOE import Setting, zupt_test, ZUPT

import array

import os


class ImuPreprocess:
    def __init__(self, source_data_file):
        '''
        Load data from file.
        :param source_data_file: 
        '''
        self.data = np.loadtxt(source_data_file, dtype=float, delimiter=',')
        # use rad replace deg
        self.data[:, 4:7] = self.data[:, 4:7] * np.pi / 180.0
        # use m/s^2 replace g.
        self.data[:, 1:4] = self.data[:, 1:4] * 9.81

        self.para = Setting.settings()
        self.para.sigma_a *= 5.0
        self.para.sigma_g *= 5.0

        self.para.sigma_acc *= 6.0
        self.para.sigma_gyro *= 6.0

        # self.para.Ts = 0.005 #np.mean(self.data[1:, 0] - self.data[:-1, 0])
        self.para.Ts = -1.0 * (self.data[1, 0] - self.data[self.data.shape[0] - 2, 0]) / float(self.data.shape[0])
        print("TS :", self.para.Ts)
        np.savetxt("../TMP_DATA/preprocess_data.csv", self.data, delimiter=',')
        # self.para.time_Window_size = 5

        return

    # def computeheigh(self):


    def computezupt(self):
        '''
        run zero velocity detector and zupt location.
         (If tmp_data/trace.txt and tmp_data/zupt_result.txt exists,
         Just load data from file to save time.)
        :return: 
        '''

        if "tmp_data" in os.listdir("./"):
            # print(os.listdir("./"))
            if False:  # "zupt_result.txt" in os.listdir("./tmp_data") and "trace.txt" in os.listdir("./tmp_data"):

                self.trace_x = np.loadtxt("./tmp_data/trace.txt")
                self.zupt_result = np.loadtxt("./tmp_data/zupt_result.txt")
                self.all_quat = np.loadtxt("./tmp_data/all_quat.txt")
                tmp_save = np.zeros([self.zupt_result.shape[0], 2])
                tmp_save[:, 0] = self.zupt_result[:]
                np.savetxt("../TMP_DATA/zupt_result.csv", tmp_save, delimiter=',')
                if self.trace_x.shape[0] == self.data.shape[0]:
                    plt.figure()
                    plt.title("ZUPT detector(quick load)")
                    plt.grid(True)
                    for i in range(1, 4):
                        plt.plot(self.data[:, i])
                    plt.plot(self.zupt_result * 80.0)

                    plt.figure()
                    plt.title("filter result (quick load)")
                    plt.plot(self.trace_x[:, 0], self.trace_x[:, 1], '.-')
                    plt.grid(True)

                    return

        else:
            os.mkdir("tmp_data")

        zero_velocity_detector = zupt_test.zv_detector(self.para)
        self.zupt_result = zero_velocity_detector.GLRT_Detector(self.data[:, 1:7])

        # debug

        plt.figure()
        plt.title("ZUPT detector")
        plt.grid(True)
        for i in range(1, 4):
            plt.plot(self.data[:, i])
        plt.plot(self.zupt_result * 80.0)
        # plt.show()

        # ZUPT
        ins_filter = ZUPT.ZUPTaidedInsPlus(self.para)

        ins_filter.init_Nav_eq(self.data[:30, 1:7])

        self.trace_x = np.zeros([9, self.data.shape[0]])
        self.all_quat = np.zeros([self.data.shape[0], 4])
        print("self .all quat :", self.all_quat)

        '''
        Try extrac 
        '''

        for index in range(self.data.shape[0]):
            # if (index > 2):
            #     ins_filter.para.Ts = self.data[index, 0] - self.data[index - 1, 0]
            # print(self.para.Ts)
            self.trace_x[:, index] = \
                ins_filter.GetPosition(self.data[index, 1:7],
                                       self.zupt_result[index]).reshape(18)[:9]

            self.all_quat[index, :] = ins_filter.quat1.reshape([1, -1])

        self.trace_x = self.trace_x.transpose()

        print(self.trace_x.shape)
        np.savetxt("./tmp_data/trace.txt", self.trace_x)
        np.savetxt("./tmp_data/zupt_result.txt", self.zupt_result)
        np.savetxt("./tmp_data/all_quat.txt", self.all_quat)
        tmp_save = np.zeros([self.zupt_result.shape[0], 1])
        tmp_save[:] = self.zupt_result[:]
        np.savetxt("../TMP_DATA/zupt_result.csv", tmp_save, delimiter=',')

        plt.figure()
        plt.title("filter result ")
        plt.plot(self.trace_x[:, 0], self.trace_x[:, 1], '.-')
        plt.grid(True)

        return

    def ShowMagnety(self):
        '''
        Find and show simple feature in sequence.
        :return: 
        '''
        plt.figure()
        plt.subplot(4, 1, 1)
        plt.title("norm")
        # print(np.linalg.norm(self.data[:,7:10],axis=-1).sh)
        plt.plot(np.linalg.norm(self.data[:, 7:10], axis=-1) ** 0.5)

        plt.subplot(4, 1, 2)
        plt.title("all")
        for i in range(3):
            plt.plot(self.data[:, i + 7], label=str(i))
        plt.legend()

        plt.subplot(4, 1, 3)
        plt.title(" pressure")
        plt.plot(self.data[:, 10] - np.mean(self.data[:, 10]))
        print(min(self.data[:, 11]), max(self.data[:, 11]))
        return

    def pressure2high(self, pressure):
        '''
        from pressure Pa to m. reduced a offset 165.0m
        :param pressure:(1013.5-imu(:,11)./100.0).*9.81
        :return:
        '''

        return (1013.5 - pressure / 100.0) * np.pi - 165.0

    def findvertex(self):
        '''
        Find vertex and compute edge(in trace graph not optimize graph).
        ((Actually also the vertex in optimize graph))
        :return: 
        '''

        vertex_point = array.array('d')
        vertex_quat = array.array('d')
        vertex_to_id = array.array('d')
        vertex_time = array.array('d')
        vertex_high = array.array('d')

        data_cols = 9

        for i in range(1, self.trace_x.shape[0]):
            if self.zupt_result[i] > 0.5 and self.zupt_result[i - 1] < 0.5:
                vertex_time.append(self.data[i, 0])
                if (self.data.shape[1] > 10):
                    vertex_high.append(self.pressure2high(self.data[i, 10]))

                for j in range(data_cols):
                    vertex_point.append(self.trace_x[i, j])
                for k in range(4):
                    vertex_quat.append(self.all_quat[i, k])
                    # vertex_point.append(self.trace_x[i,0])
                    # vertex_point.append(self.trace_x[i,1])
                    # vertex_point.append(self.trace_x[i,2])
                    # vertex_point
                vertex_to_id.append(i)
        self.vertics_high = np.frombuffer(vertex_high, dtype=np.float).reshape([-1])
        self.vertics = np.frombuffer(vertex_point, dtype=np.float)
        self.vertics = np.reshape(self.vertics, (-1, data_cols))
        self.vertex_quat = np.frombuffer(vertex_quat, dtype=np.float).reshape([-1, 4])
        self.vertics_id = np.frombuffer(vertex_to_id, dtype=np.float).reshape([-1])
        self.vertics_id = self.vertics_id.astype(dtype=np.int32)
        self.vertics_time = np.frombuffer(vertex_time, dtype=np.float).reshape([-1])

        np.savetxt("../TMP_DATA/vertex_pose.csv", self.vertics, delimiter=',')
        np.savetxt("../TMP_DATA/vertex_quat.csv", self.vertex_quat, delimiter=',')

        print(self.vertics[20, :])

        plt.figure()
        plt.title("show vertices in trace")
        plt.grid(True)

        plt.plot(self.trace_x[:, 0], self.trace_x[:, 1], 'r.-', label="trace")
        plt.plot(self.vertics[:, 0], self.vertics[:, 1], 'bo', label="vertex")

        plt.legend()

        return

    def findcorner(self):
        '''
        Find corner in vertex point.
        :return: 
        '''

        plt.figure()
        plt.title("show x y z (in findcorner)")

        for i in range(3):
            plt.plot(self.vertics[:, i], '-+', label=str(i))
        plt.grid(True)
        plt.legend()

        corner = array.array('f')
        corner_id = array.array('f')

        # plt.figure()
        # plt.title("\\delta y / \\delta x")
        tmp = (self.vertics[1:, 1] - self.vertics[:-1, 1]) / (self.vertics[1:, 0] - self.vertics[:-1, 1])
        plt.plot(tmp, label="\deltay / \delta x")
        plt.legend()
        plt.grid(True)

        threshold = 0.2
        length = 5

        itcounter = 0
        corner_index = -1

        for i in range(self.vertics.shape[0]):
            # if itcounter > 0:
            #     itcounter -= 1
            #     continue

            if i == 1 or i == self.vertics.shape[0] - 1:
                # for j in range(self.vertics.shape[1]):
                #     corner.append(self.vertics[i, j])
                itcounter = 0
            elif i < length + 1 or i > self.vertics.shape[0] - length - 1:
                itcounter = 0
                continue
            else:
                # if (np.abs(self.vertics[i, 0] -
                #                    (self.vertics[i - 1, 0] + self.vertics[i + 1, 0]) / 2.0) > threshold or
                #             np.abs(self.vertics[i, 1] -
                #                            (self.vertics[i - 1, 1] + self.vertics[i + 1, 1]) / 2.0) > threshold):
                if ((self.vertics[i, 0] - self.vertics[i - length, 0]) * (
                            self.vertics[i + length, 0] - self.vertics[i, 0]) < 0.0 and
                        (abs((self.vertics[i, 0] - self.vertics[i - length, 0]) + (
                                    self.vertics[i + length, 0] - self.vertics[i, 0])) > threshold
                         ) or (
                                    (self.vertics[i, 1] - self.vertics[i - length, 1]) * (
                                            self.vertics[i + length, 1] - self.vertics[i, 1]) < 0.0 and
                                abs((self.vertics[i, 1] - self.vertics[i - length, 1]) + (
                                            self.vertics[i + length, 1] - self.vertics[i, 1])) > threshold)
                    ):
                    itcounter += 1
                    # if itcounter < 3 or itcounter > 4:
                    #     continue
                    if itcounter == 1:
                        corner_index += 1
                    corner_id.append(corner_index)
                    corner_id.append(i)

                    for j in range(self.vertics.shape[1]):
                        corner.append(self.vertics[i, j])
                        # itcounter += length-3
                else:
                    itcounter = 0

        self.corner = np.frombuffer(corner, dtype=np.float32).reshape([-1, self.vertics.shape[1]])
        self.corner_id = np.frombuffer(corner_id, dtype=np.float32).reshape([-1, 2]).astype(np.int)

        # print("corner id :", self.corner_id)

        # for i in range(3):
        #     plt.plot(self.corner[:,i],'ro')

        plt.figure()
        plt.title("corner in vertex")
        plt.plot(self.vertics[:, 0], self.vertics[:, 1], 'r-+', label="vertics")
        plt.plot(self.corner[:, 0], self.corner[:, 1], 'bo', label="corner")

        plt.legend()
        plt.grid(True)

    def computeconerfeature(self):
        '''
        Compute corner feature and compara it 
        :return: 
        '''

        plt.figure()
        plt.title("plot vetex feature")

        plt.grid(True)

        # print(self.corner_id[:,0]==1)

        '''
        4 cols :
        front begin |front end |after begin| after end
        '''
        valid_length = 5

        self.feature_extract_range = np.zeros([np.max(self.corner_id[:, 0].astype(np.int)) + 1, 4])
        self.flag_point = np.zeros([self.feature_extract_range.shape[0], 2])
        self.flag_corner_id = np.zeros([self.feature_extract_range.shape[0]])
        self.fl = np.zeros([self.feature_extract_range.shape[0], 2])

        for i in range(self.feature_extract_range.shape[0]):
            first = -1
            last = -1

            # find first and last vertex id of this corner
            for tt in range(self.corner_id.shape[0]):
                if self.corner_id[tt, 0] == i:
                    if first == -1:
                        first = self.corner_id[tt, 1]

                    else:

                        # TODO: could be speed up
                        last = self.corner_id[tt, 1]
                        self.flag_point[i, :] = self.vertics[int((first + last) / 2), :2]
                        self.flag_corner_id[i] = int((first + last) / 2)

            self.fl[i, 0] = first
            self.fl[i, 1] = last
            # find index in source data

            self.feature_extract_range[i, 0] = self.vertics_id[first - valid_length]
            self.feature_extract_range[i, 1] = self.vertics_id[first]
            self.feature_extract_range[i, 2] = self.vertics_id[last]
            self.feature_extract_range[i, 3] = self.vertics_id[last + valid_length]

        self.feature_extract_range = self.feature_extract_range.astype(dtype=np.int)
        self.fl = self.fl.astype(dtype=np.int)

        # print("feature range : \n",self.feature_range)

        '''
        Extract feature 
        '''

        self.feature = np.zeros([self.feature_extract_range.shape[0], 12])

        # x y z -axis
        # for i in range(3):

        # before mean std
        # self.feature[i*2,0] = np.mean(self.data[
        #                                   self.feature_extract_range[]])


        # after mean std

        for index in range(self.feature.shape[0]):

            for i in range(3):
                i = int(i)

                self.feature[index, i * 2] = np.mean(
                    self.data[
                    self.feature_extract_range[index, 0]:self.feature_extract_range[index, 1]
                    , i + 7
                    ]
                )
                self.feature[index, i * 2 + 1] = np.std(
                    self.data[
                    self.feature_extract_range[index, 0]:self.feature_extract_range[index, 1]
                    , i + 7
                    ]
                )

                self.feature[index, i * 2 + 6] = np.mean(
                    self.data[
                    self.feature_extract_range[index, 2]:self.feature_extract_range[index, 3]
                    , i + 7
                    ]
                )
                self.feature[index, i * 2 + 1 + 6] = np.std(
                    self.data[
                    self.feature_extract_range[index, 2]:self.feature_extract_range[index, 3]
                    , i + 7
                    ]
                )

        # for i in range(self.feature.shape[0]):
        #     for j in range(self.feature.shape[1]):


        # print("feature:",self.feature.shape)

        ''''
        Compara features
        '''

        feature_threold = 100

        # try to get abs
        self.feature = np.abs(self.feature)

        self.distance = np.zeros([self.feature.shape[0], self.feature.shape[0]])
        for i in range(self.distance.shape[0]):
            for j in range(self.distance.shape[1]):
                self.distance[i, j] = np.linalg.norm(self.feature[i, :] - self.feature[j, :])

        # print("distantce:\n", self.distance)

        plt.contourf(self.distance)

        plt.figure()
        plt.title("relation ship ")

        plt.plot(self.vertics[:, 0], self.vertics[:, 1], 'b*-')

        close_vetices_num = 0

        for i in range(self.distance.shape[0]):
            for j in range(i + 1, self.distance.shape[1]):
                if i == j:
                    continue
                elif self.distance[i, j] < feature_threold:
                    close_vetices_num += 1
                    plt.plot([self.flag_point[i, 0], self.flag_point[j, 0]],
                             [self.flag_point[i, 1], self.flag_point[j, 1]],
                             'r-')

        '''
        Save result to csv_file
        '''

        close_vetices = np.zeros([int(close_vetices_num), 2])
        print("close shape:", close_vetices.shape)
        index = 0

        for i in range(self.distance.shape[0]):
            for j in range(i + 1, self.distance.shape[1]):
                if self.distance[i, j] < feature_threold:
                    # close_vetices[index, 0] = int(
                    #     (self.feature_extract_range[i, 1] + self.feature_extract_range[i, 2]) / 2)
                    # close_vetices[index, 1] = int(
                    #     (self.feature_extract_range[j, 1] + self.feature_extract_range[j, 2]) / 2)
                    close_vetices[index, 0] = self.flag_corner_id[i]
                    close_vetices[index, 1] = self.flag_corner_id[j]
                    index += 1

        print(close_vetices)
        np.savetxt("../TMP_DATA/close_vetices_num.csv", close_vetices, delimiter=',')

        '''
        Save all vertex in corner.
        '''
        close_plus = array.array('f')

        for i in range(self.distance.shape[0]):
            for j in range(i + 1, self.distance.shape[1]):
                if self.distance[i, j] < feature_threold:
                    for ix in range(self.fl[i, 0], self.fl[i, 1]):
                        for iy in range(self.fl[j, 0], self.fl[j, 1]):
                            close_plus.append(ix)
                            close_plus.append(iy)

        close_full = np.frombuffer(close_plus, dtype=np.float32).reshape([-1, 2]).astype(dtype=int)
        np.savetxt("../TMP_DATA/close_vetices_num_full.csv", close_full, delimiter=',')


        # plt.legend()

    def findstraightline(self):
        '''
        
        :return: false is some error occur
        '''

        # print("feature range",self.feature_extract_range)
        # print("corner range:",self.corner)
        self.line_range = np.zeros([self.feature_extract_range.shape[0] + 2, 2])

        # print("corner id :" , self.corner_id)
        current_corner_id = 0

        self.line_range[0, 0] = 0
        self.line_range[0, 1] = self.corner_id[0, 1]
        self.line_range[self.line_range.shape[0] - 1, 0] = self.corner_id[-1, 1]
        self.line_range[self.line_range.shape[0] - 1, 1] = self.vertics.shape[0] - 1

        for k in range(self.corner_id.shape[0]):

            if self.corner_id[k, 0] == current_corner_id:  # and self.corner_id[k + 1, 0] == current_corner_id + 1:

                if k > 0:
                    self.line_range[current_corner_id + 1, 0] = self.corner_id[k - 1, 1]
                self.line_range[current_corner_id + 1, 1] = self.corner_id[k, 1]

                current_corner_id += 1

        plt.figure()
        plt.title("line detector ")
        plt.grid(True)

        plt.plot(self.vertics[:, 0], self.vertics[:, 1], 'r*', label="vertex")

        for i in range(self.line_range.shape[0]):
            plt.plot(self.vertics[int(self.line_range[i, 0]):int(self.line_range[i, 1]), 0],
                     self.vertics[int(self.line_range[i, 0]):int(self.line_range[i, 1]), 1], 'y')

        # print("self.line_range:", self.line_range)
        np.savetxt("../TMP_DATA/line_range_file.csv", self.line_range, delimiter=',')

        return True

    def findSimpleFFT(self):
        print('data shape :', self.data.shape)

        offset = 100

        index = 0

        data_rows = 20

        fft_feature = np.zeros([int(self.data.shape[0] / offset), data_rows * 2])

        while (True):
            if index + offset < self.data.shape[0]:

                # print("index:", np.fft.fft(np.linalg.norm(self.data[7:10], axis=0)))

                fft_feature[int(index / offset), :data_rows] = np.imag(
                    np.fft.fft(np.linalg.norm(self.data[index:index + offset, :], axis=1), data_rows))
                fft_feature[int(index / offset), -data_rows:] = np.real(
                    np.fft.fft(np.linalg.norm(self.data[index:index + offset, :], axis=1), data_rows)
                )

                fft_feature[int(index / offset), :data_rows] /= np.linalg.norm(
                    fft_feature[int(index / offset), :data_rows])

                fft_feature[int(index / offset), -data_rows:] /= np.linalg.norm(
                    fft_feature[int(index / offset), -data_rows:])

                index += offset


            else:
                break

        # compute related distance

        fft_distance = np.zeros([fft_feature.shape[0], fft_feature.shape[0]])

        for i in range(fft_feature.shape[0]):
            # print(fft_feature[i, :])
            for j in range(fft_feature.shape[0]):
                fft_distance[i, j] = np.sum(np.abs(fft_feature[i, :] - fft_feature[j, :]))

        plt.figure()
        plt.title("fft distance")
        # plt.imshow(fft_distance)
        plt.contourf(fft_distance)

        print("average:min:max   of fft distance :", np.mean(fft_distance), np.min(fft_distance), np.max(fft_distance))

        return True


if __name__ == '__main__':
    ip = ImuPreprocess(source_data_file="../TMP_DATA/all_data3.csv")

    ip.computezupt()

    ip.ShowMagnety()

    ip.findvertex()

    ip.findcorner()

    ip.computeconerfeature()

    ip.findstraightline()

    ip.findSimpleFFT()

    plt.show()
