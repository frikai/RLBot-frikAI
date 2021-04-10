from rlbot.utils.structures.game_data_struct import Rotator

from util.vec import Vec3
from util.math import Angles
import numpy as np
import scipy.linalg as la

from util.orientation import Orientation

m = [  [0.7231255, -0.6889964, -0.0487172],
  [-0.6020598, -0.5941690, -0.5333735],
  [0.3385462,  0.4150266, -0.8444758 ]]

x = np.array([0.74, -0.59, 0.33])
y = np.array([-0.67, -0.61, 0.42])
z = np.array([-0.05, -0.53, -0.85])

A = np.array(m)
print(A.dot(np.array([1, 0, 0])))
print("...maybe?")

#o = Orientation(Rotator(0, 0, 0))
#print(o.to_matrix())
#
# # two input matrices
# mat1 = [[1, 0, 0],
#        [0, 2, 0],
#        [5, 0, 3]]
# mat2 = [[-0.279, -0.853, -0.441],
#         [-0.853, 0.431, -0.294],
#         [0.441, 0.294, -0.848]]
# A = np.array(mat1)
# B = np.array(mat2)
# v = np.array([1, 2, 3])
# print(v.dot(A))
# print(A.dot(v))
# # A transposed * B, we want the rotation from A to B. might need to swap if fixed or not or sth
# # C is our rot matrix
#
# C = A.T.dot(B)
#
# # print(A)
# # print(B)
# # print(np.round(C, 5))
# #
# # # get only real eigenvector of C (EV should be close to 1). note: EV are vertical! could just copy code below
# # eig = np.linalg.eig(C)
# # for i in range(len(eig[0])):
# #     if eig[0][i].real == eig[0][i]:
# #         print(eig[0][i])
# #         vec = [line[i].real for line in eig[1]]
# #         print(vec)
# # angle = np.arccos((np.trace(C)-1)/2)
# # print(Angles.rad_to_deg(angle))