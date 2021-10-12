#!/usr/bin/env python
#%% imports

import os
BASE_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))

# use local cython installs
import sys
#sys.path.append(f"{BASE_PATH}/gtsam/install/cython")
import gtsam
import inspect
import numpy as np
import math

import ics 
import matplotlib.pyplot as plt


#object_methods = [method_name for method_name in dir(gtsam)
#                  if callable(getattr(gtsam, method_name))]
#print(object_methods)

object_methods = [method_name for method_name in dir(ics)
                 if callable(getattr(ics, method_name))]
print(object_methods)

ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))
PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))
priorMean = gtsam.Pose2(0.0, 0.0, 0.0)  # prior at origin

def classlookup(cls):
     c = list(cls.__bases__)
     for base in c:
         c.extend(classlookup(base))
     return c

d = classlookup(ics.QuadraticUnaryFactor1D)
print(d)

def main():
    print("calling main")
    graph = gtsam.NonlinearFactorGraph()
    params_isam2 = gtsam.ISAM2Params()
    optimizer = gtsam.ISAM2(params_isam2)

    pose_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1.]))
   

    # NOTE: these calls seem to have been deprecated
    # unary_noise_model = gtsam.noiseModel_Gaussian.Covariance(cov_mat_unary)
    # binary_noise_model = gtsam.noiseModel_Gaussian.Covariance(cov_mat_binary)
    
    factor = ics.QuadraticUnaryFactor1D(gtsam.symbol('x', 0), np.array([0.]), 0, pose_noise)
    factor.printfromnonlinearfactor()
    print(factor.isConstraintFactor())
    help(factor)
    #graph.add(gtsam.PriorFactorPose2(1, priorMean, PRIOR_NOISE))


    graph.add(ics.QuadraticUnaryFactor1D(gtsam.symbol('x', 0), np.array([0.]), 0, pose_noise))
    graph.add(ics.QuadraticUnaryFactor1D(gtsam.symbol('x', 1), np.array([1.]), 0, pose_noise))
    graph.add(ics.QuadraticUnaryFactor1D(gtsam.symbol('x', 2), np.array([1.]), 0, pose_noise))
    graph.add(ics.QuadraticUnaryFactor1D(gtsam.symbol('x', 3), np.array([3.]), 0, pose_noise))

    # create graph: binary factors
    graph.add(ics.QuadraticBinaryFactor1D(gtsam.symbol('x', 0), gtsam.symbol('x', 1), np.array([1.]), pose_noise))
    graph.add(ics.QuadraticBinaryFactor1D(gtsam.symbol('x', 1), gtsam.symbol('x', 2), np.array([1.]), pose_noise))
    graph.add(ics.QuadraticBinaryFactor1D(gtsam.symbol('x', 2), gtsam.symbol('x', 3), np.array([1.]), pose_noise))

    # init values
    initial_estimate = gtsam.Values()
    initial_estimate.insert(gtsam.symbol('x', 0), np.array([0.]))
    initial_estimate.insert(gtsam.symbol('x', 1), np.array([0.]))
    initial_estimate.insert(gtsam.symbol('x', 2), np.array([0.]))
    initial_estimate.insert(gtsam.symbol('x', 3), np.array([0.]))


    optimizer.update(graph, initial_estimate)
    result = optimizer.calculateEstimate()
    print(result)





if __name__=='__main__':
    main()

