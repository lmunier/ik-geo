# Not testing for correctness of anything, that is already done in the Rust library.
# Just making sure all the test cases return results, and this should be enough to ensure that the Python bindings are working correctly.
# The results are different from these test cases, but this makes sense as the test case solution were generated with different code and there are multiple solutions to the problem.


# Import the test case csv files
generalFilenames = ['IK_2_intersecting', 'IK_2_parallel', 'IK_3_parallel_2_intersecting', 'IK_3_parallel', 'IK_gen_6_dof', 'IK_spherical', 'IK_spherical_2_parallel', 'IK_spherical_2_intersecting']
hardcodedFilenames = ['IRB_6640', 'KUKA_R800_fixed_q3', "RRC_fixed_q6", "spherical_bot", "three_parallel_bot", "two_parallel_bot", "ur5", "yumi_fixed_q3"]

from collections import namedtuple
import numpy as np
import unittest
import ik_python
from math import pi

# Import the csv files
import csv
import os

TestBot = namedtuple('TestBot', ['casename', 'robot', 'testcases'])
TestCaseGeneral = namedtuple('TestCaseGeneral', ['hVals','pVals'])
TestCaseHardCoded = namedtuple('TestCaseHardCoded', ['rotationMatrix', 'positionVector'])


zeroresult = [0.0] * 6

epsilon = 1e-2
# Test the general robots
class TestGeneralRobots(unittest.TestCase):
    def setUp(self):
        self.bots = []
        for filename in generalFilenames:
            # Setup the robot from the filename
            robot = ik_python.Robot(filename.replace('IK_', '').replace('_', '').replace("2","two").replace("3","three").replace("6","six"))

            testcases = []
            # Open ../../test_cases/FILENAME.csv
            with open(os.path.join(os.path.dirname(__file__), '..', '..', 'test_cases', filename + '.csv')) as f:
                reader = csv.reader(f)
                # Skip the header
                next(reader)
                for row in reader:
                    if not row:
                        continue
                    # Parse the given h and p vals to get a valid robot configuration
                    hVals = [float(x) for x in row[0:18]]
                    pVals = [float(x) for x in row[18:39]]
                    testcases.append(TestCaseGeneral(hVals, pVals))
            self.bots.append(TestBot(filename, robot, testcases))

    def test_general_robots(self):
        np.random.seed(0)
        for botNum, bot in enumerate(self.bots):
            print ("Testing " + bot.casename)
            for testCase in bot.testcases:
                hMatrix = np.reshape(testCase.hVals, (6,3))
                pMatrix = np.reshape(testCase.pVals, (7,3))
                kinematics = ik_python.KinematicsObject(hMatrix, pMatrix)
                bot.robot.set_kinematics(kinematics)

                # Generate 100 random robot configurations
                qVals = np.random.rand(20, 6) * pi + pi/2
                for i, q in enumerate(qVals):
                    # Get the forward kinematics result and then run inverse to see if we get the same thing
                    forward_kinematics = bot.robot.forward_kinematics(q)
                    rotation = np.array(forward_kinematics[0])
                    translation = np.array(forward_kinematics[1])


                    # Get the inverse kinematics
                    result = bot.robot.get_ik(forward_kinematics[0], forward_kinematics[1])

                    # Run forward kinematics on the result to make sure it is the same as the input
                    resultForward = bot.robot.forward_kinematics(result[0])
                    resultRotation = np.array(resultForward[0])
                    resultTranslation = np.array(resultForward[1])

                    # Check to make sure each value is roughly equal, up to 2pi
                    for resultVal, expectedVal in zip(rotation.flatten() , resultRotation.flatten()):
                        self.assertAlmostEqual(resultVal % (2*pi), expectedVal % (2*pi) , delta=epsilon, 
                            msg=f"Failed on test {i + 1} of {bot.casename} configuration {botNum + 1} with \nqVals: {str(q)}\n and result: {str(result[0])} \
                            \nRotation matrix expected: \n{str(rotation)}\nGot: \n{str(resultRotation)} \
                            \nTranslation expected: {str(translation)}\nGot: {str(resultTranslation)}")
                    for resultVal, expectedVal in zip(translation, resultTranslation):
                        self.assertAlmostEqual(resultVal, expectedVal, delta=epsilon, 
                                               msg=f"Failed on test {i + 1} of {bot.casename} configuration {botNum + 1} with \nqVals: {str(q)}\n and result: {str(result[0])} \
                                               \Rotation matrix expected: \n{str(rotation)}\nGot: \n{str(resultRotation)} \
                                               \nTranslation expected: {str(translation)}\nGot: {str(resultTranslation)}")



class TestHardcodedBots(unittest.TestCase):
    def setUp(self):
        self.bots = []
        for filename in hardcodedFilenames:
            # Setup the robot from the filename
            robot = ik_python.Robot(filename)

            testcases = []
            # Open ../../test_cases/FILENAME.csv
            with open(os.path.join(os.path.dirname(__file__), '..', '..', 'test_cases', filename + '.csv')) as f:
                reader = csv.reader(f)
                # Skip the header
                next(reader)
                for row in reader:
                    # Make sure row isn't empty
                    if not row:
                        continue
                    rotationMatrix = [float(x) for x in row[0:9]]
                    positionVector = [float(x) for x in row[9:12]]
                    testcases.append(TestCaseHardCoded(rotationMatrix, positionVector))
            self.bots.append(TestBot(filename, robot, testcases))

    def test_hardcoded_robots(self):
        for bot in self.bots:
            print ("Testing " + bot.casename)
            for testCase in bot.testcases:
                rotationMatrix = np.array([testCase.rotationMatrix[i:i+3] for i in range(0, len(testCase.rotationMatrix), 3)])
                positionVector = testCase.positionVector
                result = bot.robot.get_ik(rotationMatrix, positionVector)
                self.assertNotEqual(result[0], zeroresult)

if __name__ == '__main__':
    unittest.main()