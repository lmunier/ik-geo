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

# Import the csv files
import csv
import os

TestBot = namedtuple('TestBot', ['casename', 'robot', 'testcases'])
TestCaseGeneral = namedtuple('TestCase', ['hVals','pVals','rotationMatrix', 'positionVector'])
TestCaseHardCoded = namedtuple('TestCase', ['rotationMatrix', 'positionVector'])


zeroresult = [0.0] * 6

epsilon = 1e-4
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
                    hVals = [float(x) for x in row[0:18]]
                    pVals = [float(x) for x in row[18:39]]
                    rotationMatrix = [float(x) for x in row[39:48]]
                    positionVector = [float(x) for x in row[48:51]]
                    
                    testcases.append(TestCaseGeneral(hVals, pVals, rotationMatrix, positionVector))
            self.bots.append(TestBot(filename, robot, testcases))

    def test_general_robots(self):
        for bot in self.bots:
            print ("Testing " + bot.casename)
            for testCase in bot.testcases:
                hMatrix = np.array([testCase.hVals[i:i+3] for i in range(0, len(testCase.hVals), 3)])
                pMatrix = np.array([testCase.pVals[i:i+3] for i in range(0, len(testCase.pVals), 3)])
                
                kinematics = ik_python.KinematicsObject(hMatrix, pMatrix)
                bot.robot.set_kinematics(kinematics)
                rotationMatrix = np.array([testCase.rotationMatrix[i:i+3] for i in range(0, len(testCase.rotationMatrix), 3)])
                positionVector = testCase.positionVector
                result = bot.robot.get_ik(rotationMatrix, positionVector)
                # Run the forward kinematics and check if the error is below epsilon
                forward_kinematics = bot.robot.forward_kinematics(result[0])
                print (forward_kinematics)
                print(rotationMatrix)
                print(positionVector)
                self.assertTrue(np.allclose(forward_kinematics[0], rotationMatrix, atol=epsilon))
                self.assertTrue(np.allclose(forward_kinematics[1], positionVector, atol=epsilon))



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