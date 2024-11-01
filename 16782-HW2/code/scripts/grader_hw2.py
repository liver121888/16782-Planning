import numpy as np
import pdb
import random
import math
import argparse
import subprocess # For executing c++ executable
import pandas as pd
from timeit import default_timer as timer

plannerList = ["RRT", "RRTCONNECT", "RRTSTAR", "PRM"]

###############################################################
################### Util Functions Below ######################

def convertPIs(aString):
    """ Input: A comma seperated string like "pi/2,pi/4,pi/2,pi/4,pi/2,"
            or 1.2,4,5.3 etc
    Output: string replacing the pis 1.57079,...,...
    """
    if aString[-1] == ",":  # Remove training comma if there is one
        aString = aString[:-1]
    aString = aString.replace("pi", "3.141592") # Replace pi with 3.14... if needed
    vecOfStrings = aString.split(",")
    ans = []
    for anExpression in vecOfStrings:
        ans.append(str(eval(anExpression))) # Evaluate expressions if needed
    return ans

###############################################################
################### Main Functions Below ######################


def graderMain(executablePath, gradingCSV, args):

    problems = [["map1.txt", "1.570796,0.785398,1.570796,0.785398,1.570796",
                                "0.392699,2.356194,3.141592,2.8274328,4.712388"],
            ["map2.txt", "0.392699,2.356194,3.141592",
                                "1.570796,0.785398,1.570796"]]

    if args.arg1 == 1:

        print("Running full test")
        problems = []
        # Generate 20 random problems, always use the map2.txt
        caseNum = 20
        seedRand = 2
        numDOFs = 3
        outputTestFile = "../output/grader_out/tests.txt"
        commandGen = "./../build/test_generator {} {} {} {} {}".format(
                "map2.txt", numDOFs, caseNum, seedRand, outputTestFile)
        print("EXECUTING GEN: " + str(commandGen))
        subprocess.run(commandGen.split(" "), check=True)
        with open(outputTestFile) as f:
            for line in f:
                start, goal = line.split(" ")
                start = start.strip("\"\n")
                goal = goal.strip("\"\n")
                # print("Start: ", start, "Goal: ", goal)
                problems.append(["map2.txt", start, goal])

    else:
        print("Running small test")

    scores = []
    for aPlanner in [0, 1, 2, 3]:
        for i, data in enumerate(problems):
            print("\nTESTING " + plannerList[aPlanner] + ", PROBLEM {}\n".format(i))

            inputMap, startPos, goalPos = [*data]
            numDOFs = len(startPos.split(","))
            outputSolutionFile = "../output/grader_out/tmp.txt"
            commandPlan = "{} {} {} {} {} {} {}".format(
                executablePath,
                inputMap, numDOFs, startPos, goalPos,
                aPlanner, outputSolutionFile)
            commandVerify = "./../build/verifier {} {} {} {} {}".format(
                inputMap, numDOFs, startPos, goalPos,
                outputSolutionFile)
            try:
                print("EXECUTING PLANNER: " + str(commandPlan))
                start = timer()
                result = subprocess.run(commandPlan.split(" "), capture_output=True, text=True, check=True)
                timespent = timer() - start
                print("EXECUTING VERIFIER: " + str(commandVerify))
                returncode = subprocess.run(commandVerify.split(" "), check=False).returncode
                if returncode != 0:
                    print("Returned an invalid solution")
                
                # Parse Vnum from the output
                output = result.stdout
                nodeSize = None
                for line in output.splitlines():
                    if "Vnum:" in line:
                        nodeSize = int(line.split(":")[1].strip())  # Extract Vnum value
                    


                ### Calculate the cost from their solution
                print("CALCULATE COST")
                with open(outputSolutionFile) as f:
                    line = f.readline().rstrip()  # filepath of the map
                    solution = []
                    for line in f:
                        solution.append(line.split(",")[:-1]) # :-1 to drop trailing comma
                    solution = np.asarray(solution).astype(float)
                    numSteps = solution.shape[0]

                    ## Cost is sum of all joint angle movements
                    difsPos = np.abs(solution[1:,]-solution[:-1,])
                    cost = np.minimum(difsPos, np.abs(2*np.pi - difsPos)).sum()
                    print("Cost: ", cost)
                    success = returncode == 0
                    scores.append([aPlanner, inputMap, i, numSteps, cost, timespent, success, nodeSize])
            
                print("VISULIZATION")
                ### Visualize their results
                commandViz = "python -u visualizer.py ../output/grader_out/tmp.txt --gifFilepath=../output/grader_out/grader_{}{}.gif".format(plannerList[aPlanner], i)
                commandViz += " --incPrev=1"
                subprocess.run(commandViz.split(" "), check=True) # True if want to see failure errors
            except Exception as exc:
                print("Failed: {} !!".format(exc))
                scores.append([aPlanner, inputMap, i, -1, -1, timespent, False])

    ### Save all the scores into a csv to compute overall grades
    df = pd.DataFrame(scores, columns=["planner", "mapName", "problemIndex", "numSteps", "cost", "timespent", "success", "nodeSize"])
    df.to_csv(gradingCSV, index=False)
    print("All the scores saved")

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Grader for HW2")

    # Define positional arguments
    parser.add_argument('arg1', type=int, help="Full test or not")
    args = parser.parse_args()

    graderMain("./../build/planner", "../output/grader_out/grader_results.csv", args)