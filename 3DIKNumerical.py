# Program finds the 3 joints for a 3R robot based on user-given end-effector positions, arm lengths, and end-orientation angle
# Setup
import numpy as np


##################
### 3D FK CODE ###
##################

def forwardKinematics(aList, returnVal):
      # Get matrices
      # For "valType" - "j" or "a"
      # For "axis" - x = "x", y = "y", z = "z"
      # "num" is a float
      def getMatrix(valType, axis, num):
            if valType == "j": # Return joint matrix
              if axis == "x": # Return x-joint matrix
                return ([ 1,           0,            0 , 0],
                        [ 0, np.cos(num),  -np.sin(num), 0],
                        [ 0, np.sin(num),   np.cos(num), 0],
                        [ 0,           0,             0, 1])
              elif axis == "y": # Return y-joint matrix
                return ([ np.cos(num), 0, np.sin(num), 0],
                        [           0, 1,           0, 0],
                        [-np.sin(num), 0, np.cos(num), 0],
                        [           0, 0,           0, 1])
              elif axis == "z": # Return z-joint matrix
                return ([ np.cos(num), -np.sin(num), 0, 0],
                        [ np.sin(num),  np.cos(num), 0, 0],
                        [           0,            0, 1, 0],
                        [           0,            0, 0, 1])
            elif valType == "a": # Return leg matrix
              if axis == "x": # Return x-leg matrix
                return ([ 1, 0, 0, num],
                        [ 0, 1, 0,   0],
                        [ 0, 0, 1,   0],
                        [ 0, 0, 0,   1])
              elif axis == "y": # Return y-leg matrix
                return ([ 1, 0, 0,   0],
                        [ 0, 1, 0, num],
                        [ 0, 0, 1,   0],
                        [ 0, 0, 0,   1])
              elif axis == "z": # Return z-leg matrix
                return ([ 1, 0, 0,   0],
                        [ 0, 1, 0,   0],
                        [ 0, 0, 1, num],
                        [ 0, 0, 0,   1])

      # Get matrix by inputting "values" into "GetMatrix"
      matrixList = []
      for each in aList:
        matrixList.append(getMatrix(each[0], each[1], each[2]))

      # Multiply the matrices together
      if len(matrixList) > 1:
        result = matrixList[0]
        for i in range(len(matrixList) - 1):
          result = np.dot(result, matrixList[i + 1])
      else:
        result = matrixList[0]

      # Return results
      if returnVal == "x": # Return end X
        return result[0][3]
      elif returnVal == "y": # Return end Y
        return result[1][3]
      elif returnVal == "z": # Return end Z
        return result[2][3]
      else:
        return None


###################
### GET VALUES ####
###################
# Set up joints and links through for input
def getValue(question, valType, getDone):
  while True:
    try:
      if valType == float:
        val = input(question)
        if val == "done" and getDone:
          return "done"
        else:
          val = float(val)
      elif valType == int:
        val = input(question)
        if val == "done" and getDone:
          return "done"
        else:
          val = int(val)
      elif valType == str:
        val = str(input(question))
    except:
      print("Invalid - try again:")
      continue
    break
  return val
"""
** REMOVED TO TEST 3DR ROBOT FASTER **
# Construct 3DR
print("Say if you want a joint - \"j\" or arm - \"a\". Then, input the value for the one you select (joints are in degrees). Input \"done\" when you are finished.")
values = []
count = -1;
while True:
  # Get type of input (joint - "j" or arm - "a")
  print()
  print("Value -", count + 2)
  print("*** ARM OR JOINT ***")
  print("type \"done\" if you are done, only your last fully completed input will be used")
  typeOf = getValue("Arm (\"a\") or joint (\"j\"): ", str, True)
  while typeOf != "a" and typeOf != "j" and typeOf != "A" and typeOf != "J" and typeOf != "done":
    typeOf = getValue("Try again arm - \"a\" or joint - \"j\": ", str, True)
  if typeOf == "done":
    break
  elif typeOf != "A":
    typeOf == "a"
  elif typeOf != "J":
    typeOf == "j"

  # Get axis (x, y, z)
  print("*** AXIS ***")
  print("type \"done\" if you are done, only your last fully completed input will be used")
  axis = getValue("What axis are you using? - type x, y, or z: ", str, True)
  while axis != "x" and axis != "y" and axis != "z" and axis != "X" and axis != "Y" and axis != "Z" and axis != "done":
    axis = getValue("Choose one - \"x\", \"y\", or \"z\": ", str, True)
  if axis == "done":
    break
  elif axis != "X":
    axis == "x"
  elif axis != "Y":
    axis == "y"
  elif axis != "Z":
    axis == "z"

  # Get value (float value)
  if typeOf == "a":
    print("*** ARM LENGTH ***")
    print("type \"done\" if you are done, only your last fully completed input will be used")
    inputVal = getValue("Arm length: ", float, True)
    if inputVal == "done":
      break

  # Update values
  if typeOf == "a":
    values.append([typeOf, axis, inputVal])
  elif typeOf == "j":
    values.append([typeOf, axis, 0])
  else:
    print("ERROR")
    continue
  count += 1
"""
values = [["j", "x", 0], ["j", "z", 0], ["a", "x", 4], ["j", "z", 0], ["a", "x", 4]]
# Get end-effector position
endX = getValue("End x-end-effector position: ", float, False)
endY = getValue("End y-end-effector position: ", float, False)
endZ = getValue("End z-end-effector position: ", float, False)


###################
### 3D IK CODE ####
###################

# Set up values
sumArms = 0
for i in values:
  if i[0] == "a":
    sumArms += i[2]
distError = (sumArms)/500 # How precise calculations should be
inc = .1*np.pi/180 # How much to increase joint each time (in radians)
countMax = 10000 # Limit on how many iterations program can run until "failure"

# Setup values and assign end-effector positions
x = forwardKinematics(values, "x")
y = forwardKinematics(values, "y")
z = forwardKinematics(values, "z")
dist = np.sqrt((endX-x)**2 + (endY-y)**2 + (endZ-z)**2) # Calculate distant of end point from guess

# Guess joints code
cantReach = False
if (sumArms < np.sqrt(endX**2 + endY**2 + endZ**2)): # Removed from if statement: "or abs(l1 - l2) > np.sqrt(endX**2 + endY**2 + endZ**2"
  # Point aint getable
  print("\nNot reachable :(")
  cantReach = True
else:
  # Point is getable. Loop operates until end-effector position is in range of real end-point
  count = 0
  while (dist > distError):
    # Code to move joint
    for i in range(len(values)):
      if (values[i][0] == "j"):
        # Find the x, y, and z position for the current, positively and negatively rotated joint values
        values[i][2] += inc # Rotate joint in the positive direction
        xPos = forwardKinematics(values, "x")
        yPos = forwardKinematics(values, "y")
        zPos = forwardKinematics(values, "z")
        values[i][2] -= 2*inc # Rotate joint in the negative direction
        xNeg = forwardKinematics(values, "x")
        yNeg = forwardKinematics(values, "y")
        zNeg = forwardKinematics(values, "z")
        xStay = x
        yStay = y
        zStay = z
        # Determine the distance from target for current, positively and negatively rotated joint values
        distPos = np.sqrt((endX-xPos)**2 + (endY-yPos)**2 + (endZ-zPos)**2)
        distNeg = np.sqrt((endX-xNeg)**2 + (endY-yNeg)**2 + (endZ-zNeg)**2)
        distStay = np.sqrt((endX-xStay)**2 + (endY-yStay)**2 + (endZ-zStay)**2)
        # Determine which of positive, negative, and stay gives the best solution and sets it to for the new joint
        distVect = [distPos, distStay, distNeg]
        minPos = distVect.index(min(distVect)) # Get index of smallest value (fancy code lol)
        if (minPos == 0):
          values[i][2] += 2*inc
        elif(minPos == 1):
          values[i][2] += inc
        else:
          values[i][2] = values[i][2]
        # Set the current x/y/z position for new joint
        x = forwardKinematics(values, "x")
        y = forwardKinematics(values, "y")
        z = forwardKinematics(values, "z")

    # Wrap stuff up
    dist = np.sqrt((endX-x)**2 + (endY-y)**2 + (endZ-z)**2)
    count += 1
    # Error line
    if count > countMax:
      print("\nExceeded limit: Error")
      cantReach = True
      break

####################
### PRINT VALUES ###
####################

# Convert joints to degrees
for i in range(len(values)):
  if values[i][0] == "j":
    values[i][2] = values[i][2]*180/np.pi

# Output results
if cantReach == False:
  print("\nLoop iterated", count, "times.")
  qCount = 0
  for i in range(len(values)):
    if values[i][0] == "j":
      qCount += 1
      print("q", qCount, "=", round(values[i][2], 3), "degrees")
  print("Target end-effector position: (", endX, ", ", endY, ", ", endZ, ")",)
  print("Numerically estimated end-effector position: (", round(x, 3), ", ", round(y, 3), ", ", round(z, 3), ")")