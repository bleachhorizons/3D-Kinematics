# Program finds the 3D end effect position and orientation through forward kinematics
# Setup
import numpy as np

##################
### GET VALUES ###
##################
print("Say if you want a joint - \"j\" or arm - \"a\". Then, input the value for the one you select (joints are in degrees). Input \"done\" when you are finished.")
def getValue(question, valType):
  while True:
    try:
      if valType == float:
        val = input(question)
        if val == "done":
          return "done"
        else:
          val = float(val)
      elif valType == int:
        val = input(question)
        if val == "done":
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

# Set up joints and links through input
values = []
count = -1;
while True:
  # Get type of input (joint - "j" or arm - "a")
  print()
  print("Value -", count + 2)
  print("*** ARM OR JOINT ***")
  print("type \"done\" if you are done, only your last fully completed input will be used")
  typeOf = getValue("Arm (\"a\") or joint (\"j\"): ", str)
  while typeOf != "a" and typeOf != "j" and typeOf != "A" and typeOf != "J" and typeOf != "done":
    typeOf = getValue("Try again arm - \"a\" or joint - \"j\": ", str)

  if typeOf == "done":
    break
  elif typeOf != "A":
    typeOf == "a"
  elif typeOf != "J":
    typeOf == "j"

  # Get axis (x, y, z)
  print("*** AXIS ***")
  print("type \"done\" if you are done, only your last fully completed input will be used")
  axis = getValue("What axis are you using? - type x, y, or z: ", str)
  while axis != "x" and axis != "y" and axis != "z" and axis != "X" and axis != "Y" and axis != "Z" and axis != "done":
    axis = getValue("Choose one - \"x\", \"y\", or \"z\": ", str)
  if axis == "done":
    break
  elif axis != "X":
    axis == "x"
  elif axis != "Y":
    axis == "y"
  elif axis != "Z":
    axis == "z"

  # Get value (float value)
  print("*** ARM LENGTH/ANGLE ***")
  print("type \"done\" if you are done, only your last fully completed input will be used")
  inputVal = 0
  if typeOf == "a":
    inputVal = getValue("Arm length: ", float)
    if inputVal == "done":
      break
  elif typeOf == "j":
    inputVal = getValue("Angle (in degrees): ", float)
    if inputVal == "done":
      break
  else:
    print("ERROR")
    continue

  # Update values
  if typeOf == "a":
    values.append([typeOf, axis, inputVal])
  elif typeOf == "j":
    values.append([typeOf, axis, inputVal*np.pi/180])
  else:
    print("ERROR")
    continue
  count += 1


#############################
### DO FORWARD KINEMATICS ###
#############################

# Forward kinematics Function
def forwardKinematics(aList):
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
      if len(matrixList) < 1:
        print("No values inputted")
      elif len(matrixList) > 1:
        result = matrixList[0]
        for i in range(len(matrixList) - 1):
          result = np.dot(result, matrixList[i + 1])
      else:
        result = matrixList[0]

      if len(matrixList) > 0:
        # Find end-effector positions and orientation
        endEffectorPosX = result[0][3]
        endEffectorPosY = result[1][3]
        endEffectorPosZ = result[2][3]

        # Print results
        print("\n*** Result Matrix *** \n", result)
        print("The end effector position will be (", round(endEffectorPosX, 2), ",", round(endEffectorPosY, 2), ",", round(endEffectorPosZ, 2), ")")

# Call function
forwardKinematics(values)