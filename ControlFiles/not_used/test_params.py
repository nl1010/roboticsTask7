import brickpi
import time
import params

interface=brickpi.Interface()
interface.initialize()

motors = [0,1]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams = params.setup(interface)

interface.setMotorAngleControllerParameters(motors[0],motorParams[0])
interface.setMotorAngleControllerParameters(motors[1],motorParams[1])

logfile_name = "0_" + str(params.MOTOR0_KU) + "_1_" + str(params.MOTOR1_KU)

paramsFile = open('../Logfiles/Params/' + logfile_name + "_params.txt", 'w')
paramsFile.write(str(params.MOTOR0_KU) + '\n' + str(params.MOTOR1_KU))
paramsFile.close

interface.startLogging("../Logfiles/" + logfile_name + ".txt")

while True:
  angle = float(input("Enter a angle to rotate (in radians): "))
  interface.increaseMotorAngleReferences(motors,[angle,angle])
  while not interface.motorAngleReferencesReached(motors) :
    time.sleep(0.1)

  print "Destination reached!"

interface.stopLogging()
	

interface.terminate()
