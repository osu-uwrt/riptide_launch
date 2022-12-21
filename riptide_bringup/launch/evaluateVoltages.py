from sys import argv, stdout
import re

#open and process file into lines
voltageFile = open(argv[2])
contents = voltageFile.read()
lines = contents.split("\n")

#if the file contains a cannot find warning
if "WARNING" in contents:
    stdout.write("WARNINGNOFIND")
    
    quit()

portVoltages = []
stbdVoltages = []

#find the voltages in the madness
for line in lines:
    if "port_voltage" in line:
        numberLine = re.sub("[^0-9.]", "", line)
        portVoltages.append(float(numberLine))

    if "stbd_voltage" in line:
        numberLine = re.sub("[^0-9.]", "", line)
        stbdVoltages.append(float(numberLine))

#find lowest voltages
lowVoltageStbd = 1000
for voltage in stbdVoltages:
    if voltage < lowVoltageStbd:
        lowVoltageStbd = voltage

lowVoltagePort = 1000
for voltage in portVoltages:
    if voltage < lowVoltagePort:
        lowVoltagePort = voltage

print("Tempest Voltages Port: " + str(lowVoltagePort) + " Starboard: " + str(lowVoltageStbd) + ".")

criticalVoltage = float(argv[1])

if lowVoltagePort < criticalVoltage or lowVoltageStbd < criticalVoltage:
    stdout.write('Low')



stdout.write('Ok')