# This programm download and process data from our IMU device
# By Guillaume Biton at the University of Arizona - October 2015
# Version 1.0

import sys
import glob
import serial
import time



    
    

            


# CONSTANTS declaration :
RECORDING_FREQUENCY = 0.1
BLOCK_SIZE = 25

communicationError = False
# Establish a connection :
serialSocket = chooseSerialPorts()
# Ask for the number of blocks stored on EEPROM :
serialSocket.write(chr(125))
#Wait for a response :
waiting = True
requestTime = time.time()
while waiting :
    if serialSocket.inWaiting()>1:
        numberOfBlocks=ord(serialSocket.read(1))*256 + ord(serialSocket.read(1))
        print ('There are {} blocks stored on the device (~{} seconds of recording) !'.format(numberOfBlocks, numberOfBlocks*RECORDING_FREQUENCY))
        waiting = False
    if time.time()- requestTime > 2:
        print 'Connection timout : is the device still plugged ?'
        waiting = False
        communicationError = True
        serialSocket.close()
    time.sleep(0.1)

if communicationError==False :
    print 'Let\'s download the data'
    rawData = []
    update_progress(0)
    #Download all the blocks 1 by 1 :
    for i in range(0, numberOfBlocks):
        rawData.append(receiveABlock(serialSocket))
        update_progress(float(float(i)/float(numberOfBlocks)))
    update_progress(1)
    print('All data downloaded from device !')
    print('Generating the raw-data file...')
    # Create a new file to store the raw data
    rawFile = open('{}flight.rawdata'.format(time.strftime("%d-%m_%H-%M-%S", time.gmtime())), 'w')
    for i in range(0, len(rawData)):
        for j in range(0,len(rawData[i])):
            rawFile.write(str(rawData[i][j]))
            if j == len(rawData[i])-1:
                rawFile.write('\n')
            else :
                rawFile.write(',')
    rawFile.close()
    print('Raw-data file ready.\nProcessing the data.')
    # Create a new file to store the processed data
    processedFile = open('{}flight.csv'.format(time.strftime("%d-%m_%H-%M-%S", time.gmtime())), 'w')
    date = 0
    #Store all the lines in the file, and check that they are consistent
    for i in range(0, len(rawData)):
        error = False
        blockID = int(rawData[i][0])*256 + int(rawData[i][1])
        date = date + int(rawData[i][2])*256 + int(rawData[i][3])
        print(blockID)
        verificationBlock = int(rawData[i][24])*256 + int(rawData[i][25])
        print(verificationBlock)
        
        if blockID != i:
            print('Error on line {} : block ID inconsistent.\n Please have a look at the raw-data file to figure out the error.'.format(i+1))
            error = True
        if verificationBlock != 31565:
            print('Error on line {} : verification bytes inconsistent.\n Please have a look at the raw-data file to figure out the error.'.format(i+1))
            error = True
        if error == False:
            for x in range(0,13):
                if x == 1 :
                    processedFile.write(str(date))
                else:
                    processedFile.write(str(int(rawData[i][x*2])*256+int(rawData[i][x*2+1])))
                if x == 12:
                    processedFile.write('\n')
                else:
                    processedFile.write(',')
        else:
            raise Exception('Error with the data')