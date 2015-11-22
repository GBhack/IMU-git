import sys
import time
import glob
import serial
import general
import fileManagement

def chooseSerialPort():
    """ Lists serial port names
        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    connectionSuccessfull = False
    
    #Loop while we don't have a successfull connection with IMU device
    while connectionSuccessfull == False:
        #Adapt to the plateform we're running on :
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this excludes your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')
        result = []
        i = 0
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
                i=i+1
            except (OSError, serial.SerialException):
                pass
        if i == 0 :            
            sys.exit("No serial port available !")
        else :
            #List availbles serial ports :
            print '\nHere are the serial ports available :'
            serialPort = general.menu('Please enter the number of the one corresponding to the IMU device :',result)
            serialPort = result[serialPort-1]
            print('Sending identification request on {}'.format(serialPort))
            serialSocket = serial.Serial(port=serialPort, baudrate=19200, timeout=1) #Open a connection on this port
            serialSocket.write(chr(123))  #Send identification code on this port
            
            #Waiting for an appropriate response from the device :
            waiting = True
            requestTime = time.time()
            while waiting :
                if serialSocket.inWaiting()>0:
                    incoming=ord(serialSocket.read(1))
                    if incoming == 124:
                        print 'Connection successfull !'
                        connectionSuccessfull = True
                        waiting = False
                elif time.time()- requestTime > 5:
                    print 'Connection timout : the device is either busy or not an IMU'
                    waiting = False
                    serialSocket.close()
                time.sleep(0.2)
    return serialSocket
    
def receiveAByte(serialSocket, timeout=2):
    # receiveAByte(serialSocket, timeout) : wait for a byte for timeout
    ## As a byte should always come in double, it waits for the second one to come, check if it is the same and, otherwise ask for a third repeat

    receivedData = 0;
    requestTime = time.time()
    waiting1 = True; waiting2 = True
    incoming1 = 0;incoming2 = 0;incoming3 = 0
    while waiting1 :
        #We wait to have received 2 bytes
        if serialSocket.inWaiting()>1:
            incoming1=ord(serialSocket.read(1))
            incoming2=ord(serialSocket.read(1))
            if incoming1 == incoming2: # If both bytes are equals, we go on
                serialSocket.write(chr(100)) # Well received !
                result = incoming1
            else : #Else, we ask for a third repetition
                serialSocket.write(chr(111)) # Ask for a third !
                requestTime2 = time.time()
                while waiting2:
                    if serialSocket.inWaiting()>0:
                        incoming3=ord(serialSocket.read(1))
                        if incoming3 == incoming1:
                            result = incoming1
                        elif incoming3 == incoming2:
                            result = incoming2
                        else:
                            print('Transmission error')
                            result = 256 #No well received byte can be at 256
                    elif time.time() - requestTime2 > timeout:
                        result = 256 #No well received byte can be at 256
                        print('Connexion timeout')
                    waiting2 = False
            waiting1 = False
        elif time.time() - requestTime > timeout:
            waiting1 = False
            result = 256 #No well received byte can be at 256
            print('Connexion timeout')
    return result
    
def receiveABlock(serialSocket):
    result = []
    for i in range(0, 24):
        received = receiveAByte(serialSocket,2)
        if received == 256 : received = "###"
        result.append(str(received))
    return result
    
    
def downloadOverSerial():
    communicationError = False
    serialSocket = chooseSerialPort()
    print 'Connected'
    # Ask for the number of blocks stored on EEPROM :
    serialSocket.write(chr(125))
    print 'Asking for #bytes'
    time.sleep(0.1)
    #Wait for a response :
    waiting = True
    requestTime = time.time()
    while waiting :
        if serialSocket.inWaiting()>1:
            numberOfBlocks=ord(serialSocket.read(1))*256 + ord(serialSocket.read(1))
            print ('There are {} datapoints stored on the device ({} bytes) !'.format(numberOfBlocks, numberOfBlocks*24))
            waiting = False
        elif time.time()- requestTime > 2:
            print 'Connection timout : is the device still plugged ?'
            waiting = False
            communicationError = True
            serialSocket.close()
        time.sleep(0.1)
    if communicationError==False :
        print 'Let\'s download the data'
        rawData = []
        general.update_progress(0)
        #Download all the blocks 1 by 1 :
        for i in range(0, numberOfBlocks):
            rawData.append(receiveABlock(serialSocket))
            general.update_progress(float(float(i)/float(numberOfBlocks)))
        general.update_progress(1)
        print('All data downloaded from device !')
        fileManagement.createRawDatafile(rawData)
    else:
        sys.exit("Communication error")
    return rawData