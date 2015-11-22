import general
import struct

def processData(rawData):

    STRONG_ACCELERATION_PRECISION = 7.5
    
    
    output = []

    currentDate = 0
    print('Processing the data...')
    general.update_progress(0)
    print 'Hello'
    for i in range(0, len(rawData)):
        error = False
        blockID = int(rawData[i][0])*256 + int(rawData[i][1])
        verificationBlock = int(rawData[i][22])*256 + int(rawData[i][23])
        line = []
        if blockID != i:
            print('Error on line {} : block ID inconsistent.\n Please have a look at the raw-data file to figure out the error.'.format(i+1))
            error = True
        if verificationBlock != 28785:
            print('Error on line {} : verification bytes inconsistent.\n Please have a look at the raw-data file to figure out the error.'.format(i+1))
            error = True
        if error == False:
            line.append(int(rawData[i][0])*256+int(rawData[i][1]))
            line.append(currentDate)
            for x in range(1,11):
                data = [int(rawData[i][x*2+1]),int(rawData[i][x*2]),0,0]
                b = struct.pack('4B', *data)
                line.append(struct.unpack('>f', b)[0])

            constShift1 = 2
            constShift2 = 5
            #for j in range(0,3):
            #    if line[constShift2+j] < STRONG_ACCELERATION_PRECISION :
            #        del line[constShift2+j]
            #        constShift2 -= 1
            #    else:
            #        del line[constShift1+j]
            #        constShift1 -= 1
            #        constShift2 -= 1
            output.append(line)
            currentDate += 100.8
            general.update_progress(i/len(rawData))
        else:
            raise Exception('Error with the data')
    return output