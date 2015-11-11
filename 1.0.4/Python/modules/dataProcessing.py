import general

def processData(rawData):

    STRONG_ACCELERATION_PRECISION = 7.5
    
    
    date = []
    acceleration = []
    rotation = []
    pressure = []

    currentDate = 0

    print('Processing the data...')
    general.update_progress(0)
    
    for i in range(0, len(rawData)):
        error = False
        blockID = int(rawData[i][0])*256 + int(rawData[i][1])
        currentDate = currentDate + int(rawData[i][2])*256 + int(rawData[i][3])
        verificationBlock = int(rawData[i][24])*256 + int(rawData[i][25])
        
        line = []
        if blockID != i:
            print('Error on line {} : block ID inconsistent.\n Please have a look at the raw-data file to figure out the error.'.format(i+1))
            error = True
        if verificationBlock != 31565:
            print('Error on line {} : verification bytes inconsistent.\n Please have a look at the raw-data file to figure out the error.'.format(i+1))
            error = True
        if error == False:
            for x in range(0,13):
                if x == 1 :
                    line.append(date)
                else:
                    line.append(int(rawData[i][x*2])*256+int(rawData[i][x*2+1]))
            date.append(currentDate)
            acceleration.append([0,0,0])
            #Select the appropriate acceleration :
            for j in range(0,3):
                if line[2+j] < STRONG_ACCELERATION_PRECISION :
                    acceleration[i][j]=line[5+j]
                else:
                    acceleration[i][j]=line[2+j]
            
            rotation.append(line[9:11])
            pressure.append(line[12])
            general.update_progress(i*0.4/len(rawData))
        else:
            raise Exception('Error with the data')
            
            
    velocity = [[0,0,0]]
    
    for i in range(1,len(acceleration)-1):
        velocityLine = []
        for j in range(0,3):
            velocityLine.append((((acceleration[i-1][j]+acceleration[i][j])*(date[i]-date[i-1])/2)+((acceleration[i][j]+acceleration[i+1][j])*(date[i+1]-date[i])/2))/2)
        velocity.append(velocityLine)
        general.update_progress(0.4+i*0.3/(len(acceleration)-1))
    velocity.append([0,0,0])

    position = [[0,0,0]]
    for i in range(1,len(velocity)-1):
        positionLine = []
        for j in range(0,3):
            positionLine.append((((velocity[i-1][j]+velocity[i][j])*(date[i]-date[i-1])/2)+((velocity[i][j]+velocity[i+1][j])*(date[i+1]-date[i])/2))/2)
        position.append(positionLine)
        general.update_progress(0.7+i*0.3/(len(velocity)-1))
    position.append([0,0,0])


    general.update_progress(1)
    
    return [date, acceleration, rotation, pressure, velocity, position]