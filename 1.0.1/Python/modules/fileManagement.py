from os.path import splitext
def importFromFile():
    notOpen = True
    while notOpen:
        filePath = raw_input('Please enter the path and name of the raw-data file (name is sufficient if the file is in the same folder as this python programm) :\n')
        try:
            filename, fileExtension = splitext(filePath)
        except:
            print('Path inconsistent\n')
        try:
            rawFile = open('{}.rawdata'.format(filename), 'r')
            notOpen = False
        except:
            print('Impossible to open this file\n')
    
    numberOfLines = int(rawFile.readline()[-2:])
    result = []
    for i in range(0,numberOfLines):
        result.append(map(int,rawFile.readline()[0:-1].split(',')))
    return result
    
def createRawDatafile(content):
    rawDataFilename = raw_input('Please enter a desired name of the raw-datafile.\n You can provide a full path (if not the file we be stored into the same folder as this program) :\n')
    print('Generating the raw-data file...')
    # Create a new file to store the raw data
    rawFile = open('{}.rawdata'.format(rawDataFilename), 'w')
    rawFile.write(str('{}\n'.format(len(content))))
    for i in range(0, len(content)):
        for j in range(0,len(content[i])):
            rawFile.write(str(content[i][j]))
            if j == len(content[i])-1:
                rawFile.write('\n')
            else :
                rawFile.write(',')
    rawFile.close()
    print('Raw-data file ready.')