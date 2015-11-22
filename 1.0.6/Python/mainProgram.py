#!/usr/bin/env python

"""IMUdataTool.py: Download, store and process data from the IMU module."""

import sys
import time
import modules.general
import modules.com
import modules.fileManagement
import modules.dataProcessing

__author__ = "Guillaume Biton with Michael Liuzzolino and Gregory Largange at the University of Arizona (Dr. ENikov's labe, AME)"
__credits__ = ["Guillaume Biton", "Michael Liuzzolino","Gregory Largange"]
__license__ = "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International - http://creativecommons.org/licenses/by-nc-sa/4.0/"
__version__ = "1.0.1"
__email__ = "contact@gbweb.fr"
__status__ = "Dev"


    

print('Welcome in the IMU data manager program v1.0.1.\n')
options = {1: modules.com.downloadOverSerial,
           2: modules.fileManagement.importFromFile}
rawData = options[modules.general.menu('Please select a source :',['Download data from IMU device','Import a raw data file'])]()


processedData = modules.dataProcessing.processData(rawData)
# processedData : [date, Xaccel, Yaccel, Zaccel, Xrot, Yrot, Zrot, pressure, Xvelo, Yvelo, Zvelo, Xpos, Ypos, Zpos]

print('Data processed ! What do you want to do with it ?')
options = {1: modules.fileManagement.createCSV(processedData),
           2: modules.fileManagement.importFromFile}
options[modules.general.menu('Please select an action:',['Export the processed data to a spreadsheet (csv)','Plot the trajectory','Generate a 3D reconstitution of the flight'])]