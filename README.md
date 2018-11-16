# Network Firmware Update
aka `Robot Image Programmer`    

This repository contains the network firmware update code, as well as an
example application to get the end user started.

- Following are the steps to run the code:
  1. Ensure you have a [Ti TM4C1294](http://www.ti.com/tool/EK-TM4C1294XL) Microcontroller. (Buy if you have to!)  
  2. Install [Code Composer Studio](http://www.ti.com/tool/CCSTUDIO) and set it up for the microcontroller.
  3. Import projects `BadgerFreeRTOS`, `BadgerBootloader`, and `NetworkFirmwareUpdate`.
  4. Connect the microcontroller and run `NetworkFirmwareUpdate` project.
        
          Project->Properties->Debug->Flash
        
For mutiple runs, make the following changes:

    Settings->Erase Method->Necessary Pages Only
