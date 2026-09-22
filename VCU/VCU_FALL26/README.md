This folder is for VCU firmware used for Fall 26 vehicle testing. 

VCU_BERT26_TTT-D1_2 represents VCU firmware used for both days at test tune & teach, along with 09/12/26 vehicle testing. 
    - Firmware attempted to implement regenerative braking, however was not successful. Unsure if it is a firmware or inverter issue
    = Firmware logs data on SD card and splits it up into Measured, Derived, and Cell data
    - MAJOR ISSUE:
      - During the 2nd day of test, tune, and teach, there was a case where the TSSI was flashing red while the RTML was turned on. 
        This is a major safety concern & should never happen. The event occurred on this iteration of the firmware. Root cause has 
        yet to be determined. 
