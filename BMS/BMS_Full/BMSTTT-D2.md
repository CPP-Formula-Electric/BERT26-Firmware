BMS_Fall26TTT-D2 represents BMS code from day 2 of test tune & teach
  - This firmware version aimed to solve the following problems from Day 1 of Test Tune & Teach
  - 60C Cell Overtemp limit, upgraded from 52C in previous version
  - Implements a 30s timeout feature, where if the first 8 cells register overtemp data for more than 30s, only then does a BMS overtemp fault get triggered. 
    This is a fix towards a faulty MUX on the Distributed BMS, where the first 8 cells in the HV Battery can register inaccurate temps which triggers a BMS overtemp
    fault & red car, which ended the Day 1 autocross run. 

Results:
  - During a single autocross run on the 2nd Day of Test, Tune, & Teach, there were no registered BMS faults. Further testing is required to evaluate robustness of
    the current software
