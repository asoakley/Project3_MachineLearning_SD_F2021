# Project3_MachineLearning_SD_F2021
<html>
<body>
<h1>TI Machine Learning Robotics</h1>
<h3>Code repository for all TI-RSLK software used for NC State ECE Senior Design Fall 2021-Spring 2022</h3>
<ul>
    <li>CCS folder contains the program files for Code Composer Studios. 
    It was used in the Fall but turned to Arduino as Tensorflow Lite was no longer supported. 
    This folder contains obsolete code.</li>
    <ul>
        <li>AudioTest contains the files for running microphone</li>
        <li>I2CandUART contains program files for the accelerometer in CCS up and running</li>
        <li>IMUtest contains the program files for testing the  IMU on the sensors board</li>
        <li>LEDtest contains the program files to blink LEDs</li>
        <li>MotorsTest contains the program files to spin the motors</li>
        <li>MotorsTestV2 contains the program files to make patterns for the robot to perform (figure 8, triangle, etc)</li>
        <li>boostxl_sensors_sensorgui_msp432p401r contains program files for a sensors board GUI from TI's sensors boosterpack</li>
    </ul>
    
    <li>libraries folder contains the libraries used in Arduino IDE to get the MSP432 microcontroller and the RSLK to function. 
    It also contains some of the Edge Impulse libraries that we have tested</li>
    <li>prediction folder contains the Arduino code for running edge impulse models. 
    It contains code for both Audio and Accelerometer Machine Learning.</li>
    <li>raw data folder  contains the sample data that we used throughout our trails in different audio/accelerometer models.</li>
    <li>test and training folder contains Arduino code to sample and try out different functions of the RSLK MAX</li>
        <ul>
            <li>MultitaskTest contains program files an attempt to run multitasking using Arduino</li>
        </ul> 
</ul>

</body>
</html>

