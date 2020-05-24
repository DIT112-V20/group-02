# group-02 ![Arduino](https://github.com/DIT112-V20/group-02/workflows/Group%202%20Arduino%20CI/badge.svg)![Android](https://github.com/DIT112-V20/group-02/workflows/Group%202%20Android%20CI/badge.svg)

![Logo](https://i.imgur.com/h5MPxuJ.png)

# Table of contents
1. [Description](#description)
    1. [What](#what)
    2. [Why](#why)
    2. [How](#how)
2. [User Manual](#usermanual)
    1. [Install](#install)
    2. [Use](#use)
3. [Specifications](#specs)
    1. [Hardware](#hardware)
    2. [Software](#software)
4. [Developers](#developers)

## Description <a name="description"></a>
### What are you going to make? <a name="what"></a>

We've developed an **Eye-Seeing-Car** for people with hearing or visual impairments. The car is able to take input as commands and use sensors to drive, notice and alert of obstacles in its path. The car is controlled through **the app** we developed, which connects to the car using Bluetooth. The app will also notify the user with the help of vibration and sound.

### Why will you make it? <a name="why"></a>

There are countless people in the world who struggle with disabilities or impedances that most people aren’t even aware about. The aim of this prototype is to be a more reliable, manageable, and convenient way for people to be assisted with their struggles.

### How are you going to make it? <a name="how"></a>

With the provided arduino car, included hardware, and external devices that communicate with the arduino car and its hardware.

## User manual <a name="usermanual"></a>
### Installation <a name="install"></a>

![Download](https://i.imgur.com/FqRTUy1.png)

1. [Download](https://github.com/DIT112-V20/group-02/archive/master.zip) the package from our GitHub page
2. Copy the SmartCar.ino file to your SmartCar ([using Arduino IDE](https://www.arduino.cc/en/guide/Environment#toc9))
3. Upload the app to your Android device or through an emulator ([using Android Studios](https://developer.android.com/training/basics/firstapp/running-app))

### How to use <a name="use"></a>

![Main menu](https://i.imgur.com/EzbjDVa.png)

1. Begin by toggling Bluetooth on by clicking the **Toggle Bluetooth** switch.
2. Click the **Control Car** button.

![Control menu](https://i.imgur.com/TSSbWCN.png)

1. To control the car manually, click any of the **direction arrows**.
2. To change the speed, click ***-*** to decrease and ***+*** to increase.
3. To stop the car, click the **Stop** sign.
3. To switch to auto, click the **Manual** button.
4. To switch back, click the **Auto** button.

![Help menu](https://i.imgur.com/92QkWTV.png)

For more help, please see our included help page within the app.

## Specifications <a name="specs"></a>
### Hardware <a name="hardware"></a>

- [Arduino Board](http://arduinoinfo.mywikis.net/wiki/Esp32#-_.22ESP32_Dev_Kit_V2.22)
- [Smart Car](https://www.hackster.io/platisd/getting-started-with-the-smartcar-platform-1648ad)

### Software <a name="software"></a>

- [SmartCar Library](https://www.arduinolibraries.info/libraries/smartcar-shield)
- [Arduino IDE](https://www.arduino.cc/)
- [Android Studio](https://developer.android.com/studio)

## Developers <a name="developers"></a>

- Clementine Jensen
- Christian O'Neill 
- Hjalmar Thunberg
- Hugo Hempel
- Linus Ivarsson
- Linus Åberg
