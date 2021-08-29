# RPLidar_Arduino
RoboPeak RPLIDAR driver for Arduino and Arduino-compatible devices.
Based on RPLIDAR SDK v1.12.0 (https://github.com/Slamtec/rplidar_sdk) and ported to Arduino IDE code.

### Notes:
1. The code was tested only on RPLidar A1, but should work on other models with small modifications.
2. RPLidar motor speed is controlled via uC pin.
3. Standard and Express Scan are only supported for now.

### Usage

Include the rplidar sdk and declare the rplidar object:

```pp
#include "rplidar_driver_impl.h"
RPLidar rplidar;
```

In `setup()` call the `.begin()` function. 

```cpp
void setup(){
  rplidar.begin();
}
```

The Serial interface for the lidar communication needs to be defined in the `rplidar_driver.cpp`.
```cpp
const int lidarRxPin = 34;
const int lidarTxPin = 33;
#define lidarSerial Serial5

bool RPLidar::begin()
{
    pinMode(lidarRxPin, INPUT);
    pinMode(lidarTxPin, OUTPUT);
    lidarSerial.begin(115200);
    ...
}
```

To start standard measurement `loop()` call `rplidar.startScanNormal()`, to use the express scan call `rplidar.startScanExpress()`.

Ex:
```cpp
void loop(){
  if (!rplidar.isScanning()){
    rplidar.startScanExpress(true, RPLIDAR_CONF_SCAN_COMMAND_EXPRESS);
    digitalWrite(lidarMotorPin, HIGH); // turn on the motor
  }
}
```

In `loop()` call `rplidar.loopScan<Express>Data()` - in this loop Lidar will process newly received data samples.
```cpp
void loop(){
  if (!rplidar.isScanning()){
    ...
  }
  else{
    rplidar.loopScanExpressData();
  }
}
```

Lidar date can be grabbed from the internal buffers by `rplidar.grabScan<Express>Data()` for further processing.

Ex:
```cpp
void loop(){
  if (!rplidar.isScanning()){
    ...
  }
  else{
    rplidar.loopScanExpressData();
  
    // create object to hold received data for processing
    rplidar_response_measurement_node_hq_t nodes[512];
    size_t nodeCount = 512; // variable will be set to number of received measurement by reference
    u_result ans = rplidar.grabScanExpressData(nodes, nodeCount);
    if (IS_OK(ans)){
      for (size_t i = 0; i < nodeCount; ++i){
        // convert to standard units
        float angle_in_degrees = nodes[i].angle_z_q14 * 90.f / (1<<14);
        float distance_in_meters = nodes[i].dist_mm_q2 / 1000.f / (1<<2);
        char report[50];
        snprintf(report, sizeof(report), "%.2f %.2f %d", distance_in_meters, angle_in_degrees, nodes[i].quality);
        Serial.println(report);
      }
    }
  }
}
```
