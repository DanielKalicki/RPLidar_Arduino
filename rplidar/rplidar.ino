#include "rplidar_driver_impl.h"

const int lidarMotorPin = 35;
char report[80];
RPLidar rplidar;

void setup(){
  Serial.begin(2000000);
  pinMode(lidarMotorPin, OUTPUT);
  // INFO Serial port used by RPLidar is hardcoded in begin() function! Please modify it
  rplidar.begin();
  delay(1000);
}

void printInfo(){
  rplidar_response_device_info_t info;
  rplidar.getDeviceInfo(info);

  snprintf(report, sizeof(report), "model: %d firmware: %d.%d", info.model, info.firmware_version/256, info.firmware_version%256);
  Serial.println(report);
  delay(1000);
}

void printSampleDuration(){
  rplidar_response_sample_rate_t sampleInfo;
  rplidar.getSampleDuration_uS(sampleInfo);

  snprintf(report, sizeof(report), "TStandard: %d[us] TExpress: %d[us]", sampleInfo.std_sample_duration_us, sampleInfo.express_sample_duration_us);
  Serial.println(report);
  delay(1000);
}

// #define SCAN_TYPE_STD
#define SCAN_TYPE_EXPRESS

void loop(){
#ifdef SCAN_TYPE_STD 
  if (!rplidar.isScanning()){
    rplidar.startScanNormal(true);
    digitalWrite(lidarMotorPin, HIGH); // turn on the motor
    delay(10);
  }
  else{
    // loop needs to be send called every loop
    rplidar.loopScanData();
  
    // create object to hold received data for processing
    rplidar_response_measurement_node_hq_t nodes[512];
    size_t nodeCount = 512; // variable will be set to number of received measurement by reference
    u_result ans = rplidar.grabScanData(nodes, nodeCount);
    if (IS_OK(ans)){
      for (size_t i = 0; i < nodeCount; ++i){
        // convert to standard units
        float angle_in_degrees = nodes[i].angle_z_q14 * 90.f / (1<<14);
        float distance_in_meters = nodes[i].dist_mm_q2 / 1000.f / (1<<2);
        snprintf(report, sizeof(report), "%.2f %.2f %d", distance_in_meters, angle_in_degrees, nodes[i].quality);
        Serial.println(report);
      }
    }
  }
#endif
#ifdef SCAN_TYPE_EXPRESS
  if (!rplidar.isScanning()){
    rplidar.startScanExpress(true, RPLIDAR_CONF_SCAN_COMMAND_EXPRESS);
    digitalWrite(lidarMotorPin, HIGH); // turn on the motor
    delay(10);
  }
  else{
    // loop needs to be send called every loop
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
        snprintf(report, sizeof(report), "%.2f %.2f %d", distance_in_meters, angle_in_degrees, nodes[i].quality);
        Serial.println(report);
      }
    }
  }
#endif
}
