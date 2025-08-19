#include "api/Common.h"
#include "ultra_newping.h"

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(3, 2, MAX_DISTANCE), // Ultra 1 trigger pin, echo pin, and max distance to ping. 
  NewPing(5, 4, MAX_DISTANCE)  // Ultra 2 trigger pin, echo pin, and max distance to ping.
};

void ultra_testing(){
    delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    Serial.print("ultra1");
    Serial.print("=");
    Serial.print(sonar[0].ping_cm());
    Serial.println("cm ");
}

int ultra1Dist_cm(){
  delay(30);
  sonar[0].ping_cm();
}

int ultra2Dist_cm(){
  delay(10);
  sonar[1].ping_cm();
}