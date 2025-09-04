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

int getultra1Dist_cm(){
  delay(30);
  sonar[0].ping_cm();
}

int getultra2Dist_cm(){
  delay(30);
  sonar[1].ping_cm();
}

void ultra1Dist_cmloop(){
  ultra1Dist_new = getultra1Dist_cm() * 10;
}

void ultra2Dist_cmloop(){
  ultra2Dist_new = getultra2Dist_cm() * 10;
}
// void ultra1_event_timer_sketch(){
//   unsigned int pingSpeed = 50;
//   unsigned long pingTimer;
//   pingTimer = millis();
//   if (millis() >= pingTimer){
//     pingTimer += pingSpeed;
//     sonar[0].ping_timer(echoCheck);
//     print(
//   }
// }

// void echoCheck(){
//   if (sonar[0].check_timer()) { // This is how you check to see if the ping was received.
//     // Here's where you can add code.
//     Serial.print("Ping: ");
//     Serial.print(sonar[0].ping_result / US_ROUNDTRIP_CM); // Ping returned, uS result in ping_result, convert to cm with US_ROUNDTRIP_CM.
//     Serial.println("cm");
// }