// Liquid Level Sensor CaLibration Sketch

// Created by AJ Moon

// Configuration values:
#define SERIES_RESISTOR     2000    // Value of the series resistor in ohms (this is default from the factory. If you got the unit without the built in voltage divider your series resistor might be different. 
#define SENSOR_PIN          A0      // Analog pin which is connected to the sensor. 


// The following are calibration values you can fill in to compute the depth of measured liquid.
// To find these values first start with no liquid present and record the resistance as the
// ZERO_DEPTH_RESISTANCE value.  Next fill the container with a known depth of liquid and record
// the sensor resistance (in ohms) as the CALIBRATION_RESISTANCE value, and the depth (as read on the eTape) as CALIBRATION_DEPTH.

#define ZERO_DEPTH_RESISTANCE   2047.00    // Resistance value (in ohms) when no liquid is present.
#define CALIBRATION_RESISTANCE    732.00    // Resistance value (in ohms) when liquid is at max line.
#define CALIBRATION_DEPTH        25.3    // Depth (in any units) when liquid is at max line.

const int levelAverageNum = 250; // how many readings of the water level are being averaged
/* This number is very important. the eTape doesn't start reacting below 1". 
 *  whatever emasurement you decide to use cm, in, mm....etc. I would recomend CM 
 *  as that's what the application is written for. this vaule must be the equivilent to 1"
 *  in whatever scale you choose.
*/
const float basemeasure = 2.54  ;
 
void setup(void) {
  Serial.begin(9600);
}
 
void loop(void) {
  // Measure sensor resistance.
  float resistance = readResistance(SENSOR_PIN, SERIES_RESISTOR);
  Serial.print("Resistance: "); 
  Serial.print(resistance, 2);
  Serial.println(" ohms");
  // Map resistance to volume.
  float depth = resistanceToLevel(resistance, ZERO_DEPTH_RESISTANCE, CALIBRATION_RESISTANCE, CALIBRATION_DEPTH);
  Serial.print("Calculated depth: ");
  Serial.println(depth, 5);
  // Delay for a second.
  delay(1000);
}

float readResistance(int pin, int seriesResistance) {
  float levelSum(0);
  
  for(int i(0); i < levelAverageNum; i++)
      levelSum += round(analogRead(pin));

  // Get ADC value.
  float resistance = levelSum / levelAverageNum;
  // Convert ADC reading to resistance.
  resistance = (1023.0 / resistance) - 1.0;
  resistance = round(SERIES_RESISTOR / resistance);

  return resistance;
}

float resistanceToLevel(float resistance, float zeroResistance, float calResistance, float calLevel) {
    // Compute Depth using X = (b - Y) / m.
  float level = ((zeroResistance - resistance) / abs( (zeroResistance - calResistance) / (basemeasure - calLevel))) + basemeasure;

  // Round to nearest 10th and save to current water level
  return (floor(level * 10 + .5))/10;
}
