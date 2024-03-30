#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
imu::Quaternion quat = bno.getQuat();
imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

float q_w, q_x, q_y, q_z, e_x, e_y, e_z;
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displaySensorStatus(void)
{
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

void setup(void)
{
  Serial.begin(115200);

  while (!Serial) delay(10);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
  displaySensorDetails();
  displaySensorStatus();

  bno.setExtCrystalUse(true);
}

void loop(void)
{

  q_w = quat.w(); q_x = quat.x(); q_y = quat.y(); q_z = quat.z();
  Serial.print(q_w, 4);
  Serial.print(',');
  Serial.print(q_x, 4);
  Serial.print(',');
  Serial.print(q_y, 4);
  Serial.print(',');
  Serial.print(q_z, 4);
  Serial.print(',');

  e_x = euler.x(); e_y = euler.y(); e_z = euler.z();
  Serial.print(e_x);
  Serial.print(',');
  Serial.print(e_y);
  Serial.print(',');
  Serial.print(e_z);
  Serial.print('\n');

  delay(BNO055_SAMPLERATE_DELAY_MS);

}
