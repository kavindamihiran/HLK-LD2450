#include <HardwareSerial.h>
#include <math.h>  // Needed for sqrt()

#define RXD2 44  // Change based on your ESP32 wiring
#define TXD2 43  // Change based on your ESP32 wiring

HardwareSerial mySerial(2);  // Using UART2

// Function to process and display data for each target
void processTarget(uint8_t *data, int targetNumber) {
  // Extract raw values for X, Y, Speed and distance resolution
  int16_t raw_x = data[0] | (data[1] << 8);
  int16_t raw_y = data[2] | (data[3] << 8);
  int16_t raw_speed = data[4] | (data[5] << 8);
  uint16_t distance_resolution = data[6] | (data[7] << 8);  // Currently unused

  // Process the X coordinate:
  // If highest bit is 0, value is negative; otherwise positive (remove the sign bit)
  int16_t target_x;
  if ((raw_x & 0x8000) == 0) {
    target_x = -raw_x;
  } else {
    target_x = raw_x & 0x7FFF;
  }

  // Process the Y coordinate similarly:
  int16_t target_y;
  if ((raw_y & 0x8000) == 0) {
    target_y = -raw_y;
  } else {
    target_y = raw_y & 0x7FFF;
  }

  // (Optional) Process speed if needed:
  int16_t target_speed;
  if ((raw_speed & 0x8000) == 0) {
    target_speed = -raw_speed;
  } else {
    target_speed = raw_speed & 0x7FFF;
  }

  // Calculate Euclidean distance in mm
  float distance = sqrt(pow(target_x, 2) + pow(target_y, 2));

  // Determine if the target is valid (non-zero coordinates)
  bool target_detected = !(target_x == 0 && target_y == 0);

  if (target_detected) {
    // Position Classification: use a tighter threshold when close, wider when far
    String position;
    if (distance <= 1000) {
      // When close, threshold is -100 and +100 for left/right detection
      if (target_x < -100) position = "LEFT";
      else if (target_x > 100) position = "RIGHT";
      else position = "MIDDLE";
    } else {
      // When farther away, threshold is -500 and +500
      if (target_x < -500) position = "LEFT";
      else if (target_x > 500) position = "RIGHT";
      else position = "MIDDLE";
    }

    // Distance classification
    String range = (distance <= 1000) ? "CLOSE" : "FAR";

    // Print out target information
    Serial.println("--------------------------");
    Serial.print("Target ");
    Serial.println(targetNumber);
    Serial.print("Position: ");
    Serial.println(position);
    Serial.print("Range: ");
    Serial.println(range);
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
    Serial.print("Raw X: ");
    Serial.println(target_x);
    Serial.print("Raw Y: ");
    Serial.println(target_y);
    // If needed, you can also print target_speed
    Serial.println("--------------------------");
  } else {
    Serial.print("No target detected for Target ");
    Serial.println(targetNumber);
  }
}

void setup() {
  Serial.begin(256000);                            // Serial Monitor baud rate for debugging
  mySerial.begin(256000, SERIAL_8N1, RXD2, TXD2);  // HLK-LD2450 default baud rate
  Serial.println("HLK-LD2450 Radar Initialized");
}

void loop() {
  // Wait until a full frame of 32 bytes is available
  if (mySerial.available() >= 32) {
    uint8_t header[4];
    mySerial.readBytes(header, 4);

    // Check for the expected header pattern: AA FF 03 00
    if (header[0] == 0xAA && header[1] == 0xFF && header[2] == 0x03 && header[3] == 0x00) {
      uint8_t dataBuffer[28];  // Remaining 28 bytes of the frame
      mySerial.readBytes(dataBuffer, 28);

      // Process target 1: bytes 0 to 7
      processTarget(&dataBuffer[0], 1);

      // Process target 2: bytes 8 to 15
      processTarget(&dataBuffer[8], 2);

      // Process target 3: bytes 16 to 23
      processTarget(&dataBuffer[16], 3);

      // (Optional) The remaining 4 bytes can be used for tail or reserved data.
    } else {
      // If header doesn't match, read one byte to re-sync the frame
      mySerial.read();
    }
  }

  delay(50);  // Small delay to stabilize readings
}
