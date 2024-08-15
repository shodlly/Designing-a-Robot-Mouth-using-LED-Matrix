# Designing-a-Robot-Mouth-using-LED-Matrix
This repository demonstrates the design of a robot mouth using an LED matrix. The matrix is controlled by an Arduino board in conjunction with the MAX7219 chip, enabling precise control over each LED. The repository includes a basic animation that displays a smiling face on the LED matrix.


# Robot Mouth with LED Matrix

## Hardware Requirements
- Arduino board (e.g., Arduino Uno, Arduino Nano)
- MAX7219 LED matrix driver chip
- 4x4 LED matrix
- Jumper wires
- Breadboard (optional)

## Circuit Diagram
The circuit diagram for this project is as follows:

```
Arduino   MAX7219
  CLK  ->   CLK
  DIN  ->   DIN
  CS   ->   CS
```

## Code
```cpp
#include <SPI.h>
#define CLK 13
#define DIN 11
#define CS  10
#define X_SEGMENTS   4
#define Y_SEGMENTS   4
#define NUM_SEGMENTS (X_SEGMENTS * Y_SEGMENTS)

byte fb[8 * NUM_SEGMENTS];

void shiftAll(byte send_to_address, byte send_this_data)
{
  digitalWrite(CS, LOW);
  for (int i = 0; i < NUM_SEGMENTS; i++) {
    shiftOut(DIN, CLK, MSBFIRST, send_to_address);
    shiftOut(DIN, CLK, MSBFIRST, send_this_data);
  }
  digitalWrite(CS, HIGH);
}

void setup() {
  Serial.begin(115200);
  pinMode(CLK, OUTPUT);
  pinMode(DIN, OUTPUT);
  pinMode(CS, OUTPUT);

  shiftAll(0x0f, 0x00); // Disable test mode
  shiftAll(0x0b, 0x07); // Display all digits
  shiftAll(0x0c, 0x01); // Normal operation
  shiftAll(0x0a, 0x0f); // Maximum brightness
  shiftAll(0x09, 0x00); // No decode mode

  clear();
}

void loop() {
  clear();
  
  // Draw the face with updated positions
  drawLargeSmiley();
  
  show(); // Update the display
  delay(500); // Wait half a second before redrawing
}

void drawLargeSmiley() {
  // Draw eyes with multiple lashes, 4x4 each, wider and shifted down
  for (int i = 10; i <= 13; i++) { // Left eye
    for (int j = 8; j <= 11; j++) {
      set_pixel(i, j, 1);
    }
  }
  for (int i = 10; i <= 13; i++) { // Right eye
    for (int j = 8; j <= 11; j++) {
      set_pixel(i + 13, j, 1);
    }
  }

  // Add lashes to the eyes
  // Left eye lashes
  set_pixel(9, 7, 1); // Top left
  set_pixel(10, 7, 1); // Top middle
  set_pixel(11, 7, 1); // Top right
  set_pixel(12, 7, 1); // Extra top lash
  
  // Right eye lashes
  set_pixel(9 + 13, 7, 1); // Top left
  set_pixel(10 + 13, 7, 1); // Top middle
  set_pixel(11 + 13, 7, 1); // Top right
  set_pixel(12 + 13, 7, 1); // Extra top lash

  // Draw smiling mouth as a clear arc
  int arc_radius = 4;
  int start_x = 10;
  int end_x = 23;

  for (int x = start_x; x <= end_x; x++) {
    int offset = arc_radius - sqrt(pow(arc_radius, 2) - pow(x - (start_x + (end_x - start_x) / 2), 2));
    set_pixel(x, 28 - offset, 1); // Lower position for mouth
  }
}

void set_pixel(uint8_t x, uint8_t y, uint8_t mode) {
  byte *addr = &fb[x / 8 + y * X_SEGMENTS];
  byte mask = 128 >> (x % 8);
  switch (mode) {
    case 0: // clear pixel
      *addr &= ~mask;
      break;
    case 1: 
      *addr |= mask;
      break;
    case 2:
      *addr ^= mask;
      break;
  }
}

void safe_pixel(uint8_t x, uint8_t y, uint8_t mode) {
  if ((x >= X_SEGMENTS * 8) || (y >= Y_SEGMENTS * 8))
    return;
  set_pixel(x, y, mode);
}

void clear() {
  byte *addr = fb;
  for (byte i = 0; i < 8 * NUM_SEGMENTS; i++)
    *addr++ = 0;
}

void show() {
  for (byte row = 0; row < 8; row++) { // Iterate through each row
    digitalWrite(CS, LOW);
    byte segment = NUM_SEGMENTS;
    while (segment--) { 
      byte x = segment % X_SEGMENTS;
      byte y = segment / X_SEGMENTS * 8; // Activate segment 
      byte addr = (row + y) * X_SEGMENTS;

      if (segment & X_SEGMENTS) { 
        shiftOut(DIN, CLK, MSBFIRST, 8 - row);
        shiftOut(DIN, CLK, LSBFIRST, fb[addr + x]);
      } else { 
        shiftOut(DIN, CLK, MSBFIRST, 1 + row);
        shiftOut(DIN, CLK, MSBFIRST, fb[addr - x + X_SEGMENTS - 1]);
      }
    }
    digitalWrite(CS, HIGH);
  }
}
```

## Code Explanation
The provided code includes the following functions:

- **shiftAll(byte send_to_address, byte send_this_data)**: Sends data to all segments of the LED matrix.
- **setup()**: Initializes the Arduino pins and sets up the LED matrix.
- **loop()**: Clears the LED matrix and calls the `drawLargeSmiley()` function to display a large smiley face.
- **drawLargeSmiley()**: Draws the eyes and smiling mouth on the LED matrix, with added eyelashes for a more detailed and "girly" appearance.
- **set_pixel(uint8_t x, uint8_t y, uint8_t mode)**: Sets the state of a single pixel on the LED matrix.
- **safe_pixel(uint8_t x, uint8_t y, uint8_t mode)**: Checks the validity of the pixel coordinates before setting the pixel state.
- **clear()**: Clears the LED matrix by turning off all pixels.
- **show()**: Updates the LED matrix with the contents of the `fb` array.
  
## Usage
1. Connect the Arduino board and the MAX7219 chip according to the circuit diagram.
2. Upload the provided code to the Arduino board.
3. The LED matrix will display a large smiling face with detailed eyelashes on the eyes, updating every 500 milliseconds.

## Robot Face Image

Here’s an illustration of the robot face displayed by the LED matrix:
![image](https://github.com/user-attachments/assets/0aa3450b-9a43-451a-8f4d-0bffd6ae9707)


## Customization
You can customize the robot mouth design by modifying the `drawLargeSmiley()` function. Adjust the position and shape of the eyes and mouth, or add additional features like eyebrows or a nose.

## References
- [MAX7219 Datasheet](https://wokwi.com/projects/403456819190881281)
- [Arduino SPI Library Documentation](https://www.arduino.cc/en/Reference/SPI)
