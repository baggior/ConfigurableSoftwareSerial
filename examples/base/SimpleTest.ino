
#include <ConfigurableSoftwareSerial.h>

ConfigurableSoftwareSerial swSer(14, 12, false, 256);


#define UART_BAUD               19200
#define UART_STOP_BITS          2
#define UART_PARITY             'N'
#define UART_DATA_BITS          7


void setup() {
  Serial.begin(115200);
  swSer.begin(UART_BAUD, UART_STOP_BITS, UART_PARITY, UART_DATA_BITS);

  Serial.println("\nSoftware serial test started");

  for (char ch = ' '; ch <= 'z'; ch++) {
    swSer.write(ch);
  }
  swSer.println("");

}

void loop() {
  while (swSer.available() > 0) {
    Serial.write(swSer.read());
  }
  while (Serial.available() > 0) {
    swSer.write(Serial.read());
  }

}
