# ConfigurableSoftwareSerial
## An Arduino framework library for the ESP8266 Platform
use this library if you need a software Serial that needs to be configured in a different way rather the default 8N1 (8 bit of data, no parity, 1 stop bit).

Based from the ESPSoftwareSerial library modified to support other serial configuration options

## Configuration options and defaults
- Stop bits:
  - 1
  - 2

- Parity: 
  - 'E': even
  - 'O': odd
  - 'N': none

- Data bits: 
  - 5
  - 6
  - 7
  - 8
  - *note: more not supported (reverted to8)*

## Usage
On startup call the overloaded **begin** method with the following signature

```c

void begin(long speed, int stopbits = 1, char parity = 'N', int databits=8);

```

