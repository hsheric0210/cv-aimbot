#include <Mouse.h>

// Original source
// https://github.com/harunuysali07/ValorantAimBot/blob/main/Arduino Files/Mouse.ino

//#define DEBUG
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

const char MouseMoveMagic = 'm';
const char MouseClickMagic = 'c';
const int mlimit = 127;

// Packet has fixed size: 5-byte (40 bits)
// Packet format:
// {
//   1-byte - Magic (Opcode)
//   2-byte - Parameter 1 (int)
//   2-byte - Parameter 2 (int)
// }
char buffer[5];

struct mPacket {
  uint8_t magic;
  uint16_t param1, param2;
};

// Mouse move packet
// Send from python: ser.write(b'm' + int(x).to_bytes(2, byteorder='little', signed=True) + int(y).to_bytes(2, byteorder='little', signed=True))
struct mMove {
  uint8_t magic;
  int16_t x, y;
};

// Mouse click packet
// Send from python: ser.write(b'c' + int(button).to_bytes(2, byteorder='little', signed=True) + b'\x00\x00', signed=True))
struct mClick {
  uint8_t magic;
  uint16_t button, param2;
};

void setup() {
  Mouse.begin();
  Serial.begin(500000);
}

void loop() {
  if (!Serial.available())
    return;
  int recv;
  if ((recv = Serial.readBytes(buffer, 5)) != 5)
  {
    Serial.print("EMSGLEN=");
    Serial.println(recv);
    return;
  }
  
  mPacket* packet = (mPacket*)buffer;
  if (packet->magic == MouseMoveMagic)
  {
    mMove* move = (mMove*)buffer;
    int x = move->x, y = move->y;
#ifdef DEBUG
    Serial.print("RM");
    Serial.print((int)x);
    Serial.print(",");
    Serial.println((int)y);
#endif
    int ax, ay, rx, ry;
    while (x != 0 || y != 0)
    {
      rx = x; ry = y;
      ax = abs(x); ay = abs(y);
      if (ax > mlimit)
        rx = mlimit * sgn(x);
      if (ay > mlimit)
        ry = mlimit * sgn(y);
      Mouse.move(rx, ry, 0);
#ifdef DEBUG
      Serial.print("M");
      Serial.print((int)rx);
      Serial.print(",");
      Serial.println((int)ry);
#endif
      x -= rx;
      y -= ry;
    }
  }
  else if (packet->magic == MouseClickMagic)
  {
    mClick* click = (mClick*)buffer;
    Mouse.click(click->button);
#ifdef DEBUG
    Serial.print("C");
    Serial.println((int)click->button);
#endif
  }
  else
  {
    Serial.print("EOPCODE=");
    Serial.println(packet->magic);
  }
}
