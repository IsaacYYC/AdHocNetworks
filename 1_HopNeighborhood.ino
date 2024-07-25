#include <AceRoutine.h>
#include <ArduinoJson.h>
#include <RF24.h>

using namespace ace_routine;

RF24 radio(7,8);
const byte addr[6]="bcast";
const unsigned char MY_ADDRESS = 0x9;
#define SZ 32
#define NSZ 27

struct OneHopData {
  int sourceAddr = 0;
  unsigned char OneHops[NSZ] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  unsigned long timeStamp = 0;
};

class DataHelper {
  private:
    OneHopData* addresses;
  public:
    DataHelper(OneHopData* n) : addresses(n) {}

  void insert(byte add[]) {
        for(int i = 0; i < NSZ; i++) {
          if (addresses[i].sourceAddr == add[1]) {
            addresses[i].timeStamp = millis();
            for (int j = 0; j < NSZ; j++) {
              addresses[i].OneHops[j] = 0;
            }
            for (int j = 0; j < add[2]; j++) {
              addresses[i].OneHops[j] = add[j + 3];
            }
            break;
          } else if (addresses[i].sourceAddr == 0) {
            addresses[i].sourceAddr = add[1];
            addresses[i].timeStamp = millis();
            for (int j = 0; j < NSZ; j++) {
              addresses[i].OneHops[j] = 0;
            }
            for (int j = 0; j < add[2]; j++) {
              addresses[i].OneHops[j] = add[j + 3];
            }
            break;
          }
        }
    }

    void print() {
      Serial.print(F("One hop neighborhood: \n"));
      for (int i = 0; i < NSZ; i++) {
        if (addresses[i].sourceAddr != 0) {
          Serial.print(addresses[i].sourceAddr);
          Serial.print(F(": "));
          for (int j = 0; j < NSZ; j++){
            if (addresses[i].OneHops[j] != 0) {
              Serial.print(addresses[i].OneHops[j]);
              Serial.print(F(" "));
            }
          }
          Serial.print(F("\n"));
        }
      }
      Serial.print(F("\n"));
    }

    void clean() {
      for (int i = 0; i < NSZ; i++) {
        if ( millis() - addresses[i].timeStamp > 6000) {
          addresses[i].sourceAddr = 0;
          addresses[i].timeStamp = 0;
          for (int j = 0; j < NSZ; j++) {
              addresses[i].OneHops[j] = 0;
            }
        }
      }
    }

    void buildPacket(unsigned char result[SZ]) {
      int count = 0;
      for (int i = 0; i < NSZ; i++) {
        if (addresses[i].sourceAddr != 0) {
          result[count + 3] = addresses[i].sourceAddr;
          count++;
        }
      }
      result[2] = count;
    }
};

class ActionDelay {
  protected:
    int _delay;
  public:
    void setDelay(int i) {
      _delay = i;
    }

    int getDelay() {
      return _delay;
    }

    virtual void action() = 0;
};

class OneHop : public ActionDelay {
  private:
    OneHopData* neighbors;
  public:
    OneHop(OneHopData* n) : neighbors(n) {
      _delay = 2000;
    }

    void action() {
      DataHelper helper(neighbors);
      helper.clean();
      helper.print();
      }
};

class Sender : public ActionDelay {
  private:
    OneHopData* neighbors;
  public:
    Sender(OneHopData* n) : neighbors(n) {}

    void action() {

      radio.openWritingPipe(addr);
      radio.setPALevel(RF24_PA_MIN);
      radio.stopListening();
      radio.setAutoAck(false);
      DataHelper helper(neighbors);
      byte pkt[SZ] = {0x50, MY_ADDRESS};
      helper.buildPacket(pkt);
      if (radio.write(pkt, SZ)) {
        Serial.println(F("Transmission sent"));
      } else {
        Serial.println(F("Transmission error"));
      }
      _delay = random(1000, 5000);
    }
};

class Listener : public Coroutine {
  private:
    OneHopData* neighbors;
    byte packet[SZ];
  public:
    Listener(OneHopData* n) : neighbors(n) {}

    int runCoroutine() override {
      radio.openReadingPipe(1, addr);
      radio.setPALevel(RF24_PA_MIN);
      radio.startListening();
      radio.setAutoAck(false);
      COROUTINE_LOOP() {
        if (radio.available()) {
          radio.read(packet, SZ);
          if (packet[0] >> 4 == 5) {
            Serial.print(F("Received: "));
            Serial.println(packet[1]);
            DataHelper helper(neighbors);
            helper.insert(packet);
          }
        }
        COROUTINE_YIELD();
      }
    }
};

class ScheduleAction: public Coroutine {
  private:
    ActionDelay * a;
  public:
    ScheduleAction(ActionDelay * set) {
      a = set;
    }

  int runCoroutine() override {
    COROUTINE_LOOP() {
      a->action();
      COROUTINE_DELAY(a->getDelay());
    }
  }
};

OneHopData neighbors[NSZ];
Sender sender(neighbors);
Listener listener(neighbors);
OneHop oneHop(neighbors);
ScheduleAction s(&sender);
ScheduleAction o(&oneHop);

void setup()
{
  Serial.begin(9600);
  if (radio.begin()) {
    Serial.println(F("radio hardware OK"));
  } else {
    Serial.println(F("radio hardware not responding"));
  }
  CoroutineScheduler::setup();
}

// Demonstration of output using classes
void loop() {
  CoroutineScheduler::loop();
}