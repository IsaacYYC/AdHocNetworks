#include <AceRoutine.h>
#include <ArduinoJson.h>
#include <RF24.h>

using namespace ace_routine;

RF24 radio(7,8);
const byte addr[6]="bcast";
const unsigned char MY_ADDRESS = 0x9;
#define SZ 32
#define NSZ 24

/*
Initialization of data structure 
*/
byte Nodes[NSZ] = {MY_ADDRESS,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
__uint24 OneHops[NSZ] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
byte Timestamps[NSZ] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

/*
  prints one and two hop neighborhood information to serial monitor
*/
void print() {
  Serial.print(F("Two hop neighborhood:\n"));
  for(int i = 1; i < NSZ; i++) {
    if(Nodes[i] != 0 && (OneHops[0] >> i & 1) == 1) {
      Serial.print(Nodes[i]);
      Serial.print(F(": "));
      for(int j = 0; j < NSZ; j++) {
        if((OneHops[i] >> j & 1) == 1 && Nodes[j] != 0) {
          Serial.print(Nodes[j]);
          Serial.print(F(" "));
        }
      }
      Serial.print(F("\n"));
    }
  }
  Serial.print(F("\n"));
}

/*
  tracks amount of time since last received hello packet for all one and two hop neighbors
  cleans expired nodes
*/
void clean() {
  for(int i = 1; i < NSZ; i++) {
    if(Timestamps[i] > 6) {
      Timestamps[i] = 0;
      Nodes[i] = 0;
    } else if (Nodes[i] != 0) {
      Timestamps[i] = Timestamps[i] + 2;
    }
  }
}

/*
  inserts or updates one and two neighborhood information from incoming hello packets
*/
void insert(byte add[]) {
  int oneHopLocation = NULL;
  for(int i = 1; i < NSZ; i++) {
    if(Nodes[i] == add[1]) {
      oneHopLocation = i;
      Timestamps[i] = 0;
      OneHops[i] = 0;
      OneHops[0] |= (1 << i) ;
      break;
    }
  }
  if(oneHopLocation == NULL) {
    for(int i = 1; i < NSZ; i++) {
      if(Nodes[i] == 0) {
        Nodes[i] = add[1];
        oneHopLocation = i;
        Timestamps[i] = 0;
        OneHops[i] = 0;
        OneHops[0] |= (1 << i) ;
        break;
      }
    }
  }
  if(oneHopLocation != NULL) {
    for(int j = 0; j < add[2]; j++) {
      bool twoHopInserted = false;
      for(int i = 1; i < NSZ; i++) {
        if (Nodes[i] == add[3 + j]) {
          Timestamps[i] = 0;
          OneHops[i] = 0;
          OneHops[oneHopLocation] |= (1 << i);
          twoHopInserted = true;
          break;
        }
      }
      if (!twoHopInserted) {
        for(int i = 1; i < NSZ; i++) {
          if (Nodes[i] == 0) {
            Nodes[i] = add[j + 3];
            Timestamps[i] = 0;
            OneHops[i] = 0;
            OneHops[oneHopLocation] |= (1 << i);
            twoHopInserted = true;
            break;
          }
        }
      }
    }
  }
}

/*
  formats hello packet to transmit one hop neighborhood information
*/
void buildHello(byte pkt[]) {
  pkt[0] = 0x50;
  pkt[1] = MY_ADDRESS;
  int count = 0;
  for(int i = 1; i < NSZ; i++) {
    if(Nodes[i] != 0 && (OneHops[0] >> i & 1) == 1) {
      Serial.print(Nodes[i]);
      pkt[count + 3] = Nodes[i];
      count++;
    }
  }
  pkt[2] = count;
}

/*
  delay functionality for coroutines
*/
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

/*
  clean and display neighborhood information with set delay
*/
class OneHop : public ActionDelay {
  public:
    OneHop(int arg) {
      _delay = 2000;
    }

    void action() {
      clean();
      print();
      }
};

/*
  transmits hello packets with set delay
*/
class Hello : public ActionDelay {
  public:
    Hello(int arg) {}

    void action() {

      radio.openWritingPipe(addr);
      radio.setPALevel(RF24_PA_MIN);
      radio.stopListening();
      radio.setAutoAck(false);
      byte pkt[SZ];
      buildHello(pkt);
      if (radio.write(pkt, SZ)) {
        Serial.println(F("Hello sent"));
      } else {
        Serial.println(F("Hello error"));
      }
      _delay = random(1000, 5000);
    }
};

/*
  listens and parses incoming packets
*/
class Listener : public Coroutine {
  private:
    byte packet[SZ];
  public:

    int runCoroutine() override {
      Serial.println(F("begin listening"));
      radio.openReadingPipe(1, addr);
      radio.setPALevel(RF24_PA_MIN);
      radio.startListening();
      radio.setAutoAck(false);
      COROUTINE_LOOP() {
        if (radio.available()) {
          radio.read(packet, SZ);
          Serial.println(F("received a packet"));
          if (packet[0] >> 4 == 5) {
            Serial.print(F("Received hello: "));
            Serial.println(packet[1]);
            insert(packet);
          }
        } else {
          Serial.print(F("radio unavailable"));
        }
        COROUTINE_YIELD();
      }
    }
};

/*
  schedules functions utilizing action delays
*/
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

Hello hello(1);
OneHop oneHop(1);
Listener listener();
ScheduleAction h(&hello);
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

void loop() {
  CoroutineScheduler::loop();
}