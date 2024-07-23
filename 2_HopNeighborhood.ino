#include <AceRoutine.h>
#include <ArduinoJson.h>
#include <RF24.h>

using namespace ace_routine;

RF24 radio(7,8);
const byte addr[6]="bcast";
const unsigned char MY_ADDRESS = 0x9;
#define SZ 32
#define NSZ 24

byte Nodes[NSZ] = {MY_ADDRESS,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
__uint24 OneHops[NSZ] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
byte Timestamps[NSZ] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
byte BroadcastNodes[NSZ] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
__uint24 SequenceMap[256];
int distinctReceivedCount = 0;
int relayedCount = 0;
byte broadcastCount = 0;

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
  formats broadcast packets with incrementing sequence number
*/
void buildBroadcast(byte pkt[]) {
  pkt[0] = 0x30;
  pkt[1] = MY_ADDRESS;
  pkt[2] = 0x0;
  pkt[3] = broadcastCount;
  pkt[4] = MY_ADDRESS;
  pkt[5] = 0x0;
  broadcastCount++;
}

void printStats() {
  Serial.print(F("Total distinct broadcast messages received: "));
  Serial.println(distinctReceivedCount);
  Serial.print(F("Total number of broadcast messages relayed: "));
  Serial.println(relayedCount);
}

/*
  checks incoming packets against previously received broadcast packets
*/
bool uniqueBroadcast(byte pkt[]){
  for(int i = 1; i < NSZ; i++) {
    if (BroadcastNodes[i] == pkt[1]) {
      if (SequenceMap[pkt[3]] >> i & 1 == 1) {
        return false;
      } else {
        SequenceMap[pkt[3]] |= (1 << i);
        distinctReceivedCount++;
        return true;
      }
    }
  }
  for (int i = 1; i < NSZ; i++) {
    if (BroadcastNodes[i] == 0) {
      BroadcastNodes[i] = pkt[1];
      SequenceMap[pkt[3]] |= (1 << i);
      distinctReceivedCount++;
      return true;
    }
  }
  Serial.println(F("Unique broadcast error"));
  return false;
}

/*
  checks if incoming coverages set is less then or equal to two hop neighborhood
  any nodes existing in two neighborhood that are not yet covered will be appended to coverage set and broadcast retransmitted
*/
bool isCovered(byte pkt[]) {
  byte toAppend[NSZ];
  byte appendCount = 0;
  for(int i = 1; i < NSZ; i++){
    for(int j = 0; j < pkt[5]; j++){
      if(Nodes[i] == pkt[6 + j]) {
        break;
      } else {
        toAppend[appendCount] = Nodes[i];
        appendCount++;
      }
    }
  }
  if(appendCount == 0) {
    return true;
  } else {
    for(int i = 0; i < appendCount; i++) {
    pkt[5 + pkt[5]] = toAppend[appendCount];
  }
  pkt[5] += appendCount;
    return false;
  }
}

/*
  determines if broadcast is to be retransmitted
  transmits broadcast with updated sender address and coverage set
*/
void relay(byte pkt[]) {
  if (uniqueBroadcast(pkt) && !isCovered(pkt)) {
    radio.openWritingPipe(addr);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();
    radio.setAutoAck(false);
    pkt[4] = MY_ADDRESS;
    if (radio.write(pkt, SZ)) {
      Serial.println(F("Relay sent"));
      relayedCount++;
    } else {
      Serial.println(F("Relay error"));
    }
  } else {
    Serial.println(F("Relay denied"));
  }
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
  transmits broadcast packets with set delay
*/
class Broadcast : public ActionDelay {
  public:
    Broadcast(int arg) {
      _delay = 10000;
    }

    void action() {

      radio.openWritingPipe(addr);
      radio.setPALevel(RF24_PA_MIN);
      radio.stopListening();
      radio.setAutoAck(false);
      byte pkt[SZ];
      buildBroadcast(pkt);
      if (radio.write(pkt, SZ)) {
        Serial.println(F("Broadcast sent"));
      } else {
        Serial.println(F("Broadcast error"));
      }
      printStats();
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
          }/* else if (packet[0] >> 4 == 3) {
            Serial.print(F("Received broadcast: "));
            Serial.println(packet[1]);
            relay(packet);
          }*/
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
Broadcast broadcast(1);
OneHop oneHop(1);
Listener listener();
ScheduleAction h(&hello);
ScheduleAction b(&broadcast);
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