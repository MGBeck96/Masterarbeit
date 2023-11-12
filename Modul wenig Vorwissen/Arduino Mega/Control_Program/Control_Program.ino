/*
  Programm zur Steuerung eines Roboters mit einem Arduino Mega für das Modul für Schüler*innen mit wenig Vorwissen
  Ein Micro:Bit ist über die SPI-Schnittstelle angeschlossen. Dafür ist der Arduino Mega als SPI-Sub konfiguriert.
  SPI-Pins: ChipSelect: Pin 53 ; CLK: Pin 52 ; MOSI: Pin 51 ; MISO: Pin 50
  Ein ESP32 ist über eine UART-Verbindung angeschlossen. Dazwischen ist ein Level-Shifter geschaltet von 5V auf 3.3V
  UART-Pins: RX3 (Pin 15) und TX3 (Pin 14)
  Ein DOBOT Magician ist über eine UART-Verbindung angeschlossen.
  UART-Pins: RX1 (Pin 19) und TX1 (Pin 18)
*/
#include "SPI.h" // Arduion Bibliothek für SPI
#include "stdio.h"
#include "Protocol.h"    // DOBOT Magician Bibliothek
#include "command.h"     // DOBOT Magician Bibliothek
#include "FlexiTimer2.h" // Timer-Bibliothek benötigt für die DOBOT Bibliothek

// Set Serial TX&RX Buffer Size
#define SERIAL_TX_BUFFER_SIZE 64
#define SERIAL_RX_BUFFER_SIZE 256

/*********************************************************************************************************
** I/O Pins
*********************************************************************************************************/
const int SensorPin = 46; // Definition von SensorPin an Pin 46 für das Förderband
int sensorValue = 1;      // Wert von Sensor an Band, 1 =  nichts vor Sensor, 0 = Gegenstand vor Sensor
/*********************************************************************************************************
** Globale Parameter
*********************************************************************************************************/
// Parameter sind aus der DOBOT-Bibliothek übernommen
PTPCoordinateParams gPTPCoordinateParams;
PTPCommonParams gPTPCommonParams;
PTPCmd gPTPCmd;
Pose gPose;
Message gMessage;
EMotor gMotorCMD;

uint64_t gQueuedCmdIndex;
/*********************************************************************************************************
** Globale Variablen
*********************************************************************************************************/

// Sichere Start-Position
float startPosX = 200.00;
float startPosY = 0.00;
float startPosZ = 40.00;
float startPosR = 0.00;
// Positionen für Roboterarm
float currentX = startPosX;
float currentY = startPosY;
float currentZ = startPosZ;
float currentR = startPosR;
bool currentVac = false;

int selectRobot = 0;      // Aktiver Roboter, vorbereitet für zwei Roboter beim Abschicken der Nachricht in Protocol.cpp
uint8_t motorEnabled = 0; // Status des Förderbands -> 0 = aus, 1 = ein

// Union um die Ankommende Nachricht vom Roboter in Float zu konvertieren
// Gleicher Speicherbereich wird als Bytestring und als vier Float-Werte interpretiert
typedef union
{
  unsigned char a_c[16];
  float f_a[4];
} dobot_to_float;
dobot_to_float dfL; // Für linken Arm
// dobot_to_float dfR; // Für rechten Arm

/*****************************************************************************************************
 ********************************************** SPI *************************************************
 ****************************************************************************************************/
char buff[50];
char message[50];
volatile byte indx;
volatile boolean process;       // Flag für abgeschlossene SPI Nachricht. Wird true bei "\r" als angekommene Nachricht.
boolean spi_angekommen = false; // Zweite Flag, die ein Senden des Aufgaben-Arrays an den ESP32 auslöst
// Maximum der Substrings
const int max_substrings = 7;
// Array aus Zeigern, die auf die einzelnen Substrings zeigen
static char *substrings[max_substrings];
// Anzahl der Aufgaben
const int anzahlAufgaben = 20;
// Struct zum Speichern der Gruppen-Nachrichten für zwei Roboter
/*
  struct AufgabenDaten {
  bool angekommen;
  unsigned char roboter;
  float x_coord;
  float y_coord;
  float z_coord;
  bool vac;
  // Setze default-Werte
  AufgabenDaten() : angekommen(false), roboter(0), x_coord(0.0), y_coord(0.0), z_coord(0.0), vac(false) {}
  };
  AufgabenDaten aufgaben[anzahlAufgaben];
*/
struct AufgabenDatenSingle
{
  bool angekommen;
  bool band;
  float x_coord;
  float y_coord;
  float z_coord;
  bool vac;
  // Setze default-Werte
  AufgabenDatenSingle() : angekommen(false), band(false), x_coord(0.0), y_coord(0.0), z_coord(0.0), vac(false) {}
};
// Array aus dem Struct - für 20 Aufgaben
AufgabenDatenSingle aufgaben[anzahlAufgaben];

/*****************************************************************************************************
 ********************************************** UART *************************************************
 ****************************************************************************************************/
char uartBuff[10];
char uartMsg[10];
// Variablen für die Modi: Zu Beginn Fernsteuerung, frei bewegbar. Danach Programmablauf aufzeichnen, danach Programm starten
bool fernsteuerung = true;
bool aufzeichnen = false;
bool startProgramm = false;

/*********************************************************************************************************
********************************************* Funktionen *************************************************
*********************************************************************************************************/

// Interrupt Service Routine, die bei ChipSelect an Pin 53 reagiert
// Kurz halten, da zeitkritisch
ISR(SPI_STC_vect) // SPI interrupt routine
{
  byte c = SPDR; // Lese angekommenes Byte aus dem SPI Data Register
  if (indx < sizeof buff)
  {
    buff[indx++] = c; // Speichere angekommenes Byte im Buffer
    if (c == '\r')    // Schaue, ob Flag für Ende der Nachricht angekommen ist
      process = true;
  }
}

void initSPI()
{
  pinMode(MISO, OUTPUT); // Setze Main In Sub Out als Output
  SPCR |= _BV(SPE);      // Setze den SPI-Modus als Sub
  indx = 0;              // setze den indx-Buffer auf 0
  process = false;       // setze Marker für vollständige Nachricht auf false
  SPI.attachInterrupt(); // aktiviere SPI-Interrupt
}

void initSerial()
{
  // Kommunikation mit Debug-Konsole über USB
  Serial.begin(115200);
  // Serial 1 für DOBOT Magician
  Serial1.begin(115200);
  // Serial 2 für zweiten Roboter
  // Serial2.begin(115200);
  // Serial 3 für ESP32
  Serial3.begin(115200);
  // Setze Timer Interrupt, der alle 100ms den Serial1-Port abhört für Nachrichten vom Roboter
  FlexiTimer2::set(100, Serial1read);
  FlexiTimer2::start();
}

void split_message(char msg)
{
  // Lösche alle Daten aus dem Substring-Array, da dort die alte Nachricht steht
  for (int i = 0; i < max_substrings; i++)
    substrings[i] = 0;
  // Teile die eingehende Nachricht an dem Marker "#"
  char *text = strtok(msg, "#");
  int i = 0;
  while (text != 0 && i < max_substrings)
  {
    // Ein Marker wurde gefunden und die Nachricht in das Substring-Array gespeichert
    substrings[i++] = text;
    text = strtok(0, "#");
  }
}

// Unterprogramm, das die vom Micro:Bit empfangene Nachricht verwertet
void message_handler(char *message)
{
  char msg[30];
  // strcpy, damit die Nachricht nicht durch einen Interrupt verändert wird
  strcpy(msg, message);
  // memcpy(&msg, message, 30);
  split_message(msg);
  // Format der Nachrichten
  // Für einen Roboter:         [AufgabenID]#[X]#[Y]#[Z]#[Vakuum]#\r
  // Oder zum Start des Bands:  [AufgabenID]#Band#\r
  // Falsche Nachrichten wurden schon vom Micro:Bit abgefangen
  int i = atoi(substrings[0]); // i ist hier die AufgabenID
  aufgaben[i].angekommen = true; // Nachricht angekommen, deshalb wird sie berücksichtigt und auf der Website dargestellt
  if (strcmp(substrings[1], "true") == 0)
  {
    // Nachricht für starte Band ist angekommen
    aufgaben[i].band = true;
    return;
    // Nur das wichtig, der Rest bleibt bei 0
  }
  // Wenn Koordinaten-Nachricht
  // X-Koordinate
  aufgaben[i].x_coord = atof(substrings[1]);
  // Y-Koordinate
  aufgaben[i].y_coord = atof(substrings[2]);
  // Z-Koordinate
  aufgaben[i].z_coord = atof(substrings[3]);
  // Pumpe an oder aus
  aufgaben[i].vac = (strcmp(substrings[4], "true") == 0);
  // FÜR BEIDE ROBOTER
  /*
    // Roboter Auswahl 0=links, 1=rechts
    aufgaben[i].roboter = atoi(substrings[1]);
    // X-Koordinate
    aufgaben[i].x_coord = atof(substrings[2]);
    // Y-Koordinate
    aufgaben[i].y_coord = atof(substrings[3]);
    // Z-Koordinate
    aufgaben[i].z_coord = atof(substrings[4]);
    // Pumpe an oder aus
    aufgaben[i].vac = (atoi(substrings[5]) == 1);
  */
}

void completedMessageReceived()
{
  // wenn \r ankommt (-Konvention, dass dann Übertragung fertig ist-), wird process = true
  // dann gilt es die Nachricht zu verarbeiten
  // Konvention: AufgabenID#payload mit # getrennt -> zB. x#y#z
  // abschließendes "\r"
  if (strcmp(message, buff) != 0)
  { 
    // nur wenn buff sich ändert aktualisiere message -> "Prellschutz"
    strcpy(message, buff); // kopiert den volatilen char array "buff" in den char array "message"
    /*
    Serial.println("***********************************");
    Serial.println("SPI Nachricht angekommen");
    Serial.println(message);
    */
    message_handler(message);
  }
  for (int i = 0; i < 50; i++)
  {
    // Den buffer löschen
    buff[i] = 0;
  }
  process = false; // Neue Nachricht abgearbeitet, also process = false
  indx = 0;        // Setze index für buff[indx] wieder auf 0
}

void setDemo()
{
  // Fülle das Aufgaben-Array mit Daten (nur zu Demonstrationszwecken)
  aufgaben[0].angekommen = true; // Aufnehmen;
  aufgaben[0].band = false;
  aufgaben[0].x_coord = 158.0;
  aufgaben[0].y_coord = 100.0;
  aufgaben[0].z_coord = -46.0;
  aufgaben[0].vac = true;

  aufgaben[1].angekommen = true; // Safe Pos;
  aufgaben[1].band = false;
  aufgaben[1].x_coord = 182.0;
  aufgaben[1].y_coord = 61.0;
  aufgaben[1].z_coord = 45.0;
  aufgaben[1].vac = true;

  aufgaben[2].angekommen = true; // abwerfen;
  aufgaben[2].band = false;
  aufgaben[2].x_coord = 264.0;
  aufgaben[2].y_coord = -17.0;
  aufgaben[2].z_coord = 22.0;
  aufgaben[2].vac = false;

  aufgaben[3].angekommen = true; // Safe Pos 2;
  aufgaben[3].band = false;
  aufgaben[3].x_coord = 232.0;
  aufgaben[3].y_coord = 54.0;
  aufgaben[3].z_coord = 65.0;
  aufgaben[3].vac = false;

  aufgaben[4].angekommen = true; // Aufnehmen 2;
  aufgaben[4].band = false;
  aufgaben[4].x_coord = 232.0;
  aufgaben[4].y_coord = 99.0;
  aufgaben[4].z_coord = 8.0;
  aufgaben[4].vac = true;

  aufgaben[5].angekommen = true; // Safe Pos 3;
  aufgaben[5].band = false;
  aufgaben[5].x_coord = 232.0;
  aufgaben[5].y_coord = 99.0;
  aufgaben[5].z_coord = 60.0;
  aufgaben[5].vac = true;

  aufgaben[6].angekommen = true; // abwerfen 2;
  aufgaben[6].band = false;
  aufgaben[6].x_coord = 264.0;
  aufgaben[6].y_coord = -17.0;
  aufgaben[6].z_coord = 40.0;
  aufgaben[6].vac = false;

  aufgaben[7].angekommen = true; // Band anschalten;
  aufgaben[7].band = true;
  aufgaben[7].x_coord = 0.0;
  aufgaben[7].y_coord = 0.0;
  aufgaben[7].z_coord = 0.0;
  aufgaben[7].vac = false;

  for (unsigned char i = 8; i < 20; i++)
  {
    aufgaben[i].angekommen = false;
    aufgaben[i].band = false;
    aufgaben[i].x_coord = 0;
    aufgaben[i].y_coord = 0;
    aufgaben[i].z_coord = 0;
    aufgaben[i].vac = false;
  }
  // Setze spi_angekommen, damit an ESP übermittelt wird
  spi_angekommen = true;
}

/*********************************************************************************************************
************* Setup-Methode die alle nötigen Einstellungen und Konfigurationen durchführt ****************
*********************************************************************************************************/

void setup()
{
  // Füllen das Aufgaben-Array mit Daten (nur zu Demonstrationszwecken)
  pinMode(SensorPin, INPUT); // Setze SensorPin als Eingang
  initSerial();
  initSPI();
  InitRAM();
  ProtocolInit();
  // Lösche Alarm-Status des Roboters. Falls Kollision passiert ist oder andere Probleme aufgetreten sind.
  ProtocolClearAlarmState();
  delay(50);
  ProtocolProcess(selectRobot, &gMessage);
  delay(500);
  GoHome();
  ProtocolProcess(selectRobot, &gMessage);
  Serial.println("====== Steuerung bereit ======");
  // Fahre zur sicheren Startposition
  moveArm(startPosX, startPosY, startPosZ, startPosR, false);
  delay(500);
}

/*********************************************************************************************************
********************************* Lese Daten an Serial1 aus **********************************************
*********************************************************************************************************/
void Serial1read()
{
  // Methode ist aus der DOBOT-Bibliothek übernommen
  while (Serial1.available())
  {
    uint8_t data = Serial1.read();
    if (RingBufferIsFull(&gSerialProtocolHandler.rxRawByteQueue) == false)
    {
      RingBufferEnqueue(&gSerialProtocolHandler.rxRawByteQueue, &data);
    }
  }
}
/*********************************************************************************************************
********************** Initailisiere die Datentypen aus der DOBOT Bibliothek *****************************
*********************************************************************************************************/
void InitRAM(void)
{
  // Setze Point to Point Variablen
  // Methode ist aus der DOBOT-Bibliothek übernommen
  gPTPCoordinateParams.xyzVelocity = 100;
  gPTPCoordinateParams.rVelocity = 100;
  gPTPCoordinateParams.xyzAcceleration = 80;
  gPTPCoordinateParams.rAcceleration = 80;

  gPTPCommonParams.velocityRatio = 50;
  gPTPCommonParams.accelerationRatio = 50;

  // gPTPCmd.ptpMode = JUMP_XYZ; // Für ein "springen" von Punkt zu Punkt
  gPTPCmd.ptpMode = MOVL_XYZ; // Für eine lineare Bewegung von Punkt zu Punkt

  gQueuedCmdIndex = 0;

  gMotorCMD.index = 0;
  gMotorCMD.isEnabled = 0;       // Motor an oder aus
  gMotorCMD.speed_motor = 10000; // Gute Geschwindigkeit, ca 60mm pro sec
}

/*********************************************************************************************************
*********************************** Bewege den Robterarm *************************************************
*********************************************************************************************************/

void moveArm(float x, float y, float z, float r, bool vacuumOn)
{
  // Methode um den Roboterarm zu bewegen
  gPTPCmd.x = x;
  gPTPCmd.y = y;
  gPTPCmd.z = z;
  gPTPCmd.r = r;

  Serial.print("move to x:");
  Serial.print(gPTPCmd.x);
  Serial.print(" y:");
  Serial.print(gPTPCmd.y);
  Serial.print(" z:");
  Serial.println(gPTPCmd.r);
  // Setze den Point to Point Befehl
  SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
  // Sende die Nachricht an den Roboter
  ProtocolProcess(selectRobot, &gMessage);
  delay(500);
  // Warte 0.5 Sekunden bis der Sauger aktiviert oder deaktiviert wird
  if (vacuumOn == false)
    SetEndEffectorSuctionCup(false, false, true, &gQueuedCmdIndex);
  if (vacuumOn == true)
    SetEndEffectorSuctionCup(true, true, true, &gQueuedCmdIndex);
  ProtocolProcess(selectRobot, &gMessage);
}

/*********************************************************************************************************
**************************** Frage Position ab und speichere sie *****************************************
*********************************************************************************************************/

void getPoseToFloat()
{
  // Setze Positionsabfrage-Befehl
  GetPose();
  // Sende die Nachricht an den Roboter
  ProtocolProcess(selectRobot, &gMessage, true); // erwarte Antwort
  for (int i = 0; i < 16; i++)
  {
    // in g.Message.params steht der Antwort-Byte-String des DOBOT
    // Hier sind nur die ersten 16 Byte relevant für die 4 Floats -> Siehe "DOBOT Communication Protocol" bei Message ID = 10
    dfL.a_c[i] = gMessage.params[i];
  }
  // Sende die 16 Byte an den ESP32
  Serial3.write('C'); // Schicke C für Koordinaten
  Serial3.write('0'); // Schicke 0 für Links
  Serial3.write(dfL.a_c, 16);
  // Serial3.write("\r");
  //  Nachricht ist verarbeitet, nun zu float konvertieren
  currentX = dfL.f_a[0];
  currentY = dfL.f_a[1];
  currentZ = dfL.f_a[2];
  currentR = dfL.f_a[3];
}

/*********************************************************************************************************
************************ Frage ab, ob eine Nachricht von der Website kommt *******************************
*********************************************************************************************************/

void checkUARTESP32()
{
  if (Serial3.available() >= 1)
  {
    Serial.print("Nachricht ESP32 ");
    // Die 16 Bytes empfangen und im Byte-Array speichern
    for (int i = 0; i < 10; i++)
    {
      uartBuff[i] = Serial3.read();
    }
    strcpy(uartMsg, uartBuff); // kopiert den volatilen char array "buff" in den char array "message"
    // Button Reset wurde gedrückt
    if (uartMsg[0] == 'V')
    {
      fernsteuerung = true;
      aufzeichnen = false;
      startProgramm = false;
      // Lösche alle Einträge im aufgaben[]-Array
      for (int i = 0; i < anzahlAufgaben; i++)
      { 
        aufgaben[i].angekommen = false;
        aufgaben[i].band = false;
        aufgaben[i].x_coord = 0.0;
        aufgaben[i].y_coord = 0.0;
        aufgaben[i].z_coord = 0.0;
        aufgaben[i].vac = false;
      }
      Serial.println("Neustart");
      // Setze spi_angekommen, damit leeres aufgaben-Array übermittelt wird
      spi_angekommen = true;
      delay(50);
    }
    // Button Aufzeichnen wurde gedrückt
    if (uartMsg[0] == 'R')
    {
      fernsteuerung = false;
      aufzeichnen = true;
      startProgramm = false;
      Serial.println("Aufzeichnen");
    }
    // Button Starte Programm wurde gedrückt
    if (uartMsg[0] == 'S')
    {
      fernsteuerung = false;
      aufzeichnen = false;
      startProgramm = true;
      Serial.println("Start Programm");
      delay(50);
    }
    // Button Home wurde gedrückt
    if (uartMsg[0] == 'H')
    {
      GoHome();
      ProtocolProcess(selectRobot, &gMessage);
      delay(10000);
      // Warte 10 Sekunden und fahre dann an sichere Position
      moveArm(startPosX, startPosY, startPosZ, startPosR, false);
      ProtocolProcess(selectRobot, &gMessage);
    }
    // Button Demo Programm wurde gedrückt
    if (uartMsg[0] == 'D')
    {
      setDemo(); // Lade die Demo-Koordinaten
      delay(50);
    }
    // Lösche Buffer für unterschiedlich lange Nachrichten
    for (int i = 0; i < 10; i++)
    {
      buff[i] = 0;
    }
  }
}

/*********************************************************************************************************
******************** Sende die von den SuS gesendeten Nachrichten an die Website *************************
*********************************************************************************************************/

void sendToESPSingle()
{
  // Für einen Roboter
  int counter = 0;
  for (unsigned char i = 0; i < anzahlAufgaben; i++)
  {
    // Konvertiere die Daten in Bytes
    byte *dataBytes = (byte *)&aufgaben[i];
    // Sende die Bytes über UART
    Serial3.write('G');
    Serial3.write(i);
    byte aufgabenBytesSingle[sizeof(AufgabenDatenSingle)];
    memcpy(aufgabenBytesSingle, &aufgaben[i], sizeof(AufgabenDatenSingle));
    Serial3.write(aufgabenBytesSingle, sizeof(AufgabenDatenSingle));
    delay(50); // Schicke Nachrichtenblock alle 50ms, um ESP32 nicht zu überlasten
  }
}

/*
   Für beide Roboter
  void sendToESP()
  {
  // Für beie Roboter
  int counter = 0;
  for (unsigned char i = 0; i < anzahlAufgaben; i++) {
    // Konvertiere die Daten in Bytes
    byte* dataBytes = (byte*)&aufgaben[i];
    // Sende die Bytes über UART
    Serial3.write('G');
    Serial3.write(i);
    byte aufgabenBytes[sizeof(AufgabenDaten)];
    memcpy(aufgabenBytes, &aufgaben[i], sizeof(AufgabenDaten));

    Serial3.write(aufgabenBytes, sizeof(AufgabenDaten));
    //for (int j = 0; j < 17; j++) {
    // Serial3.write(dataBytes[j]);
    //}
    delay(50);
  }
  }
*/

/*********************************************************************************************************
************************************** Steuere das Förderband ********************************************
*********************************************************************************************************/
void setBand(bool status)
{
  if (status)
  {
    motorEnabled = 1;
    gMotorCMD.isEnabled = motorEnabled;
    SetEMotor(&gMotorCMD, &gQueuedCmdIndex); // Bereite die Nachricht vor
    ProtocolProcess(selectRobot, &gMessage); // Schicke Nachricht ab
    // Nun läuft das Band
  }
  else if (!status)
  {
    motorEnabled = 0;
    gMotorCMD.isEnabled = motorEnabled;
    SetEMotor(&gMotorCMD, &gQueuedCmdIndex); // Bereite die Nachricht vor
    ProtocolProcess(selectRobot, &gMessage); // Schicke Nachricht ab
    // Nun steht das Band
  }
}

/*********************************************************************************************************
******************************* Fahre das Programm der SuS ab ********************************************
*********************************************************************************************************/

void runProgram()
{
  // Starte gespeichertes Programm
  // Fahre an eine Sichere Ausgangsposition
  moveArm(startPosX, startPosY, startPosZ, startPosR, false);
  delay(1500);
  // Arbeite alle Aufgaben ab
  for (int i = 0; i < anzahlAufgaben; i++)
  {
    if (aufgaben[i].angekommen)
    {
      // Wenn Nachricht angekommen ist
      // Unterscheide zwischen Koordinaten-Nachricht und Starte-Band-Nachricht mit .band == true
      if (aufgaben[i].band)
        // Band-Nachricht
      {
        // Starte Band und warte, bis Sensor auslöst
        // Annahme: Das Band ist hier immer gestoppt!
        setBand(true); // Nun läuft das Band
        while (sensorValue == 1)
        {
          // Checke ob SensorPin = 0, dann liegt etwas vor dem Sensor, also halte Band an
          sensorValue = digitalRead(SensorPin);
          delay(10);
        }
        // Etwas liegt vor dem Sensor, halte Band an und warte bis es Weggenommen wird
        setBand(false); // Nun steht das Band
        while (sensorValue == 0)
        {
          // Checke ob SensorPin = 0, dann liegt etwas vor dem Sensor, also halte Band an
          sensorValue = digitalRead(SensorPin);
          delay(10);
        }
        // Nun liegt nichts mehr vor dem Sensor, fahre mit Programm fort
      }
      else
        // Koordinaten-Nachricht
      {
        // Koordinaten anfahren. Erst hinfahren und dann Vac an oder aus!
        // Funktionsaufrauf: moveArm(float x, float y, float z, float r, bool vacuumOn)
        moveArm(aufgaben[i].x_coord, aufgaben[i].y_coord, aufgaben[i].z_coord, 0.0, aufgaben[i].vac);
        while (true)
        {
          // Frage Position alle 100ms ab und vergleiche, ob  == wert von aufgabe[i].x_coord
          // Sende zusätzlich die aktuellen Koordinaten an den Webserver auf dem ESP32
          GetPose();
          ProtocolProcess(0, &gMessage, true); // erwarte Antwort
          for (int i = 0; i < 16; i++)
          {
            // Speichere aktuelle Antwort in union aus char[16] und float[4]
            dfL.a_c[i] = gMessage.params[i];
          }
          // Sende die 16 Byte an den ESP32
          Serial3.write('C'); // Schicke C für Koordinaten
          Serial3.write('0'); // Schicke 0 für Links
          Serial3.write(dfL.a_c, 16);
          // Nachricht ist verarbeitet, nun zu float konvertieren
          currentX = dfL.f_a[0];
          currentY = dfL.f_a[1];
          currentZ = dfL.f_a[2];
          currentR = dfL.f_a[3];

          if (currentX > aufgaben[i].x_coord - 2.0 && currentX < aufgaben[i].x_coord + 2.0)
            // Warte bis Roboterarm an den Zielkoordinaten +- 2 ist und breche die Endlosschleife ab
          {
            delay(800); // Guter Wert um an die richtige Stelle zu kommen
            break;
          }
          delay(100);
        }
      }
    }
    else continue; // wenn nachricht nicht angekommen, dann weiter
    delay(50);
  }
  // Alle Nachrichten abgearbeitet
  startProgramm = false;
  fernsteuerung = true;
  moveArm(startPosX, startPosY, startPosZ, startPosR, false);
}

/*********************************************************************************************************
********************************* Arduino-loop() Dauerschleife *******************************************
*********************************************************************************************************/
void loop()
{
  // Checke, ob Nachricht vom ESP32 kommt mit Update über Modus, der auf Website gewählt wurde
  checkUARTESP32();
  // SPI-Kommunikation über Interrupts. Wenn "end of communication-Flag" "\r" empfangen wird, dann ist process = true
  if (process)
  {
    completedMessageReceived();
    spi_angekommen = true; // Setze Nachricht angekommen, um später, wenn von Seite angefordert, das Nachrichten-Array übermittelt wird
  }
  if (fernsteuerung || aufzeichnen)
  {
    // Frage die Positionen der Roboter ab
    getPoseToFloat(); // linker Roboter
    delay(100);
  }
  if (aufzeichnen && spi_angekommen)
  {
    // Sende Nachrichten nur, wenn neue Nachricht von Micro:Bit über SPI gekommen ist
    sendToESPSingle();
    spi_angekommen = false;
  }

  if (startProgramm)
  {
    runProgram();
  }
  delay(500);
}
