/*
  Programm zur Steuerung eines Roboters mit einem Arduino Mega für das Modul für Schüler*innen mit viel Vorwissen
  Das Programm kann in der void loop() Methode geschrieben werden und beliebig angepasst werden.
  Ein DOBOT Magician ist über eine UART-Verbindung angeschlossen.
  UART-Pins: RX1 (Pin 19) und TX1 (Pin 18)
*/
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

/*  Hier können eigene Datenstrukturen oder Variablen definiert werden  */

// Positionen für Roboterarm
float currentX = 200.00;
float currentY = 0.00;
float currentZ = 40.00;
float currentR = 0.00;
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

/*********************************************************************************************************
********************************************* Funktionen *************************************************
*********************************************************************************************************/

void initSerial()
{
  // Kommunikation mit Debug-Konsole über USB
  Serial.begin(115200);
  // Serial 1 für DOBOT Magician
  Serial1.begin(115200);
  // Setze Timer Interrupt, der alle 100ms den Serial1-Port abhört für Nachrichten vom Roboter
  FlexiTimer2::set(100, Serial1read);
  FlexiTimer2::start();
}

/*********************************************************************************************************
************* Setup-Methode die alle nötigen Einstellungen und Konfigurationen durchführt ****************
*********************************************************************************************************/

void setup()
{
  // Füllen das Aufgaben-Array mit Daten (nur zu Demonstrationszwecken)
  pinMode(SensorPin, INPUT); // Setze SensorPin als Eingang
  initSerial();
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
  // Fahre zur sicheren Startposition und schalte Pumpe aus
  moveArm(currentX, currentY, currentZ, currentR, false);
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

void getPosition()
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
  //  Nachricht ist verarbeitet, nun zu float konvertieren
  currentX = dfL.f_a[0];
  currentY = dfL.f_a[1];
  currentZ = dfL.f_a[2];
  currentR = dfL.f_a[3];
}
/*********************************************************************************************************
************************************ Steuere das Förderband **********************************************
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
********************************* Arduino-loop() Dauerschleife *******************************************
*********************************************************************************************************/
void loop()
{
  /*  Hier Code einfügen
      moveArm(X,Y,Z,R,vac); bewegt den Roboterarm und Steuert den Sauger
      getPosition(); speichert die aktuelle Position in currentX/Y/Z/R
      digitalRead(SensorPin); SensorPin ist Low-Aktiv und repräsentiert den Status des Anwesenheitssensor am Förderband
      setBand(bool); steuert das Förderband
  */
}
