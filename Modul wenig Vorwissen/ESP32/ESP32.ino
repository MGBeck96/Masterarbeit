#include "WiFi.h"
#include "ESPAsyncWebSrv.h"
#include "ArduinoJson.h"
#include "SPIFFS.h"
#include <map>
#include <string>

#define RX2 16
#define TX2 17
#define LOCAL_NETWORK
// #define DEBUG_PRINT
char buff[50]; // 20 Gruppen-Nachrichten brauchen 340 Byte
char message[50];
volatile boolean message_processed = false;
static char *rob[3] = {"200.00", "00.00", "040.00"};
volatile byte indx;
volatile boolean process;
typedef union
{
  unsigned char a_c[16];
  float f_a[4];
} dobot_to_float;
dobot_to_float dfL;

/*
  Folgende Datenstruktur ist 1:1 identisch mit der auf dem Arduino Mega,
  um eine einfache Übertragung zu ermöglichen.
*/
// Anzahl der Gruppen
const int anzahlAufgaben = 20;
// Array zum Checken, ob alle Gruppen-Nachrichten angekommen sind
bool angekommen_check[anzahlAufgaben];
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
// Array aus dem Struct - für 5 unterschiedliche Gruppen
AufgabenDatenSingle aufgaben[anzahlAufgaben];

#ifdef LOCAL_NETWORK
const char *ssid = "aFRITZ!Box 7530";         // SSID des WiFi-Netzwerks
const char *password = "77736ImBuchenfeld16"; // Passwort des WiFi-Netzwerks
#endif
#ifndef LOCAL_NETWORK
const char *ssid = "DOBOTControllNetwork"; // SSID des WiFi-Netzwerks
const char *password = "DOBOTNetwork";     // Passwort des WiFi-Netzwerks
#endif
// Flag für die Ausgangszustand
bool reset = false;
// Flag für die Programmaufzeichnung
bool aufzeichnen = false;
// Flag für Abfahren des Programms
bool startProg = false;
// AsyncWebServer Objekt an Port 80
AsyncWebServer server(80);



/*********************************************************************************************************
*************** Verbinde mit dem WLAN nach gewähltem Modus und gebe die IP-Adresse aus *******************
*********************************************************************************************************/
void setup_wifi()
{
  // Funktion, die die Verbindung mit dem Wifi herstellt.
  Serial.print("Verbinde zu Netzwerk: ");
  Serial.println(ssid);
  // WiFi Verbindung mit Router.
  WiFi.begin(ssid, password);
  // Warte, bis Verbindung hergestellt und gebe so lange alle 0,5 Sekunden Punkte aus.
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi verbunden");
  Serial.println("IP Addresse: ");
  Serial.println(WiFi.localIP()); // WiFI.localIP() gibt die IP-Adresse des ESP8266 im Netzwerk an.
}

/*********************************************************************************************************
********************* Verarbeiten der Schüler*innen-Nachrichten vom Arduino Mega *************************
*********************************************************************************************************/

void message_group_handler(char *message)
{
  // Nachricht-Format: ['C'][byte Aufgabennummer][AufgabenDaten]
  char msg[16];                       // Nachricht = len(message) - 2 Identifyer Byte
  unsigned char aufgabe = message[1]; // atoi() hört nach dem ersten nicht numerischen Zeichen auf
  Serial.println(aufgabe);
  for (int i = 0; i < 16; i++)
  {
    // df.a_c[i] = gMessage.params[15 - i]; for big endian, not needed
    msg[i] = message[2 + i];
  }
  // händisch einfügen
  // .angekommen und .band sind bool
  aufgaben[aufgabe].angekommen = message[2];
  aufgaben[aufgabe].band = message[3];
  // Koordinaten sind float, also 4 Byte
  memcpy(&aufgaben[aufgabe].x_coord, &message[4], 4);
  memcpy(&aufgaben[aufgabe].y_coord, &message[8], 4);
  memcpy(&aufgaben[aufgabe].z_coord, &message[12], 4);
  aufgaben[aufgabe].vac = message[16];
#ifdef DEBUG_PRINT
  for (int i = 0; i < 18; i++)
  {
    Serial.print("0x");
    if (message[i] < 0x10)
    {
      // Füge eine führende Null hinzu, wenn das Byte-Wert kleiner als 0x10 ist
      Serial.print("0");
    }
    Serial.print(message[i], HEX);
    Serial.print(" ");
  }
  Serial.println(); // Neue Zeile am Ende
  Serial.println(aufgaben[2].x_coord);
#endif
}

/*********************************************************************************************************
****************** Verarbeiten der Roboter-Koordinaten-Nachrichten vom Arduino Mega **********************
*********************************************************************************************************/

void message_coord_handler(char *message)
{
  /*
    Format der Nachricht: ['C'][0][4 Mal 4 Bytes mit den Koordinaten]
    Die 0 an der 2. Stelle ist eine Vorbereitung für die Unterscheidung zwischen zwei Robotern
  */
  if (message[1] == '0')
  {
#ifdef DEBUG_PRINT
    for (int i = 0; i < 19; i++)
    {
      Serial.print("0x");
      if (message[i] < 0x10)
      {
        // Füge eine führende Null hinzu, wenn das Byte-Wert kleiner als 0x10 ist
        Serial.print("0");
      }
      Serial.print(message[i], HEX);
      Serial.print(" ");
    }
    Serial.println(); // Neue Zeile am Ende
#endif
    // Speichere den ankommenden Byte-String in dem struct zur Konvertierung in float
    for (int i = 0; i < 16; i++)
    {
      dfL.a_c[i] = message[2 + i];
    }
    // Konvertiere die ankommenden 4-Byte-float in String mit 2 Nachkommastellen
    sprintf(rob[0], "%.2f", dfL.f_a[0]); // x
    sprintf(rob[1], "%.2f", dfL.f_a[1]); // y
    sprintf(rob[2], "%.2f", dfL.f_a[2]); // z
    // r-Koordinate wird hier Ignoriert, da nicht für Programm relevant
  }
  return;
}

/*********************************************************************************************************
************************ Server Setup für XML- und HTML-Anfragen vom Client ******************************
*********************************************************************************************************/

void setup_server()
{
  // Funktion, die den Server initalisiert.
  // Pfad für root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    request->send(SPIFFS, "/index.html");
  });
  // Pfad zur style.css-Datei
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    request->send(SPIFFS, "/style.css", "text/css");
  });
  // Pfad für  coords.html web page
  server.on("/coords.html", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    request->send(SPIFFS, "/coords.html");
    aufzeichnen = false;
  });
  // Pfad für  record.html web page
  server.on("/record.html", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    aufzeichnen = true;
    startProg = false;
    reset = false;
    request->send(SPIFFS, "/record.html");
  });
  //*******************************************************************************************
  // Anfragen aus der record.html
  // Hier wird ein JSON Objekt aus einem char-Array erstellt und zur Website geschickt
  server.on("/nachrichten", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    // Erstellen eines JSON-Dokuments
    const size_t capacity = 3000; // Ausreichend Speicher - 2000 Byte reichen für 16,5 Zeilen
    DynamicJsonDocument doc(capacity);
    JsonArray jsonArray = doc.to<JsonArray>();
    // Füge jedes AufgabenDaten-Element zum JSON-Array hinzu
    for (int i = 0; i < anzahlAufgaben; i++) {
      JsonObject jsonData = jsonArray.createNestedObject();
      jsonData["nachricht"] = i;
      jsonData["angekommen"] = aufgaben[i].angekommen;
      // 2 Roboter jsonData["roboter"] = aufgaben[i].roboter;
      jsonData["band"] = aufgaben[i].band;
      jsonData["x_coord"] = aufgaben[i].x_coord;
      jsonData["y_coord"] = aufgaben[i].y_coord;
      jsonData["z_coord"] = aufgaben[i].z_coord;
      jsonData["vac"] = aufgaben[i].vac;
    }
    // JSON-Dokument als Antwort senden
    String jsonStr;
    serializeJson(doc, jsonStr);
    request->send(200, "application/json", jsonStr);
  });

  server.on("/startProg", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    startProg = true;
    reset = false;
    aufzeichnen = false;
  });
  //*******************************************************************************************
  // Anfragen aus der coords.html
  // Verarbeiten der "xhttp.open("GET", "/xy", true);" XML/Javascript-Befehle aus der HTML-Datei.
  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    reset = true;
    aufzeichnen = false;
    startProg = false;
  });
  server.on("/home", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    Serial2.write('H'); // Schicke request für Home Command an den Mega
    delay(1000);
  });
  server.on("/demo", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    Serial2.write('D'); // Schicke request für Home Command an den Mega
    delay(100);
  });

  // Handler für die Abfrage der Koordinaten
  /*
    Einzelne Abfragen haben sich als deutlich schneller erwiesen im Vergleich zum Senden
    aller Daten in einem Objekt.
  */
  server.on("/x-coord-l", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    request->send_P(200, "text/plain", String(rob[0]).c_str());
  });
  server.on("/y-coord-l", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    request->send_P(200, "text/plain", String(rob[1]).c_str());
  });
  server.on("/z-coord-l", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    request->send_P(200, "text/plain", String(rob[2]).c_str());
  });
  // Starte Server
  server.begin();
}

void setup()
{
  // setup-Funktion, die Teil eines Arduino-Sketch ist. Wird einmalig ausgeführt.
  Serial.begin(115200);                        // Baud-Rate einstellen
  Serial2.begin(115200, SERIAL_8N1, RX2, TX2); // Serial Port 2 an Pin 16 RX und Pin 17 TX initialisieren
  setup_wifi();                                // Die WiFi-Setup Funktion aufrufen
  // Initialize SPIFFS
  if (!SPIFFS.begin(true))
  {
    Serial.println("Fehler bei der Bereitstellung des SPIFFS-Dateisystems");
    return;
  }
  setup_server(); // Server initialisieren.
  // Sende Signal für Neustart an Arduino Mega
  Serial2.write('V');
  delay(200);

  for (int i = 0; i < 3; i++)
  {
    // Platz für die Zeichenkette erstellen (einschließlich Null-Terminierungszeichen)
    rob[i] = new char[7];
  }
}

void loop()
{
  if (reset)
  {
    // Sende Signal für Neustart an Arduino Mega. Koordinaten des Roboters werden Übermittelt
    Serial2.write('V');
    delay(200);
    reset = false;
  }
  if (aufzeichnen)
  {
    /* Sende Signal für Nachrichten der SuS an Arduino Mega
      Nun werden Nachrichten, die vom Micro:Bit an den Arduino Mega geschickt werden
      weitergeleitet an den ESP32
    */
    Serial2.write('R');
    delay(200);
    aufzeichnen = false;
  }
  if (startProg)
  {
    // Nun wird das übermittelte Programm gestartet
    Serial2.write('S');
    delay(200);
    startProg = false;
  }
  // Prüfe ob Nachricht vom Arduino Mega empfangen wurde
  if (Serial2.available() >= 16)
  {
    // Ein Byte-Array zum Speichern der empfangenen Daten erstellen
    // char receivedData[19];
    int len = Serial2.available();

    // Die 16 Bytes empfangen und im Byte-Array speichern
    for (int i = 0; i < len; i++)
    {
      buff[i] = Serial2.read();
    }
    if (strcmp(message, buff))
    {
      // strcpy macht ganz komische sachen und wohl bei bool = false schluss
      // strcpy(message, buff); // kopiert den volatilen char array "buff" in den char array "message"
      memcpy(message, buff, sizeof(buff));
      // Serial.println(message);
      if (message[0] == 'C')
        message_coord_handler(message);
      // if (message[0] == 'G' && len == 18) message_group_handler(message); für 2 rob
      if (message[0] == 'G' && len == 17)
        message_group_handler(message);
    }
    for (int i = 0; i < 50; i++)
    {
      buff[i] = 0;
      message[i] = 0;
    }
  }
}
