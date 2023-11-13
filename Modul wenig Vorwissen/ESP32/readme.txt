Zum Hochladen der Daten für die Darstellung der Webseite im "data"-Ordner auf den Flash-Speicher des ESP32 muss die Erweiterung "ESP32 Filesystem Uploader" in der Arduino-Programmierumgebung instaliert werden.
Die Erweiterung ist nur mit der Arduino 1 Programmierumgebung kompatibel.
Eine Anleitung ist unter folgendem Link zu finden:
https://randomnerdtutorials.com/install-esp32-filesystem-uploader-arduino-ide/#installing

Damit das Programm auf den ESP32 übertragen werden kann, muss in den "Voreinstellungen" der Arduino-IDE die Boardinformation bereitgestellt werden.
Unter "Zusätzliche Boardverwalter-URLs" muss der Link "https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json" eingefügt werden.
Danach kann im "Board-Manager" die Board-Information installiert werden.
Eine Anleitung ist unter https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/ zu finden.
