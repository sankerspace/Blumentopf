Hier ist ein Spannungsmess-Testprogramm (VoltageCheck) sowie der ganze restliche 
Code (noch in Entwicklung) vom Messknoten (SensorNode) und einem 
Beispiel-Controller (RF_Receiver).
###### Einrichtung Arduino IDE #########
Im Ordner libraries sind s�mtliche Bibliotheken die wir d�r das Projekt 
ben�tigen In der Arduino IDE muss der Standard Ordner f�r die installierten 
Libraries ge�ndert werden auf die Location von der Blumentopf Repository, damit 
sind automatisch s�mtlichen notwendigen Libraries f�r unser Projekt aufgenommen.


###### Sensor Node ######

X	Sensoren auslesen (Implementiert und getestet. Geht sehr gut)
X	Protokoll
		implementiert und getestet:
			Beim Startup beim Controller registrieren 
			ID beziehen, falls noch keine vorhanden
			Daten �bertragen
			Schlafdauer empfangen und schlafen gehen
			Speichern der Daten im EEPROM bei RF Ausfall und auslesen der Daten, 
			wenn vom Server angefordert
X	RTC synchronisieren
-	Interrupt Alarm von RTC

** Known problems: **

	If RTC is unplugged, sketch will hang due to a bug in the I2C library.


###### Controller  ######

Ist ein simpler Controller, der zeigt wie man die Funktionen von dem Protokoll 
verwendet.
Features:
	ID-verwaltung:
		-) Hat der Node noch keine ID wird eine vom Controller vergeben.
		-) Der Controller f�hrt eine Liste mit den ihm bekannten Nodes.
	Kommunikationsprotokoll:
		-) Der Controller antwortet auf Registrations und Messdaten-Nachrichten.
		   Dabei schickt er Instruktionen mit.


Fehlt:
	Die M�glichkeit �ber ein Front-End die Messintervalle eizustellen und 
	Messknoten mit Pumpknoten zu verkn�pfen.	Ein Algorithmus wann welche Pumpen 
	aktiviert werden und ggf. ein Frontend f�r die Parameter des Algorithmus

###### Raspberry Pi  ######

    fehlt noch!!!

	Data Logging:
		-) Alle Nachrichten werden passiv geloggt.
		   Den Teil kann man dann auf dem Pie �bernehmen und ein Datenbankinterface 
		   basteln

###### PumpNode ######

Hardware fertig

Software:
    Pumpe ansteuerbar
    RF Kommunikation

fehlt noch:
    Interrupt/Sleep vom Pump-Node
    Protokoll Implementierung
