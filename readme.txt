Hier ist ein Spannungsmess-Testprogramm (VoltageCheck) sowie der ganze restliche Code (noch in Entwicklung) vom Messknoten (SensorNode) und einem Beispiel-Controller (RF_Receiver).


###### Sensor Node ######

X	Sensoren auslesen (Implementiert und getestet. Geht sehr gut)
X	Protokoll
		implementiert und getestet:
			Beim Startup beim Controller registrieren 
			ID beziehen, falls noch keine vorhanden
			Daten übertragen
			Schlafdauer empfangen und schlafen gehen
			Speichern der Daten im EEPROM bei RF Ausfall und auslesen der Daten, wenn vom Server angefordert
X	RTC synchronisieren
-	Interrupt Alarm von RTC

** Known problems: **

	If RTC is unplugged, sketch will hang due to a bug in the I2C library.


###### Controller  ######

Ist ein simpler Controller, der zeigt wie man die Funktionen von dem Protokoll verwendet.
Features:
	ID-verwaltung:
		-) Hat der Node noch keine ID wird eine vom Controller vergeben.
		-) Der Controller führt eine Liste mit den ihm bekannten Nodes.
	Kommunikationsprotokoll:
		-) Der Controller antwortet auf Registrations und Messdaten-Nachrichten.
		   Dabei schickt er Instruktionen mit.
	Data Logging:
		-) Alle Nachrichten werden auf die SD Karte geloggt.
		   Den Teil kann man dann auf dem Pie übernehmen und ein Datenbankinterface basteln

Fehlt:
	Die Möglichkeit über ein Front-End die Messintervalle eizustellen und Messknoten mit Pumpknoten zu verknüpfen.
	Ein Algorithmus wann welche Pumpen aktiviert werden und ggf. ein Frontend für die Parameter des Algorithmus





###### PumpNode ######

Fehlt völlig