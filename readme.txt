Hier ist ein Spannungsmess-Testprogramm (VoltageCheck) sowie der ganze restliche Code (noch in Entwicklung) vom Messknoten (SensorNode) und einem Beispiel-Controller (RF_Receiver).


###### Sensor Node ######

X	Sensoren auslesen (Implementiert und getestet. Geht sehr gut)
~	Protokoll
		implementiert und getestet:
			Beim Startup beim Controller registrieren 
			ID beziehen, falls noch keine vorhanden
			Daten übertragen
			Schlafdauer empfangen und schlafen gehen
		Todo:
			Speichern der Daten im EEPROM bei RF Ausfall ist noch nicht implementiert
X	RTC synchronisieren
-	Interrupt Alarm von RTC

** Known problems: **

	If RTC is unplugged, sketch will hang due to a bug in the I2C library.


###### Controller  ######

Ist ein simpler Controller, der zeigt wie man die Funktionen von dem Protokoll verwendet.
Es fehlt:
	ID-verwaltung.
	Ein Konzept wann die Sensoren ausgelesen und wann die Pumpen wie gesteuert werden.
	Data logging Konzept




###### PumpNode ######

Fehlt völlig