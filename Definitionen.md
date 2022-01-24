# Definitionen zum Datenaustausch 
NFC-THMS-Sensor-Tag Arduino-PC-Bridge
Die Daten werden über die serielle Schnittstelle übertragen (COM-Port, 115200 Baud, 8 Databit, 1 Stopbit, No Parity). 
Der Arduino startet automatisch mit dem Auslesen des aufliegenden NFC-THMS-Sensor-Tags.
Die ausgelesene NDEF-Textnachricht wird direkt über die serielle Schnittstelle ausgegeben.
> z.B. "Do:01;No:1;SS:123;MS:456;RSQPB:1203;"   

Zwischendurch werden Informationsstrings (hilfreich zum Debuggen) ausgegeben sofern diese im Programm aktiviert wurden.
Informationsstrings beginnen immer mit ">>>".

Durch eingaben über die serielle Schnittstelle kann der Arduino gesteuert werden.
Jede Eingabe sollte mit einem '\n' (Zeilenende) enden.
Folgende Eingaben sind möglich:


Eingabe| Definition
-------------- | --------
S | Sensor suchen (Start/Stop).			
M | Einzelne Messung triggern (Es wird "Do:02" an den Tag gesendet).	
I | Senden einer bestimmten Do-Instruction an den Tag    (Z.B. "I:04" für einen Reset oder "I:06" um den Tag Konfigurationsdaten ausgeben zu lassen. Diese müssen nochmal gesondert ausgelesen werden.)
R | (Noch nicht implementiert) Auslesen der aktuellen NDEF-Nachricht auf dem NFC-TMS-Sensor-Tag.
W | (Noch nicht implementiert) Schreiben einer NDEF-Nachricht auf den Sensor-Tag.
C | Kontinuierliche Messung aktivierten/deaktivieren.		
T | Intervallzeit einstellen für die kontinuierliche Messung (Z.B. "T:120" für alle 120 Sekunden).
X | (Noch nicht implementiert) Zurücksetzen und neu starten.
