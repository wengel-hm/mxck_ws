# Dokumentation ACC & Integration Lidar/Radar

Projektmodul
Projektleiter: Professor Krug, William Engel

Abgabe: 23.07.2024

Christopher Bautz
FAB7
christopher.bautz@hm.edu



# 1. Einleitung


In diesem Projekt wird ein System entwickelt, das die Daten eines Lidarsensors mit Kamerabildern kombiniert, um die Erkennung und Verfolgung von Objekten in Echtzeit zu verbessern. Diese Fusion verschiedener Sensordaten ist besonders für autonome Fahrzeuge nützlich, da sie eine robustere und genauere Erkennung der Umgebung ermöglicht. Dies wird durch die Stärken der einzelnen Sensoren erreicht: Lidar liefert präzise Abstandsmessungen, während Kameras detaillierte visuelle Informationen liefern.

Das gesamte System wird in Python unter Verwendung des Robot Operating System (ROS) implementiert. ROS bietet eine flexible Kommunikationsinfrastruktur und ist in der Robotik weit verbreitet. Es ermöglicht eine effiziente Verteilung von Sensordaten und die Implementierung komplexer Algorithmen zur Sensorfusion und Entscheidungsfindung. In diesem Projekt wurden ROS-Knoten entwickelt, die unter anderem Lidardaten verarbeiten, Kamerabilder empfangen, Sensorfusion durchführen und die Ergebnisse zur weiteren Verarbeitung und Analyse bereitstellen.

Die Integration dieser Sensoren erfolgt durch eine Kombination von ROS-Nachrichten, Transformationsrahmen und Bildverarbeitungsalgorithmen. Besonderes Augenmerk wird auf die Echtzeitfähigkeit des Systems gelegt, um eine kontinuierliche und zuverlässige Objekterkennung und -verfolgung zu gewährleisten. Die Anwendung von Kalibrierungsmethoden und die kontinuierliche Überprüfung der Systemleistung stellen darüber hinaus eine hohe Genauigkeit der fusionierten Sensordaten sicher.

Diese Dokumentation beschreibt die Implementierung des Systems, die verwendeten Algorithmen zur Sensorfusion und die durchgeführten Tests zur Validierung der Ergebnisse. Ziel ist die Entwicklung eines zuverlässigen und effizienten Objekterkennungssystems für autonome Modellfahrzeuge.


# 2. ACC / Integration Lidar / Fusion Lidar mit Kamera

Dieses Kapitel beschreibt die verschiedenen Aspekte der Implementierung der ACC-Funktionen, die Integration des Lidars sowie die Fusion des Lidars mit der Kamera des autonomen Modellfahrzeugs. Die einzelnen Komponenten des Systems werden detailliert beschrieben. Die Dokumentation folgt strukturell dem Code, beschreibt aber nicht die genaue Implementierung in Python bzw. ROS, für die eine ausführliche Dokumentation direkt im Code durch Kommentare gewährleistet ist.

## 2.1 Initialisierungen

Zu Beginn der Initialisierung werden einige wesentliche Einstellungen vorgenommen, die den Betrieb des Systems beeinflussen. Eine wichtige Einstellung ist use_cluster_code_without_detection. Mit diesem Flag wird der Standalone-Betrieb ohne die integrierte Objekterkennung aktiviert. Eine ähnliche Einstellung ist use_boundry_code_without_detection, die es ermöglicht, das System ohne die Mittellinienerkennung zu betreiben. Es ist zu beachten, dass die Seitenlinienerkennung zum Zeitpunkt der Projektabgabe noch nicht implementiert ist.

Das System verwendet eine Reihe von ROS Subscribern und Publishern, um Sensordaten zu empfangen und verarbeitete Daten zu veröffentlichen. Die Subscriber sind dafür verantwortlich, Daten von verschiedenen Sensoren wie Lidar, Kameras und anderen Quellen zu empfangen. Beispielsweise empfängt der Lidar-Subscriber (self.lidar_sub) die Lidar-Scans, während der Kamera-Subscriber (self.camera_sub) die Kamerabilder empfängt. Andere Subscriber sammeln Daten wie die Geschwindigkeit des Fahrzeugs und die erkannten Objekte.

Auf der anderen Seite veröffentlichen Publisher die aufbereiteten Daten, die von anderen Komponenten des Systems oder zur Visualisierung benötigt werden. Beispiele hierfür sind der distance_pub, der den berechneten Abstand zum vorausfahrenden Fahrzeug publiziert, und der image_pub, der die projizierten Lidar-Bilder ausgibt. Diese Kommunikationsstruktur ermöglicht eine effiziente Verteilung und Nutzung der Sensordaten im gesamten System.

Zur Speicherung und Verarbeitung der Sensordaten werden verschiedene Datenplatzhalter und Standardwerte definiert. Diese Platzhalter dienen der Zwischenspeicherung von Informationen, wie z.B. die erkannten Objekte, die Koordinaten der Mittellinie und die Streckenbegrenzungen. Standardwerte wie die Eigengeschwindigkeit des Fahrzeugs oder die Kameramatrix werden initialisiert, um eine Basis für weitere Berechnungen zu schaffen.

Callback-Funktionen sind zentrale Bestandteile des ROS-Systems. Sie werden automatisch aufgerufen, sobald neue Daten von den Sensoren eintreffen. In diesem Projekt gibt es mehrere Callback-Funktionen, die jeweils bestimmte Aufgaben übernehmen. Beispielsweise verarbeitet die lidar_callback-Funktion die empfangenen Lidar-Daten, während die camera_callback-Funktion die Kamerabilder in ein verarbeitbares Format umwandelt. Diese Funktionen gewährleisten eine kontinuierliche und zeitnahe Verarbeitung der Sensordaten.

Ein weiterer wichtiger Aspekt der Initialisierung ist die Bereitstellung von Kamerainformationen. Dies geschieht über eine spezielle Callback-Funktion, die periodisch aufgerufen wird, um die aktuellen Kameradaten zu publizieren. Diese Informationen sind für die Kalibrierung und die korrekte Projektion der Lidarpunkte auf die Bildebene der Kamera unerlässlich. Wird eine neue Kamera verwendet, müssen die Kameradaten entsprechend angepasst werden, um eine genaue Synchronisation und Kalibrierung zu gewährleisten.

Insgesamt bilden die beschriebenen Initialisierungen und Einstellungen die Grundlage für die Funktionalität des Systems und ermöglichen eine effektive und effiziente Verarbeitung der Sensordaten.

## 2.2 Lidar-Punkt-Projektion

Die Lidar-Punktprojektion ist eine zentrale Komponente des Systems, die es ermöglicht, die von den Lidar-Sensoren erfassten Daten auf die Bildebene der Kamera zu projizieren bzw. Lidar und Kamera zu fusionieren.

Ein wichtiger Aspekt der Lidarpunktprojektion ist die Skalierung. Da die Entfernung der Objekte zum Lidar-Sensor variiert, wird eine dynamische Skalierungsfunktion verwendet, um die Punkte korrekt auf die Kamerabildebene zu projizieren. Die Skalierungsfunktion wird durch eine Kalibrierung bestimmt, bei der bekannte Entfernungen und beobachtete Offsets verwendet werden, um die Parameter der Funktion zu berechnen. Diese Funktion sorgt dafür, dass die Punkte proportional zur Entfernung skaliert werden, was eine realistische und genaue Darstellung der Lidardaten im Kamerabild ermöglicht. Wie die konkreten Werte für diese Skalierung erzeugt wurden wird im Kapitel zur Kalibrierung genauer erklärt.

Nach der Skalierung erfolgt die Projektion der Lidarpunkte auf die Bildebene der Kamera. Hierbei wird die Kameramatrix verwendet, um die dreidimensionalen Lidarkoordinaten in zweidimensionale Pixelkoordinaten zu transformieren. Dieser Schritt ist entscheidend, da er die räumliche Information der Lidar-Daten in das Koordinatensystem der Kamera überträgt. Die Kameramatrix, die durch Kalibrierdaten definiert wird, spielt dabei eine zentrale Rolle, da sie die interne Geometrie und die optischen Eigenschaften der Kamera berücksichtigt.

Ein weiterer wichtiger Schritt ist die Berücksichtigung des Offsets zwischen den Sensoren. In unserem Fall befinden sich der Lidar-Sensor und die Kamera nicht am selben Punkt des Fahrzeugs, was zu Koordinatenabweichungen führt. Um diese Abweichungen zu korrigieren, wird ein Offset verwendet, der die Position des Lidar-Sensors relativ zur Kamera berücksichtigt. Dieser Offset wird bei der Berechnung der Punktkoordinaten entsprechend berücksichtigt, so dass die projizierten Punkte exakt den tatsächlichen Positionen der Objekte entsprechen.

Die gesamte Implementierung der Lidar-Punktprojektion erfolgt in mehreren Funktionen. Die Funktion project_to_image_plane führt die eigentliche Projektion der Lidarpunkte auf die Kamerabildebene unter Berücksichtigung der Skalierungsfaktoren durch. Die Funktion transform_point schließlich transformiert die Lidar-Koordinaten in das Kamerakoordinatensystem.

Zusammen ermöglichen diese Schritte eine präzise und zuverlässige Projektion der Lidar-Daten auf die Bildebene der Kamera, was die Grundlage für die weitere Verarbeitung und Fusion der Sensordaten bildet.


## 2.3 Mittellinienerkennung & Streckenbegrenzung


Die Mittellinien- und Streckenbegrenzungserkennung ist ein wesentlicher Bestandteil des Systems, um die von den Lidarsensoren erfassten Punkte zu analysieren und sicherzustellen, dass sie sich innerhalb der Fahrspur befinden. Diese Funktion verwendet eine erkannte Mittellinie und leitet daraus die seitlichen Begrenzungslinien der Fahrspur ab. Dies ist wichtig, um sicherzustellen, dass nur relevante Punkte für die Objekterkennung und andere Funktionen des autonomen Fahrzeugs berücksichtigt werden und störende Objekte außerhalb der Fahrspur ausgeblendet werden.

Die Hauptaufgabe der Mittellinienerkennung besteht darin, die Koordinaten der Mittellinie (x_ref und y_ref) zu verarbeiten und daraus die seitlichen Begrenzungen der Fahrspur zu bestimmen. Die Funktion update_boundaries übernimmt diese Aufgabe, indem sie die Mittellinienpunkte extrapoliert und die linken und rechten Begrenzungslinien berechnet. Diese Begrenzungslinien werden durch eine feste Verschiebung der Mittellinienpunkte um einen bestimmten Abstand nach links und rechts erzeugt.

Die Mittellinie selbst wird ständig aktualisiert und ihre Punkte werden verwendet, um die seitlichen Begrenzungen zu bestimmen. Dazu werden die Punkte der Mittellinie nach links und rechts verschoben, um die Grenzen der Fahrspur zu markieren. Diese Linien dienen als Referenz, um zu überprüfen, ob die von den Lidar-Sensoren erfassten Punkte innerhalb der Fahrspur liegen. Dies ist wichtig, um sicherzustellen, dass nur relevante Objekte erkannt und verfolgt werden, während störende Objekte außerhalb der Fahrspur ignoriert werden.

Ein großer Teil des Codes in diesem Abschnitt dient der Visualisierung der Mittellinie und der seitlichen Begrenzungslinien. Die Punkte dieser Linien werden als Pfadnachrichten veröffentlicht, die dann von anderen Systemkomponenten zur Visualisierung und Weiterverarbeitung verwendet werden können. Die kontinuierliche Veröffentlichung dieser Linien ermöglicht eine Echtzeitdarstellung der Fahrspur, die für die Navigation und Entscheidungsfindung des Fahrzeugs unerlässlich ist.

Ein wichtiger Bestandteil dieses Prozesses ist die Funktion detect_object_within_boundaries, die überprüft, ob ein von den Lidar-Sensoren erfasster Punkt innerhalb der definierten Fahrspurgrenzen liegt. Diese Funktion erhält als Eingabe den Abstand und den Winkel des detektierten Objektes und berechnet dessen x- und y-Koordinaten. Anschließend wird geprüft, ob diese Koordinaten innerhalb der linken und rechten Begrenzungslinie liegen. Ist dies der Fall, wird der Punkt als innerhalb des Fahrwegs liegend betrachtet und weiterverarbeitet, andernfalls wird er ignoriert.

Zusammenfassend lässt sich sagen, dass die Mittellinien- und Fahrspurbegrenzungserkennung eine effektive Filterung der von den Lidarsensoren erfassten Punkte ermöglicht, indem sichergestellt wird, dass nur Punkte innerhalb der Fahrspur berücksichtigt werden. Dies trägt zur Genauigkeit und Zuverlässigkeit der Objekterkennung und anderer Funktionen des autonomen Fahrzeugs bei. In Zukunft könnte diese Funktion durch die direkte Erkennung von Seitenlinien weiter verbessert werden, um eine noch präzisere Spurführung zu ermöglichen.


## 2.4 Geschwindigkeitsberechnung

Die Geschwindigkeitsberechnung ist ein zentraler Bestandteil der adaptiven Geschwindigkeitsregelung (ACC). Sie ermöglicht es, die Geschwindigkeit des vorausfahrenden Fahrzeugs zu bestimmen und diese Information an das Steuergerät weiterzuleiten, das die Geschwindigkeit des autonomen Fahrzeugs entsprechend anpasst.

Die Berechnung der Geschwindigkeit des vorausfahrenden Fahrzeugs erfolgt durch die Auswertung von Lidardaten. Die Funktion calculate_front_vehicle_speed gibt den aktuellen Abstand zum vorausfahrenden Fahrzeug ein und berechnet die Relativgeschwindigkeit aus der zeitlichen Änderung dieses Abstands. Dazu wird die Differenz zwischen dem aktuellen und dem vorherigen Abstand sowie die entsprechende Zeitdifferenz verwendet. Die Relativgeschwindigkeit gibt an, wie schnell sich das vorausfahrende Fahrzeug relativ zum eigenen Fahrzeug bewegt.

Ein wichtiger Aspekt der Geschwindigkeitsberechnung ist die Anwendung eines PT1-Filters (Proportional-Integral-Filter). Dieser Filter wird verwendet, um die berechnete Geschwindigkeit zu glätten und kurzzeitige Schwankungen zu reduzieren. Dies ist wichtig, um stabile und zuverlässige Geschwindigkeitswerte zu erhalten, die dem ACC-Controller als Eingabe dienen. Der PT1-Filter berechnet die gefilterte Geschwindigkeit als gewichtete Summe der aktuellen und der vorhergehenden Geschwindigkeit, wobei die Gewichtung durch eine Filterkonstante bestimmt wird. Dieser Ansatz hilft, plötzliche Änderungen in der Geschwindigkeitsmessung zu dämpfen und eine gleichmäßigere Regelung zu ermöglichen.

Die gefilterte Geschwindigkeit wird schließlich an das ACC-Steuergerät übermittelt, das die Geschwindigkeit des autonomen Fahrzeugs entsprechend regelt. Durch den Einsatz des PT1-Filters wird sichergestellt, dass das Steuergerät mit stabilen und zuverlässigen Geschwindigkeitswerten arbeitet, was zu einer gleichmäßigen und komfortablen Fahrweise führt. Die kontinuierliche Berechnung und Filterung der Geschwindigkeit ist daher ein wesentlicher Bestandteil des ACC-Systems, das die Sicherheit und Effizienz des autonomen Fahrzeugs gewährleistet.


## 2.5 Kalibrierung

Die Kalibrierung ist ein wesentlicher Prozess bei der Entwicklung eines autonomen Fahrzeugsystems, der sicherstellt, dass die Sensordaten korrekt und genau sind. Eine genaue Kalibrierung ist entscheidend, um zuverlässige Messungen zu gewährleisten und die Integrität der Datenfusion zwischen verschiedenen Sensoren wie Lidar und Kamera zu erhalten.

Es ist zu beachten, dass die Kalibrierdaten standardmäßig deaktiviert sind, aber durch den Parameter "calibrate_fusion_on" am Anfang des Codes aktiviert werden können, wenn eine neue Kalibrierung erforderlich ist.

Der Kalibrierungsprozess beginnt mit dem Sammeln von Daten aus bekannten Entfernungen. Dazu werden Lidar-Daten verwendet, um die tatsächlichen Positionen von Objekten zu messen. Diese bekannten Entfernungen werden dann mit den beobachteten Offsets verglichen, die sich aus der Projektion der Lidarpunkte auf die Kamerabildebene ergeben. Die Unterschiede zwischen den bekannten Entfernungen und den gemessenen Offsets werden verwendet, um eine Skalierungsfunktion zu kalibrieren, die die Projektionen korrigiert.


![Kalibrierung 1](https://raw.githubusercontent.com/TheFightAtom/Projektarbeit_ACC/master/Pictures/Kalibrierung_Fusion_Bild1.png)
![Kalibrierung 2](https://raw.githubusercontent.com/TheFightAtom/Projektarbeit_ACC/master/Pictures/Kalibrierung_Fusion_Bild2.png)

Ein wichtiger Bestandteil der Kalibrierung ist die Funktion collect_calibration_data. Diese Funktion sammelt kontinuierlich Kalibrierdaten während des Fahrzeugbetriebs. Sie überwacht die Stabilität der Messungen und stellt sicher, dass die Daten innerhalb bestimmter Toleranzgrenzen liegen. Wenn stabile Messungen vorliegen, werden diese zur Kalibrierung verwendet, um die Genauigkeit der Skalierungsfunktion weiter zu verbessern.

Die Kalibrierungsdaten werden in einer CSV-Datei gespeichert, um eine einfache Überprüfung und Anpassung zu ermöglichen. Die Funktion save_calibration_data speichert die gesammelten Daten, einschließlich der bekannten Entfernungen und der beobachteten Offsets. Diese Daten können später verwendet werden, um die Kalibrierungsparameter zu überprüfen oder anzupassen.

Insgesamt ermöglicht die Kalibrierung eine präzise und zuverlässige Nutzung der Sensordaten, indem sie sicherstellt, dass die Messungen korrekt und konsistent sind. Dies bildet die Grundlage für die Datenfusion und weitere Verarbeitungsschritte im System und trägt wesentlich zur Genauigkeit und Effizienz des autonomen Fahrzeugs bei.


## 2.6 Hauptlogik / Ablauf

Die Hauptlogik des Systems wird durch die Funktion process_data realisiert, die kontinuierlich aufgerufen wird, um die Sensordaten zu verarbeiten und die notwendigen Entscheidungen für die autonome Fahrzeugsteuerung zu treffen. Diese Funktion spielt eine zentrale Rolle, da sie die verschiedenen Datenquellen integriert, die relevanten Informationen extrahiert und die Steuerbefehle generiert.

Die Funktion process_data prüft zunächst, ob aktuelle Lidardaten vorliegen. Wenn keine neuen Lidar-Daten empfangen wurden, wird die Verarbeitung abgebrochen und gewartet, bis neue Daten eintreffen. Diese Überprüfung ist wichtig, um sicherzustellen, dass das System immer mit den aktuellsten Informationen arbeitet.

Die Hauptlogik der Datenverarbeitung erfolgt in zwei Varianten, abhängig von der Konfiguration des Systems: Clustering ohne Objekterkennung und erweitertes Clustering mit Objekterkennung.

In der ersten Variante, bei der use_cluster_code_without_detection aktiviert ist, werden die Lidardaten direkt zur Bildung von Clustern verarbeitet. Die Clusterbildung erfolgt durch Gruppierung von Punkten, die räumlich nahe beieinander liegen. Diese Punkte werden anhand ihrer x- und y-Position im Raum gruppiert, wobei ein bestimmter Abstand nicht überschritten werden darf, um als zusammenhängendes Cluster zu gelten.

Nach der Clusterbildung wird das beste Cluster ausgewählt. Dies ist das Cluster, das am nächsten zur Mitte der Fahrspur liegt. Die Auswahl erfolgt durch Berechnung des durchschnittlichen Abstands der Punkte im Cluster zur Mittellinie. Das beste Cluster wird dann verwendet, um den aktuellen Abstand zum vorausfahrenden Fahrzeug und dessen Geschwindigkeit zu berechnen. Diese Werte werden dann gefiltert und angezeigt.

![Erkennen irgendeines Objekts in der Mitte](https://raw.githubusercontent.com/TheFightAtom/Projektarbeit_ACC/master/Pictures/Beispielbild_Erkennung2.jpg)
In der zweiten Variante, bei der die Objekterkennung integriert ist, werden die Lidardaten mit den erkannten Objekten der Kamera kombiniert. Jedes erkannte Objekt wird auf mögliche zugehörige Lidarpunkte überprüft, um die genaue Position und Entfernung zu bestimmen. Diese Datenfusion ermöglicht eine genauere Erkennung und Verfolgung der Objekte. Auch hier wird das beste Cluster aus den erkannten Fahrzeugen ausgewählt und weiterverarbeitet.

![Erkennen eines Fahrzeugs](https://raw.githubusercontent.com/TheFightAtom/Projektarbeit_ACC/master/Pictures/Beispielbild_Erkennung1.jpg)

Ein wichtiger Schritt in beiden Varianten ist die Transformation der Lidarpunkte in das Kamerakoordinatensystem, die mit der zuvor beschriebenen Funktion transform_and_project_lidar_points durchgeführt wird und die korrekte Zuordnung der Punkte sicherstellt.

Ein entscheidender Teil der Hauptlogik ist die Behandlung von Fällen, in denen kein Objekt erkannt wird. In solchen Fällen wird die Entfernung auf einen Standardwert von 999 Metern und die Geschwindigkeit des vorausfahrenden Fahrzeugs auf 0.0 m/s gesetzt. Diese Werte sind für die Steuerung wichtig, da sie signalisieren, dass kein Fahrzeug in unmittelbarer Nähe erkannt wurde. Der Regler kann dann geeignete Maßnahmen ergreifen, z. B. die Geschwindigkeit anpassen oder die aktuelle Geschwindigkeit beibehalten.

Die Funktion publish_best_cluster_as_laserscan dient zur Visualisierung des besten erkannten Clusters. Sie konvertiert das ausgewählte Cluster in ein LaserScan-Nachrichtenformat, das dann über einen ROS-Publisher veröffentlicht wird. Diese Visualisierung ermöglicht die Überwachung und Überprüfung der Position und Struktur des besten Clusters in der Umgebung des Fahrzeugs. Diese Visualisierung ist besonders hilfreich bei der Entwicklung und Fehlersuche, da sie ein visuelles Feedback über die erkannten Objekte und ihre relativen Positionen bietet.

Zusätzlich wird bei der Variante mit Objekterkennung das empfangene Multiarray mit Hilfe der Funktion publish_multiarray_with_distance um die berechnete Geschwindigkeit der detektierten Objekte erweitert. 

Zusammenfassend beschreibt die Funktion process_data die zentrale Logik der Datenverarbeitung im autonomen Fahrzeugsystem. Sie integriert verschiedene Datenquellen, führt Clustering und Selektion durch, transformiert die Daten und stellt sicher, dass die Steuerbefehle auf Basis der aktuellsten und genauesten Informationen generiert werden. Diese Funktion ist das Herzstück der autonomen Fahrzeugsteuerung und gewährleistet eine zuverlässige und effiziente Navigation.


# 3. Radar Test

Im Rahmen dieses Projektes wurde das Radar der Firma RadarIQ getestet und auf seine Eignung für unsere Anwendung geprüft. Das Radar bietet grundsätzlich vielversprechende Möglichkeiten zur Objekterkennung und -verfolgung. Allerdings traten bei der Integration in unser System erhebliche Probleme auf, die insbesondere auf die mangelnde Unterstützung durch den Hersteller zurückzuführen sind.

Das größte Problem, das während der Tests auftrat, war das Ende des Supports für alle Software und Informationen durch den Hersteller. Die offizielle Windows Standalone Software ist nicht mehr verfügbar und auch die Informationen zur Integration des Radars in das Robot Operating System (ROS) fehlen vollständig. Dies stellte uns vor die Herausforderung, das Radar ohne Unterstützung der vorgesehenen Tools in Betrieb zu nehmen.

Da die herstellereigene Software nicht mehr verfügbar war, blieb nur die Möglichkeit, das Radar über die Python-Schnittstelle zu nutzen. Diese bietet grundlegende Funktionen zur Steuerung und Auswertung der Radardaten. Trotz dieser Einschränkungen konnten wir das Radar erfolgreich testen und die gewonnenen Daten visualisieren. Dazu wurde ein 3D-Plot erstellt, der die vom Radar erfassten Punkte darstellt. Diese Visualisierung half uns, die Funktionsweise und die Erkennungsgenauigkeit des Radars besser zu verstehen.

![Radar Plot](https://raw.githubusercontent.com/TheFightAtom/Projektarbeit_ACC/master/Pictures/Radar_Plot.png)

Die Testergebnisse zeigten, dass das Radar in der Lage ist, Objekte in seiner Umgebung zu erkennen. Die fehlende Unterstützung durch den Hersteller wird jedoch die zukünftige Integration des Radars in unser System erheblich erschweren. Insbesondere die fehlende ROS-Integration stellt ein großes Hindernis dar, da wir dadurch keine nahtlose Kommunikation zwischen den verschiedenen Sensoren und Komponenten unseres Projektes gewährleisten können.

Zusammenfassend lässt sich sagen, dass das Radar zwar prinzipiell für unsere Anwendung geeignet wäre, aber die fehlende Unterstützung und die damit verbundenen technischen Herausforderungen eine vollständige Integration in unser System problematisch machen. Zukünftige Arbeiten sollten daher entweder auf eine Lösung zur Integration des Radars in ROS abzielen oder alternative Radarsysteme mit besserer Unterstützung und Dokumentation in Betracht ziehen.


# 4. Fazit und Ausblick

In diesem Projekt wurde erfolgreich demonstriert, wie die Lidar-Technologie effektiv bei der Entwicklung autonomer Fahrzeuge eingesetzt werden kann. Lidar spielte eine zentrale Rolle bei der Erfassung und Verarbeitung von Umgebungsdaten, die für verschiedene Aspekte der Fahrzeugnavigation und -steuerung von entscheidender Bedeutung sind.

Durch die Integration von Lidar-Daten konnten Abstand und Geschwindigkeit des vorausfahrenden Fahrzeugs genau bestimmt werden. Dies ist für die adaptive Geschwindigkeitsregelung (ACC) unerlässlich, um ein sicheres und effizientes Fahren zu gewährleisten. Darüber hinaus wurden die Lidar-Daten verwendet, um den Abstand von Objekten, die durch die Objekterkennung identifiziert wurden, genau zu bestimmen, was die Genauigkeit und Zuverlässigkeit des Systems weiter erhöht. Dies alles unter Berücksichtigung der Streckenbeschränkungen.

Durch die Fusion von Lidar-Daten mit Kamerabildern konnte eine robustere und genauere Umgebungserkennung erreicht werden. Dies wurde durch den Einsatz von ROS und verschiedenen Bildverarbeitungsalgorithmen ermöglicht, die eine Echtzeitverarbeitung der Sensordaten sicherstellten. Darüber hinaus wurde das Lidar kalibriert, um die Messgenauigkeit zu gewährleisten und zuverlässige Daten für die Entscheidungsfindung zu liefern.

Der entwickelte Code bietet eine solide Grundlage für die weitere Optimierung und Erweiterung des Systems. Ein Bereich, der noch verbessert werden kann, ist die Linienerkennung. Sobald eine verbesserte Linienerkennung implementiert ist, können weitere Schritte unternommen werden, um die erkannten Linien als seitliche Begrenzungen für das autonome Fahren zu nutzen. Diese Erweiterung würde die Navigation des Fahrzeugs in komplexeren Umgebungen weiter verbessern.

Insgesamt zeigt dieses Projekt, dass die Lidar-Technologie ein wesentlicher Bestandteil autonomer Fahrzeugsysteme ist. Durch die kontinuierliche Weiterentwicklung und Optimierung des Codes sowie die Integration zusätzlicher Sensoren und Algorithmen kann die Leistungsfähigkeit und Sicherheit autonomer Fahrzeuge weiter gesteigert werden.
