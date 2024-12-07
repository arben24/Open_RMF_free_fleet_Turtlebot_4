# Integration eines Turtlebot 4 in Open-RMF mit dem Free Fleet Adapter

Dieses Repository zeigt, wie ein Turtlebot 4 mithilfe des [Free Fleet Adapters](https://github.com/open-rmf/free_fleet) in [Open-RMF](https://www.open-rmf.org) eingebunden werden kann. Ziel ist es, die praktische Anwendung in einer realistischen Umgebung zu demonstrieren. Dafür wurde ein kostengünstiger Demonstrator entwickelt, der Tests von Robotern, Türen und Aufzügen ermöglicht.

---

## Demo Video

[![Demonstration](https://img.youtube.com/vi/EfvdhYBY6P0/0.jpg)](https://www.youtube.com/watch?v=EfvdhYBY6P0)

---

## Überblick

1. [Aufbau des Demonstrators](#1-aufbau-des-demonstrators)
2. [Programmierung der Tür- und Aufzugssteuerung](#2-programmierung-der-tür-und-aufzugssteuerung)
3. [Einrichten von Open-RMF und des Free Fleet Adapters](#3-einrichten-von-open-rmf-und-des-free_fleet_adapters)
4. [Ergebnis Demonstration](#4-ergebnis-demonstration)

---

## 1. Aufbau des Demonstrators

Der Demonstrator ist modular aufgebaut, um Flexibilität und einfache Anpassung zu gewährleisten.

### Materialien und Konstruktion:
- **Grundmaterial**: Spanplatten (Maße: 1690 x 634 x 12 mm, andere Maße möglich).
- **Verbindungen**: 3D-gedruckte Verbinder sorgen für eine schnelle und flexible Montage.
- **Türen**: 
  - Ausschnitte aus den Spanplatten mit günstigen Scharnieren befestigt.
  - Bewegung durch Standard-RC-Servos, die in 3D-gedruckten Gehäusen montiert sind.
  - Tür und Servo sind über ein "Servo Horn" und ein 3D-gedrucktes „U“-Element verbunden, um Fehlstellungen zu tolerieren.

---

## 2. Programmierung der Tür- und Aufzugssteuerung

Zur Steuerung der Türen und des Aufzugs wird ein **ESP32** verwendet. Als **Entwicklungsumgebung** wird hier [Visual Studio Code](https://code.visualstudio.com) mit der Erweiterung [PlatformIO](https://platformio.org) empfohlen.

Hierfür muss dann noch [Micro-ROS](https://micro.ros.org) eingerichtet werden. Somit kann der ESP32 dann direkt in die die Kommunikation des ROS-Netzwerks eingreifen und den Status der Türen und Aufzüge lesen und schreiben.
Genutzte Topics sind hierbei: `/door_states` und `/door_requests`.

Beispielcode ist [hier](/Platformio/) zu finden.

---

## 3. Einrichten von Open-RMF und des Free_Fleet_Adapters

### Erstellung der Karte für Open-RMF

Ein Grundriss im PNG-Format muss vorliegen, welcher später der Darstellung in RViz dient. 

![Beispiel Grundriss](/images/map_L1.png)

Bevor die Einrichtung beginnen kann muss der Roboter eine Karte seiner Umgebung aufzeichnen. Ähnlich wie [HIER](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html). Dabei entstehen zwei Dateien: `.yaml`-Datei (mit Metadaten) und  `.pgm`-Datei (visuelle Darstellung der Karte)

![Beispiel Karte](/images/lidar_map.png)

#### Schritte zur Kartenerstellung mit dem Traffic Editor:

Eine detailliertere Anleitung findet sich [HIER](https://docs.google.com/presentation/d/1Lt79xlM_XkITmURSbI5hkAAgnjSX8dHKBkgvz3x3Uzw/edit?pli=1#slide=id.g117b0289c78_0_0l)

1. **Traffic Editor starten**: `traffic-editor` im gesourcten Open-RMF-Workspace.
2. **Projekt und Etagen erstellen**: Namen, Höhe und Grundriss definieren.
3. **Karte skalieren und Elemente hinzufügen**:
   - Wände, Boden, Türen und Aufzüge zeichnen.
4. **Roboterpfade definieren**: Knoten und Kanten einfügen.
5. **LiDAR-Scan anpassen**:
   - Für die spätere Koordinatentransformation benötigt.

Speichern Sie die `.building.yaml`-Datei im Verzeichnis `rmf_demos_maps`.

---

### Einrichtung des Free Fleet Adapters für Turtlebot 4

Der Turtlebot 4 wird über den Free Fleet Adapter in Open-RMF integriert. Hierfür wurde eine extra ROS-Workspace angelegt basierend auf der Anleitung des [Free Fleet](https://github.com/open-rmf/free_fleet/tree/main) Adapters. Folgende Änderungen wurden gemacht:

#### Server-Seite:
1. Launch-Datei ([Beispiel](/server.launch)) im Verzeichnis `ff_examples_ros2/launch` erstellen
2. Konfigurieren Sie:
   - Namen.
   - Koordinatentransformation-Parameter wie `translation_x`, `translation_y`, `rotation`, `scale` (aus dem Traffic Editor).

#### Client-Seite:
1. Launch-Datei ([Beispiel](/turtlebot4_world_ff.launch.xml)) im Verzeichnis `ff_examples_ros2/launch` erstellen
2. Verwenden Sie die aufgezeichnete Karte (`.yaml` und `.pgm`).
3. Passen Sie den Pfad in der Launch-Datei an.
4. Bei Darstellungsproblemen: `origin`-Parameter in der `.yaml` auf `0` setzen.

---

### Einrichtung des Flottenadapters für Open-RMF

Damit Open-RMF den Roboter erkennt:
1.  eine Launch-Datei ([Beispiel](/fleet_adapter.launch.xml)) für den Flottenadapter erstellen (basierend auf [LINK](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/launch/fleet_adapter.launch.xml)).
2. im Verzeichnis `rmf_demos_fleet_adapter/launch` ablegen
3. Konfiguration an die Roboterparameter anpassen

---

## Start des Systems

### Schritte:
1. **Builden und Sourcen**:
   -  `colcon build` in jedem Workspace ausführen und nicht vergessen zu sourcen.
2. **Systeme starten**:
   1. Free Fleet Server:  
      `ros2 launch ff_examples_ros2 server.launch`
   2. Free Fleet Client:  
      `ros2 launch ff_examples_ros2 turtlebot4_world_ff.launch.xml`
   3. Open-RMF:  
      `ros2 launch rmf_demos fleet_adapter.launch.xml`

Nach Schritt 3 sollte sich RViz öffnen und die erstellte Karte anzeigen.

---

## Mögliche Probleme und Lösungen

| Problem                                   | Lösung                                                                 |
|-------------------------------------------|------------------------------------------------------------------------|
| Roboter wird an falscher Stelle angezeigt | `origin`-Parameter in der `.yaml`-Datei prüfen.                       |
|                                           | Falsche Koordinatentransformation in der Server-Launch-Datei.          |
|                                           | Neu bauen und sourcen.                                                |
| Flottenadapter von Open-RMF nicht erkannt | Überprüfen, ob Free Fleet Client und Server korrekt laufen.            |
|                                           | Flottenadapter-Launch-Datei auf Fehler überprüfen.                     |

## To-Do-Liste  
- [ ] README verbessern  
- [ ] Beispiele vervollständigen  
- [ ] Weitere Lösungen für Probleme hinzufügen  

