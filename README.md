# Dijkstra Shortest Path Visualizer – Qt C++ Application

This project is a **Qt-based GUI application** that loads a graph from an XML file, renders it visually, and allows the user to select two nodes to compute the **shortest path** between them using **Dijkstra's algorithm**.

## Features

- Load directed graphs from structured `.xml` files (nodes with coordinates, arcs as edges)
- Interactive node selection by mouse click
- Calculates shortest path with Dijkstra and highlights it in green
- Auto-scales node positions for proper visualization
- Uses `QGraphicsScene` for dynamic and scalable rendering

## Technologies Used

- **C++**
- **Qt 6** (Widgets, Graphics View Framework)
- XML parsing with `QXmlStreamReader`
- Dijkstra's algorithm (priority queue-based)
- File I/O and scene scaling logic

## Main Components

- `mainwindow.cpp/.h`: Main application logic, GUI events, file loading, and Dijkstra algorithm
- `main.cpp`: Entry point for the Qt application
- `mainwindow.ui`: UI layout designed with Qt Designer
- `CMakeLists.txt`: Build configuration

## XML File Format (Example)

```xml
<graph>
  <node id="0" latitude="45.0" longitude="25.0"/>
  <node id="1" latitude="46.0" longitude="26.0"/>
  <arc from="0" to="1"/>
</graph>
