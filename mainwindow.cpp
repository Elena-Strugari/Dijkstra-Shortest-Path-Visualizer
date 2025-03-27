#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QDebug>
#include <fstream>
#include <sstream>
#include <cmath>
#include <QFile>
#include <QXmlStreamReader>
#include <stdexcept>
#include <queue>
#include <QMouseEvent>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), scene(new QGraphicsScene(this)) {
    ui->setupUi(this);

    // Conectăm scena la QGraphicsView din UI
    ui->graphicsView->setScene(scene);
    scene->setSceneRect(0, 0, 1000, 1000);
    ui->graphicsView->setRenderHint(QPainter::Antialiasing);

    // Conectăm butonul de încărcare la funcția onLoadGraph
    connect(ui->loadGraphButton, &QPushButton::clicked, this, &MainWindow::onLoadGraph);
}

MainWindow::~MainWindow() {
    delete ui;
}

void Graph::addNode(int id, double x, double y) {
    nodes[id] = {id, x, y};
}

void Graph::addEdge(int from, int to) {
    double weight = calculateDistance(nodes[from], nodes[to]);
    edges.push_back({from, to, weight});
}

double Graph::calculateDistance(const Node &a, const Node &b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

void MainWindow::onLoadGraph() {
    QString filePath = QFileDialog::getOpenFileName(this, "Open Graph File", "", "XML Files (*.xml);;All Files (*)");

    if (filePath.isEmpty()) {
        QMessageBox::warning(this, "Warning", "No file selected!");
        return;
    }

    try {
        loadGraph(filePath.toStdString());
        drawEdges();
        QMessageBox::information(this, "Success", "Graph loaded and drawn successfully!");
    } catch (const std::exception &e) {
        QMessageBox::critical(this, "Error", e.what());
    }
}


void MainWindow::loadGraph(const std::string &filePath) {
    QFile file(QString::fromStdString(filePath));
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        throw std::runtime_error("Unable to open file: " + filePath);
    }

    QXmlStreamReader xmlReader(&file);
    int cnt = 0;

    while (!xmlReader.atEnd() && !xmlReader.hasError()) {
        xmlReader.readNext();

        if (xmlReader.isStartElement()) {
            if (xmlReader.name() == QLatin1String("node")) {
                // Citim atributele nodului
                QString id = xmlReader.attributes().value("id").toString();
                QString latitude = xmlReader.attributes().value("latitude").toString();
                QString longitude = xmlReader.attributes().value("longitude").toString();

                // if (cnt % 1000 == 0)
                //     qDebug() << "Node read: ID=" << id << ", Lat=" << latitude << ", Long=" << longitude;

                bool idOk, latOk, lonOk;
                int nodeId = id.toInt(&idOk);
                double lat = latitude.toDouble(&latOk);
                double lon = longitude.toDouble(&lonOk);

                if (!idOk || !latOk || !lonOk) {
                    qWarning() << "Eroare la conversia nodului:" << id << latitude << longitude;
                    continue;
                }

                // if (cnt % 1000 == 0)
                //     qDebug() << "Nod:" << nodeId << "Lat:" << lat << "Long:" << lon;

                // Adăugăm nodul în graf
                graph.addNode(nodeId, lon, lat);
            } else if (xmlReader.name() == QLatin1String("arc")) {
                // Citim atributele muchiei
                QString from = xmlReader.attributes().value("from").toString();
                QString to = xmlReader.attributes().value("to").toString();

                if (cnt % 1000 == 0)
                    qDebug() << "Edge read: From=" << from << ", To=" << to;

                bool fromOk, toOk;
                int fromId = from.toInt(&fromOk);
                int toId = to.toInt(&toOk);

                if (!fromOk || !toOk) {
                    qWarning() << "Eroare la conversia muchiei: From=" << from << ", To=" << to;
                    continue;
                }

                // Adăugăm muchia în graf
                graph.addEdge(fromId, toId);
            }
        }

        cnt++;
    }

    if (xmlReader.hasError()) {
        qWarning() << "Eroare la citirea XML:" << xmlReader.errorString();
    }

    file.close();
    qDebug() << "Graph loading complete.";
}

void MainWindow::drawEdges() {
    scene->clear();

    // Verificăm dacă există muchii
    if (graph.edges.empty()) {
        qWarning() << "No edges to draw!";
        return;
    }

    // Determinăm limitele coordonatelor
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double maxY = std::numeric_limits<double>::lowest();

    for (const auto &nodePair : graph.nodes) {
        minX = std::min(minX, nodePair.second.x);
        minY = std::min(minY, nodePair.second.y);
        maxX = std::max(maxX, nodePair.second.x);
        maxY = std::max(maxY, nodePair.second.y);
    }

    // Calculăm factorii de scalare
    double sceneWidth = 1000.0;
    double sceneHeight = 1000.0;
    double scaleX = sceneWidth / (maxX - minX);
    double scaleY = sceneHeight / (maxY - minY);

    qDebug() << "Scaling factors:" << scaleX << scaleY;

    // Desenăm muchiile
    for (const auto &edge : graph.edges) {
        auto &from = graph.nodes[edge.from];
        auto &to = graph.nodes[edge.to];

        double scaledX1 = (from.x - minX) * scaleX;
        double scaledY1 = (from.y - minY) * scaleY;
        double scaledX2 = (to.x - minX) * scaleX;
        double scaledY2 = (to.y - minY) * scaleY;

        scene->addLine(scaledX1, scaledY1, scaledX2, scaledY2, QPen(Qt::black));

        // qDebug() << "Edge from:" << edge.from << "to:" << edge.to
        //          << "Scaled coordinates: (" << scaledX1 << "," << scaledY1 << ") -> ("
        //          << scaledX2 << "," << scaledY2 << ")";
    }

    qDebug() << "Edges drawn successfully!";
}


void MainWindow::mousePressEvent(QMouseEvent *event) {
    QPointF clickedPoint = ui->graphicsView->mapToScene(event->pos());
    int closestNode = findClosestNode(clickedPoint);

    if (closestNode != -1) {
        selectedNodes.push_back(closestNode);
        qDebug() << "Node selected:" << closestNode;

        // Evidențiem nodurile selectate
        auto &node = graph.nodes[closestNode];
        scene->addEllipse(node.x * 10 - 5, node.y * 10 - 5, 10, 10, QPen(Qt::red), QBrush(Qt::red));

        // Actualizăm scena după desenarea nodurilor
        scene->update();

        // Dacă avem două noduri selectate, calculăm cel mai scurt drum
        if (selectedNodes.size() == 2) {
            int start = selectedNodes[0];
            int end = selectedNodes[1];
            auto path = dijkstra(start, end);
            if (!path.empty()) {
                qDebug() << "Shortest path from" << start << "to" << end << ":";
                for (int node : path) {
                    qDebug() << node;
                }
            } else {
                qDebug() << "No path found from" << start << "to" << end;
            }
            highlightPath(path);

            // Actualizăm scena după desenarea drumului
            scene->update();

            selectedNodes.clear();
        }
    }
}


int MainWindow::findClosestNode(const QPointF &point) {
    // Determinăm limitele coordonatelor pentru scalare
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double maxY = std::numeric_limits<double>::lowest();

    for (const auto &nodePair : graph.nodes) {
        const auto &node = nodePair.second;
        minX = std::min(minX, node.x);
        minY = std::min(minY, node.y);
        maxX = std::max(maxX, node.x);
        maxY = std::max(maxY, node.y);
    }

    // Calculăm factorii de scalare
    double sceneWidth = 1000.0; // Lățimea scenei
    double sceneHeight = 1000.0; // Înălțimea scenei
    double scaleX = sceneWidth / (maxX - minX);
    double scaleY = sceneHeight / (maxY - minY);

    double minDist = std::numeric_limits<double>::max(); // Valoare mare inițială
    int closestNode = -1; // Nod implicit invalid

    for (const auto &nodePair : graph.nodes) {
        const auto &node = nodePair.second;

        // Scalează coordonatele nodului pentru scena curentă
        double scaledX = (node.x - minX) * scaleX;
        double scaledY = (node.y - minY) * scaleY;

        // Calculează distanța pătrată (mai eficient decât sqrt pentru comparație)
        double dist = std::pow(scaledX - point.x(), 2) + std::pow(scaledY - point.y(), 2);

        // Dacă distanța este mai mică decât cea minimă, actualizează
        if (dist < minDist) {
            minDist = dist;
            closestNode = nodePair.first;
        }

        // // Debug pentru a verifica distanțele
        // qDebug() << "Checking node:" << nodePair.first
        //          << "Scaled coords: (" << scaledX << "," << scaledY << ")"
        //          << "Click coords: (" << point.x() << "," << point.y() << ")"
        //          << "Distance:" << dist;
    }

    qDebug() << "Closest node found:" << closestNode << "with distance:" << minDist;
    return closestNode;
}


std::vector<int> MainWindow::dijkstra(int start, int end) {
    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> prev;
    for (const auto &node : graph.nodes) {
        dist[node.first] = std::numeric_limits<double>::max();
        prev[node.first] = -1;
    }
    dist[start] = 0;

    using P = std::pair<double, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    pq.push({0, start});

    while (!pq.empty()) {
        auto [currentDist, u] = pq.top();
        pq.pop();

        if (u == end) break;

        for (const auto &edge : graph.edges) {
            if (edge.from == u) {
                int v = edge.to;
                double weight = edge.weight;
                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    prev[v] = u;
                    pq.push({dist[v], v});
                }
            }
        }
    }

    // Reconstruim drumul
    std::vector<int> path;
    for (int at = end; at != -1; at = prev[at])
        path.push_back(at);
    std::reverse(path.begin(), path.end());
    if (!path.empty()) {
        qDebug() << "Dijkstra path from" << start << "to" << end << ":";
        for (int node : path) {
            qDebug() << node;
        }
    } else {
        qDebug() << "No path found from" << start << "to" << end;
    }
    return path;
}
void MainWindow::highlightPath(const std::vector<int> &path) {
    if (path.empty()) {
        qDebug() << "No path found!";
        return;
    }

    for (size_t i = 0; i < path.size() - 1; ++i) {
        int from = path[i];
        int to = path[i + 1];
        auto &fromNode = graph.nodes[from];
        auto &toNode = graph.nodes[to];
        QPen pathPen(Qt::green);
        pathPen.setWidth(10);
        scene->addLine(fromNode.x * 10, fromNode.y * 10, toNode.x * 10, toNode.y * 10, pathPen);
    }
    scene->update();


    qDebug() << "Path highlighted!";
}

