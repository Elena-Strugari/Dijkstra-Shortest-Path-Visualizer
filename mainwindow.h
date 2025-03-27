#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <unordered_map>
#include <vector>

namespace Ui {
class MainWindow;
}

struct Node {
    int id;
    double x, y;
};

struct Edge {
    int from, to;
    double weight;
};

class Graph {
public:
    std::unordered_map<int, Node> nodes;
    std::vector<Edge> edges;

    void addNode(int id, double x, double y);
    void addEdge(int from, int to);

private:
    double calculateDistance(const Node &a, const Node &b);
};

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void mousePressEvent(QMouseEvent *event) override;

private slots:
    void onLoadGraph();

private:
    Ui::MainWindow *ui;
    QGraphicsScene *scene;
    Graph graph;

    std::vector<int> selectedNodes;

    void loadGraph(const std::string &filePath);
    void drawEdges();
    int findClosestNode(const QPointF &point);
    std::vector<int> dijkstra(int start, int end);
    void highlightPath(const std::vector<int> &path);
};

#endif // MAINWINDOW_H
