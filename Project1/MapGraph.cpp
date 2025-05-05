#include "MapGraph.h"
#include <unordered_map>
#include <vector>
#include <utility>
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <queue>
#include <limits>
#include <unordered_map>
using namespace std;

void MapGraph::addNode(int id, double x, double y) {
    if (nodes.find(id) == nodes.end()) {
        nodes[id] = { x, y };
        adjList[id] = vector<pair<int, double>>();  // Ensure the node has an empty adjacency list
    }
}

void MapGraph::addRoad(int sourceId, int destId, double length, double speed) {
    double time = (length / speed) * 60;  // time in minutes
    adjList[sourceId].emplace_back(destId, time);
    adjList[destId].emplace_back(sourceId, time);  // assuming bidirectional roads
}

MapGraph MapGraph::constructGraph(const string& mapFilePath) {
    MapGraph graph;
    ifstream file(mapFilePath);

    if (!file.is_open()) {
        cout << "Error opening file: " << mapFilePath << endl;
        return graph;
    }
    int n;
    file >> n;
    int id;
    double x, y;
    for (int i = 0; i < n; i++) {
        file >> id >> x >> y;
        graph.addNode(id, x, y);
    }
    int m;
    file >> m;
    int sourceId, destId;
    double length, speed;
    for (int i = 0; i < m; i++) {
        file >> sourceId >> destId >> length >> speed;
        graph.addRoad(sourceId, destId, length, speed);
    }
    file.close();
    return graph;
}

void MapGraph::printNodes() const {
    cout << "nodes = {" << endl;
    for (const auto& node : nodes) {
        cout << "    " << node.first << ": ("
            << fixed << setprecision(2) << node.second.first << ", "
            << fixed << setprecision(2) << node.second.second << ")," << endl;
    }
    cout << "}" << endl;
}

void MapGraph::printAdjList() const {
    cout << "adjList = {" << endl;
    for (const auto& entry : adjList) {
        cout << "    " << entry.first << ": [";
        for (size_t i = 0; i < entry.second.size(); ++i) {
            cout << "(" << entry.second[i].first << ", "
                << fixed << setprecision(2) << entry.second[i].second << ")";
            if (i != entry.second.size() - 1) {
                cout << ", ";
            }
        }
        cout << "]," << endl;
    }
    cout << "}" << endl;
}








