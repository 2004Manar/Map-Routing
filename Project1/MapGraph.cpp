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
        adjList[id] = vector<pair<int, double>>();  
    }
}

void MapGraph::addRoad(int sourceId, int destId, double length, double speed) {
    double time = (length / speed) * 60;  
    adjList[sourceId].emplace_back(destId, time);
    adjList[destId].emplace_back(sourceId, time);  
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





vector<int> MapGraph::findShortestPath(int startId, int endId) {
    unordered_map<int, double> dist;
    unordered_map<int, int> parent;
    unordered_map<int, bool> visited;

    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;

    for (const auto& node : nodes) {
        dist[node.first] = numeric_limits<double>::infinity();
        visited[node.first] = false;
    }

    dist[startId] = 0.0;
    pq.push({ 0.0, startId });

    while (!pq.empty()) {
        int current = pq.top().second;
        pq.pop();

        if (visited[current]) continue;
        visited[current] = true;

        if (current == endId) break;

        for (const auto& neighbor : adjList[current]) {
            int neighborId = neighbor.first;
            double weight = neighbor.second;

            if (dist[current] + weight < dist[neighborId]) {
                dist[neighborId] = dist[current] + weight;
                parent[neighborId] = current;
                pq.push({ dist[neighborId], neighborId });
            }
        }
    }

    vector<int> path;
    if (dist[endId] == numeric_limits<double>::infinity()) {
        return path;
    }

    for (int at = endId; at != startId; at = parent[at]) {
        path.push_back(at);
    }
    path.push_back(startId);
    reverse(path.begin(), path.end());
    return path;
}



