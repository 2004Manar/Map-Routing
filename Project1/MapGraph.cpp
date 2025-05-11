#include "MapGraph.h"
#include <unordered_map>
#include <vector>
#include <map>
#include <utility>
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <queue>
#include <limits>
#include <unordered_map>
#include <cmath>
#include <unordered_set>




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





vector<int> MapGraph::multiSourceDijkstra(
    const vector<int>& startNodes,
    const set<int>& endNodes,
    double sourceX, double sourceY,
    double destX, double destY) const
{
 
    unordered_map<int, double> dist;
    unordered_map<int, int>    prev;
   
    unordered_set<int> toSettle(endNodes.begin(), endNodes.end());
   
    priority_queue<Node, vector<Node>, greater<Node>> pq;

    
    for (int s : startNodes) {
        double walkIn = computeDistance({ sourceX, sourceY }, getNodeCoords(s))
            / 5.0 * 60.0;  
        dist[s] = walkIn;
        pq.push({ s, walkIn });
    }

    double bestTime = numeric_limits<double>::infinity();
    int    bestEnd = -1;

    
    while (!pq.empty() && !toSettle.empty()) {
        Node cur = pq.top(); pq.pop();
        int  u = cur.id;
        double t = cur.time;


        if (t > dist[u]) continue;

       
        if (toSettle.erase(u)) {
            double walkOut = computeDistance(getNodeCoords(u), { destX, destY })
                / 5.0 * 60.0;
            double totalT = t + walkOut;
            if (totalT < bestTime) {
                bestTime = totalT;
                bestEnd = u;
            }
           
        }

        
        auto it = adjList.find(u);
        if (it == adjList.end()) continue;
        for (auto& e : it->second) {
            int    v = e.first;
            double cost = e.second;
            double nt = t + cost;
            if (!dist.count(v) || nt < dist[v]) {
                dist[v] = nt;
                prev[v] = u;
                pq.push({ v, nt });
            }
        }
    }

    vector<int> bestPath;
    if (bestEnd >= 0) {
        for (int at = bestEnd; ; at = prev[at]) {
            bestPath.push_back(at);
            if (!prev.count(at)) break; 
        }
        reverse(bestPath.begin(), bestPath.end());
    }

    return bestPath;
}










vector<int> MapGraph::getPossibleStartingNodes(double Source_X, double Source_Y, double R)
{
    vector<int> possibleStartingNodesVector;

    double R_KM = R / 1000.0;

    double person_speed = 5;

    auto it = nodes.begin();

    while (it != nodes.end()) {
       
        double X_def = (Source_X - it->second.first);
        double Y_def = (Source_Y - it->second.second);
        double dest = sqrt(pow(X_def, 2) + pow(Y_def, 2)) * 1000; 

        
        if (dest <= R) {
            
            double distance_km = dest / 1000.0;
            double walking_time_h = distance_km / person_speed;
            possibleStartingNodes[it->first] = { walking_time_h, distance_km };
            possibleStartingNodesVector.push_back(it->first);

        }

        it++;
    }

    return possibleStartingNodesVector;
}




vector<int> MapGraph::getPossibleFinalingNodes(double Destination_X, double Destination_Y, double R)
{
    vector<int> possibleFinalNodesVector;
    double R_KM = R / 1000.0;

    double person_speed = 5;

    
    auto it = nodes.begin();

    while (it != nodes.end()) {
        double X_def = (Destination_X - it->second.first);
        double Y_def = (Destination_Y - it->second.second);

        double dest = sqrt(pow(X_def, 2) + pow(Y_def, 2)) * 1000; 


        if (dest <= R) {
           
            double distance_km = dest / 1000.0;
            double walking_time_h = distance_km / person_speed;
            possibleFinalNodes[it->first] = { walking_time_h, distance_km };
            possibleFinalNodesVector.push_back(it->first);
        }

        ++it;
    }

    return possibleFinalNodesVector;
}


double MapGraph::CalculateTotalWalkingDistance(double toStartNode, double toFinalNode)
{
    return toStartNode + toFinalNode;
}

double MapGraph::CalculateTotalWalkingTime(double toStartNode, double toFinalNode)
{
    return toStartNode + toFinalNode;
}



double MapGraph::computeDistance(const pair<double, double>& a, const pair<double, double>& b) {
    double dx = a.first - b.first;
    double dy = a.second - b.second;
    return sqrt(dx * dx + dy * dy);
}
double MapGraph::getEdgeTime(int from, int to) const {
    for (const auto& neighbor : adjList.at(from)) {
        if (neighbor.first == to) {
            return neighbor.second;
        }
    }
    return std::numeric_limits<double>::infinity();
}
pair<double, double> MapGraph::getNodeCoords(int id) const {
    return nodes.at(id);
}
MapGraph::RouteResult MapGraph::computeRouteDetails(
    const vector<int>& path,
    const MapGraph& graph,
    double sourceX, double sourceY,
    double destX, double destY)
{
    MapGraph::RouteResult result;
    result.path = path;
    result.totalTime = 0.0;
    result.totalDistance = 0.0;
    result.walkDistance = 0.0;
    result.vehicleDistance = 0.0;

    if (path.empty()) return result;

    double walkStart = MapGraph::computeDistance({ sourceX, sourceY }, graph.getNodeCoords(path.front()));
    result.walkDistance += walkStart;
    result.totalTime += (walkStart / 5.0) * 60.0; 

   
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        auto from = path[i];
        auto to = path[i + 1];


        double segmentDistance = MapGraph::computeDistance(graph.getNodeCoords(from), graph.getNodeCoords(to));
        result.vehicleDistance += segmentDistance;

       
        double time = graph.getEdgeTime(from, to);
        if (time == std::numeric_limits<double>::infinity()) {
            cerr << "Warning: No edge between nodes " << from << " and " << to << ". Skipping time for this segment." << endl;
            continue;
        }

        result.totalTime += time;
    }

    
    double walkEnd = MapGraph::computeDistance(graph.getNodeCoords(path.back()), { destX, destY });
    result.walkDistance += walkEnd;
    result.totalTime += (walkEnd / 5.0) * 60.0;

    
    result.totalDistance = result.walkDistance + result.vehicleDistance;

    return result;
}

void  MapGraph::writeResultToFile(const string& outputPath, const MapGraph::RouteResult& result) {
    ofstream out(outputPath, ios::app); 
    if (!out.is_open()) {
        cout << "Error writing to file." << endl;
        return;
    }

    out << "Path: ";
    for (int node : result.path) {
        out << node << " ";
    }
    out << endl;

    out << fixed << setprecision(2);
    out << "Total Time (minutes): " << result.totalTime << endl;
    out << "Total Distance (km): " << result.totalDistance << endl;
    out << "Walking Distance (km): " << result.walkDistance << endl;
    out << "Vehicle Distance (km): " << result.vehicleDistance << endl;
    out << "-------------------------------" << endl;

    out.close();
}