#include "MapGraph.h"
#include <iostream>

int main() {
    
    MapGraph graph = MapGraph::constructGraph("C:\\Users\\VIP.DESKTOP-F6DB1GS\\Desktop\\ALGO project\\MapRoutingProject\\Project1\\Project1\\map1.txt");


    
    graph.printNodes();
    graph.printAdjList();

    vector<int> shortestPath = graph.findShortestPath(1, 3);

    cout << "Shortest path from 1 to 3: ";
    for (int node : shortestPath) {
        cout << node << " ";
    }
    cout << endl;


    return 0;
}
