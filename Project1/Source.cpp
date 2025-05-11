#include "MapGraph.h"
#include <iostream>
#include <chrono>
#include <fstream>
#include <vector>
#include <set>

int main() {

    string mapFile = "SFMap.txt";
    string queryFile = "C:\\Users\\VIP.DESKTOP-F6DB1GS\\Desktop\\ALGO project\\MapRoutingProject\\Project1\\Project1\\SFQueries.txt";
    string outputFile = "C:\\Users\\VIP.DESKTOP-F6DB1GS\\Desktop\\ALGO project\\large.txt";

    MapGraph graph = graph.constructGraph(mapFile);

    ifstream queryIn(queryFile);
    if (!queryIn.is_open()) {
        cout << "Error opening query file: " << queryFile << endl;
        return 1;
    }

    ofstream clearOut(outputFile); 
    clearOut.close();

    int Q;
    queryIn >> Q;

    auto totalStart = chrono::high_resolution_clock::now();

    for (int q = 0; q < Q; ++q) {
        double srcX, srcY, dstX, dstY, R;
        queryIn >> srcX >> srcY >> dstX >> dstY >> R;

        auto queryStart = chrono::high_resolution_clock::now();

     
        vector<int> startNodes = graph.getPossibleStartingNodes(srcX, srcY, R);
        vector<int> endNodesVec = graph.getPossibleFinalingNodes(dstX, dstY, R);
        set<int> endNodes(endNodesVec.begin(), endNodesVec.end());

        if (startNodes.empty() || endNodes.empty()) {
            cout << "No valid start or end nodes for query " << q + 1 << endl;
            continue;
        }
        //cout << "Start candidates: ";
        //for (int id : startNodes) cout << id << " ";
        //cout << "\nEnd candidates: ";
        //for (int id : endNodesVec) cout << id << " ";
        //cout << endl;

        
        vector<int> bestPath = graph.multiSourceDijkstra(startNodes, endNodes, srcX, srcY, dstX, dstY);

        if (bestPath.empty()) {
            cout << "No path found for query " << q + 1 << endl;
            continue;
        }
        cout << "--------------------" << endl;
        cout << "Best path for query " << q << ": ";

        for (int node : bestPath) {
            cout << node << " ";
        }
        cout << endl;

        
        MapGraph::RouteResult result = MapGraph::computeRouteDetails(bestPath, graph, srcX, srcY, dstX, dstY);

      
        MapGraph::writeResultToFile(outputFile, result);

        auto queryEnd = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(queryEnd - queryStart).count();
        cout << "Query " << q + 1 << " executed in " << duration << " ms" << endl;
    }

    auto totalEnd = chrono::high_resolution_clock::now();
    auto totalDuration = chrono::duration_cast<chrono::milliseconds>(totalEnd - totalStart).count();
    cout << "All queries executed in " << totalDuration << " ms" << endl;


    return 0;
}