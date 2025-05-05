#pragma once
#include <unordered_map>
#include <vector>
#include <utility> // std::pair
#include <string>
using namespace std;
class MapGraph
{
private:
	unordered_map<int, pair<double, double>> nodes;
	unordered_map<int, vector<pair<int, double>>> adjList;
public:
	void addNode(int id, double x, double y);
	void addRoad(int sourceId, int destId, double length, double speed);
	static MapGraph constructGraph(const string& mapFilePath);


	vector<int> findShortestPath(int startId, int endId) const;

	void printNodes() const;
	void printAdjList() const;


};

