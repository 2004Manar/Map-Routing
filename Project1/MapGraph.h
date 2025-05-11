#pragma once
#include <unordered_map>
#include <vector>
#include <map>
#include <utility> 
#include <string>
#include <set>
using namespace std;
class MapGraph
{

private:
	unordered_map<int, pair<double, double>> nodes;
	unordered_map<int, vector<pair<int, double>>> adjList;
	unordered_map<int, pair<double, double>> possibleFinalNodes;
	unordered_map<int, pair<double, double>> possibleStartingNodes;
	bool hasEdge(int u, int v) const;
public:
	void addNode(int id, double x, double y);
	void addRoad(int sourceId, int destId, double length, double speed);
	static MapGraph constructGraph(const string& mapFilePath);

	struct WalkInfo {
		int nodeID;
		double distance;
		double time;
	};

	struct RouteResult {
		vector<int> path;
		double totalTime;
		double totalDistance;
		double walkDistance;
		double vehicleDistance;
	};

	struct Node {
		int id;
		double time;
		bool operator>(const Node& other) const {
			return time > other.time;
		}
	};

	vector<int> multiSourceDijkstra(const vector<int>& startNodes, const set<int>& endNodes,
		double sourceX, double sourceY, double destX, double destY) const;


	pair<double, double> getNodeCoords(int id) const;
	static RouteResult computeRouteDetails(const vector<int>& path, const MapGraph& graph,
		double sourceX, double sourceY,
		double destX, double destY);
	double getEdgeTime(int from, int to) const;
	static	void writeResultToFile(const string& outputPath, const MapGraph::RouteResult& result);
	static double computeDistance(const pair<double, double>& a, const pair<double, double>& b);


	void printNodes() const;
	void printAdjList() const;


	vector<int> getPossibleFinalingNodes(double Destination_X, double Destination_Y, double R);
	vector<int> getPossibleStartingNodes(double Source_X, double Source_Y, double R);

	double CalculateTotalWalkingDistance(double toStartNode, double toFinalNode);
	double CalculateTotalWalkingTime(double toStartNode, double toFinalNode);

};