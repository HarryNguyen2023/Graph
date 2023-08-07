#include <list>
#include <stack>
#include <vector>

// Initiate the class of graph
#define edge std::pair<int, int>

#ifndef _GRAPH_
#define _GRAPH_

class Graph
{
    int numVert;
    std::list<edge>* adj;
    bool* visited;

    void DFSearch(int vertex);
    void BFSearch(int vertex);
    void printBF(std::vector<int>& dist, int vertex);
    void BellmanFord(std::vector<int>& dist, int vertex);
    void topologicalSortUtil(std::stack<int>& stack, int vertex);
    void DijkstraUtil(std::vector<int>& dist, int src);
    void PrimUtil(std::vector<int>& mst, std::vector<int>& dist);
    int find_parent(std::vector<int>& parent, int vertex);
    void KruskalUtil(std::vector<std::pair<int,edge>>& graph, std::vector<std::pair<int, edge>>& mst, std::vector<int>& parent);
    bool BFSmaxFlow(std::vector<std::vector<int>>& mat, int src, int dest, std::vector<int>& parent);
    int FordFulkersonUtil(int src, int dest);
    void ListtoMat(std::vector<std::vector<int>>& mat);
    void KosarajuUtil(std::stack<int>& gstack);
    Graph transpose();
    void printMat(std::vector<std::vector<int>>& mat);
    void FloydWarshallUtil(std::vector<std::vector<int>>& mat);
    
    public:
    
    Graph(int vertice);
    void addEdge(int src, int end, int weight);
    void removeEdge(int src, int dest, int weight);
    void DFS(int vertex);
    void BFS(int vertex);
    void BellmanFord(int vertex);
    void topologicalSort();
    void Dijkstra(int src);
    void Prim();
    void Kruskal();
    void FordFulkerson(int src, int dest);
    void Kosaraju();
    void FloydWarshall();
};

#endif