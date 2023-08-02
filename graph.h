#include <list>

// Initiate the class of graph
#define edge std::pair<int, int>
class Graph
{
    int numVert;
    std::list<edge>* adj;
    bool* visited;

    void DFSearch(int vertex);
    void BFSearch(int vertex);
    void printBF(std::vector<int>& dist, int vertex);
    void BellmanFord(std::vector<int>& dist, int vertex);
    
    public:
    
    Graph(int vertice);
    void addEdge(int src, int end, int weight);
    void DFS(int vertex);
    void BFS(int vertex);
    void BellmanFord(int vertex);
};
