#include <list>

// Initiate the class of graph
class Graph
{
    int numVert;
    std::list<int>* adj;
    bool* visited;

    void DFSearch(int vertex);
    void BFSearch(int vertex);
    
    public:
    
    Graph(int vertice);
    void addEdge(int src, int end);
    void DFS(int vertex);
    void BFS(int vertex);
};
