#include <iostream>
#include <algorithm>
#include <queue>
#include <limits.h>
#include "graph.h"

// Constructor of the class
Graph::Graph(int vertices)
{
    numVert = vertices;
    adj = new std::list<edge>[vertices];
    visited = new bool[vertices];
}

// Function to add edge into the class
void Graph::addEdge(int src, int dest, int weight)
{
    adj[src].push_front(edge(dest, weight));
    adj[dest].push_front(edge(src, weight));
}

// Function to remove edge from the graph
void Graph::removeEdge(int src, int dest, int weight)
{
    adj[src].remove(edge(dest, weight));
    adj[dest].remove(edge(src, weight));
}

// Function to perform depth first search algorithm
void Graph::DFSearch(int vertex)
{
    // Update the visisted element
    visited[vertex] = true;

    // Display to the terminal the visiting vertex
    std::cout<<vertex<<" ";

    // Visited all other unvisited adjacent node of the current vertex
    for(auto it = adj[vertex].begin(); it != adj[vertex].end(); ++it)
    {
        if(! visited[(*it).first])
            DFSearch((*it).first);
    }
}

// Function to generally call the depth first search algorithm
void Graph::DFS(int vertex)
{
    // Set the visited array
    for(int i = 0; i < numVert; ++i)
        visited[i] = false;
    std::cout<<"Element of the graph using DFS: ";
    DFSearch(vertex);
    std::cout<<std::endl;
}

// Function to perform breadth first search
void Graph::BFSearch(int vertex)
{
    std::queue<int> bfqueue;
    // Update visited the current vertex
    visited[vertex] = true;

    // Push the current vertex into the queue
    bfqueue.push(vertex);

    // BFS the graph 
    while(! bfqueue.empty())
    {
        int temp = bfqueue.front();
        std::cout<<temp<<" ";
        bfqueue.pop();

        // Visit adjacent vertices
        for(auto it = adj[temp].begin(); it != adj[temp].end(); ++it)
        {
            if(! visited[(*it).first])
            {
                visited[(*it).first] = true;
                bfqueue.push((*it).first);
            }
        }
    }
}

// Function to generally call the BFS algorithm
void Graph::BFS(int vertex)
{
    // Set the visited array
    for(int i = 0; i < numVert; ++i)
        visited[i] = false;
    std::cout<<"Element of the graph using BFS: ";
    BFSearch(vertex);
    std::cout<<std::endl;
}

// Function to print elements in a vector 
void Graph::printBF(std::vector<int>& dist, int vertex)
{
    std::cout<<"\nBellman Ford shortest distance"<<std::endl;
    for(int i = 0; i < dist.size(); ++i)
        std::cout<<"Shortest distance from "<<vertex<<" to "<<i<<" node is: "<<dist[i]<<std::endl;
}

// Function to perfrom the Bellman Ford algorithm to find the shortest paths 
void Graph::BellmanFord(std::vector<int>& dist, int vertex)
{
    // Set the distance of the start node to 0
    dist[vertex] = 0;
    
    // First round loop to relax the distances to the nodes
    for(int i = 1; i < numVert - 1; ++i)
    {
        // Trvaverse all the edges in the graph
        for(int j = 0; j < numVert; ++j)
        {
            for(auto it = adj[j].begin(); it != adj[j].end(); ++it)
            {
                int dest = (*it).first;
                int weight = (*it).second;
                if(dist[j] != INT_MAX && dist[j] + weight < dist[dest])
                    dist[dest] = dist[j] + weight;
            }
        }
    }

    // Second loop to define the negative weight path
    for(int j = 0; j < numVert; ++j)
    {
        for(auto it = adj[j].begin(); it != adj[j].end(); ++it)
        {
            int dest = (*it).first;
            int weight = (*it).second;
            if(dist[j] != INT_MAX && dist[j] + weight < dist[dest])
            {
                std::cout<<"Graph contains negative weight cycle"<<std::endl;
                return;
            }
        }
    }
}

// General function to call the Bellman Ford algorithm
void Graph::BellmanFord(int vertex)
{
    // Initiate the vector 
    std::vector<int> dist(numVert);

    // Set all the initial distance to maximum
    for(int i = 0; i < dist.size(); ++i)
        dist[i] = INT_MAX;

    BellmanFord(dist, vertex);

    // Display the result to the terminal
    printBF(dist, vertex);

    return;
}

// Function to perdorm the topological sort for Directed Acyclic Graph (DAG)
void Graph::topologicalSortUtil(std::stack<int>& stack, int vertex)
{
    // Update the visited of the current node
    visited[vertex] = true;

    // DFS
    for(auto it = adj[vertex].begin(); it != adj[vertex].end(); ++it)
    {
        if(! visited[(*it).first])
            topologicalSortUtil(stack, (*it).first);
    }
    // Push the node into the stack
    stack.push(vertex);
}

// General function to perform topological sort
void Graph::topologicalSort()
{
    // Set the visited array
    for(int i = 0; i < numVert; ++i)
        visited[i] = false;
    
    std::stack<int> stack;

    // Reacursively call the Util function for topological sorting
    for(int i = 0; i < numVert; ++i)
    {
        if(! visited[i])
            topologicalSortUtil(stack, i);
    }

    // Display content of the stack
    std::cout<<"The topological sort of the graph: ";
    while(! stack.empty())
    {
        std::cout<<stack.top()<<" ";
        stack.pop();
    }
    std::cout<<std::endl;
}

// Function to perform Dijkstra algorithm to find the shortest path 
void Graph::DijkstraUtil(std::vector<int>& dist, int src)
{
    // Initiate the min heap
    std::priority_queue<edge, std::vector<edge>, std::greater<edge>> mheap;

    // BFS the whole graph
    mheap.push(std::make_pair(0, src));
    dist[src] = 0;

    while(! mheap.empty())
    {
        src = mheap.top().second;
        mheap.pop();

        // Loop through all of the current node's adjacent nodes
        for(auto it = adj[src].begin(); it != adj[src].end(); ++it)
        {
            int dest = (*it).first;
            int weight = (*it).second;

            // Update the shortest distance if there is path to v
            if(dist[src] + weight < dist[dest])
            {
                dist[dest] = dist[src] + weight;
                mheap.push(std::make_pair(weight, dest));
            }
        }
    }
}

// General function to perform Dijkstra algorithm and interface with user
void Graph::Dijkstra(int src)
{
    // Initiate the distance vector
    std::vector<int> dist(numVert);

    // Set all the initial distance to maximum
    for(int i = 0; i < dist.size(); ++i)
        dist[i] = INT_MAX;

    DijkstraUtil(dist, src);

    // Display the result to the terminal
    std::cout<<"\nDijkstra shortest path"<<std::endl;
    for(int i = 0; i < dist.size(); ++i)
        std::cout<<"Distance from "<<src<<" to "<<i<<" is: "<<dist[i]<<std::endl;
    
    return;
}

// Function to perform the prim algorithm to find the minimum spanning tree in the graph
void Graph::PrimUtil(std::vector<int>& mst, std::vector<int>& dist)
{
    int src = 0;

    // Intiiate the priority queue
    std::priority_queue<edge, std::vector<edge>, std::greater<edge>> mheap;

    mheap.push(std::make_pair(0, src));
    dist[src] = 0;

    // BFS the graph
    while(! mheap.empty())
    {
        src = mheap.top().second;
        mheap.pop();

        // Check wether we had add the edge to the MST
        if(visited[src])
            continue;
        visited[src] = true;

        // Loop through all the adjacent nodes
        for(auto it = adj[src].begin(); it != adj[src].end(); ++it)
        {
            int dest = (*it).first;
            int weight = (*it).second;

            // Update the MST
            if(visited[dest] == false && dist[dest] > weight)
            {
                dist[dest] = weight;
                mheap.push(std::make_pair(weight, dest));
                mst[dest] = src;
            }
        }
    }
    return;
}

// General function to perform Prim algorithm to find the MST
void Graph::Prim()
{
    // Set the visited array
    for(int i = 0; i < numVert; ++i)
        visited[i] = false;

    // Initiate some variables
    std::vector<int> mst(numVert);
    std::vector<int> dist(numVert);
    for(int i = 0; i < dist.size(); ++i)
        dist[i] = INT_MAX;

    PrimUtil(mst, dist);

    // Dsipaly the MST
    std::cout<<"\nThe MST using Prim algorithm\n";
    for(int i = 1; i < mst.size(); ++i)
    {
        std::cout<<"Edge from "<<mst[i]<<" to "<<i<<std::endl;
    }
}

// Function to find the parent of nodes in an edge to define if adding the edge into MST will create a cycle
int Graph::find_parent(std::vector<int>& parent, int vertex)
{
    if(parent[vertex] == vertex)
        return vertex;
    return find_parent(parent, parent[vertex]);
}

// Function to find the MST by Kruskal algorithm
void Graph::KruskalUtil(std::vector<std::pair<int, edge>>& graph, std::vector<std::pair<int, edge>>& mst, std::vector<int>& parent)
{
    // Sort the graph according to the weight of the paths
    std::sort(graph.begin(), graph.end());

    // Traverse all the edges and input in the MST if there is no cycle
    for(int i = 0; i < graph.size(); ++i)
    {
        int fiPar = find_parent(parent, graph[i].second.first);
        int sePar = find_parent(parent, graph[i].second.second);
        if(fiPar != sePar)
        {
            mst.push_back(graph[i]);
            parent[fiPar] = parent[sePar];
        }
    }
}

// Function to generally perform Kruskal algorithm to find the MST
void Graph::Kruskal()
{
    // Initiate some variables
    std::vector<std::pair<int, edge>> mst;
    std::vector<std::pair<int, edge>> graph;
    // Copy all the edge into the vector
    for(int j = 0; j < numVert; ++j)
    {
        for(auto it = adj[j].begin(); it != adj[j].end(); ++it)
        {
            graph.push_back(std::make_pair((*it).second, edge(j, (*it).first)));
        }
    }

    // Initiate the parent vector
    std::vector<int> parent(numVert);
    for(int i = 0; i < parent.size(); ++i)
        parent[i] = i;

    KruskalUtil(graph, mst, parent);

    // Display to the terminal
    std::cout<<"\nThe MST by Kruskal algorithm\n";
    for(int i = 0; i < mst.size(); ++i)
        std::cout<<"Edge from "<<mst[i].second.first<<" to "<<mst[i].second.second<<" : "<<mst[i].first<<std::endl;
    
    return;
}

// Function to find the possible path between 2 nodes in the network using BFS
bool Graph::BFSmaxFlow(std::vector<std::vector<int>>& mat, int src, int dest, std::vector<int>& parent)
{
    // Set up the visited array
    for(int i = 0; i < numVert; ++i)
        visited[i] = false;

    // Initiate the queue
    std::queue<int> qgraph;

    qgraph.push(src);
    visited[src] = true;
    parent[src] = -1;

    while(! qgraph.empty())
    {
        src = qgraph.front();
        qgraph.pop();

        // Loop through all the adjacent vertices
        for(int i = 0; i < numVert; ++i)
        {
            if(! visited[i] && mat[src][i] > 0)
            {
                qgraph.push(i);
                parent[i] = src;
                visited[i] = true;
            }
        }
    }
    return visited[dest];
}

// Function to convert the adjacent list to the adjecent matrix
void Graph::ListtoMat(std::vector<std::vector<int>>& mat)
{
    for(int i = 0; i < numVert; ++i)
    {
        for(auto it = adj[i].begin(); it != adj[i].end(); ++it)
        {
            mat[i][(*it).first] = (*it).second;
        }
    }
}

// Function to perform Ford Fulkerson algorithm to find the maximum flow among the whole network
int Graph::FordFulkersonUtil(int src, int dest)
{
    // Initiate some variables
    int u, v;
    int max_flow = 0;
    std::vector<int> parent(numVert);
    std::vector<std::vector<int>> mat(numVert, std::vector<int>(numVert, 0));

    // Convert the adjacent list to the adjacent matrix
    ListtoMat(mat);

    // Find the path from src to dest and update the residual graph
    while(BFSmaxFlow(mat, src, dest, parent))
    {
        int path_flow = INT_MAX;
        // Find the minimum flow on the path
        for(v = dest; v != src; v = parent[v])
        {
            u = parent[v];
            path_flow = (path_flow < mat[u][v]) ? path_flow : mat[u][v];    
        }

        // Update the residual graph
        for(v = dest; v != src; v = parent[v])
        {
            u = parent[v];
            mat[u][v] -= path_flow;
            mat[v][u] += path_flow;    
        }
        // Update the max flow
        max_flow += path_flow;
    }
    return max_flow;
}

// Function to generally find maximum flow using Ford Fulkerson algorithm
void Graph::FordFulkerson(int src, int dest)
{
    std::cout<<"The maimum flow from "<<src<<" to "<<dest<<" in the network is : "<<FordFulkersonUtil(src, dest)<<std::endl;
}

int main()
{
    // Intiiate the graph
    Graph g(9);
 
    //  making above shown graph
    g.addEdge(0, 1, 4);
    g.addEdge(0, 7, 8);
    g.addEdge(1, 2, 8);
    g.addEdge(1, 7, 11);
    g.addEdge(2, 3, 7);
    g.addEdge(2, 8, 2);
    g.addEdge(2, 5, 4);
    g.addEdge(3, 4, 9);
    g.addEdge(3, 5, 14);
    g.addEdge(4, 5, 10);
    g.addEdge(5, 6, 2);
    g.addEdge(6, 7, 1);
    g.addEdge(6, 8, 6);
    g.addEdge(7, 8, 7);

    // DAG to perform topological sort
    // g.addEdge(5, 2, 3);
    // g.addEdge(5, 0, 7);
    // g.addEdge(4, 0, 4);
    // g.addEdge(4, 1, 2);
    // g.addEdge(2, 3, 1);
    // g.addEdge(3, 1, 3);

    // DFS the graph  
    g.BFS(3);

    // Bellman Ford algorithm
    g.BellmanFord(2);

    // Remove some edge in the tree
    g.removeEdge(5, 2, 2);

    // Shortest path algorithms
    g.BellmanFord(2);
    g.Dijkstra(2);

    // MST algorithms
    g.Prim();
    g.Kruskal();

    // Maimum flow algorithms
    // g.FordFulkerson(0, 8);

    // Topological sort
    // g.topologicalSort();

    return 0;
}