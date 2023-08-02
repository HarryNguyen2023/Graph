#include <iostream>
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

int main()
{
    // Intiiate the graph
    Graph g(6);
    g.addEdge(0, 1, 4);
    g.addEdge(0, 2, 4);
    g.addEdge(1, 2, 2);
    g.addEdge(1, 0, 4);
    g.addEdge(2, 0, 4);
    g.addEdge(2, 1, 2);
    g.addEdge(2, 3, 3);
    g.addEdge(2, 5, 2);
    g.addEdge(2, 4, 4);
    g.addEdge(3, 2, 3);
    g.addEdge(3, 4, 3);
    g.addEdge(4, 2, 4);
    g.addEdge(4, 3, 3);
    g.addEdge(5, 2, 2);
    g.addEdge(5, 4, 3);   

    // DAG to perform topological sort
    // g.addEdge(5, 2, 3);
    // g.addEdge(5, 0, -7);
    // g.addEdge(4, 0, 4);
    // g.addEdge(4, 1, 2);
    // g.addEdge(2, 3, -1);
    // g.addEdge(3, 1, 3);

    // DFS the graph  
    g.BFS(3);

    // Bellman Ford algorithm
    g.BellmanFord(2);

    // Topological sort
    // g.topologicalSort();

    return 0;
}