#include <iostream>
#include <queue>
#include "graph.h"

// Constructor of the class
Graph::Graph(int vertices)
{
    numVert = vertices;
    adj = new std::list<int>[vertices];
    visited = new bool[vertices];
}

// Function to add edge into the class
void Graph::addEdge(int src, int end)
{
    adj[src].push_front(end);
}

// Function to perform depth first search algorithm
void Graph::DFSearch(int vertex)
{
    // Update the visisted element
    visited[vertex] = true;

    // Display to the terminal the visiting vertex
    std::cout<<vertex<<" ";

    std::list<int>::iterator it;
    // Visited all other unvisited adjacent node of the current vertex
    for(it = adj[vertex].begin(); it != adj[vertex].end(); ++it)
    {
        if(! visited[*it])
            DFSearch(*it);
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
            if(! visited[*it])
            {
                visited[*it] = true;
                bfqueue.push(*it);
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

int main()
{
    // Intiiate the graph
    Graph g(4);
    g.addEdge(0, 1);
    g.addEdge(0, 2);
    g.addEdge(1, 2);
    g.addEdge(2, 0);
    g.addEdge(2, 3);   

    // DFS the graph  
    g.BFS(2);

    return 0;
}