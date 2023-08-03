#include <iostream>
#include <limits.h>
#include "minHeap.h"

// Function to swap the value of 2 pointer in the heap
template <typename T>
void minHeap<T>::swap(T* a, T* b)
{
    T temp = *a;
    *a = *b;
    *b = temp;
    return;
}

// Function to heapify the heap
template <typename T>
void minHeap<T>::Heapify(int size, int index)
{
    // Assign the leaft and right child of the current node position
    int smallest = index;
    int left = 2*index + 1;
    int right = 2*index + 2;

    // Find the smallest between the three nodes
    if(left < size && minheap[left] < minheap[smallest])
        smallest = left;
    if(right < size && minheap[right] < minheap[smallest])
        smallest = right;
    
    // Swap the position of the current node with the smallest node
    if(smallest != index)
    {
        swap(&minheap[index], &minheap[smallest]);
        Heapify(size, smallest);
    }
}

// Function to insert a new node into the heap
template <typename T>
void minHeap<T>::insert(T data)
{
    // Insert the new node at the end of the array
    if(minheap.size() == 0)
        minheap.push_back(data);
    else
    {
        minheap.push_back(data);
        // Heapify the array
        for(int i = minheap.size() / 2 - 1; i >= 0; --i)
            Heapify(minheap.size(), i);
    }
}

// Function to display the heap 
template <typename T>
void minHeap<T>::printHeap()
{
    // Check if the heap is empty
    if(minheap.size() == 0)
        return;
    std::cout<<"Elements in the min heap: ";
    for(int i = 0; i < minheap.size(); ++i)
        std::cout<<minheap[i]<<" ";
    std::cout<<std::endl;
}

// Function to delete a node in the heap
template <typename T>
void minHeap<T>::remove(T data)
{
    // Search for the position of the node to be deleted in the array
    int i;
    for(i = 0; i < minheap.size(); ++i)
    {
        if(minheap[i] == data)
            break;
    }
    // Check if the node is presented in the heap
    if(i == minheap.size())
    {
        std::cout<<"Node "<<data<<" not found!"<<std::endl;
        return;
    }
    // Swap the deleted node with the last node in the heap
    swap(&minheap[i], &minheap[minheap.size() - 1]);
    // Pop it out
    minheap.pop_back();
    // Heapify the array once again
    for(int j = minheap.size() / 2 - 1; j >= 0; --j)
        Heapify(minheap.size(), j);
}

// Function to get the minimum number of the heap
template <typename T>
T minHeap<T>::getMin()
{
    if(minheap.size() == 0)
        return INT_MIN;
    return minheap[0];
}

// Function to get the size of the heap array
template <typename T>
int minHeap<T>::getSize()
{
    return minheap.size();
}

// Function to sort the array using the heap property
template <typename T>
void minHeap<T>::heapsort()
{
    // Craete the min heap if it is not done yet
    // for(int i = minheap.size() / 2 - 1; i >= 0 ; --i)
    //     Heapify(minheap.size(), i);
    // Heap sort algorithm
    for(int i = minheap.size() - 1; i >= 0; i--)
    {
        swap(&minheap[i], &minheap[0]);
        Heapify(i, 0);
    }
}

int main()
{
    // Initiate the heap 
    minHeap<int> mheap;
    int random[] = {23, 76, 3, 4, 8, 5, 90, 101, 32, 22, 27, 35, 109};
    for(int i = 0; i < sizeof(random)/sizeof(int); ++i)
        mheap.insert(random[i]);
    // Display the the heap
    mheap.printHeap();
    // Delete a few node in the heap
    mheap.remove(35);
    mheap.remove(22);
    mheap.remove(510);
    // Display the heap once again
    mheap.printHeap();
    // Get the minimum element in the heap
    std::cout<<"The minimum element in the heap is: "<<mheap.getMin()<<std::endl;
    // Get the size of the heap
    std::cout<<"Size of the heap is: "<<mheap.getSize()<<std::endl;
    // Sort the heap array
    mheap.heapsort();
    mheap.printHeap();

    return 0;
}