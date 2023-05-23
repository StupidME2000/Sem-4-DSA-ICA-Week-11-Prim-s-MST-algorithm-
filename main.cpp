#include <iostream>
#include <vector>
#include <queue>

using namespace std;

// Structure to represent an edge in the graph
struct Edge {
    int source;
    int destination;
    int weight;

    Edge(int src, int dest, int w) : source(src), destination(dest), weight(w) {}
};

// Function to add an edge to the graph
void addEdge(vector<vector<Edge>>& graph, int src, int dest, int weight) {
    graph[src].push_back(Edge(src, dest, weight));
    graph[dest].push_back(Edge(dest, src, weight));
}

// Function to find the minimum spanning tree using Prim's algorithm
void primMST(vector<vector<Edge>>& graph, int startVertex) {
    int V = graph.size();

    // Create a priority queue to store vertices and their key values
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

    // Create a vector to store the key values of vertices
    vector<int> key(V, INT_MAX);

    // Create a vector to store the parent of vertices in MST
    vector<int> parent(V, -1);

    // Create a vector to store whether a vertex is included in MST or not
    vector<bool> inMST(V, false);

    // Insert the start vertex into the priority queue and set its key value to 0
    pq.push(make_pair(0, startVertex));
    key[startVertex] = 0;

    while (!pq.empty()) {
        // Extract the vertex with the minimum key value from the priority queue
        int u = pq.top().second;
        pq.pop();

        // Include the extracted vertex in the MST
        inMST[u] = true;

        // Traverse all adjacent edges of the extracted vertex
        for (auto edge : graph[u]) {
            int v = edge.destination;
            int weight = edge.weight;

            // If v is not in MST and weight of the edge is smaller than the current key value of v
            if (!inMST[v] && weight < key[v]) {
                // Update the key value of v
                key[v] = weight;

                // Insert v and its key value into the priority queue
                pq.push(make_pair(key[v], v));

                // Set the parent of v in MST as u
                parent[v] = u;
            }
        }
    }

    // Print the edges of the minimum spanning tree
    cout << "Minimum Spanning Tree Edges:" << endl;
    for (int i = 1; i < V; ++i) {
        cout << parent[i] << " - " << i << endl;
    }
}

int main() {
    int V = 6; // Number of vertices

    // Create an empty graph
    vector<vector<Edge>> graph(V);

    // Add edges to the graph
    addEdge(graph, 0, 1, 3);
    addEdge(graph, 0, 2, 2);
    addEdge(graph, 1, 4, 10);
    addEdge(graph, 0, 5, 1);
    addEdge(graph, 3, 1, 1);
    addEdge(graph, 2, 5, 5);
    addEdge(graph, 2, 3, 3);
    addEdge(graph, 3, 4, 5);
    addEdge(graph, 4, 5, 4);


    // Starting vertex for Prim's algorithm
    int startVertex = 0;

    // Find the minimum spanning tree using Prim's algorithm
    primMST(graph, startVertex);

    return 0;
}
