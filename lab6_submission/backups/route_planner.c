#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include <float.h>
#include "route_planner.h"

#define MAX_NODES 10000
#define MAX_EDGES 50000
#define EARTH_RADIUS 6371.0

// Node structure
typedef struct {
    int id;
    double lat;
    double lon;
    double earliest;
    double latest;
} Node;

// Edge structure
typedef struct Edge {
    int to;
    double weight;      // Cost or distance of traversal
    struct Edge* next;  // Pointer to next edge in adjacency list
} Edge;

// Graph structure
typedef struct {
    Node nodes[MAX_NODES];      // Array of all nodes with their CSV properties filled in
    Edge* adj_list[MAX_NODES];  // Adjacency list (linked list of edges from node i)
    int node_count;             // Total nodes currently in the graph
    int node_ids[MAX_NODES];    // Maps array index to actual node ID for lookups
} Graph;

// Priority queue node for Dijkstra and A*
typedef struct {
    int node;
    double priority;            // Needed for part 1.2 implementation
} PQNode;

// Priority queue
typedef struct {
    PQNode heap[MAX_NODES];     // Binary heap
    int size;                   // Current number of elements in the heap
} PriorityQueue;

// Priority queue functions
void pq_init(PriorityQueue* pq) {
    pq->size = 0;               // Start with an empty heap
}

void pq_push(PriorityQueue* pq, int node, double priority) {
    // Add element to the end and increment the Priority Queue's size.
    int i = pq->size++;
    pq->heap[i].node = node;
    pq->heap[i].priority = priority;
    
    // Bubble up to restore heap (move the element where it belongs)
    while (i > 0) {
        int parent = (i - 1) / 2;
        // If the parent has a smaller priority, the heap property is satisfied
        if (pq->heap[parent].priority <= pq->heap[i].priority) 
            break;

        // Swap with parent and continue up the binary tree
        PQNode temp = pq->heap[parent];
        pq->heap[parent] = pq->heap[i];
        pq->heap[i] = temp;
        i = parent;
    }
}

PQNode pq_pop(PriorityQueue* pq) {
    PQNode result = pq->heap[0];        // Save the minimum element found at the root
    pq->heap[0] = pq->heap[--pq->size]; // Move the last element to the root
    
    // Bubble down to restore heap (move the new root where it belongs)
    int i = 0;
    while (1) {
        int left = 2 * i + 1;   // left child
        int right = 2 * i + 2;  // right child
        int smallest = i;       // track which position has minimum priority
        
        // Find the smallest among the parent and its children
        if (left < pq->size && pq->heap[left].priority < pq->heap[smallest].priority)
            smallest = left;
        if (right < pq->size && pq->heap[right].priority < pq->heap[smallest].priority)
            smallest = right;
        
        // If the parent is already the smallest, the heap property is restored
        if (smallest == i) break;
        
        // Swap with the smaller child and continue to bubble down
        PQNode temp = pq->heap[i];
        pq->heap[i] = pq->heap[smallest];
        pq->heap[smallest] = temp;
        i = smallest;
    }
    
    return result; // return min
}

// Return whether the Priority Queue is empty
int pq_empty(PriorityQueue* pq) {
    return pq->size == 0;
}

// Graph functions
void graph_init(Graph* g) {
    g->node_count = 0; // Graph starts with no nodes

    for (int i = 0; i < MAX_NODES; i++) {
        g->adj_list[i] = NULL;      // No edges yet
        g->node_ids[i] = -1;        // Give node IDs sentinel values
    }
}

int find_node_index(Graph* g, int node_id) {
    // Linear search through all of the nodes
    for (int i = 0; i < g->node_count; i++) {
        if (g->node_ids[i] == node_id) 
            return i;   // Node found
    }
    return -1;          // Node not found
}

void add_edge(Graph* g, int from, int to, double weight) {
    // Allocate memory for a new edge
    Edge* edge = (Edge*)malloc(sizeof(Edge));

    // Initialize edge properties
    edge->to = to;
    edge->weight = weight;
    edge->next = g->adj_list[from];

    // Insert at the head of the adjacency list
    g->adj_list[from] = edge;
}

/**************************************************************************************************
 *                                      Dijkstra's Algorithm                                      *
 *************************************************************************************************/ 
void dijkstra(Graph* g, int start_idx, int end_idx, double* dist, int* prev, int* nodes_explored) {
    PriorityQueue pq;
    pq_init(&pq); // Initialize an empty Priority Queue
    
    // Initialize all distances to infinity and previous pointers to an invalid sentinel value
    for (int i = 0; i < g->node_count; i++) {
        dist[i] = DBL_MAX;
        prev[i] = -1;
    }
    
    // Starting node has a distance of 0
    dist[start_idx] = 0;

    // Add starting node to the Priority Queue
    pq_push(&pq, start_idx, 0);
    *nodes_explored = 0;
    
    // Main Loop of Dijkstra
    while (!pq_empty(&pq)) {

        // Find the node with the minimum distance
        PQNode current = pq_pop(&pq);
        int u = current.node;
        (*nodes_explored)++; // Track the number of nodes looked at
        
        // Stop if the end has been reached
        if (u == end_idx) break;
        
        // Skip if a better path has already been found for this node
        if (current.priority > dist[u]) continue;
        
        // Check neighbors of the current node 
        Edge* edge = g->adj_list[u]; // Get the head of the linked list for the node u
        while (edge != NULL) {
            int v = edge->to;   // Neighbor
            double alt = dist[u] + edge->weight; // Distance from current node
            
            // If a shorter path is found to neighbor v, update it
            if (alt < dist[v]) {
                dist[v] = alt;  // Update shortest distance
                prev[v] = u;    // Record that we came from u
                pq_push(&pq, v, alt); // Add the neighbor to queue with the new distance
            }
            edge = edge->next; // Move to evaluate the next edge in the adjacency list
        }
    }
}


void print_path(Graph* g, int* prev, int start_idx, int end_idx, double distance) {

    // Ensure a valid path exists
    if (prev[end_idx] == -1) {
        printf("No path found\n");
        return;
    }
    
    // Build path (follow prev pointers)
    int path[MAX_NODES];
    int path_len = 0;
    int current = end_idx;
    
    // Trace from destination to starting node
    while (current != -1) {
        path[path_len++] = current; // Add the current node to the path
        current = prev[current];
    }
    
    // Print the path from start to finish
    printf("Path from %d to %d: ", g->node_ids[start_idx], g->node_ids[end_idx]);
    for (int i = path_len - 1; i >= 0; i--) {
        printf("%d", g->node_ids[path[i]]);
        if (i > 0) printf(" -> ");
    }
    printf("\nTotal distance: %.2f km\n", distance);
}

void print_all_nodes(Graph* g) {
    
    // Print header
    printf("\n");
    printf("================================================================================\n");
    printf("                          ALL NODES IN GRAPH\n");
    printf("================================================================================\n");
    printf("Total Nodes: %d\n", g->node_count);
    printf("================================================================================\n\n");
    
    // Print column headers with proper alignment
    printf("%-8s %-8s %-12s %-12s %-12s %-12s\n",
           "Index", "ID", "Latitude", "Longitude", "Earliest", "Latest");
    printf("%-8s %-8s %-12s %-12s %-12s %-12s\n",
           "-----", "-----", "----------", "----------", "----------", "----------");
    
    // Print each node
    for (int i = 0; i < g->node_count; i++) {
        printf("%-8d %-8d %-12.4f %-12.4f %-12.1f %-12.1f\n",
               i,                          // Array index
               g->nodes[i].id,             // Node ID
               g->nodes[i].lat,            // Latitude
               g->nodes[i].lon,            // Longitude
               g->nodes[i].earliest,       // Earliest arrival time
               g->nodes[i].latest);        // Latest arrival time
    }
    
    printf("\n================================================================================\n\n");
}

void print_all_edges(Graph* g) {
    
    // Print header
    printf("\n");
    printf("================================================================================\n");
    printf("                          ALL EDGES IN GRAPH\n");
    printf("================================================================================\n");
    
    // Count total edges first
    int total_edges = 0;
    for (int i = 0; i < g->node_count; i++) {
        Edge* edge = g->adj_list[i];
        while (edge != NULL) {
            total_edges++;
            edge = edge->next;
        }
    }
    
    printf("Total Edges: %d\n", total_edges);
    printf("================================================================================\n\n");
    
    // Print column headers with proper alignment
    printf("%-8s %-10s %-10s %-12s\n",
           "Edge #", "From", "To", "Distance");
    printf("%-8s %-10s %-10s %-12s\n",
           "-------", "-------", "-------", "----------");
    
    // Print each edge
    int edge_count = 0;
    for (int i = 0; i < g->node_count; i++) {
        Edge* edge = g->adj_list[i];
        
        while (edge != NULL) {
            edge_count++;
            
            printf("%-8d %-10d %-10d %-12.4f\n",
                   edge_count,                    // Edge number
                   g->node_ids[i],                // From node ID
                   g->node_ids[edge->to],         // To node ID
                   edge->weight);                 // Distance/Weight
            
            edge = edge->next;
        }
    }
    
    printf("\n================================================================================\n\n");
}

int main(int argc, char* argv[]) {
    if (argc != 6) {
        printf("Usage: %s <nodes.csv> <edges.csv> <start_node> <end_node> <algorithm>\n", argv[0]);
        printf("Algorithms: dijkstra, astar, bellman-ford\n");
        return 1;
    }
    
    char* nodes_file = argv[1];
    char* edges_file = argv[2];
    int start_node = atoi(argv[3]);
    int end_node = atoi(argv[4]);
    char* algorithm = argv[5];
    
    Graph g;
    graph_init(&g);
    
    // Load nodes
    FILE* fp = fopen(nodes_file, "r");
    if (!fp) {
        printf("Error opening nodes file\n");
        return 1;
    }
    
    char line[256];
    fgets(line, sizeof(line), fp); // Skip header
    
    // UPDATED TO READ IN START AND END TIMES FOR TIME WINDOW 1.1 IMPLEMENTATION
    while (fgets(line, sizeof(line), fp)) {
        int id;
        double lat, lon, earliest, latest;
        if (sscanf(line, "%d,%lf,%lf,%lf,%lf", &id, &lat, &lon, &earliest, &latest) == 5) {
            g.nodes[g.node_count].id = id;
            g.nodes[g.node_count].lat = lat;
            g.nodes[g.node_count].lon = lon;
            g.node_ids[g.node_count] = id;
            g.nodes[g.node_count].earliest = earliest;
            g.nodes[g.node_count].latest = latest;
            g.node_count++;
        }
    }
    fclose(fp);
    
    // Load edges
    fp = fopen(edges_file, "r");
    if (!fp) {
        printf("Error opening edges file\n");
        return 1;
    }
    
    fgets(line, sizeof(line), fp); // Skip header
    
    while (fgets(line, sizeof(line), fp)) {
        int from, to;
        double distance;
        if (sscanf(line, "%d,%d,%lf", &from, &to, &distance) == 3) {
            int from_idx = find_node_index(&g, from);
            int to_idx = find_node_index(&g, to);
            if (from_idx != -1 && to_idx != -1) {
                add_edge(&g, from_idx, to_idx, distance);
            }
        }
    }
    fclose(fp);
    
    // Find start and end indices
    int start_idx = find_node_index(&g, start_node);
    int end_idx = find_node_index(&g, end_node);
    
    if (start_idx == -1 || end_idx == -1) {
        printf("Invalid start or end node\n");
        return 1;
    }

    print_all_nodes(&g);
    print_all_edges(&g);
    
    double dist[MAX_NODES];
    int prev[MAX_NODES];
    int nodes_explored = 0;
    
    // Run selected algorithm
    if (strcmp(algorithm, "dijkstra") == 0) {
        printf("=== Dijkstra's Algorithm ===\n");
        dijkstra(&g, start_idx, end_idx, dist, prev, &nodes_explored);
    } else if (strcmp(algorithm, "astar") == 0) {
        printf("=== A* Algorithm ===\n");
        printf("Sorry, A* was removed from this implementation....");
    } else if (strcmp(algorithm, "bellman-ford") == 0) {
        printf("=== Bellman-Ford Algorithm ===\n");
        printf("Sorry, Bellman-Ford was removed from this implementation....");
    } else {
        printf("Unknown algorithm: %s\n", algorithm);
        printf("Available algorithms: dijkstra, astar, bellman-ford\n");
        return 1;
    }
    
    print_path(&g, prev, start_idx, end_idx, dist[end_idx]);
    printf("Nodes explored: %d\n", nodes_explored);
    
    // Cleanup
    for (int i = 0; i < g.node_count; i++) {
        Edge* edge = g.adj_list[i];
        while (edge != NULL) {
            Edge* next = edge->next;
            free(edge);
            edge = next;
        }
    }
    
    return 0;
}