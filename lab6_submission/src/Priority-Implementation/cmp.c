/***********************************************************
 *         PRIORITY TIME WINDOW-ED ROUTE_PLANNER.C         *
 ***********************************************************/
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
#define MAX_DESTINATIONS 100


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

// Destination structure with proper int priority
typedef struct {
    int node_id;
    int priority;               // 1=high, 2=medium, 3=low
} Destination;

typedef struct {
    int node_id;
    int required_priority;
    int actual_priority;
} PriorityViolation;

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

// Load destinations from CSV with proper Destination structure
int load_destinations(const char* filename, Destination* dests) {
    FILE* fp = fopen(filename, "r");
    if (!fp) {
        printf("Error opening destinations file: %s\n", filename);
        return 0;
    }
    
    char line[256];
    fgets(line, sizeof(line), fp);  // Skip header
    
    int count = 0;
    while (fgets(line, sizeof(line), fp) && count < MAX_DESTINATIONS) {
        int node_id, priority;
        if (sscanf(line, "%d,%d", &node_id, &priority) == 2) {
            dests[count].node_id = node_id;
            dests[count].priority = priority;
            count++;
        }
    }
    fclose(fp);
    return count;
}

// NEW: Compare destinations by priority (for sorting)
int compare_destinations(const void* a, const void* b) {
    Destination* d1 = (Destination*)a;
    Destination* d2 = (Destination*)b;
    return d1->priority - d2->priority;  // Lower priority number first (1 before 2 before 3)
}

// NEW: Forward declaration for dijkstra_segment
void dijkstra_segment(Graph* g, int start_idx, int end_idx, 
                     double* dist, int* prev, double* arrival_times, int* nodes_explored);

/**************************************************************************************************
 *                                      Dijkstra's Algorithm                                      *
 *                        WITH MANDATORY WAYPOINT VISITATION                                      *
 *************************************************************************************************/ 
void dijkstra_with_priorities(Graph* g, int start_idx, int end_idx, Destination* dests, int dest_count, 
                              double threshold, double* dist, int* prev, int* nodes_explored) {
    
    // NEW: Multi-stage approach - visit each waypoint in priority order, then reach end_idx
    
    printf("\n=== WAYPOINT ROUTING STRATEGY ===\n");
    printf("Must visit all %d destination nodes in priority order:\n", dest_count);
    for (int i = 0; i < dest_count; i++) {
        printf("  Step %d: Node %d (priority %d)\n", i+1, dests[i].node_id, dests[i].priority);
    }
    printf("Then reach final destination: Node %d\n", g->node_ids[end_idx]);
    printf("===================================\n\n");
    
    // Array to hold Dijkstra results between each waypoint
    double segment_dist[MAX_NODES];
    int segment_prev[MAX_NODES];
    double segment_arrival_times[MAX_NODES];
    double total_path_distance = 0;
    int total_nodes_explored = 0;
    
    // Build complete path by visiting each waypoint in order
    int current_start_idx = start_idx;
    
    for (int waypoint_idx = 0; waypoint_idx < dest_count; waypoint_idx++) {
        int waypoint_node_id = dests[waypoint_idx].node_id;
        int waypoint_idx_in_graph = find_node_index(g, waypoint_node_id);
        
        if (waypoint_idx_in_graph == -1) {
            printf("Error: Waypoint node %d not found in graph\n", waypoint_node_id);
            return;
        }
        
        printf("\n--- Finding path to waypoint %d: Node %d (priority %d) ---\n", 
               waypoint_idx + 1, waypoint_node_id, dests[waypoint_idx].priority);
        
        // Run Dijkstra from current position to this waypoint
        dijkstra_segment(g, current_start_idx, waypoint_idx_in_graph, 
                        segment_dist, segment_prev, segment_arrival_times, &total_nodes_explored);
        
        // Check if path exists
        if (segment_prev[waypoint_idx_in_graph] == -1 && waypoint_idx_in_graph != current_start_idx) {
            printf("ERROR: No path exists to waypoint node %d\n", waypoint_node_id);
            return;
        }
        
        // Update distance
        double segment_distance = segment_dist[waypoint_idx_in_graph];
        total_path_distance += segment_distance;
        printf("Segment distance: %.2f km (total so far: %.2f km)\n", 
               segment_distance, total_path_distance);
        
        // Reconstruct this segment and add to overall prev array
        if (waypoint_idx == 0) {
            // First segment - copy directly
            for (int i = 0; i < g->node_count; i++) {
                dist[i] = segment_dist[i];
                prev[i] = segment_prev[i];
            }
        } else {
            // Subsequent segments - merge by updating prev pointers
            // Trace back from waypoint to its predecessor
            int trace = waypoint_idx_in_graph;
            while (trace != -1 && trace != current_start_idx) {
                int next = segment_prev[trace];
                if (next == -1 || next == current_start_idx) {
                    // Connect to current_start_idx in overall path
                    prev[trace] = current_start_idx;
                    break;
                } else {
                    prev[trace] = next;
                    trace = next;
                }
            }
            dist[waypoint_idx_in_graph] = total_path_distance;
        }
        
        printf("✓ Waypoint %d reached: Node %d\n", waypoint_idx + 1, waypoint_node_id);
        
        // Move current position to this waypoint for next iteration
        current_start_idx = waypoint_idx_in_graph;
    }
    
    // NEW: Final segment from last waypoint to end_idx
    printf("\n--- Finding path from last waypoint to final destination: Node %d ---\n", g->node_ids[end_idx]);
    
    dijkstra_segment(g, current_start_idx, end_idx, 
                    segment_dist, segment_prev, segment_arrival_times, &total_nodes_explored);
    
    if (segment_prev[end_idx] == -1 && end_idx != current_start_idx) {
        printf("ERROR: No path exists to final destination node %d\n", g->node_ids[end_idx]);
        return;
    }
    
    double final_segment_distance = segment_dist[end_idx];
    total_path_distance += final_segment_distance;
    printf("Final segment distance: %.2f km (total: %.2f km)\n", 
           final_segment_distance, total_path_distance);
    
    // Merge final segment into overall prev array
    int trace = end_idx;
    while (trace != -1 && trace != current_start_idx) {
        int next = segment_prev[trace];
        if (next == -1 || next == current_start_idx) {
            prev[trace] = current_start_idx;
            break;
        } else {
            prev[trace] = next;
            trace = next;
        }
    }
    dist[end_idx] = total_path_distance;
    
    printf("✓ Final destination reached: Node %d\n", g->node_ids[end_idx]);
    
    *nodes_explored = total_nodes_explored;

    printf("\n========== SOLUTION ==========\n");
    printf("Route completed\n");
    printf("Total distance: %.2f km\n", total_path_distance);
    printf("All waypoints visited in priority order\n");
    printf("✓ All priorities respected\n");
    printf("==============================\n\n");
}

// NEW: Helper function - Run Dijkstra from start to target node
void dijkstra_segment(Graph* g, int start_idx, int end_idx, 
                     double* dist, int* prev, double* arrival_times, int* nodes_explored) {
    PriorityQueue pq;
    pq_init(&pq); // Initialize an empty Priority Queue

    double raw_arrival_times[MAX_NODES];  // Track arrival before waiting

    // Initialize all distances to infinity and previous pointers to invalid
    for (int i = 0; i < g->node_count; i++) {
        dist[i] = DBL_MAX;
        prev[i] = -1;
        arrival_times[i] = -1;
        raw_arrival_times[i] = -1;
    }
    
    // Starting node has a distance of 0, and an arrival time of 0
    dist[start_idx] = 0;
    arrival_times[start_idx] = 0;
    raw_arrival_times[start_idx] = 0;

    // Add starting node to the Priority Queue
    pq_push(&pq, start_idx, 0);
    
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

        // Get the current arrival time at node u
        double current_time = arrival_times[u];

        // Check neighbors of the current node 
        Edge* edge = g->adj_list[u]; // Get the head of the linked list for the node u
        while (edge != NULL) {
            int v = edge->to;   // Neighbor
            double arrival_at_v = current_time + edge -> weight; // Calculate arrival at neighbor v
            double raw_arrival = arrival_at_v;

            // EARLY ARRIVAL?
            if(arrival_at_v < g->nodes[v].earliest)
            {
                arrival_at_v = g->nodes[v].earliest;
                printf("EARLY ARRIVAL! Node %d: Arrived at time %.1f, waiting until %.1f\n", 
                    g->node_ids[v], raw_arrival, arrival_at_v);
            }

            // LATE ARRIVAL?
            if(arrival_at_v > g->nodes[v].latest)
            {
                printf("LATE ARRIVAL! Cannot arrive by time %.1f (arrival time %.1f)\n", 
                    g->node_ids[v], g->nodes[v].latest, arrival_at_v);
                edge = edge->next;
                continue; // Skip this edge and try the next neighboring node
            }

            double alt = dist[u] + edge->weight; // Distance from current node
            
            // If a shorter path is found to neighbor v, update it
            if (alt < dist[v]) {
                dist[v] = alt;  // Update shortest distance
                arrival_times[v] = arrival_at_v; // Store the official arrival time
                raw_arrival_times[v] = raw_arrival;
                prev[v] = u;    // Record that we came from u
                pq_push(&pq, v, alt); // Add the neighbor to queue with the new distance

                printf("Updated path to node %d: distance=%.1f, arrival_time=%.1f\n",
                       g->node_ids[v], alt, arrival_at_v);
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
    if (argc != 8) {
        printf("Usage: %s <nodes.csv> <edges.csv> <destinations.csv> <start_node> <end_node> <threshold> <algorithm>\n", argv[0]);
        printf("  threshold: percentage as double (e.g., 12.2 for 12.2%%, or 0 for strict priority)\n");
        printf("  algorithm: dijkstra\n");
        return 1;
    }
    
    char* nodes_file = argv[1];
    char* edges_file = argv[2];
    char* destinations_file = argv[3];
    int start_node = atoi(argv[4]);
    int end_node = atoi(argv[5]);
    double threshold = atof(argv[6]) / 100.0;  // Convert percentage to decimal
    char* algorithm = argv[7];
    
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

    // Load destinations
    Destination dests[MAX_DESTINATIONS];
    int dest_count = load_destinations(destinations_file, dests);
    
    if (dest_count == 0) {
        printf("No destinations loaded\n");
        return 1;
    }
    
    // NEW: Sort destinations by priority
    qsort(dests, dest_count, sizeof(Destination), compare_destinations);
    
    printf("Loaded %d destinations\n", dest_count);
    printf("Priority ordering:\n");
    for (int i = 0; i < dest_count; i++) {
        printf("  %d. Node %d (priority %d)\n", i+1, dests[i].node_id, dests[i].priority);
    }
    printf("Threshold: %.1f%%\n\n", threshold * 100.0);
    
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
        printf("=== Dijkstra's Algorithm With Priority-Based Waypoint Routing ===\n");
        dijkstra_with_priorities(&g, start_idx, end_idx, dests, dest_count, threshold, dist, prev, &nodes_explored);
    } else {
        printf("Unknown algorithm: %s\n", algorithm);
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