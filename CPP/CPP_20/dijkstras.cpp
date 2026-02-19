#include <iostream>
#include <vector>
#include <queue>
#include <tuple>
#include <unordered_set>
#include <iomanip>
#include <string>

using namespace std;

const long long INF = 999; // 999 for cleaner terminal formatting

// 1. Dynamically prints any graph topology
void printGraphTopology(int n, const vector<vector<pair<int,int>>>& adj, const unordered_set<int>& blocked) {
    cout << "\n=== DYNAMIC GRAPH TOPOLOGY ===\n";
    for (int i = 0; i < n; ++i) {
        cout << "Node " << i << (blocked.count(i) ? " [BLOCKED]" : "          ") << " -> ";
        if (adj[i].empty()) {
            cout << "(None)";
        }
        for (const auto& [neighbor, weight] : adj[i]) {
            cout << "[N:" << neighbor << " W:" << weight << "] ";
        }
        cout << "\n";
    }
    cout << "==============================\n\n";
}

// 2. Dynamically prints the state of both "Universes"
void printDistMatrix(int n, const vector<vector<long long>>& dist) {
    cout << "Current Shortest Known Distances:\n";
    cout << "Node | [0: Unlock Avail] | [1: Unlock Used]\n";
    cout << "-------------------------------------------\n";
    for (int i = 0; i < n; ++i) {
        cout << setw(4) << i << " | ";
        
        if (dist[i][0] == INF) cout << setw(17) << "INF";
        else cout << setw(17) << dist[i][0];
        
        cout << " | ";
        
        if (dist[i][1] == INF) cout << setw(16) << "INF";
        else cout << setw(16) << dist[i][1];
        
        cout << "\n";
    }
    cout << "\n";
}

int fastestRouteDynamic(
    int n,
    const vector<vector<pair<int,int>>>& adj,
    int start,
    int goal,
    const unordered_set<int>& blocked
) {
    printGraphTopology(n, adj, blocked);

    int startUnlockState = blocked.count(start) ? 1 : 0;
    vector<vector<long long>> dist(n, vector<long long>(2, INF));

    cout<<"\nINF:"<<INF;
 
    //     ✅ In short:
    // State lets you track extra conditions per node.
    // priority_queue with greater<State> ensures min-heap behavior for Dijkstra.
    // This is the canonical pattern for “Dijkstra with extra state” problems — exactly what they could ask in your live coding.    
    using State = tuple<long long, int, int>;
    priority_queue<State, vector<State>, greater<State>> pq;


    dist[start][startUnlockState] = 0;
    pq.push({0, start, startUnlockState});

    int step = 1;
    while (!pq.empty()) {
        cout << ">>> Press ENTER to process Step " << step++ << "...";
        cin.get(); // Pauses for user input

        auto [currDist, node, used] = pq.top();
        pq.pop();

        cout << "\n[ACTION] Popped Node " << node 
             << " (Dist: " << currDist 
             << ", State: " << (used ? "[1: Used]" : "[0: Avail]") << ")\n";

        if (currDist > dist[node][used]) {
            cout << "         -> Stale state. Skipping.\n";
            continue;
        }

        if (node == goal) {
            cout << "\n🎯 GOAL REACHED! Shortest Path: " << currDist << "\n";
            printDistMatrix(n, dist);
            return static_cast<int>(currDist);
        }

        for (const auto& [neighbor, weight] : adj[node]) {
            bool isBlocked = blocked.count(neighbor);
            long long newDist = currDist + weight;

            if (!isBlocked) {
                if (newDist < dist[neighbor][used]) {
                    dist[neighbor][used] = newDist;
                    pq.push({newDist, neighbor, used});
                    cout << "         -> Updated Node " << neighbor << " (Normal Path). New Dist: " << newDist << "\n";
                }
            }
            else if (isBlocked && used == 0) {
                if (newDist < dist[neighbor][1]) {
                    dist[neighbor][1] = newDist;
                    pq.push({newDist, neighbor, 1});
                    cout << "         -> ⚡ UNLOCK FIRED on Node " << neighbor << "! New Dist: " << newDist << "\n";
                }
            }
        }
        cout << "\n";
        printDistMatrix(n, dist);
    }

    cout << "\n❌ GOAL UNREACHABLE.\n";
    return -1;
}

int main() {
    // Try changing these values! The visualizer will adapt automatically.
    int n = 6; 
    vector<vector<pair<int, int>>> adj(n);
    
    // Dynamic wiring
    adj[0].push_back({1, 4});
    adj[0].push_back({2, 2});
    adj[1].push_back({3, 3});
    adj[2].push_back({4, 1});
    adj[3].push_back({4, 1});
    
    // Let's add a new dynamic path
    adj[4].push_back({5, 5});
    adj[3].push_back({5, 2});

    unordered_set<int> blocked = {2, 4}; // Now two nodes are blocked!

    fastestRouteDynamic(n, adj, 0, 5, blocked);

    return 0;
}