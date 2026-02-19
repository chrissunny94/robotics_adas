#include <vector>
#include <queue>
#include <unordered_set>
#include <limits>

using namespace std;

int fastestRoute(
    int n,
    const vector<vector<pair<int,int>>>& adj,
    int start,
    int goal,
    const unordered_set<int>& blocked
) {
    // Edge case: start or goal blocked
    if (blocked.count(start) || blocked.count(goal))
        return -1;

    // Distance array
    vector<long long> dist(n, numeric_limits<long long>::max());
    dist[start] = 0;

    // Min heap: {distance, node}
    priority_queue<
        pair<long long, int>,
        vector<pair<long long, int>>,
        greater<pair<long long, int>>
    > pq;

    pq.push({0, start});

    while (!pq.empty()) {
        auto [currDist, node] = pq.top();
        pq.pop();

        // Skip outdated entries
        if (currDist > dist[node])
            continue;

        if (node == goal)
            return currDist;

        // Explore neighbors
        for (const auto& [neighbor, weight] : adj[node]) {

            if (blocked.count(neighbor))
                continue;

            long long newDist = currDist + weight;

            if (newDist < dist[neighbor]) {
                dist[neighbor] = newDist;
                pq.push({newDist, neighbor});
            }
        }
    }

    return -1; // unreachable
}
