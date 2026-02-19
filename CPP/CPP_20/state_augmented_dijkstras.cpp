#include <vector>
#include <queue>
#include <limits>

using namespace std;

int fastestRouteWithBoost(
    int n,
    const vector<vector<pair<int,int>>>& adj,
    int start,
    int goal
) {
    const long long INF = numeric_limits<long long>::max();

    // dist[node][boost_used]
    vector<vector<long long>> dist(n, vector<long long>(2, INF));
    
    // Min-heap: {distance, node, boost_used}
    using State = tuple<long long, int, int>;
    priority_queue<State, vector<State>, greater<State>> pq;

    dist[start][0] = 0;
    pq.push({0, start, 0});

    while (!pq.empty()) {
        auto [currDist, node, used] = pq.top();
        pq.pop();

        if (currDist > dist[node][used])
            continue;

        for (const auto& [neighbor, weight] : adj[node]) {

            // Option 1: go normally
            long long newDist = currDist + weight;
            if (newDist < dist[neighbor][used]) {
                dist[neighbor][used] = newDist;
                pq.push({newDist, neighbor, used});
            }

            // Option 2: use boost if not used yet
            if (used == 0) {
                long long boostedDist = currDist;  // cost becomes 0
                if (boostedDist < dist[neighbor][1]) {
                    dist[neighbor][1] = boostedDist;
                    pq.push({boostedDist, neighbor, 1});
                }
            }
        }
    }

    long long result = min(dist[goal][0], dist[goal][1]);
    return (result == INF) ? -1 : static_cast<int>(result);
}
