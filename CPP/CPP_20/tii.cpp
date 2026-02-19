#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <numeric>

using namespace std;

// ================= GRAPH BFS =================
vector<int> bfsShortestPath(
    const unordered_map<int, vector<int>>& graph,
    int source,
    int n)
{
    vector<int> dist(n, -1);
    queue<int> q;

    dist[source] = 0;
    q.push(source);

    while (!q.empty()) {
        int node = q.front();
        q.pop();

        for (const auto& neighbor : graph.at(node)) {
            if (dist[neighbor] == -1) {
                dist[neighbor] = dist[node] + 1;
                q.push(neighbor);
            }
        }
    }

    return dist;
}

// ================= TOP K USING HEAP =================
vector<int> topKElements(const vector<int>& nums, int k)
{
    priority_queue<int, vector<int>, greater<int>> minHeap;

    for (const auto& num : nums) {
        minHeap.push(num);
        if (minHeap.size() > k)
            minHeap.pop();
    }

    vector<int> result;
    while (!minHeap.empty()) {
        result.push_back(minHeap.top());
        minHeap.pop();
    }

    return result;
}

// ================= SLIDING WINDOW =================
int maxSubarraySum(const vector<int>& nums, int windowSize)
{
    if (nums.empty() || windowSize <= 0 || windowSize > nums.size())
        return 0;

    int windowSum = accumulate(nums.begin(), nums.begin() + windowSize, 0);
    int maxSum = windowSum;

    for (size_t i = windowSize; i < nums.size(); ++i) {
        windowSum += nums[i] - nums[i - windowSize];
        maxSum = max(maxSum, windowSum);
    }

    return maxSum;
}

// ================= MAIN =================
int main()
{
    // Sample graph
    unordered_map<int, vector<int>> graph = {
        {0, {1, 2}},
        {1, {0, 3}},
        {2, {0, 3}},
        {3, {1, 2}}
    };

    int n = 4;

    // BFS
    vector<int> distances = bfsShortestPath(graph, 0, n);

    cout << "Shortest distances from node 0:\n";
    for (const auto& d : distances)
        cout << d << " ";
    cout << "\n\n";

    // Node weights
    vector<int> weights = {10, 40, 20, 30};

    // Top K elements
    int k = 2;
    vector<int> topK = topKElements(weights, k);

    cout << "Top K elements:\n";
    for (const auto& val : topK)
        cout << val << " ";
    cout << "\n\n";

    // Sliding window
    int windowSize = 2;
    int maxSum = maxSubarraySum(weights, windowSize);
    cout << "Max subarray sum (window size 2): " << maxSum << "\n\n";

    // Sorting with custom comparator
    sort(weights.begin(), weights.end(),
         [](const int& a, const int& b) {
             return a > b;  // descending
         });

    cout << "Sorted weights (descending):\n";
    for (const auto& w : weights)
        cout << w << " ";
    cout << "\n";

    return 0;
}
