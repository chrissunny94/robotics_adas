#include <iostream>
#include <vector>
#include <algorithm>
#include <execution>
using namespace std;

int main() {
    vector<int> v = {5, 2, 9, 1, 7};
    sort(execution::par, v.begin(), v.end()); // parallel sort
    for (int x : v) cout << x << " ";
}

