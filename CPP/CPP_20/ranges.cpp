#include <iostream>
#include <vector>
#include <ranges>
using namespace std;

int main() {
    vector<int> v = {1, 2, 3, 4, 5, 6};

    for (int x : v | ranges::views::filter([](int n){ return n % 2 == 0; })
                   | ranges::views::transform([](int n){ return n*n; }))
    {
        cout << x << " ";  // 4 16 36
    }
}

