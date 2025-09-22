#include <iostream>
#include <map>
using namespace std;

int main() {
    map<int, string> m = {{1, "One"}, {2, "Two"}};

    for (auto& [key, value] : m) {  // Structured binding
        cout << key << " => " << value << "\n";
    }
}

