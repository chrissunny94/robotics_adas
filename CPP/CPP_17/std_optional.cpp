#include <iostream>
#include <optional>
using namespace std;

optional<int> divide(int a, int b) {
    if (b == 0) return nullopt;
    return a / b;
}

int main() {
    auto result = divide(10, 0);
    cout << (result ? to_string(*result) : "Division by zero") << "\n";
}

