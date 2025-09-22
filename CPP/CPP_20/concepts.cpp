#include <iostream>
#include <concepts>
using namespace std;

template <integral T>  // constraint
T add(T a, T b) {
    return a + b;
}

int main() {
    cout << add(2, 3) << "\n";   // OK
    // cout << add(2.5, 3.1);   // Error at compile-time
}

