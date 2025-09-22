#include <iostream>
#include <type_traits>
using namespace std;

template <typename T>
void print(T val) {
    if constexpr (is_integral_v<T>)   // compile-time check
        cout << "Integral: " << val << "\n";
    else
        cout << "Non-integral: " << val << "\n";
}

int main() {
    print(42);     // Integral
    print(3.14);   // Non-integral
}

