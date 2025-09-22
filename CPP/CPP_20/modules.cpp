// math.ixx (module file)
export module math;
export int add(int a, int b) { return a + b; }

// main.cpp
import math;
#include <iostream>
using namespace std;

int main() {
    cout << add(2, 3) << "\n";  // 5
}

