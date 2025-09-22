#include <iostream>
#include <coroutine>
using namespace std;

struct Generator {
    struct promise_type;
    using handle_type = coroutine_handle<promise_type>;
    struct promise_type {
        int current_value;
        auto get_return_object() { return Generator{handle_type::from_promise(*this)}; }
        auto initial_suspend() { return suspend_always{}; }
        auto final_suspend() noexcept { return suspend_always{}; }
        void return_void() {}
        auto yield_value(int value) { current_value = value; return suspend_always{}; }
        void unhandled_exception() { exit(1); }
    };
    handle_type h;
    Generator(handle_type h) : h(h) {}
    ~Generator() { h.destroy(); }
    bool move_next() { return !h.done() && (h.resume(), true); }
    int value() { return h.promise().current_value; }
};

Generator counter(int n) {
    for (int i = 1; i <= n; i++) co_yield i;
}

int main() {
    auto gen = counter(5);
    while (gen.move_next()) {
        cout << gen.value() << " ";  // 1 2 3 4 5
    }
}

