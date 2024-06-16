#include <cppext_cyclic_stack.hpp>

#include <catch2/catch_test_macros.hpp>

TEST_CASE("cyclic stack", "[cppext][container]")
{
    cppext::cyclic_stack<int> stack(3);

    CHECK(stack.empty());

    stack.push(1);
    stack.push(2);
    stack.push(3);

    CHECK_FALSE(stack.empty());

    CHECK(stack.top() == 1);

    stack.cycle();
    CHECK(stack.top() == 2);

    stack.cycle();
    CHECK(stack.top() == 3);

    stack.cycle();
    CHECK(stack.top() == 1);

    stack.cycle();
    CHECK(stack.top() == 2);

    stack.pop();
    CHECK(stack.top() == 3);
}
