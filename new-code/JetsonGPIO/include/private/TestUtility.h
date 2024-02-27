/*
Copyright (c) 2019-2023, NVIDIA CORPORATION.
Copyright (c) 2019-2023, Jueon Park(pjueon) <bluegbgb@gmail.com>.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#pragma once
#ifndef SIMPLE_UNIT_TEST_H
#define SIMPLE_UNIT_TEST_H

#include <functional>
#include <sstream>
#include <string>
#include <vector>

namespace assert
{
    void is_true(bool b, std::string msg = "");
    void is_false(bool b, std::string msg = "");
    void expect_exception(const std::function<void(void)>& func, std::string msg = "");

    template <class T1, class T2> void are_equal(const T1& expected, const T2& actual, std::string msg = "")
    {
        std::ostringstream s{};
        s << msg << "(expected: " << expected << ", actual: " << actual << ")";
        is_true(expected == actual, s.str());
    }

} // namespace assert

struct TestFunction
{
    std::string name;
    std::function<void(void)> func;

    TestFunction(const std::string& name, const std::function<void(void)>& func) : name(name), func(func) {}
    void execute() const;
};

class TestSuit
{
public:
    virtual ~TestSuit() = default;

    int run();
    void reserve(size_t n);
    void add(const TestFunction& test);

protected:
    virtual void setup() {}
    virtual void on_failed() {}
    std::vector<TestFunction> tests;
};

#endif
