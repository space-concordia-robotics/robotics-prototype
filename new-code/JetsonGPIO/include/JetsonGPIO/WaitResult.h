/*
Copyright (c) 2019-2023, Jueon Park(pjueon) <bluegbgb@gmail.com>.
Copyright (c) 2021-2023, Adam Rasburn <blackforestcheesecake@protonmail.ch>.

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
#ifndef WAIT_RESULT_H
#define WAIT_RESULT_H

#include <string>

namespace GPIO
{
    class WaitResult
    {
    public:
        WaitResult(const std::string& channel);
        WaitResult(const WaitResult&) = default;
        WaitResult(WaitResult&&) = default;
        WaitResult& operator=(const WaitResult&) = default;
        WaitResult& operator=(WaitResult&&) = default;

        inline const std::string& channel() const { return _channel; }
        bool is_event_detected() const;
        inline operator bool() const { return is_event_detected(); }

    private:
        std::string _channel;
    };

} // namespace GPIO

#endif
