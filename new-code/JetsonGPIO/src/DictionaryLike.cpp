/*
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

#include "private/DictionaryLike.h"
#include "private/PythonFunctions.h"
#include <iostream>
#include <utility>

namespace GPIO
{
    DictionaryLike::DictionaryLike(const std::string& data)
    : data(data), dictionary{}, is_parsed(false), is_dictionary(false)
    {
    }

    std::string DictionaryLike::get(const std::string& key) const
    {
        if (is_parsed == false)
            parse();

        if (is_dictionary)
        {
            auto itr = dictionary.find(key);
            if (itr == dictionary.end())
                return None;

            return itr->second;
        }

        return strip(data);
    }

    void DictionaryLike::parse() const
    {
        is_parsed = true;
        is_dictionary = false;

        auto _data = strip(data);
        if (_data.size() < 2 || _data.front() != '{' || _data.back() != '}')
            return;

        // remove "{}"
        _data = strip(_data.substr(1, _data.size() - 2));

        std::map<std::string, std::string> _dictionary{};

        auto elements = split(_data, ',');
        for (auto& e : elements)
        {
            auto pair = split(e, ':');
            if (pair.size() != 2)
                return;

            auto key = strip(pair[0]);
            auto value = strip(pair[1]);

            if (key.empty() || value.empty())
                return;

            _dictionary[key] = value;
        }

        is_dictionary = true;
        dictionary = std::move(_dictionary);
    }

} // namespace GPIO
