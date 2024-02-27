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

#include "private/ModelUtility.h"
#include "private/Model.h"
#include "private/PythonFunctions.h"
#include <stdexcept>

namespace GPIO
{
    constexpr auto number_of_models = static_cast<int>(sizeof(MODEL_NAMES) / sizeof(Model));

    std::string model_name(Model model)
    {
        int idx = static_cast<int>(model);
        if (idx < 0 || idx >= number_of_models)
            throw std::runtime_error("model_name error");

        return MODEL_NAMES[idx];
    }

    int model_name_index(const std::string& name)
    {
        auto _name = strip(name);

        for (int idx = 0; idx < number_of_models; idx++)
        {
            if (_name == MODEL_NAMES[idx])
                return idx;
        }

        return None;
    }

    Model index_to_model(int idx)
    {
        if (idx < 0 || idx >= number_of_models)
            throw std::runtime_error("index_to_model error");

        return static_cast<Model>(idx);
    }

    Model name_to_model(const std::string& name) { return index_to_model(model_name_index(name)); }

} // namespace GPIO
