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

#include <algorithm>
#include <cctype>
#include <dirent.h>
#include <sstream>
#include <unistd.h>

#include "private/PythonFunctions.h"

using namespace std;

namespace GPIO
{
    bool startswith(const string& s, const string& prefix)
    {
        size_t pre_size = prefix.size();
        if (s.size() < pre_size)
            return false;

        return prefix == s.substr(0, prefix.size());
    }

    std::string lower(const std::string& s)
    {
        auto copied = s;
        transform(copied.begin(), copied.end(), copied.begin(), [](unsigned char c) { return tolower(c); });
        return copied;
    }

    vector<string> split(const string& s, const char d)
    {
        stringstream buffer(s);

        string tmp{};
        vector<string> outputVector{};

        while (getline(buffer, tmp, d))
            outputVector.push_back(tmp);

        return outputVector;
    }

    bool os_access(const string& path, int mode) // os.access
    {
        return access(path.c_str(), mode) == 0;
    }

    vector<string> os_listdir(const string& path) // os.listdir
    {
        DIR* dir{};
        struct dirent* ent{};
        vector<string> outputVector{};

        if ((dir = opendir(path.c_str())) != nullptr)
        {
            while ((ent = readdir(dir)) != nullptr)
            {
                outputVector.emplace_back(ent->d_name);
            }
            closedir(dir);
            return outputVector;
        }
        else
        {
            throw runtime_error("could not open directory: " + path);
        }
    }

    bool os_path_isdir(const std::string& path) // os.path.isdir
    {
        bool exists = false;

        DIR* dir = opendir(path.c_str());
        if (dir != nullptr)
        {
            exists = true;
            closedir(dir);
        }

        return exists;
    }

    bool os_path_exists(const string& path) // os.path.exists
    {
        return os_access(path, F_OK);
    }

    string strip(const string& s)
    {
        int start_idx = 0;
        int total = s.size();
        int end_idx = total - 1;
        for (; start_idx < total; start_idx++)
        {
            if (!isspace(s[start_idx]))
                break;
        }
        if (start_idx == total)
            return "";
        for (; end_idx > start_idx; end_idx--)
        {
            if (!isspace(s[end_idx]))
                break;
        }
        return s.substr(start_idx, end_idx - start_idx + 1);
    }

    bool is_None(const std::string& s) { return s == (std::string)None; }

    bool is_None(int i) { return i == None; }

    template <class T> std::string read_impl(const T& f)
    {
        stringstream buffer{};
        buffer << f.rdbuf();
        return buffer.str();
    }

    std::string read(const std::ifstream& f) { return read_impl(f); }

    std::string read(const std::fstream& f) { return read_impl(f); }

} // namespace GPIO
