#ifndef FILE_UTIL_HH
#define FILE_UTIL_HH

#include <cstring>
#include <cassert>
#include <iostream>
#include <vector>
#include <string>
#include <zlib.h>
#include "string_util.hh"

class file_or_gz {
private:
    bool gzipped;
    union { FILE *in; gzFile gz_in; };
    char *line;
    const size_t max_line_size;

public:
    file_or_gz(const std::string &filename, size_t mls=10000)
        : gzipped(filename.size() > 3 &&
                  filename.substr(filename.size() - 3) == std::string(".gz")),
          max_line_size(mls)
    {
        if (gzipped) {
            gz_in = gzopen(filename.c_str(), "r");
        } else {
            in = filename != "-" ? fopen(filename.c_str(), "r") : stdin;
        }
        if (gzipped ? gz_in == Z_NULL : in  == nullptr) {
            perror(filename.c_str());
            exit(EXIT_FAILURE);
        }
        line = new char[max_line_size];
    }

    ~file_or_gz() {
        delete[] line;
    }

    // returns "" at end of file
    std::string get_line() {
        bool eof = (gzipped ? gzgets(gz_in, line, max_line_size)
                            : fgets(line, max_line_size, in)) == NULL;
        if (eof) return std::string("");
        else return std::string(line);
    }

    void close() {
        if (gzipped) {
            gzclose(gz_in);
        } else {
            if(in != stdin) fclose(in);
        }
    }

};

std::vector<std::vector<std::string> >
read_tuples(const std::string &filename, const size_t ncols) {
    std::vector<std::vector<std::string> > rows;
    file_or_gz in(filename);
    while(true) {
        std::string line = rtrim(in.get_line());
        if (line == "") break;
        auto v = split(line, ' ');
        if (v.size() != ncols) {
            std::cerr << "Wrong ncols: expected " << ncols << ", got "
                      << v.size() << " for '" << line << "' in " << filename
                      << ".\n";
            assert(v.size() == ncols);
        }
        rows.push_back(v);
    }
    return rows;
}

std::vector<std::vector<std::string> >
read_csv(const std::string &filename, const size_t ncol, ...) { // ... = column names
    std::vector<std::vector<std::string> > rows;
    file_or_gz in(filename);
    bool first = true;

    std::vector<std::string> colnames(ncol);
    va_list args;
    va_start(args, ncol);
    for (size_t j = 0; j < ncol; ++j) {
        colnames[j] = va_arg(args, char *);
    }
    va_end(args);

    std::vector<size_t> cols(ncol, -1);
    while(true) {
        std::string line = rtrim(in.get_line());
        if (line == "") break;
        if (first) {
            first = false;
            int i = 0;
            for (auto s : split(line, ',')) {
                for (size_t j = 0; j < ncol; ++j) {
                    if (s == colnames[j]) cols[j] = i;
                }
                ++i;
            }
            for (size_t j = 0; j < ncol; ++j) {
                if (cols[j] < 0)
                    throw std::invalid_argument("Missing column: " + colnames[j]
                                                + " in:\n'" + line + "'\n"
                                                + filename);
            }
        } else {
            auto v = split(line, ',');
            std::vector<std::string> r(ncol);
            for (size_t j = 0; j < ncol; ++j) {
                assert(cols[j] < v.size());
                r[j] = v[cols[j]];
            }
            rows.push_back(r);
        }
    }
    in.close();

    return rows;
}


#endif // FILE_UTIL_HH
