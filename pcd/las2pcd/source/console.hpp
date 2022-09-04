#include "console.h"


std::string GetProgramOptionAsString(
    int argc,
    char** argv,
    const std::string& option,
    const std::string& default_value /* = ""*/) {
    char** itr = std::find(argv, argv + argc, option);
    if (itr != argv + argc && ++itr != argv + argc) {
        return std::string(*itr);
    }
    return default_value;
}

int GetProgramOptionAsInt(int argc,
    char** argv,
    const std::string& option,
    const int default_value /* = 0*/) {
    std::string str = GetProgramOptionAsString(argc, argv, option, "");
    if (str.length() == 0) {
        return default_value;
    }
    char* end;
    errno = 0;
    long l = std::strtol(str.c_str(), &end, 0);
    if ((errno == ERANGE && l == LONG_MAX) || l > INT_MAX) {
        return default_value;
    }
    else if ((errno == ERANGE && l == LONG_MIN) || l < INT_MIN) {
        return default_value;
    }
    else if (*end != '\0') {
        return default_value;
    }
    return (int)l;
}

double GetProgramOptionAsDouble(int argc,
    char** argv,
    const std::string& option,
    const double default_value /* = 0.0*/) {
    std::string str = GetProgramOptionAsString(argc, argv, option, "");
    if (str.length() == 0) {
        return default_value;
    }
    char* end;
    errno = 0;
    double l = std::strtod(str.c_str(), &end);
    if (errno == ERANGE && (l == HUGE_VAL || l == -HUGE_VAL)) {
        return default_value;
    }
    else if (*end != '\0') {
        return default_value;
    }
    return l;
}


bool ProgramOptionExists(int argc, char** argv, const std::string& option) {
    return std::find(argv, argv + argc, option) != argv + argc;
}

bool ProgramOptionExistsAny(int argc,
    char** argv,
    const std::vector<std::string>& options) {
    for (const auto& option : options) {
        if (ProgramOptionExists(argc, argv, option)) {
            return true;
        }
    }
    return false;
}