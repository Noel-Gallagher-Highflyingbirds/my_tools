#pragma once
#include <iostream>
#include <string>
#include <vector>

std::string GetProgramOptionAsString(int argc,
    char** argv,
    const std::string& option,
    const std::string& default_value = "");

int GetProgramOptionAsInt(int argc,
    char** argv,
    const std::string& option,
    const int default_value = 0);

double GetProgramOptionAsDouble(int argc,
    char** argv,
    const std::string& option,
    const double default_value = 0.0);


bool ProgramOptionExists(int argc, char** argv, const std::string& option);

bool ProgramOptionExistsAny(int argc,
    char** argv,
    const std::vector<std::string>& options);



