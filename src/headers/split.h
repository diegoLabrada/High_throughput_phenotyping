/**
 * Copyright (c) 2015 Copyright Holder All Rights Reserved.
 *
 *
 */
#ifndef SPLIT_H
#define SPLIT_H

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

using std::vector;
using std::string;

// splits a string into a vector of strings by a delimiter.
vector<string> split(const string &s, char delim, vector<string> &elems);
vector<string> split(const string &s, char delim);

#endif // SPLIT_H
