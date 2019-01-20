/**
 * Copyright (c) 2015 Copyright Holder All Rights Reserved.
 *
 *
 *
 */

#include "headers/split.h"
#include <string>
#include <vector>

using std::vector;
using std::string;
using std::stringstream;

vector<string> split(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;

    while (std::getline(ss, item, delim))
        elems.push_back(item);
    
    return elems;
}

vector<string> split(const string &s, char delim) {
    vector<string> elems;

    split(s, delim, elems);
    return elems;
}
