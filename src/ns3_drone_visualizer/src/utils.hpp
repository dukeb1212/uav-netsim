// utils.h
#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <cstring>
#include <array>
#include <sstream>
#include <memory>
#include <ifaddrs.h>
#include <arpa/inet.h>

struct PointLLA {
    double latitude;
    double longitude;
    double altitude;
};

struct PointECEF {
    double x;
    double y;
    double z;
};

struct PointENU {
    double east;
    double north;
    double up;
};

// Function to convert LLA to ECEF
PointECEF llaToEcef(const PointLLA& lla);

// Function to convert ECEF to ENU given a reference point
PointENU ecefToEnu(const PointECEF& ecef, const PointECEF& refEcef, const PointLLA& refLla);

// Function to calculate the 3D distance between two LLA points
double calculateDistance(const PointLLA& point1, const PointLLA& point2);

std::string exec_command(const std::string& cmd);

// Get  device IP address
std::string get_ip_address_in_namespace(const std::string& namespace_name,
    const std::string& interface_name);

#endif  // UTILS_H
