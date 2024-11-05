// utils.cpp
#include "utils.hpp"

// WGS-84 constants
const double a = 6378137.0;  // Semi-major axis
const double f = 1 / 298.257223563;  // Flattening
const double e2 = f * (2 - f);  // First eccentricity squared
const double R = 6371000;  // Mean Earth radius

// Function to convert LLA to ECEF
PointECEF llaToEcef(const PointLLA& lla) {
    double latRad = lla.latitude * M_PI / 180.0;
    double lonRad = lla.longitude * M_PI / 180.0;

    double N = a / std::sqrt(1 - e2 * std::sin(latRad) * std::sin(latRad));
    double x = (N + lla.altitude) * std::cos(latRad) * std::cos(lonRad);
    double y = (N + lla.altitude) * std::cos(latRad) * std::sin(lonRad);
    double z = (N * (1 - e2) + lla.altitude) * std::sin(latRad);

    return { x, y, z };
}

// Function to convert ECEF to ENU given a reference point
PointENU ecefToEnu(const PointECEF& ecef, const PointECEF& refEcef, const PointLLA& refLla) {
    double latRefRad = refLla.latitude * M_PI / 180.0;
    double lonRefRad = refLla.longitude * M_PI / 180.0;

    double dx = ecef.x - refEcef.x;
    double dy = ecef.y - refEcef.y;
    double dz = ecef.z - refEcef.z;

    double east = -std::sin(lonRefRad) * dx + std::cos(lonRefRad) * dy;
    double north = -std::sin(latRefRad) * std::cos(lonRefRad) * dx
        - std::sin(latRefRad) * std::sin(lonRefRad) * dy
        + std::cos(latRefRad) * dz;
    double up = std::cos(latRefRad) * std::cos(lonRefRad) * dx
        + std::cos(latRefRad) * std::sin(lonRefRad) * dy
        + std::sin(latRefRad) * dz;

    return { east, north, up };
}

// Function to calculate the 3D distance between two LLA points
double calculateDistance(const PointLLA& point1, const PointLLA& point2) {
    double lat1Rad = point1.latitude * M_PI / 180.0;
    double lon1Rad = point1.longitude * M_PI / 180.0;
    double lat2Rad = point2.latitude * M_PI / 180.0;
    double lon2Rad = point2.longitude * M_PI / 180.0;

    double deltaLat = lat2Rad - lat1Rad;
    double deltaLon = lon2Rad - lon1Rad;

    double a = std::sin(deltaLat / 2) * std::sin(deltaLat / 2) +
        std::cos(lat1Rad) * std::cos(lat2Rad) *
        std::sin(deltaLon / 2) * std::sin(deltaLon / 2);
    double horizontalDistance = 2 * R * std::asin(std::sqrt(a));

    double deltaAlt = point2.altitude - point1.altitude;

    return std::sqrt(horizontalDistance * horizontalDistance + deltaAlt * deltaAlt);
}

// Function to execute a shell command and get its output
std::string exec_command(const std::string& cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

// Get IP addresses for interfaces inside a network namespace
std::string get_ip_address_in_namespace(const std::string& namespace_name,
    const std::string& interface_name) {
    std::string ip_address = "";

    std::string cmd = "sudo ip netns exec " + namespace_name + " ip -4 addr show " + interface_name + " | grep -oP '(?<=inet\\s)\\d+(\\.\\d+){3}'";
    ip_address = exec_command(cmd);
    if (ip_address.empty()) {
        std::cerr << "No IP found for interface " << interface_name << " in namespace " << namespace_name << std::endl;
    }
    return ip_address;
}


