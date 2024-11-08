#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/aodv-module.h"
#include "ns3/internet-apps-module.h"

#include "rclcpp/rclcpp.hpp"
#include "drone.hpp"
#include "ground_station.hpp"
#include "utils.hpp"

// Network device: 2 (1 ground station, 1 drone)
static const int COUNT = 2;

void set_up_ns3(ns3::NodeContainer& ns3_nodes) {
	ns3::GlobalValue::Bind("SimulatorImplementationType",
		ns3::StringValue("ns3::RealtimeSimulatorImpl"));

	ns3::GlobalValue::Bind("ChecksumEnabled", ns3::BooleanValue(true));

	/*ns3::Address ground_station_ip_;
	ns3::Address drone_ip_;
	ns3::Address test_ip_;
	ns3::Address test1_ip_;*/

	// Create ns3 nodes
	ns3_nodes.Create(COUNT);

	// Set up wifi
	ns3::WifiHelper wifi;
	wifi.SetStandard(ns3::WIFI_STANDARD_80211a);
	wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
		"DataMode", ns3::StringValue("OfdmRate54Mbps"));

	// choose ad-hoc Wifi network
	ns3::WifiMacHelper wifiMac;
	wifiMac.SetType("ns3::AdhocWifiMac");

	// Physical layer
	ns3::YansWifiChannelHelper wifiChannel(ns3::YansWifiChannelHelper::Default());
	ns3::YansWifiPhyHelper wifiPhy;
	wifiPhy.SetChannel(wifiChannel.Create());

	// Install wifi to ns3 nodes
	ns3::NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, ns3_nodes);

	// Set ground station and drone locations
	ns3::MobilityHelper mobility;
	ns3::Ptr<ns3::ListPositionAllocator> positionAlloc = ns3::CreateObject<ns3::ListPositionAllocator>();
	positionAlloc->Add(ns3::Vector(0.0, 0.0, 0.0)); // Ground station position
	positionAlloc->Add(ns3::Vector(5.0, 0.0, 0.0)); // Drone position
	/*positionAlloc->Add(ns3::Vector(0.0, 0.0, 0.0));
	positionAlloc->Add(ns3::Vector(0.0, 0.0, 0.0));*/
	mobility.SetPositionAllocator(positionAlloc);
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.Install(ns3_nodes);

	// Install Internet stack
	//ns3::InternetStackHelper internet;

	/*Ipv4NixVectorHelper nixRouting;
	internet.SetRoutingHelper(nixRouting);*/
	/*ns3::AodvHelper aodv;
	internet.SetRoutingHelper(aodv);
	internet.Install(ns3_nodes);*/

	

	// Assign IP addresses
	/*ns3::Ipv4AddressHelper ipv4;
	ipv4.SetBase("10.0.0.0", "255.255.255.0");
	ns3::Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);*/

	/*ground_station_ip_ = interfaces.GetAddress(0, 0);
	drone_ip_ = interfaces.GetAddress(1, 0);
	test_ip_ = interfaces.GetAddress(2, 0);
	test1_ip_ = interfaces.GetAddress(3, 0);*/

	// Set up tap bridge for tap devices
	ns3::TapBridgeHelper tapBridge;
	tapBridge.SetAttribute("Mode", ns3::StringValue("UseLocal"));
	tapBridge.SetAttribute("DeviceName", ns3::StringValue("tap_uav1")); // Tap device of ground station
	tapBridge.Install(ns3_nodes.Get(0), devices.Get(0));

	tapBridge.SetAttribute("DeviceName", ns3::StringValue("tap_uav2")); // Tap device of drone
	tapBridge.Install(ns3_nodes.Get(1), devices.Get(1));

	/*ns3::PingHelper pingHelper(test_ip_, test1_ip_);
	pingHelper.SetAttribute("Interval", ns3::TimeValue(ns3::Seconds(1)));
	pingHelper.SetAttribute("Count", ns3::UintegerValue(10));
	pingHelper.SetAttribute("Size", ns3::UintegerValue(32));
	ns3::ApplicationContainer apps = pingHelper.Install(ns3_nodes.Get(3));
	apps.Start(ns3::Seconds(1));
	apps.Stop(ns3::Seconds(50));*/
}

void drone_thread_function(ns3::Ptr<ns3::Node> ns3_node_ptr) {

	// Create the ROS2 node
	std::shared_ptr<rclcpp::Node> rclcpp_node = rclcpp::Node::make_shared(
		"drone_node");

	// create the drone, give the rclcpp_node and ns3_compnent nodes to it
	std::unique_ptr<drone::Drone> drone_ =
		std::make_unique<drone::Drone>(rclcpp_node,
			ns3_node_ptr);

	/*ns3_node_ptr->TraceConnectWithoutContext("PingReply",
		ns3::MakeCallback(&drone::Drone::onPingReplyReceived, drone_.get()));

	ns3::Simulator::Schedule(ns3::Seconds(1), &drone::Drone::checkConnection, drone_.get());
	ns3::Simulator::Schedule(ns3::Seconds(1.5), &drone::Drone::increasePingFailures, drone_.get());*/

	// Start the rclcpp node
	std::cout << "Starting drone in thread.\n";
	rclcpp::spin(rclcpp_node);
	rclcpp::shutdown();
	std::cout << "Stopped drone in thread.\n";
}

void ground_station_thread_function(ns3::Ptr<ns3::Node> ns3_node_ptr) {
	// Create the ROS2 node
	std::shared_ptr<rclcpp::Node> rclcpp_node = rclcpp::Node::make_shared(
		"ground_station_node");

	// create the ground station, give the rclcpp_node and ns3_compnent nodes to it
	std::unique_ptr<ground_station::GroundStation> ground_station_ =
		std::make_unique<ground_station::GroundStation>(rclcpp_node,
			ns3_node_ptr);

	// Start the rclcpp node in its own thread to accept user input
	std::thread rclcpp_spin_thread([&]() {
		std::cout << "Starting ground station in thread.\n";
		rclcpp::spin(rclcpp_node);
		});

	// Move input handling to a separate loop
	std::string user_input;
	while (true) {
		std::cout << "Enter command (takeoff/landing): ";
		std::getline(std::cin, user_input);

		if (user_input == "takeoff" || user_input == "landing") {
			ground_station_->sendCommand(user_input);
		}
		else if (user_input == "exit") {
			break;
		}
		else {
			std::cout << "Invalid command. Try again." << std::endl;
		}
	}

	rclcpp::shutdown();
	std::cout << "Stopped ground station in thread.\n";
}

int main(int argc, char* argv[]) {
	// information
	std::cout << "Starting ns-3 Wifi simulator and ns-3 feedback robot.\n"
		<< "Press Ctrl-C twice to stop both.\n";

	// Force flush of the stdout buffer.
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	// Initialize ROS2 
	rclcpp::init(argc, argv);

	// create the ns-3 nodes container
	ns3::NodeContainer ns3_nodes;

	// set up ns-3
	set_up_ns3(ns3_nodes);

	ns3::Ptr<ns3::Node> drone_node = ns3_nodes.Get(1);
	// start the drone as a second thread
	std::thread drone_thread(drone_thread_function, drone_node);

	ns3::Ptr<ns3::Node> ground_station_node = ns3_nodes.Get(0);
	// start the ground station as a third thread
	std::thread ground_station_thread(ground_station_thread_function, ground_station_node);

	// set to run for one year
	ns3::Simulator::Stop(ns3::Seconds(60 * 60 * 24 * 365.));

	// run until Ctrl-C
	std::cout << "Starting ns-3 Wifi simulator in main.\n";
	ns3::Simulator::Run();

	ns3::Simulator::Destroy();
	drone_thread.join(); // gracefully let the drone thread stop
	std::cout << "Stopped ns-3 Wifi simulator in main.\n";
	std::cout << "Done.\n";
	return 0;
}