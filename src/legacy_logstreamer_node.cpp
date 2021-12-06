#include "ros/ros.h"
#include "std_msgs/String.h"
#include "oscpp/server.hpp"
#include "oscpp/client.hpp"

#include <thread>
#include <string>
#include <mutex>
#include <iostream>

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "ReportRequestorSet.hpp"

#include "rio_control_node/Motor_Status.h"
#include "rio_control_node/Motor_Control.h"
#include "rio_control_node/Motor_Configuration.h"
#include "rio_control_node/IMU_Data.h"

#define PORT     5809
#define MAXLINE 1024
#define BUFSIZE 1500

std::mutex lockMutex;
ReportRequestorSet mReportRequestorSet;

ros::NodeHandle* node;
bool runThread;

int fd;
bool socketInitSuccess;

size_t constructPacket(void* buffer, size_t size)
{
    OSCPP::Client::Packet packet(buffer, size);
    packet
        .openMessage("/LogData", 12)
        .closeMessage();
    return packet.size();
}

std::string sockaddrToIPStr(sockaddr &ipAddr)
{
    sockaddr_in *tAddr = (sockaddr_in *)(&ipAddr);
    std::string ipStr(inet_ntoa(tAddr->sin_addr));
    return ipStr;
}

void handlePacket(const OSCPP::Server::Packet& packet, sockaddr &recvFromAddr)
{
    if (packet.isBundle()) {
        // Convert to bundle
        OSCPP::Server::Bundle bundle(packet);

        // Get packet stream
        OSCPP::Server::PacketStream packets(bundle.packets());

        // Iterate over all the packets and call handlePacket recursively.
        // Cuidado: Might lead to stack overflow!
        while (!packets.atEnd()) {
            handlePacket(packets.next(), recvFromAddr);
        }
    } else {
        // Convert to message
        OSCPP::Server::Message msg(packet);

        // Get argument stream
        OSCPP::Server::ArgStream args(msg.args());

        // Directly compare message address to string with operator==.
        // For handling larger address spaces you could use e.g. a
        // dispatch table based on std::unordered_map.
        if (msg == "/RegisterRequestor")
		{
            sockaddr_in *tAddr = (sockaddr_in *)(&recvFromAddr);
            tAddr->sin_family = AF_INET;
            tAddr->sin_port = htons(PORT);
            if (tAddr->sin_addr.s_addr == 0)
            {
                //Not valid, stop processing
                return;
            }

            std::lock_guard<std::mutex> lock(lockMutex);
            if (mReportRequestorSet.add(*tAddr))
            {
                ROS_INFO("Added client: %s", sockaddrToIPStr(recvFromAddr).c_str());
            }
        }
		else
		{
            //Unknown messages received
        }
    }
}

bool socket_init()
{
    if (!socketInitSuccess)
    {
        fd = socket(AF_INET,SOCK_DGRAM,0);
        if(fd<0){
            ROS_ERROR("cannot open socket");
            return false;
        }
        socketInitSuccess = true;
    }
    return socketInitSuccess;
}

void run_heartbeat_handler()
{
	char buffer[BUFSIZE];
	memset(buffer, 0, BUFSIZE);

    while (!socket_init()){}

	sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

	// Bind the socket with the server address
    if ( bind(fd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0 )
    {
        ROS_ERROR("bind failed");
    }

	ros::Rate rate(10);
	while (runThread)
	{
		{
			sockaddr recvFromAddr;
			socklen_t recvFromAddrSize;
			int numBytes = recvfrom(fd, &buffer, sizeof(buffer), MSG_WAITALL, &recvFromAddr, &recvFromAddrSize);
			OSCPP::Server::Packet oscPacket(&buffer, numBytes);
            handlePacket(oscPacket, recvFromAddr);
		}
		rate.sleep();
	}
}

void send_data_handler()
{
    char buffer[BUFSIZE];
	memset(buffer, 0, BUFSIZE);

    while (!socket_init()){}

    ros::Rate rate(10);
	while (runThread)
	{
		{
			std::lock_guard<std::mutex> lock(lockMutex);
            size_t packetSize = constructPacket(buffer, BUFSIZE);
            mReportRequestorSet.forEach([&](ReportRequestor* r)
            {
                sockaddr_in s = r->getIpAddr();
                size_t bytesSent = sendto(fd, buffer, packetSize, 0, (struct sockaddr*)&s, sizeof(s));
            });
            mReportRequestorSet.removeExpiredEntries();
		}
		rate.sleep();
	}
}

void motorStatusCallback(const rio_control_node::Motor_Status& msg)
{

}

void imuDataCallback(const rio_control_node::IMU_Data& msg)
{

}

void motorConfigurationCallback(const rio_control_node::Motor_Configuration& msg)
{

}

void motorControlCallback(const rio_control_node::Motor_Control& msg)
{

}


int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "legacy_logstreamer_node");

	ros::NodeHandle n;

	node = &n;

	runThread = true;
	std::thread heartbeatThread(&run_heartbeat_handler);
    std::thread sendDataThread(&send_data_handler);
	
	ros::Subscriber motorStatus = node->subscribe("MotorStatus", 10, motorStatusCallback);
	ros::Subscriber imuData = node->subscribe("IMUData", 10, imuDataCallback);
	ros::Subscriber motorConfiguration = node->subscribe("MotorConfiguration", 10, motorConfigurationCallback);
	ros::Subscriber motorControl = node->subscribe("MotorControl", 10, motorControlCallback);

	ros::spin();
	return 0;
}