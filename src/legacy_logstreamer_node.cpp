#include "ros/ros.h"
#include "std_msgs/String.h"
#include "oscpp/server.hpp"

#include <thread>
#include <string>
#include <mutex>
#include <iostream>

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "ReportRequestor.hpp"

#define PORT     5809
#define MAXLINE 1024
#define HOSTNAME "10.0.2.104"

std::mutex lockMutex;

ros::NodeHandle* node;
bool runThread;

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
        if (msg == "/ReportRequestor")
		{
            sockaddr_in *tAddr = (sockaddr_in *)(&recvFromAddr);
            std::string ipAddr(inet_ntoa(tAddr->sin_addr));
            if (ipAddr == "0.0.0.0")
            {
                //Not valid, stop processing
                return;
            }
            std::cout << msg.address() << " ";
            std::cout << ipAddr << " ";
            std::cout << std::endl;
        }
		else
		{
            //Unknown messages received
        }
    }
}

void run_heartbeat_handler()
{
	char buffer[1500];
	memset(buffer, 0, 1500);

	sockaddr_in servaddr;
    int fd = socket(AF_INET,SOCK_DGRAM,0);
    if(fd<0){
        ROS_ERROR("cannot open socket");
    }
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
			std::lock_guard<std::mutex> lock(lockMutex);
			sockaddr recvFromAddr;
			socklen_t recvFromAddrSize;
			int numBytes = recvfrom(fd, &buffer, sizeof(buffer), MSG_WAITALL, &recvFromAddr, &recvFromAddrSize);
			OSCPP::Server::Packet oscPacket(&buffer, numBytes);
            handlePacket(oscPacket, recvFromAddr);
		}
		rate.sleep();
	}
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
	

	ros::spin();
	return 0;
}