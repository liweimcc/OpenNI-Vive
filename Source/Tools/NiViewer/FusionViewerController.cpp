
#include "FusionViewerController.h"

#ifdef _WIN32
#  define far
//#  include <Windows.h>
#  include <WinSock2.h>
#  include <stdint.h> // portable: uint64_t   MSVC: __int64 
#  include <time.h>

//#define inet_pton InetPton
#include <Ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")

#else
#  include <sys/socket.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#endif


FusionViewerController::FusionViewerController(std::string ip, int port)
{
	sockfd = -1;
	isConnected = false;

	servIP = ip;
	servPort = port;
	
	init();
	tryConnect(servIP, servPort);
}

FusionViewerController::~FusionViewerController()
{

}

int FusionViewerController::sendCommand(std::string cmd)
{
	return send(sockfd, cmd.c_str(), cmd.length(), 0);
}
	
int FusionViewerController::init()
{
#ifdef WIN32
	WSADATA wsaData;
	WORD sockVersion = MAKEWORD(2, 2);
	WSAStartup(sockVersion, &wsaData);
#endif

	sockfd = socket(AF_INET, SOCK_STREAM, 0);

	if ( sockfd <= 0)
	{
		printf("\nSocket create fail \n");
		return -1;
	}

	return 1;
}

int FusionViewerController::tryConnect(std::string ip, int port)
{
	struct sockaddr_in addr;

	memset(&addr, '0', sizeof(addr));

	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);

#if _WIN32
	std::wstring wip(ip.begin(), ip.end());
	if (InetPton(AF_INET, (PCWSTR)wip.c_str(), &addr.sin_addr) <= 0)
#else
	if (inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr) <= 0)
#endif
	{
		printf("\nInvalid address/ Address not supported \n");
		return -1;
	}

	if (connect(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		printf("\nConnection Failed \n");
		return -1;
	}

}
