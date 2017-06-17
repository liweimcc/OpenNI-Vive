
#ifndef FUSIONVIEWERCONTROLLER_H_
#define FUSIONVIEWERCONTROLLER_H_

#include <string>

class FusionViewerController
{
public:
	FusionViewerController(std::string ip, int port);
	~FusionViewerController();

	int sendCommand(std::string cmd);
	//void sendCommand(std::string cmd, std::string value);

private:
	int init();
	int tryConnect(std::string ip, int port);

	std::string servIP;
	int servPort;

	bool isConnected;
	int sockfd;
};

#endif /* FUSIONVIEWERCONTROLLER_H_ */
