
#ifndef FUSIONVIEWERCONTROLLER_H_
#define FUSIONVIEWERCONTROLLER_H_

#include <string>

class FusionViewerController
{
public:
	FusionViewerController(std::string ip, int port);
	~FusionViewerController();

	int sendCpatureImageCmd(std::string name);
	int sendCommand(std::string cmd);

private:
	int init();
	int tryConnect(std::string ip, int port);

	std::string servIP;
	int servPort;

	bool isConnected;
	int sockfd;
};

#endif /* FUSIONVIEWERCONTROLLER_H_ */
