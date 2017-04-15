
#ifndef VIVETRACKER_H
#define VIVETRACKER_H

#include "openvr.h"

enum ELogLevel
{
	LogError,
	LogWarning,
	LogInfo,
};

class ViveTracker
{
public:
	ViveTracker();
	~ViveTracker();

	bool InitOpenVR();
	void LogMessage(ELogLevel nLogLevel, const char *pMessage, ...);

	bool getPose(vr::TrackedDevicePose_t &pose);

private:
	vr::IVRSystem *m_pVRSystem;
	int m_trackerIdx;
	vr::TrackedDevicePose_t m_trackerPose;

	vr::TrackedDevicePose_t* m_poseBuf;
};

#endif // VIVETRACKER_H
