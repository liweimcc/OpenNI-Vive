
#ifndef VIVETRACKER_H
#define VIVETRACKER_H

#ifndef WITHOUT_OPENVR

#include "openvr.h"

class ViveTracker
{
public:

	enum ELogLevel
	{
		LogError,
		LogWarning,
		LogInfo,
	};
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
#else

class ViveTracker
{
public:
	ViveTracker() {}
	~ViveTracker() {}

	bool InitOpenVR() { return true; }
};

#endif

#endif // VIVETRACKER_H
