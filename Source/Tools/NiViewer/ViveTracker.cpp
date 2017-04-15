
#include "ViveTracker.h"
#include <stdio.h>
#include <stdarg.h> 
#include <stdlib.h>

ViveTracker::ViveTracker()
{
}

ViveTracker::~ViveTracker()
{
}

void ViveTracker::LogMessage(ELogLevel nLogLevel, const char *pMessage, ...)
{
	va_list args;
	char formattedMessage[2048];

	va_start(args, pMessage);
	_vsnprintf_s(formattedMessage, sizeof(formattedMessage), pMessage, args);
	formattedMessage[sizeof(formattedMessage)-1] = 0;
	va_end(args);

	fprintf(stderr, "%s", formattedMessage);
}

bool ViveTracker::InitOpenVR()
{
	// Loading the SteamVR Runtime
	LogMessage(LogInfo, "\nStarting OpenVR...\n");
	vr::EVRInitError eError = vr::VRInitError_None;
	m_pVRSystem = vr::VR_Init(&eError, vr::VRApplication_Other);
	if (eError != vr::VRInitError_None)
	{
		m_pVRSystem = nullptr;
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "Unable to init VR runtime: %s", vr::VR_GetVRInitErrorAsSymbol(eError));
		LogMessage(LogError, "%s\n", buf);
		return false;
	}
	else
	{
		char systemName[1024];
		char serialNumber[1024];

		for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++)
		{
			vr::ETrackedDeviceClass c = m_pVRSystem->GetTrackedDeviceClass(i);

			if (c == vr::TrackedDeviceClass_GenericTracker)
			{
				m_trackerIdx = i;
				m_pVRSystem->GetStringTrackedDeviceProperty(i, vr::Prop_TrackingSystemName_String, systemName, sizeof(systemName));
				m_pVRSystem->GetStringTrackedDeviceProperty(i, vr::Prop_SerialNumber_String, serialNumber, sizeof(serialNumber));
			}
		}

		LogMessage(LogInfo, "VR Tracker: %s %s\n", systemName, serialNumber);
	}

	m_poseBuf = new vr::TrackedDevicePose_t[vr::k_unMaxTrackedDeviceCount];

	return true;
}

bool ViveTracker::getPose(vr::TrackedDevicePose_t &pose)
{
	m_pVRSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, m_poseBuf, vr::k_unMaxTrackedDeviceCount);

	pose = m_poseBuf[m_trackerIdx];

	return false;
}
