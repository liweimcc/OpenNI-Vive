/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
// --------------------------------
// Includes
// --------------------------------
#include <XnOS.h>
#include <fstream>
#include <sstream>
#include "Capture.h"
#include "Device.h"
#include "Draw.h"

#include "FusionViewerController.h"
#include "ViveTracker.h"
#include "opencv2/opencv.hpp"

#if (XN_PLATFORM == XN_PLATFORM_WIN32)
#include <Commdlg.h>
#endif

void initPhoneCtrler();
void capturePhoneImage(std::string name);

// --------------------------------
// Defines
// --------------------------------
#define CAPTURED_FRAMES_DIR_NAME "CapturedFrames"

// --------------------------------
// Types
// --------------------------------
typedef enum
{
	NOT_CAPTURING,
	SHOULD_CAPTURE,
	CAPTURING,
} CapturingState;

typedef enum
{
	CAPTURE_DEPTH_STREAM,
	CAPTURE_COLOR_STREAM,
	CAPTURE_IR_STREAM,
	CAPTURE_STREAM_COUNT
} CaptureSourceType;

typedef enum
{
	STREAM_CAPTURE_LOSSLESS = FALSE,
	STREAM_CAPTURE_LOSSY = TRUE,
	STREAM_DONT_CAPTURE,
} StreamCaptureType;

typedef struct StreamCapturingData
{
	StreamCaptureType captureType;
	const char* name;
	bool bRecording;
	openni::VideoFrameRef& (*getFrameFunc)();
	openni::VideoStream&  (*getStream)();	
	bool (*isStreamOn)();
	int startFrame;
} StreamCapturingData;

typedef struct CapturingData
{
	StreamCapturingData streams[CAPTURE_STREAM_COUNT];
	openni::Recorder recorder;
	char csFileName[256];
	int nStartOn; // time to start, in seconds
	bool bSkipFirstFrame;
	CapturingState State;
	int nCapturedFrameUniqueID;
	char csDisplayMessage[500];
} CapturingData;

// --------------------------------
// Static Global Variables
// --------------------------------
CapturingData g_Capture;

ViveTracker g_tracker;

DeviceParameter g_DepthCapturing;
DeviceParameter g_ColorCapturing;
DeviceParameter g_IRCapturing;

std::ofstream g_poseLogFile;
std::stringstream g_poseLogString;

FusionViewerController* phoneCtrler = NULL;
static bool isNeedPhoneCapture = false;

void takePhonePicture()
{
	if (isNeedPhoneCapture == false)
	{
		isNeedPhoneCapture = true;
	}
}

// --------------------------------
// Code
// --------------------------------
void captureInit()
{
	// init phone connect
	//initPhoneCtrler();

	// init tracker first
	g_tracker.InitOpenVR();

	// Depth Formats
	int nIndex = 0;

	g_DepthCapturing.pValues[nIndex] = STREAM_CAPTURE_LOSSLESS;
	g_DepthCapturing.pValueToName[nIndex] = "Lossless";
	nIndex++;

	g_DepthCapturing.pValues[nIndex] = STREAM_DONT_CAPTURE;
	g_DepthCapturing.pValueToName[nIndex] = "Don't Capture";
	nIndex++;

	g_DepthCapturing.nValuesCount = nIndex;

	// Color Formats
	nIndex = 0;

	g_ColorCapturing.pValues[nIndex] = STREAM_CAPTURE_LOSSLESS;
	g_ColorCapturing.pValueToName[nIndex] = "Lossless";
	nIndex++;

	g_ColorCapturing.pValues[nIndex] = STREAM_CAPTURE_LOSSY;
	g_ColorCapturing.pValueToName[nIndex] = "Lossy";
	nIndex++;

	g_ColorCapturing.pValues[nIndex] = STREAM_DONT_CAPTURE;
	g_ColorCapturing.pValueToName[nIndex] = "Don't Capture";
	nIndex++;

	g_ColorCapturing.nValuesCount = nIndex;

	// IR Formats
	nIndex = 0;

	g_IRCapturing.pValues[nIndex] = STREAM_CAPTURE_LOSSLESS;
	g_IRCapturing.pValueToName[nIndex] = "Lossless";
	nIndex++;

	g_IRCapturing.pValues[nIndex] = STREAM_DONT_CAPTURE;
	g_IRCapturing.pValueToName[nIndex] = "Don't Capture";
	nIndex++;

	g_IRCapturing.nValuesCount = nIndex;

	// Init
	g_Capture.csFileName[0] = 0;
	g_Capture.State = NOT_CAPTURING;
	g_Capture.nCapturedFrameUniqueID = 0;
	g_Capture.csDisplayMessage[0] = '\0';
	g_Capture.bSkipFirstFrame = false;

	g_Capture.streams[CAPTURE_DEPTH_STREAM].captureType = STREAM_CAPTURE_LOSSLESS;
	g_Capture.streams[CAPTURE_DEPTH_STREAM].name = "Depth";
	g_Capture.streams[CAPTURE_DEPTH_STREAM].getFrameFunc = getDepthFrame;
	g_Capture.streams[CAPTURE_DEPTH_STREAM].getStream = getDepthStream;
	g_Capture.streams[CAPTURE_DEPTH_STREAM].isStreamOn = isDepthOn;
	g_Capture.streams[CAPTURE_COLOR_STREAM].captureType = STREAM_CAPTURE_LOSSY;
	g_Capture.streams[CAPTURE_COLOR_STREAM].name = "Color";
	g_Capture.streams[CAPTURE_COLOR_STREAM].getFrameFunc = getColorFrame;
	g_Capture.streams[CAPTURE_COLOR_STREAM].getStream = getColorStream;
	g_Capture.streams[CAPTURE_COLOR_STREAM].isStreamOn = isColorOn;
	g_Capture.streams[CAPTURE_IR_STREAM].captureType = STREAM_CAPTURE_LOSSLESS;
	g_Capture.streams[CAPTURE_IR_STREAM].name = "IR";
	g_Capture.streams[CAPTURE_IR_STREAM].getFrameFunc = getIRFrame;
	g_Capture.streams[CAPTURE_IR_STREAM].getStream = getIRStream;
	g_Capture.streams[CAPTURE_IR_STREAM].isStreamOn = isIROn;
}

bool isCapturing()
{
	return (g_Capture.State != NOT_CAPTURING);
}

void captureBrowse(int)
{
#if (0) && (ONI_PLATFORM == ONI_PLATFORM_WIN32)
    OPENFILENAME ofn  = { 0 };
    ofn.lStructSize   = sizeof(ofn);
    ofn.lpstrFilter   = TEXT("Oni Files (*.oni)\0*.oni\0");
    ofn.nFilterIndex  = 1;
    ofn.lpstrFile     = g_Capture.csFileName;
    ofn.nMaxFile      = sizeof(g_Capture.csFileName);
    ofn.lpstrTitle    = TEXT("Capture to...");
    ofn.lpstrDefExt   = TEXT("oni");
    ofn.Flags         = OFN_EXPLORER | OFN_NOCHANGEDIR;
    BOOL gotFileName = GetSaveFileName(&ofn);

    if (gotFileName)
    {
		if (g_Capture.csFileName[0] != 0)
		{
			if (strstr(g_Capture.csFileName, ".oni") == NULL)
			{
				strcat(g_Capture.csFileName, ".oni");
			}
		}
	}
#else
    // Set capture file to defaults.
    strcpy(g_Capture.csFileName, "./Captured.oni");
#endif // ONI_PLATFORM_WIN32

	// as we waited for user input, it's probably better to discard first frame (especially if an accumulating
	// stream is on, like audio).
	g_Capture.bSkipFirstFrame = true;
}

void captureStart(int nDelay)
{
    captureBrowse(0);

    // On some platforms a user can cancel capturing. Whenever he cancels
    // capturing, the gs_filePath[0] remains empty.
    if ('\0' == g_Capture.csFileName[0])
    {
        return;
    }

    openni::Status rc = g_Capture.recorder.create(g_Capture.csFileName);
	if (rc != openni::STATUS_OK)
	{
		displayError("Failed to create recorder!");
		return;
	}

	g_poseLogString.clear();

	XnUInt64 nNow;
	xnOSGetTimeStamp(&nNow);
	nNow /= 1000;

	g_Capture.nStartOn = (XnUInt32)nNow + nDelay;
	g_Capture.State = SHOULD_CAPTURE;
}

void captureRestart(int)
{
    captureStop(0);
    captureStart(0);
}

void captureStop(int)
{
	g_poseLogFile.open("PoseLog.txt", std::ofstream::out);

	if (g_poseLogFile.is_open())
	{
		g_poseLogFile.write(g_poseLogString.str().data(), g_poseLogString.str().length());
		g_poseLogFile.close();
	}

    if (g_Capture.recorder.isValid())
    {
        g_Capture.recorder.destroy();
		g_Capture.State = NOT_CAPTURING;
    }
}

#define START_CAPTURE_CHECK_RC(rc, what)												\
	if (nRetVal != XN_STATUS_OK)														\
	{																					\
		displayError("Failed to %s: %s\n", what, openni::OpenNI::getExtendedError());	\
		g_Capture.recorder.destroy();													\
		g_Capture.State = NOT_CAPTURING;												\
		return;																			\
	}

void captureRun()
{
	XnStatus nRetVal = XN_STATUS_OK;

	if (g_Capture.State == SHOULD_CAPTURE)
	{
		XnUInt64 nNow;
		xnOSGetTimeStamp(&nNow);
		nNow /= 1000;

		// check if time has arrived
		if ((XnInt64)nNow >= g_Capture.nStartOn)
		{
			// check if we need to discard first frame
			if (g_Capture.bSkipFirstFrame)
			{
				g_Capture.bSkipFirstFrame = false;
			}
			else
			{
				// start recording
				for (int i = 0; i < CAPTURE_STREAM_COUNT; ++i)
				{
					g_Capture.streams[i].bRecording = false;

					if (g_Capture.streams[i].isStreamOn() && g_Capture.streams[i].captureType != STREAM_DONT_CAPTURE)
					{
						nRetVal = g_Capture.recorder.attach(g_Capture.streams[i].getStream(), g_Capture.streams[i].captureType == STREAM_CAPTURE_LOSSY);
						START_CAPTURE_CHECK_RC(nRetVal, "add stream");
						g_Capture.streams[i].bRecording = TRUE;
						g_Capture.streams[i].startFrame = g_Capture.streams[i].getFrameFunc().getFrameIndex();
					}
				}

				nRetVal = g_Capture.recorder.start();
				START_CAPTURE_CHECK_RC(nRetVal, "start recording");
				g_Capture.State = CAPTURING;
			}
		}
	}
	else if (g_Capture.State == CAPTURING)
	{
		int startIdx = g_Capture.streams[CAPTURE_DEPTH_STREAM].startFrame;
		int idx = g_Capture.streams[CAPTURE_DEPTH_STREAM].getFrameFunc().getFrameIndex();
		uint64_t timestamp = g_Capture.streams[CAPTURE_DEPTH_STREAM].getFrameFunc().getTimestamp();
		static int lastIdx = idx;

		if (lastIdx != idx)
		{
			if (isNeedPhoneCapture == true)
			{
				std::string HDFileName = "HD" + std::to_string(idx - startIdx) + ".jpg";
				capturePhoneImage(HDFileName);
				isNeedPhoneCapture = false;
			}

			lastIdx = idx;

			// Get Hololen pose and write into log file
			const char * strPose = captureTrackerPose();
			if ( strPose != NULL)
			{
				g_poseLogString << timestamp << " " << idx - startIdx << "\n" << strPose;
			}
		}
	}
}

void captureSetDepthFormat(int format)
{
	g_Capture.streams[CAPTURE_DEPTH_STREAM].captureType = (StreamCaptureType)format;
}

void captureSetColorFormat(int format)
{
	g_Capture.streams[CAPTURE_COLOR_STREAM].captureType = (StreamCaptureType)format;
}

void captureSetIRFormat(int format)
{
	g_Capture.streams[CAPTURE_IR_STREAM].captureType = (StreamCaptureType)format;
}

void getCaptureMessage(char* pMessage)
{
	switch (g_Capture.State)
	{
	case SHOULD_CAPTURE:
		{
			XnUInt64 nNow;
			xnOSGetTimeStamp(&nNow);
			nNow /= 1000;
			sprintf(pMessage, "Capturing will start in %u seconds...", g_Capture.nStartOn - (XnUInt32)nNow);
		}
		break;
	case CAPTURING:
		{
			int nChars = sprintf(pMessage, "* Recording! Press any key or use menu to stop *\nRecorded Frames: ");
			for (int i = 0; i < CAPTURE_STREAM_COUNT; ++i)
			{
				if (g_Capture.streams[i].bRecording)
				{
					nChars += sprintf(pMessage + nChars, "%s-%d ", g_Capture.streams[i].name, g_Capture.streams[i].getFrameFunc().getFrameIndex() - g_Capture.streams[i].startFrame);
				}
			}
		}
		break;
	default:
		pMessage[0] = 0;
	}
}

void getColorFileName(int num, char* csName)
{
	sprintf(csName, "%s/Color_%d.png", CAPTURED_FRAMES_DIR_NAME, num);
}

void getDepthFileName(int num, char* csName)
{
	sprintf(csName, "%s/Depth_%d.png", CAPTURED_FRAMES_DIR_NAME, num);
}

void getIRFileName(int num, char* csName)
{
	sprintf(csName, "%s/IR_%d.png", CAPTURED_FRAMES_DIR_NAME, num);
}

int findUniqueFileName()
{
	xnOSCreateDirectory(CAPTURED_FRAMES_DIR_NAME);

	int num = g_Capture.nCapturedFrameUniqueID;

	XnBool bExist = FALSE;
	XnStatus nRetVal = XN_STATUS_OK;
	XnChar csColorFileName[XN_FILE_MAX_PATH];
	XnChar csDepthFileName[XN_FILE_MAX_PATH];
	XnChar csIRFileName[XN_FILE_MAX_PATH];

	for (;;)
	{
		// check color
		getColorFileName(num, csColorFileName);

		nRetVal = xnOSDoesFileExist(csColorFileName, &bExist);
		if (nRetVal != XN_STATUS_OK)
			break;

		if (!bExist)
		{
			// check depth
			getDepthFileName(num, csDepthFileName);

			nRetVal = xnOSDoesFileExist(csDepthFileName, &bExist);
			if (nRetVal != XN_STATUS_OK || !bExist)
				break;
		}

		if (!bExist)
		{
			// check IR
			getIRFileName(num, csIRFileName);

			nRetVal = xnOSDoesFileExist(csIRFileName, &bExist);
			if (nRetVal != XN_STATUS_OK || !bExist)
				break;
		}

		++num;
	}

	return num;
}

int g_captureSingleFrameNum;
void captureSingleFrame(int)
{
	int num = findUniqueFileName();
	g_captureSingleFrameNum = num;

	XnChar csColorFileName[XN_FILE_MAX_PATH];
	XnChar csDepthFileName[XN_FILE_MAX_PATH];
	XnChar csIRFileName[XN_FILE_MAX_PATH];
	getColorFileName(num, csColorFileName);
	getDepthFileName(num, csDepthFileName);
	getIRFileName(num, csIRFileName);

	std::string HDFileName = "HD" + std::to_string(num) + ".jpg";
	capturePhoneImage(HDFileName);

	openni::VideoFrameRef& colorFrame = getColorFrame();
	if (colorFrame.isValid())
	{
		//xnOSSaveFile(csColorFileName, colorFrame.getData(), colorFrame.getDataSize());

		cv::Mat colorData(colorFrame.getHeight(), colorFrame.getWidth(), CV_8UC3, (void*)colorFrame.getData());
		cv::imwrite(csColorFileName, colorData);
	}

	openni::VideoFrameRef& depthFrame = getDepthFrame();
	if (depthFrame.isValid())
	{
		//xnOSSaveFile(csDepthFileName, depthFrame.getData(), depthFrame.getDataSize());

		cv::Mat depthData(depthFrame.getHeight(), depthFrame.getWidth(), CV_16UC1, (void*)depthFrame.getData());
		cv::imwrite(csDepthFileName, depthData);
	}

	openni::VideoFrameRef& irFrame = getIRFrame();
	if (irFrame.isValid())
	{
		//xnOSSaveFile(csIRFileName, irFrame.getData(), irFrame.getDataSize());
		//std::cout << irFrame.getDataSize();

		cv::Mat irData(irFrame.getHeight(), irFrame.getWidth(), CV_16UC1, (void*)irFrame.getData());
		irData = irData * 400;
		cv::imwrite(csIRFileName, irData);
	}

	g_Capture.nCapturedFrameUniqueID = num + 1;

	displayMessage("Frames saved with ID %d", num);
}

const char* getCaptureTypeName(StreamCaptureType type)
{
	switch (type)
	{
	case STREAM_CAPTURE_LOSSLESS: return "Lossless";
	case STREAM_CAPTURE_LOSSY: return "Lossy";
	case STREAM_DONT_CAPTURE: return "Don't Capture";
	default:
		XN_ASSERT(FALSE);
		return "";
	}
}

const char* captureGetDepthFormatName()
{
	return getCaptureTypeName(g_Capture.streams[CAPTURE_DEPTH_STREAM].captureType);
}

const char* captureGetColorFormatName()
{
	return getCaptureTypeName(g_Capture.streams[CAPTURE_COLOR_STREAM].captureType);
}

const char* captureGetIRFormatName()
{
	return getCaptureTypeName(g_Capture.streams[CAPTURE_IR_STREAM].captureType);
}

inline const char* captureTrackerPose()
{
	static char buf[512];
	memset(buf, 0, sizeof(buf));

#ifndef WITHOUT_OPENVR
	std::stringstream s;
	vr::TrackedDevicePose_t pose;
	if (g_tracker.getPose(pose))
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				s << pose.mDeviceToAbsoluteTracking.m[i][j] << " ";
			}
			s << "\n";
		}
		s << "\n";

		std::string ss = s.str();
		memcpy_s(buf, sizeof(buf), ss.c_str(), ss.size());

		return buf;
	}
#endif WITHOUT_OPENVR

	return buf;
}

void initPhoneCtrler()
{
	std::string ip;
	int port;

	std::ifstream f("PhoneCfg.txt");
	if (f.is_open())
	{
		getline(f, ip);
		f >> port;

		std::cout << "[initPhoneCtrler] ip : " << ip << "  port :" << port;

		phoneCtrler = new FusionViewerController(ip, port);
	}
	else
	{
		std::cout << "[initPhoneCtrler] cannot open  PhoneCfg.txt ";
	}
}

void capturePhoneImage(std::string name)
{
	if ( phoneCtrler != NULL)
	{
		phoneCtrler->sendCpatureImageCmd(name);
	}
}
