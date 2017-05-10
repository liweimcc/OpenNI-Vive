#include "DepthKinect2Stream.h"

#include "Kinect2StreamImpl.h"

using namespace oni::driver;
using namespace kinect2_device;
#define DEFAULT_FPS 30

#define DEVICE_MAX_DEPTH_VAL 10000
#define FILTER_RELIABLE_DEPTH_VALUE(VALUE) (((VALUE) < DEVICE_MAX_DEPTH_VAL) ? (VALUE) : 0)

DepthKinect2Stream::DepthKinect2Stream(Kinect2StreamImpl* pStreamImpl)
  : BaseKinect2Stream(pStreamImpl)
{
  m_videoMode.pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
  m_videoMode.fps = DEFAULT_FPS;
  m_videoMode.resolutionX = 512;
  m_videoMode.resolutionY = 424;
  m_colorSpaceCoords = new ColorSpacePoint[512*424];
  m_registeredDepthMap = new UINT16[512*424];

  m_distortLUT = new int[512 * 424];
  m_undistortDepth = new UINT16[512 * 424];

  m_depthIntrinsic.fx = 361.3657;
  m_depthIntrinsic.fy = 360.0796;
  m_depthIntrinsic.cx = 259.6403;
  m_depthIntrinsic.cy = 202.9793;
  m_depthIntrinsic.k1 = 0.1006;
  m_depthIntrinsic.k2 = -0.2297;
  m_depthIntrinsic.k3 = 0.0;
  m_depthIntrinsic.p1 = 0.0;
  m_depthIntrinsic.p2 = 0.0;
  setupUndistortion();
}

DepthKinect2Stream::~DepthKinect2Stream()
{
  delete[] m_colorSpaceCoords;
  delete[] m_registeredDepthMap;
}

void DepthKinect2Stream::frameReady(void* data, int width, int height, double timestamp)
{
  OniFrame* pFrame = getServices().acquireFrame();
  pFrame->videoMode.resolutionY = m_videoMode.resolutionY;
  pFrame->videoMode.resolutionX = m_videoMode.resolutionX;
  pFrame->croppingEnabled = m_cropping.enabled;
  if (m_cropping.enabled)
  {
    pFrame->width = m_cropping.width;
    pFrame->height = m_cropping.height;
    pFrame->cropOriginX = m_cropping.originX;
    pFrame->cropOriginY = m_cropping.originY;
  }
  else {
    pFrame->cropOriginX = 0;
    pFrame->cropOriginY = 0;
    pFrame->width = m_videoMode.resolutionX;
    pFrame->height = m_videoMode.resolutionY;
  }
  pFrame->dataSize = pFrame->height * pFrame->width * 2;
  pFrame->stride = pFrame->width * 2;
  pFrame->videoMode.pixelFormat = m_videoMode.pixelFormat;
  pFrame->videoMode.fps = m_videoMode.fps;
  pFrame->sensorType = ONI_SENSOR_DEPTH;
  pFrame->frameIndex = m_frameIdx++;
  pFrame->timestamp = static_cast<int>(timestamp);

  UINT16* data_in = reinterpret_cast<UINT16*>(data);
  if (m_pStreamImpl->getImageRegistrationMode() == ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR) {
    copyDepthPixelsWithImageRegistration(data_in, width, height, pFrame);
  } else {
    copyDepthPixelsStraight(data_in, width, height, pFrame);
  }

  raiseNewFrame(pFrame);
  getServices().releaseFrame(pFrame);
}

OniStatus DepthKinect2Stream::getProperty(int propertyId, void* data, int* pDataSize)
{
  OniStatus status = ONI_STATUS_NOT_SUPPORTED;
  switch (propertyId)
  {
  case ONI_STREAM_PROPERTY_MAX_VALUE:
    {
      XnInt * val = (XnInt *)data;
      *val = DEVICE_MAX_DEPTH_VAL;
      status = ONI_STATUS_OK;
      break;
    }
  case ONI_STREAM_PROPERTY_MIRRORING:
    {
      XnBool * val = (XnBool *)data;
      *val = TRUE;
      status = ONI_STATUS_OK;
      break;
    }
  default:
    status = BaseKinect2Stream::getProperty(propertyId, data, pDataSize);
    break;
  }

  return status;
}

OniBool DepthKinect2Stream::isPropertySupported(int propertyId)
{
  OniBool status = FALSE;
  switch (propertyId)
  {
  case ONI_STREAM_PROPERTY_MAX_VALUE:
  case ONI_STREAM_PROPERTY_MIRRORING:
    status = TRUE;
  default:
    status = BaseKinect2Stream::isPropertySupported(propertyId);
    break;
  }
  return status;
}

void DepthKinect2Stream::notifyAllProperties()
{
  XnInt nInt;
  int size = sizeof(nInt);
  getProperty(ONI_STREAM_PROPERTY_MAX_VALUE, &nInt, &size);
  raisePropertyChanged(ONI_STREAM_PROPERTY_MAX_VALUE, &nInt, size);

  BaseKinect2Stream::notifyAllProperties();
}

void DepthKinect2Stream::copyDepthPixelsStraight(const UINT16* data_in, int width, int height, OniFrame* pFrame)
{
  // Copy the depth pixels to OniDriverFrame
  // with applying cropping but NO depth-to-image registration.

  const int xStride = width/m_videoMode.resolutionX;
  const int yStride = height/m_videoMode.resolutionY;
  const int frameX = pFrame->cropOriginX * xStride;
  const int frameY = pFrame->cropOriginY * yStride;
  const int frameWidth = pFrame->width * xStride;
  const int frameHeight = pFrame->height * yStride;

  unsigned short* data_out = (unsigned short*) pFrame->data;
  for (int y = frameY; y < frameY + frameHeight; y += yStride) {
    for (int x = frameX; x < frameX + frameWidth; x += xStride) {
      unsigned short* iter = const_cast<unsigned short*>(data_in + (y*width + x));
      *data_out = FILTER_RELIABLE_DEPTH_VALUE(*iter);
      data_out++;
    }
  }
}

void DepthKinect2Stream::copyDepthPixelsWithImageRegistration(const UINT16* data_in, int width, int height, OniFrame* pFrame)
{
  // Copy the depth pixels to OniDriverFrame
  // with applying cropping and depth-to-image registration.

  const int xStride = width/m_videoMode.resolutionX;
  const int yStride = height/m_videoMode.resolutionY;
  const int frameX = pFrame->cropOriginX * xStride;
  const int frameY = pFrame->cropOriginY * yStride;
  const int frameWidth = pFrame->width * xStride;
  const int frameHeight = pFrame->height * yStride;
  const float xFactor = static_cast<float>(width)/1920.0f;
  const float yFactor = static_cast<float>(height)/1080.0f;

  ICoordinateMapper* coordinateMapper = m_pStreamImpl->getCoordinateMapper();
  if (coordinateMapper == NULL) {
    return;
  }

  HRESULT hr = coordinateMapper->MapDepthFrameToColorSpace(width*height, data_in, width*height, m_colorSpaceCoords);
  if (FAILED(hr)) {
    return;
  }

  unsigned short* data_out = (unsigned short*) m_registeredDepthMap;
  xnOSMemSet(data_out, 0, width*height*2);

  undistortDepth(data_in, m_undistortDepth);
  data_in = m_undistortDepth;

  const ColorSpacePoint* mappedCoordsIter = m_colorSpaceCoords;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      const float fX = mappedCoordsIter->X*xFactor;
      const float fY = mappedCoordsIter->Y*yFactor;
      const int cx = static_cast<int>(fX + 0.5f);
      const int cy = static_cast<int>(fY + 0.5f);
      if (cx >= 0 && cy >= 0 && cx < width && cy < height) {
        unsigned short* iter = const_cast<unsigned short*>(data_in + (y*width + x));
        const unsigned short d = FILTER_RELIABLE_DEPTH_VALUE(*iter);
        unsigned short* const p = data_out + cx + cy * width;
        if (*p == 0 || *p > d) *p = d;
      }
      mappedCoordsIter++;
    }
  }

  // Fill vertical gaps caused by the difference in the aspect ratio between depth and color resolutions
  data_out = (unsigned short*) pFrame->data;
  for (int y = frameY; y < frameY + frameHeight; y += yStride) {
    for (int x = frameX; x < frameX + frameWidth; x += xStride) {
      unsigned short* iter = const_cast<unsigned short*>(m_registeredDepthMap + (y*width + x));
      if (*iter == 0) {
        unsigned short davg = 0;
        int dw = 0;
        for (int ky = max(y - 1, 0); ky <= y + 1 && ky < height; ky++) {
          unsigned short* kiter = const_cast<unsigned short*>(m_registeredDepthMap + (ky*width + x));
          if (*kiter != 0) {
            davg += *kiter;
            dw += abs(ky - y);
          }
        }
        *data_out = davg;
        if (dw) {
          *data_out /= dw;
        }
      }
      else {
        *data_out = FILTER_RELIABLE_DEPTH_VALUE(*iter);
      }
      data_out++;
    }
  }
}

void DepthKinect2Stream::distort(int mx, int my, float& x, float& y)
{
	float dx = ((float)mx - m_depthIntrinsic.cx) / m_depthIntrinsic.fx;
	float dy = ((float)my - m_depthIntrinsic.cy) / m_depthIntrinsic.fy;
	float dx2 = dx * dx;
	float dy2 = dy * dy;
	float r2 = dx2 + dy2;
	float dxdy2 = 2 * dx * dy;
	float kr = 1 + ((m_depthIntrinsic.k3 * r2 + m_depthIntrinsic.k2) * r2 + m_depthIntrinsic.k1) * r2;
	x = m_depthIntrinsic.fx * (dx * kr + m_depthIntrinsic.p2 * (r2 + 2 * dx2) + m_depthIntrinsic.p1 * dxdy2) + m_depthIntrinsic.cx;
	y = m_depthIntrinsic.fy * (dy * kr + m_depthIntrinsic.p1 * (r2 + 2 * dy2) + m_depthIntrinsic.p2 * dxdy2) + m_depthIntrinsic.cy;
}

void DepthKinect2Stream::setupUndistortion()
{
	float mx, my;
	int ix, iy, index;
	int *map_dist = m_distortLUT;
	for (int y = 0; y < 424; y++) {
		for (int x = 0; x < 512; x++) {
			// compute the dirstored coordinate for current pixel
			distort(x, y, mx, my);
			// rounding the values and check if the pixel is inside the image
			ix = (int)(mx + 0.5f);
			iy = (int)(my + 0.5f);
			if (ix < 0 || ix >= 512 || iy < 0 || iy >= 424)
				index = -1;
			else
				// computing the index from the coordianted for faster access to the data
				index = iy * 512 + ix;
			*map_dist++ = index;
		}
	}
}

void DepthKinect2Stream::undistortDepth(const UINT16* data_in, UINT16* data_undistorted)
{
	// Check if all frames are valid and have the correct size
	if (!data_in || !data_undistorted)
		return;

	const int *map_dist = m_distortLUT;
	const int size_depth = 512 * 424;

	for (int i = 0; i < size_depth; ++i, ++data_undistorted, ++map_dist) {
		// getting index of distorted depth pixel
		const int index = *map_dist;

		// check if distorted depth pixel is outside of the depth image
		if (index < 0) {
			*data_undistorted = 0;
			continue;
		}

		// getting depth value for current pixel
		const UINT16 z = data_in[index];
		*data_undistorted = z;
	}
}
