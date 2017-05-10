#ifndef _DEPTH_KINECT2_STREAM_H_
#define _DEPTH_KINECT2_STREAM_H_

#include "BaseKinect2Stream.h"
#include <Kinect.h>

namespace kinect2_device
{
  class DepthKinect2Stream : public BaseKinect2Stream
  {
    public:
      DepthKinect2Stream(Kinect2StreamImpl* pStreamImpl);
      virtual ~DepthKinect2Stream();

      virtual void frameReady(void* data, int width, int height, double timestamp);

      virtual OniStatus getProperty(int propertyId, void* data, int* pDataSize);
      virtual OniBool isPropertySupported(int propertyId);
      virtual void notifyAllProperties();

    private:
      void copyDepthPixelsStraight(const UINT16* data_in, int width, int height, OniFrame* pFrame);
      void copyDepthPixelsWithImageRegistration(const UINT16* data_in, int width, int height, OniFrame* pFrame);
	  void distort(int mx, int my, float& x, float& y);
	  void undistortDepth(const UINT16* data_in, UINT16* data_undistorted);
	  void setupUndistortion();

    private:
		struct IrCameraParams
		{
			float fx; ///< Focal length x (pixel)
			float fy; ///< Focal length y (pixel)
			float cx; ///< Principal point x (pixel)
			float cy; ///< Principal point y (pixel)
			float k1; ///< Radial distortion coefficient, 1st-order
			float k2; ///< Radial distortion coefficient, 2nd-order
			float k3; ///< Radial distortion coefficient, 3rd-order
			float p1; ///< Tangential distortion coefficient
			float p2; ///< Tangential distortion coefficient
		} m_depthIntrinsic;
	  int* m_distortLUT;
	  UINT16* m_undistortDepth;
      ColorSpacePoint* m_colorSpaceCoords;
      UINT16* m_registeredDepthMap;
  };
} // namespace kinect2_device

#endif //_DEPTH_KINECT2_STREAM_H_
