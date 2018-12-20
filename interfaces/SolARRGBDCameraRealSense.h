/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SOLAR_RGBDCAMERA_REALSENSE_H
#define SOLAR_RGBDCAMERA_REALSENSE_H

#include <vector>
#include "api/input/devices/IRGBDCamera.h"
#include "SolARRealSenseAPI.h"
#include "xpcf/component/ConfigurableBase.h"

#include <librealsense2/rs.hpp>

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace REALSENSE {

/**
 * @class RGBDCamera
 * @brief This component handles a real sense RGBD camera and provides access to the color image, the depth image, and the 3D point cloud.
 */
class SOLARREALSENSE_EXPORT_API RGBDCamera : public org::bcom::xpcf::ConfigurableBase,
    public api::input::devices::IRGBDCamera
{
public:
    RGBDCamera();
    ~RGBDCamera()= default;

    org::bcom::xpcf::XPCFErrorCode onConfigured() override;

    /// @brief At the moment we are not able to deliver you only the RGB Image, as we aquiere both depth and image at the same time
    /// Please use getNextRGBDFrame() instead
    /// @param colorImg Will always be null
    /// @see getNextRGBDFrame()
    /// @return FrameworkReturnCode Return fail event
    FrameworkReturnCode getNextImage(SRef<Image> & colorImg) override;

    /// @brief At the moment we are not able to deliver you only the Depth Image, as we aquiere both depth and image at the same time
    /// Please use getNextRGBDFrame() instead.
    /// @param depthImg Will always be null
    /// @see getNextRGBDFrame()
    /// @return FrameworkReturnCode Return fail event
    FrameworkReturnCode getNextDepthFrame(SRef<Image>& depthImg) override;

    /// @brief Provides the last color image, depth image, corresponding 3D point cloud, and aligned images (RGB on depth and depth on RGB).
    /// If output parameters are null (nullptr), it means that the implementation, or the requested mode does not provide this feature.
    /// @param colorImg the RGB image captured by the RGBD camera
    /// @param depthImg the depth image captured by the RGBD camera
    /// @return FrameworkReturnCode to track sucessful or failing event.
    FrameworkReturnCode getNextRGBDFrame(SRef<Image>& colorImg, SRef<Image>& depthImg) override;


    /// @brief Provides the corresponding 3D point cloud to last depth image aquiered (getNextRGBDFrame())
    /// Should have no effect if the user didn't call getNextRGBDFrame() beforehand
    /// @param pc the 3D point cloud reconstructed from the depth image. Points coordinates are defined according to the RGBD camera coordinate system.
    /// @return FrameworkReturnCode to track sucessful or failing event.
    FrameworkReturnCode getPointCloud(SRef<PointCloud>& pc) override;


    /// @brief Start the acquisition device reference
    /// @return FrameworkReturnCode to track sucessful or failing event.
    FrameworkReturnCode start() override ;

    /// @brief Start the acquisition device reference
    /// @return FrameworkReturnCode to track sucessful or failing event.
    FrameworkReturnCode startRGBD() override ;

    /// @brief Provides a depth image alingned on the color image
    /// @param alignedDepthImg the depth image captured by the RGBD camera and aligned on the color image
    virtual FrameworkReturnCode alignDepthToColor (SRef<Image>& alignedDepthImg) const override;

    /// @brief Provides a color image alingned on the depth image
    /// @param alignedColorImg the RGB image captured by the RGBD camera and aligned on the depth image
    virtual FrameworkReturnCode alignColorToDepth (SRef<Image>& alignedColorImg) const override;

    /// @brief Provides the 3D point in the depth sensor coordinate system corresponding to a given pixel of the color image
    /// @param inPixel The pixel for which we want the 3 position
    /// @return a 3D point corresponding to the input pixel
    virtual Point3Df getPixelToWorld (const Point2Di& inPixel) const override;

    /// @brief Provides the pixel of the color image to the projection of a given 3D point
    /// @param in3DPoint The 3Dpoint we want to project on the color image
    /// @return a 2D point representing a pixel of the color image on which the 3D point in projected
    virtual Point2Di getWorldToPixel (const Point3Df& in3DPoint) const override;

    /// @brief Provides the pixels of the color image to the projection of given 3D points
    /// @param in3DPoints The 3D points we want to project on the color image
    /// @return a 2D points vector representing the pixels of the color image on which the 3D points are projected
    virtual std::vector<Point2Di> getWorldToPixels (const std::vector<Point3Df>& in3DPoints) const override;

    /// @brief Set the color image resolution of the acquisition device
	FrameworkReturnCode setResolution(Sizei resolution) override;

    /// @brief Set the depth image resolution of the acquisition device
	FrameworkReturnCode setDepthResolution(Sizei resolution) override;

    /// @brief Set the intrinsic RGB camera parameters
	FrameworkReturnCode setIntrinsicParameters(const CamCalibration & intrinsic_parameters) override ;

    /// @brief Set the intrinsic parameters of the depth camera
	FrameworkReturnCode setIntrinsicDepthParameters(const CamCalibration & intrinsic_parameters) override ;

    /// @brief Set the distortion intrinsic parameters of the RGB camera
    FrameworkReturnCode setDistortionParameters(const CamDistortion & distortion_parameters) override ;

    /// @brief Set the distortion intrinsic parameters of the depth camera
	FrameworkReturnCode setDistortionDepthParameters(const CamDistortion & distortion_parameters) override ;

    /// @brief Get the image resolution of the RGB acquisition device
    Sizei getResolution() override ;

    /// @brief Get the image resolution of the depth acquisition device
    Sizei getDepthResolution() override ;

    /// @return Return the intrinsic RGB camera parameters
    const CamCalibration& getIntrinsicsParameters() const override ;

    /// @return Return the intrinsic depth camera parameters
    const CamCalibration& getIntrinsicsDepthParameters() const override ;

    /// @return Return the distortion RGB camera lens parameters
    const CamDistortion& getDistortionParameters() const override ;

    /// @return Return the distortion depth camera lens parameters
    const CamDistortion& getDistortionDepthParameters() const override ;

    void unloadComponent () override ;



private:
	/**
	 * \brief Convert a realsense color frame to a SolAR image
	 * \param color_frame realsense color frame
	 * \return Solar Image corresponding to given frame
	 */
	static SRef<Image> rgbFrameToImage(const rs2::frame& color_frame);
	/**
	 * \brief Convert a realsense depth frame to a SolAR image
	 * \param depth_frame realsense depth frame
	 * \return Solar Image corresponding to given frame
	 */
	static SRef<Image> depthFrameToImage(const rs2::frame& depth_frame);

	/**
	 * \brief Update frameset with lastest images
	 * \return True if success, false otherwise
	 */
	bool updateFrameset();

	/**
	 * \brief Internal helper function to calculate pixel to world
	 * \param pixels_depth Data gathered from an aligned depth frame
	 * \param w  Width of aligned depth frame
	 * \param h  Height of aligned depth frame
	 * \param i  2d point x
	 * \param j  2d point y
	 * \param point output 3dpoint
	 */
	void pixelToWorld(const uint16_t* pixels_depth,
		int w, int h, int i, int j, Point3Df& point) const;


	/**
	 * \brief Takes last color image from realsense frameset and put it in colorImg
	 * \param colorImg Output image
	 * \return true if success, false otherwise
	 */
	bool fillRGBImage(SRef<Image>& colorImg);

	/**
	 * \brief Takes last depth image from realsense frameset and put it in depthImg
	 * \param depthImg Output image
	 * \return true if success, false otherwise
	 */
	bool fillDepthImage(SRef<Image>& depthImg);


	/// Current depth scale
	float m_depth_scale = 1.f;

	// Default values

	/// Default value for color stream
	Sizei requested_rgb_size{ 1280, 720 };
	/// Default value for depth stream
	Sizei requested_depth_size{ 640, 480 };


	// Flags

	/// True if at least a stream is opened
	bool m_is_opened = false;



	// Realsense

	/// Used to query available devices
	rs2::context m_context;
	/// Realsense pipeline
    rs2::pipeline m_pipe;

	/// Last frameset available
    rs2::frameset m_frameset;
	/// Last depth frame available
	rs2::frame m_last_depth_frame;

	/// Depth intrinsics
    rs2_intrinsics m_depth_intrin;
	/// Color intrinsics
    rs2_intrinsics m_color_intrin;

	/// Color -> depth transformation
    rs2_extrinsics m_color_to_depth;
	/// Depth -> color transformation
    rs2_extrinsics m_depth_to_color;

	// SolAR

	/// Contains the calibration and distortion of a stream (rbg or depth)
	struct CameraInformation
	{
		CamCalibration calibration = CamCalibration::Identity();
		CamDistortion distortion = CamDistortion::Zero();

		/// Extract realsense intrinsics to fill this structure (calibration and distortion)
		void extractRSIntrinsics(const rs2_intrinsics& intrinsics);
	};
	/// RGB camera information (distortion + calibration)
	CameraInformation m_rgb_camera_information;
	/// Depth camera information (distortion + calibration)
	CameraInformation m_depth_camera_information;

};

}
}
}

#endif // SOLAR_RGBDCAMERA_REALSENSE_H
