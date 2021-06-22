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

#ifndef SOLARSTEREOCAMERAREALSENSE_H
#define SOLARSTEREOCAMERAREALSENSE_H

#include <vector>
#include "api/input/devices/IARDevice.h"
#include "SolARRealSenseAPI.h"
#include "xpcf/component/ConfigurableBase.h"
#include <librealsense2/rs.hpp>

namespace SolAR {
namespace MODULES {
namespace REALSENSE {

/**
 * @class SolARStereoCameraRealsense
 * @brief This component gets data from a stereo camera of Realsense.
 * <TT>UUID: 0e9c544f-64af-41d8-96d2-58b5e2f816a0</TT>
 *
 */

class SOLARREALSENSE_EXPORT_API SolARStereoCameraRealsense : public org::bcom::xpcf::ConfigurableBase,
    public api::input::devices::IARDevice
{
public:
	SolARStereoCameraRealsense();
    ~SolARStereoCameraRealsense() override;

	/// @brief Start the connection to the device for sensors data streaming.
	/// @return FrameworkReturnCode::_SUCCESS if successful, eiher FrameworkReturnCode::_ERROR_.
	FrameworkReturnCode start() override;

	/// @brief Stop the connection to the device.
	/// @return FrameworkReturnCode::_SUCCESS if successful, eiher FrameworkReturnCode::_ERROR_.
	FrameworkReturnCode stop() override;

	/// @brief Get number of cameras of the device.
	/// @return the number of cameras.
	int getNbCameras() override;

	/// @brief Retrieve a set of images and their associated poses from the sensors as well as timestamp.
	/// @param[out] images: the captured images.
	/// @param[out] poses: the associated poses.
	/// @param[out] timestamp: the timestamp.
	/// @return FrameworkReturnCode to track successful or failing event.
	FrameworkReturnCode getData(std::vector<SRef<datastructure::Image>> & images, std::vector<datastructure::Transform3Df> & poses, std::chrono::system_clock::time_point &timestamp) override;

	/// @brief Get the distortion and intrinsic camera parameters
	/// @param[in] camera_id: The id of the camera.
	/// @return the camera parameters
	const datastructure::CameraParameters & getParameters(const int & camera_id) const override;

	/// @brief Set the distortion and intrinsic camera parameters
	/// @param[in] camera_id: The id of the camera.
	/// @param[in] parameters: the camera parameters.
	void setParameters(const int & camera_id, const datastructure::CameraParameters & parameters) override;

    org::bcom::xpcf::XPCFErrorCode onConfigured() override;
    void unloadComponent () override final;

private:
	/// @brief Convert a realsense infrared frame to a SolAR image
	/// @param frame realsense infrared frame
	/// @return Solar Image corresponding to given frame
	static SRef<datastructure::Image> frameToImage(const rs2::frame& frame);

private:
	/// Used to query available devices
	rs2::context m_context;
	/// Realsense pipeline
    rs2::pipeline m_pipe;
	/// Last frameset available
    rs2::frameset m_frameset;
	/// camera parameters
	std::vector<datastructure::CameraParameters> m_camParameters;
	/// number of cameras
	int	m_nbCameras = 2;
	/// width of image
	int m_width;
	/// height of image
	int m_height;
	/// framerate of camera
	int m_framerate;
};

}
}
}

#endif // SOLARSTEREOCAMERAREALSENSE_H
