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

#include "SolARStereoCameraRealSense.h"
#include "datastructure/Image.h"
#include <librealsense2/rsutil.h>
#include "xpcf/core/helpers.h"
#include "core/Log.h"

namespace xpcf = org::bcom::xpcf;


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::REALSENSE::SolARStereoCameraRealsense)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace REALSENSE {

SolARStereoCameraRealsense::SolARStereoCameraRealsense():ConfigurableBase(xpcf::toUUID<SolARStereoCameraRealsense>())
{
	declareInterface<api::input::devices::IARDevice>(this);
	declareProperty("width", m_width);
	declareProperty("height", m_height);
	declareProperty("framerate", m_framerate);
	LOG_DEBUG("SolARStereoCameraRealsense constructor");
}

SolARStereoCameraRealsense::~SolARStereoCameraRealsense()
{
	LOG_DEBUG("SolARStereoCameraRealsense destructor");
}

const SolAR::datastructure::CameraRigParameters & SolARStereoCameraRealsense::getCameraParameters() const
{
	return m_camRigParameters;
}

org::bcom::xpcf::XPCFErrorCode SolARStereoCameraRealsense::onConfigured()
{
	return xpcf::XPCFErrorCode::_SUCCESS;
}

SRef<datastructure::Image> SolARStereoCameraRealsense::frameToImage(const rs2::frame & frame)
{
	auto infraredFrame = frame.as<rs2::video_frame>();
	return xpcf::utils::make_shared<Image>(
		(void*)infraredFrame.get_data(),
		infraredFrame.get_width(),
		infraredFrame.get_height(),
		Image::ImageLayout::LAYOUT_GREY,
		Image::PixelOrder::PER_CHANNEL,
		Image::DataType::TYPE_8U);
}

FrameworkReturnCode SolARStereoCameraRealsense::start()
{
	rs2::config config;
	rs2::pipeline_profile pipeline_profile;
	// Get a snapshot of currently connected devices
	auto list = m_context.query_devices();
	if (list.size() == 0) {
		LOG_ERROR("No cam found. Please connect a camera to the laptop");
		return FrameworkReturnCode::_ERROR_;
	}
	const std::string device_name = list.front().get_info(RS2_CAMERA_INFO_NAME);
	LOG_INFO("Opening the camera device : {} ", device_name);

	config.enable_stream(RS2_STREAM_INFRARED, 1, m_width, m_height, RS2_FORMAT_Y8, m_framerate);
	config.enable_stream(RS2_STREAM_INFRARED, 2, m_width, m_height, RS2_FORMAT_Y8, m_framerate);		
	// Start the pipeline on the first detected device
	pipeline_profile = m_pipe.start(config);

	// Disable emitter
	auto depth_sensor = pipeline_profile.get_device().first<rs2::depth_sensor>();
	if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
		depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); 
		
	// Setup intrinsics and distortion
	auto infrared1_stream = pipeline_profile.get_stream(RS2_STREAM_INFRARED, 1).as<rs2::video_stream_profile>();
	auto infrared2_stream = pipeline_profile.get_stream(RS2_STREAM_INFRARED, 2).as<rs2::video_stream_profile>();
	rs2_intrinsics infrared1Intrinsic = infrared1_stream.get_intrinsics();
	rs2_intrinsics infrared2Intrinsic = infrared2_stream.get_intrinsics();
	std::vector<CameraParameters> camParameters(2);
	// set id
	camParameters[0].id = 0;
	camParameters[1].id = 1;
	//set name
	camParameters[0].name = "Infrared 1";
	camParameters[1].name = "Infrared 2";

	// set resolution
	camParameters[0].resolution.width = infrared1Intrinsic.width;
	camParameters[0].resolution.height = infrared1Intrinsic.height;
	camParameters[1].resolution.width = infrared2Intrinsic.width;
	camParameters[1].resolution.height = infrared2Intrinsic.height;

	// set intrinsic
	camParameters[0].intrinsic = CamCalibration::Identity();
	camParameters[0].intrinsic(0, 0) = infrared1Intrinsic.fx;
	camParameters[0].intrinsic(1, 1) = infrared1Intrinsic.fy;
	camParameters[0].intrinsic(0, 2) = infrared1Intrinsic.ppx;
	camParameters[0].intrinsic(1, 2) = infrared1Intrinsic.ppy;

	camParameters[1].intrinsic = CamCalibration::Identity();
	camParameters[1].intrinsic(0, 0) = infrared2Intrinsic.fx;
	camParameters[1].intrinsic(1, 1) = infrared2Intrinsic.fy;
	camParameters[1].intrinsic(0, 2) = infrared2Intrinsic.ppx;
	camParameters[1].intrinsic(1, 2) = infrared2Intrinsic.ppy;

	// set distortion
	for (int i = 0; i < 5; ++i) {
		camParameters[0].distortion(i) = infrared1Intrinsic.coeffs[i];
		camParameters[1].distortion(i) = infrared2Intrinsic.coeffs[i];
	}

	// get extrinsic parameters
	rs2_extrinsics infrared1To2Extrinsic = infrared1_stream.get_extrinsics_to(infrared2_stream);
	Transform3Df transformation = Transform3Df::Identity();
	transformation(0, 3) = infrared1To2Extrinsic.translation[0];
	std::vector<RectificationParameters> rectParams(2);
	for (int i = 0; i < 2; ++i) {
		rectParams[i].baseline = std::abs(infrared1To2Extrinsic.translation[0]);
		rectParams[i].rotation.setIdentity();
		rectParams[i].type = StereoType::Horizontal;
		for (int r = 0; r < 3; ++r)
			for (int c = 0; c < 3; ++c)
				rectParams[i].projection(r, c) = camParameters[i].intrinsic(r, c);
	}
	rectParams[1].projection(0, 3) = infrared1To2Extrinsic.translation[0] * camParameters[1].intrinsic(0, 0);

	// set parameters to rig
	m_camRigParameters.cameraParams[0] = camParameters[0];
	m_camRigParameters.cameraParams[1] = camParameters[1];
	m_camRigParameters.transformations[std::make_pair(0, 1)] = transformation;
	m_camRigParameters.rectificationParams[std::make_pair(0, 1)] = std::make_pair(rectParams[0], rectParams[1]);
	LOG_INFO("The camera device is now opened and configured");
	return FrameworkReturnCode();
}

FrameworkReturnCode SolARStereoCameraRealsense::stop()
{
	try {
		m_pipe.stop();
	}
	catch (const rs2::error & e) {
		LOG_ERROR("RealSense::stop - error calling {} ({}) : {}",
			e.get_failed_function(), e.get_failed_args(), e.what());
		// Don't forget to put the is opened flag to false in case of errors
		return FrameworkReturnCode::_ERROR_;
	}
	catch (...)
	{
		LOG_ERROR("error : unknown error");
		return FrameworkReturnCode::_ERROR_;
	}
}

FrameworkReturnCode SolARStereoCameraRealsense::getData(std::vector<SRef<datastructure::Image>>& images, std::vector<datastructure::Transform3Df>& poses, std::chrono::system_clock::time_point & timestamp)
{
	images.clear();
	poses.clear();
	if (!m_pipe.try_wait_for_frames(&m_frameset, 1000U)) // 1s timeout
	{
		LOG_ERROR("RealSense::open - timeout waiting for live camera frame");
		return FrameworkReturnCode::_ERROR_;
	}
	auto infrared1Frame = m_frameset.get_infrared_frame(1);
	auto infrared2Frame = m_frameset.get_infrared_frame(2);
	images.push_back(frameToImage(infrared1Frame));
	images.push_back(frameToImage(infrared2Frame));
	return FrameworkReturnCode::_SUCCESS;
}

}
}
}
