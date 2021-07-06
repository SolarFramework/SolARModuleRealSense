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


#include <iostream>
#include <string>
#include <vector>
#include <boost/log/core.hpp>

// ADD XPCF HEADERS HERE
#include "xpcf/xpcf.h"
#include "core/Log.h"

// ADD COMPONENTS HEADERS HERE
#include "api/input/devices/IARDevice.h"
#include "api/display/IImageViewer.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf  = org::bcom::xpcf;

int main(int argc, char *argv[])
{

#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif
    LOG_ADD_LOG_TO_CONSOLE();

	try {
		SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

        if (xpcfComponentManager->load("SolARTest_ModuleRealSense_StereoCamera_conf.xml") != org::bcom::xpcf::_SUCCESS)
		{
            LOG_ERROR("Failed to load the configuration file SolARTest_ModuleRealSense_StereoCamera_conf.xml")
				return -1;
		}

		// declare and create components
		LOG_INFO("Start creating components");
		auto camera = xpcfComponentManager->resolve<input::devices::IARDevice>();
		auto viewer1 = xpcfComponentManager->resolve<display::IImageViewer>("camera1");
		auto viewer2 = xpcfComponentManager->resolve<display::IImageViewer>("camera2");
		LOG_INFO("Components created");

		if (!camera || !viewer1 || !viewer2) {
			LOG_ERROR("One or more component creations have failed");
			return -1;
		}

		//declaration
		std::vector<SRef<Image>>				images;
		std::vector<Transform3Df>				poses;
		std::chrono::system_clock::time_point	timestamp;
		char lastKey = ' ';

		// start depth camera
		if (camera->start() != FrameworkReturnCode::_SUCCESS) {
			LOG_ERROR("Can't start camera stream. Check if camera is connected on a USB3 port.")
				return EXIT_FAILURE;
		}

		// get intrinsic parameters
		CameraParameters param1 = camera->getParameters(0);
		CameraParameters param2 = camera->getParameters(1);
		LOG_INFO("Camera 1 parameters:\nResolution: {} x {}\nIntrinsic:\n{}\nDistortion:\n{}",
			param1.resolution.width, param1.resolution.height, param1.intrinsic, param1.distortion);
		LOG_INFO("Camera 2 parameters:\nResolution: {} x {}\nIntrinsic:\n{}\nDistortion:\n{}",
			param2.resolution.width, param2.resolution.height, param2.intrinsic, param2.distortion);

		// get rectification parameters
		std::vector<RectificationParameters> rectParams;
		LOG_INFO("Rectification parameters:");
		if (camera->getRectificationParameters(std::make_pair(0, 1), rectParams) == FrameworkReturnCode::_SUCCESS) {
			LOG_INFO("Baseline: {}", rectParams[0].baseline);
			LOG_INFO("fb: {}", rectParams[0].fb);
			LOG_INFO("Type: {}", rectParams[0].type);
			for (int i = 0; i < 2; ++i)
				LOG_INFO("Camera {} parameters:\nRotation:\n{}\nProjection:\n{}", i, rectParams[i].rotation, rectParams[i].projection);
		}
		else
			LOG_WARNING("Not existed rectification between camera 0 and 1");

		while (true) {
			camera->getData(images, poses, timestamp);	
			if (viewer1->displayKey(images[0], lastKey) == FrameworkReturnCode::_STOP ||
				viewer2->displayKey(images[1],lastKey) == FrameworkReturnCode::_STOP)
			{
				LOG_INFO("End of SolARStereoCamera test");
				break;
			}
		}
	}
	catch (xpcf::Exception e)
	{
		LOG_ERROR ("The following exception has been catch : {}", e.what());
		return -1;
	}
    return 0;
}
