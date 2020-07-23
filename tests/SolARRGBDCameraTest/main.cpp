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

 // ADD MODULES TRAITS HEADERS HERE
#include "SolARModuleOpencv_traits.h"
#include "SolARModuleRealSense_traits.h"

// ADD COMPONENTS HEADERS HERE
#include "api/input/devices/IRGBDCamera.h"
#include "api/image/IImageConvertor.h"
#include "api/display/IImageViewer.h"

#define DEPTH_SCALE 4.f * 255.0f/65535.0f

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::MODULES::REALSENSE;

namespace xpcf  = org::bcom::xpcf;

int main(int argc, char *argv[])
{

#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

	try {
		SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

		if (xpcfComponentManager->load("SolARRGBDCameraTest_conf.xml") != org::bcom::xpcf::_SUCCESS)
		{
			LOG_ERROR("Failed to load the configuration file SolARRGBDCameraTest_conf.xml")
				return -1;
		}

		// declare and create components
		LOG_INFO("Start creating components");
		auto camera = xpcfComponentManager->resolve<input::devices::IRGBDCamera>();
		auto imageConvertor = xpcfComponentManager->resolve<image::IImageConvertor>();
		auto viewerRGB = xpcfComponentManager->create<SolARImageViewerOpencv>("color")->bindTo<display::IImageViewer>();
		auto viewerDepth = xpcfComponentManager->create<SolARImageViewerOpencv>("depth")->bindTo<display::IImageViewer>();
		LOG_INFO("Components created");

		if (!camera || !imageConvertor || !viewerRGB || !viewerDepth) {
			LOG_ERROR("One or more component creations have failed");
			return -1;
		}

		//declaration
		SRef<Image>				imageRGB;
		SRef<Image>				imageDepth;
		SRef<Image>				imageConvertedDepth = xpcf::utils::make_shared<Image>(Image::LAYOUT_GREY, Image::PER_CHANNEL, Image::DataType::TYPE_8U);
		char lastKey = ' ';

		// start depth camera
		if (camera->start() != FrameworkReturnCode::_SUCCESS) {
			LOG_ERROR("Can't start camera stream. Check if camera is connected on a USB3 port.")
				return EXIT_FAILURE;
		}

		while (true) {
			camera->getNextRGBDFrame(imageRGB, imageDepth);
			imageConvertor->convert(imageDepth, imageConvertedDepth, Image::LAYOUT_GREY, DEPTH_SCALE);


			if (viewerRGB->displayKey(imageRGB, lastKey) == FrameworkReturnCode::_STOP ||
				viewerDepth->displayKey(imageConvertedDepth,lastKey) == FrameworkReturnCode::_STOP)
			{
				LOG_INFO("End of SolARRGBDCamera test");
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
