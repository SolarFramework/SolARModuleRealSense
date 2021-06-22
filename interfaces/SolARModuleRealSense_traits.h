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

#ifndef SOLARMODULEREALSENSE_TRAITS_H
#define SOLARMODULEREALSENSE_TRAITS_H

#include "xpcf/api/IComponentManager.h"

namespace SolAR {
namespace MODULES {
/**
 * @namespace SolAR::MODULES::REALSENSE
 * @brief <B>Provides a component to access realsense RGB-D camera : https://www.intelrealsense.com/ </B>
 * <TT>UUID: 63b92983-f790-448b-8124-3b686d481aaf</TT>
 *
 */
namespace REALSENSE {
class SolARRGBDCamera;
class SolARStereoCameraRealsense;
}
}
}

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::REALSENSE::SolARRGBDCamera,
                             "315dfef4-26f3-4a79-a809-874b1006cd88",
                             "SolARRGBDCamera",
                             "A component to handle a Real Sense RGBD Camera")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::REALSENSE::SolARStereoCameraRealsense,
							"0e9c544f-64af-41d8-96d2-58b5e2f816a0",
							"SolARStereoCameraRealsense",
							"A component to get data from a stereo camera of Realsense")

#endif // SOLARMODULEREALSENSE_TRAITS_H
