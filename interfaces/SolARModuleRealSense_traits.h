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
namespace REALSENSE {
class SolARRGBDCamera;
}
}
}

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::REALSENSE::SolARRGBDCamera,
                             "315dfef4-26f3-4a79-a809-874b1006cd88",
                             "SolARRGBDCamera",
                             "A component to handle a Real Sense RGBD Camera")

#endif // SOLARMODULEREALSENSE_TRAITS_H
