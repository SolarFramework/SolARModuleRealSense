/**
 * @copyright Copyright (c) 2015 All Right Reserved, B-com http://www.b-com.com/
 *
 * This file is subject to the B<>Com License.
 * All other rights reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 */

#include <iostream>

#include "xpcf/module/ModuleFactory.h"
#include "SolARModuleRealSense_traits.h"

#include "SolARRGBDCameraRealSense.h"
#include "SolARStereoCameraRealSense.h"

namespace xpcf=org::bcom::xpcf;

XPCF_DECLARE_MODULE("63b92983-f790-448b-8124-3b686d481aaf", "SolARModuleRealSense", "SolARModuleRealSense")

extern "C" XPCF_MODULEHOOKS_API xpcf::XPCFErrorCode XPCF_getComponent(const boost::uuids::uuid& componentUUID, SRef<xpcf::IComponentIntrospect>& interfaceRef)
{
    xpcf::XPCFErrorCode errCode = xpcf::XPCFErrorCode::_FAIL;
    errCode = xpcf::tryCreateComponent<SolAR::MODULES::REALSENSE::SolARRGBDCamera>(componentUUID,interfaceRef);
	if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
	{
		errCode = xpcf::tryCreateComponent<SolAR::MODULES::REALSENSE::SolARStereoCameraRealsense>(componentUUID, interfaceRef);
	}

    return errCode;
}

XPCF_BEGIN_COMPONENTS_DECLARATION
XPCF_ADD_COMPONENT(SolAR::MODULES::REALSENSE::SolARRGBDCamera)
XPCF_ADD_COMPONENT(SolAR::MODULES::REALSENSE::SolARStereoCameraRealsense)
XPCF_END_COMPONENTS_DECLARATION
