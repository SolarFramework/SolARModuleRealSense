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

#include "SolARRGBDCameraRealSense.h"

namespace xpcf = org::bcom::xpcf;


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::REALSENSE::RGBDCamera)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace REALSENSE {

RGBDCamera::RGBDCamera():ConfigurableBase(xpcf::toUUID<RGBDCamera>())
{
    addInterface<api::input::devices::IRGBDCamera>(this);
    SRef<xpcf::IPropertyMap> params = getPropertyRootNode();


    m_RGBIntrinsic = CamCalibration::Identity();
    m_RGBDistortion = CamDistortion::Zero();
    m_DepthIntrinsic = CamCalibration::Identity();
    m_DepthDistortion = CamDistortion::Zero();
}

org::bcom::xpcf::XPCFErrorCode RGBDCamera::onConfigured()
{
    return xpcf::_SUCCESS;
}


FrameworkReturnCode RGBDCamera::getNextImage(SRef<Image>& colorImg) const
{
    return FrameworkReturnCode::_SUCCESS;

}

FrameworkReturnCode RGBDCamera::getNextDepthFrame(const SRef<Image>& depthImg, const SRef<PointCloud>& pc) const
{
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode RGBDCamera::getNextRGBDFrame(const SRef<Image>& colorImg, const SRef<Image>& depthImg, const SRef<PointCloud>& pc, const SRef<Image>& alignedColorImg, const SRef<Image>& alignedDepthImg) const
{
    return FrameworkReturnCode::_SUCCESS;

}

FrameworkReturnCode RGBDCamera::start()
{
    return FrameworkReturnCode::_SUCCESS;

}

void RGBDCamera::setResolution(Sizei resolution)
{

}

void RGBDCamera::setDepthResolution(Sizei resolution)
{

}

void RGBDCamera::setIntrinsicParameters(const CamCalibration & intrinsic_parameters)
{

}

void RGBDCamera::setIntrinsicDepthParameters(const CamCalibration & intrinsic_parameters)
{

}

void RGBDCamera::setDistorsionParameters(const CamDistortion & distorsion_parameters)
{

}

void RGBDCamera::setDistorsionDepthParameters(const CamDistortion & distorsion_parameters)
{

}

Sizei RGBDCamera::getResolution()
{
    return {0,0};
}

Sizei RGBDCamera::getDepthResolution()
{
    return {0,0};
}

const CamCalibration& RGBDCamera::getIntrinsicsParameters() const
{
    return m_RGBIntrinsic;
}

const CamCalibration& RGBDCamera::getIntrinsicsDepthParameters() const
{
    return m_DepthIntrinsic;
}

const CamDistortion& RGBDCamera::getDistorsionParameters() const
{
    return m_RGBDistortion;
}

const CamDistortion& RGBDCamera::getDistorsionDepthParameters() const
{
   return m_DepthDistortion;
}

}
}
}
