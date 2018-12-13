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

    /// @brief Fill the SRef img buffer with a new RGB image captured by the camera device.
    /// @param img the image captured by the RGBD camera
    /// @return FrameworkReturnCode to track sucessful or failing event.
    FrameworkReturnCode getNextImage(SRef<Image> & colorImg) const override;

    /// @brief Provides the last depth image and corresponding 3D point cloud.
    /// If output parameters are null (nullptr), it means that the implementation, or the requested mode does not provide this feature.
    /// @param depthImg the image captured by the RGBD camera
    /// @param pc the 3D point cloud reconstructed from the depth image. Points coordinates are defined according to the RGBD camera coordinate system.
    /// @return FrameworkReturnCode to track sucessful or failing event.
    FrameworkReturnCode getNextDepthFrame(const SRef<Image>& depthImg, const SRef<PointCloud>& pc) const override;

    /// @brief Provides the last color image, depth image, corresponding 3D point cloud, and aligned images (RGB on depth and depth on RGB).
    /// If output parameters are null (nullptr), it means that the implementation, or the requested mode does not provide this feature.
    /// @param colorImg the RGB image captured by the RGBD camera
    /// @param depthImg the depth image captured by the RGBD camera
    /// @param pc the 3D point cloud reconstructed from the depth image. Points coordinates are defined according to the RGBD camera coordinate system.
    /// @return FrameworkReturnCode to track sucessful or failing event.
    FrameworkReturnCode getNextRGBDFrame(const SRef<Image>& colorImg, const SRef<Image>& depthImg, const SRef<PointCloud>& pc) const  override;

    /// @brief Start the acquisition device reference by its device_id
    /// @return FrameworkReturnCode to track sucessful or failing event.
    FrameworkReturnCode start() override ;

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

    /// @brief Set the color image resolution of the acquisition device
    void setResolution(Sizei resolution) override ;

    /// @brief Set the depth image resolution of the acquisition device
    void setDepthResolution(Sizei resolution) override ;

    /// @brief Set the intrinsic RGB camera parameters
    void setIntrinsicParameters(const CamCalibration & intrinsic_parameters) override ;

    /// @brief Set the intrinsic parameters of the depth camera
    void setIntrinsicDepthParameters(const CamCalibration & intrinsic_parameters) override ;

    /// @brief Set the distorsion intrinsic parameters of the RGB camera
    void setDistorsionParameters(const CamDistortion & distorsion_parameters) override ;

    /// @brief Set the distorsion intrinsic parameters of the depth camera
    void setDistorsionDepthParameters(const CamDistortion & distorsion_parameters) override ;

    /// @brief Get the image resolution of the RGB acquisition device
    Sizei getResolution() override ;

    /// @brief Get the image resolution of the depth acquisition device
    Sizei getDepthResolution() override ;

    /// @return Return the intrinsic RGB camera parameters
    const CamCalibration& getIntrinsicsParameters() const override ;

    /// @return Return the intrinsic depth camera parameters
    const CamCalibration& getIntrinsicsDepthParameters() const override ;

    /// @return Return the distorsion RGB camera lens parameters
    const CamDistortion& getDistorsionParameters() const override ;

    /// @return Return the distorsion depth camera lens parameters
    const CamDistortion& getDistorsionDepthParameters() const override ;

    void unloadComponent () override ;



private:
    CamCalibration m_RGBIntrinsic;
    CamDistortion m_RGBDistortion;
    CamCalibration m_DepthIntrinsic;
    CamDistortion m_DepthDistortion;

};

}
}
}

#endif // SOLAR_RGBDCAMERA_REALSENSE_H
