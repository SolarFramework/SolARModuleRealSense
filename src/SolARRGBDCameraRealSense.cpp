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
#include "datastructure/Image.h"
#include <librealsense2/rsutil.h>
#include "xpcf/core/helpers.h"
#include "core/Log.h"

namespace xpcf = org::bcom::xpcf;


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::REALSENSE::SolARRGBDCamera)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace REALSENSE {

SolARRGBDCamera::SolARRGBDCamera():ConfigurableBase(xpcf::toUUID<SolARRGBDCamera>())
{
		declareInterface<api::input::devices::IRGBDCamera>(this);
		declareProperty("rgb_width", m_rgb_camera_information.size.width);
		declareProperty("rgb_height", m_rgb_camera_information.size.height);
		declareProperty("rgb_framerate", m_rgb_camera_information.framerate);
		declareProperty("depth_width", m_depth_camera_information.size.width);
		declareProperty("depth_height", m_depth_camera_information.size.height);
		declareProperty("depth_framerate", m_depth_camera_information.framerate);
		declareProperty("depth_colorize", m_depth_colorize);
		declareProperty("depth_minimum", m_depth_minimum_distance);
		LOG_DEBUG("SolARRGBDCamera constructor");
}

org::bcom::xpcf::XPCFErrorCode SolARRGBDCamera::onConfigured()
{
        return xpcf::XPCFErrorCode::_SUCCESS;
}


FrameworkReturnCode SolARRGBDCamera::getNextImage(ATTRIBUTE(maybe_unused) SRef<Image>& colorImg)
{
	return FrameworkReturnCode::_NOT_IMPLEMENTED;
}

FrameworkReturnCode SolARRGBDCamera::getNextDepthFrame(ATTRIBUTE(maybe_unused) SRef<Image>& depthImg)
{
	return FrameworkReturnCode::_NOT_IMPLEMENTED;
}

FrameworkReturnCode SolARRGBDCamera::getNextRGBDFrame(ATTRIBUTE(maybe_unused) SRef<Image>& colorImg,
        ATTRIBUTE(maybe_unused) SRef<Image>& depthImg)
{
        if (!updateFrameset())
                return FrameworkReturnCode::_ERROR_;

        if (!fillRGBImage(colorImg))
                return FrameworkReturnCode::_ERROR_;

        if (!fillDepthImage(depthImg))
                return FrameworkReturnCode::_ERROR_;

        return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARRGBDCamera::getPointCloud(SRef<PointCloud> &outputPointCloud)
{
        // See if it doesn't cost too much to do this
        // otherwise just save the last depth frame
        auto depth_frame = m_last_depth_frame;

        if (!depth_frame) {
                outputPointCloud = nullptr;
                return FrameworkReturnCode::_ERROR_;
        }

        rs2::pointcloud pointCloud;

        // If we got a depth frame, generate the pointcloud and texture mappings
        auto points = pointCloud.calculate(depth_frame);

        auto vertices = points.get_vertices(); // get vertices

        if (outputPointCloud == nullptr)
            outputPointCloud = xpcf::utils::make_shared<PointCloud>();

        for (auto i = 0u; i < points.size(); i++)
        {
			outputPointCloud->addPoint(CloudPoint(vertices[i].x, vertices[i].y, vertices[i].z));
        }

        return  FrameworkReturnCode::_SUCCESS;
}

bool SolARRGBDCamera::fillRGBImage(SRef<Image>& colorImg) {
        auto rgb_frame = m_frameset.get_color_frame();

        if (!rgb_frame)
                return false;

        colorImg = rgbFrameToImage(rgb_frame);

        return true;
}

bool SolARRGBDCamera::fillDepthImage(SRef<Image>& depthImg) {
        // Save last depth frame to compute point could
        m_last_depth_frame = m_frameset.get_depth_frame();

        if (!m_last_depth_frame)
                return false;

        depthImg = depthFrameToImage(m_last_depth_frame,m_depth_colorize);

        return true;
}

void SolARRGBDCamera::CameraInformation::extractRSIntrinsics(const rs2_intrinsics& intrinsics)
{
        // @TODO Not sure that fx and fy are the same in realsense or openCV
        calibration(0, 0) = intrinsics.fx;
        calibration(1, 1) = intrinsics.fy;

        calibration(0, 2) = intrinsics.ppx;
        calibration(1, 2) = intrinsics.ppy;

        calibration(2, 2) = 1;

        // Copy distortion parameters
        // Realsense orders the parameters the same way than ours (k1, k2, p1, p2, k3)
        std::copy(intrinsics.coeffs, intrinsics.coeffs + 5,
                distortion.data());
}

bool SolARRGBDCamera::updateFrameset()
{
        if (!m_is_opened)
                return false;

        if (!m_pipe.try_wait_for_frames(&m_frameset, 1000U)) // 1s timeout
        {
                LOG_ERROR("RealSense::open - timeout waiting for live camera frame");
                return false;
        }

        return true;
}

SRef<Image> SolARRGBDCamera::rgbFrameToImage(const rs2::frame& frame)
{
        auto video_frame = frame.as<rs2::video_frame>();

        return xpcf::utils::make_shared<Image>(
                (void*)video_frame.get_data(),
                video_frame.get_width(),
                video_frame.get_height(),
                Image::ImageLayout::LAYOUT_RGB,
                Image::PixelOrder::INTERLEAVED,
                Image::DataType::TYPE_8U);
}

SRef<Image> SolARRGBDCamera::depthFrameToImage(const rs2::frame& frame,const int colorize)
{
        auto depth_frame = frame.as<rs2::depth_frame>();

		if (colorize>=0) {
			//colorize image
			rs2::colorizer color_map(colorize);
			auto depth_colorized = color_map.colorize(depth_frame);

			return xpcf::utils::make_shared<Image>(
				(void*)depth_colorized.get_data(),
				depth_colorized.get_width(),
				depth_colorized.get_height(),
				Image::ImageLayout::LAYOUT_RGB,
				Image::PixelOrder::PER_CHANNEL,
				Image::DataType::TYPE_8U);
		}
		else {
			return xpcf::utils::make_shared<Image>(
				(void*)depth_frame.get_data(),
				depth_frame.get_width(),
				depth_frame.get_height(),
				Image::ImageLayout::LAYOUT_GREY,
				Image::PixelOrder::PER_CHANNEL,
				Image::DataType::TYPE_16U);
		}
}

FrameworkReturnCode SolARRGBDCamera::start()
{
    try
    {
        rs2::config config;
        rs2::pipeline_profile pipeline_profile;

        // Get a snapshot of currently connected devices
        auto list = m_context.query_devices();

        if (list.size() > 0)
        {
            const std::string device_name = list.front().get_info(RS2_CAMERA_INFO_NAME);

            LOG_INFO("Opening the camera device : {} ", device_name);
            LOG_INFO("-> Enabling rgb stream resolution {}x{} - {} fps", m_rgb_camera_information.size.width, m_rgb_camera_information.size.height, m_rgb_camera_information.framerate);
            config.enable_stream(RS2_STREAM_COLOR, m_rgb_camera_information.size.width, m_rgb_camera_information.size.height, RS2_FORMAT_BGR8, m_rgb_camera_information.framerate);
            LOG_INFO("-> Enabling depth stream resolution {}x{} - {} fps", m_depth_camera_information.size.width, m_depth_camera_information.size.height, m_depth_camera_information.framerate);
            config.enable_stream(RS2_STREAM_DEPTH, m_depth_camera_information.size.width, m_depth_camera_information.size.height, RS2_FORMAT_Z16, m_depth_camera_information.framerate);
            // Set camera stream as opened
            m_is_opened = true;
            // Start the pipeline on the first detected device
            pipeline_profile = m_pipe.start(config);
            // Remember depth scale as it is used later
            auto depth_sensor = pipeline_profile.get_device().first<rs2::depth_sensor>();
            m_depth_scale = depth_sensor.get_depth_scale();
        }
        else
        {
            LOG_ERROR("No cam found. Please connect a camera to the laptop");
            return FrameworkReturnCode::_ERROR_;
        }

        // Setup intrinsics and distortion
        auto depth_stream = pipeline_profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        auto color_stream = pipeline_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

        // Raw data from RealSense
        m_depth_intrin = depth_stream.get_intrinsics();
        m_color_intrin = color_stream.get_intrinsics();

        // Extract calibration and distortion matrix from raw data
        m_depth_camera_information.extractRSIntrinsics(m_depth_intrin);
        m_depth_camera_information.extractRSIntrinsics(m_color_intrin);

        // Transformation matrix from RGB world to Depth world and vice versa
        m_color_to_depth = color_stream.get_extrinsics_to(depth_stream);
        m_depth_to_color = depth_stream.get_extrinsics_to(color_stream);

        LOG_INFO("The camera device is now opened and configured");
    }
    catch (const rs2::error & e)
    {
            LOG_ERROR("RealSense::open - error calling {} ({}) : {}",
                    e.get_failed_function(), e.get_failed_args(), e.what());
            // Don't forget to put the is opened flag to false in case of errors
            m_is_opened = false;
            return FrameworkReturnCode::_ERROR_;
    }
    catch (std::exception& e)
    {
            LOG_ERROR("error : {}", e.what());
            // Don't forget to put the is opened flag to false in case of errors
            m_is_opened = false;
            return FrameworkReturnCode::_ERROR_;
    }
    catch (...)
    {
            LOG_ERROR("error : unknown error");
            // Don't forget to put the is opened flag to false in case of errors
            m_is_opened = false;
            return FrameworkReturnCode::_ERROR_;
    }

        return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARRGBDCamera::stop()
{	
	try {
		m_pipe.stop();
		m_is_opened = false;
	}
	catch (const rs2::error & e){
		LOG_ERROR("RealSense::stop - error calling {} ({}) : {}",
		e.get_failed_function(), e.get_failed_args(), e.what());
		// Don't forget to put the is opened flag to false in case of errors
		m_is_opened = false;
		return FrameworkReturnCode::_ERROR_;
	}
	catch (...)
	{
		LOG_ERROR("error : unknown error");
		return FrameworkReturnCode::_ERROR_;
	}
}

FrameworkReturnCode SolARRGBDCamera::alignDepthToColor(SRef<Image>& alignedDepthImg) const
{
        if (m_frameset.size() == 0) {
                alignedDepthImg = nullptr;
                return FrameworkReturnCode::_ERROR_;
        }

        rs2::align align(RS2_STREAM_COLOR);
        const auto rsDepthFrame = align.process(m_frameset).get_depth_frame();

        alignedDepthImg = depthFrameToImage(rsDepthFrame);

        return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARRGBDCamera::alignColorToDepth(SRef<Image>& alignedColorImg) const
{
        if (m_frameset.size() == 0) {
                alignedColorImg = nullptr;
                return FrameworkReturnCode::_ERROR_;
        }

        rs2::align align(RS2_STREAM_DEPTH);
        const auto rsColorFrame = align.process(m_frameset).get_color_frame();

        alignedColorImg = rgbFrameToImage(rsColorFrame);

        return FrameworkReturnCode::_SUCCESS;
}


Point3Df SolARRGBDCamera::getPixelToWorld(const Point2Di& inPixel) const
{
        Point3Df outputPoint;

        if (m_frameset.size() == 0)
                return outputPoint;

        // rs2::align allows you to perform alignment of depth frames to others
        rs2::align align(RS2_STREAM_COLOR);
        auto aligned = align.process(m_frameset);
        auto aligned_depth_frame = aligned.get_depth_frame();
        const auto w = aligned_depth_frame.get_width();
        const auto h = aligned_depth_frame.get_height();

        const auto pixels_depth = static_cast<const uint16_t*>(aligned_depth_frame.get_data());

        pixelToWorld(pixels_depth, w, h, inPixel.getX(), inPixel.getY(),
                outputPoint);

        return outputPoint;
}

Point2Di SolARRGBDCamera::getWorldToPixel(const CloudPoint& in3DPoint) const
{
        Point2Di pixel;
        if (m_color_intrin.width == 0 || m_color_intrin.height == 0) // intrinsics not initialized
                return pixel;

        float color_pixel[2];
        float color_point[3];
        float depth_point[3] = { in3DPoint.getX(),in3DPoint.getY(), in3DPoint.getZ() };
        rs2_transform_point_to_point(color_point, &m_depth_to_color, depth_point);
        rs2_project_point_to_pixel(color_pixel, &m_color_intrin, color_point);

        const auto roundedX = static_cast<int>(round(color_pixel[0]));
        const auto roundedY = static_cast<int>(round(color_pixel[1]));

        pixel.setX(roundedX);
        pixel.setY(roundedY);

        return pixel;
}

std::vector<Point2Df> SolARRGBDCamera::getWorldToPixels (const std::vector<SRef<datastructure::CloudPoint>>& in3DPoints) const
{
        std::vector<Point2Df> out2DPoints;

        for( const auto& pt : in3DPoints )
        {
            auto pti = getWorldToPixel( *pt );
            out2DPoints.push_back(Point2Df(pti.x(), pti.y()));
        }

        return out2DPoints;
}

void SolARRGBDCamera::pixelToWorld(const uint16_t* pixels_depth,
        const int w, const int h, const int i,
        const int j, Point3Df& point) const
{
        float color_point[3];
        float depth_point[3];

        const float color_pixel[2] = { static_cast<float>(i), static_cast<float>(j) };
        const int inc = color_pixel[1] * w + color_pixel[0];

        if (inc > -1 && inc < ((w - 1)*(h - 1)))
        {
                const auto color_value_in_color_frame = pixels_depth[inc];
                const float distance = color_value_in_color_frame * m_depth_scale;
                rs2_deproject_pixel_to_point(color_point, &m_color_intrin, color_pixel, distance);
                rs2_transform_point_to_point(depth_point, &m_color_to_depth, color_point);

                point.setX(depth_point[0]);
                point.setY(depth_point[1]);
                point.setZ(depth_point[2]);
        }
        else
        {
                point.setX(0);
                point.setY(0);
                point.setZ(0);
        }
}


void SolARRGBDCamera::setResolution(const Sizei & resolution)
{
	m_rgb_camera_information.size = resolution;
}

FrameworkReturnCode SolARRGBDCamera::setDepthResolution(Sizei resolution)
{
	m_depth_camera_information.size = resolution;
	return FrameworkReturnCode::_SUCCESS;
}

void SolARRGBDCamera::setIntrinsicParameters(ATTRIBUTE(maybe_unused) const CamCalibration & intrinsic_parameters)
{

}

FrameworkReturnCode SolARRGBDCamera::setIntrinsicDepthParameters(ATTRIBUTE(maybe_unused) const CamCalibration & intrinsic_parameters)
{
     return FrameworkReturnCode::_NOT_IMPLEMENTED;
}

void SolARRGBDCamera::setDistortionParameters(const CamDistortion & distortion_parameters)
{
	m_parameters.distortion = distortion_parameters;
}

FrameworkReturnCode SolARRGBDCamera::setDistortionDepthParameters(const CamDistortion & distortion_parameters)
{
	m_depth_camera_information.distortion = distortion_parameters;
    return FrameworkReturnCode::_SUCCESS;
}


void SolARRGBDCamera::setParameters(const CameraParameters & parameters)
{
	m_parameters = parameters;
}

Sizei SolARRGBDCamera::getResolution() const
{
    if (!m_is_opened)
            return { 0, 0 };

    return { static_cast<uint32_t>(m_color_intrin.width),
            static_cast<uint32_t>(m_color_intrin.height) };
}

Sizei SolARRGBDCamera::getDepthResolution() const
{
        if (!m_is_opened)
                return { 0, 0 };

        return { static_cast<uint32_t>(m_depth_intrin.width),
                static_cast<uint32_t>(m_depth_intrin.height) };
}

float SolARRGBDCamera::getDepthMinDistance() const
{
	return m_depth_minimum_distance;
}

const CamCalibration & SolARRGBDCamera::getIntrinsicsParameters() const {
	return m_rgb_camera_information.calibration;
}

const CameraParameters & SolARRGBDCamera::getParameters() const {
	return m_parameters;
}

const CamCalibration& SolARRGBDCamera::getIntrinsicsDepthParameters() const
{
    return m_depth_camera_information.calibration;
}

const CamDistortion& SolARRGBDCamera::getDistortionParameters() const
{
	return m_rgb_camera_information.distortion;
}

const CamDistortion& SolARRGBDCamera::getDistortionDepthParameters() const
{
    return m_depth_camera_information.distortion;
}

}
}
}
