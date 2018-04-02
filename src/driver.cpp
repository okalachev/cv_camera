// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/driver.h"
#include <string>

namespace
{
const double DEFAULT_RATE = 30.0;
const int32_t PUBLISHER_BUFFER_SIZE = 1;
}

namespace cv_camera
{

Driver::Driver(ros::NodeHandle &private_node, ros::NodeHandle &camera_node)
    : private_node_(private_node),
      camera_node_(camera_node)
{
}

void Driver::setup()
{
  double hz(DEFAULT_RATE);
  int32_t device_id(0);
  std::string device_path("");
  std::string frame_id("camera");
  std::string file_path("");

  private_node_.getParam("device_id", device_id);
  private_node_.getParam("frame_id", frame_id);
  private_node_.getParam("rate", hz);

  int32_t image_width(640);
  int32_t image_height(480);

  camera_.reset(new Capture(camera_node_,
                            "image_raw",
                            PUBLISHER_BUFFER_SIZE,
                            frame_id));

  if (private_node_.getParam("file", file_path) && file_path != "")
  {
    camera_->openFile(file_path);
  }
  else if (private_node_.getParam("device_path", device_path) && device_path != "")
  {
    camera_->open(device_path);
  }
  else
  {
    camera_->open(device_id);
  }
  if (private_node_.getParam("image_width", image_width))
  {
    if (!camera_->setWidth(image_width))
    {
      ROS_WARN("fail to set image_width");
    }
  }
  if (private_node_.getParam("image_height", image_height))
  {
    if (!camera_->setHeight(image_height))
    {
      ROS_WARN("fail to set image_height");
    }
  }

  camera_->setPropertyFromParam(CV_CAP_PROP_POS_MSEC, "cv_cap_prop_pos_msec");
  camera_->setPropertyFromParam(CV_CAP_PROP_POS_AVI_RATIO, "cv_cap_prop_pos_avi_ratio");
  camera_->setPropertyFromParam(CV_CAP_PROP_FRAME_WIDTH, "cv_cap_prop_frame_width");
  camera_->setPropertyFromParam(CV_CAP_PROP_FRAME_HEIGHT, "cv_cap_prop_frame_height");
  camera_->setPropertyFromParam(CV_CAP_PROP_FPS, "cv_cap_prop_fps");
  camera_->setPropertyFromParam(CV_CAP_PROP_FOURCC, "cv_cap_prop_fourcc");
  camera_->setPropertyFromParam(CV_CAP_PROP_FRAME_COUNT, "cv_cap_prop_frame_count");
  camera_->setPropertyFromParam(CV_CAP_PROP_FORMAT, "cv_cap_prop_format");
  camera_->setPropertyFromParam(CV_CAP_PROP_MODE, "cv_cap_prop_mode");
  camera_->setPropertyFromParam(CV_CAP_PROP_BRIGHTNESS, "cv_cap_prop_brightness");
  camera_->setPropertyFromParam(CV_CAP_PROP_CONTRAST, "cv_cap_prop_contrast");
  camera_->setPropertyFromParam(CV_CAP_PROP_SATURATION, "cv_cap_prop_saturation");
  camera_->setPropertyFromParam(CV_CAP_PROP_HUE, "cv_cap_prop_hue");
  camera_->setPropertyFromParam(CV_CAP_PROP_GAIN, "cv_cap_prop_gain");
  camera_->setPropertyFromParam(CV_CAP_PROP_EXPOSURE, "cv_cap_prop_exposure");
  camera_->setPropertyFromParam(CV_CAP_PROP_CONVERT_RGB, "cv_cap_prop_convert_rgb");

  camera_->setPropertyFromParam(CV_CAP_PROP_RECTIFICATION, "cv_cap_prop_rectification");
  camera_->setPropertyFromParam(CV_CAP_PROP_ISO_SPEED, "cv_cap_prop_iso_speed");
#ifdef CV_CAP_PROP_WHITE_BALANCE_U
  camera_->setPropertyFromParam(CV_CAP_PROP_WHITE_BALANCE_U, "cv_cap_prop_white_balance_u");
#endif // CV_CAP_PROP_WHITE_BALANCE_U
#ifdef CV_CAP_PROP_WHITE_BALANCE_V
  camera_->setPropertyFromParam(CV_CAP_PROP_WHITE_BALANCE_V, "cv_cap_prop_white_balance_v");
#endif // CV_CAP_PROP_WHITE_BALANCE_V
#ifdef CV_CAP_PROP_BUFFERSIZE
  camera_->setPropertyFromParam(CV_CAP_PROP_BUFFERSIZE, "cv_cap_prop_buffersize");
#endif // CV_CAP_PROP_BUFFERSIZE

  rate_.reset(new ros::Rate(hz));

  set_config_ = private_node_.advertiseService("set_config", &Driver::setConfig, this);
}

void Driver::proceed()
{
  if (camera_->capture())
  {
    camera_->publish();
  }
  rate_->sleep();
}

bool Driver::setConfig(cv_camera::SetConfig::Request& request, cv_camera::SetConfig::Response& response)
{
  if (std::isfinite(request.image_width) && std::isfinite(request.image_height))
  {
    // Set image width and height
    if (!camera_->setWidth(request.image_width))
    {
      ROS_WARN("fail to set image_width");
      response.message = "failed to set image_width";
      return true;
    }
    if (!camera_->setHeight(request.image_height))
    {
      ROS_WARN("fail to set image_height");
      response.message = "failed to set image_height";
      return true;
    }
  }
  if (std::isfinite(request.cv_cap_prop_fps))
  {
    if (!camera_->setProperty(CV_CAP_PROP_FPS, request.cv_cap_prop_fps)) {
      ROS_WARN("fail to set image_height");
      response.message = "failed to set fps";
      return true;
    }
  }
  return true;
}

Driver::~Driver()
{
}

} // namespace cv_camera
