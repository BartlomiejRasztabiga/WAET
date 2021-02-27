/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "turtlesim/turtle_frame.h"
#include "turtlesim/Pose.h"

#include <QPointF>

#include <ros/package.h>
#include <cstdlib>
#include <ctime>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#define DEFAULT_BG_R 0x45
#define DEFAULT_BG_G 0x56
#define DEFAULT_BG_B 0xff
#define DEFAULT_WIDTH 1920
#define DEFAULT_HEIGHT 1080
#define DEFAULT_PNG "roads.png"

namespace turtlesim
{

TurtleFrame::TurtleFrame(QWidget* parent, Qt::WindowFlags f)
: QFrame(parent, f)
, path_image_(QImage(QString(ros::package::getPath("turtlesim").c_str())+QString("/images/")+QString(DEFAULT_PNG)).scaled(DEFAULT_WIDTH, DEFAULT_HEIGHT, Qt::KeepAspectRatio))
, path_painter_(&path_image_)
, frame_count_(0)
, id_counter_(0)
, private_nh_("~")
{
  setFixedSize(DEFAULT_WIDTH, DEFAULT_HEIGHT);
  setWindowTitle("TurtleSim");
  QImage start_image_ = path_image_;
  srand(time(NULL));

  update_timer_ = new QTimer(this);
  update_timer_->setInterval(16);
  update_timer_->start();

  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

  if (!private_nh_.hasParam("background_r"))
  {
    private_nh_.setParam("background_r", DEFAULT_BG_R);
  }
  if (!private_nh_.hasParam("background_g"))
  {
    private_nh_.setParam("background_g", DEFAULT_BG_G);
  }
  if (!private_nh_.hasParam("background_b"))
  {
    private_nh_.setParam("background_b", DEFAULT_BG_B);
  }
  if (!private_nh_.hasParam("background_png"))
  {
  	std::string png_path_default = ros::package::getPath("turtlesim")+"/images/"+ DEFAULT_PNG;
    private_nh_.setParam("background_png", png_path_default);
  }

  QVector<QString> turtles;
  turtles.append("box-turtle.png");
  turtles.append("robot-turtle.png");
  turtles.append("sea-turtle.png");
  turtles.append("diamondback.png");
  turtles.append("electric.png");
  turtles.append("fuerte.png");
  turtles.append("groovy.png");
  turtles.append("hydro.svg");
  turtles.append("indigo.svg");
  turtles.append("jade.png");
  turtles.append("kinetic.png");
  turtles.append("lunar.png");
  turtles.append("melodic.png");
  turtles.append("noetic.png");

  QString images_path = (ros::package::getPath("turtlesim") + "/images/").c_str();
  for (int i = 0; i < turtles.size(); ++i)
  {
    QImage img;
    img.load(images_path + turtles[i]);
    turtle_images_.append(img);
  }

  meter_ = turtle_images_[0].height();

  clear();

  clear_srv_ = nh_.advertiseService("clear", &TurtleFrame::clearCallback, this);
  reset_srv_ = nh_.advertiseService("reset", &TurtleFrame::resetCallback, this);
  spawn_srv_ = nh_.advertiseService("spawn", &TurtleFrame::spawnCallback, this);
  kill_srv_ = nh_.advertiseService("kill", &TurtleFrame::killCallback, this);
  get_turtles_srv_ = nh_.advertiseService("get_turtles", &TurtleFrame::getTurtlesCallback, this);
  get_pose_srv_ = nh_.advertiseService("get_pose", &TurtleFrame::getPoseCallback, this);
  get_camera_image_srv_ = nh_.advertiseService("get_camera_image", &TurtleFrame::getCameraImageCallback, this);

  ROS_INFO("Starting turtlesim with node name %s", ros::this_node::getName().c_str()) ;

  width_in_meters_ = (width() - 1) / meter_;
  height_in_meters_ = (height() - 1) / meter_;
  spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);

  // spawn all available turtle types
  if(false)
  {
    for(int index = 0; index < turtles.size(); ++index)
    {
      QString name = turtles[index];
      name = name.split(".").first();
      name.replace(QString("-"), QString(""));
      spawnTurtle(name.toStdString(), 1.0 + 1.5 * (index % 7), 1.0 + 1.5 * (index / 7), PI / 2.0, index);
    }
  }
}

TurtleFrame::~TurtleFrame()
{
  delete update_timer_;
}

bool TurtleFrame::spawnCallback(turtlesim::Spawn::Request& req, turtlesim::Spawn::Response& res)
{
  std::string name = spawnTurtle(req.name, req.x, req.y, req.theta);
  if (name.empty())
  {
    ROS_ERROR("A turtled named [%s] already exists", req.name.c_str());
    return false;
  }

  res.name = name;

  return true;
}

bool TurtleFrame::killCallback(turtlesim::Kill::Request& req, turtlesim::Kill::Response&)
{
  M_Turtle::iterator it = turtles_.find(req.name);
  if (it == turtles_.end())
  {
    ROS_ERROR("Tried to kill turtle [%s], which does not exist", req.name.c_str());
    return false;
  }

  turtles_.erase(it);
  update();

  return true;
}

bool TurtleFrame::hasTurtle(const std::string& name)
{
  return turtles_.find(name) != turtles_.end();
}

std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float angle)
{
  return spawnTurtle(name, x, y, angle, rand() % turtle_images_.size());
}

bool TurtleFrame::getCameraImageCallback(turtlesim::GetCameraImage::Request& req, turtlesim::GetCameraImage::Response& res)
{
	TurtlePtr t = turtles_[req.name];
	turtlesim::Pose pose = t->getPose();
    ROS_ERROR("pose.theta: %f",pose.theta);
	if (pose.theta < 0)
	{
		pose.theta = 2*M_PI + pose.theta;
	}
    ROS_ERROR("pose.theta: %f",pose.theta);
    ROS_ERROR("cos: %f",cos(pose.theta));
    ROS_ERROR("sin: %f",sin(pose.theta));
    pose.y = height_in_meters_ - pose.y;
	float x = floor(cos(pose.theta)*(pose.x+0-pose.x)*meter_ - sin(pose.theta)*(pose.y+0-pose.y)*meter_)+pose.x*meter_;
	float y = floor(cos(pose.theta)*(pose.y+0-pose.y)*meter_ + sin(pose.theta)*(pose.x+0-pose.x)*meter_)+ pose.y*meter_;
	QRect rect(y, x, 10*meter_, 10*meter_);
    ROS_ERROR("x: %f | y: %f | meter: %f",x,y,meter_);
	QPixmap original(QPixmap::fromImage(path_image_));
	QFile file3("oryg.png");
	file3.open(QIODevice::WriteOnly);
	path_image_.save(&file3, "PNG");

	QPixmap cropped = original.copy(rect);
	QFile file("yourFile.png");
	file.open(QIODevice::WriteOnly);
	cropped.save(&file, "PNG");
	QTransform rot_matrix;
	rot_matrix = rot_matrix.rotateRadians(pose.theta,Qt::ZAxis);
	// rot_matrix.rotate();
	QImage clippedImage = path_image_.transformed(rot_matrix);//.copy(rect).transformed(rot_matrix.inverted());
	QFile file2("yourFile2.png");
	file2.open(QIODevice::WriteOnly);
	clippedImage.save(&file2, "PNG");

	QImage clippedImage2 = path_image_.transformed(rot_matrix).copy(rect);//.transformed(rot_matrix.inverted());
	QFile file4("yourFile3.png");
	file4.open(QIODevice::WriteOnly);
	clippedImage2.save(&file4, "PNG");

	cv::Mat original_cv(path_image_.height(), path_image_.width(),CV_8UC3, path_image_.bits()); 
switch(path_image_.format()) {
        case QImage::Format_Invalid:
        {
            cv::Mat empty;
            empty.copyTo(original_cv);
            break;
        }
        case QImage::Format_RGB32:
        {
		    ROS_ERROR("format: Format_RGB32");
            cv::Mat view(path_image_.height(),path_image_.width(),CV_8UC4,(void *)path_image_.constBits(),path_image_.bytesPerLine());
            view.copyTo(original_cv);
            break;
        }
        case QImage::Format_RGB888:
        {
		    ROS_ERROR("format: Format_RGB888");
            cv::Mat view(path_image_.height(),path_image_.width(),CV_8UC3,(void *)path_image_.constBits(),path_image_.bytesPerLine());
            cvtColor(view, original_cv, cv::COLOR_RGB2BGR);
            break;
        }
        default:
        {
		    ROS_ERROR("format: Format_ARGB32");
            QImage conv = path_image_.convertToFormat(QImage::Format_ARGB32);
            cv::Mat view(conv.height(),conv.width(),CV_8UC4,(void *)conv.constBits(),conv.bytesPerLine());
            view.copyTo(original_cv);
            break;
        }
    }
	// cv::imshow("image",original_cv);
	// cv::waitKey();

	int frame_size = 200;
	float boundary = sqrt(frame_size*frame_size+frame_size*frame_size);
	float target_width =original_cv.cols+2*boundary;
	cv::Mat dst;
	int borderType = cv::BORDER_CONSTANT;
	cv::copyMakeBorder( original_cv, dst, boundary, boundary, boundary, boundary, borderType, cv::Scalar(0,0,0) );
	cv::Rect myROI_initial(pose.x*meter_,pose.y*meter_, 2*boundary, 2*boundary);
	cv::Mat dst_cropped = dst(myROI_initial);
	cv::imshow("dst_cropped",dst_cropped);

 //    cv::Rect rectBefore(0, 0, dst.cols, dst.rows);
 //    cv::Rect rectAfter(10, 10, dst.cols, dst.rows);
 //    cv::Mat dbg1 = dst.clone();
 //    // cv::rectangle(dbg1, rectBefore, cv::Scalar(0,255,0), 2);
	// cv::imshow("dbg1",dbg1);
 //    cv::Mat roiBefore = dst(rectBefore).clone();  // Copy the data in the source position
	// cv::imshow("roiBefore",roiBefore);
 //    cv::Mat roiAfter = dst(rectAfter); 
	// cv::imshow("roiAfter",roiAfter);
 //    roiBefore.copyTo(roiAfter);
 //    cv::Mat dbg2 = dst.clone();
 //    // rectangle(dbg2, rectAfter, cv::Scalar(0,0,255), 2);

	// cv::imshow("dbg2",dbg2);

	cv::Rect myROI(boundary,boundary-100, frame_size, frame_size);
	cv::Point2f center (boundary,boundary);
	cv::Mat rot_mat = cv::getRotationMatrix2D (center, -pose.theta*180/M_PI,1);
	cv::Mat dst_cropped_cropped = dst_cropped(myROI);
	cv::imshow("dst_cropped_cropped",dst_cropped_cropped);
	cv::Mat result;
	dst_cropped.copyTo(result);
	// cv::imshow("image",dst);
	// cv::waitKey();
	cv::warpAffine(dst_cropped, result,rot_mat, dst_cropped.size());
	// auto image_rect = cv::Rect({}, dst.size());
	// auto intersection =  myROI &image_rect ;
	// auto inter_roi = intersection - myROI.tl();
	// cv::Mat crop = cv::Mat::zeros(myROI.size(), dst.type());
	// dst(intersection).copyTo(crop(inter_roi));
	cv::Mat croppedImage = result(myROI);
	cv::imshow("dst2",croppedImage);
	cv_bridge::CvImage img_bridge;
	sensor_msgs::Image img_msg; // >> message to be sent

	std_msgs::Header header; // empty header
	header.seq = 0; // user defined counter
	header.stamp = ros::Time::now(); // time

	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, croppedImage);
	img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image

	res.image = img_msg;
	return true;
}

std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float angle, size_t index)
{
  std::string real_name = name;
  if (real_name.empty())
  {
    do
    {
      std::stringstream ss;
      ss << "turtle" << ++id_counter_;
      real_name = ss.str();
    } while (hasTurtle(real_name));
  }
  else
  {
    if (hasTurtle(real_name))
    {
      return "";
    }
  }

  TurtlePtr t(new Turtle(ros::NodeHandle(real_name), turtle_images_[index], QPointF(x, height_in_meters_ - y), angle));
  turtles_[real_name] = t;
  update();

  ROS_INFO("Spawning turtle [%s] at x=[%f], y=[%f], theta=[%f]", real_name.c_str(), x, y, angle);

  return real_name;
}

void TurtleFrame::clear()
{
  int r = DEFAULT_BG_R;
  int g = DEFAULT_BG_G;
  int b = DEFAULT_BG_B;
  std::string png_path = ros::package::getPath("turtlesim")+"/images/"+ DEFAULT_PNG;
  private_nh_.param("background_r", r, r);
  private_nh_.param("background_g", g, g);
  private_nh_.param("background_b", b, b);
  private_nh_.param("background_png", png_path, png_path);
  path_image_.fill(qRgb(r, g, b));
  path_painter_.setCompositionMode(QPainter::CompositionMode_SourceOver);
  path_painter_.drawImage(0, 0, QImage(png_path.c_str()).scaled(DEFAULT_WIDTH, DEFAULT_HEIGHT, Qt::KeepAspectRatio));
  update();
}

bool TurtleFrame::getTurtlesCallback(turtlesim::GetTurtles::Request& req, turtlesim::GetTurtles::Response& res)
{
	std::map<std::string, TurtlePtr>::iterator it;

	for (it = turtles_.begin(); it != turtles_.end(); it++)
	{

	    res.list.push_back(it->first );
	}
	return true;
}

bool TurtleFrame::getPoseCallback(turtlesim::GetPose::Request& req, turtlesim::GetPose::Response& res)
{
	TurtlePtr t = turtles_[req.name];
	res.pose = t->getPose();
	return true;
}

void TurtleFrame::onUpdate()
{
  ros::spinOnce();

  updateTurtles();
  if (!ros::ok())
  {
    close();
  }
}

void TurtleFrame::paintEvent(QPaintEvent*)
{
  QPainter painter(this);

  painter.drawImage(QPoint(0, 0), path_image_);

  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it)
  {
    it->second->paint(painter);
  }
}

void TurtleFrame::updateTurtles()
{
  if (last_turtle_update_.isZero())
  {
    last_turtle_update_ = ros::WallTime::now();
    return;
  }

  bool modified = false;
  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it)
  {
    modified |= it->second->update(0.001 * update_timer_->interval(), path_painter_, path_image_, width_in_meters_, height_in_meters_);
  }
  if (modified)
  {
    update();
  }

  ++frame_count_;
}


bool TurtleFrame::clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Clearing turtlesim.");
  clear();
  return true;
}

bool TurtleFrame::resetCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Resetting turtlesim.");
  turtles_.clear();
  id_counter_ = 0;
  spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
  clear();
  return true;
}

}
