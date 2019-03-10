#include <ros/ros.h>
#include <ros/console.h>
//#include "dense_flow.h"
#include "common.h"
#include "opencv2/gpu/gpu.hpp"
//#include "opencv2/video/tracking.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//#include <stdio.h>
//#include <iostream>
#include "algtype.h"
using namespace cv::gpu;
//using namespace cv;

void rosCalcDenseFlowGPU(const sensor_msgs::ImageConstPtr& msg);

//INITIALIZE_EASYLOGGINGPP
//enum AlgType = {farn, tvl1, brox, unknown};
AlgType hashit (std::string const& inString) {
    if (inString == "farn") return farn;
    if (inString == "tvl1") return tvl1;
    if (inString == "brox") return brox;
		else return unknown;
}

string file_name;
string xFlowFile;
string yFlowFile;
string imgFile;
string type;

int bound;
int dev_id;
int step;
int new_height;
int new_width;
bool save_images;

image_transport::Publisher pub;
image_transport::Publisher pubx;
image_transport::Publisher puby;

Mat flow_img_x;
Mat flow_img_y;

vector<vector<uchar> > output_x, output_y, output_img;

cv_bridge::CvImagePtr cv_ptr;
Mat capture_image, prev_image, capture_gray, prev_gray;
Mat flow_x, flow_y;
Size new_size;

GpuMat d_frame_0, d_frame_1;
GpuMat d_flow_x, d_flow_y;

FarnebackOpticalFlow alg_farn;
OpticalFlowDual_TVL1_GPU alg_tvl1;
BroxOpticalFlow alg_brox(0.197f, 50.0f, 0.8f, 10, 77, 10);

bool do_resize;

bool initialized = false;

int main(int argc, char** argv){

	ros::init(argc, argv, "df_publisher", ros::init_options::AnonymousName);
  ROS_DEBUG("df node started. will try to load parameters and startup stuff. ");
	ros::NodeHandle nh;
	ros::NodeHandle local_nh("~");

	local_nh.param("vidFile", file_name, std::string("input.avi"));
	local_nh.param("xFlowFile", xFlowFile, std::string("flow_x"));
	local_nh.param("yFlowFile", yFlowFile, std::string("flow_y"));
	local_nh.param("imgFile", imgFile, std::string("img"));
	local_nh.param("bound", bound, 15);
	local_nh.param("type", type, std::string("farn")); //TODO:this should be a string and compare to tvl1, farn and brox
	local_nh.param("device_id", dev_id, 0);
	local_nh.param("step", step, 1);
	local_nh.param("new_height", new_height, 0);
	local_nh.param("new_width", new_width, 0);
	local_nh.param("save_images", save_images, true);
  std::string readtopic;
  local_nh.param("read_topic", readtopic, std::string("/videofiles/image_raw"));

	image_transport::ImageTransport local_it(local_nh);
	pub = local_it.advertise("image", 1);
	pubx = local_it.advertise("flow_x", 1);
	puby = local_it.advertise("flow_y", 1);


	image_transport::ImageTransport it(nh);

  ROS_INFO("Reading topic: %s",readtopic.c_str());
	image_transport::Subscriber sub = it.subscribe(readtopic, step, rosCalcDenseFlowGPU); //probably i should go for a different nodehandle here without the ~ or use the remap thing
	new_size.width = new_width;
	new_size.height = new_height;
	do_resize = (new_height > 0) && (new_width > 0);
	setDevice(dev_id);
  ROS_INFO("Defined everything ready to acquire.");
  ros::spin();

  if(save_images){
		writeImages(output_x, xFlowFile);
		writeImages(output_y, yFlowFile);
		writeImages(output_img, imgFile);
	}

	return 0;
}

void rosCalcDenseFlowGPU(const sensor_msgs::ImageConstPtr& msg){
        if (!initialized)
        {
        	ROS_INFO("Initializing...");
          try
            {
              cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
              if (cv_ptr->image.empty()) return; // if the image is empty, return and do not initialize

              if (!do_resize){
                  initializeMats(cv_ptr->image, capture_image, capture_gray,
                             prev_image, prev_gray);
                  cv_ptr->image.copyTo(prev_image);
              }else{
                  capture_image.create(new_size, CV_8UC3);
                  capture_gray.create(new_size, CV_8UC1);
                  prev_image.create(new_size, CV_8UC3);
                  prev_gray.create(new_size, CV_8UC1);
                  cv::resize(cv_ptr->image, prev_image, new_size);
              }
              cvtColor(prev_image, prev_gray, CV_BGR2GRAY);
              initialized = true;
              for(int s = 0; s < step; ++s){
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
              }
              ROS_INFO("Attempting to create flow matrices");
              flow_img_x.create(flow_x.size(), CV_8UC1);
              flow_img_y.create(flow_y.size(), CV_8UC1);
              ROS_INFO("Flow matrices created!");
            }
        catch(const std::exception &e)
            {
              ROS_ERROR("ERROR while initializing: %s", e.what());
              std::cout << e.what() << "\n";
            }
        }
      else
      {
          try
            {
              if (!do_resize)
                  cv_ptr->image.copyTo(capture_image);
              else
                  cv::resize(cv_ptr->image, capture_image, new_size);

              cvtColor(capture_image, capture_gray, CV_BGR2GRAY);
              d_frame_0.upload(prev_gray);
              d_frame_1.upload(capture_gray);

              switch(hashit(type)){
                  case farn: {
                      alg_farn(d_frame_0, d_frame_1, d_flow_x, d_flow_y);
                      break;
                  }
                  case tvl1: {
                      alg_tvl1(d_frame_0, d_frame_1, d_flow_x, d_flow_y);
                      break;
                  }
                  case brox: {
                      GpuMat d_buf_0, d_buf_1;
                      d_frame_0.convertTo(d_buf_0, CV_32F, 1.0 / 255.0);
                      d_frame_1.convertTo(d_buf_1, CV_32F, 1.0 / 255.0);
                      alg_brox(d_buf_0, d_buf_1, d_flow_x, d_flow_y);
                      break;
                  }
                  default:
                      ROS_ERROR("Unknown optical method: %s",type.c_str());
              }

              //prefetch while gpu is working
              bool hasnext = true;
              for(int s = 0; s < step; ++s){
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                hasnext = !cv_ptr->image.empty();        // read frames until end
              }

              //get back flow map
              d_flow_x.download(flow_x);
              d_flow_y.download(flow_y);

  						sensor_msgs::ImagePtr msgi = cv_bridge::CvImage(std_msgs::Header(), "bgr8", capture_image).toImageMsg();
  						pub.publish(msgi);

              //need to normalize them as well!
              //alpha = 255/(ubound - lbound) ; beta = 255* lbound/((ubound - lbound)
              // but I will simplify it, because of reasons.
              flow_x.convertTo(flow_img_x, CV_8UC1, 255/(-2*bound),255/2);
              flow_y.convertTo(flow_img_y, CV_8UC1, 255/(-2*bound),255/2);
              //ROS_INFO("so far so good");
              //convertFlowToImage_fred(flow_x, flow_y, flow_img_x, flow_img_y,
              //                   -bound, bound);
              //there is something wrong with the cast.
  						sensor_msgs::ImagePtr msgx = cv_bridge::CvImage(std_msgs::Header(), "mono8",flow_img_x).toImageMsg();
  						pubx.publish(msgx);
  						sensor_msgs::ImagePtr msgy = cv_bridge::CvImage(std_msgs::Header(), "mono8",flow_img_y).toImageMsg();
  						puby.publish(msgy);

  						if (save_images){
  	            vector<uchar> str_x, str_y, str_img;
  	            encodeFlowMap(flow_x, flow_y, str_x, str_y, bound);
  	            imencode(".jpg", capture_image, str_img);

  	            output_x.push_back(str_x);
  	            output_y.push_back(str_y);
  	            output_img.push_back(str_img);
  						}
              std::swap(prev_gray, capture_gray);
              std::swap(prev_image, capture_image);

              if (!hasnext){
                  return;
              }

        }
      catch(const std::exception &e) {
        ROS_ERROR("ERROR while running loop: %s", e.what());
        std::cout << e.what() << "\n";
        }
    }
  }
