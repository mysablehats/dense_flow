#include <ros/ros.h>
#include "dense_flow.h"
#include "opencv2/gpu/gpu.hpp"
using namespace cv::gpu;

void rosCalcDenseFlowGPU(string file_name, int bound, int type, int step, int dev_id,
                      vector<vector<uchar> >& output_x,
                      vector<vector<uchar> >& output_y,
                      vector<vector<uchar> >& output_img,
                      int new_width=0, int new_height=0);

INITIALIZE_EASYLOGGINGPP

using namespace cv::gpu;

int main(int argc, char** argv){

	ros::init(argc, argv, "df_publisher", ros::init_options::AnonymousName);

	ros::NodeHandle local_nh("~");

	string vidFile;
	string xFlowFile;
	string yFlowFile;
	string imgFile;
	int bound;
  int type;
  int device_id;
  int step;
  int new_height;
  int new_width;

	local_nh.param("vidFile", vidFile, std::string("input.avi"));
	local_nh.param("xFlowFile", xFlowFile, std::string("flow_x"));
	local_nh.param("yFlowFile", yFlowFile, std::string("flow_y"));
	local_nh.param("imgFile", imgFile, std::string("img"));
	local_nh.param("bound", bound, 15);
	local_nh.param("type", type, 1); //TODO:this should be a string and compare to tvl1, farn and brox
	local_nh.param("device_id", device_id, 0);
	local_nh.param("step", step, 1);
	local_nh.param("new_height", new_height, 0);
	local_nh.param("new_width", new_width, 0);

	vector<vector<uchar> > out_vec_x, out_vec_y, out_vec_img;

	rosCalcDenseFlowGPU(vidFile, bound, type, step, device_id,
					 out_vec_x, out_vec_y, out_vec_img, new_width, new_height);

		writeImages(out_vec_x, xFlowFile);
		writeImages(out_vec_y, yFlowFile);
		writeImages(out_vec_img, imgFile);

	return 0;
}

void rosCalcDenseFlowGPU(string file_name, int bound, int type, int step, int dev_id,
                      vector<vector<uchar> >& output_x,
                      vector<vector<uchar> >& output_y,
                      vector<vector<uchar> >& output_img,
                      int new_width, int new_height){
    VideoCapture video_stream(file_name);
    CHECK(video_stream.isOpened())<<"Cannot open video stream \""
                                  <<file_name
                                  <<"\" for optical flow extraction.";

    setDevice(dev_id);
    Mat capture_frame, capture_image, prev_image, capture_gray, prev_gray;
    Mat flow_x, flow_y;
    Size new_size(new_width, new_height);

    GpuMat d_frame_0, d_frame_1;
    GpuMat d_flow_x, d_flow_y;

    FarnebackOpticalFlow alg_farn;
    OpticalFlowDual_TVL1_GPU alg_tvl1;
    BroxOpticalFlow alg_brox(0.197f, 50.0f, 0.8f, 10, 77, 10);

    bool do_resize = (new_height > 0) && (new_width > 0);

    bool initialized = false;
    int cnt = 0;
    while(true){

        //build mats for the first frame
        if (!initialized){
           video_stream >> capture_frame;
           if (capture_frame.empty()) return; // read frames until end

            if (!do_resize){
                initializeMats(capture_frame, capture_image, capture_gray,
                           prev_image, prev_gray);
                capture_frame.copyTo(prev_image);
            }else{
                capture_image.create(new_size, CV_8UC3);
                capture_gray.create(new_size, CV_8UC1);
                prev_image.create(new_size, CV_8UC3);
                prev_gray.create(new_size, CV_8UC1);
                cv::resize(capture_frame, prev_image, new_size);
            }
            cvtColor(prev_image, prev_gray, CV_BGR2GRAY);
            initialized = true;
            for(int s = 0; s < step; ++s){
                video_stream >> capture_frame;
		cnt ++;
                if (capture_frame.empty()) return; // read frames until end
            }
        }else {
            if (!do_resize)
                capture_frame.copyTo(capture_image);
            else
                cv::resize(capture_frame, capture_image, new_size);

            cvtColor(capture_image, capture_gray, CV_BGR2GRAY);
            d_frame_0.upload(prev_gray);
            d_frame_1.upload(capture_gray);

            switch(type){
                case 0: {
                    alg_farn(d_frame_0, d_frame_1, d_flow_x, d_flow_y);
                    break;
                }
                case 1: {
                    alg_tvl1(d_frame_0, d_frame_1, d_flow_x, d_flow_y);
                    break;
                }
                case 2: {
                    GpuMat d_buf_0, d_buf_1;
                    d_frame_0.convertTo(d_buf_0, CV_32F, 1.0 / 255.0);
                    d_frame_1.convertTo(d_buf_1, CV_32F, 1.0 / 255.0);
                    alg_brox(d_buf_0, d_buf_1, d_flow_x, d_flow_y);
                    break;
                }
                default:
                    LOG(ERROR)<<"Unknown optical method: "<<type;
            }

            //prefetch while gpu is working
            bool hasnext = true;
            for(int s = 0; s < step; ++s){
                video_stream >> capture_frame;
		cnt ++;
                hasnext = !capture_frame.empty();
                // read frames until end
            }

            //get back flow map
            d_flow_x.download(flow_x);
            d_flow_y.download(flow_y);

            vector<uchar> str_x, str_y, str_img;
            encodeFlowMap(flow_x, flow_y, str_x, str_y, bound);
            imencode(".jpg", capture_image, str_img);

            output_x.push_back(str_x);
            output_y.push_back(str_y);
            output_img.push_back(str_img);

            std::swap(prev_gray, capture_gray);
            std::swap(prev_image, capture_image);

            if (!hasnext){
                return;
            }
        }


    }

}