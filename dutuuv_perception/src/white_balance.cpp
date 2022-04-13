#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <string>
#include <vector> 

static const std::string OPENCV_WINDOW = "Image enhanced";

class ImageEnhancer
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub;
    image_transport::Subscriber image_sub;

public:
    ImageEnhancer(/* args */);
    ~ImageEnhancer();
    void grayWorld(cv::Mat& imageSource);
    void imageCb(const sensor_msgs::ImageConstPtr &msg);
};

void ImageEnhancer::imageCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    grayWorld(cv_ptr -> image);
    
    image_pub.publish(cv_ptr -> toImageMsg());

}

ImageEnhancer::ImageEnhancer(/* args */) : it_(nh_)
{
    image_pub = it_.advertise("image_enhanced", 5);
    image_sub = it_.subscribe("/tank/tank/camera/camera_image", 5, &ImageEnhancer::imageCb, this);
    cv::namedWindow(OPENCV_WINDOW);
}

ImageEnhancer::~ImageEnhancer()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

void ImageEnhancer::grayWorld(cv::Mat& imageSource) {
	
	cv::imshow("原始图像", imageSource);
	std::vector<cv::Mat> imageRGB;
 
	//RGB三通道分离
	cv::split(imageSource, imageRGB);
 
	//求原始图像的RGB分量的均值
	double R, G, B;
	B = mean(imageRGB[0])[0];
	G = mean(imageRGB[1])[0];
	R = mean(imageRGB[2])[0];
 
	//需要调整的RGB分量的增益
	double KR, KG, KB;
	KB = (R + G + B) / (3 * B);
	KG = (R + G + B) / (3 * G);
	KR = (R + G + B) / (3 * R);
 
	//调整RGB三个通道各自的值
	imageRGB[0] = imageRGB[0] * KB;
	imageRGB[1] = imageRGB[1] * KG;
	imageRGB[2] = imageRGB[2] * KR;
 
	//RGB三通道图像合并
	cv::merge(imageRGB, imageSource);
	cv::imshow("白平衡调整后", imageSource);
	cv::waitKey(1);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_enhancement");
    ImageEnhancer image_enhancer;
    ros::spin();
}