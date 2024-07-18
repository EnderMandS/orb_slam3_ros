/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono_inertial.cc and ros_rgbd.cc
*
*/

#include "common.h"

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);
    void sortImuQueue();

    queue<sensor_msgs::ImuConstPtr> imuBuf, imuBuf_front;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ImuGrabber *pImuGb): mpImuGb(pImuGb){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    sensor_msgs::ImageConstPtr imgRGB=nullptr, imgD=nullptr;
    std::mutex mBufMutex;
    ImuGrabber *mpImuGb;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD_Inertial");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    std::string node_name = ros::this_node::getName();

    ros::NodeHandle node_handler;
    image_transport::ImageTransport image_transport(node_handler);

    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");       
        ros::shutdown();
        return 1;
    }

    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "world");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");
    node_handler.param<std::string>(node_name + "/imu_frame_id", imu_frame_id, "imu");

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::IMU_RGBD;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, false);

    ImuGrabber imugb;
    ImageGrabber igb(&imugb);

    ros::Subscriber sub_imu = node_handler.subscribe("imu", 1000, &ImuGrabber::GrabImu, &imugb);

    message_filters::Subscriber<sensor_msgs::Image> sub_rgb_img(node_handler, "camera/rgb/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth_img(node_handler, "camera/depth_registered/image_raw", 100);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), sub_rgb_img, sub_depth_img);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    setup_publishers(node_handler, image_transport, node_name);
    setup_services(node_handler, node_name);

    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    ros::spin();

    // Stop all threads
    pSLAM->Shutdown();
    ros::shutdown();

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    mBufMutex.lock();
    imgRGB = msgRGB;
    imgD = msgD;
    mBufMutex.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    return cv_ptr->image.clone();
}

void ImageGrabber::SyncWithImu()
{
    std::chrono::milliseconds tSleep(1);
    while(1)
    {
        if (this->mBufMutex.try_lock()) {
            if (nullptr!=imgRGB && nullptr!=imgD) {
                if(mpImuGb->mBufMutex.try_lock()) {
                    if (mpImuGb->imuBuf.size()>1) {
                        cv::Mat im, depth;
                        double tIm = imgRGB->header.stamp.toSec();  // image time
                        if (tIm < mpImuGb->imuBuf.front()->header.stamp.toSec()) {  // drop image without imu
                            ROS_WARN("Skip image without imu.");
                            imgRGB=nullptr;
                            imgD=nullptr;
                            this->mBufMutex.unlock();
                            mpImuGb->mBufMutex.unlock();
                        } else {
                            ros::Time msg_time = imgRGB->header.stamp;
                            im = GetImage(imgRGB);
                            depth = GetImage(imgD);
                            imgRGB=nullptr;
                            imgD=nullptr;
                            this->mBufMutex.unlock();

                            vector<ORB_SLAM3::IMU::Point> vImuMeas;
                            vImuMeas.clear();
                            Eigen::Vector3f Wbb;
                            mpImuGb->sortImuQueue();
                            // Load imu measurements from buffer
                            while(mpImuGb->imuBuf.front()->header.stamp.toSec() <= tIm && !mpImuGb->imuBuf.empty()) {
                                cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
                                cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
                                double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                                vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                                Wbb << mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z;
                                mpImuGb->imuBuf.pop();
                            }
                            mpImuGb->mBufMutex.unlock();
                            // ORB-SLAM3 runs in TrackRGBD()
                            if (vImuMeas.empty()) {
                                ROS_WARN("vImuMeas empty");
                                continue;
                            }
                            pSLAM->TrackRGBD(im, depth, tIm, vImuMeas);
                            publish_topics(msg_time, Wbb);
                        }
                    } else {
                        mpImuGb->mBufMutex.unlock();
                    }
                }
            } else {
                this->mBufMutex.unlock();
            }
        }
        std::this_thread::sleep_for(tSleep);
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    imuBuf_front.push(imu_msg);
    if (mBufMutex.try_lock()) {
        while (!imuBuf_front.empty()) {
            imuBuf.push(imuBuf_front.front());
            imuBuf_front.pop();
        }
        mBufMutex.unlock();
    }
}

void ImuGrabber::sortImuQueue() {
    // Transfer elements from queue to vector
    std::vector<sensor_msgs::ImuConstPtr> vec;
    while (!imuBuf.empty()) {
        vec.push_back(imuBuf.front());
        imuBuf.pop();
    }

    // Sort vector based on timestamp
    std::sort(vec.begin(), vec.end(),  [](sensor_msgs::ImuConstPtr a, sensor_msgs::ImuConstPtr b){
        return a->header.stamp.toSec() < b->header.stamp.toSec();
    });

    // Transfer sorted elements back to queue
    for (const auto& imu : vec) {
        imuBuf.push(imu);
    }
}
