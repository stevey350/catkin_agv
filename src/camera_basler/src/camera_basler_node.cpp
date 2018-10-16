

#include <pylon/PylonIncludes.h>
#ifdef PYLON_WIN_BUILD
#    include <pylon/PylonGUI.h>
#endif
#include <pylon/CameraEventHandler.h>
#include <pylon/ImageFormatConverter.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
using namespace Pylon;
using namespace cv;
using namespace std;


// The name of the pylon feature stream file.
const char Filename[] = "NodeMap1.pfs";

int main(int argc, char* argv[])
{
    int exitCode = 0;

    ros::init(argc, argv, "basler_camera");
    ros::NodeHandle nh("~");

    string frame_id;
    if(!nh.getParam("frame_id", frame_id))
      frame_id = "camera";

    image_transport::ImageTransport it_(nh);
    image_transport::CameraPublisher cam_pub;
    cam_pub = it_.advertiseCamera("image_rect", 1);

    ros::Rate loop_rate(50);

    // cinfo
    std::string camera_name = nh.getNamespace();        // ??
    std::string camera_info_url = "file:///home/nvidia/catkin_agv/src/camera_basler/config/cam_calibration_config.yaml";

    camera_info_manager::CameraInfoManager cinfo_manager_(nh, camera_name);
    cinfo_manager_.loadCameraInfo(camera_info_url);
    sensor_msgs::CameraInfo::Ptr cinfo(new sensor_msgs::CameraInfo(cinfo_manager_.getCameraInfo()));


//    cinfo->D = std::vector<double>(5, 0.);
    ROS_INFO_STREAM("gg"<<*cinfo);
//    std::cout << "calibration D" << cinfo->D[0]<<endl;
//    std::cout << "calibration K" << cinfo->K <<endl;
//    std::cout << "calibration R" << cinfo->R <<endl;
//    std::cout << "calibration P" << cinfo->P <<endl;
    ros::Time timestamp;
    // Before using any pylon methods, the pylon runtime must be initialized. 
    PylonInitialize();

    try
    {
        // Create an instant camera object with the camera device found first.
        CInstantCamera camera( CTlFactory::GetInstance().CreateFirstDevice());

        // Print the model name of the camera.
        cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;

        camera.Open();
        // The parameter MaxNumBuffer can be used to control the count of buffers
        // allocated for grabbing. The default value of this parameter is 10.
        camera.MaxNumBuffer = 5;

        GenApi::INodeMap& nodemap = camera.GetNodeMap();
        GenApi::CIntegerPtr ptrInter_Packet_Delay( nodemap.GetNode("GevSCPD"));
        if(IsWritable(ptrInter_Packet_Delay))
        {
            ptrInter_Packet_Delay->SetValue(8825);
        }

//        CFeaturePersistence::Save( Filename, &camera.GetNodeMap() );
//        CFeaturePersistence::Load( Filename, &camera.GetNodeMap(), true );


        camera.StartGrabbing();

        // This smart pointer will receive the grab result data.
        CGrabResultPtr ptrGrabResult;
        CImageFormatConverter converter_;
        CPylonImage pylon_image_;

        GenApi::CIntegerPtr width(camera.GetNodeMap().GetNode("Width"));
        GenApi::CIntegerPtr height(camera.GetNodeMap().GetNode("Height"));
        cv::Mat cv_img_rgb(width->GetValue(), height->GetValue(), CV_8UC1);// mono8 --> CV_8UC1 ; rgb8 --> CV_8UC3  ZXP
        cv::Mat cv_image_resize;
        while ( camera.IsGrabbing() && ros::ok())
        {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            camera.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);

            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                converter_.Convert(pylon_image_, ptrGrabResult);

                cv_img_rgb = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1,(uint8_t*)pylon_image_.GetBuffer()); // mono8 --> CV_8UC1 ; rgb8 --> CV_8UC3  ZXP
                cv::resize(cv_img_rgb, cv_image_resize, Size(cinfo->width, cinfo->height) );

                cv::imshow("camera basler window2", cv_image_resize);
                waitKey(1);

                sensor_msgs::ImagePtr image = cv_bridge::CvImage(std_msgs::Header(), "mono8", cv_image_resize).toImageMsg();  //"Mono8""rgb8" //Note: must be lowercase ZXP


                timestamp = ros::Time::now();
                image->header.frame_id = frame_id;
                image->header.stamp = timestamp;
                cinfo->header.frame_id = frame_id;
                cinfo->header.stamp = timestamp;

                cam_pub.publish(image, cinfo);
                ros::spinOnce();

                //cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
                //cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;
            }
            else
            {
                cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
            }

            loop_rate.sleep();
        }

        camera.Close();
    }
    catch (const GenericException &e)
    {
        // Error handling.
        cerr << "An exception occurred." << endl
        << e.GetDescription() << endl;
        exitCode = 1;
    }

    // Releases all pylon resources. 
    PylonTerminate();  

    return exitCode;
}
