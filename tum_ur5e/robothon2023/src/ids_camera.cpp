/**
 * @file ids_camera.cpp
 * @authors Adrian Müller (adrian.mueller@study.thws.de), 
 *          Maximilian Hornauer (maximilian.hornauer@study.thws.de),
 *          Usama Ali (usama.ali@study.thws.de)
 * @brief Program for wrapping the IDS camera library for ros
 * @version 0.1
 * @date 2023-04-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <peak/peak.hpp>
#include <peak_ipl/peak_ipl.hpp>
#include <peak/converters/peak_buffer_converter_ipl.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <chrono>


std::shared_ptr<peak::core::Device> m_device = nullptr;
std::shared_ptr<peak::core::DataStream> m_dataStream = nullptr;
std::shared_ptr<peak::core::NodeMap> m_nodemapRemoteDevice = nullptr;
 
bool OpenCamera()
{
    try
    {
        // Create instance of the device manager
        auto& deviceManager = peak::DeviceManager::Instance();
 
        // Update the device manager
        
        deviceManager.Update();
 
        // Return if no device was found
        while(deviceManager.Devices().empty())
        {
            std::cout << "No device found. Try again." << std::endl;
            deviceManager.Update();
        }
        
 
        // open the first openable device in the device manager's device list
        size_t deviceCount = deviceManager.Devices().size();
        for (size_t i = 0; i < deviceCount; ++i)
        {
            if (deviceManager.Devices().at(i)->IsOpenable())
            {
                m_device = deviceManager.Devices().at(i)->OpenDevice(peak::core::DeviceAccessType::Control);
 
                // Get NodeMap of the RemoteDevice for all accesses to the GenICam NodeMap tree
                m_nodemapRemoteDevice = m_device->RemoteDevice()->NodeMaps().at(0);  
            }
        }

        try
        {
            // print model name, not knowing if device has the node "DeviceModelName"
            std::cout
                << "IDS device found: "
                << m_nodemapRemoteDevice->FindNode<peak::core::nodes::StringNode>("DeviceModelName")->Value()
                << std::endl;
        }
        catch (const std::exception&)
        {
            // if "DeviceModelName" is not a valid node name, do error handling here...
            std::cout << "Something went wrong when connecting. Exit." << std::endl;
        }

         //load paramter file for camera
        try
        {  
            // file contains the fully qualified path to the file
            std::string path = "/home/robothon/Documents/IDS_Peak/Settings/230403_ids_settings_Auto-HQ.cset";  
            std::cout << "Try to load user settings form path: " << path << std::endl;
            // Load from file
            m_nodemapRemoteDevice->LoadFromFile(path);
            std::cout << "Successful" << std::endl;
        

        }
        catch (const std::exception&)
        {
            std::cout << "Faild loading user settings" << std::endl;
        }
  
        return true;
    }
    catch (std::exception& e)
    {
        std::cerr << "Error open camera: " << e.what() << std::endl;
    }
 
  return false;
}
 
bool PrepareAcquisition()
{
  try
  {
      auto dataStreams = m_device->DataStreams();
      if (dataStreams.empty())
      {
          // no data streams available
          return false;
      }
 
      m_dataStream = m_device->DataStreams().at(0)->OpenDataStream();
 
      return true;
  }
  catch (std::exception& e)
  {
        std::cerr << "Error prepare Acquistion: " << e.what() << std::endl;
  }
 
  return false;
}
 
 
bool AllocAndAnnounceBuffers()
{
  try
  {
      if (m_dataStream)
      {
          // Flush queue and prepare all buffers for revoking
          m_dataStream->Flush(peak::core::DataStreamFlushMode::DiscardAll);
 
          // Clear all old buffers
          for (const auto& buffer : m_dataStream->AnnouncedBuffers())
          {
              m_dataStream->RevokeBuffer(buffer);
          }
 
          int64_t payloadSize = m_nodemapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("PayloadSize")->Value();
 
          // Get number of minimum required buffers
          int numBuffersMinRequired = m_dataStream->NumBuffersAnnouncedMinRequired();
 
          // Alloc buffers
          for (size_t count = 0; count < numBuffersMinRequired; count++)
          {
              auto buffer = m_dataStream->AllocAndAnnounceBuffer(static_cast<size_t>(payloadSize), nullptr);
              m_dataStream->QueueBuffer(buffer);
          }
 
          return true;
      }
  }
  catch (std::exception& e)
  {
        std::cerr << "Error alloc and announce : " << e.what() << std::endl;
  }
 
  return false;
}
 
bool StartAcquisition()
{
  try
  {
      m_dataStream->StartAcquisition(peak::core::AcquisitionStartMode::Default, peak::core::DataStream::INFINITE_NUMBER);
      m_nodemapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("TLParamsLocked")->SetValue(1);
      m_nodemapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStart")->Execute();
      
      std::cout << "Stream open ids camera is running" << std::endl;
 
      return true;
  }
  catch (std::exception& e)
  {
      std::cerr << "Error start acqusition : " << e.what() << std::endl;
  }
 
  return false;
}

void PublishAcquisition()
{
    
    
    //cv image variable
    cv::Mat undistortedImage;
    cv::Mat showImage;
    cv::Mat oldCamMtx = (cv::Mat1d(3,3) << 6.81319947e+03, 0.0, 2.72265296e+03, 0.0, 6.81553939e+03, 1.83752098e+03, 0.0, 0.0, 1.00000000e+00);
    cv::Mat oldDistMtx = (cv::Mat1d(14,1) << -2.13791206e+01,  1.10419012e+02, -4.95266348e-04,  5.61749557e-04, 1.49212748e+02, -2.12123035e+01,  1.06669216e+02,  1.71757700e+02, 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 0.00000000e+00, 0.00000000e+00);
    cv::Mat newCamMtx = (cv::Mat1d(3,3) << 6.59120557e+03, 0.00000000e+00, 2.72294039e+03, 0.00000000e+00, 6.59485693e+03, 1.83543881e+03, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00);
    cv::Size2i sizeImage = cv::Size2i(5536, 3692);
    cv::Mat cvImage(3692,5536, CV_8UC3);
    cv::Mat R = cv::Mat::eye(3,3,  CV_32F);
    cv::Mat mapx;
    cv::Mat mapy;
    
    //ros publisher
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub = it.advertise("ids/rgb", 1);
    ros::Rate loop_rate(30);

    peak::ipl::ImageConverter m_imageConverterIPL;
    m_imageConverterIPL.SetConversionMode(peak::ipl::ConversionMode::Fast);
    cv::initUndistortRectifyMap(oldCamMtx, oldDistMtx, R, newCamMtx, sizeImage, CV_32FC1, mapx, mapy);
    
    while (ros::ok())
    {
        try
        {
            // Get buffer from device's DataStream. Wait 5000 ms. The buffer is automatically locked until it is queued again
            const auto buffer = m_dataStream->WaitForFinishedBuffer(5000);
    
            const auto image = peak::ipl::Image(peak::BufferTo<peak::ipl::Image>(buffer));
            auto imageProcessed = m_imageConverterIPL.Convert(image, peak::ipl::PixelFormatName::BGR8, cvImage.data,  static_cast<size_t>(cvImage.step[0] * cvImage.rows));
    
            cv::Mat img_cv(imageProcessed.Height(), imageProcessed.Width(), CV_8UC3, imageProcessed.Data());
            //cv::rotate(cvImage, cvImage, cv::ROTATE_180);

            //auto start = std::chrono::high_resolution_clock::now();
            cv::remap(img_cv, undistortedImage, mapx, mapy, cv::INTER_LINEAR);
            //std::cout << "remap time: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << std::endl;
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", undistortedImage).toImageMsg();
            pub.publish(msg);
            
            //cv::resize(undistortedImage, showImage, cv::Size(1384, 923), cv::INTER_LINEAR);

            //cv::imshow("Image", showImage);
            //cv::waitKey(1);

            //std::cout << "Pic." << std::endl;

            // Queue buffer again
            m_dataStream->QueueBuffer(buffer);
            ros::spinOnce();
            loop_rate.sleep();
        }   
        catch (const std::exception& e)
        {
            std::cout << "Error processing image " << e.what() << std::endl;
        }
        
    }
}


 
int main(int argc, char **argv)
{
    // initialize ids library
    peak::Library::Initialize();
    // initilaize ros 
    ros::init(argc, argv,"ids_node");

    if (!OpenCamera())
    {
        return -1;
    }

    if (!PrepareAcquisition())
    {
        return -2;
    }
    
    if (!AllocAndAnnounceBuffers())
    {
        return -3;
    }
    
    if (!StartAcquisition())
    {
        return -4;
    }

    PublishAcquisition();

    std::cout << "End." << std::endl;
    
    peak::Library::Close();
    return 0;
}