/*
获取kinect原始图片序列并依时间保存，以100张为单位，获取的图片可用于kinect标定
*/

#include "kinect.h"
#include <iostream>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <time.h>
#include <vector>
#include <tchar.h>
using namespace cv;
using namespace std;

// 安全释放指针
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
    if (pInterfaceToRelease != NULL)
    {
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
    }
}

// 保存所需数据的结构体
struct eachFrame
{
    string depth_name;
    string rgb_name;
    cv::Mat tmp_itD1;
    cv::Mat tmp_itRGB1;
};

int main()
{
    // 创建保存目录
    CreateDirectory(".//images", NULL);

    // 获取Kinect设备
    IKinectSensor* m_pKinectSensor;
    ICoordinateMapper*      m_pCoordinateMapper=NULL;
    CameraIntrinsics* m_pCameraIntrinsics = new CameraIntrinsics();
    HRESULT hr;
    hr = GetDefaultKinectSensor(&m_pKinectSensor);

    if (FAILED(hr))
    {
        return hr;
    }

    IMultiSourceFrameReader* m_pMultiFrameReader;
    IBodyFrameSource* m_pBodyFrameSource;
    IBodyFrameReader* m_pBodyFrameReader;
    if (m_pKinectSensor)
    {
        hr = m_pKinectSensor->Open();
        Sleep(1000);
        if (SUCCEEDED(hr))
        {
            m_pKinectSensor->get_BodyFrameSource(&m_pBodyFrameSource);
            // 获取多数据源到读取器  
            hr = m_pKinectSensor->OpenMultiSourceFrameReader(
                FrameSourceTypes::FrameSourceTypes_Color |
                FrameSourceTypes::FrameSourceTypes_Infrared |
                FrameSourceTypes::FrameSourceTypes_Depth,
                &m_pMultiFrameReader);
        }
    }

    if (SUCCEEDED(hr))
    {
        hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        return E_FAIL;
    }

    // 获取深度相机内参并打印
    if (SUCCEEDED(hr))
    {
        hr = m_pCoordinateMapper->GetDepthCameraIntrinsics(m_pCameraIntrinsics);
    }
    if (SUCCEEDED(hr))
    {
        cout << "FocalLengthX : " << m_pCameraIntrinsics->FocalLengthX << endl;
        cout << "FocalLengthY : " << m_pCameraIntrinsics->FocalLengthY << endl;
        cout << "PrincipalPointX : " << m_pCameraIntrinsics->PrincipalPointX << endl;
        cout << "PrincipalPointY : " << m_pCameraIntrinsics->PrincipalPointY << endl;
        cout << "RadialDistortionFourthOrder : " << m_pCameraIntrinsics->RadialDistortionFourthOrder << endl;
        cout << "RadialDistortionSecondOrder : " << m_pCameraIntrinsics->RadialDistortionSecondOrder << endl;
        cout << "RadialDistortionSixthOrder : " << m_pCameraIntrinsics->RadialDistortionSixthOrder << endl;
    }


    // 三个数据帧及引用
    IDepthFrameReference* m_pDepthFrameReference;
    IColorFrameReference* m_pColorFrameReference;
    IInfraredFrameReference* m_pInfraredFrameReference;
    IInfraredFrame* m_pInfraredFrame;
    IDepthFrame* m_pDepthFrame;
    IColorFrame* m_pColorFrame;
    // 四个个图片格式
    Mat i_rgb(1080, 1920, CV_8UC4);      //注意：这里必须为4通道的图，Kinect的数据只能以Bgra格式传出
    Mat i_depth(424, 512, CV_8UC1);
    Mat i_depth_raw(424, 512, CV_16UC1);
    Mat i_ir(424, 512, CV_16UC1);

    UINT16 *depthData = new UINT16[424 * 512];
    UINT16 *irData = new UINT16[424 * 512];
    IMultiSourceFrame* m_pMultiFrame = nullptr;

    DepthSpacePoint*        m_pDepthCoordinates;
    ColorSpacePoint*        m_pColorCoordinates;
    CameraSpacePoint*        m_pCameraCoordinates;
    m_pDepthCoordinates = new DepthSpacePoint[1920 * 1080];
    m_pColorCoordinates = new ColorSpacePoint[512 * 424];
    m_pCameraCoordinates = new CameraSpacePoint[512 * 424];

    clock_t start_time;
    vector<eachFrame> framvec;
    SYSTEMTIME sys;
    size_t framecount = 0;

    while (true)
    {
        if (framecount == 0)
        {
            start_time = clock();
        }

        eachFrame thisframe;
        char depth_name[200] = { '\0' };
        char rgb_name[200] = { '\0' };

        // 获取新的一个多源数据帧
        hr = m_pMultiFrameReader->AcquireLatestFrame(&m_pMultiFrame);

        //// 按照时间来保存
        //GetLocalTime(&sys);
        //sprintf(depth_name, "%s%4d%02d%02d%02d_%02d_%02d_%03d%s", "images//depth_", sys.wYear, sys.wMonth, sys.wDay, sys.wHour, sys.wMinute, sys.wSecond, sys.wMilliseconds, ".png");//保存图片名
        //sprintf(rgb_name, "%s%4d%02d%02d%02d_%02d_%02d_%03d%s", "images//rgb_", sys.wYear, sys.wMonth, sys.wDay, sys.wHour, sys.wMinute, sys.wSecond, sys.wMilliseconds, ".png");//保存图片名

        // 按照序号来保存
        GetLocalTime(&sys);
        sprintf_s(depth_name, "%s%d%s", "images//depth_", framecount, ".tif");//保存图片名
        sprintf_s(rgb_name, "%s%d%s", "images//rgb_", framecount, ".jpg");//保存图片名

        if (FAILED(hr) || !m_pMultiFrame)
        {
            //cout << "!!!" << endl;
            continue;
        }

        // 从多源数据帧中分离出彩色数据，深度数据和红外数据
        if (SUCCEEDED(hr))
            hr = m_pMultiFrame->get_ColorFrameReference(&m_pColorFrameReference);
        if (SUCCEEDED(hr))
            hr = m_pColorFrameReference->AcquireFrame(&m_pColorFrame);
        if (SUCCEEDED(hr))
            hr = m_pMultiFrame->get_DepthFrameReference(&m_pDepthFrameReference);
        if (SUCCEEDED(hr))
            hr = m_pDepthFrameReference->AcquireFrame(&m_pDepthFrame);
        if (SUCCEEDED(hr))
            hr = m_pMultiFrame->get_InfraredFrameReference(&m_pInfraredFrameReference);
        if (SUCCEEDED(hr))
            hr = m_pInfraredFrameReference->AcquireFrame(&m_pInfraredFrame);

        // color拷贝到图片中
        UINT nColorBufferSize = 1920 * 1080 * 4;
        if (SUCCEEDED(hr))
            hr = m_pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, i_rgb.data, ColorImageFormat::ColorImageFormat_Bgra);

        // depth拷贝到图片中
        if (SUCCEEDED(hr))
        {
            hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, depthData);
            hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_depth_raw.data));
            for (int i = 0; i < 512 * 424; i++)
            {
                // 0-255深度图，为了显示明显，只取深度数据的低8位
                BYTE intensity = static_cast<BYTE>(depthData[i] % 256);
                reinterpret_cast<BYTE*>(i_depth.data)[i] = intensity;
            }

            // 实际是16位unsigned int数据
            //hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_depth.data));
        }

        // ir拷贝到图片中
        if (SUCCEEDED(hr))
        {
            hr = m_pInfraredFrame->CopyFrameDataToArray(424 * 512, irData);
            hr = m_pInfraredFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_ir.data));
        }

        // 深度图映射到彩色图上
        if (SUCCEEDED(hr))
        {
            HRESULT hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(512 * 424, depthData, 512 * 424, m_pColorCoordinates);      // 注意这里的彩色点数要写成424*512，至于为什么。。。可能是为了代表下一个参数colorSpacePoints的大小
        }
        Mat i_depthToRgb(424, 512, CV_8UC4);
        if (SUCCEEDED(hr))
        {
            for (int i = 0; i < 424 * 512; i++)
            {
                ColorSpacePoint p = m_pColorCoordinates[i];
                if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
                {
                    int colorX = static_cast<int>(p.X + 0.5f);
                    int colorY = static_cast<int>(p.Y + 0.5f);

                    if ((colorX >= 0 && colorX < 1920) && (colorY >= 0 && colorY < 1080))
                    {
                        i_depthToRgb.data[i * 4] = i_rgb.data[(colorY * 1920 + colorX) * 4];
                        i_depthToRgb.data[i * 4 + 1] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 1];
                        i_depthToRgb.data[i * 4 + 2] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 2];
                        i_depthToRgb.data[i * 4 + 3] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 3];
                    }
                }
            }
        }

        thisframe.depth_name = depth_name;
        thisframe.rgb_name = rgb_name;
        //thisframe.tmp_itD1 = i_depth_raw.clone();
        thisframe.tmp_itD1 = i_ir.clone();
        //thisframe.tmp_itRGB1 = i_depthToRgb.clone();
        thisframe.tmp_itRGB1 = i_rgb.clone();
        framvec.push_back(thisframe);

        // 释放资源
        SafeRelease(m_pColorFrame);
        SafeRelease(m_pDepthFrame);
        SafeRelease(m_pInfraredFrame);
        SafeRelease(m_pColorFrameReference);
        SafeRelease(m_pDepthFrameReference);
        SafeRelease(m_pInfraredFrameReference);
        SafeRelease(m_pMultiFrame);
        framecount++;
        if (100 == framecount)
        {
            clock_t end_time = clock();
            float time = (end_time - start_time) / CLOCKS_PER_SEC;

            //save
            for (int i = 0; i < framvec.size(); i++)
            {
                imwrite(framvec[i].depth_name, framvec[i].tmp_itD1);
                imwrite(framvec[i].rgb_name, framvec[i].tmp_itRGB1);
            }
            framvec.clear();
            std::cout << "fps: " << framecount / time << std::endl;
            framecount = 0;
        }
    }
    // 关闭窗口，设备
    cv::destroyAllWindows();
    SafeRelease(m_pCoordinateMapper);
    m_pKinectSensor->Close();
    std::system("pause");
    return 0;
}