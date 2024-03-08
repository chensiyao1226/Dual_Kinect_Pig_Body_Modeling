/*
��ȡkinectԭʼͼƬ���в���ʱ�䱣�棬��100��Ϊ��λ����ȡ��ͼƬ������kinect�궨
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

// ��ȫ�ͷ�ָ��
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
    if (pInterfaceToRelease != NULL)
    {
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
    }
}

// �����������ݵĽṹ��
struct eachFrame
{
    string depth_name;
    string rgb_name;
    cv::Mat tmp_itD1;
    cv::Mat tmp_itRGB1;
};

int main()
{
    // ��������Ŀ¼
    CreateDirectory(".//images", NULL);

    // ��ȡKinect�豸
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
            // ��ȡ������Դ����ȡ��  
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

    // ��ȡ�������ڲβ���ӡ
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


    // ��������֡������
    IDepthFrameReference* m_pDepthFrameReference;
    IColorFrameReference* m_pColorFrameReference;
    IInfraredFrameReference* m_pInfraredFrameReference;
    IInfraredFrame* m_pInfraredFrame;
    IDepthFrame* m_pDepthFrame;
    IColorFrame* m_pColorFrame;
    // �ĸ���ͼƬ��ʽ
    Mat i_rgb(1080, 1920, CV_8UC4);      //ע�⣺�������Ϊ4ͨ����ͼ��Kinect������ֻ����Bgra��ʽ����
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

        // ��ȡ�µ�һ����Դ����֡
        hr = m_pMultiFrameReader->AcquireLatestFrame(&m_pMultiFrame);

        //// ����ʱ��������
        //GetLocalTime(&sys);
        //sprintf(depth_name, "%s%4d%02d%02d%02d_%02d_%02d_%03d%s", "images//depth_", sys.wYear, sys.wMonth, sys.wDay, sys.wHour, sys.wMinute, sys.wSecond, sys.wMilliseconds, ".png");//����ͼƬ��
        //sprintf(rgb_name, "%s%4d%02d%02d%02d_%02d_%02d_%03d%s", "images//rgb_", sys.wYear, sys.wMonth, sys.wDay, sys.wHour, sys.wMinute, sys.wSecond, sys.wMilliseconds, ".png");//����ͼƬ��

        // �������������
        GetLocalTime(&sys);
        sprintf_s(depth_name, "%s%d%s", "images//depth_", framecount, ".tif");//����ͼƬ��
        sprintf_s(rgb_name, "%s%d%s", "images//rgb_", framecount, ".jpg");//����ͼƬ��

        if (FAILED(hr) || !m_pMultiFrame)
        {
            //cout << "!!!" << endl;
            continue;
        }

        // �Ӷ�Դ����֡�з������ɫ���ݣ�������ݺͺ�������
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

        // color������ͼƬ��
        UINT nColorBufferSize = 1920 * 1080 * 4;
        if (SUCCEEDED(hr))
            hr = m_pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, i_rgb.data, ColorImageFormat::ColorImageFormat_Bgra);

        // depth������ͼƬ��
        if (SUCCEEDED(hr))
        {
            hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, depthData);
            hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_depth_raw.data));
            for (int i = 0; i < 512 * 424; i++)
            {
                // 0-255���ͼ��Ϊ����ʾ���ԣ�ֻȡ������ݵĵ�8λ
                BYTE intensity = static_cast<BYTE>(depthData[i] % 256);
                reinterpret_cast<BYTE*>(i_depth.data)[i] = intensity;
            }

            // ʵ����16λunsigned int����
            //hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_depth.data));
        }

        // ir������ͼƬ��
        if (SUCCEEDED(hr))
        {
            hr = m_pInfraredFrame->CopyFrameDataToArray(424 * 512, irData);
            hr = m_pInfraredFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_ir.data));
        }

        // ���ͼӳ�䵽��ɫͼ��
        if (SUCCEEDED(hr))
        {
            HRESULT hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(512 * 424, depthData, 512 * 424, m_pColorCoordinates);      // ע������Ĳ�ɫ����Ҫд��424*512������Ϊʲô������������Ϊ�˴�����һ������colorSpacePoints�Ĵ�С
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

        // �ͷ���Դ
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
    // �رմ��ڣ��豸
    cv::destroyAllWindows();
    SafeRelease(m_pCoordinateMapper);
    m_pKinectSensor->Close();
    std::system("pause");
    return 0;
}