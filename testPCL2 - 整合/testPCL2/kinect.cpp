//1、初始化NUI   
HRESULT hr = NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH);

//2、定义事件句柄   
//创建读取下一帧的信号事件句柄，控制KINECT是否可以开始读取下一帧数据  
HANDLE nextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
HANDLE colorStreamHandle = NULL; //保存彩色图像数据流的句柄，用以提取数据  

HANDLE nextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
HANDLE depthStreamHandle = NULL;//保存深度图像数据流的句柄，用以提取数据  

//3、打开KINECT设备的彩色图信息通道，并用colorStreamHandle保存该流的句柄，以便于以后读取  
hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480,
	0, 2, nextColorFrameEvent, &colorStreamHandle);
namedWindow("colorImage", CV_WINDOW_AUTOSIZE);

hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, NUI_IMAGE_RESOLUTION_640x480,
	0, 2, nextDepthFrameEvent, &depthStreamHandle);
namedWindow("depthImage", CV_WINDOW_AUTOSIZE);

//4、开始读取彩色图数据   
while (1)
{
	const NUI_IMAGE_FRAME * pColorImageFrame = NULL;
	const NUI_IMAGE_FRAME * pDepthImageFrame = NULL;

	//4.1、无限等待新的彩色图像数据，等到后返回  
	if (WaitForSingleObject(nextColorFrameEvent, INFINITE) == 0)
	{

		//4.2、从刚才打开数据流的流句柄中得到该帧数据，读取到的数据地址存于pColorImageFrame  
		hr = NuiImageStreamGetNextFrame(colorStreamHandle, 0, &pColorImageFrame);

		INuiFrameTexture * pTexture = pColorImageFrame->pFrameTexture;
		NUI_LOCKED_RECT LockedRect;

		//4.3、提取数据帧到LockedRect，它包括两个数据对象：pitch每行字节数，pBits第一个字节地址  
		//并锁定数据，这样当我们读数据的时候，kinect就不会去修改它  
		pTexture->LockRect(0, &LockedRect, NULL, 0);
		//4.4、确认获得的数据是否有效  
		if (LockedRect.Pitch != 0)
		{
			//4.5、将数据转换为OpenCV的Mat格式  
			for (int i = 0; i<colorImage.rows; i++)
			{
				uchar *ptr = colorImage.ptr<uchar>(i);  //第i行的指针  

				//每个字节代表一个颜色信息，直接使用uchar  
				uchar *pBuffer = (uchar*)(LockedRect.pBits) + i * LockedRect.Pitch;
				for (int j = 0; j<colorImage.cols; j++)
				{
					ptr[3 * j] = pBuffer[4 * j];  //内部数据是4个字节，0-1-2是BGR，第4个现在未使用   
					ptr[3 * j + 1] = pBuffer[4 * j + 1];
					ptr[3 * j + 2] = pBuffer[4 * j + 2];
				}
			}
			imshow("colorImage", colorImage);
		}

		//5、这帧已经处理完了，所以将其解锁  
		pTexture->UnlockRect(0);
		//6、释放本帧数据，准备迎接下一帧   
		NuiImageStreamReleaseFrame(colorStreamHandle, pColorImageFrame);
	}

	//7.1、无限等待新的深度数据，等到后返回  
	if (WaitForSingleObject(nextDepthFrameEvent, INFINITE) == 0)
	{
		//7.2、从刚才打开数据流的流句柄中得到该帧数据，读取到的数据地址存于pImageFrame  
		hr = NuiImageStreamGetNextFrame(depthStreamHandle, 0, &pDepthImageFrame);

		INuiFrameTexture * pTexture = pDepthImageFrame->pFrameTexture;
		NUI_LOCKED_RECT LockedRect;
		//7.3、提取数据帧到LockedRect，它包括两个数据对象：pitch每行字节数，pBits第一个字节地址  
		//并锁定数据，这样当我们读数据的时候，kinect就不会去修改它  
		pTexture->LockRect(0, &LockedRect, NULL, 0);
		//7.4、确认获得的数据是否有效  
		if (LockedRect.Pitch != 0)
		{

			//7.5、将数据转换为OpenCV的Mat格式  
			for (int i = 0; i<depthImage.rows; i++)
			{
				uchar *ptr = depthImage.ptr(i);  //第i行的指针  
				//每个字节代表一个颜色信息，直接使用uchar  
				uchar *pBuffer = (uchar*)(LockedRect.pBits) + i * LockedRect.Pitch;
				USHORT *pBufferRun = (USHORT *)pBuffer;//这里需要转换，因为每个深度数据是2个字节，应将BYTE转成USHORT  
				for (int j = 0; j<depthImage.cols; j++)
				{
					ptr[j] = 255 - (BYTE)(256 * pBufferRun[j] / 0x1fff); //将数据归一化处理*  
				}
			}
			imshow("depthImage", depthImage); //显示图像  

		}

		//8、这帧已经处理完了，所以将其解锁  
		pTexture->UnlockRect(0);
		//9、释放本帧数据，准备迎接下一帧  
		NuiImageStreamReleaseFrame(depthStreamHandle, pDepthImageFrame);
		if (cvWaitKey(20) == 'q')
		{

			imwrite("2.jpg", depthImage);
			imshow("截取的深度图", depthImage);
			imwrite("1.jpg", colorImage);
			imshow("截取的彩色图", colorImage);

			//Sleep(10000);
			break;
		}
	}
