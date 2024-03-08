//1����ʼ��NUI   
HRESULT hr = NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH);

//2�������¼����   
//������ȡ��һ֡���ź��¼����������KINECT�Ƿ���Կ�ʼ��ȡ��һ֡����  
HANDLE nextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
HANDLE colorStreamHandle = NULL; //�����ɫͼ���������ľ����������ȡ����  

HANDLE nextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
HANDLE depthStreamHandle = NULL;//�������ͼ���������ľ����������ȡ����  

//3����KINECT�豸�Ĳ�ɫͼ��Ϣͨ��������colorStreamHandle��������ľ�����Ա����Ժ��ȡ  
hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480,
	0, 2, nextColorFrameEvent, &colorStreamHandle);
namedWindow("colorImage", CV_WINDOW_AUTOSIZE);

hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, NUI_IMAGE_RESOLUTION_640x480,
	0, 2, nextDepthFrameEvent, &depthStreamHandle);
namedWindow("depthImage", CV_WINDOW_AUTOSIZE);

//4����ʼ��ȡ��ɫͼ����   
while (1)
{
	const NUI_IMAGE_FRAME * pColorImageFrame = NULL;
	const NUI_IMAGE_FRAME * pDepthImageFrame = NULL;

	//4.1�����޵ȴ��µĲ�ɫͼ�����ݣ��ȵ��󷵻�  
	if (WaitForSingleObject(nextColorFrameEvent, INFINITE) == 0)
	{

		//4.2���ӸղŴ���������������еõ���֡���ݣ���ȡ�������ݵ�ַ����pColorImageFrame  
		hr = NuiImageStreamGetNextFrame(colorStreamHandle, 0, &pColorImageFrame);

		INuiFrameTexture * pTexture = pColorImageFrame->pFrameTexture;
		NUI_LOCKED_RECT LockedRect;

		//4.3����ȡ����֡��LockedRect���������������ݶ���pitchÿ���ֽ�����pBits��һ���ֽڵ�ַ  
		//���������ݣ����������Ƕ����ݵ�ʱ��kinect�Ͳ���ȥ�޸���  
		pTexture->LockRect(0, &LockedRect, NULL, 0);
		//4.4��ȷ�ϻ�õ������Ƿ���Ч  
		if (LockedRect.Pitch != 0)
		{
			//4.5��������ת��ΪOpenCV��Mat��ʽ  
			for (int i = 0; i<colorImage.rows; i++)
			{
				uchar *ptr = colorImage.ptr<uchar>(i);  //��i�е�ָ��  

				//ÿ���ֽڴ���һ����ɫ��Ϣ��ֱ��ʹ��uchar  
				uchar *pBuffer = (uchar*)(LockedRect.pBits) + i * LockedRect.Pitch;
				for (int j = 0; j<colorImage.cols; j++)
				{
					ptr[3 * j] = pBuffer[4 * j];  //�ڲ�������4���ֽڣ�0-1-2��BGR����4������δʹ��   
					ptr[3 * j + 1] = pBuffer[4 * j + 1];
					ptr[3 * j + 2] = pBuffer[4 * j + 2];
				}
			}
			imshow("colorImage", colorImage);
		}

		//5����֡�Ѿ��������ˣ����Խ������  
		pTexture->UnlockRect(0);
		//6���ͷű�֡���ݣ�׼��ӭ����һ֡   
		NuiImageStreamReleaseFrame(colorStreamHandle, pColorImageFrame);
	}

	//7.1�����޵ȴ��µ�������ݣ��ȵ��󷵻�  
	if (WaitForSingleObject(nextDepthFrameEvent, INFINITE) == 0)
	{
		//7.2���ӸղŴ���������������еõ���֡���ݣ���ȡ�������ݵ�ַ����pImageFrame  
		hr = NuiImageStreamGetNextFrame(depthStreamHandle, 0, &pDepthImageFrame);

		INuiFrameTexture * pTexture = pDepthImageFrame->pFrameTexture;
		NUI_LOCKED_RECT LockedRect;
		//7.3����ȡ����֡��LockedRect���������������ݶ���pitchÿ���ֽ�����pBits��һ���ֽڵ�ַ  
		//���������ݣ����������Ƕ����ݵ�ʱ��kinect�Ͳ���ȥ�޸���  
		pTexture->LockRect(0, &LockedRect, NULL, 0);
		//7.4��ȷ�ϻ�õ������Ƿ���Ч  
		if (LockedRect.Pitch != 0)
		{

			//7.5��������ת��ΪOpenCV��Mat��ʽ  
			for (int i = 0; i<depthImage.rows; i++)
			{
				uchar *ptr = depthImage.ptr(i);  //��i�е�ָ��  
				//ÿ���ֽڴ���һ����ɫ��Ϣ��ֱ��ʹ��uchar  
				uchar *pBuffer = (uchar*)(LockedRect.pBits) + i * LockedRect.Pitch;
				USHORT *pBufferRun = (USHORT *)pBuffer;//������Ҫת������Ϊÿ�����������2���ֽڣ�Ӧ��BYTEת��USHORT  
				for (int j = 0; j<depthImage.cols; j++)
				{
					ptr[j] = 255 - (BYTE)(256 * pBufferRun[j] / 0x1fff); //�����ݹ�һ������*  
				}
			}
			imshow("depthImage", depthImage); //��ʾͼ��  

		}

		//8����֡�Ѿ��������ˣ����Խ������  
		pTexture->UnlockRect(0);
		//9���ͷű�֡���ݣ�׼��ӭ����һ֡  
		NuiImageStreamReleaseFrame(depthStreamHandle, pDepthImageFrame);
		if (cvWaitKey(20) == 'q')
		{

			imwrite("2.jpg", depthImage);
			imshow("��ȡ�����ͼ", depthImage);
			imwrite("1.jpg", colorImage);
			imshow("��ȡ�Ĳ�ɫͼ", colorImage);

			//Sleep(10000);
			break;
		}
	}
