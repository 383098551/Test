#include "dmcoderecognition.h"
#include "defines.h"
#include <QFile.h>
#include <QJsonDocument>
#include <QFileDialog>

DMCodeRecognition::DMCodeRecognition(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
}

DMCodeRecognition::~DMCodeRecognition()
{}

void DMCodeRecognition::action()
{
    //��ȡ�ļ��������е��ļ�
    QStringList imagesList;
    if (!GetImagesList(imagesList))
    {
        return;
    }

    for (int p = 0; p < imagesList.size(); p++)
    {
        QString filePath = ui.lineEdit->text() + "/" + imagesList.at(p);
        HTuple fileName = (HTuple)filePath.toStdString().c_str();

        HObject ho_Image_slanted, ho_SymbolXLDs, ho_Rectangle, ho_ImageSC;
        HTuple  hv_WindowHandle, hv_DataCodeHandle, hv_ResultHandles;
        HTuple  hv_DecodedDataStrings, hv_Row, hv_Col, hv_Length, hv_HandlesUndecoded;
        HTuple  hv_i, hv_row, hv_col, hv_j;
        HTuple  hv_GMin, hv_GMax, hv_GRange, hv_m, hv_muilt, hv_add;
        HTuple hv_Width, hv_Height;


        //��ȡͼ��
        ReadImage(&ho_Image_slanted, fileName);

        //תΪMat
        Mat image;
        HImage2Mat(ho_Image_slanted, image);

        //��ȡͼ��������С�Ҷ�ֵ
        GetImageSize(ho_Image_slanted, &hv_Width, &hv_Height);
        GenRectangle1(&ho_Rectangle, 0, 0, hv_Height, hv_Width);
        MinMaxGray(ho_Rectangle, ho_Image_slanted, 0, &hv_GMin, &hv_GMax, &hv_GRange);

        //��ǿͼ��Աȶ�
        hv_m = hv_GMax - hv_GMin;
        hv_muilt = 255 / hv_m;
        hv_add = (-hv_muilt) * hv_GMin;
        ScaleImage(ho_Image_slanted, &ho_ImageSC, hv_muilt, hv_add);

        //������ά����
        CreateDataCode2dModel("Data Matrix ECC 200", HTuple(), HTuple(), &hv_DataCodeHandle);
        //ʶ��ͼ���еĶ�ά��
        FindDataCode2d(ho_ImageSC, &ho_SymbolXLDs, hv_DataCodeHandle, "stop_after_result_num", 10, &hv_ResultHandles, &hv_DecodedDataStrings);

        //��ȡ����ʶ���������������
        GetDataCode2dResults(hv_DataCodeHandle, "all_results", "handle", &hv_HandlesUndecoded);

        HTuple end_val21 = (hv_HandlesUndecoded.TupleLength()) - 1;
        HTuple step_val21 = 1;

        //����ʶ������Ķ�ά�룬�����ӿ�����
        std::vector<std::vector<double>> rowss;
        std::vector<std::vector<double>> colss;
        HObject ho_SymbolXLD;
        for (hv_i = 0; hv_i.Continue(end_val21, step_val21); hv_i += step_val21)
        {
            std::vector<double> rows;
            std::vector<double> cols;
            GetDataCode2dObjects(&ho_SymbolXLD, hv_DataCodeHandle, HTuple(hv_HandlesUndecoded[hv_i]), "candidate_xld");
            GetContourXld(ho_SymbolXLD, &hv_Row, &hv_Col);
            TupleLength(hv_Row, &hv_Length);
            HTuple end_val16 = hv_Length - 1;
            HTuple step_val16 = 1;
            for (hv_j = 0; hv_j.Continue(end_val16, step_val16); hv_j += step_val16)
            {
                HTuple hv_row, hv_col;
                hv_row = HTuple(hv_Row[hv_j]);
                hv_col = HTuple(hv_Col[hv_j]);
                rows.push_back(hv_row[0].D());
                cols.push_back(hv_col[0].D());
            }
            rowss.push_back(rows);
            colss.push_back(cols);
        }

        //�����껭��ͼ����
        QString s = "";
        //���ɲ�����json�ļ�
        SaveJson(rowss, colss, imagesList.at(p), image.rows, image.cols);
    }
    ui.textBrowser->append(u8"��ע��ɣ���");
}

bool DMCodeRecognition::HImage2Mat(const HalconCpp::HObject& c_Hobj, cv::Mat& Imat)
{
    //�ж�Halcon������Ƿ��ʼ��
    if (true != c_Hobj.IsInitialized())
    {
        return false;
    }
    try
    {
        ////�ж�ͼ�������Ƿ�Ϊ��
        HTuple   htIsEqual = NULL;
        HObject  hRegionEmpt;

        GenEmptyObj(&hRegionEmpt);
        CompareObj(hRegionEmpt, c_Hobj, 0, &htIsEqual);

        if (1 == htIsEqual.I())///�ж�Ϊ��
        {
            return  false;
        }

        //��ȡHObject������
        HTuple htObjType = 0;
        GetObjClass(c_Hobj, &htObjType);

        ///�ж�ͼ�����ݽṹ��������Ƿ���ȷ
        if ("image" != htObjType[0])
        {
            return  false;
        }

        //Halconͼ�������ݽṹ����
        HObject  hoImagetempt;
        GenEmptyObj(&hoImagetempt);

        ///�����������ʼ��
        HTuple   htChannels;
        Hlong    hlwidth;
        Hlong    hlmwidth;
        Hlong    hlheight;

        ////�Ҷ�ͼ��������buffer
        uchar* ptrGaryData = NULL;

        ///��ɫͼ����������Buffer
        void* Rptr = NULL;
        void* Gptr = NULL;
        void* Bptr = NULL;

        CountChannels(c_Hobj, &htChannels);////��ȡͼ���ͨ����

        HString  hsImageType;
        if (htChannels[0].I() == 1)	////��ͨ��ͼ��
        {
            HImage hImg(c_Hobj);
            ///��ȡ��ͨ��ͼ���ͼ�񲿷�ָ��
            ptrGaryData = (uchar*)hImg.GetImagePointer1(&hsImageType, &hlwidth, &hlheight);
            ///��ָ������ж�
            if (NULL == ptrGaryData)
            {
                return  false;
            }
            ///�Գߴ�����ж�
            if (hlwidth < 10 || hlheight < 10)
            {
                return  false;
            }
            ////�жϿ���Ƿ��ܱ�4������Mat�ṹ�Զ�����4�ֽڶ���
            if (0 == hlwidth % 4)
            {
                Imat.create(hlheight, hlwidth, CV_8UC1);
                if (Imat.empty())
                {
                    return  false;
                }
                ////��Ϊ��4�ֽڶ��룬��������buffer��������ֱ��copy
                memcpy(Imat.data, ptrGaryData, hlheight * hlwidth);
            }
            else
            {
                hlmwidth = hlwidth + 4 - hlwidth % 4;
                Imat.create(hlheight, hlmwidth, CV_8UC1);
                if (Imat.empty())
                {
                    return  false;
                }
                for (int i = 0; i < hlheight; i++)
                {	 ////ÿһ�ж��в����ֽ�
                    memcpy(Imat.ptr<uchar>(i), ptrGaryData + i * hlwidth, hlwidth);
                }
            }
        }
        else if (htChannels[0].I() == 3)
        {
            HImage hImg(c_Hobj);
            //��ȡ��ͨ��������buffer��ָ��
            hImg.GetImagePointer3(&Rptr, &Gptr, &Bptr, &hsImageType, &hlwidth, &hlheight);

            if (NULL == Rptr || NULL == Gptr || NULL == Bptr)
            {
                return false;
            }
            /////����ߴ�ͼ��
            if (hlwidth < 10 || hlheight < 10)
            {
                return  false;
            }
            if (0 == hlwidth % 4)
            {
                Imat.create(hlheight, hlwidth, CV_8UC3);
                if (Imat.empty())
                {
                    return false;
                }
                std::vector<cv::Mat> VecM(3);

                VecM[0].create(hlheight, hlwidth, CV_8UC1);
                if (VecM[0].empty())
                {
                    return false;
                }
                VecM[1].create(hlheight, hlwidth, CV_8UC1);
                if (VecM[1].empty())
                {
                    return false;
                }
                VecM[2].create(hlheight, hlwidth, CV_8UC1);
                if (VecM[2].empty())
                {
                    return false;
                }
                /////4�ֽڶ���ֱ��copy
                memcpy(VecM[2].data, (uchar*)Rptr, hlwidth * hlheight);
                memcpy(VecM[1].data, (uchar*)Gptr, hlwidth * hlheight);
                memcpy(VecM[0].data, (uchar*)Bptr, hlwidth * hlheight);

                merge(VecM, Imat);
            }
            else
            {
                hlmwidth = hlwidth + 4 - hlwidth % 4;
                Imat.create(hlheight, hlwidth, CV_8UC3);
                if (Imat.empty())
                {
                    return false;
                }

                std::vector<cv::Mat> VecM(3);

                VecM[0].create(hlheight, hlwidth, CV_8UC1);
                if (VecM[0].empty())
                {
                    return false;
                }
                VecM[1].create(hlheight, hlwidth, CV_8UC1);
                if (VecM[1].empty())
                {
                    return false;
                }
                VecM[2].create(hlheight, hlwidth, CV_8UC1);
                if (VecM[2].empty())
                {
                    return false;
                }
                for (int i = 0; i < hlheight; i++)
                {	/////copy����������
                    memcpy(VecM[0].ptr<uchar>(i), (uchar*)Rptr + i * hlwidth, hlwidth);
                    memcpy(VecM[1].ptr<uchar>(i), (uchar*)Gptr + i * hlwidth, hlwidth);
                    memcpy(VecM[2].ptr<uchar>(i), (uchar*)Bptr + i * hlwidth, hlwidth);

                }
                merge(VecM, Imat);
            }
        }
    }
    catch (HException& HDevExpDefaultException)
    {
        return false;
    }
    catch (cv::Exception OpenCVDefaultException)
    {
        return false;
    }

    return  true;
}

bool DMCodeRecognition::SaveJson(std::vector<std::vector<double>> rows, std::vector<std::vector<double>> cols, QString imageName, int imagdHeight, int imageWIdth)
{
    //��ͼ���ļ�����תΪjson
    QStringList l = imageName.split(".");
    l.removeLast();
    QString jsonName = l.join(".");
    jsonName += ".json";

    QJsonObject obj;
    QJsonArray shapes;

    QJsonObject nULL;

    //���ɻ���Ԫ��
    obj[VERSION] = "4.5.13";
    obj[FLAGS] = nULL;
    obj[IMAGEPATH] = imageName;
    obj[IMAGEDATA] = {};
    obj[IMAGEHEIGHT] = imagdHeight;
    obj[IMAGEWIDTH] = imageWIdth;

    //����ʶ���ά������
    for (int i = 0; i < rows.size(); i++)
    {
        QJsonObject temp;
        temp[LABEL] = "DMCODE";
        temp[GROUPID] = {};
        temp[SHAPETYPE] = "polygon";
        temp[FLAGS] = nULL;
        //����ʶ��������
        QJsonArray points;
        std::vector<double> row = rows.at(i);
        std::vector<double> col = cols.at(i);
        for (int j = 0; j < row.size(); j++)
        {
            QJsonArray t;
            t.insert(0, col.at(j));
            t.insert(1, row.at(j));
            points.append(t);
        }
        temp[POINTS] = points;
        shapes.append(temp);
    }
    obj[SHAPES] = shapes;

    QString savePath = ui.lineEdit->text() + "/" + jsonName;
    QFile file(savePath);
    if (file.open(QIODevice::Truncate | QIODevice::WriteOnly))
    {
        file.write(QJsonDocument(obj).toJson());
        file.close();
    }
    return true;
}

void DMCodeRecognition::on_pushButton_clicked()
{
    QString name = QFileDialog::getExistingDirectory(this, QString::fromLocal8Bit("ѡ���ļ���"), "./");
    if (!name.isEmpty())
    {
        ui.lineEdit->setText(name);
    }
}

int DMCodeRecognition::on_pushButton_2_clicked()
{
    int ret = 0;
    try
    {
        action();
    }
    catch (HException& exception)
    {
        ui.textBrowser->append(u8"���ִ���");
        ret = 1;
    }
    return ret;
}

bool DMCodeRecognition::GetImagesList(QStringList& imagesList)
{
    QString mFolderPath = ui.lineEdit->text();
    if (mFolderPath.isEmpty())
    {
        ui.textBrowser->append(u8"δѡ���ļ���·������");
        return false;
    }

    // ��ȡ�����ļ���
    QDir dir(mFolderPath);
    mFolderPath = dir.fromNativeSeparators(mFolderPath);//"\\"תΪ"/"
    if (!dir.exists())
    {
        ui.textBrowser->append(u8"�ļ���·�������ڣ���");
        return false;
    }
    dir.setFilter(QDir::Files);
    dir.setSorting(QDir::Name);
    dir.setNameFilters(QString("*.jpg;*.jpeg;*.png").split(";"));
    imagesList = dir.entryList();
    if (imagesList.size() == 0)
    {
        ui.textBrowser->append(u8"�ļ�����û��ͼƬ�ļ�����");
        return false;
    }
}