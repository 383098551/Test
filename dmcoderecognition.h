#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_dmcoderecognition.h"
#include "HalconCpp.h"
#include "HDevThread.h"
#include <opencv2/opencv.hpp>
#include <QJsonArray>
#include <QJsonObject>

using namespace HalconCpp;
using namespace cv;

class DMCodeRecognition : public QMainWindow
{
    Q_OBJECT

public:
    DMCodeRecognition(QWidget *parent = nullptr);
    ~DMCodeRecognition();

public:
    void action();

private slots:
    void on_pushButton_clicked();

    int on_pushButton_2_clicked();


private:

    bool HImage2Mat(const HalconCpp::HObject& c_Hobj, cv::Mat& Imat);

    bool SaveJson(std::vector<std::vector<double>> rows, std::vector<std::vector<double>> cols, QString imageName, int imagdHeight, int imageWIdth);

    bool GetImagesList(QStringList& imagesList);

private:
    Ui::DMCodeRecognitionClass ui;
};
