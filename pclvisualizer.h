#ifndef PCLVISUALIZER_H
#define PCLVISUALIZER_H

#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);


#include <QtWidgets/QMainWindow>
#include <QDialog>  
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "ui_pclvisualizer.h"

class myd : public QDialog
{
	Q_OBJECT
public:
	myd(QWidget* parent = 0);
	~myd();
	bool flag;
private:
	QPushButton *mybtn;
	QGridLayout *myGrid;
	QLabel *lab1;
	QLabel *lab2;
	QLineEdit *text1;
	QLineEdit *text2;
	
	public slots:
	void Sure();
};

class PCLVisualizer : public QMainWindow
{
	Q_OBJECT
public:
	PCLVisualizer(QWidget *parent = 0);
	~PCLVisualizer();

private:
	Ui::PCLVisualizerClass ui;
	myd *mydialog;
	bool reg;
	//点云数据存储
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	void onExportExcel(QString filePath,float data[][13], float data_1[][4], time_t &time_);
	void onShow(float data[][13], float data_1[][4]);
	void onSituation(time_t time_[6]);
	//初始化vtk部件
	void initialVtkWidget();
	
	QString text1 = QString::fromLocal8Bit("请确认当前输入文件为经过处理的点云数据！");
	QString text2 = QString::fromLocal8Bit("Author：许星\nTime:2017-03-09");
	private slots:
	//创建打开槽
	void onOpen();
	void onCaculate();
	void onExit();
	void onDesign();
	void onExplain();
	void onRegister();
	void onHelp();

	
};

#endif // PCLVISUALIZER_H

