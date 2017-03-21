#include <QFileDialog>
#include <QMessageBox>
#include <QLineEdit>
#include <QDesktopServices>
#include <QUrl>
#include <ActiveQt/QAxObject>
#include <iostream>
#include <vtkRenderWindow.h>
#include "pclvisualizer.h"
#include "my_function.h"
#include "diskid32.h"


PCLVisualizer::PCLVisualizer(QWidget *parent)
: QMainWindow(parent)
{
	ui.setupUi(this);
	//��ʼ��
	initialVtkWidget();
	ui.Out->setEnabled(false);
	ui.actionCACULATE->setEnabled(false);
	ui.actionOPEN->setEnabled(false);
	//�����źźͲ�
	connect(ui.actionOPEN, SIGNAL(triggered()), this, SLOT(onOpen()));
	connect(ui.actionCACULATE, SIGNAL(triggered()), this, SLOT(onCaculate()));
	connect(ui.actionEXIT, SIGNAL(triggered()), this, SLOT(onExit()));
	connect(ui.actionDesign, SIGNAL(triggered()), this, SLOT(onDesign()));
	connect(ui.actionExplain, SIGNAL(triggered()), this, SLOT(onExplain()));
	connect(ui.actionRegister, SIGNAL(triggered()), this, SLOT(onRegister()));
	connect(ui.Out, SIGNAL(clicked()), this, SLOT(onHelp()));
}

PCLVisualizer::~PCLVisualizer()
{

}
void PCLVisualizer::initialVtkWidget()
{
	cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	viewer->addPointCloud(cloud, "cloud");
	reg = false;
	ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());
	ui.qvtkWidget->update();
}

void PCLVisualizer::onExportExcel(QString filePath, float data[][13], float data_1[][4], time_t &time_)
{
	QAxObject *work_book = NULL;
	QAxObject excel("Excel.Application");
	excel.setProperty("Visible", false);
	excel.setProperty("DisplayAlerts", false);//����ʾ�κξ�����Ϣ
	QAxObject *work_books = excel.querySubObject("WorkBooks");
	QFile xlsFile(filePath);
	//if (xlsFile.exists()){
		//work_book = work_books->querySubObject("Open(const QString &)", filePath);
	//}
	//else{
		work_books->dynamicCall("Add");
		work_book = excel.querySubObject("ActiveWorkBook");
	//}
	char* a[13] = { "�������","�����̨/mm","�����̨��Ӧ�Ƕ�/��","���᳤(ǰ)/mm","����Ƕ�(ǰ)/��","���᳤(ǰ)/mm","����Ƕ�(ǰ)/��",
		"����Բ��(ǰ)/ǧ��", "���᳤(��)/mm", "����Ƕ�(��)/��", "���᳤(��)/mm", "����Ƕ�(��)/��","����Բ��(��)/ǧ��" };
	char *b[4] = { "�������","���ڷ���", "���ڴ�̨(ǰ)/mm", "���ڴ�̨(��)/mm" };
	QAxObject *work_sheets = work_book->querySubObject("Sheets");
	QAxObject *first_sheet = work_sheets->querySubObject("Item(int)", 1);

	int cnt = work_sheets->property("Count").toInt();
	QAxObject *pLastWorkSheet = work_sheets->querySubObject("Item(int)", cnt);
	work_sheets->querySubObject("Add(QVariant)", pLastWorkSheet->asVariant());
	QAxObject *pNewSheet = work_sheets->querySubObject("Item(int)", cnt);

	for (int i = 0; i < 13; i++){
		QAxObject *cell = first_sheet->querySubObject("Cells(int,int)", 1, i+1);
		cell->setProperty("ColumnWidth", 15);  //���õ�Ԫ���п�
		cell->setProperty("HorizontalAlignment", -4108);
		cell->setProperty("VerticalAlignment", -4108);//���ж���
		cell->dynamicCall("Value", QString::fromLocal8Bit(a[i]));
		QAxObject *font = cell->querySubObject("Font");  //��ȡ��Ԫ������
		font->setProperty("Bold", true);  //���õ�Ԫ������Ӵ�
	}
	for (int m = 0; m < 5; m++){
		for (int i = 0; i < 13; i++){
			QAxObject *cell = first_sheet->querySubObject("Cells(int,int)", m+2, i + 1);
			char *buffer = new char[10];
			sprintf(buffer, "%0.1lf", data[m][i]);
			if (i == 1 && data[m][i] > 6.0){
				QAxObject *font = cell->querySubObject("Font");  //��ȡ��Ԫ������
				font->setProperty("Color", QColor(255, 0, 0));  //���õ�Ԫ��������ɫ����ɫ��
			}
			if ((i == 7||i==12) && data[m][i] > 5.0){
				QAxObject *font = cell->querySubObject("Font");  //��ȡ��Ԫ������
				font->setProperty("Color", QColor(255, 0, 0));  //���õ�Ԫ��������ɫ����ɫ��
			}
			cell->setProperty("HorizontalAlignment", -4108);
			cell->setProperty("VerticalAlignment", -4108);//���ж���
			cell->dynamicCall("Value", buffer);
		}
	}
	
	for (int i = 0; i < 4; i++){
		QAxObject *cell = pNewSheet->querySubObject("Cells(int,int)", 1, i + 1);
		cell->setProperty("ColumnWidth", 15);  //���õ�Ԫ���п�
		cell->setProperty("HorizontalAlignment", -4108);
		cell->setProperty("VerticalAlignment", -4108);//���ж���
		cell->dynamicCall("Value", QString::fromLocal8Bit(b[i]));
		QAxObject *font = cell->querySubObject("Font");  //��ȡ��Ԫ������
		font->setProperty("Bold", true);  //���õ�Ԫ������Ӵ�
	}
	for (int m = 0; m < 30; m++){
		if (data_1[m][0] > 6.0 || data_1[m][0] < 1.0){
			continue;
		}
		for (int i = 0; i < 4; i++){
			QAxObject *cell = pNewSheet->querySubObject("Cells(int,int)", m + 2, i + 1);
			char*buffer = new char[10];
			sprintf(buffer, "%0.1lf", data_1[m][i]);
			//sscanf(buffer, "%0.1lf", &data_1[m][i]);
			if ((i == 2 || i == 3) && data_1[m][i] > 5.0){
				QAxObject *font = cell->querySubObject("Font");  //��ȡ��Ԫ������
				font->setProperty("Color", QColor(255, 0, 0));  //���õ�Ԫ��������ɫ����ɫ��
			}
			cell->setProperty("HorizontalAlignment", -4108);
			cell->setProperty("VerticalAlignment", -4108);//���ж���
			cell->dynamicCall("Value", buffer);
			//cell->dynamicCall("Value", data_1[m][i]);
		}
	}
	

	//cell->setProperty("Value", "aaa");
	//    cell->setProperty("RowHeight", 50);  //���õ�Ԫ���и�
	//    cell->setProperty("ColumnWidth", 30);  //���õ�Ԫ���п�
	//    cell->setProperty("HorizontalAlignment", -4108); //����루xlLeft����-4131  ���У�xlCenter����-4108  �Ҷ��루xlRight����-4152
	//    cell->setProperty("VerticalAlignment", -4108);  //�϶��루xlTop��-4160 ���У�xlCenter����-4108  �¶��루xlBottom����-4107
	//    cell->setProperty("WrapText", true);  //���ݹ��࣬�Զ�����
	//    //cell->dynamicCall("ClearContents()");  //��յ�Ԫ������
	//    QAxObject* interior = cell->querySubObject("Interior");
	//    interior->setProperty("Color", QColor(0, 255, 0));   //���õ�Ԫ�񱳾�ɫ����ɫ��
	//    QAxObject* border = cell->querySubObject("Borders");
	//    border->setProperty("Color", QColor(0, 0, 255));   //���õ�Ԫ��߿�ɫ����ɫ��
	//    QAxObject *font = cell->querySubObject("Font");  //��ȡ��Ԫ������
	//    font->setProperty("Name", QStringLiteral("���Ĳ���"));  //���õ�Ԫ������
	//    font->setProperty("Bold", true);  //���õ�Ԫ������Ӵ�
	//    font->setProperty("Size", 20);  //���õ�Ԫ�������С
	//    font->setProperty("Italic", true);  //���õ�Ԫ������б��
	//    font->setProperty("Underline", 2);  //���õ�Ԫ���»���
	//    font->setProperty("Color", QColor(255, 0, 0));  //���õ�Ԫ��������ɫ����ɫ��

	work_book->dynamicCall("SaveAs(const QString &)", QDir::toNativeSeparators(filePath)); //ת��·�������٣���������
	work_book->dynamicCall("Close(Boolean)", false);  //�ر��ļ�
	excel.dynamicCall("Quit(void)");  //�˳�
	time_ = time(NULL);
}

//��ȡ�ı��ͺͶ������͵�������
void PCLVisualizer::onOpen()
{
	//ֻ�ܴ�PCD�ļ�
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open PointCloud"), ".",
		tr("Open PCD files(*.pcd *.bin)"));

	if (!fileName.isEmpty())
	{
		std::string file_name = fileName.toStdString();
		//sensor_msgs::PointCloud2 cloud2;
		pcl::PCLPointCloud2 cloud2;
		//pcl::PointCloud<Eigen::MatrixXf> cloud2;
		Eigen::Vector4f origin;
		Eigen::Quaternionf orientation;
		int pcd_version;
		int data_type;
		unsigned int data_idx;
		int offset = 0;
		pcl::PCDReader rd;
		rd.readHeader(file_name, cloud2, origin, orientation, pcd_version, data_type, data_idx);
		if (data_type == 0)
		{
			pcl::io::loadPCDFile(fileName.toStdString(), *cloud);
		}
		else //if (data_type == 2)
		{
			pcl::PCDReader reader;
			reader.read<pcl::PointXYZRGB>(fileName.toStdString(), *cloud);
			for (size_t i = 0; i<cloud->width; i++){
				if (cloud->points[i].rgb == 0)
					cloud->points[i].rgb = Rgb2Float(255, 255, 255);
				else
					break;
			}
		}

		viewer->updatePointCloud(cloud, "cloud");
		viewer->resetCamera();
		ui.qvtkWidget->update();
	}
}
void PCLVisualizer::onCaculate()
{
	float result[5][13];
	float result_1[30][4];
	time_t time_0;
	time_0 = time(NULL);
	time_t time_1[3];
	time_t time_2;
	time_t time_3;
	time_t time_all[6];
	//memset(result, 0, sizeof(result));
	//memset(result_1, 0, sizeof(result_1));
	QMessageBox message(QMessageBox::Warning, "warning", text1, QMessageBox::Yes | QMessageBox::No, NULL);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (message.exec() == QMessageBox::Yes)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pure(new pcl::PointCloud<pcl::PointXYZ>);
		for (int i = 0; i < cloud->width; i++){
			pcl::PointXYZ tmppoint;
			tmppoint.x = cloud->points[i].x;
			tmppoint.y = cloud->points[i].y;
			tmppoint.z = cloud->points[i].z;
			cloud_pure->push_back(tmppoint);
		}
		Work_before_caculate(cloud_pure, time_1);
		Caculate(result, result_1, time_2);
		onShow(result, result_1);
		
		QString path;
		QDir dir;
		path = dir.currentPath();
		QString picPath = path + "/" + "result.xls";
		onExportExcel(picPath, result, result_1, time_3);

		ui.Out->setEnabled(true);
		time_all[0] = time_0;
		time_all[1] = time_1[0];
		time_all[2] = time_1[1];
		time_all[3] = time_1[2];
		time_all[4] = time_2;
		time_all[5] = time_3;
		onSituation(time_all);

		pcl::PCDReader reader;
		QString cutPath = path + "/" + "cut.pcd";
		reader.read<pcl::PointXYZRGB>("cut.pcd", *cloud_1);
		for (size_t i = 0; i<cloud_1->width; i++){
			if (cloud_1->points[i].rgb == 0)
				cloud_1->points[i].rgb = Rgb2Float(255, 255, 255);
			else
				break;
		}
	}
	viewer->updatePointCloud(cloud_1, "cloud");
	viewer->resetCamera();
	ui.qvtkWidget->update();
	QMessageBox::information(this, QString::fromLocal8Bit("��ʾ"), QString::fromLocal8Bit("������ɣ�"));
}
void PCLVisualizer::onExit()
{
	exit(0);
}

void PCLVisualizer::onExplain(){
	QString path;
	QDir dir;
	path = dir.currentPath();
	QString picPath = "file:///" + path + "/" + "read.pdf";
	QDesktopServices::openUrl(QUrl(picPath));
}

void PCLVisualizer::onRegister()
{
	/*if (reg){
		QMessageBox message(QMessageBox::NoIcon, "About", QString::fromLocal8Bit("���Ѿ�ע��ɹ�������Ҫ��ע�ᣡ"));
		message.exec();
		return;
	}
	mydialog = new myd(this);
	mydialog->exec();
	if (mydialog->flag){
		ui.actionCACULATE->setEnabled(true);
		ui.actionOPEN->setEnabled(true);
		mydialog->close();
		reg = true;
	}*/
	ui.actionCACULATE->setEnabled(true);
	ui.actionOPEN->setEnabled(true);
}

void PCLVisualizer::onDesign(){
	QMessageBox message(QMessageBox::NoIcon, "About", text2);
	message.exec();
}

void PCLVisualizer::onShow(float data[][13], float data_1[][4]){
	float sample[6][4];
	for (int i = 0; i < 5; i++){
		sample[i][0] = data[i][0];
		sample[i][1] = data[i][1];
		sample[i][2] = (data[i][7] + data[i][12]) / 2;
	}
	sample[5][0] = 6;
	sample[5][1] = 0;
	sample[5][2] = 0;
	map<int, vector<double>>avg;
	for (int j = 0; j < 30; j++){
		int a = (int)data_1[j][0];
		avg[a].push_back((double)(data_1[j][2] + data_1[j][3]) / 2);
	}
	for (int j = 0; j < 6; j++){
		sample[j][3] = Avg(avg[j + 1]);
	}
	
	QString show = QString::fromLocal8Bit("����� �����̨ ����Բ�� ���ڴ�̨\n");
	for (int i = 0; i < 6; i++){
		for (int j = 0; j < 4; j++){
			//if (sample[i][j] == NULL){
			//	continue;
			//}
			QString f = QString::number(sample[i][j], 'f', 1);
			if (j == 0){
				f = QString::number(sample[i][j], 'f', 0);
			}
			show += f + "     ";
		}
		show += "\n";
	}
	ui.textEdit_2->setText(show);
}

void PCLVisualizer::onSituation(time_t time_[6]){
	QString sit = QString::fromLocal8Bit("����ʱ����ʾ���£�\n");
	char * type[5] = {
		"���ἰȥ����ɣ���ʱ�� ",
		"�������ȡ��ɣ���ʱ�� ",
		"���ڷ���ȡ��ɣ���ʱ�� ",
		"���ݼ�����ɣ���ʱ�� ",
		"���ݵ�����ɣ���ʱ�� "
	};
	for (int i = 0; i < 5; i++){
		QString name = QString::fromLocal8Bit(type[i]);
		QString time_cost = QString::number(time_[i+1] - time_[i], 10);
		QString unit = QString::fromLocal8Bit(" s\n");
		sit += name;
		sit += time_cost;
		sit += unit;
	}
	sit += QString::fromLocal8Bit("����ʱ��");
	sit += QString::number(time_[5] - time_[0], 10);
	sit += QString::fromLocal8Bit(" s");
	ui.textEdit_1->setText(sit);
}

void PCLVisualizer::onHelp(){
	QString path;
	QDir dir;
	path = dir.currentPath();
	QString picPath = "file:///" + path + "/" + "result.xls";
	QDesktopServices::openUrl(QUrl(picPath));
}

myd::myd(QWidget* parent) : QDialog(parent)
{
	mybtn = new QPushButton(QString::fromLocal8Bit("ȷ��"));
	myGrid = new QGridLayout(this);
	text1 = new QLineEdit();
	text2 = new QLineEdit(QString::fromLocal8Bit("ע��������"));
	lab1 = new QLabel();
	lab2 = new QLabel();
	lab1->setText(QString::fromLocal8Bit("��ĵ��Ի�����Ϊ��"));
	lab2->setText(QString::fromLocal8Bit("������ע���룺"));
	myGrid->addWidget(lab1);
	myGrid->addWidget(text1);
	myGrid->addWidget(lab2);
	myGrid->addWidget(text2);
	myGrid->addWidget(mybtn);
	getHardDriveComputerID();
	QString hdID = QString(HardDriveSerialNumber).simplified();
	text1->setText(hdID);
	text1->setReadOnly(true);
	flag = false;
	connect(mybtn, SIGNAL(clicked(bool)), this, SLOT(Sure()));

}
void myd::Sure()
{
	//char* lp= (char*)malloc(128 * sizeof(char));
	//GetHDSerial(lp);
	QString hdID = QString(HardDriveSerialNumber).simplified();
	bool sim = true;
	QString str = text2->text();
	int len = str.length();
	if (len != 5){
		QMessageBox message(QMessageBox::NoIcon, "About", QString::fromLocal8Bit("ע�������"));
		message.exec();
		text2->setText("");
		sim = false;
	}
	else{
		for (int i = 1; i < 11; )
		{
			int j = (i - 1) / 2;
			QChar t = str.at(j);
			if (hdID[i] != t.toLatin1()){
				QMessageBox message(QMessageBox::NoIcon, "About", QString::fromLocal8Bit("ע�������"));
				message.exec();
				text2->setText("");
				sim = false;
				break;
			}
			i = i + 2;
		}
	}
	
	if (sim){
		QMessageBox message(QMessageBox::NoIcon, "About", QString::fromLocal8Bit("ע��ɹ�����ر�ע�ᴰ�ڣ�"));
		message.exec();
		flag = true;
	}
	
}
myd::~myd()
{

}