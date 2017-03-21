
#include <afxdb.h>
#include <odbcinst.h>
#include "my_function.h"
#include <iostream>
#include <fstream>
#include <time.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pipeline (new pcl::PointCloud<pcl::PointXYZRGB>);//程序读入的进行计算处理的第一手数据
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr test (new pcl::PointCloud<pcl::PointXYZRGB>);//程序测试数据

double maxZ = -100000,minZ = 100000;
map< int,vector<double> > A,B,C;
Eigen::Matrix3f rotation_matrix;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_extract[P_P][P_IN];
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_left[P_P];
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cut(new pcl::PointCloud<pcl::PointXYZRGB>);

/*
矩阵计算
参数：初始向量，结果向量，生成的旋转矩阵
作用：在拟轴后对点云数据进行调整
*/
void CrossProduct(const Eigen::Vector3f &a, const Eigen::Vector3f &b, Eigen::Vector3f &result)
{
	Eigen::Vector3f c;

	result(0) = a(1) * b(2) - a(2) * b(1);
	result(1) = a(2) * b(0) - a(0) * b(2);
	result(2) = a(0) * b(1) - a(1) * b(0);

}
double DotProduct(const Eigen::Vector3f a, const Eigen::Vector3f b)
{
	double result;
	result = a(0) * b(0) + a(1) * b(1) + a(2) * b(2);

	return result;
}
double Normalize(const Eigen::Vector3f v)
{
	double result;

	result = sqrt(v(0) * v(0) + v(1) * v(1) + v(2) * v(2));

	return result;
}
void RotationMatrix(double angle, Eigen::Vector3f u, Eigen::Matrix3f &rotatinMatrix)
{
	double norm = Normalize(u);

	u(0) = u(0) / norm;
	u(1) = u(1) / norm;
	u(2) = u(2) / norm;

	rotatinMatrix(0, 0) = cos(angle) + u(0) * u(0) * (1 - cos(angle));
	rotatinMatrix(0, 1) = u(0) * u(1) * (1 - cos(angle) - u(2) * sin(angle));
	rotatinMatrix(0, 2) = u(1) * sin(angle) + u(0) * u(2) * (1 - cos(angle));

	rotatinMatrix(1, 0) = u(2) * sin(angle) + u(0) * u(1) * (1 - cos(angle));
	rotatinMatrix(1, 1) = cos(angle) + u(1) * u(1) * (1 - cos(angle));
	rotatinMatrix(1, 2) = -u(0) * sin(angle) + u(1) * u(2) * (1 - cos(angle));

	rotatinMatrix(2, 0) = -u(1) * sin(angle) + u(0) * u(2) * (1 - cos(angle));
	rotatinMatrix(2, 1) = u(0) * sin(angle) + u(1) * u(2) * (1 - cos(angle));
	rotatinMatrix(2, 2) = cos(angle) + u(2) * u(2) * (1 - cos(angle));
}
void CalculationRotationMatrix(const Eigen::Vector3f &vector_before, const Eigen::Vector3f &vector_after, Eigen::Matrix3f &rotation_matrix)
{
	Eigen::Vector3f rotation_axis;
	double rotation_angle;
	CrossProduct(vector_before, vector_after, rotation_axis);
	rotation_angle = acos(DotProduct(vector_before, vector_after) / Normalize(vector_before) / Normalize(vector_after));
	RotationMatrix(rotation_angle, rotation_axis, rotation_matrix);
}

/*
颜色信息添加
参数：对应的RGB值，原始点云，颜色点云
作用：为没有颜色信息的点云数据添加颜色，转化为不同数据类型
*/
float Rgb2Float(uint8_t a ,uint8_t b ,uint8_t c){
	uint8_t r1 = a;
	uint8_t g1 = b;
	uint8_t b1 = c;
	int32_t rgb1 = (r1 << 16) | (g1 << 8) | b1; 
	return *(float *)(&rgb1);
}
int RgbAdd(int a,int b,int c,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_1,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_2){
	if(cloud_2->width < cloud_1->width)
		return 0;
	for(size_t i = 0; i<cloud_1->width; i++){
		cloud_2->points[i].x = cloud_1->points[i].x;
		cloud_2->points[i].y = cloud_1->points[i].y;
		cloud_2->points[i].z = cloud_1->points[i].z;
		cloud_2->points[i].rgb = Rgb2Float(a,b,c);
	}
	return 1;
}

/*
平均数计算
参数：要计算的数据组
作用：计算一组数据的平均值
*/
double Avg(vector<double> a)
{
	vector<double>::iterator it;
	double sum = 0;
	for(it = a.begin();it!=a.end();it++){
		sum = sum + *it;
	}
	int size = a.size();
	return sum/size;
}

/*
快速排序（x,z）
参数：点云数据组，排序的点的首尾标号
作用：将一组数据分别按照x,z的值进行排序
*/
void Sort_x(vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> &s, int l, int r)
{
	if (l < r)
	{
		//Swap(s[l], s[(l + r) / 2]); //将中间的这个数和第一个数交换 参见注1
		int i = l, j = r;
		pcl::PointXYZRGB x(255,255,255);
		x = s[l];
		while (i < j)
		{
			while(i < j && s[j].x >= x.x) // 从右向左找第一个小于x的数
				j--;  
			if(i < j) 
				s[i++] = s[j];

			while(i < j && s[i].x < x.x) // 从左向右找第一个大于等于x的数
				i++;  
			if(i < j) 
				s[j--] = s[i];
		}
		s[i] = x;
		Sort_x(s, l, i - 1); // 递归调用 
		Sort_x(s, i + 1, r);
	}
}
void Sort_z(vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> &s, int l, int r)
{
	if (l < r)
	{
		//Swap(s[l], s[(l + r) / 2]); //将中间的这个数和第一个数交换 参见注1
		int i = l, j = r;
		pcl::PointXYZRGB x(255,255,255);
		x = s[l];
		while (i < j)
		{
			while(i < j && s[j].z >= x.x) // 从右向左找第一个小于x的数
				j--;  
			if(i < j) 
				s[i++] = s[j];

			while(i < j && s[i].z < x.x) // 从左向右找第一个大于等于x的数
				i++;  
			if(i < j) 
				s[j--] = s[i];
		}
		s[i] = x;
		Sort_z(s, l, i - 1); // 递归调用 
		Sort_z(s, i + 1, r);
	}
}

/*
投影
参数：原始点云数据和投影后点云数据
作用：为方便处理，将所有点云投影到xy或yz平面*/
int Projection_XY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_2){
	/*double maxY = -100000, maxX = -100000, minY = 100000, maxZ = -100000, minZ = 100000,minX = 100000;
	for(int i = 0; i < cloud_1->width; i++)
	{
		minX = (minX < cloud_1->points[i].x?minX:cloud_1->points[i].x);
		minY = (minY < cloud_1->points[i].y?minY:cloud_1->points[i].y);
		minZ = (minZ < cloud_1->points[i].z?minZ:cloud_1->points[i].z);
		maxX = (maxX > cloud_1->points[i].x?maxX:cloud_1->points[i].x);
		maxY = (maxY > cloud_1->points[i].y?maxY:cloud_1->points[i].y);
		maxZ = (maxZ > cloud_1->points[i].z?maxZ:cloud_1->points[i].z);
	}

	int w_size = (int)((maxX - minX) / (grid_edge)) + 1;
	int h_size = (int)((maxY - minY) / (grid_edge)) + 1;

	vector<vector<int>> part;
	part.resize(w_size);
	for(int k = 0; k < w_size; k ++)
	{
		part[k].resize(h_size);
	}

	for(int i = 0; i < cloud_1->width; i ++)
	{
		int yy = ((int)((cloud_1->points[i].y - minY) / (grid_edge))) % h_size;
		int xx = ((int)((cloud_1->points[i].x - minX) / (grid_edge))) % w_size;
			part[xx][yy] ++;
	}

	for(int i = 0; i < w_size; i++)
	{
		for(int j = 0; j < h_size; j++){			
			if(num < part[i][j]){
				pcl::PointXYZRGB tmppoint(255,255,255);
				tmppoint.x = minX + i * (grid_edge), tmppoint.y = minY + j * (grid_edge);
				tmppoint.z = minZ;
				cloud_2->push_back(tmppoint);
			}
		}
	}*/
	for (int i = 0; i < cloud_1->width; i++){
		pcl::PointXYZRGB tmppoint(255, 255, 255);
		tmppoint.x = cloud_1->points[i].x, tmppoint.y = cloud_1->points[i].y;
		tmppoint.z = 1;
		cloud_2->push_back(tmppoint);
	}
	return 1;
}
int Projection_YZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_2){
	/*int num = 0;
	double maxY = -100000, maxX = -100000, minY = 100000, maxZ = -100000, minZ = 100000,minX = 100000;
	for(int i = 0; i < cloud_1->width; i++)
	{
	minX = (minX < cloud_1->points[i].x?minX:cloud_1->points[i].x);
	minY = (minY < cloud_1->points[i].y?minY:cloud_1->points[i].y);
	minZ = (minZ < cloud_1->points[i].z?minZ:cloud_1->points[i].z);
	maxX = (maxX > cloud_1->points[i].x?maxX:cloud_1->points[i].x);
	maxY = (maxY > cloud_1->points[i].y?maxY:cloud_1->points[i].y);
	maxZ = (maxZ > cloud_1->points[i].z?maxZ:cloud_1->points[i].z);
	}

	int w_size = (int)((maxZ - minZ) / (grid_edge)) + 1;
	int h_size = (int)((maxY - minY) / (grid_edge)) + 1;

	vector<vector<int>> part;
	part.resize(w_size);
	for(int k = 0; k < w_size; k ++)
	{
	part[k].resize(h_size);
	}

	for(int i = 0; i < cloud_1->width; i ++)
	{
	int yy = ((int)((cloud_1->points[i].y - minY) / (grid_edge))) % h_size;
	int zz = ((int)((cloud_1->points[i].z - minZ) / (grid_edge))) % w_size;
	part[zz][yy] ++;
	}

	for(int i = 0; i < w_size; i++)
	{
	for(int j = 0; j < h_size; j++){
	if(num < part[i][j]){
	pcl::PointXYZRGB tmppoint(255,0,0);
	tmppoint.z = minZ + i * grid_edge, tmppoint.y = minY + j * grid_edge;
	tmppoint.x = minX;
	cloud_2->push_back(tmppoint);
	}
	}
	}*/
	for (int i = 0; i < cloud_1->width; i++){
		pcl::PointXYZRGB tmppoint(255, 255, 255);
		tmppoint.z = cloud_1->points[i].z, tmppoint.y = cloud_1->points[i].y;
		tmppoint.x = 1;
		cloud_2->push_back(tmppoint);
	}
	return 1;
}

/*
稀疏化
参数：原始点云信息和稀疏化后点云信息
作用：将点云数据稀疏化，去除部分杂点
*/
int Thin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_a,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_b){
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloud_a);
	sor.setLeafSize (grid_edge*5, grid_edge*5, grid_edge*5);
	sor.filter (*cloud_b);
	return 1;
}

/*
获取点云信息
参数：投影在YZ平面上的点云数据
作用：获得点云数据Y值最大（小）的点对应的Z值
*/
vector<double> Get_Z_by_maxY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_m){
	double maxy_1 = -100000,miny_1 = 100000;
	vector<double> res(2,0);
	for(int i=0;i<cloud_m->width;i++){
		if(maxy_1 < cloud_m->points[i].y){
			maxy_1 = cloud_m->points[i].y;
			res[0] = cloud_m->points[i].z;
		}
		if(miny_1 > cloud_m->points[i].y){
			miny_1 = cloud_m->points[i].y;
			res[1] = cloud_m->points[i].z;
		}
	}
	return res;
}

/*
求斜率
参数：投影在XY平面上的点云数据
作用：获得呈线装分布的点云数据的斜率值
*/
double GetK(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_1){
	vector<double> tempx(4);
	vector<double> tempy(4);
	vector<double> total(4);

	double xmin = 10000;
	double xmax = -10000;
	double ymin = 10000;
	double ymax = -10000;

	for(int i = 0;i < cloud_1->width;i++){
		if(cloud_1->points[i].x<xmin){
			xmin  = cloud_1->points[i].x;
			tempx[0] = cloud_1->points[i].x;
			tempx[1] = cloud_1->points[i].y;
		}
		if(cloud_1->points[i].x>xmax){
			xmax  = cloud_1->points[i].x;
			tempx[2] = cloud_1->points[i].x;
			tempx[3] = cloud_1->points[i].y;
		}
	}

	double k = (tempx[3]-tempx[1])/(tempx[2]-tempx[0]);
	return k;
}

/*
点云数据旋转校正
参数：旋转方向，原始点云和生成点云
作用：将点云数据沿K方向旋转，即使得点云数据基本与某一坐标轴（此处为X）平行*/
int Rotate(double k,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_1,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_2){
	float theta =  atan(k);
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();				
	transform.rotate (Eigen::AngleAxisf (-theta, Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud (*cloud_1, *cloud_2, transform);
	return 1;
}

/*
环间缝，数据处理
参数：输入的点云环管片段
作用：获得输入的点云数据（截取出来的环间缝片段）中，环缝的中点对应的Z值
*/
vector<double> Detect(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_XY (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_XY_L (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rotate (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_YZ (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_YZ_L (new pcl::PointCloud<pcl::PointXYZRGB>);
	Projection_XY(cloud, cloud_XY);
	Thin(cloud_XY,cloud_XY_L);
	Sort_x(cloud_XY_L->points,0,cloud_XY_L->width-1);
	double k = GetK(cloud_XY_L);
	Rotate(k,cloud,cloud_rotate);
	Projection_YZ(cloud_rotate, cloud_YZ);
	Thin(cloud_YZ,cloud_YZ_L);
	Sort_z(cloud_YZ_L->points,0,cloud_YZ_L->width-1);
	vector<double> res = Get_Z_by_maxY(cloud_YZ_L);

	/*savePCDFile("cloud.pcd", *cloud);
	savePCDFile("cloud_XY.pcd", *cloud_XY);
	savePCDFile("cloud_XY_L.pcd", *cloud_XY_L);
	savePCDFile("cloud_rotate.pcd", *cloud_rotate);
	savePCDFile("cloud_YZ.pcd", *cloud_YZ);
	savePCDFile("cloud_YZ_L.pcd", *cloud_YZ_L);*/

	return res;
}

/*
拟圆
参数：输入的类圆参数点，输出的圆心坐标，以及半径
作用：对类圆数据进行拟圆，并获得圆心坐标以及半径
*/
bool CircleLeastFit(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_1, double &center_x, double &center_y, double &radius)
{
     center_x = 0.0f;
     center_y = 0.0f;
     radius = 0.0f;
	 if (cloud_1->width < 3)
     {
         return false;
     }

     double sum_x = 0.0f, sum_y = 0.0f;
     double sum_x2 = 0.0f, sum_y2 = 0.0f;
     double sum_x3 = 0.0f, sum_y3 = 0.0f;
     double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;

     int T = cloud_1->width;
     for (int i = 0; i < T; i++)
     {
         double x = cloud_1->points[i].x;
         double y = cloud_1->points[i].y;
         double x2 = x * x;
         double y2 = y * y;
         sum_x += x;
         sum_y += y;
         sum_x2 += x2;
         sum_y2 += y2;
         sum_x3 += x2 * x;
         sum_y3 += y2 * y;
         sum_xy += x * y;
         sum_x1y2 += x * y2;
         sum_x2y1 += x2 * y;
     }

     double C, D, E, G, H;
     double a, b, c;

     C = T * sum_x2 - sum_x * sum_x;
     D = T * sum_xy - sum_x * sum_y;
     E = T * sum_x3 + T * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
     G = T * sum_y2 - sum_y * sum_y;
     H = T * sum_x2y1 + T * sum_y3 - (sum_x2 + sum_y2) * sum_y;
     a = (H * D - E * G) / (C * G - D * D);
     b = (H * C - E * D) / (D * D - G * C);
     c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / T;

     center_x = a / (-2);
     center_y = b / (-2);
     radius = sqrt(a * a + b * b - 4 * c) / 2;
     return true;
}

/*
获取圆的椭圆度信息
输入参数：圆的中心坐标，原始点云数据
作用：通过遍历获得圆的长短轴角度和距离，并进行椭圆度和椭圆率的计算
*/
vector<double> GetOvals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, double X, double Y){
	double distance_1 = 0,distance_2 = 0;
	double an_1 = 0,an_2 = 0;
	for(int i = 0;i<cloud->width;i++){
		double angel;
		
		if(cloud->points[i].x == X){
			if(cloud->points[i].y > Y)
				angel = 90;
			else
				angel = 270;
		}
		else if(cloud->points[i].y == Y){
			if(cloud->points[i].x > X)
				angel = 0;
			else
				angel = 180;
		}
		else
		{
			double k = (cloud->points[i].y - Y)/(cloud->points[i].x - X);
			if(cloud->points[i].y > Y && cloud->points[i].x > X)
				angel = (int)(atan(k)/PI*180);
			else if(cloud->points[i].x < X)
				angel = (int)(atan(k)/PI*180) + 180;
			else
				angel = (int)(atan(k)/PI*180) + 360;
		}
		double distance = sqrt(pow(cloud->points[i].y - Y,2) + pow(cloud->points[i].x - X,2));
		C[angel].push_back(distance);
	}

	map< int,double >A1;
	map< int,vector<double> >::iterator it_1;
	for(it_1=C.begin();it_1!=C.end();++it_1)
	{
		A1[it_1->first] = Avg(it_1->second);
	}
	double max = 0;
	for(int i=0;i<360;i++){
		if(A1[i]!=NULL && A1[(i+90)%360]!=NULL){
			double max_1 = 2*abs(A1[i]-A1[(i+90)%360]);
			if(max < max_1){
				distance_1 = 2*A1[i];
				distance_2 = 2*A1[(i+90)%360];
				if(A1[i]>=A1[(i+90)%360]){
					an_1 = i;
					an_2 = (i+90)%360;
				}
				else{
					an_2 = i;
					an_1 = (i+90)%360;
				}
				max = max_1;
			}
		}
	}
	double ovality = ((Max_(distance_1,distance_2) - Min(distance_1,distance_2))/(15.2))*1000;
	vector<double> result;
	result.push_back(Max_(distance_1,distance_2)*1000);
	result.push_back(an_1);
	result.push_back(Min(distance_1,distance_2)*1000);
	result.push_back(an_2);
	result.push_back(ovality);
	return result;
}

/*
原始数据寻找圆心
参数：截取的类圆对应的Z值，即环间缝中心
作用：在原始点云数据上，截取环间缝中心左右一定距离的类圆数据，分别拟圆
对拟出的圆心数据进行平均，作为环间缝所在圆圆心的标准
*/
vector<double> Fit(double a)
{
	int Gap_width = 10;
	int Cut_width = 50;
	double gap_1 = a + Gap_width*grid_edge;
	double gap_2 = a - Gap_width*grid_edge;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_circle_1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_circle_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	for(int i=0;i<cloud_pipeline->width;i++){
		if (cloud_pipeline->points[i].z > gap_1 && cloud_pipeline->points[i].z < gap_1 + Cut_width*grid_edge){
			pcl::PointXYZRGB tmppoint(0,255,0);
			tmppoint.z = gap_1, tmppoint.y = cloud_pipeline->points[i].y;
			tmppoint.x = cloud_pipeline->points[i].x;
			cloud_circle_1->push_back(tmppoint);
		}
		if (cloud_pipeline->points[i].z < gap_2 && cloud_pipeline->points[i].z > gap_2 - Cut_width*grid_edge){
			pcl::PointXYZRGB tmppoint(0,255,0);
			tmppoint.z = gap_2, tmppoint.y = cloud_pipeline->points[i].y;
			tmppoint.x = cloud_pipeline->points[i].x;
			cloud_circle_2->push_back(tmppoint);
		}
	}
	
	double center_x_1 = 0, center_y_1 = 0, radius_1 = 0;
	double center_x_2 = 0, center_y_2 = 0, radius_2 = 0;

	pcl::PointXYZRGB tmppoint(0, 0, 255);
	tmppoint.z = gap_1, tmppoint.y = center_y_1;
	tmppoint.x = center_x_1;
	cloud_circle_1->push_back(tmppoint);

	tmppoint.z = gap_2, tmppoint.y = center_y_2;
	tmppoint.x = center_x_2;
	cloud_circle_2->push_back(tmppoint);

	CircleLeastFit(cloud_circle_1,center_x_1,center_y_1,radius_1);	
	CircleLeastFit(cloud_circle_2,center_x_2,center_y_2,radius_2);
	
	/*savePCDFile("cloud_circle_1.pcd", *cloud_circle_1);
	savePCDFile("cloud_circle_2.pcd", *cloud_circle_2);*/

	vector<double> center(2);
	center[0] = 0.5*(center_x_1 + center_x_2);
	center[1] = 0.5*(center_y_1 + center_y_2);
	
	return center;
}

/*
计算点云到中心距离
参数：环间缝中心对应的Z值，环间缝的圆对应的圆心，环间缝对应的点云片段
作用：计算在中心附近的片段中的点到圆心的距离，作为错台值计算的标准
*/
double Distance(double position, vector<double> center, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
	int Gap_width = 10;
	int Cut_width = 300;
	double begin_1 = position - Gap_width * grid_edge;
	double end_1 = begin_1 - Cut_width * grid_edge;

	double begin_2 = position + Gap_width * grid_edge;
	double end_2 = begin_2 + Cut_width * grid_edge;

	for(int i=0;i<cloud->width;i++)
	{
		if((cloud->points[i].z < begin_1 && cloud->points[i].z > end_1)||(cloud->points[i].z < end_2 && cloud->points[i].z > begin_2)){
			double angel;
			if(cloud->points[i].x == center[0]){
				if(cloud->points[i].y > center[1])
					angel = 90;
				else
					angel = 270;
			}
			else if(cloud->points[i].y == center[1]){
				if(cloud->points[i].x > center[0])
					angel = 0;
				else
					angel = 180;
			}
			else
			{
				double k = (cloud->points[i].y - center[1])/(cloud->points[i].x - center[0]);
				if(cloud->points[i].y > center[1] && cloud->points[i].x > center[0])
					angel = (int)(atan(k)/PI*180);
				else if(cloud->points[i].x < center[0])
					angel = (int)(atan(k)/PI*180) + 180;
				else
					angel = (int)(atan(k)/PI*180) + 360;
			}
			double distance = sqrt(pow(cloud->points[i].y - center[1],2) + pow(cloud->points[i].x - center[0],2));
			if(cloud->points[i].z < begin_1 && cloud->points[i].z > end_1)
				A[angel].push_back(distance);
			else
				B[angel].push_back(distance);

				/*pcl::PointXYZRGB tmppoint(0,255,0);
				tmppoint.x = cloud->points[i].x;
				tmppoint.y = cloud->points[i].y;
				tmppoint.z = cloud->points[i].z;
				test->push_back(tmppoint);
				//测试用例
				*/
		}
	}
	return 0;
}

/*
圆切割
参数：切割出的圆片段
作用：将整个圆环切割成N个片段，特别注意切割的出的环片的顺序，上下呈Z状
*/
int CutCircle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud[N]){
	double maxY = -1000, maxX = -1000, minY = 1000, minX = 1000;
	double maxY_x = 0, maxX_y = 0, minY_x = 0, minX_y = 0;
	for (int i = 0; i < cloud_pipeline->width; i++)
	{
		if (minX > cloud_pipeline->points[i].x){
			minX = cloud_pipeline->points[i].x;
			minX_y = cloud_pipeline->points[i].y;
		}
		if (maxX < cloud_pipeline->points[i].x){
			maxX = cloud_pipeline->points[i].x;
			maxX_y = cloud_pipeline->points[i].y;
		}
		if (minY > cloud_pipeline->points[i].y){
			minY = cloud_pipeline->points[i].y;
			minY_x = cloud_pipeline->points[i].x;
		}
		if (maxY < cloud_pipeline->points[i].y){
			maxY = cloud_pipeline->points[i].y;
			maxY_x = cloud_pipeline->points[i].x;
		}
	}
	double A = maxX - minX;
	double B = maxY - minY;
	if (A > B){
		double dis1 = maxY - (maxX_y + minX_y) / 2;
		double dis2 = (maxX_y + minX_y) / 2 - minY;
		if (dis1 > dis2){
			minY = maxY -  maxX + minX;
		}
		else{
			maxY = minY + maxX - minX;
		}
	}
	else{
		double dis1 = maxX - (maxY_x + minY_x) / 2;
		double dis2 = (maxY_x + minY_x) / 2 - minX;
		if (dis1 > dis2){
			minX = maxX - maxY + minY;
		}
		else{
			maxX = minX + maxY - minY;
		}
	}

	for(int i = 0; i < cloud_pipeline->width; i++)
	{
		int j = (int)((cloud_pipeline->points[i].x - minX)*(N-1)*0.5/A);
		if(cloud_pipeline->points[i].y > (minY+maxY)/2){
			cloud[2*j]->push_back(cloud_pipeline->points[i]);
		}
		else{
			cloud[2*j+1]->push_back(cloud_pipeline->points[i]);
		}
	}
	return 1;
}

/*
分模块
参数：处理之前的点云数据和分割后的点云数据
作用：对于用于计算环内缝的片段，提取两边靠中间的两部分作为参照，进行单独的环缝边缘计算
*/
int Part(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_rgb,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_rgb_1,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_rgb_2){
	for(int i = 0; i < cloud_rgb->width; i ++)
	{
		minZ = (minZ < cloud_rgb->points[i].z?minZ:cloud_rgb->points[i].z);
		maxZ = (maxZ > cloud_rgb->points[i].z?maxZ:cloud_rgb->points[i].z);
	}
	for(int i = 0; i < cloud_rgb->width; i ++)
	{
		if((0.8*minZ + 0.2*maxZ < cloud_rgb->points[i].z)&&(cloud_rgb->points[i].z < 0.6*minZ + 0.4*maxZ)){
			cloud_rgb_1->push_back(cloud_rgb->points[i]);
		}
		else if((0.4*minZ + 0.6*maxZ < cloud_rgb->points[i].z)&&(cloud_rgb->points[i].z< 0.2*minZ + 0.8*maxZ)){
			cloud_rgb_2->push_back(cloud_rgb->points[i]);
		}
	}
	return 1;
}

/*
滤波以及点云排序
参数：原始点云信息和滤波排序后点云信息
作用：将点云数据滤波，去除部分杂点，再按照X值大小排列
*/
int Thin_Sort(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_a,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_b){
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloud_a);
	sor.setLeafSize (grid_edge*4, grid_edge*4, grid_edge*4);
	sor.filter (*cloud_b);
	
	double k = GetK(cloud_b);

	float theta =  atan(k);
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();				
	transform.rotate (Eigen::AngleAxisf (-theta, Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud (*cloud_b, *cloud_b, transform);

	Sort_x(cloud_b->points,0,cloud_b->width-1);

	Eigen::Affine3f transform1 = Eigen::Affine3f::Identity();
	transform1.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));								
	pcl::transformPointCloud (*cloud_b, *cloud_b, transform1);

	return 1;
}

/*
移除离群点
参数：原始点云数据和移除离群点后的点云数据
作用：移除原始点云数据中的离群点
*/
int Remove(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_a, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_b, int num)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>sor;
	sor.setInputCloud(cloud_a);
	sor.setMeanK(num);
	sor.setStddevMulThresh(1);
	sor.filter(*cloud_b);
	return 1;
}

/*
获取弯道处点云信息
参数：原始点云数据和提取的弯道处点云数据
作用：对弯道点云数据进行提取
*/
int Separate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_m,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_n){
	int i =20;
	while(i < cloud_m->width){
		double x1 = cloud_m->points[i-20].x;
		double y1 = cloud_m->points[i-20].y;
		
		double x2 = cloud_m->points[i-10].x;
		double y2 = cloud_m->points[i-10].y;

		double x3 = cloud_m->points[i].x;
		double y3 = cloud_m->points[i].y;

		double vx1 = x1-x2;
		double vy1 = y1-y2;

		double vx2 = x3-x2;
		double vy2 = y3-y2;

		double angle = acos((vx1*vx2+vy1*vy2)/(sqrt(vx1*vx1+vy1*vy1)*sqrt(vx2*vx2+vy2*vy2)));

		if(angle < M_PI*2/3 && angle>0)
		{
			for(int k = i-20;k<i;k++){
				cloud_m->points[k].rgb = Rgb2Float(255,0,0);
				cloud_n->push_back(cloud_m->points[k]);
			}
		}
		i++;
	}
	return 1;
}

/*最小二乘法拟合直线*/
vector<double> LeastSquare(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_m)
{
	double t1 = 0, t2 = 0, t3 = 0, t4 = 0;
	for (int i = 0; i<cloud_m->width; ++i)
	{
		t1 += cloud_m->points[i].x * cloud_m->points[i].x;
		t2 += cloud_m->points[i].x;
		t3 += cloud_m->points[i].x * cloud_m->points[i].y;
		t4 += cloud_m->points[i].y;
	}
	double a = (t3*cloud_m->width - t2*t4) / (t1*cloud_m->width - t2*t2);
	double b = (t1*t4 - t2*t3) / (t1*cloud_m->width - t2*t2);
	vector<double> result(2);
	result[0] = a;
	result[1] = b;
	return result;
}

/*
去除弯道的凸起部分，提取平行线条
参数：斜率，含弯道的点云数据，要输出的平行部分，作为提取的数据
作用：去除弯道的凸起部分，提取平行线条
*/
vector<double> Extract(double k1,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_s,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_e,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_f){
	float theta =  atan(k1);
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();				
	transform.rotate (Eigen::AngleAxisf (-theta, Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud (*cloud_s, *cloud_s, transform);
	pcl::transformPointCloud(*cloud_f, *cloud_f, transform);

	Sort_x(cloud_s->points,0,cloud_s->width-1);
	Sort_x(cloud_f->points, 0, cloud_f->width - 1);


	double x0=0,y0=0,x1=0,y1=0,k;
	if (cloud_s->width > 20){
		for (int i = 0; i < 10; i++){
			x0 = cloud_s->points[i].x + x0;
			y0 = cloud_s->points[i].y + y0;
			x1 = cloud_s->points[cloud_s->width - 1 - i].x + x1;
			y1 = cloud_s->points[cloud_s->width - 1 - i].y + y1;
		}
		x0 = x0 / 10;
		y0 = y0 / 10;
		x1 = x1 / 10;
		y1 = y1 / 10;
	}
	else{
		x0 = cloud_s->points[0].x;
		y0 = cloud_s->points[0].y;
		x1 = cloud_s->points[cloud_s->width - 1].x;
		y1 = cloud_s->points[cloud_s->width - 1].y;
	}
	
	k = ( y0-y1 )/ (x0 -x1 );
	double b = y0 - k*x0;
	double tmpx,tmpy,tmpz;
	//vector<double> line = LeastSquare(cloud_f);
	for(int i = 0; i < cloud_f->width; i ++)
	{
		tmpx = cloud_f->points[i].x;
		tmpy = cloud_f->points[i].y;
		tmpz = cloud_f->points[i].z;

		double distance = fabs((k * tmpx - tmpy + b) / sqrt(k * k + 1));
		double X0 = max(x1, x0);
		double X1 = min(x1, x0);
		if (((tmpx<x0 + grid_edge * 10 && tmpx>x1 - grid_edge * 10) || (tmpx<x1 + grid_edge * 10 && tmpx>x0 - grid_edge * 10)) && fabs(distance) <= grid_edge * 5){
		//if(((tmpx<x0+grid_edge*10&&tmpx>x1-grid_edge*10)||(tmpx<x1+grid_edge*10&&tmpx>x0-grid_edge*10))&&abs((tmpx-x0)*k + y0 - tmpy) <grid_edge*10){

			pcl::PointXYZRGB tmppoint(0,255,0);
			tmppoint.x = tmpx, tmppoint.y = tmpy, tmppoint.z = tmpz;
			cloud_e->push_back(tmppoint);		
		}
		/*double distance = fabs((line[0] * cloud_f->points[i].x - cloud_f->points[i].y + line[1]) / sqrt(line[0] * line[0] + 1));
		if (distance <= 2 * grid_edge){
			pcl::PointXYZRGB tmppoint(0, 255, 0);
			tmppoint.x = cloud_f->points[i].x, tmppoint.y = cloud_f->points[i].y, tmppoint.z = cloud_f->points[i].z;
			cloud_e->push_back(tmppoint);
		}*/
	}
	vector<double> res(4);

	
	Eigen::Affine3f transform1 = Eigen::Affine3f::Identity();
	transform1.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud(*cloud_s, *cloud_s, transform1);
	pcl::transformPointCloud(*cloud_f, *cloud_f, transform1);

	Sort_x(cloud_e->points, 0, cloud_e->width - 1);

	pcl::transformPointCloud(*cloud_e, *cloud_e, transform1);
	
	/*double Init = pow(cloud_e->points[1].x - cloud_e->points[0].x, 2) + pow(cloud_e->points[1].y - cloud_e->points[0].y, 2);
	for(int i = 1; i < cloud_e->width; i ++)
	{
		double m = pow(cloud_e->points[i].x-cloud_e->points[i-1].x,2)+pow(cloud_e->points[i].y-cloud_e->points[i-1].y,2);
		if ( m >init/*pow(grid_edge*20,2))
		{	
			res[0] = cloud_e->points[i].x;//(cloud_e->points[i].x + cloud_e->points[i+1].x + cloud_e->points[i+2].x + cloud_e->points[i+3].x + cloud_e->points[i+4].x)/5;
			res[1] = cloud_e->points[i].y;//(cloud_e->points[i].y + cloud_e->points[i + 1].y + cloud_e->points[i + 2].y + cloud_e->points[i + 3].y + cloud_e->points[i + 4].y) / 5;
			res[2] = cloud_e->points[i - 1].x;//(cloud_e->points[i - 5].x + cloud_e->points[i - 1].x + cloud_e->points[i - 2].x + cloud_e->points[i - 3].x + cloud_e->points[i - 4].x) / 5;
			res[3] = cloud_e->points[i - 1].y;//(cloud_e->points[i - 5].y + cloud_e->points[i - 1].y + cloud_e->points[i - 2].y + cloud_e->points[i - 3].y + cloud_e->points[i - 4].y) / 5;
			init = m;
		}
	}*/
	double init = pow(cloud_e->points[1].x - cloud_e->points[0].x, 2) + pow(cloud_e->points[1].y - cloud_e->points[0].y, 2);
	for (int i = 1; i < cloud_e->width; i++)
	{
		double m = pow(cloud_e->points[i].x - cloud_e->points[i - 1].x, 2) + pow(cloud_e->points[i].y - cloud_e->points[i - 1].y, 2);
		if (m >init/*pow(grid_edge*20,2)*/)
		{
			res[0] = cloud_e->points[i].x;//(cloud_e->points[i].x + cloud_e->points[i+1].x + cloud_e->points[i+2].x + cloud_e->points[i+3].x + cloud_e->points[i+4].x)/5;
			res[1] = cloud_e->points[i].y;//(cloud_e->points[i].y + cloud_e->points[i + 1].y + cloud_e->points[i + 2].y + cloud_e->points[i + 3].y + cloud_e->points[i + 4].y) / 5;
			res[2] = cloud_e->points[i - 1].x;//(cloud_e->points[i - 5].x + cloud_e->points[i - 1].x + cloud_e->points[i - 2].x + cloud_e->points[i - 3].x + cloud_e->points[i - 4].x) / 5;
			res[3] = cloud_e->points[i - 1].y;//(cloud_e->points[i - 5].y + cloud_e->points[i - 1].y + cloud_e->points[i - 2].y + cloud_e->points[i - 3].y + cloud_e->points[i - 4].y) / 5;
			init = m;
		}
	}
	return res;
}

/*
获得环内缝错台值
参数：环内缝检测片段，环内缝片段所在圆管的圆心
作用：获取环内缝的两边，计算到中心距离，得出环内缝错台值
*/
vector<double> Pipedetect(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, vector<double> avg_center){
	maxZ = -1000,minZ = 1000;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);//原始点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_A (new pcl::PointCloud<pcl::PointXYZ>);//着色后的点云数据
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);//着色后的点云数据
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb_1 (new pcl::PointCloud<pcl::PointXYZRGB>);//提取出来的点云数据模块
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZRGB>);//基于网格将模块数据压缩在平面
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rem_1(new pcl::PointCloud<pcl::PointXYZRGB>);//移除离群点后点云数据
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rem_2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_thin_1 (new pcl::PointCloud<pcl::PointXYZRGB>);//稀疏化点云数据
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_thin_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sep_1 (new pcl::PointCloud<pcl::PointXYZRGB>);//点云数据弯道信息
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sep_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_extra_1 (new pcl::PointCloud<pcl::PointXYZRGB>);//去除弯道后的点云线条
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_extra_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	/*loadPCDFile(pcdpath, *cloud);//载入原始文件点云数据
	for(int i=0;i<cloud->width;i++){
		pcl::PointXYZ tmppoint;
		tmppoint.x = cloud->points[i].x * rotation_matrix(0,0) + cloud->points[i].y * rotation_matrix(0,1) + cloud->points[i].z * rotation_matrix(0,2);
		tmppoint.y = cloud->points[i].x * rotation_matrix(1,0) + cloud->points[i].y * rotation_matrix(1,1) + cloud->points[i].z * rotation_matrix(1,2);
		tmppoint.z = cloud->points[i].x * rotation_matrix(2,0) + cloud->points[i].y * rotation_matrix(2,1) + cloud->points[i].z * rotation_matrix(2,2);
		cloud_A->push_back(tmppoint);
	}
	cloud_rgb->resize(cloud_A->width);
	RgbAdd(255,255,255,cloud_A,cloud_rgb);*/
	Part(cloud, cloud_rgb_1, cloud_rgb_2);
	Projection_XY(cloud_rgb_1, cloud_1);
	Projection_XY(cloud_rgb_2, cloud_2);
	Remove(cloud_1, cloud_rem_1, 50);
	Remove(cloud_2, cloud_rem_2, 50);
	Thin_Sort(cloud_rem_1, cloud_thin_1);
	Thin_Sort(cloud_rem_2, cloud_thin_2);
	Separate(cloud_thin_1,cloud_sep_1);
	Separate(cloud_thin_2,cloud_sep_2);

	double k1 = GetK(cloud_thin_1);
	double k2 = GetK(cloud_thin_2);
	vector<double> point_1,point_2;
	point_1 = Extract(k1, cloud_sep_1, cloud_extra_1, cloud_thin_1);
	point_2 = Extract(k2, cloud_sep_2, cloud_extra_2, cloud_thin_2);
	/*savePCDFile("o_cloud_rgb.pcd", *cloud_rgb);
	savePCDFile("o_cloud_rgb_1.pcd",*cloud_rgb_1);
	savePCDFile("o_cloud_rgb_2.pcd",*cloud_rgb_2);
	savePCDFile("o_cloud_1.pcd",*cloud_1);
	savePCDFile("o_cloud_2.pcd",*cloud_2);
	savePCDFile("o_cloud_rem_1.pcd", *cloud_rem_1);
	savePCDFile("o_cloud_rem_2.pcd", *cloud_rem_2);
	savePCDFile("o_cloud_thin_1.pcd",*cloud_thin_1);
	savePCDFile("o_cloud_thin_2.pcd",*cloud_thin_2);
	savePCDFile("o_cloud_sep_1.pcd",*cloud_sep_1);
	savePCDFile("o_cloud_sep_2.pcd",*cloud_sep_2);
	savePCDFile("o_cloud_extra_1.pcd",*cloud_extra_1);
	savePCDFile("o_cloud_extra_2.pcd",*cloud_extra_2);
	*/
	double t = (maxZ-minZ)/grid_edge;
	for(double i = 0; i < t; i += 1)
	{
		pcl::PointXYZRGB tmppoint1(255,0,0);
		tmppoint1.x = ((4*point_1[0]-point_2[0])*(t-i)/3 + (4*point_2[0]-point_1[0])*i/3)/t;
		tmppoint1.y = ((4*point_1[1]-point_2[1])*(t-i)/3 + (4*point_2[1]-point_1[1])*i/3)/t;
		tmppoint1.z = i*grid_edge + minZ;

		pcl::PointXYZRGB tmppoint2(255,0,0);
		tmppoint2.x = ((4*point_1[2]-point_2[2])*(t-i)/3 + (4*point_2[2]-point_1[2])*i/3)/t;
		tmppoint2.y = ((4*point_1[3]-point_2[3])*(t-i)/3 + (4*point_2[3]-point_1[3])*i/3)/t;
		tmppoint2.z = i*grid_edge + minZ;

		cloud_rgb->push_back(tmppoint2);
		cloud_rgb->push_back(tmppoint1);
	}
	//savePCDFile("piece.pcd",*cloud_rgb);
	/*double a = (point_1[0]+point_2[0])/2;
	double b = (point_1[1]+point_2[1])/2;
	double c = (point_1[2]+point_2[2])/2;
	double d = (point_1[3]+point_2[3])/2;
	double dis_1 = sqrt(pow(avg_center[0]-a,2)+pow(avg_center[1]-b,2));
	double dis_2 = sqrt(pow(avg_center[0]-c,2)+pow(avg_center[1]-d,2));
	return dis_2-dis_1;*/
	vector<double> dis(2);
	double dis_1 = sqrt(pow(avg_center[0] - point_1[0], 2) + pow(avg_center[1] - point_1[1], 2));
	double dis_2 = sqrt(pow(avg_center[0] - point_1[2], 2) + pow(avg_center[1] - point_1[3], 2));
	double dis_3 = sqrt(pow(avg_center[0] - point_2[0], 2) + pow(avg_center[1] - point_2[1], 2));
	double dis_4 = sqrt(pow(avg_center[0] - point_2[2], 2) + pow(avg_center[1] - point_2[3], 2));
	dis[0] = dis_2 - dis_1;
	dis[1] = dis_4 - dis_3;
	return dis;
}

/*
方差计算
参数：要计算的数据组
作用：对于在同一个角度的点，其错台值是所有点的平均值，引入方差减少误差
*/
double Variance(vector<double> A)
{
	vector<double>::iterator it = A.begin();
	double sum = 0,p = 0,ave = 0,s = 0;
	for(;it!=A.end();it++)
	{
		sum += *it;
	}
	ave=sum/A.size();
	for(it = A.begin();it!=A.end();it++)
	{
		p=p+(*it-ave)*(*it-ave);
	}
	s=sqrt((1.0/A.size())*p);
	return s;
}

/*
计算环间缝两边的椭圆度
参数：环缝对应的Z值
作用：根据环缝的Z值，计算环缝左右两边一定距离内的圆管的椭圆度
*/
vector<double> Ovals(double position){
	double gap_width = 30;
	double cut_width = 20;
	double gap_1 = position + gap_width * grid_edge;
	double gap_2 = position - gap_width * grid_edge;
	//vector<double> center = Fit(position);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_circle_1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_circle_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	for(int i=0;i<cloud_pipeline->width;i++){
		if (cloud_pipeline->points[i].z > gap_1 && cloud_pipeline->points[i].z < gap_1 + cut_width * grid_edge){
			pcl::PointXYZRGB tmppoint;
			tmppoint.z = gap_1, tmppoint.y = cloud_pipeline->points[i].y;
			tmppoint.x = cloud_pipeline->points[i].x;
			cloud_circle_1->push_back(tmppoint);
		}
		if (cloud_pipeline->points[i].z > gap_2 - cut_width * grid_edge && cloud_pipeline->points[i].z < gap_2){
			pcl::PointXYZRGB tmppoint;
			tmppoint.z = gap_2, tmppoint.y = cloud_pipeline->points[i].y;
			tmppoint.x = cloud_pipeline->points[i].x;
			cloud_circle_2->push_back(tmppoint);
		}
	}
	double center_x_1 = 0, center_y_1 = 0, radius_1 = 0;
	double center_x_2 = 0, center_y_2 = 0, radius_2 = 0;

	CircleLeastFit(cloud_circle_1,center_x_1,center_y_1,radius_1);	
	CircleLeastFit(cloud_circle_2,center_x_2,center_y_2,radius_2);
	vector<double> result_right = GetOvals(cloud_circle_1,center_x_1,center_y_1);
	vector<double> result_left = GetOvals(cloud_circle_2,center_x_2,center_y_2);
	for(int i=0;i<result_right.size();i++){
		result_left.push_back(result_right[i]);
	}
	return result_left;
}

/*
初始化
参数：输入的环间缝文件路径
作用：依照标准完成拟轴工作，调整数据坐标
*/
int Init(string pcdpath){
	maxZ = -1000,minZ = 1000;
	A.clear();
	B.clear();
	C.clear();
	//test->clear();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pipeline_old (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_standard(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pipeline_old_change (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pipeline_filter (new pcl::PointCloud<pcl::PointXYZ>);
	string stand_path = "E:\\data2\\all\\standard.pcd";
	loadPCDFile(stand_path, *cloud_standard);
	loadPCDFile(pcdpath, *cloud_pipeline_old);//载入原始文件点云数据

	//int m = cloud_pipeline_old->width/10000 + 1; 
	int m = 1;
	for(int i=0;i<cloud_pipeline_old->width;){
		pcl::PointXYZ tmppoint;
		tmppoint.x = cloud_pipeline_old->points[i].x;
		tmppoint.y = cloud_pipeline_old->points[i].y;
		tmppoint.z = cloud_pipeline_old->points[i].z;
		cloud_pipeline_filter->push_back(tmppoint);
		i = i + m;
	}
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normObj;  //创建法向量计算对象  
   // normObj.setInputCloud (cloud_pipeline_filter);                  //设置输入点云  
	normObj.setInputCloud(cloud_standard);                  //设置输入点云
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());     
    normObj.setSearchMethod(tree); //设置搜索方法  
    normObj.setRadiusSearch (0.2); //设置半径邻域搜索  
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>); //创建法向量点云  
    normObj.compute (*normals); //计算法向量
	double a1 = 0,a2 = 0, a3 = 0,a4 = 0, a5 = 0;
	for(int i=0;i<normals->width;i++){
		if(_isnan(normals->points[i].normal_x)){
			continue;
		}
		a1 = normals->points[i].normal_x * normals->points[i].normal_y + a1;
		a2 = normals->points[i].normal_z * normals->points[i].normal_y + a2;
		a3 = normals->points[i].normal_x * normals->points[i].normal_x + a3;
		a4 = normals->points[i].normal_x * normals->points[i].normal_z + a4;
		a5 = normals->points[i].normal_z * normals->points[i].normal_z + a5;
	}
	double A = 0,B = 0,C = 0;
	A = a5/(a2*a2);
	B = (2*a4-2*A*a1*a2)/a2;
	C = a3 - B*a1 - A*a1*a1;
	double k = (-2*C/B - a1)/a2;
	double x = -(a1+k*a2)/(a3+2*k*a4+a5*k*k);
	double z = k*x;
	Eigen::Vector3f point_before,point_after; 
	double _long = sqrt(x*x + 1 + z*z);
	point_before[0] = x / _long;
	point_before[1] = 1/_long;
	point_before[2] = z/_long;
	point_after[0] = 0;
	point_after[1] = 0;
	point_after[2] = 1;
	CalculationRotationMatrix(point_before,point_after,rotation_matrix);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_change (new pcl::PointCloud<pcl::PointXYZ>);
	for(int i=0;i<cloud_pipeline_old->width;i++){
		pcl::PointXYZ tmppoint;
		tmppoint.x = cloud_pipeline_old->points[i].x * rotation_matrix(0,0) + cloud_pipeline_old->points[i].y * rotation_matrix(0,1) + cloud_pipeline_old->points[i].z * rotation_matrix(0,2);
		tmppoint.y = cloud_pipeline_old->points[i].x * rotation_matrix(1,0) + cloud_pipeline_old->points[i].y * rotation_matrix(1,1) + cloud_pipeline_old->points[i].z * rotation_matrix(1,2);
		tmppoint.z = cloud_pipeline_old->points[i].x * rotation_matrix(2,0) + cloud_pipeline_old->points[i].y * rotation_matrix(2,1) + cloud_pipeline_old->points[i].z * rotation_matrix(2,2);
		cloud_change->push_back(tmppoint);
	}
	//savePCDFile("cloud_change.pcd",*cloud_change);
	for(int i=0;i<cloud_change->width;i++){
		pcl::PointXYZ tmppoint1;
		tmppoint1.x = cloud_change->points[i].x;
		tmppoint1.y = cloud_change->points[i].y;
		tmppoint1.z = cloud_change->points[i].z;
		cloud_pipeline_old_change->push_back(tmppoint1);
	}
	//savePCDFile("cloud_pipeline_old_change.pcd", *cloud_pipeline_old_change);
	cloud_pipeline->resize(cloud_pipeline_old_change->width);
	RgbAdd(255, 255, 255, cloud_pipeline_old_change, cloud_pipeline);
	return 1;
}

/*
环间缝计算
作用：计算环间缝，输出结果
*/
vector<double> Circle_circle(){
	vector<double> result;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud[N];
	for(int i=0;i<N;i++){
		cloud[i] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	}
	CutCircle(cloud);
	vector<double> avg_center_1(2,0);
	vector<double> avg_center_2(2,0);
	for(int i=0;i<N;i++){
		if(cloud[i]->width < 50){
			continue;
		}
		vector<double> res = Detect(cloud[i]);//找边界
		double max_res;
		if(i%2 == 0)
			max_res = res[0];
		else
			max_res = res[1];
		vector<double> center = Fit(max_res);//找圆心
		avg_center_1[0] += center[0];
		avg_center_1[1] += center[1];

		Distance(max_res,center,cloud[i]);
	}
	map< int,double >A1,B1,A2,B2;
	map< int,vector<double> >::iterator it_1,it_2;
	for(it_1=A.begin();it_1!=A.end();++it_1)
	{
		A1[it_1->first] = Avg(it_1->second);
		A2[it_1->first] = Variance(it_1->second);
	}
	for(it_2=B.begin();it_2!=B.end();++it_2)
	{
		B1[it_2->first] = Avg(it_2->second);
		B2[it_2->first] = Variance(it_2->second);
	}
	double max_distance = 0;
	int num;
		
	/*FILE *fp_;
	fp_ = fopen("my.txt","w");
	if(!fp_){
		;
	}*/
	for(int i=0;i<360;i++){
		if (A1[i] != NULL && B1[i] != NULL && A2[i] < 0.015 && B2[i] < 0.015 && fabs(A1[i] - B1[i])<0.02){
			//fprintf(fp_,"i = %d, A = %f, B=%f,a2=%f,b2=%f D = %f\n",i,A1[i],B1[i],A2[i],B2[i],fabs(A1[i]-B1[i]));
			if(fabs(A1[i]-B1[i])>max_distance){
				num = i;
				max_distance = fabs(A1[i]-B1[i]);
			}
		}
	}

	//fclose(fp_);
	result.push_back(max_distance*1000);
	result.push_back(num);
	return result;
}

/*
环内缝计算
作用：计算环内缝，输出结果
*/
vector<double> Circle_in(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud[N];
	for(int i=0;i<N;i++){
		cloud[i] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	}
	CutCircle(cloud);
	vector<double> avg_center(2,0);
	int k = 0;
	for(int i=0;i<N;i++){
		if(cloud[i]->width < 100){
			continue;
		}
		vector<double> res = Detect(cloud[i]);//找边界
		double max_res;
		if(i%2 == 0)
			max_res = res[0];
		else
			max_res = res[1];
		if(max_res != 0.0){
			k++;
		}
		vector<double> center = Fit(max_res);//找圆心
		avg_center[0] += center[0];
		avg_center[1] += center[1];
	}
	avg_center[0] = avg_center[0]/k;
	avg_center[1] = avg_center[1]/k;
	vector<double> dis = Pipedetect(cloud_in, avg_center);
	vector<double> result;
	result.push_back(abs(dis[0]));
	result.push_back(abs(dis[1]));
	return result;
}

/*
椭圆度计算
作用：计算椭圆度和椭圆率，输出结果
*/
vector<double> Get_ovals(){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud[N];
	for(int i=0;i<N;i++){
		cloud[i] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	}
	CutCircle(cloud);
	double avg_res=0,k=0;
	for(int i=0;i<N;i++){
		if(cloud[i]->width < 100){
			continue;
		}
		vector<double> res = Detect(cloud[i]);//找边界
		double max_res;
		if(i%2 == 0)
			max_res = res[0];
		else
			max_res = res[1];
		avg_res += max_res;
		if(max_res!=0.0){
			k++;
		}
	}
	vector<double> result = Ovals(avg_res/k);
	return result;
}

/*
拟轴
参数：输入点云和对准之后的点云数据
作用：计算环内缝，输出结果
*/
void Standard( pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_change)
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normObj;  //创建法向量计算对象  
	normObj.setInputCloud(point_cloud);//设置输入点云  
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	normObj.setSearchMethod(tree); //设置搜索方法  
	normObj.setRadiusSearch(0.15); //设置半径邻域搜索  
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //创建法向量点云  
	normObj.compute(*normals); //计算法向量
	Eigen::Matrix3f result_matrix;
	double a1 = 0, a2 = 0, a3 = 0, a4 = 0, a5 = 0;
	for (int i = 0; i<normals->width; i++){
		if (_isnan(normals->points[i].normal_x)){
			continue;
		}
		a1 = normals->points[i].normal_x * normals->points[i].normal_y + a1;
		a2 = normals->points[i].normal_z * normals->points[i].normal_y + a2;
		a3 = normals->points[i].normal_x * normals->points[i].normal_x + a3;
		a4 = normals->points[i].normal_x * normals->points[i].normal_z + a4;
		a5 = normals->points[i].normal_z * normals->points[i].normal_z + a5;
	}
	double A = 0, B = 0, C = 0;
	A = a5 / (a2*a2);
	B = (2 * a4 - 2 * A*a1*a2) / a2;
	C = a3 - B*a1 - A*a1*a1;
	double k = (-2 * C / B - a1) / a2;
	double x = -(a1 + k*a2) / (a3 + 2 * k*a4 + a5*k*k);
	double z = k*x;
	Eigen::Vector3f point_before, point_after;
	point_before[0] = x;
	point_before[1] = 1;
	point_before[2] = z;
	point_after[0] = 0;
	point_after[1] = 0;
	point_after[2] = 1;
	CalculationRotationMatrix(point_before, point_after, result_matrix);
	for (int i = 0; i<point_cloud->width; i++){
		pcl::PointXYZ tmppoint;
		tmppoint.x = point_cloud->points[i].x * result_matrix(0, 0) + point_cloud->points[i].y * result_matrix(0, 1) + point_cloud->points[i].z * result_matrix(0, 2);
		tmppoint.y = point_cloud->points[i].x * result_matrix(1, 0) + point_cloud->points[i].y * result_matrix(1, 1) + point_cloud->points[i].z * result_matrix(1, 2);
		tmppoint.z = point_cloud->points[i].x * result_matrix(2, 0) + point_cloud->points[i].y * result_matrix(2, 1) + point_cloud->points[i].z * result_matrix(2, 2);
		cloud_change->push_back(tmppoint);
	}
	return;
}

/*
去噪处理
作用：去除原始点云数据中偏差较大的离群点
*/
void Clear(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clear)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0_0(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_4(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_5(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_origin);
	sor.setLeafSize(grid_edge * 5, grid_edge * 5, grid_edge * 5);
	sor.filter(*cloud_0);
	savePCDFile("cloud_0.pcd", *cloud_0);

	/*Eigen::Vector3f point_before, point_after;
	Eigen::Matrix3f result_matrix;
	point_before[0] = 0;
	point_before[1] = 0;
	point_before[2] = 1;
	point_after[0] = 0.5;
	point_after[1] = 0.5;
	point_after[2] = 0.5;
	CalculationRotationMatrix(point_before, point_after, result_matrix);
	for (int i = 0; i<cloud_0->width; i++){
		pcl::PointXYZ tmppoint;
		tmppoint.x = cloud_0->points[i].x * result_matrix(0, 0) + cloud_0->points[i].y * result_matrix(0, 1) + cloud_0->points[i].z * result_matrix(0, 2);
		tmppoint.y = cloud_0->points[i].x * result_matrix(1, 0) + cloud_0->points[i].y * result_matrix(1, 1) + cloud_0->points[i].z * result_matrix(1, 2);
		tmppoint.z = cloud_0->points[i].x * result_matrix(2, 0) + cloud_0->points[i].y * result_matrix(2, 1) + cloud_0->points[i].z * result_matrix(2, 2);
		cloud_0_0->push_back(tmppoint);
	}*/

	Standard(cloud_0, cloud_1);
	savePCDFile("cloud_1.pcd", *cloud_1);
	for (int i = 0; i<cloud_1->width; i++){
		pcl::PointXYZRGB tmppoint(0, 0, 0);
		tmppoint.x = cloud_1->points[i].x;
		tmppoint.y = cloud_1->points[i].y;
		tmppoint.z = 1;
		cloud_2->push_back(tmppoint);
	}
	double center_x = 0, center_y = 0, radius = 0;
	savePCDFile("cloud_2.pcd", *cloud_2);
	CircleLeastFit(cloud_2, center_x, center_y, radius);

	for (int i = 0; i < cloud_1->width; i++){
		double distance = sqrt(pow(cloud_1->points[i].x - center_x, 2) + pow(cloud_1->points[i].y - center_y, 2));
		if (6.9 < distance && distance < 7.0){
			pcl::PointXYZ tmppoint;
			tmppoint.x = cloud_1->points[i].x;
			tmppoint.y = cloud_1->points[i].y;
			tmppoint.z = cloud_1->points[i].z;
			cloud_clear->push_back(tmppoint);
		}
	}
	/*//完成第一轮过滤
	savePCDFile("cloud_3.pcd", *cloud_3);
	Standard(cloud_3, cloud_4);
	savePCDFile("cloud_4.pcd", *cloud_4);
	for (int i = 0; i<cloud_4->width; i++){
		pcl::PointXYZRGB tmppoint(0, 0, 0);
		tmppoint.x = cloud_4->points[i].x;
		tmppoint.y = cloud_4->points[i].y;
		tmppoint.z = 1;
		cloud_5->push_back(tmppoint);
	}
	savePCDFile("cloud_5.pcd", *cloud_5);
	CircleLeastFit(cloud_5, center_x, center_y, radius);
	for (int i = 0; i < cloud_4->width; i++){
		double distance = sqrt(pow(cloud_4->points[i].x - center_x, 2) + pow(cloud_4->points[i].y - center_y, 2));
		if (6.9 < distance && distance < 7.0){
			pcl::PointXYZ tmppoint;
			tmppoint.x = cloud_4->points[i].x;
			tmppoint.y = cloud_4->points[i].y;
			tmppoint.z = cloud_4->points[i].z;
			cloud_clear->push_back(tmppoint);
		}
	}*/
	//savePCDFile("cloud_clear.pcd", *cloud_clear);
}

/*
判断数据是否在容器中
*/
int In(vector<int>a, int b)
{
	for (int i = 0; i < a.size(); i++){
		if (a[i] == b || a[i] == b-1 || a[i] == b+1){
			return i;
		}
	}
	return -1;
}

/*
环间缝提取
作用：提取环间缝
*/
void Extract_1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin)
{
	double minZ = 1000, maxZ = -1000;
	for (int i = 0; i < cloud_origin->width; i++)
	{
		minZ = (minZ < cloud_origin->points[i].z ? minZ : cloud_origin->points[i].z);
		maxZ = (maxZ > cloud_origin->points[i].z ? maxZ : cloud_origin->points[i].z);
	}
	int num = (int)((maxZ - minZ) * 10) + 1;//以0.1为1个单位，分成num个
	vector<double> point_num(num, 0);
	double all_num[20];
	for (int i = 0; i < 20; i++){
		all_num[i] = 0;
	}
	for (int i = 0; i < cloud_origin->width; i++)
	{
		int position = (int)((cloud_origin->points[i].z - minZ) * 10);
		point_num[position]++;
	}
	int init_num = 0;
	int init_position = 0;
	for (int i = 0; i < 20; i++){
		int j = 1;
		while (j<4){
			all_num[i] += point_num[i + j * 20];
			j++;
		}
		if (all_num[i] > init_num){
			init_num = all_num[i];
			init_position = i;
		}
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cut_1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cut_2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cut_3(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cut_4(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cut_5(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pipe_1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pipe_2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pipe_3(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pipe_4(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pipe_5(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pipe_6(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < cloud_origin->width; i++)
	{
		int position = (cloud_origin->points[i].z - minZ) * 10;
		int part_num = position / 20;
		int gap_num = position % 20;
		pcl::PointXYZRGB tmppoint(255, 255, 255);
		tmppoint.x = cloud_origin->points[i].x;
		tmppoint.y = cloud_origin->points[i].y;
		tmppoint.z = cloud_origin->points[i].z;
		if (gap_num <= init_position + 1 && gap_num >= init_position - 1){

			cloud_extract[part_num][0]->push_back(tmppoint);
			if (part_num == 0){
				tmppoint.rgb = Rgb2Float(255, 0, 0);
			}
			if (part_num == 1){
				tmppoint.rgb = Rgb2Float(255, 192, 203);
			}
			cloud_cut->push_back(tmppoint);
			
		}
		else
		{
			if (position < init_position){
				cloud_left[0]->push_back(tmppoint);
			}
			else if ((position < init_position + 20) && (position > init_position)){
				cloud_left[1]->push_back(tmppoint);
			}
			else if ((position < init_position + 40) && (position > init_position + 20)){
				cloud_left[2]->push_back(tmppoint);
			}
			else if ((position < init_position + 60) && (position > init_position + 40)){
				cloud_left[3]->push_back(tmppoint);
			}
			else if ((position < init_position + 80) && (position > init_position + 60)){
				cloud_left[4]->push_back(tmppoint);
			}
			else{
				cloud_left[5]->push_back(tmppoint);
			}
		}
	}
}

/*
环内缝提取
作用：提取环内缝
*/
void Extract_2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1, int f)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_3(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_4(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_5(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_origin(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_shadow(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < cloud_1->width; i++){
		pcl::PointXYZRGB tmppoint(255, 255, 255);
		tmppoint.x = cloud_1->points[i].x;
		tmppoint.y = cloud_1->points[i].y;
		tmppoint.z = 0;
		cloud_shadow->push_back(tmppoint);
	}
	double center_x = 0, center_y = 0, radius = 0;
	CircleLeastFit(cloud_shadow, center_x, center_y, radius);

	for (int i = 0; i < cloud_1->width; i++){
		double dis = sqrt((cloud_1->points[i].y - center_y)*(cloud_1->points[i].y - center_y) + (cloud_1->points[i].x - center_x)*(cloud_1->points[i].x - center_x)) - radius;
		if (abs(dis) < 0.023){
			pcl::PointXYZRGB tmppoint(255, 255, 255);
			tmppoint.x = cloud_1->points[i].x;
			tmppoint.y = cloud_1->points[i].y;
			tmppoint.z = cloud_1->points[i].z;
			cloud_2->push_back(tmppoint);
		}
	}
	//savePCDFile("origin_2.pcd", *cloud_2);

	Remove(cloud_2, cloud_3, 40);

	//savePCDFile("origin_3.pcd", *cloud_3);

	for (int i = 0; i < cloud_3->width; i++){
		pcl::PointXYZRGB tmppoint(255, 255, 255);
		tmppoint.x = cloud_3->points[i].x;
		tmppoint.y = cloud_3->points[i].y;
		tmppoint.z = 0;
		cloud_4->push_back(tmppoint);
	}

	Remove(cloud_4, cloud_5, 40);
	Remove(cloud_5, cloud_origin, 20);

	//savePCDFile("origin_4.pcd", *cloud_4);
	//savePCDFile("origin_5.pcd", *cloud_5);
	//savePCDFile("origin.pcd", *cloud_origin);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_divid[M];
	for (int i = 0; i<M; i++){
		cloud_divid[i] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	}
	map< int, vector<double> > SIM;
	for (int i = 0; i<cloud_origin->width; i++){
		int angel;
		if (cloud_origin->points[i].x == center_x){
			if (cloud_origin->points[i].y > center_y)
				angel = 180;
			else
				angel = 540;
		}
		else if (cloud_origin->points[i].y == center_y){
			if (cloud_origin->points[i].x > center_x)
				angel = 0;
			else
				angel = 360;
		}
		else
		{
			double k = (cloud_origin->points[i].y - center_y) / (cloud_origin->points[i].x - center_x);
			if (cloud_origin->points[i].y > center_y && cloud_origin->points[i].x > center_x)
				angel = (int)(atan(k) / PI * 360 + 0.5);
			else if (cloud_origin->points[i].x < center_x)
				angel = (int)(atan(k) / PI * 360 + 0.5 + 360);
			else
				angel = (int)(atan(k) / PI * 360 + 0.5 + 720);
		}
		pcl::PointXYZRGB tmppoint(0, 255, 0);
		tmppoint.x = cloud_origin->points[i].x;
		tmppoint.y = cloud_origin->points[i].y;
		tmppoint.z = cloud_origin->points[i].z;
		cloud_divid[angel % 720]->push_back(tmppoint);

		double distance = sqrt(pow(cloud_origin->points[i].y - center_y, 2) + pow(cloud_origin->points[i].x - center_x, 2));
		SIM[angel % 720].push_back(distance);
	}

	map< int, double >SIM_1;
	vector<int> flag;
	map< int, vector<double> >::iterator iter;
	map< int, double >::iterator iter_1;
	for (iter = SIM.begin(); iter != SIM.end(); ++iter)
	{
		SIM_1[iter->first] = Variance(iter->second);
	}
	for (iter_1 = SIM_1.begin(); iter_1 != SIM_1.end(); ++iter_1)
	{
		if (SIM_1[iter_1->first] >= 0.0038){
			flag.push_back(iter_1->first);
		}
	}
	vector<int>::iterator it;
	int perior = 0;
	for (it = flag.begin(); it != flag.end();)
	{
		if (*it == perior + 1){
			perior = *it;
			it = flag.erase(it); //删除元素，返回值指向已删除元素的下一个位置
		}
		else{
			perior = *it;
			++it; //指向下一个位置
		}
	}//由于一条环缝很有可能包含两个数值，这个时候要加以剔除
	
	perior = flag[0];
	it = flag.begin();
	it++;
	for (;it != flag.end();)
	{
		if (*it - perior < 76){
			it = flag.erase(it); //删除元素，返回值指向已删除元素的下一个位置
		}
		else{
			perior = *it;
			++it; //指向下一个位置
		}
	}//提出可能出现的杂点生成的缝隙

	int min = 0;
	int key = 0;

	for (int i = 0; i<cloud_3->width; i++){
		int angel;
		if (cloud_3->points[i].x == center_x){
			if (cloud_3->points[i].y > center_y)
				angel = 180;
			else
				angel = 540;
		}
		else if (cloud_3->points[i].y == center_y){
			if (cloud_3->points[i].x > center_x)
				angel = 0;
			else
				angel = 360;
		}
		else
		{
			double k = (cloud_3->points[i].y - center_y) / (cloud_3->points[i].x - center_x);
			if (cloud_3->points[i].y > center_y && cloud_3->points[i].x > center_x)
				angel = (int)(atan(k) / PI * 360 + 0.5);
			else if (cloud_3->points[i].x < center_x)
				angel = (int)(atan(k) / PI * 360 + 0.5 + 360);
			else
				angel = (int)(atan(k) / PI * 360 + 0.5 + 720);
		}
		int result = In(flag, angel);
		if (result != -1){
			pcl::PointXYZRGB tmppoint(255, 255, 255);
			tmppoint.x = cloud_3->points[i].x;
			tmppoint.y = cloud_3->points[i].y;
			tmppoint.z = cloud_3->points[i].z;
			cloud_extract[f][result+1]->push_back(tmppoint);
			//cloud_cut->push_back(tmppoint);
		}
	}
	int e = 0;
	for (int k = 0; k < P_P; k++){
		e = 0;
		for (int h = 1; h < P_IN; h++){
			if (cloud_extract[k][h]->width == 0){
				continue;
			}
			if (!Out_wrong(cloud_extract[k][h])){
				cloud_extract[k][h]->width = 0;
				cloud_extract[k][h]->clear();
				continue;
			}
			e++;
			for (int l = 0; l < cloud_extract[k][h]->width; l++){
				pcl::PointXYZRGB tmppoint(255, 255, 255);
				tmppoint.x = cloud_extract[k][h]->points[l].x;
				tmppoint.y = cloud_extract[k][h]->points[l].y;
				tmppoint.z = cloud_extract[k][h]->points[l].z;
				if (e == 1){
					tmppoint.rgb = Rgb2Float(255, 0, 0);
				}
				if (e == 2){
					tmppoint.rgb = Rgb2Float(255, 192, 203);
				}
				cloud_cut->push_back(tmppoint);
			}
		}
	}
}

void Work_before_caculate(pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud_origin, time_t time_[3])
{
	for (int i = 0; i<P_P; i++){
		for (int j = 0; j < P_IN; j++){
			cloud_extract[i][j] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
		}
	}
	for (int i = 0; i<P_P; i++){
		cloud_left[i] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
	Clear(point_cloud_origin, cloud_1);
	time_[0] = time(NULL);
	Extract_1(cloud_1);
	time_[1] = time(NULL);
	for (int i = 0; i < P_P; i++){
		if (cloud_left[i]->width > 0){
			Extract_2(cloud_left[i], i);
		}
	}
	time_[2] = time(NULL);
	savePCDFile("cut.pcd", *cloud_cut);
}

void Caculate(float data[][13], float data_1[][4], time_t &time_)
{
	FILE *fp;
	fp = fopen("result.txt", "w");
	if (!fp){
		;
	}
	vector<double> result_1;
	vector<double> result_2;
	vector<double> result_3;
	int a1 = 0, b1 = 0;
	int cacu = 0;
	for (int i = 0; i < P_P; i++){
		if (cloud_extract[i][0]->width == 0){
			continue;
		}
		a1++;
		cloud_pipeline->resize(cloud_extract[i][0]->width);
		for (size_t m = 0; m<cloud_extract[i][0]->width; m++){
			cloud_pipeline->points[m].x = cloud_extract[i][0]->points[m].x;
			cloud_pipeline->points[m].y = cloud_extract[i][0]->points[m].y;
			cloud_pipeline->points[m].z = cloud_extract[i][0]->points[m].z;
			cloud_pipeline->points[m].rgb = cloud_extract[i][0]->points[m].rgb;
		}
		result_1 = Circle_circle();
		result_2 = Get_ovals();
		fprintf(fp, "\n环间缝-------------- \n缝隙编号: %d,最大错台值: %0.1lf   角度 : %0.1lf \n--------------\n ", a1,result_1[0]*1000,result_1[1]);
		fprintf(fp, "椭圆度(前):: \n长轴长: %0.1lf  长轴角度: %0.1lf\n 短轴长: %0.1lf  短轴角度: %0.1lf\n 椭圆度:%0.1lf\n",
			result_2[0] * 1000, result_2[1], result_2[2] * 1000, result_2[3], result_2[4]);
		fprintf(fp, "椭圆度(后):: \n长轴长: %0.1lf  长轴角度: %0.1lf\n 短轴长: %0.1lf  短轴角度: %0.1lf\n 椭圆度:%0.1lf\n环内缝\n",
			result_2[5] * 1000, result_2[6], result_2[7] * 1000, result_2[8], result_2[9]);
		data[a1 - 1][0] = i+1;
		data[a1 - 1][1] = result_1[0];
		data[a1 - 1][2] = result_1[1];
		for (int l = 1; l < 11; l++){
			data[a1 - 1][l + 2] = result_2[l-1];
		}
		b1 = 0;
		for (int j = 1; j < P_IN; j++){
			if (cloud_extract[i][j]->width == 0){
				continue;
			}
			b1++;
			result_3 = Circle_in(cloud_extract[i][j]);
			data_1[cacu][0] = i+1;
			data_1[cacu][1] = b1;
			data_1[cacu][2] = result_3[0] * 1000;
			data_1[cacu][3] = result_3[1] * 1000;
			fprintf(fp, "缝隙编号:%d   错台值（前）:%0.1lf   错台值（后）:%0.1lf \n", b1, result_3[0]*1000, result_3[1]*1000);
			cacu++;
		}
	}
	int q = 0;
	for (int j = 1; j < P_IN; j++){
		if (cloud_extract[5][j]->width == 0){
			continue;
		}
		q++;
		result_3 = Circle_in(cloud_extract[5][j]);
		data_1[cacu][0] = P_P;
		data_1[cacu][1] = q;
		data_1[cacu][2] = result_3[0] * 1000;
		data_1[cacu][3] = result_3[1] * 1000;
		fprintf(fp, "缝隙编号:%d   错台值（前）:%0.1lf   错台值（后）:%0.1lf \n", q, result_3[0]*1000, result_3[1]*1000);
		cacu++;
	}
	for (int t = cacu; t < 30; t++){
		data_1[cacu][0] = 0;
		data_1[cacu][1] = 0;
		data_1[cacu][2] = 0;
		data_1[cacu][3] = 0;
	}
	fclose(fp);
	time_ = time(NULL);
}

int Out_wrong(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud)
{

	double max_ = -100, min_ = 100;
	int one = 0, two = 0;
	for (int i = 0; i < point_cloud->width; i++)
	{
		min_ = (min_ < point_cloud->points[i].z ? min_ : point_cloud->points[i].z);
		max_ = (max_ > point_cloud->points[i].z ? max_ : point_cloud->points[i].z);
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2_1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_3_1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_4_1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_5_1(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2_2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_3_2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_4_2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_5_2(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < point_cloud->width; i++)
	{
		if ((0.8*min_ + 0.2*max_ < point_cloud->points[i].z) && (point_cloud->points[i].z < 0.6*min_ + 0.4*max_)){
			cloud_1_1->push_back(point_cloud->points[i]);
		}
		else if ((0.4*min_ + 0.6*max_ < point_cloud->points[i].z) && (point_cloud->points[i].z< 0.2*min_ + 0.8*max_)){
			cloud_1_2->push_back(point_cloud->points[i]);
		}
	}
	if (cloud_1_2->width <= 80 || cloud_1_1->width <= 80){
		return 0;
	}

	Projection_XY(cloud_1_1, cloud_2_1);
	Projection_XY(cloud_1_2, cloud_2_2);
	Remove(cloud_2_1, cloud_3_1, 50);
	Remove(cloud_2_2, cloud_3_2, 50);
	Thin_Sort(cloud_3_1, cloud_4_1);
	Thin_Sort(cloud_3_2, cloud_4_2);
	Separate(cloud_4_1, cloud_5_1);
	Separate(cloud_4_2, cloud_5_2);

	if (cloud_5_1->width < 20 || cloud_5_2->width < 20){
		return 0;
	}
	return 1;
}
