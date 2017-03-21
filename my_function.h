#ifndef MY_H_FILE      //如果没有定义这个宏
#define MY_H_FILE      //定义这个宏 
#define PCL_NO_PRECOMPILE

#include <iostream>
#include <string>
#include <math.h>
#include <time.h>
#include <vector>

#undef max
#undef min
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/search.h>
#include <pcl/features/boundary.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>




using pcl::io::loadPCDFile;
using pcl::io::savePCDFile;
using namespace std;
using std::vector;
using std::string;

#define Min(a,b) (a<b?a:b)
#define Max_(a,b) (a>b?a:b)
#define PI 3.14159265
#define G2 1.414213
#define grid_edge 0.001 
#define N 100
#define M 720
#define P_P 6
#define P_IN 10

float Rgb2Float(uint8_t a ,uint8_t b ,uint8_t c);
double Avg(vector<double> a);
//快速排序
void Sort_x(vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> &s, int l, int r);
void Sort_z(vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> &s, int l, int r);

int RgbAdd(int a,int b,int c,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_1,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_2);
int Projection_XY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_1,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_2);
/*将复杂的点云数据稀疏化*/
int Thin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_a,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_b);
/*去除接口，获取点云信息*/
vector<double> Get_Z_by_maxY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_m);
/*求斜率*/
double GetK(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_1);
/*将点云数据沿K方向旋转*/
int Rotate(double k,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_1,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_2);
/*向YZ平面投影*/
int Projection_YZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_2);
/*文件处理*/
vector<double> Detect(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

/*拟合圆*/
bool CircleLeastFit(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_1, double &center_x, double &center_y, double &radius);
vector<double> GetOvals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, double X, double Y);
vector<double> Fit(double a);
double Distance(double position,vector<double> center,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
int CutCircle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud[8]);
/*分模块进行计算*/
int Part(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_rgb,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_rgb_1,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_rgb_2);

int Thin_Sort(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_a,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_b);
/*获取弯道处点云信息*/
int Separate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_m,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_n);
/*去除弯道的凸起部分，提取平行线条*/
vector<double> Extract(double k1,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_s,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_e,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_f);
vector<double> Pipedetect(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<double> avg_center);
double Variance(vector<double> A);
vector<double> Ovals(double position);
int Init(string pcdpath);
vector<double> Circle_circle();
vector<double> Circle_in(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
vector<double> Get_ovals();
void CrossProduct(const Eigen::Vector3f &a, const Eigen::Vector3f &b, Eigen::Vector3f &result);
double DotProduct(const Eigen::Vector3f a, const Eigen::Vector3f b);
double Normalize(const Eigen::Vector3f v);
void RotationMatrix(double angle, Eigen::Vector3f u, Eigen::Matrix3f &rotatinMatrix);
void CalculationRotationMatrix(const Eigen::Vector3f &vector_before, const Eigen::Vector3f &vector_after, Eigen::Matrix3f &rotation_matrix);
void Clear(string pcdpath);
void Standard(pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_change);
vector<double> LeastSquare(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_m);
int Remove(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_a, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_b, int num);
int In(vector<int>a, int b);
void Extract_1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin);
void Extract_2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1, int f);
void Work_before_caculate(pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud_origin, time_t time_[3]);
void Caculate(float data[][13], float data_1[][4], time_t &time_);
int Out_wrong(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud);

#endif 