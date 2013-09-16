#include <ros/ros.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <math.h>
#include <geometry_msgs/Point.h>

#define PI 3.14159265

using namespace std;

ros::Publisher pub_Plano_suelo;
ros::Publisher pub_cloud_sobrante;
ros::Publisher pub_Objeto_cercano;
ros::Publisher pub_Objetos_detectados;
ros::Publisher pub_Objet;

geometry_msgs::Point Centroide;
bool error=true;
sensor_msgs::PointCloud2 cloud_voxel;

void Fitrar_Voxel(sensor_msgs::PointCloud2ConstPtr input, float size_grid){ 
  // reduce el numero de punto con el voxel grid
  pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
  sor.setInputCloud (input);
  sor.setLeafSize (size_grid, size_grid, size_grid);
  sor.filter (cloud_voxel);
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){
//cloud definition  
   pcl::PointCloud<pcl::PointXYZ> cloud;
   pcl::PointCloud<pcl::PointXYZ> cloud2;
   pcl::PointCloud<pcl::PointXYZ> cloud3;
   pcl::ExtractIndices<pcl::PointXYZ> extract;
   pcl::PointCloud<pcl::PointXYZ> obj_cloud1;
   pcl::PointCloud<pcl::PointXYZ> cloud_objetos;

//sensor definition 
   sensor_msgs::PointCloud2 Plano_suelo;
   sensor_msgs::PointCloud2 cloud_sobrante; 
   sensor_msgs::PointCloud2 Objeto_cercano;
   sensor_msgs::PointCloud2 Objetos_detectados;

  Fitrar_Voxel(input, 0.008);

 if (cloud_voxel.height*cloud_voxel.width>9000){ 
    if (error==true){   
	  std::cerr << "DETECTANDO PUNTOS " << cloud_voxel.height*cloud_voxel.width<< std::endl;
	  error=false;
    }
  
  pcl::fromROSMsg (cloud_voxel, cloud);
 
  // segmentacion plana
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers; 
  float anguloK_S;

 while(true){ 
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true); 
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  seg.setMaxIterations (200); 
  seg.setInputCloud (cloud.makeShared ());
  seg.segment (inliers, coefficients); 
 
  //calculo del angulo de la kinect (angulo formado por la kinect y el suelo)
  anguloK_S=acos(-coefficients.values[1]/sqrt(pow(coefficients.values[2],2)+pow(-coefficients.values[0],2)+pow(-coefficients.values[1],2)))-PI; 
  if (anguloK_S*180/PI>-75){
    //std::cerr << "Angulo kinect/suelo: " << anguloK_S*180/PI << std::endl;
    break;
  }
  else{
    //std::cerr << "Angulo kinect/suelo: " << anguloK_S*180/PI <<  "Recalculo del plan "  << std::endl;
    extract.setInputCloud (cloud.makeShared ());
    extract.setIndices (boost::make_shared<pcl::PointIndices> (inliers));
    extract.setNegative (true);
    extract.filter (cloud);  
      	
  }
 }
	  /*std::cerr << "Model coefficients: " << coefficients.values[0] << " " 
		                              << coefficients.values[1] << " "
		                              << coefficients.values[2] << " " 
		                              << coefficients.values[3] << std::endl;
	  std::cerr << "Model inliers: " << inliers.indices.size () << std::endl;*/
    
    pcl::copyPointCloud(cloud, inliers, cloud2);  
    cloud3=cloud;

    extract.setInputCloud (cloud.makeShared ());
    extract.setIndices (boost::make_shared<pcl::PointIndices> (inliers));
    extract.setNegative (true);
    extract.filter (cloud3);      	

  // Creating the KdTree object for the search method of the extraction
  //Segmentacion por agrupacion
  pcl::search::KdTree<pcl::PointXYZ> tree;
  tree.setInputCloud (cloud3.makeShared ());

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (80);
  ec.setMaxClusterSize (1500);
  ec.setSearchMethod (boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> > (tree));
  ec.setInputCloud (cloud3.makeShared ());
  ec.extract (cluster_indices);

//std::cerr << "Numero de objecto detectado " << cluster_indices.size() << std::endl;


// Examine the clusters and find the nearest centroid
    float min_d = 10000;
    int memorize_indice=0;
    Eigen::Vector4f close;
    cloud_objetos.header=cloud3.header;
    for (unsigned int i = 0; i < cluster_indices.size(); ++i){
      pcl::PointCloud<pcl::PointXYZ> obj_cloud;
      pcl::copyPointCloud(cloud3, cluster_indices[i], obj_cloud); 

      cloud_objetos+=obj_cloud;

      Eigen::Vector4f xyz_centroid;
      pcl::compute3DCentroid(obj_cloud, xyz_centroid);

      if (sqrt(pow(xyz_centroid[0],2)+pow(xyz_centroid[1],2)+ pow(xyz_centroid[2],2))< min_d){ 
        min_d = sqrt(pow(xyz_centroid[0],2)+pow(xyz_centroid[1],2)+ pow(xyz_centroid[2],2));
	close [0]=xyz_centroid[2];
	close [1]=-xyz_centroid[0];
	close [2]=-xyz_centroid[1];
        memorize_indice=i;
      }
    }

//std::cerr << "Distancia minima " << min_d << std::endl;
/*std::cerr << "Nearest " << close[0] << "," 
                        << close[1] << ","
                        << close[2] << std::endl;*/


//Despues de la matriz de rotacion 
Eigen::Vector4f close_Robot;
close_Robot[0]=(close[0]*cos(-anguloK_S)+close[2]*sin(-anguloK_S))*1000-20;
close_Robot[1]=(close[1])*1000;
close_Robot[2]=(-close[0]*sin(-anguloK_S)+close[2]*cos(-anguloK_S))*1000+360;


int sizeMax=0;
if(cluster_indices.size()>0){
  pcl::copyPointCloud(cloud3, cluster_indices[memorize_indice], obj_cloud1);  
  pcl::toROSMsg(obj_cloud1, Objeto_cercano);
  sizeMax=1;
}

 Centroide.x=close_Robot[0];
 Centroide.y=close_Robot[1];
 Centroide.z=close_Robot[2];
	 
//std::cerr << "Nearest en Base_r "<<Centroide.x<<" "<< Centroide.y<<" "<<Centroide.z << std::endl;
//std::cerr << "Puntos del objeto " << obj_cloud1.size() << std::endl;
 

  //Convert the pcl cloud back to rosmsg
  pcl::toROSMsg(cloud2, Plano_suelo);
  pcl::toROSMsg(cloud3, cloud_sobrante);
  pcl::toROSMsg(cloud_objetos, Objetos_detectados);

  //Set the header of the cloud
  //cloud_filtered.header.frame_id = cloud.header.frame_id;
  cloud_sobrante.header.frame_id = cloud.header.frame_id;

  // Publish the data
  pub_Objet.publish(Centroide);
  pub_Plano_suelo.publish (Plano_suelo);
  pub_cloud_sobrante.publish (cloud_sobrante);
  pub_Objeto_cercano.publish (Objeto_cercano);  
  pub_Objetos_detectados.publish (Objetos_detectados); 
 }

 else{
	if (error==false){   
	  std::cerr << "ERROR!! NUMERO DE PUNTOS INSUFICIANTE PARA CALCULOS!! ESPERANDO MAS PUNTOS " << std::endl;
	  error=true;
	}
 }
}


int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "vision");
  ros::NodeHandle nha;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nha.subscribe ("/camera/depth_registered/points", 1, cloud_cb); //input:=/camera/depth_registered/points
  
  pub_Objet=nha.advertise<geometry_msgs::Point> ("Objet",1);
  pub_Plano_suelo = nha.advertise<sensor_msgs::PointCloud2> ("Plano_suelo", 1);
  pub_cloud_sobrante = nha.advertise<sensor_msgs::PointCloud2> ("cloud_sobrante", 1);
  pub_Objeto_cercano = nha.advertise<sensor_msgs::PointCloud2> ("Objeto_cercano", 1);
  pub_Objetos_detectados = nha.advertise<sensor_msgs::PointCloud2> ("Objetos_detectados", 1);
  std::cerr << "PROGRAMA DE VISION EMPEZANDO " << std::endl;


 while (nha.ok()){ 
  ros::spinOnce();
 }
}
