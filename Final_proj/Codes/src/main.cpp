#include "CGL/CGL.h"
#include "collada.h"
#include "meshEdit.h"
#include "bezierPatch.h"
#include "bezierCurve.h"
#include "mergeVertices.h"
#include "shaderUtils.h"
#include "front.hpp"
#include <stdlib.h>
#include <iostream>

// Poisson surface reconstruction
#include <pcl/point_types.h>				
#include <pcl/point_cloud.h>				
#include <pcl/kdtree/kdtree_flann.h>		
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>			
#include <pcl/io/ply_io.h>
#include <pcl/surface/poisson.h>     
#include <pcl/common/common.h>				
#include <pcl/common/angles.h>				
#include <pcl/common/centroid.h>			
#include <pcl/common/distances.h>			
#include <pcl/common/file_io.h>				
#include <pcl/common/random.h>				
#include <pcl/common/geometry.h>			
#include <pcl/common/intersections.h>		
#include <pcl/common/norms.h>				
#include <pcl/common/time.h>


using namespace std;
using namespace CGL;

#define msg(s) cerr << "[Collada Viewer] " << s << endl;
bool bp=1;




void poisson(string plyInputPath, string plyOutputPath, Polymesh &Poly) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPLYFile (plyInputPath, cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *object_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(*object_cloud, *cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::PassThrough<pcl::PointXYZ> filter;
  filter.setInputCloud(cloud);
  filter.filter(*filtered);
  cout << "passthrough filter complete" << endl;

  cout << "begin normal estimation" << endl;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(filtered);
  ne.setRadiusSearch(0.1); // set the radius as 0.1
  Eigen::Vector4f centroid;
  compute3DCentroid(*filtered, centroid);
  ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
  ne.compute(*cloud_normals);
  cout << "normal estimation complete" << endl;
  cout << "reverse normals' direction" << endl;

  for(size_t i = 0; i < cloud_normals->size(); ++i)
  {
      cloud_normals->points[i].normal_x *= -1;
      cloud_normals->points[i].normal_y *= -1;
      cloud_normals->points[i].normal_z *= -1;
  }

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointNormal>());
  concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);

  //Poisson reconstruction
  cout << "begin poisson reconstruction" << endl;
  pcl::Poisson<pcl::PointNormal> poisson;
  poisson.setDepth(8);
  poisson.setSolverDivide (6);
  poisson.setIsoDivide (6);

  poisson.setConfidence(false);
  poisson.setManifold(true);
  poisson.setOutputPolygons(false);

  poisson.setInputCloud(cloud_smoothed_normals);
  pcl::PolygonMesh mesh;
  poisson.reconstruct(mesh);

  cout << "finish poisson reconstruction" << endl;

  pcl::PointCloud<pcl::PointXYZ> cloud_mesh;
  pcl::fromPCLPointCloud2(mesh.cloud, cloud_mesh);

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud);
  // K nearest neighbor search
  int K = 5;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  for(int i=0;i<cloud_mesh.points.size();++i)
  {
      float dist = 0.0;

      if ( kdtree.nearestKSearch (cloud_mesh.points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
      {
          for (int j = 0; j < pointIdxNKNSearch.size (); ++j)
          {
              dist += 1.0/pointNKNSquaredDistance[j];
              // cout<<"dis:"<<dist<<endl;
          }
      }
  }
  toPCLPointCloud2(cloud_mesh, mesh.cloud);
  PolyList pl;
  for (auto p: mesh.polygons){
    Polygon tri;
    tri.vertex_indices.push_back(p.vertices[0]);
    tri.vertex_indices.push_back(p.vertices[1]);
    tri.vertex_indices.push_back(p.vertices[2]);
    pl.push_back(tri);
  }
  Poly.polygons = pl;
  pcl::io::savePLYFile(plyOutputPath, mesh);
}


int parsePly(const char *path, vector<Vector3D> *vertices, vector< vector<int> > *face_vertex_indices) {
  // open file
  FILE* file = fopen(path, "r");
  int vertex_count = 0;
  int face_count = 0;
  char lineHeader[512];

  // scan file line by line
  while (true) {
    int res = fscanf(file, "%s", lineHeader);

    // finish scanning
    if (res == EOF || strcmp(lineHeader, "end_header") == 0) {
      break;
    }
    
    // allocate memory based on element type
    if (strcmp(lineHeader, "element") == 0) {
      char type[128];
      int count;
      fscanf(file, "%s %d\n", type, &count);
      if (strcmp(type, "vertex") == 0) vertex_count = count;
      if (strcmp(type, "face") == 0) face_count = count;
    }
  }
  
  // cout << "ply parsing\nVertex count: " << vertex_count << "\nFace count: " << face_count << endl;
  
  // parse vertices
  while (--vertex_count >= 0) {
    Vector3D vertex;
    fscanf(file, "%lf %lf %lf\n", &vertex.x, &vertex.y, &vertex.z);
    vertices->push_back(vertex);
  }

  // parse faces (if have)
  while (--face_count >= 0) {
    int vertex_num, v1_index, v2_index, v3_index;
    fscanf(file, "%d %d %d %d\n", &vertex_num, &v1_index, &v2_index, &v3_index);
    // if (vertex_num != 3) {
    //   cout << "Encounter non-triangular shape in .ply!\n";
    // }
    vector<int> vertex_indices;
    vertex_indices.push_back(v1_index);
    vertex_indices.push_back(v2_index);
    vertex_indices.push_back(v3_index);
    face_vertex_indices->push_back(vertex_indices);
  } 
  return 1;
}

int loadFile(MeshEdit* collada_viewer, const char* path) {

  Scene* scene = new Scene();

  std::string path_str = path;
  if (path_str.substr(path_str.length()-4, 4) == ".dae") {
    if (ColladaParser::load(path, scene) < 0) {
      delete scene;
      return -1;
    }
  }

  else if (path_str.substr(path_str.length()-4, 4) == ".bez") {
    Camera* cam = new Camera();
    cam->type = CAMERA;
    Node node;
    node.instance = cam;
    scene->nodes.push_back(node);
    Polymesh* mesh = new Polymesh();

    FILE* file = fopen(path, "r");
    int n = 0;
    fscanf(file, "%d", &n);
    for (int i = 0; i < n; i++)
    {
      BezierPatch patch;
      patch.loadControlPoints(file);
      patch.add2mesh(mesh);
      mergeVertices(mesh);
    }
    fclose(file);

    mesh->type = POLYMESH;
    node.instance = mesh;
    scene->nodes.push_back(node);
  }

  // load .ply file for bpa
  else if ((path_str.substr(path_str.length()-4, 4) == ".ply") && (bp)) {
    vector<Vector3D> vertices = vector<Vector3D>();
    vector<vector<int>> face_vertex_indices = vector< vector<int> >();
    
    parsePly(path, &vertices, &face_vertex_indices);

    // Construct vertex-face mapping
    vector<vector<int>> vertex_face_indices(vertices.size());
    for (int face_iter = 0; face_iter < face_vertex_indices.size(); face_iter++) {
      vector<int> vertex_indices = face_vertex_indices[face_iter];
      for (int vertex_index : vertex_indices) {
        vertex_face_indices[vertex_index].push_back(face_iter);
      }
    }

    // Compute face normals
    vector<Vector3D> face_normals = vector<Vector3D>();
    for (vector<int> vertex_indices : face_vertex_indices) {
      Vector3D v1 = vertices[vertex_indices[0]];
      Vector3D v2 = vertices[vertex_indices[1]];
      Vector3D v3 = vertices[vertex_indices[2]];
      face_normals.push_back(cross(v2 - v1, v3 - v1).unit());
    }

    // Compute vertex normals
    vector<Vector3D> vertex_normals = vector<Vector3D>();
    for (int vertex_iter = 0; vertex_iter < vertices.size(); vertex_iter++) {
      Vector3D vertex_normal = Vector3D();
      for (int face_index : vertex_face_indices[vertex_iter]) vertex_normal += face_normals[face_index];
      vertex_normals.push_back(vertex_normal.unit());
    }
    
    Camera* cam = new Camera();
    cam->type = CAMERA;
    Node node;
    node.instance = cam;
    scene->nodes.push_back(node);
    PointCloud* point_cloud = new PointCloud();
    Polymesh *polymesh = new Polymesh();
    
    // Add vertices to scene
    for (Vector3D v : vertices) {
      point_cloud->vertices.push_back(v);
      polymesh->vertices.push_back(v);
    }

    // Add normals to scene
    for (Vector3D n : vertex_normals) {
      point_cloud->normals.push_back(n);
    }
    
    point_cloud->type = POINT_CLOUD;
    node.instance = point_cloud;
    scene->nodes.push_back(node);
  }
  else {
    return -1;
  }

  collada_viewer->load( scene );

  GLuint tex = makeTex("envmap/envmap.png");
  if(!tex) tex = makeTex("../envmap/envmap.png");
  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D, tex);
  glActiveTexture(GL_TEXTURE2);

  return 0;
}

int main( int argc, char** argv ) {
  if( argc != 4 ) {msg("Usage: ./meshedit xxx.ply rou_value BPA_or_Poisson_flag_0_or_1"); exit(0);}
  // ./meshedit xxx.ply 0.5 1

  // load file path
  const char* path = argv[1];
  std::string path_str = path;

  // create viewer
  Viewer viewer = Viewer();

  // create collada_viewer
  MeshEdit* collada_viewer = new MeshEdit();
  collada_viewer->rou = atof(argv[2]);
  // cout << collada_viewer->rou;

  // reconstruction mode selection
  Polymesh poisson_mesh = Polymesh();
  bp = atof(argv[3]);
  if (!bp) {
    poisson(path_str, "poisson_reconstruction_result.ply", poisson_mesh);
    cout << "reconstruction completed, output had been written to the current directory\n";
    return 0;
  }
  
  // set collada_viewer as renderer
  viewer.set_renderer(collada_viewer);

  // init viewer
  viewer.init();

  // load tests
  if ( argc == 4 ) {
    if (loadFile(collada_viewer, argv[1]) < 0) exit(0);
  } 
  else {
    msg("Usage: ./meshedit xxx.ply rou_value BPA_or_Poisson_flag_0_or_1"); 
    exit(0);
  }

  // start viewer
  viewer.start();
  
  return 0;
}
