#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<Eigen/SVD>
// #include<eigen3/Eigen/SVD>
// #include<eigen3>
#include<iostream>
#include<opencv2/core.hpp> // normalize
#include<opencv2/core/eigen.hpp> // cv2eigen
#include<opencv2/core/mat.hpp> // Mat
#include<opencv2/imgcodecs.hpp> // imread
#include<opencv2/imgproc.hpp> // cvtColor
#include<opencv2/highgui.hpp> // imshow, waitKey
// #include <opencv2/core/eigen.hpp> // cv2eigen

using namespace Eigen;
// using namespace cv2;
using namespace cv;
// using namespace std;




int main(int argc, char** argv){
  // prob 1
  Vector4f v, w;
  Vector3f norm_v, norm_w;
  v << 1, 1.5, 2, 3;
  w << 0, 1, 2, 4;
  norm_v = v.hnormalized();
  norm_w = w.hnormalized();
  std::cout << "Problem 1:" << std::endl;
  std::cout << "v is " << std::endl << v << std::endl;
  std::cout << "w is " << std::endl << w << std::endl;
  std::cout << "normalized v+w is " << std::endl << norm_v+norm_w << std::endl;
  std::cout << "normalized v inner product w is " << std::endl << norm_v.dot(norm_w) << std::endl;
  std::cout << "normalized norm_v cross norm_w is " << std::endl << norm_v.cross(norm_w) << std::endl;
  std::cout << "normalized norm_w cross norm_v is " << std::endl << norm_w.cross(norm_v) << std::endl;


  // prob 2
  Matrix4f i, j;
  i << 1, 2, 3, 4,
      5, 6, 7, 8,
      9, 10, 11, 12,
      13, 14, 15, 16;
  j << 4, 3, 2, 1,
      8, 7, 6, 5,
      12, 11, 10, 9,
      16, 15, 14, 13;
  std::cout << std::endl << "Problem 2:" << std::endl;
  std::cout << "i is " << std::endl << i << std::endl;
  std::cout << "j is " << std::endl << j << std::endl;
  std::cout << "i+j is " << std::endl << i+j << std::endl;
  std::cout << "i*j is " << std::endl << i*j << std::endl;
  std::cout << "i*v is " << std::endl << i*v << std::endl;


  // prob 3
  std::string path = "./lenna.png";
  cv::Mat lenna = cv::imread(path, 1), lennaGrey; 

  // grey-scale cvting
  // lenna *= 1./255;
  cv::cvtColor(lenna, lennaGrey, COLOR_BGR2GRAY); 

  // cvt eigen and do range normalization
  MatrixXf imgNorm(lennaGrey.rows, lennaGrey.cols), imgOriGrey(lennaGrey.rows, lennaGrey.cols);
  cv::cv2eigen(lennaGrey, imgOriGrey);
  // std::cout << "eig img: " << imgNorm << "\n";
  imgNorm = imgOriGrey * (1./255); 
  // std::cout << "hey alive here!\n";
  // std::cout << "norm img rows and cols: " << imgNorm.rows() << " " << imgNorm.cols() << std::endl;


  // SVD 
  // do SVD
  VectorXf singVals(imgNorm.cols());
  MatrixXf U(imgNorm.rows(), imgNorm.rows()),
          S(imgNorm.rows(), imgNorm.cols()),
          V(imgNorm.cols(), imgNorm.cols());
  JacobiSVD<MatrixXf> svd(imgNorm, ComputeFullU | ComputeFullV);
  // unpack matrices and singular values
  U = svd.matrixU();
  V = svd.matrixV();
  singVals = svd.singularValues();

  // SET when given
  // std::cout << "dim(U)="<<U.rows()<<"*"<<U.cols()<<", din(V)="<<V.rows()<<"*"<<V.cols()<<", dim(singval)="<<singVals.rows()<< "\n";

  S.setZero();
  for (int i = 0; i < imgNorm.cols(); i++) {
    S(i,i) = singVals[i];
    // std::cout << "assigning " << singVals[i] << " to " <<  i << "th diag entry" << std::endl;
  }

  // std::cout << "S is : " << S << std::endl;


  // dif sing-val saving results: def mat reses and do slicing
  MatrixXf imgOne(imgNorm.rows(), imgNorm.cols()), 
          imgTen(imgNorm.rows(), imgNorm.cols()), 
          imgFifty(imgNorm.rows(), imgNorm.cols());
  
  // multiply to get results and show img
  imgOne = U * S(all, seq(0,0)) * V(all, seq(0,0)).transpose();
  imgTen = U * S(all, seq(0,9)) * V(all, seq(0,9)).transpose();
  imgFifty = U * S(all, seq(0,49)) * V(all, seq(0,49)).transpose();

  // cvt back cv and show out
  cv::Mat lennaOne, lennaTen, lennaFifty, dst;
  cv::eigen2cv(imgOne, lennaOne);
  cv::eigen2cv(imgTen, lennaTen);
  cv::eigen2cv(imgFifty, lennaFifty);


  cv::imwrite("lennagrey.png", lennaGrey);
  cv::imwrite("lenna.png", lenna);
  cv::imwrite("lenna-1-singular-value.png", lennaOne*255);
  cv::imwrite("lenna-10-singular-value.png", lennaTen*255);
  cv::imwrite("lenna-50-singular-value.png", lennaFifty*255);
  // std::cout << "svd done \n";
  // cv::waitKey(0);
  std::cout << std::endl << "Problem 3:" << std::endl << "Results had been written at build/" << std::endl;


  // prob 4
  // a -> initial pt, b -> final pt
  Eigen::Vector3d a, b;
  Eigen::Vector4d a_hom, b_hom;
  Eigen::Matrix4d tran_ori, rot_x, rot_y, rot_z, tran_rot_pt;
  a << 1, 2, 3;
  a_hom << a, 1; 
  // tran to origin
  tran_ori << 1, 0, 0, -4,
              0, 1, 0, -5,
              0, 0, 1, -6,
              0, 0, 0, 1;
  // std::cout << "tran dn" << tran_ori *a_hom << std::endl;
  // rot x, y, z
  rot_x << 1, 0, 0, 0, 
           0, cos(M_PI/4), -sin(M_PI/4), 0,
           0, sin(M_PI/4), cos(M_PI/4), 0,
           0, 0, 0, 1;

  // std::cout << "rot x dn" << rot_x *tran_ori *a_hom << std::endl;

  rot_y << cos(M_PI/6), 0, sin(M_PI/6), 0,
           0, 1, 0, 0,
           -sin(M_PI/6), 0, cos(M_PI/6), 0,
           0, 0, 0, 1;
  // std::cout << "rot y dn" << rot_y *rot_x *tran_ori *a_hom << std::endl;

  rot_z << cos(M_PI/3), -sin(M_PI/3), 0, 0,
           sin(M_PI/3), cos(M_PI/3), 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1;
  // std::cout << "rot z dn" << rot_z *rot_y *rot_x *tran_ori *a_hom << std::endl;

  // tran to rot pt
  tran_rot_pt << 1, 0, 0, 4,
              0, 1, 0, 5,
              0, 0, 1, 6,
              0, 0, 0, 1;
  // std::cout << "rot tran dn" << tran_rot_pt* rot_z *rot_y *rot_x *tran_ori *a_hom << std::endl;

  // transform 
  // std::cout << "rot starts\n";
  b_hom = tran_rot_pt* rot_x *rot_y *rot_z *tran_ori *a_hom;
  b(0) = b_hom(0) / b_hom(3);
  b(1) = b_hom(1) / b_hom(3);
  b(2) = b_hom(2) / b_hom(3);

  // print results
  std::cout << std::endl << "Problem 4:" << std::endl;
  std::cout << "Original point " << std::endl << a << std::endl;
  std::cout << "New point " << std::endl << b << std::endl;
  return 0;
}
