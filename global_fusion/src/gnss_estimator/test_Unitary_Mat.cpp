/*******************************************************
 * Copyright (C) 2019, Intelligent Positioning and Navigation Lab, Hong Kong Polytechnic University
 * 
 * This file is part of fgo_gnss.
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Weisong Wen (weisong.wen@connect.polyu.hk)
 * Function: test the unitary matrix generation using Eigen library
 * Date: 2021/01/19
 *******************************************************/

// std inputs and outputs, fstream
#include <iostream>
#include <string>  
#include <fstream>
#include<sstream>
#include <stdlib.h>
#include <iomanip>

// math
#include <math.h>
//time 
#include <time.h>
//algorithm 
#include <algorithm>

// google eigen
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include<Eigen/Core>

// google implements commandline flags processing.
#include <gflags/gflags.h>
// google loging tools
#include <glog/logging.h>
// ros
#include <ros/ros.h>
/* Reference from NovAtel GNSS/INS */
#include <novatel_msgs/INSPVAX.h> // novatel_msgs/INSPVAX
#include "gnss_tools.h"
#include <nlosExclusion/GNSS_Raw_Array.h>

#include <geometry_msgs/Point32.h>
#include <stdio.h>
#include <queue>
#include <map>
#include <queue>
#include <mutex>
#include <thread>
#include <nav_msgs/Odometry.h>

#include <nav_msgs/Path.h>


#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "ceres/dynamic_autodiff_cost_function.h"


#include "../tic_toc.h"

// rtklib /
#include <stdarg.h>
#include "../RTKLIB/src/rtklib.h"

/* library for generate unitary matrix */
#include<random>
#include<ctime>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Eigen/LU> // required for MatrixBase::determinant
#include <Eigen/SVD> // required for SVD

#include <armadillo> 
// #include <RcppArmadillo.h>
// #include <RcppEigen.h>

using namespace Eigen;

using namespace std;
using namespace Eigen;
using namespace arma; 


#define realUnitaryMatrix 0

static default_random_engine e(time(0));
static normal_distribution<double> gaussian(0,1);

MatrixXd randomOrthogonalMatrix(const unsigned long n){
  MatrixXd X = MatrixXd::Zero(n,n).unaryExpr([](double dummy){return gaussian(e);});
  MatrixXd XtX = X.transpose() * X;
  SelfAdjointEigenSolver<MatrixXd> es(XtX);
  MatrixXd S = es.operatorInverseSqrt();
  return X * S;
}

//  Constructs a random matrix from the unitary group U(size).
template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> randMatrixUnitary(int size)
{
  typedef T Scalar;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixType;

  MatrixType Q;

  int max_tries = 40;
  double is_unitary = false;


  while (!is_unitary && max_tries > 0)
  {
    // initialize random matrix
    Q = MatrixType::Random(size, size);

    // orthogonalize columns using the Gram-Schmidt algorithm
    for (int col = 0; col < size; ++col)
    {
      typename MatrixType::ColXpr colVec = Q.col(col);
      for (int prevCol = 0; prevCol < col; ++prevCol)
      {
        typename MatrixType::ColXpr prevColVec = Q.col(prevCol);
        colVec -= colVec.dot(prevColVec)*prevColVec;
      }
      Q.col(col) = colVec.normalized();
    }

    // this additional orthogonalization is not necessary in theory but should enhance
    // the numerical orthogonality of the matrix
    for (int row = 0; row < size; ++row)
    {
      typename MatrixType::RowXpr rowVec = Q.row(row);
      for (int prevRow = 0; prevRow < row; ++prevRow)
      {
        typename MatrixType::RowXpr prevRowVec = Q.row(prevRow);
        rowVec -= rowVec.dot(prevRowVec)*prevRowVec;
      }
      Q.row(row) = rowVec.normalized();
    }

    // final check
    is_unitary = Q.isUnitary();
    --max_tries;
  }

  if (max_tries == 0)
    eigen_assert(false && "randMatrixUnitary: Could not construct unitary matrix!");

  return Q;
}


//  Constructs a random matrix from the unitary group U(size).
// template <typename T>
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> CusrandMatrixUnitary(int size)
{
//   typedef T Scalar;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixType;

  MatrixType Q;

  int max_tries = 400;
  double is_unitary = false;

  while (!is_unitary && max_tries > 0)
  {
    // initialize random matrix
    Q = MatrixType::Random(size, size);

    // orthogonalize columns using the Gram-Schmidt algorithm
    for (int col = 0; col < size; ++col)
    {
      typename MatrixType::ColXpr colVec = Q.col(col);
      for (int prevCol = 0; prevCol < col; ++prevCol)
      {
        typename MatrixType::ColXpr prevColVec = Q.col(prevCol);
        colVec -= colVec.dot(prevColVec)*prevColVec;
      }
      Q.col(col) = colVec.normalized();
    }

    // this additional orthogonalization is not necessary in theory but should enhance
    // the numerical orthogonality of the matrix
    for (int row = 0; row < size; ++row)
    {
      typename MatrixType::RowXpr rowVec = Q.row(row);
      for (int prevRow = 0; prevRow < row; ++prevRow)
      {
        typename MatrixType::RowXpr prevRowVec = Q.row(prevRow);
        rowVec -= rowVec.dot(prevRowVec)*prevRowVec;
      }
      Q.row(row) = rowVec.normalized();
    }

    // final check
    is_unitary = Q.isUnitary();
    --max_tries;
  }

  if (max_tries == 0)
    eigen_assert(false && "randMatrixUnitary: Could not construct unitary matrix!");

  return Q;
}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> kernel_COD(
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& M) {
  Eigen::CompleteOrthogonalDecomposition<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>
      cod;
  cod.compute(M);
  unsigned rk = cod.rank();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P =
      cod.colsPermutation();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V =
      cod.matrixZ().transpose();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Kernel =
      P * V.block(0, rk, V.rows(), V.cols() - rk);
  // std::cout<<"Kernel -> \n" << Kernel << std::endl;
  return Kernel;
}


/* transfer the armadillo matrix to the eigen matrix */
Eigen::MatrixXd armadillo_to_eigen(arma::mat arma_A) {

  Eigen::MatrixXd eigen_B = Eigen::Map<Eigen::MatrixXd>(arma_A.memptr(),
                                                        arma_A.n_rows,
                                                        arma_A.n_cols);

  return eigen_B;
} 

/* transfer the eigen matrix to the armadillo matrix */
arma::mat eigen_to_arma(Eigen::MatrixXd eigen_A) {

  arma::mat arma_B = arma::mat(eigen_A.data(), eigen_A.rows(), eigen_A.cols(),
                               false, false);

  return arma_B;
}

int main(int argc, char **argv)
{

    
    FLAGS_logtostderr = 1;  // output to console
    google::InitGoogleLogging(argv[0]); // init the google logging
    // google::ParseCommandLineFlags(&argc, &argv, true); // parseCommandLineFlags 
    ros::init(argc, argv, "test_Unitary_Mat_node");   
    ROS_INFO("\033[1;32m----> test_Unitary_Mat_node (test unitary matrix) Started.\033[0m");

    int iterNum = 0;
    int matSize = 100;
    Eigen::MatrixXd orMat;

    #if realUnitaryMatrix
    /* get the orthogonal matrix */
    while(iterNum < 1)
    { 
        iterNum++;
        orMat = randomOrthogonalMatrix(matSize);
        std::cout<<"orthogonal matrix -> \n" << orMat << std::endl;
        std::cout<<"test norm -> \n" << orMat * orMat.transpose() << std::endl;
    }

    /* generate the diagonal matrix */
    std::vector<double> diagVector;
    int max;
    max = 100; //set the upper bound to generate the random number
    srand(time(0));
    for(int i = 0; diagVector.size()<matSize; i++) { //generate random numbers
        double value = (rand()%max);
        value = value /(float)max;
        cout << "The random number is: "<<value << endl;
        bool exists = std::find(std::begin(diagVector), std::end(diagVector), value) != std::end(diagVector); 
        if(!exists)
            diagVector.push_back(value);
    }
    Eigen::MatrixXd diaMat;
    diaMat.resize(matSize, matSize);
    diaMat.setIdentity();
    for(int i = 0; i < matSize; i++)
    {
        diaMat(i,i) = diaMat(i,i) * diagVector[i];
    }
    std::cout<<"diaMat matrix -> \n" << diaMat << std::endl;

    /* generate diaMat2, diaMat2Square */
    Eigen::MatrixXd IMat, diaMat2, diaMat2Square, BMat;
    IMat.resize(matSize, matSize);
    IMat.setIdentity();
    diaMat2Square = IMat - diaMat * diaMat;
    diaMat2 = diaMat2Square.cwiseSqrt();
    std::cout<<"diaMat2 matrix -> \n" << diaMat2 << std::endl;
    BMat = orMat.transpose() * diaMat2 * orMat;
    std::cout<<"BMat matrix -> \n" << BMat << std::endl;

    Eigen::MatrixXd ranDomUnaryMat = CusrandMatrixUnitary(5);
    // ranDomUnaryMat.col(0) *= numext::conj(ranDomUnaryMat.determinant());
    std::cout<<"ranDomUnaryMat matrix -> \n" << ranDomUnaryMat << std::endl;

    Eigen::MatrixXd oneVector;
    oneVector.resize(matSize,1);
    for(int i = 0; i < matSize; i++)
    {
        oneVector(i) = 1;
    }
    std::cout<<"ranDomUnaryMat matrix check -> \n" << ranDomUnaryMat * oneVector<< std::endl;
    #endif

    /* generate vector */
    Eigen::MatrixXd onz;
    onz.resize(matSize, 1);
    for(int i = 0; i < matSize; i++)
    {
      onz(i,0) = 1;
    }

    /* get the null space using Eigen (FAIL) */
    // Eigen::FullPivLU<Eigen::MatrixXd> lu(onz);
    // Eigen::MatrixXd A_null_space = lu.kernel(); 
    // std::cout<<"onz -> \n" << onz << std::endl;
    // std::cout<<"A_null_space -> \n" << A_null_space << std::endl;

    /* get the null space using armadillo */
    /* Step 1: calculate null space in armadillo */
    arma::mat onzArma = arma::mat(onz.data(), onz.cols(), onz.rows(),
                               false, false);
    onzArma.print("onzArma:"); 
    std::cout<<"onzArma.size() -> \n" << onzArma.size() << std::endl;
    
    std::cout<<"onzArma.n_rows() -> \n" << onzArma.n_rows << std::endl;
    std::cout<<"onzArma.n_cols() -> \n" << onzArma.n_cols << std::endl;
    arma::mat nullSpaceMatArma = arma::null(onzArma);
    nullSpaceMatArma.print("nullSpaceMatArma:"); 
    std::cout<<"nullSpaceMatArma.size() -> \n" << nullSpaceMatArma.size() << std::endl;
    std::cout<<"nullSpaceMatArma.n_rows() -> \n" << nullSpaceMatArma.n_rows << std::endl;
    std::cout<<"nullSpaceMatArma.n_cols() -> \n" << nullSpaceMatArma.n_cols << std::endl;

    Eigen::MatrixXd nullSpaceEigen = Eigen::Map<Eigen::MatrixXd>(nullSpaceMatArma.memptr(),
                                                        nullSpaceMatArma.n_rows,
                                                        nullSpaceMatArma.n_cols);
    /* transform the armadillo matrix to Eigen matrix */
    std::cout<<"nullSpaceEigen -> \n" << nullSpaceEigen<< std::endl;

    /************* get the H matrix ***************/
    /* Step 1: construct the random matrix */
    Eigen::MatrixXd H1, H2;
    Eigen::MatrixXcd H12;
    H1.resize(matSize-1,matSize-1);
    H2.resize(matSize-1,matSize-1);
    H12.resize(matSize-1,matSize-1);
    H1 = Eigen::MatrixXd::Random(matSize-1,matSize-1);
    H2 = Eigen::MatrixXd::Random(matSize-1,matSize-1);
    H12.real() = H1;
    H12.imag() = H2;
    H12 = 2 * H12;
    std::cout<<"H1 -> \n" << H1<< std::endl;
    std::cout<<"H2 -> \n" << H2<< std::endl;
    std::cout<<"H12 -> \n" << H12<< std::endl;

    Eigen::MatrixXcd H34;
    H34.resize(matSize-1,matSize-1);
    Eigen::MatrixXd H34Value;
    H34Value.resize(matSize-1,matSize-1);
    for(int i = 0; i < matSize-1; i++)
    {
      for(int j = 0; j < matSize-1; j++)
      {
        H34Value(i,j) = 1.0;
      }
    }
    H34.real() = H34Value;
    H34.imag() = H34Value;
    std::cout<<"H34 -> \n" << H34<< std::endl;

    /* Step 2: construct the H matrix */
    Eigen::MatrixXcd H;
    H.resize(matSize-1,matSize-1);
    H = H12 - H34;
    std::cout<<"H -> \n" << H<< std::endl;
    std::cout<<"H.real() -> \n" << H.real()<< std::endl;

    /* Step 3: get the Q matrix from H using QR decomposition */
    Eigen::ColPivHouseholderQR<Eigen::MatrixXcd> qrT(H);
    Eigen::MatrixXcd r = qrT.matrixQR().triangularView<Upper>();
    // std::cout<<"r -> \n" << r<< std::endl;
    // std::cout<<"Q -> \n" << qrT.householderQ<< std::endl;
    Eigen::ColPivHouseholderQR< Eigen::MatrixXcd >::HouseholderSequenceType  seq = qrT.householderQ();
    std::cout<<"Q -> \n" << (Eigen::MatrixXcd)seq<< std::endl;
    Eigen::MatrixXcd Qmatrix = (Eigen::MatrixXcd)seq;

    Eigen::MatrixXcd UnitaryMat;
    UnitaryMat.resize(matSize,matSize);
    UnitaryMat = nullSpaceEigen * Qmatrix * nullSpaceEigen.transpose();
    UnitaryMat = UnitaryMat + (1/(double)matSize) * onz * onz.transpose();
    std::cout<<"UnitaryMat -> \n" << UnitaryMat << std::endl;
    std::cout<<"UnitaryMat.imag() -> \n" << UnitaryMat.imag()<< std::endl;
    std::cout<<"UnitaryMat* onz -> \n" << UnitaryMat * onz<< std::endl;


    // std::cout << q.diagonal().transpose() << std::endl;

    // arma::mat Q, R;
    // arma::qr(Q,R,H);
    // Q.print("Q:"); 
    // R.print("R:"); 




    /* construct an (n-1)x(n-1) unitary matrix by employing random numbers 
       uniformly distributed on {-1, 1} x {-i, i} */

    // Eigen::MatrixXd testMat = MatrixXd::Random(5,5);
    // testMat = kernel_COD(testMat);
    // std::cout<<"kernel_COD(testMat) -> \n" << testMat << std::endl;

    // arma::mat A(5, 1);// directly specify the matrix size (elements are uninitialised) 
    // A << 1 << 1 << 1 << 1 <<1 << endr ; 
    // A.print("A:"); 
    // // arma::mat B = arma::orth(A); 
    // arma::mat B = arma::null(A); 
    // B.print("B:"); 
    // cout << "The rank of A is " << arma::rank(A) << endl; 

    ros::spin();
    return 0;
}
