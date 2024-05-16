/*
 *    IDMP - Interactive Distance Field Mapping and Planning to Enable Human-Robot Collaboration
 *    Copyright (C) 2024 Usama Ali
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License v3 as published by
 *    the Free Software Foundation.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License v3 for more details.
 *
 *    You should have received a copy of the GNU General Public License v3
 *    along with this program.  If not, see https://www.gnu.org/licenses/gpl-3.0.html.
 *
 *    Authors: Usama Ali <usama.ali@thws.de>
 *             Adrian Mueller <adrian.mueller@thws.de>
 *             Lan Wu <Lan.Wu-2@uts.edu.au>
 */

#include <IDMP/gp.h>
#include <Eigen/Cholesky>

#define SQRT_3  1.732051

namespace IDMP_ros {

#ifndef RevertingKernel
    inline float kf(float r, float a) {return (1.0+a*r)*exp(-a*r);}
    inline float kf1(float r, float dx, float a) {return a*a*dx*exp(-a*r);}
#else
    inline double kf(float r, float a){return exp(-r*r*a*0.5);}
    inline double kf1(float r, float dx,float a){return -dx*a*exp(-r*r*a*0.5);}
#endif

    void gp::reset() {
        trained = false;
        return;
    }

    // 3D train
    EMatrixX gp::matern32_sparse_deriv1_3D(EMatrixX const& x1, float scale_param, float sigx)
    {
        int dim = x1.rows();
        int n = x1.cols();
        float a = scale_param;
        EMatrixX K = EMatrixX::Zero(n,n);
    
        for (int k=0;k<n;k++){
            for (int j=k;j<n;j++){
                if (k==j){
                    K(k,k) = 1.0+sigx;
                }
                else{
                    float r = (x1.col(k)-x1.col(j)).norm();
                    K(k,j) = kf(r,a);
                    K(j,k) = K(k,j);
                }
            }
        }
    
        return K;
    }

    void gp::train_new(const vecNode3 &samples){
        reset();
    
        int N = samples.size();
        int dim = 3;
    
        if (N > 0){
            x = EMatrixX::Zero(dim,N);
            EVectorX f = EVectorX::Zero(N);
            float sigx = 0.0001;
    
            int k=0;
            for (auto it = samples.begin(); it!=samples.end(); it++, k++){
                x(0,k) = (*it)->getPosX();
                x(1,k) = (*it)->getPosY();
                x(2,k) = (*it)->getPosZ();
                f(k) = 1;
            }
            
            EVectorX y(N);
            y << f;
            EMatrixX K = matern32_sparse_deriv1_3D(x, scale, sigx);
    
            L = K.llt().matrixL();
    
            alpha = y;
            L.template triangularView<Eigen::Lower>().solveInPlace(alpha);
            L.transpose().template triangularView<Eigen::Upper>().solveInPlace(alpha);
    
            trained = true;
    
        }
        return;
    }

    void gp::train_new(const Eigen::Matrix<double,3,Eigen::Dynamic> &samples){
        reset();
    
        int N = samples.cols();    
        if (N > 0){
            auto f = EVectorX::Ones(N);
            float sigx = 0.0001;

            x = samples;
            
            EMatrixX K = matern32_sparse_deriv1_3D(x, scale, sigx);
    
            L = K.llt().matrixL();
    
            alpha = f;
            L.template triangularView<Eigen::Lower>().solveInPlace(alpha);
            L.transpose().template triangularView<Eigen::Upper>().solveInPlace(alpha);
    
            trained = true;
    
        }
        return;
    }
    
    //Test matern
    EMatrixX gp::matern32_sparse_deriv1_3D(EMatrixX const& x1, EMatrixX const& x2, float scale_param)
    {
        int dim = x1.rows();
        int n = x1.cols();
        int m = x2.cols();
        float a = scale_param;
        EMatrixX K = EMatrixX::Zero(n,m);
    
        for (int k=0;k<n;k++){
            for (int j=0;j<m;j++){
                float r = (x1.col(k)-x2.col(j)).norm();
                K(k,j) = kf(r,a);
            }
        }
    
        return K;
    }

    void gp::testSinglePoint_new(const EVectorX& xt, double& val, double grad[], double var[]) {
        if (!isTrained())
            return;
    
        if (x.rows() != xt.size())
            return;
    
        EMatrixX K = matern32_sparse_deriv1_3D(x, xt, scale);
        EVectorX res = K.transpose()*alpha;
        val = res(0);
    
        int n = x.cols();
        int m = xt.cols();    
        float a = scale;
    
        EMatrixX Grx = EMatrixX::Zero(n,m);
        EMatrixX Gry = EMatrixX::Zero(n,m);
        EMatrixX Grz = EMatrixX::Zero(n,m);
    
        for (int k=0;k<n;k++){
            for (int j=0;j<m;j++){
                float r = (x.col(k)-xt.col(j)).norm();
                Grx(k, j) = kf1(r,x(0, k) - xt(0, j),a);
                Gry(k, j) = kf1(r,x(1, k) - xt(1, j),a);
                Grz(k, j) = kf1(r,x(2, k) - xt(2, j),a);
            }
        }
    
        EVectorX gradx = Grx.transpose()*alpha;
        EVectorX grady = Gry.transpose()*alpha;
        EVectorX gradz = Grz.transpose()*alpha;

        grad[0] = gradx(0);  
        grad[1] = grady(0);
        grad[2] = gradz(0);
    
        return;
    }
}
