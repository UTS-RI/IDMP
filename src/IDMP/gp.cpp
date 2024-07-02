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
    inline double kf(double r, double a){return exp(-r*r*a*0.5);}
    inline double kf1(double r, double dx,double a){return -dx*a*exp(-r*r*a*0.5);}
    inline double kf2(double r, double dx1, double dx2, double delta, double a){return (dx1*dx2*a*a-delta*a)*exp(-r*r*a*0.5);}
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
#ifndef RevertingKernel
        val = -(inv_scale)*log(abs(res(0)));
#else
        val = sqrt(abs(-2*inv_scale*log(abs(res(0)))));
#endif

        if(val > 0.05) {
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
        } else {
            double a = scale;
            EMatrixX Hrxx = EMatrixX::Zero(n,m);
            EMatrixX Hrxy = EMatrixX::Zero(n,m);
            EMatrixX Hrxz = EMatrixX::Zero(n,m);
            EMatrixX Hryx = EMatrixX::Zero(n,m);
            EMatrixX Hryy = EMatrixX::Zero(n,m);
            EMatrixX Hryz = EMatrixX::Zero(n,m);
            EMatrixX Hrzx = EMatrixX::Zero(n,m);
            EMatrixX Hrzy = EMatrixX::Zero(n,m);
            EMatrixX Hrzz = EMatrixX::Zero(n,m);

            EMatrixX normHessian = EMatrixX::Zero(3,3);

            for (int k=0;k<n;k++){
                for (int j=0;j<m;j++){
                    double r = (x.col(k)-xt.col(j)).norm();
                    Hrxx(k, j) = kf2(r,x(0,k)-xt(0,j),x(0,k)-xt(0,j),1.0,a);
                    Hrxy(k, j) = kf2(r,x(0,k)-xt(0,j),x(1,k)-xt(1,j),0.0,a);
                    Hrxz(k, j) = kf2(r,x(0,k)-xt(0,j),x(2,k)-xt(2,j),0.0,a);
                    Hryx(k, j) = Hrxy(k, j);
                    Hryy(k, j) = kf2(r,x(1,k)-xt(1,j),x(1,k)-xt(1,j),1.0,a);
                    Hryz(k, j) = kf2(r,x(1,k)-xt(1,j),x(2,k)-xt(2,j),0.0,a);
                    Hrzx(k, j) = Hrxz(k, j);
                    Hrzy(k, j) = Hryz(k, j);
                    Hrzz(k, j) = kf2(r,x(2,k)-xt(2,j),x(2,k)-xt(2,j),1.0,a);
                }
            }

            EVectorX hesxx = Hrxx.transpose()*alpha;
            EVectorX hesxy = Hrxy.transpose()*alpha;
            EVectorX hesxz = Hrxz.transpose()*alpha;
            EVectorX hesyx = Hryx.transpose()*alpha;
            EVectorX hesyy = Hryy.transpose()*alpha;
            EVectorX hesyz = Hryz.transpose()*alpha;
            EVectorX heszx = Hrzx.transpose()*alpha;
            EVectorX heszy = Hrzy.transpose()*alpha;
            EVectorX heszz = Hrzz.transpose()*alpha;

            normHessian(0,0) = hesxx(0);
            normHessian(0,1) = hesxy(0);
            normHessian(0,2) = hesxz(0);
            normHessian(1,0) = hesyx(0);
            normHessian(1,1) = hesyy(0);
            normHessian(1,2) = hesyz(0);
            normHessian(2,0) = heszx(0);
            normHessian(2,1) = heszy(0);
            normHessian(2,2) = heszz(0);

            Eigen::EigenSolver<EMatrixX> es(normHessian);
            EVectorX evalue = es.eigenvalues().real();
            EMatrixX evector = es.eigenvectors().real();
            EVectorX::Index index;
            double minv = evalue.minCoeff(&index);
            auto normalV = evector.col(index);
            grad[0] = normalV(0);  
            grad[1] = normalV(1);
            grad[2] = normalV(2);
        }
#ifndef RevertingKernel
        double gradLen = -1 * sqrt(pow(grad[0], 2) + pow(grad[1], 2) + pow(grad[2], 2)); 
#else
        double gradLen = sqrt(pow(grad[0], 2) + pow(grad[1], 2) + pow(grad[2], 2)); 
#endif
        if(gradLen != 0){
            grad[0]/=gradLen;
            grad[1]/=gradLen;
            grad[2]/=gradLen;
        } else {
            grad[0]=0;
            grad[1]=0;
            grad[2]=0;
        }  
        return;
    }
}
