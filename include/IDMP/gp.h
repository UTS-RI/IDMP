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

#ifndef __GP_H__
#define __GP_H__

#include <vector>
#include <memory>
#include <iostream>
#include <cstdint>
#include <Eigen/Dense>
#include "strct.h"
#include "params.h"

namespace IDMP_ros
{

    typedef Eigen::MatrixXd EMatrixX;
    typedef Eigen::VectorXd EVectorX;
    typedef Eigen::RowVectorXd ERowVectorX;

    typedef std::vector<std::shared_ptr<Node3> > vecNode3;

    class gp {
        EMatrixX x;
        EMatrixX L;
        EVectorX alpha;
        std::vector<float> gradflag;

        double scale;
        double inv_scale;
        double inv_sqrt_scale;
        
        bool trained;

    public:
        gp() :  scale(DEFAULT_MAP_SCALE_PARAM),
                inv_scale(1 / DEFAULT_MAP_SCALE_PARAM),
                inv_sqrt_scale(1/sqrt(DEFAULT_MAP_SCALE_PARAM)),
                trained(false) {}

        gp(float s) :   scale(s),
                        inv_scale(1 / s),
                        inv_sqrt_scale(1/sqrt(s)),
                        trained(false) {}

        void reset();

        bool isTrained() { return trained; }

        void setGPScaleParam(float l) { scale = l; }

        //Training
        void train_new(const vecNode3 &samples);
        void train_new(const Eigen::Matrix<double,3,Eigen::Dynamic> &samples);
        EMatrixX matern32_sparse_deriv1_3D(EMatrixX const& x1, float scale_param, float sigx);
        //Testing
        void testSinglePoint_new(const EVectorX &xt, double &val, double grad[], double var[]);
        EMatrixX matern32_sparse_deriv1_3D(EMatrixX const& x1, EMatrixX const& x2, float scale_param);
    };

}

#endif
