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

#ifndef __STRCT_H_
#define __STRCT_H_

#include <vector>
#include <memory>
#include "params.h"
#include <iostream>

namespace IDMP_ros
{
    template<typename T>
    struct Point3 {
        T x;
        T y;
        T z;

        Point3(T _x, T _y, T _z) {
            x = _x;
            y = _y;
            z = _z;
        }

        Point3() {
            x = 0;
            y = 0;
            z = 0;
        }
    };

    class Node3 {
        Point3<float> pos;
        Point3<uint8_t> color;

    public:

        Node3(Point3<float> _pos, Point3<uint8_t> _color) {
            pos = _pos;
            color = _color;
        }

        Node3() {}

        const Point3<float> &getPos() { return pos; }

        float getPosX() { return pos.x; }

        float getPosY() { return pos.y; }

        float getPosZ() { return pos.z; }

        uint8_t getColorR() { return color.x; }

        uint8_t getColorG() { return color.y; }

        uint8_t getColorB() { return color.z; }

        void setColor(Point3<uint8_t> col) {color = col;};
    };

//////////////////////////////////////////////////////////////////////////

    typedef struct tree_param_ {
        float initroot_halfleng;
        float min_halfleng;         // minimum (leaf) resolution of tree
        float min_halfleng_sqr;
        float max_halfleng;         // maximum (root) resolution of tree
        float max_halfleng_sqr;
        float cluster_halfleng;    // the resolution of GP clusters
        float cluster_halfleng_sqr;
    public:
        tree_param_() : min_halfleng(DEFAULT_TREE_MIN_HALFLENGTH),
                        min_halfleng_sqr(DEFAULT_TREE_MIN_HALFLENGTH * DEFAULT_TREE_MIN_HALFLENGTH),
                        max_halfleng(DEFAULT_TREE_MAX_HALFLENGTH),
                        max_halfleng_sqr(DEFAULT_TREE_MAX_HALFLENGTH * DEFAULT_TREE_MAX_HALFLENGTH),
                        initroot_halfleng(DEFAULT_TREE_INIT_ROOT_HALFLENGTH),
                        cluster_halfleng(DEFAULT_TREE_CLUSTER_HALFLENGTH),
                        cluster_halfleng_sqr(DEFAULT_TREE_CLUSTER_HALFLENGTH * DEFAULT_TREE_CLUSTER_HALFLENGTH) {}

        tree_param_(float mi, float ma, float ini, float c) :
                min_halfleng(mi),
                min_halfleng_sqr(mi * mi),
                max_halfleng(ma),
                max_halfleng_sqr(ma * ma),
                initroot_halfleng(ini),
                cluster_halfleng(c),
                cluster_halfleng_sqr(c * c) {}
    } tree_param;

}

#endif
