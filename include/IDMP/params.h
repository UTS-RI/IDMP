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

#ifndef PARAMS_H_
#define PARAMS_H_

// comment to use Log Kernel instead of Reverting one
#define RevertingKernel 

/*-----------------------------------------------------------------------------
 *  Tree Parameters
 *-----------------------------------------------------------------------------*/
// Note: the parameters must be (some common base)^n (n: integer)
#define DEFAULT_TREE_INIT_ROOT_HALFLENGTH (3.2)  // 0.05*2^7
#define DEFAULT_TREE_MIN_HALFLENGTH       (0.025/2.0)  // 0.05*2^0
#define DEFAULT_TREE_MAX_HALFLENGTH       (12.8) // 0.05*2^10
#define DEFAULT_TREE_CLUSTER_HALFLENGTH   (0.025)   // 0.05*2^2

#define DEFAULT_MAP_SCALE_PARAM 15*15
#define DEFAULT_RLENG (2.0)

//Depth range
#define DEPTH_MAX_RANGE   1.9
#define DEPTH_MIN_RANGE   1e-1
//use every 'skip'-th pixel
#define DEPTH_SKIP       10
#endif   /* ----- #ifndef PARAMS_H_  ----- */
