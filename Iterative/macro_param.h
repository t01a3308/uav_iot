 /* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 IITP RAS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
 * Authors: Tien Pham Van <tien.phamvan1@hust.edu.vn>
 *          Nguyen Pham Van <nguyen.pv152737@sis.hust.edu.vn>
 */
/*Global parameters*/
#define NUM_UAV 3 //per cell
#define NUM_SENSOR 30 // per cell
#define NUM_GW 1 //per cell
#define NUM_CELL 7
#define MAX_NUM_CELL 37
#define X0 500.0 //  position of cell centre 0
#define Y0 500.0 //
#define CELL_RADIUS 500.0 // cell radius
//
#define MAX_SITE_PER_CELL 30
#define VUAV 5.0 // m/s
#define MAX_RESOURCE_PER_UAV 250.0
//sensor data
#define MIN_VALUE 0.0
#define THRESHOLD 50.0
//
#define K_FACTOR 1
#define Rd 2.0
//#define ALPHA_FACTOR 1
#define RESOURCE_FACTOR 1.0
#define VISITED_TIME_FACTOR 2.0
#define MIN_URGENCY 100
#define MAX_URGENCY 200
//power factor
#define STOP 0.0
//#define HOVERING 100.0
#define FLYING 150.0
#define HANDLING 250.0
