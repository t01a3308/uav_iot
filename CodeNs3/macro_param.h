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
#define NUM_SCENARIO 100
#define NUM_UAV 5 //per cell
#define NUM_SENSOR 150 // per cell
#define NUM_GW 1 //per cell
#define NUM_CELL 7
#define MAX_NUM_CELL 37
#define X0 3000.0 //  m // position of cell centre 0
#define Y0 3000.0 //  m
#define CELL_RADIUS 1000.0 // m // cell radius
#define height 20.0
//
#define TOTAL_SITE 500
#define MAX_SITE_PER_CELL 150
#define VUAV 5.0 // m/s
#define MAX_RESOURCE_PER_UAV 650
//sensor data
#define MIN_VALUE 50.0
#define MAX_VALUE 100.0
//
#define K_FACTOR 1.0
#define Rd 12000.0
#define UTILITY_FACTOR 2.0
#define VISITED_TIME_UTILITY_FACTOR 10.0
#define RESOURCE_FACTOR 1.0
#define VISITED_TIME_FACTOR 0.3
#define MIN_URGENCY 100
#define MAX_URGENCY 200
//power 
#define STOP 0.0
#define FLYING 350.0
#define HANDLING 400.0
//
#define MIN_TIME 10.0
#define MAX_TIME 600.0 // site appear time
//
#define PI 3.14159
//TSP
#define MAX 4194304// 2^(MAX_SITE_PER_CELL+1)
// parameters for communication
// interval to send periodic packets
#define UAV_INTERVAL 1 // MINUTE
#define SENSOR_INTERVAL 2 // minute
// packet size
#define UAV_PACKET_SIZE 1024 //byte
#define SENSOR_PACKET_SIZE 256 // byte
// number of periodic packets send to gateway
#define UAV_NUM_PACKET 10 // 
#define SENSOR_NUM_PACKET 1//
//
#define NUM_PACKET_SITE 20 // number of packets send to gateway when uav finish task at a site
//
#define INTERVAL_BETWEEN_TWO_PACKETS 0.001// SECOND
//
#define INTERVAL_BETWEEN_TWO_ROUNDS 1 //minute
// energy when uav send or receive 
#define ENERGY_SEND 0.1 // J
#define ENERGY_RECEIVE 0.01 // J
