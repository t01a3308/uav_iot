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
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/aodv-module.h"
#include "ns3/netanim-module.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/yans-wifi-helper.h"
#include <iostream>
#include <math.h>
#include <map>
#include "macro_param.h"

using namespace ns3;
using namespace std;
class Site: public Object 
{
	/*This class defines positions that measurements are available 
	(provided by sensors or UAVs traversing). UAVs may spray 
	chemicals theredown */
public:
	short cellID;
    short visited_times;
	double x, y;
	double peak_conc, curr_conc;
	double variance_x, variance_y;
    double over_thr_radius;
	Site()
	{
		x=0; y=0;
		peak_conc=0;
		curr_conc=0;
		variance_x=0;
		variance_y=0;
		over_thr_radius = 0; //= sqrt(x*x+y*y);
	};
	~Site();
	double sprayVol(double c_h, double c_l);
	double dustVol(void);
};
class UAV: public Node
{
public:
	short uav_id, home_cell, visit_cell;
	double x, y;
	double max_vol, ground_speed, curr_vol, curr_energy;
	double sensed_stat = 0;
	double max_energy = 999999;
	double flied_distance = 0;
	double data_load;
	UAV()
	{
	};
	~UAV()
	{
	};
	double distanceTo(Vector p);
	double energyTo(Vector p);
	void handleSite(Site &site, double to_conc);
};
class SENSOR: public Node
{
public:
	short uav_id, home_cell, visit_cell;
	double x, y;
	double max_vol, ground_speed, curr_vol, curr_energy;
	double sensed_stat = 0;
	double max_energy = 999999;
	double flied_distance = 0;
	double data_load;
	SENSOR()
	{
	};
	~SENSOR()
	{
	};
	double distanceTo(Vector p);
	double energyTo(Vector p);
	void handleSite(Site &site, double to_conc);
};
class GW: public Node
{
public:
	short uav_id, home_cell, visit_cell;
	double x, y;
	double max_vol, ground_speed, curr_vol, curr_energy;
	double sensed_stat = 0;
	double max_energy = 999999;
	double flied_distance = 0;
	double data_load;
	GW()
	{
	};
	~GW()
	{
	};
	double distanceTo(Vector p);
	double energyTo(Vector p);
	void handleSite(Site &site, double to_conc);
};
map<short, Site*>all_site_list;
map<short, Site*>cell_site_list[numCell];




