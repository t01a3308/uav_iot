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
#include "misc.h"
using namespace ns3;
using namespace std;
class SITE: public Object 
{
	/*This class defines positions that measurements are available 
	(provided by sensors or UAVs traversing). UAVs may spray 
	chemicals theredown */
private:
	//short cellID;
	Vector position;
    double visited_time; //second	
	double resource;
	double urgency;
	double w;
	double utility;
public:
	SITE(Vector p, double data_value);
	~SITE()
	{

	};
	Vector GetSitePosition();
	double GetVisitedTime();
	double GetResource();
	double GetUrgency();
	double GetUtility();
	//double sprayVol(double c_h, double c_l);
	//double dustVol(void);
};
SITE::SITE(Vector p, double data_value)
{
	Ptr<UniformRandomVariable> rd = CreateObject<UniformRandomVariable>();
	position = p;
	visited_time = VISITED_TIME_FACTOR * data_value;
	resource = RESOURCE_FACTOR * data_value;
	urgency = rd -> GetValue(MIN_URGENCY, MAX_URGENCY);
	w = rd -> GetValue(1.0, 2.0);
	utility = K_FACTOR * w * resource * urgency;
}
Vector SITE::GetSitePosition()
{
	return position;
}
double SITE::GetVisitedTime()
{
	return visited_time;
}
double SITE::GetResource()
{
	return resource;
}
double SITE::GetUrgency()
{
	return urgency;
}
double SITE::GetUtility()
{
	return utility;
}
//
class SiteList
{
private:
	vector< Ptr<SITE> > m_list;
public:
	SiteList()
	{

	};
	~SiteList()
	{

	};
	void Add(Ptr<SITE> site);
	Ptr<SITE> Get(uint32_t i);
	uint32_t GetSize();
	double GetUtility();
};
void SiteList::Add(Ptr<SITE> site)
{
	m_list.push_back(site);
}
Ptr<SITE> SiteList::Get(uint32_t i)
{
	return m_list[i];
}
uint32_t SiteList::GetSize()
{
	return m_list.size();
}
double SiteList::GetUtility()
{
	double u = 0;
	for(uint32_t i = 0; i < this->GetSize(); i++)
	{
		u += m_list[i]->GetUtility();
	}
	return u;
}
class UAV: public Node
{
private:
	double consumed_energy = 0;
	double sensed_stat = 0;
	//double max_energy = 999999;
	double flied_distance = 0;
	double data_load = 0;
	double power = 0; //
	double t_old = 0; //power and t_old are used to calculate energy
public:
	UAV()
	{
	};
	~UAV()
	{
	};
	void UpdateEnergy(double new_power);
	void UpdateFliedDistance(double distance);
	double GetConsumedEnergy();
	double GetFliedDistance();
	//void handleSite(Site &site, double to_conc);
};
void UAV::UpdateEnergy(double new_power)
{
	double now = GetNow();
	consumed_energy += power*(now - t_old);
	t_old = now;
	power = new_power;
}
void UAV::UpdateFliedDistance(double distance)
{
	flied_distance += distance;
}
double UAV::GetConsumedEnergy()
{
	return consumed_energy;
}
double UAV::GetFliedDistance()
{
	return flied_distance;
}
//
class SENSOR: public Node
{
private:
	double curr_energy;
	double sensed_stat = 0;
	double max_energy = 999999;
	double flied_distance = 0;
	double data_load;
public:
	SENSOR()
	{
	};
	~SENSOR()
	{
	};
	double distanceTo(Vector p);
	double energyTo(Vector p);
	//void handleSite(Site &site, double to_conc);
};
class GW: public Node
{
private:
	short uav_id, home_cell, visit_cell;
	double x, y;
	double max_vol, ground_speed, curr_vol, curr_energy;
	double sensed_stat = 0;
	double max_energy = 999999;
	double flied_distance = 0;
	double data_load;
public:
	GW()
	{
	};
	~GW()
	{
	};
};
//
class UAVContainer: public NodeContainer
{
public:
	UAVContainer()
	{

	};
	~UAVContainer()
	{
	};
	Ptr<UAV> GetUav(uint32_t i);
	double CalculateEnergyConsumption();
	double CalculateFliedDistance();
};
Ptr<UAV> UAVContainer::GetUav(uint32_t i)
{
	return DynamicCast<UAV, Node>(this->Get(i));
}
double UAVContainer::CalculateEnergyConsumption()
{
	double energy = 0;
	for(int i = 0; i < NUM_UAV; i++)
	{
		energy += this->GetUav(i)->GetConsumedEnergy();
	}
	return energy;
}
double UAVContainer::CalculateFliedDistance()
{
	double distance = 0;
	for(int i = 0; i < NUM_UAV; i++)
	{
		distance += this->GetUav(i)->GetFliedDistance();
	}
	return distance;
}
//
class SensorContainer: public NodeContainer
{
public:
	SensorContainer()
	{

	};
	~SensorContainer()
	{
	};
	Ptr<SENSOR> GetSensor(uint32_t i);
};
Ptr<SENSOR> SensorContainer::GetSensor(uint32_t i)
{
	return DynamicCast<SENSOR, Node>(this->Get(i));
}
//
class GwContainer: public NodeContainer
{
public:
	GwContainer()
	{

	};
	~GwContainer()
	{
	};
};