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
	
	Vector position;
    double visited_time; //second	
	int resource;
	double urgency;
	double w;
	double utility;
	int id;
public:
	SITE(Vector p, int data_value, int idx);
	SITE(Vector p, double visitedtime, int res, double uti, double urg, int idx);
	~SITE()
	{

	};
	Vector GetSitePosition();
	double GetVisitedTime();
	int GetResource();
	double GetUrgency();
	double GetUtility();
	int GetId();
	//double sprayVol(double c_h, double c_l);
	//double dustVol(void);
};
SITE::SITE(Vector p, int data_value, int idx)
{
	Ptr<UniformRandomVariable> rd = CreateObject<UniformRandomVariable>();
	position = Vector(p.x, p.y, height);
	visited_time = VISITED_TIME_FACTOR * data_value;
	resource = RESOURCE_FACTOR * data_value;
	urgency = rd -> GetValue(MIN_URGENCY, MAX_URGENCY);
	w = rd -> GetValue(1.0, 2.0);
	utility = K_FACTOR * w * resource * urgency - VISITED_TIME_UTILITY_FACTOR*visited_time;
	id = idx;
}
SITE::SITE(Vector p, double visitedtime, int res, double uti, double urg, int idx)
{
	position = Vector(p.x, p.y, height);
	visited_time = visitedtime;
	resource = res;
	utility = uti;
	urgency = urg;
	id = idx;
	//std::cout<<"site "<<idx<<", pos: "<<position<<", visitedtime = "<<visited_time<<", res = "<<resource<<", utility = "<<utility<<", urg = "<<urgency<<std::endl;
}
Vector SITE::GetSitePosition()
{
	return position;
}
double SITE::GetVisitedTime()
{
	return visited_time;
}
int SITE::GetResource()
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
int SITE::GetId()
{
	return id;
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
	double GetLength(Vector pos);
	double GetResource();
	void Pop();
	void Print();
	void Clear();
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
double SiteList::GetLength(Vector pos)
{
	int n = m_list.size();
	double distance = 0;
	distance += CalculateDistance(pos, m_list[0]->GetSitePosition());
	distance += CalculateDistance(pos, m_list[n-1]->GetSitePosition());
	for(int i = 0; i < n - 1; i++)
	{
		distance += CalculateDistance(m_list[i]->GetSitePosition(), m_list[i+1]->GetSitePosition());
	}
	return distance;
}
double SiteList::GetResource()
{
	double rs = 0;
	for(int i = 0; i < (int)m_list.size(); i++)
	{
		rs += m_list[i]->GetResource();
	}
	return rs;
}
void SiteList::Pop()
{
	m_list.erase(m_list.begin());
}
void SiteList::Print()
{
	int n = (int)m_list.size();
	for(int i = 0; i < n; i++)
	{
		std::cout<<m_list[i]->GetId()<<" ";
	}
	std::cout<<std::endl;
}
void SiteList::Clear()
{
	m_list.clear();
}
class UAV: public Node
{
private:
	int cell_id;
	int uav_id;
	double flight_time = 0;
	double consumed_energy = 0;
	double flied_distance = 0;
	double power = 0; //
	double t_old = 0; //power and t_old are used to calculate energy
	queue < Ptr<SITE> > q;
public:
	UAV()
	{
	};
	~UAV()
	{
	};
	UAV(int cellId, int uavId);
	int GetCellId();
	int GetUavId();
	void UpdateEnergy(double new_power);
	void IncreaseEnergy(double delta);
	void UpdateFliedDistance(double distance);
	double GetConsumedEnergy();
	double GetFliedDistance();
	void UpdateFlightTime(double t);
	double GetFlightTime();
	uint32_t GetSiteSize();
	void AddSite(Ptr<SITE> s);
	Ptr<SITE> GetSite();
	void RemoveSite();
};
UAV::UAV(int cellId, int uavId)
{
	this->cell_id = cellId;
	this->uav_id = uavId;
}
int UAV::GetCellId()
{
	return cell_id;
}
int UAV::GetUavId()
{
	return uav_id;
}
void UAV::UpdateEnergy(double new_power)
{
	double now = GetNow();
	consumed_energy += power*(now - t_old);
	t_old = now;
	power = new_power;
}
void UAV::IncreaseEnergy(double delta)
{
	consumed_energy += delta;
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
void UAV::UpdateFlightTime(double t)
{
	flight_time += t;
}
double UAV::GetFlightTime()
{
	return flight_time;
}
uint32_t UAV::GetSiteSize()
{
	return q.size();
}
void UAV::AddSite(Ptr<SITE> s)
{
	q.push(s);
}
Ptr<SITE> UAV::GetSite()
{
	return q.front();
}
void UAV::RemoveSite()
{
	q.pop();
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
	double CalculateFlightTime();
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
double UAVContainer::CalculateFlightTime()
{
	double t = 0;
	for(int i = 0; i < NUM_UAV; i++)
	{
		Ptr<UAV> u = this->GetUav(i);
		double ti = u->GetFlightTime();
		t += ti;
		//std::cout<<"flight_time cell "<<u->GetCellId()<<", uav "<<u->GetUavId()<<": "<<ti<<std::endl;
	}
	return t;
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