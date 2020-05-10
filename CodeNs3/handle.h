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
extern double X[MAX_NUM_CELL], Y[MAX_NUM_CELL];
class SITE: public Object 
{
	/*This class defines positions that measurements are available 
	(provided by sensors or UAVs traversing). UAVs may spray 
	chemicals theredown */
private:
	
	Vector position;
    double visited_time; //second	
	int resource;
	int delta_resource = 0;
	double urgency;
	double w;
	double utility;
	int id;
	int cellId;
	double appear_time;
	double delta_appear_time;
	double begin_time = -1;
	double end_time = -1;;
	int status = 0;
	vector < pair<Ptr<UAV>, double > > bidder;
public:
	static int cnt;
	SITE(Vector p, int data_value, int idx, int cellId, int dt_resource, double delta_appear_time);
	~SITE()
	{

	};
	Vector GetSitePosition();
	double GetVisitedTime();
	void SetVisitedTime(double newValue);
	void SetBeginTime(double t);
	void SetEndTime (double t);
	double GetBeginTime();
	double GetEndTime();
	double GetTime();
	int GetResource();
	int GetDeltaResource();
	int GetRealResource();
	double GetDeltaAppearTime();
	void SetResource(int newValue);
	void SetDeltaResource(int newValue);
	double GetUrgency();
	double GetUtility();
	void SetUtility(double newValue);
	double GetRealUtility();
	double GetExpectedUtility(double t);
	int GetId();
	int GetCellId();
	void AddBidder(Ptr<UAV> u, double distance);
	Ptr<UAV> GetBidder();
	void RemoveBidder();
	int CheckPrice(Ptr<UAV> u, double distance);
	int GetStatus();
	void SetStatus(int sta);
};
int SITE::cnt = 0;
SITE::SITE(Vector p, int data_value, int idx, int idCell, int change_resource, double dt_appear_time)
{
	Ptr<UniformRandomVariable> rd = CreateObject<UniformRandomVariable>();
	position = Vector(p.x, p.y, height);
	visited_time = VISITED_TIME_FACTOR * data_value;
	resource = RESOURCE_FACTOR * data_value;
	if(change_resource == 1)
	{
		int m = (int)resource/5;
		int dt = rand() % m;
		delta_resource += dt;
	}
	urgency = rd -> GetValue(MIN_URGENCY, MAX_URGENCY);
	w = rd -> GetValue(1.0, 2.0);
	utility = K_FACTOR * w * resource * urgency;
	id = idx;
	cellId = idCell;
	delta_appear_time = dt_appear_time;
	appear_time = GetNow() + dt_appear_time;
}
Vector SITE::GetSitePosition()
{
	return position;
}
double SITE::GetVisitedTime()
{
	return visited_time;
}
void SITE::SetVisitedTime(double newValue)
{
	visited_time = newValue;
}
double SITE::GetBeginTime()
{
	return begin_time;
}
double SITE::GetEndTime()
{
	return end_time;
}
double SITE::GetTime()
{
	return end_time - appear_time;
}
void SITE::SetBeginTime(double t)
{
	begin_time = t;
}
void SITE::SetEndTime(double t)
{
	end_time = t;
}
int SITE::GetResource()
{
	return resource;
}
int SITE::GetDeltaResource()
{
	return delta_resource;
}
double SITE::GetDeltaAppearTime()
{
	return delta_appear_time;
}
int SITE::GetRealResource()
{
	return delta_resource + resource;
}
void SITE::SetResource(int newValue)
{
	resource = newValue;
}
void SITE::SetDeltaResource(int newValue)
{
	delta_resource = newValue;
}
double SITE::GetUrgency()
{
	return urgency;
}
double SITE::GetUtility()
{
	return utility;
}
void SITE::SetUtility(double newValue)
{
	utility = newValue;
}
double SITE::GetRealUtility()
{
	if(begin_time == -1)
	{
		std::cout<<"begin_time = -1"<<std::endl;
	}
	return utility*(1 - UTILITY_FACTOR*(end_time - appear_time)/60.0/100);
}
double SITE::GetExpectedUtility(double t)
{

	return utility*(1 - UTILITY_FACTOR*(t - appear_time)/60.0/100);
}
int SITE::GetId()
{
	return id;
}
int SITE::GetCellId()
{
	return cellId;
}
void SITE::AddBidder(Ptr<UAV> u, double distance)
{
	bidder.push_back(make_pair(u, distance));
}
Ptr<UAV> SITE::GetBidder()
{
	return bidder[0].first;
}
void SITE::RemoveBidder()
{	
	bidder.pop_back();
}
int SITE::CheckPrice(Ptr<UAV> newBidder, double distance)
{
	if(distance < bidder[0].second)
	{
		return 1;
	}
	return 0;
}
int SITE::GetStatus()
{
	return status;
}
void SITE::SetStatus(int sta)
{
	status = sta;
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
		m_list.clear();
	};
	void Add(Ptr<SITE> site);
	Ptr<SITE> Get(uint32_t i);
	uint32_t GetSize();
	double GetUtility();
	double GetRealUtility();
	double GetExpectedUtility(int cellId);
	double GetReverseUtility(int cellId);
	double GetLength(Vector pos);
	int GetResource();
	void Remove(Ptr<SITE> site);
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
double SiteList::GetExpectedUtility(int cellId)
{
	double currentTime = GetNow();
	Vector currentPos = Vector(X[cellId], Y[cellId], height);
	double utility = 0;
	int size = m_list.size();
	for(int i = 0; i < size; i++)
	{
		Ptr<SITE> s = m_list[i];
		Vector nextPos = s -> GetSitePosition();
		double distance = CalculateDistance(currentPos, nextPos);
		double time = distance/VUAV;
		double visitedtime = s -> GetVisitedTime();
		double endtime = time + visitedtime + currentTime;
		double uti = s -> GetExpectedUtility(endtime);
		utility += uti;
		currentTime = endtime;
		currentPos = nextPos;
	}
	return utility;
}
double SiteList::GetReverseUtility(int cellId)
{
	double currentTime = GetNow();
	Vector currentPos = Vector(X[cellId], Y[cellId], height);
	double utility = 0;
	int size = m_list.size();
	for(int i = size-1; i >= 0; i--)
	{
		Ptr<SITE> s = m_list[i];
		Vector nextPos = s -> GetSitePosition();
		double distance = CalculateDistance(currentPos, nextPos);
		double time = distance/VUAV;
		double visitedtime = s -> GetVisitedTime();
		double endtime = time + visitedtime + currentTime;
		double uti = s -> GetExpectedUtility(endtime);
		utility += uti;
		currentTime = endtime;
		currentPos = nextPos;
	}
	return utility;
}
double SiteList::GetRealUtility()
{
	double u = 0;
	for(uint32_t i = 0; i < this->GetSize(); i++)
	{
		u += m_list[i]->GetRealUtility();
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
int SiteList::GetResource()
{
	int rs = 0;
	for(int i = 0; i < (int)m_list.size(); i++)
	{
		rs += m_list[i]->GetResource();
	}
	return rs;
}
void SiteList::Remove(Ptr<SITE> site)
{
	vector < Ptr<SITE> >:: iterator it = m_list.begin();
	while(it != m_list.end())
	{
		if(*it == site)
		{
			m_list.erase(it);
			break;
		}
		it++;
	}
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
	double current_resource = MAX_RESOURCE_PER_UAV;
	double flight_time = 0;
	double consumed_energy = 0;
	double flied_distance = 0;
	double data_load = 0;
	double sensed_stat = 0;
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
	double GetResource();
	void SetResource(double newResource);
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
double UAV::GetResource()
{
	return current_resource;
}
void UAV::SetResource(double newResource)
{
	current_resource = newResource;
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