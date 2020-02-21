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
#include <utility>
#include "macro_param.h"

using namespace ns3;
using namespace std;
double GetNow()
{
  return Simulator::Now().GetSeconds();
}
pair<int, int> FindLargestElement(double a[][MAX_SITE_PER_CELL], int m, int n)
{
	int row = 0, col = 0;
	double max = a[0][0];
	for(int i = 0; i < m; i++)
	{
		for(int j = 0; j < n; j++)
		{
			if(a[i][j] > max)
			{
				max = a[i][j];
				row = i;
				col = j;
			}
		}
	}
	return make_pair(row, col);
}
template <class T>
void RemoveValueInVector(vector<T> &v, T value)
{
	typename vector <T>:: iterator it = v.begin();
	while(it != v.end())
	{
		if(*it == value)
		{
			v.erase(it);
			break;
		}
		it++;
	}
}
template<class D>
D GetValue(std::list<D>& l,int n)
{
  typename std::list<D>::iterator i = l.begin();
  std::advance(i, n);
  return *i; 
}