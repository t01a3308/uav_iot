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
#include "macro_param.h"

using namespace ns3;
class UAV;
double GetNow()
{
  return Simulator::Now().GetSeconds();
}
int FindId(double a[], int n, int m) // find id of m-th smallest element of array a which has size n
{
	for(int i = 0; i < n; i++)
	{
		int rank = 0;
		for(int j = 0; j < n; j++)
		{
			if(a[j] < a[i])
			{
				rank++;
			}
		}
		if(rank == m)
		{
			return i;
		}
	}
	return 9999;
}
void ReadData(std::string filename, std::vector<double> &arr)
{
 // std::cout<<filename<<std::endl;
  std::string line;
  std::ifstream myfile(filename.c_str());
  if(myfile.is_open())
  {
    int row = 0;
    while(getline(myfile, line))
    {
      int size = line.size();
      if(line.size() == 0)
      {
        continue;
      }
      //std::cout<<"line "<<row<<" size = "<<line.size()<<std::endl;
      row++;
      std::string temp;
      for(int i = 0; i < size; i++)
      {
        if(line[i] == ' ')
        {
          arr.push_back(atof(temp.c_str()));
          temp.clear();
        }
        else if(i == size-1)
        {
          temp.push_back(line[i]);
          arr.push_back(atof(temp.c_str()));
          temp.clear();
        }
        else
        {
          temp.push_back(line[i]);
        }
      }
    }
    // std::cout<<"size = "<<arr.size()<<std::endl;
    // for(int i = 0; i < (int)arr.size(); i++)
    // {
    //     std::cout<<arr[i]<<" ";
    // }
    // std::cout<<std::endl;
  }
  else
  {
    std::cout<<"cannot open file "<<filename<<std::endl;
  }
}