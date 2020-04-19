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
using namespace std;
class UAV;
double GetNow()
{
  return Simulator::Now().GetSeconds();
}
pair<int, int> FindLargestElement(double a[][MAX_SITE_PER_CELL], int m, int n) // find max value in matrix
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
void ReadDataIntoArray(std::string filename, std::vector< std::vector<int> > &arr)// 2 dimentions array
{
	//
	std::string line;
	std::ifstream myfile(filename.c_str());
	int row = 0;
	if(myfile.is_open())
	{
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
			std::vector<int> r;
			for(int i = 0; i < size; i++)
			{
				if(line[i] == ' ')
				{
					r.push_back(atoi(temp.c_str()));
					temp.clear();
				}
				else if(i == size-1)
				{
					temp.push_back(line[i]);
					r.push_back(atoi(temp.c_str()));
				}
				else
				{
					temp.push_back(line[i]);
				}
			}
			arr.push_back(r);
		}
		// std::cout<<"num row = "<<arr.size()<<std::endl;
		// for(int i = 0; i < (int)arr.size(); i++)
		// {
		// 	std::vector<int> v = arr[i];
		// 	int s = v.size();
		// 	for(int j = 0; j < s; j++)
		// 	{
		// 		std::cout<<arr[i][j]<<" ";
		// 	}
		// 	std::cout<<std::endl;
		// }
	}
	else
	{
		std::cout<<"cannot open file "<<filename<<std::endl;
	}
}
void ReadData(std::string filename, std::vector<double> &arr)// data in single line, seperated by space character
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
template <class D>
int IdMin(D a[], int n)
{
	int id = 0;
	int min = a[0];
	for(int i = 1; i < n; i++)
	{
		if(a[i] < min)
		{
			min = a[i];
			id = i;
		}
	}
	return id;
}