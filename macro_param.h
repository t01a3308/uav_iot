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
#define numUav 5 //per cell
#define numSensor 30 // per cell
#define numGw 1 //per cell
#define numCell 1
#define maxNumCell 37
#define x0 500.0 //  position of cell centre 0
#define y0 500.0 //
#define r 500.0 // cell radius
//data value
#define minValue 50.0
#define maxValue 200.0
#define threshold 100.0 // determine high data
//
#define energyPerUav 1000000.0 // J
#define maxTaskPerUav 5
#define vuav 5.0 // m/s

#define K_FACTOR 1
#define ALPHA_FACTOR 1