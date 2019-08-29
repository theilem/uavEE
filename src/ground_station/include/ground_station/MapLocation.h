////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavEE.
//
// uavEE is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavEE is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
#ifndef LOCATION_H
#define LOCATION_H

#include <QJsonObject>
#include <uavAP/Core/LinearAlgebra.h>
#include <uavAP/Core/PropertyMapper/Configuration.h>
class MapLocation
{
public:
	MapLocation(double east = 0, double north = 0);

	static MapLocation
	fromJson(const Configuration & json);
	static MapLocation
	fromLatLong(double Lat, double Long);
	static MapLocation
	fromMapTileCoords(double x, double y, int z);
	static MapLocation
	fromUTM(double northing, double easting);
	static MapLocation
	fromVector3(Vector3 vect, int zone = 0, char hemi = 'N');

	double
	northing() const;
	double
	easting() const;
	double
	x(int z) const;
	double
	y(int z) const;
	double
	latitude() const;
	double
	longitude() const;
	int
	getZone() const;
	char
	getHemi() const;

	void
	setZone(int);

	void
	setHemi(char);

private:
	double n;
	double e;
	int zone;
	char hemi;

	double lat;
	double lon;

	static void
	latLongToUTM(double Lat, double Long, double * northing, double * easting, int * zone,
			char * hemi);
	static void
	mapTileCoordsToLatLong(double x, double y, int z, double * Lat, double * Long);
};

#endif // LOCATION_H
