parts of this code that are (somewhat) user-friendly:

roads_all.mat -> contains:
	roads = an nX1 structure containing all roads.  One field is the road's name, the other is a matrix describing the road's geometry.  Hopefully the details of this matrix won't be necessary for using it.
	transitions = an nXn matrix specifying which roads lead to which others.  For instance, if you are trying to extend a prediction for road 1, and transitions(1,34) = 1, then continue your prediction method using road 34.
	adjacencies = an nX1 array assigning a number to each road.  Roads with the same number are parallel lanes.

processSegment_XXX.m -> takes a segment of data gathered in the DAQ stage and transforms the coordinates in some way.  Gives a 'data' file and a 'truth' file, representing information from before and after the designated current time.
	_raw -> returns the basic x/y coordinates for the car, and nothing else.
	_roadCoord -> transforms the x/y coordinates for the car into road-oriented coordinates. Also includes the road name at each timepoint (gathered from NGSIM labels).
	_roadFind -> same as roadCoord, but finds the roads using a recursive method rather than labels.  Currently can't handle lane changes well.


other useable functions:
plotMap -> plots all roads, useful for finding problems
modifyRoads -> lengthens some of the roads after the fact, so that processSegment_roadCoord works better...

helper functions:
constructSegment, closestPoint, startPoint, endPoint -> basic methods for using the road data

interpolate(road, spacing) -> takes a road matrix (i.e. the segments field from roads_all.mat structure) and returns a set of points from the road.  The space argument can be:
	a single distance in meters = interpolate will return points spanning the road, spaced by this distance.  This is useful for plotting the road.
	an increasing vector of distances = interpolate will return the points that are this far along the road.  For instance, if the input is [3, 10, 20] then the result will be the point 3 meters from the start of the road, the point 7 meters from that point, and the point 10 meters from the second point.  If some specified distances are longer than the road itself, those points will not be returned.  Instead, a third output will return the distance REMAINING for the other points.

shortenRoad(road, point,direction) -> given a point on a road, cuts the road so that it starts or ends at this point.  direction = 'before' or 'after', depending on whether you want to keep the road before or after this point.
	Combined with interpolate, this lets you select a point on a road then travel some distance along it, (find the example testMap??).

labelDirection -> used for all processSegment functions, uses road labels to determine whether a vehicle makes a turn, lane change, etc. in the near future.  This can be used as truth for maneuver prediction.

findRoad, recurseRoad, calculateError -> helpers for processSegment_roadFind
findRoadCoords -> helper for processSegment_roadCoord
