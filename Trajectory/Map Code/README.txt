parts of this code that are (somewhat) user-friendly:

findClosestRoad -> given a trajectory (an mX2 matrix), finds the closest trajectory on the map.  It returns the closest map-following trajectory as an mX2 matrix, the gradients at those points, and the index of the road it belongs to.  This function should work just as well for a single point (a length-1 trajectory).

roads_all.mat -> contains:
	roads = an nX1 structure containing all roads.  One field is the road's name, the other is a matrix describing the road's geometry.  Hopefully the details of this matrix won't be necessary for using it.
	transitions = an nXn matrix specifying which roads lead to which others.  For instance, if you are trying to extend a prediction for road 1, and transitions(1,34) = 1, then continue your prediction method using road 34.

interpolate(road, spacing) -> takes a road matrix (i.e. the segments field from roads_all.mat structure) and returns a set of points from the road.  The space argument can be:
	a single distance in meters = interpolate will return points spanning the road, spaced by this distance.  This is useful for plotting the road.
	an increasing vector of distances = interpolate will return the points that are this far along the road.  For instance, if the input is [3, 10, 20] then the result will be the point 3 meters from the start of the road, the point 7 meters from that point, and the point 10 meters from the second point.  If some specified distances are longer than the road itself, those points will not be returned.  Instead, a third output will return the distance REMAINING for the other points.

shortenRoad(road, point,direction) -> given a point on a road, cuts the road so that it starts or ends at this point.  direction = 'before' or 'after', depending on whether you want to keep the road before or after this point.
	Combined with interpolate, this lets you select a point on a road then travel some distance along it, (like the example testMap).


examples of use:
plotMap -> a plot of all roads
testMap.m -> using the map to predict a future trajectory (with a very simple physics model).
transformRoadCoords -> viewing each trajectory as forward and left-right motion along its corresponding road, instead of in x-y coordinates.
