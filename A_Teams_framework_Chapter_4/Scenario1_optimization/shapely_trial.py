from shapely.geometry import LineString
from shapely.geometry import Point

p = Point(5,5)
c = p.buffer(3).boundary
l = LineString([(0,0), (10, 10)])
i = c.intersection(l)

print(i.geoms[0].coords[0])


print(i.geoms[1].coords[0])
