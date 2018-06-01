import coords, geom, osm, router

import sys

in_fname = sys.argv[1]
out_fname = sys.argv[2]

points = []
with open(in_fname, 'r') as f:
	for line in f:
		line = line.strip()
		if not line:
			continue
		parts = line.split(',')
		seqno, lat, lon = int(float(parts[0])), float(parts[1]), float(parts[2])
		p = geom.FPoint(lon, lat)
		p = coords.lonLatToMeters(p).to_point()
		p.seqno = seqno
		if len(points) == 0 or points[-1].distance(p) > 5:
			points.append(p)
rect = points[0].bounds()
for p in points[1:]:
	rect = rect.extend(p)
rect = rect.add_tol(512)
g = osm.read('massachusetts-latest.osm.pbf', bb=rect)
r = router.Router(g)
positions = r.match(points)
with open(out_fname, 'w') as f:
	for i in range(len(points)):
		turn_info = r._get_turn_info(points[i], positions[i:])
		if turn_info is not None:
		f.write('{},{},{}\n'.format(points[i].seqno, turn_info.distance, turn_info.angle))

'''
bb = geom.Rectangle(geom.FPoint(-71.124881, 42.331305), geom.FPoint(-71.037418, 42.375642))
g = osm.read('massachusetts-latest.osm.pbf', ll_bb=bb)
r = router.Router(g)
points = [
	(42.362000, -71.097892),
	(42.360898, -71.096165),
	(42.360375, -71.095253),
	(42.360954, -71.093193),
	(42.362080, -71.091616),
	(42.362484, -71.090265),
	(42.363245, -71.089771),
	(42.364133, -71.089557),
	(42.365290, -71.089342),
	(42.366154, -71.088784),
	(42.366320, -71.087443),
	(42.366138, -71.085952),
	(42.366138, -71.085952),
	(42.367231, -71.084161),
	(42.367952, -71.083957),
	(42.368206, -71.085030),
	(42.368293, -71.086511),
	(42.368341, -71.087756),
	(42.367842, -71.087874),
	(42.367113, -71.088089),
	(42.366709, -71.088164),
	(42.366368, -71.087788),
	(42.366320, -71.086683),
]
points = map(lambda p: geom.FPoint(p[1], p[0]), points)
points = map(lambda p: coords.lonLatToMeters(p).to_point(), points)
positions = r.match(points)
'''
