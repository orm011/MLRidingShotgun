import coords, geom, graph

from imposm.parser import OSMParser

BLACKLIST = set([
	"pedestrian",
	"footway",
	"bridleway",
	"steps",
	"path",
	"sidewalk",
	"cycleway",
	"proposed",
	"construction",
	"bus_stop",
	"crossing",
	"elevator",
	"emergency_access_point",
	"escape",
	"give_way",
])

def read(fname, bb=None):
	g = graph.Graph()
	vertex_map = {}
	def coords_callback(clist):
		for osmid, lon, lat in clist:
			p = geom.FPoint(lon, lat)
			if bb is not None and not bb.contains(p):
				continue
			p = coords.lonLatToMeters(p).to_point()
			vertex_map[osmid] = g.add_vertex(p)
	def ways_callback(ways):
		for osmid, tags, refs in ways:
			if 'highway' not in tags or tags['highway'] in BLACKLIST:
				continue
			for i in range(len(refs) - 1):
				src = refs[i]
				dst = refs[i + 1]
				if src not in vertex_map or dst not in vertex_map:
					continue
				g.add_bidirectional_edge(vertex_map[src], vertex_map[dst])
	p = OSMParser(coords_callback=coords_callback)
	p.parse(fname)
	p = OSMParser(ways_callback=ways_callback)
	p.parse(fname)
	return g
