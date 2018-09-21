import coords, geom, graph
import sys
from collections import deque

import pandas as pd
from imposm.parser import OSMParser
import numpy as np

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

def read(fname, bb=None, ll_bb=None):

	vertex_set = set([])
	vertices = deque([]) # (osm_id, lon, lat)

	used_vertices = set([])
	out_ways = deque([])
	edges = deque([]) # (osm_id, index, src, dst)

	def coords_callback(clist):
		for osmid, lon, lat in clist:
			p = geom.FPoint(lon, lat)
			if ll_bb is not None and not ll_bb.contains(p):
				continue
			p = coords.lonLatToMeters(p).to_point()
			if bb is not None and not bb.contains(p):
				continue

			vertices.append((osmid, np.double(lon), np.double(lat)))
			vertex_set.add(osmid)

	def ways_callback(ways):
		for osmid, tags, refs in ways:
			if 'highway' not in tags or tags['highway'] in BLACKLIST:
				continue
			for i in range(len(refs) - 1):
				src = refs[i]
				dst = refs[i + 1]
				if src not in vertex_set or dst not in vertex_set:
					continue

				edges.append((osmid, i, src, dst))
				used_vertices.add(src)
				used_vertices.add(dst)
				out_ways.append((osmid, refs))

	p = OSMParser(coords_callback=coords_callback)
	p.parse(fname)
	p = OSMParser(ways_callback=ways_callback)
	p.parse(fname)

	final_vertices = deque([])
	for v in vertices:
		(osm_id, _, _) = v
		if osm_id in used_vertices:
			final_vertices.append(v)

	return final_vertices, edges, out_ways


if __name__ == '__main__':
		import json
		(v, e, w) = read(sys.argv[1], bb=None, ll_bb=None)
		# vtx = pd.DataFrame(data=list(v), columns=['osm_id', 'lon', 'lat'])
		# vtx.to_csv('osm_vertices_ma2.csv', index=False)
		# edg = pd.DataFrame(data=list(e), columns=['osm_id', 'index', 'src', 'dst'])
		# edg.to_csv('osm_edges_ma2.csv', index=False)
		ways = pd.DataFrame(data=list(w), columns=['osm_id', 'refs'])
		ways['refs'] = ways.refs.map(json.dumps)
		ways.to_csv('osm_ways_ma2.csv', index=False)



