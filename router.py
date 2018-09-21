import coords, geom

import math
import numpy
from PIL import Image

TOLERANCE = 100
SIGMA = 30
DENSITY = 10

def densify(points):
	npoints = [points[0]]
	orig_indices = [0]
	for point in points[1:]:
		segment = geom.Segment(npoints[-1], point)
		n = int(math.floor(segment.length() / DENSITY))
		for i in range(n):
			npoints.append(segment.point_at_factor((i + 1) * DENSITY))
		orig_indices.append(len(npoints))
		npoints.append(point)
	return npoints, orig_indices

def haversine_distance(p1, p2):
	p1 = coords.metersToLonLat(p1)
	p2 = coords.metersToLonLat(p2)
	R = 6378.137
	dlon = p2.x * math.pi / 180 - p1.x * math.pi / 180
	dlat = p2.y * math.pi / 180 - p1.y * math.pi / 180
	a = math.sin(dlat / 2) * math.sin(dlat / 2) + math.cos(p1.y * math.pi / 180) * math.cos(p2.y * math.pi / 180) * math.sin(dlon / 2) * math.sin(dlon / 2)
	c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
	d = R * c
	return d *1000

class Router(object):
	def __init__(self, g):
		self.g = g

		# precompute transition probabilities
		self.transition_probs = []
		for edge in self.g.edges:
			probs = {}
			probs[edge.id] = 0.5
			for other in edge.dst.out_edges:
				probs[other.id] = 0.2
			self.transition_probs.append(probs)

		self.edge_index = self.g.edgeIndex()

	def emission_probs(self, point):
		candidates = self.edge_index.search(point.bounds().add_tol(TOLERANCE))
		if len(candidates) == 0:
			return None
		scores = {}
		total_score = 0
		for edge in candidates:
			distance = edge.segment().distance(point)
			score = math.exp(-0.5 * distance * distance / SIGMA / SIGMA) / 75
			scores[edge.id] = score
			total_score += score
		#for edge_id in scores:
		#	scores[edge_id] /= total_score
		return scores

	def match(self, points):
		points, orig_indices = densify(points)

		# initial probs are all the same
		probs = {}
		for edge in self.edge_index.search(points[0].bounds().add_tol(TOLERANCE)):
			probs[edge.id] = 0

		# run viterbi
		backpointers = [{edge_id: edge_id for edge_id in probs}]
		for pointIdx, p in enumerate(points[1:]):
			if pointIdx % 100 == 99:
				print '{}/{}'.format(pointIdx, len(points))
			next_probs = {}
			next_backpointers = {}
			# find the most likely to match the emission+transition
			emissions = self.emission_probs(p)
			for prev_edge_id in probs:
				transitions = self.transition_probs[prev_edge_id]
				for next_edge_id in transitions:
					if next_edge_id not in emissions:
						continue
					prob = probs[prev_edge_id] + math.log(transitions[next_edge_id]) + math.log(emissions[next_edge_id])
					if next_edge_id not in next_probs or prob > next_probs[next_edge_id]:
						next_probs[next_edge_id] = prob
						next_backpointers[next_edge_id] = prev_edge_id
			backpointers.append(next_backpointers)
			if not next_probs:
				print 'stuck at point {}'.format(pointIdx + 1)
				return None
			probs = next_probs

		# collect state sequence
		best_edge_id = None
		for edge_id in probs:
			if best_edge_id is None or probs[edge_id] > probs[best_edge_id]:
				best_edge_id = edge_id
		cur_edge_id = best_edge_id
		positions = []
		for i in range(len(points), 0, -1):
			edge = self.g.edges[cur_edge_id]
			point = points[i - 1]
			pos = edge.closest_pos(point)
			positions.append(pos)
			cur_edge_id = backpointers[i - 1][cur_edge_id]
		positions.reverse()

		# grab positions corresponding to original non-densified points
		orig_positions = []
		for i in range(len(orig_indices)):
			idx = orig_indices[i]
			pos = positions[idx]

			# label this position with positions that we will skip
			# this is useful if you want to know the intermediate edge transitions
			if i == len(orig_indices) - 1:
				next_idx = len(positions)
			else:
				next_idx = orig_indices[i + 1]
			pos.intermediates = positions[idx+1:next_idx]

			orig_positions.append(pos)
		return orig_positions

	def _get_forward_edge(self, edge):
		# pick outgoing edge from edge.dst with most similar angle to edge
		best_edge = None
		best_angle = None
		for other in edge.dst.out_edges:
			angle = edge.segment().vector().signed_angle(other.segment().vector())
			angle = abs(angle)
			if best_edge is None or angle < best_angle:
				best_edge = other
				best_angle = angle
		return best_edge

	def _get_turn_info(self, start_point, positions):
		# search for the next turn
		# positions[0] should correspond to start_point
		npositions = []
		for pos in positions:
			npositions.append(pos)
			npositions.extend(pos.intermediates)
		distance = 0
		for i in range(len(npositions) - 1):
			start = npositions[i].edge
			end = npositions[i + 1].edge
			if start == end:
				continue

			if start == npositions[0].edge:
				#distance += start_point.distance(start.dst.point)
				distance += haversine_distance(start_point, start.dst.point)
			else:
				#distance += start.segment().length()
				distance += haversine_distance(start.src.point, start.dst.point)

			if len(start.dst.out_edges) == 2:
				continue
			elif end == self._get_forward_edge(start):
				continue

			# found a turn! left or right?
			angle = start.segment().vector().signed_angle(end.segment().vector())
			return TurnInfo(distance, angle, end)
		return None

	def visualize(self, points, positions, fname=None):
		rect = points[0].bounds()
		for p in points:
			rect = rect.extend(p)
		for pos in positions:
			rect = rect.extend(pos.point())
		rect = rect.add_tol(16)
		l = rect.lengths()
		origin = rect.start
		im = numpy.zeros((l.x, l.y, 3), dtype='uint8')
		# draw graph edges
		for edge in self.edge_index.search(rect):
			start = edge.src.point.sub(origin)
			end = edge.dst.point.sub(origin)
			for p in geom.draw_line(start, end, l):
				im[p.x, p.y, :] = [0, 0, 255]
		# draw matched trajectory
		for i in range(len(positions) - 1):
			start = positions[i].point().sub(origin)
			im[start.x-2:start.x+2, start.y-2:start.y+2, :] = [255, 0, 0]
			end = positions[i + 1].point().sub(origin)
			for p in geom.draw_line(start, end, l):
				im[p.x, p.y, :] = [255, 0, 0]
		# draw actual trajectory
		for i in range(len(points) - 1):
			start = points[i].sub(origin)
			im[start.x-2:start.x+2, start.y-2:start.y+2, :] = [0, 255, 0]
			end = points[i + 1].sub(origin)
			for p in geom.draw_line(start, end, l):
				im[p.x, p.y, :] = [0, 255, 0]
		if fname:
			Image.fromarray(numpy.flip(im.swapaxes(0, 1), 0)).save(fname)
		return im

class TurnInfo(object):
	def __init__(self, distance, angle, edge):
		self.distance = distance
		self.angle = angle
		self.edge = edge

	def __repr__(self):
		if self.angle < 0:
			lr = 'right'
		else:
			lr = 'left'
		return 'Turn {} in {}'.format(lr, int(self.distance))
