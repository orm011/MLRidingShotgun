import pandas as pd
import sys

import file_info as fi
import vizutils as vu

out_fname = sys.argv[1]

trip = fi.g_trips[-19]
(df, boxes) = vu.get_merged_ios_dataset(g_current_trip=trip, g_current_class='fire hydrant', all_features=[], force_guess=True)
f = open(out_fname, 'w')
for t in df[['video_seqno', 'latitude', 'longitude']].iterrows():
	seqno, lat, lon = t[1].video_seqno, t[1].latitude, t[1].longitude
	f.write('{},{},{}\n'.format(seqno, lat, lon))
f.close()
