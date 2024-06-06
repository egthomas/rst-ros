Radar Control Program Name:
==========================
interleavescan

Control Program ID (CPID):
=========================
190 / 191

Parameters:
==========
nbeams: 16+
intt: 3 s
scan: 1 min
ngates: 75+
frang: 180 km
rsep: 45 km

Description:
===========
The interleaved_normalscan is basically a variant of the 1-min
normal scan in which a scan goes not in a simple sequential manner
but "interleaves" the beam number. For example of a 16-beam radar, 
the beam sequence proceeds like (0-4-8-12)-(2-6-10-14)-(1-5-9-13)-(3-7-11-15)
for the forward scan, and (15-11-7-3)-(13-9-5-1)-(14-10-6-2)-(12-8-4-0) 
for the backward scan. The parameters rsep, intt, scan_period, etc. are 
set to be the same as the 1-min normal scan of each radar.

Source:
======
S. Shepherd (20160926)
