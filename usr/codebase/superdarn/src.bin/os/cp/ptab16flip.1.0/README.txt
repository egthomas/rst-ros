Radar Control Program Name:
==========================
ptab16flip

Control Program ID (CPID):
=========================
9916

Parameters:
==========
nbeams: 16+
intt: 3 s
scan: 2 min
ngates: 300
frang: 180 km
rsep: 15 km

Description:
===========
ptab16flip is a variant on the normalscan radar control program
that performs 2-min scans in a non-standard manner. During the
first minute of each scan, all beams are sampled in a sequential
manner using the standard 8-pulse katscan sequence. During the
second minute, all beams are sampled again using an extended
16-pulse asymmetric sequence (originally designed by Mrinal Balaji,
UAF MS student).

Source:
======
E.G. Thomas (20230918)
