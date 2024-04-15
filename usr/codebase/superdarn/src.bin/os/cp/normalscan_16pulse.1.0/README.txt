Radar Control Program Name:
==========================
normalscan_16pulse

Control Program ID (CPID):
=========================
9100 / 9101

Parameters:
==========
nbeams: 16+
intt: 6 s / 3 s
scan: 2 min / 1 min
ngates: 300
frang: 180 km
rsep: 15 km

Description:
===========
normalscan_16pulse is a variant on the normalscan radar control
program that performs a 1- or 2-min scan in a sequential manner.
Rather than the standard 8-pulse katscan sequence, an extended
16-pulse asymmetric sequence (originally designed by Mrinal Balaji,
UAF MS student) is used. This means the multipulse increment (mpinc),
sample separation (smsep), and transmitter pulse length (txpl) are
all shortened to 100 us, resulting in 15 km range separation and a
duration of ~150 ms for each sequence.

Source:
======
E.G. Thomas (20220901)
