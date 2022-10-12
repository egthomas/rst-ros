Radar Control Program Name:
==========================
interleavesound

Control Program ID (CPID):
=========================
197

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
interleavesound is a variant on the interleaved_normalscan and
normalsound radar control programs. interleavesound performs
a scan in a nonsequential manner by "interleaving" the beam
number, e.g. (0-4-8-12)-(2-6-10-14)-(1-5-9-13)-(3-7-11-15).
In the remaining time until the end of the minute it performs
scans through a set of up to 12 frequencies and through all
beams [even/odd]. Note that unlike normalsound, this information
is not used to adjust the radar operating frequency in real-time.

The control program requires a radar-specific sounding file called
"interleave_sounder.dat". By default, the control program will look
for this file in the SD_SND_PATH directory. This file should
contain the following values (one per line):

Number of sounder frequencies (maximum of 12)
The sounder frequencies [kHz]

If this file does not exist, default values are used. This
is not a good idea, as the program may try to sound at
forbidden frequencies.

The sounding data are written to *.snd files in the SD_SND_PATH
directory. If this environment variable is not set, the control
program will attempt to write the sounding data to the "/data/ros/snd"
directory. If this directory does not exist, no sounding data will
be written. The 2-hr sounding files contain a reduced set of
radar operating parameters and fitted values (e.g., velocity,
power, spectral width, phi0, elevation) in dmap-format.

Source:
======
E.G. Thomas (20200625)
