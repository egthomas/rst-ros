Radar Control Program Name:
==========================
eclipsesound

Control Program ID (CPID):
=========================
1103

Parameters:
==========
nbeams: 10+
intt: 2.5 s
scan: 1 min
ngates: 75+
frang: 180 km
rsep: 45 km

Description:
===========
eclipsesound is a variant on the normalsound radar control
program that samples every other beam during the first 30 sec
of each 1-min scan. In the remaining time until the end of the
minute it performs scans through a set of up to 9 frequencies
on 2 selected beams. This control program was originally
designed for the Oct 2023 annular and Apr 2024 total eclipses.

The control program requires a radar-specific sounding file called
"sounder_[rad].dat", where "[rad]" should be replaced by
the three-letter radar station string. By default, the control
program will look for this file in the SD_SITE_PATH directory.
This file should contain the following values (one per line):

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
E.G. Thomas (20240408)
