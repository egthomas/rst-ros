Radar Control Program Name:
==========================
campsound

Control Program ID (CPID):
=========================
1105 / 1106

Parameters:
==========
nbeams: 4 / 12
intt: 1.6 s
scan: 1 min
ngates: 75+
frang: 180 km
rsep: 45 km

Description:
===========
campsound is a variant on the normalsound radar control program
that steps through a set of 9 frequencies along 4 (or 12) selected
beams during each 1-min scan. This control program was originally
designed for a downrange HF receiver experiment with the Iceland
radars.

The fitted data are also written to *.snd files in the SD_SND_PATH
directory. If this environment variable is not set, the control
program will attempt to write the sounding data to the "/data/ros/snd"
directory. If this directory does not exist, no sounding data will
be written. The 2-hr sounding files contain a reduced set of
radar operating parameters and fitted values (e.g., velocity,
power, spectral width, phi0, elevation) in dmap-format.

Source:
======
E.G. Thomas (20241001)
