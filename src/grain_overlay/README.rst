Grain Overlay
=============

Blend a pre-generated film grain plate (ML/GRAIN.GRN) into photos taken
with the normal shutter and save the result as GRN_nnnn.DNG.

Requires picture quality RAW or RAW+JPEG.

Source "Auto" picks the raw buffer on DIGIC 4/5 (550D; needs Canon image
review) and the CR2 file on DIGIC 6/7 (750D): the module waits for the CR2,
decodes it and writes the DNG in the background (several seconds per photo).

The plate must match the active area: 550D 5202x3464, 750D 6000x4000.

:Author: Bjarnepw
:License: GPL
:Summary: Film grain plate overlay for still photos (DNG output)
