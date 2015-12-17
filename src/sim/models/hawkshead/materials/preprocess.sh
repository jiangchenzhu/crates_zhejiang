#!/bin/bash

# Delete all intermediae products
rm *.tif *.xml *.png

# Merge the two depth maps
gdal_merge.py DSM/TL2302_DSM_1M.asc DSM/TL2303_DSM_1M.asc -o merged.tif

# Project merged to OSGB36
gdal_translate -a_srs EPSG:27700 merged.tif projected.tif

# Crop projected 450m x 450m raster
gdalwarp -te 523538 202802 523988 203252 projected.tif rvc-depth.tif
gdalwarp -ts 512 512 rvc-depth.tif rvc-depth-small.tif

# Crop aerial raster
gdalwarp -te 523538 202802 523988 203252 SAT/rvc-raster.tif rvc-raster.tif
gdalwarp -ts 512 512 rvc-raster.tif rvc-raster-small.tif

# Copy raster to material directory
cp rvc-raster-small.tif ../../../resources/media/materials/textures

# Draw a debug hillshade
gdaldem hillshade ./rvc-depth.tif ./rvc-hillshade.tif

# Convert to the wgs84 ellipsoid
# Use gdalinfo to get center OSGB coordinates
# then http://www.nearby.org.uk/coord.cgi?p=523763.000%2C++203027.000&f=full
# fo find the WGS84 equivalent to use as your center point reference!
