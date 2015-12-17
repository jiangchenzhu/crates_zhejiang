#!/bin/bash

# Delete all intermediae products
rm *.tif *.xml *.png

# Merge the two depth maps
gdal_merge.py ./DSM/SU9671_DSM_1M.asc ./DSM/SU9672_DSM_1M.asc -o merged.tif

# Project merged to OSGB36
gdal_translate -a_srs EPSG:27700 merged.tif projected.tif

# Crop projected 450m x 450m raster
gdalwarp -te 496000 171500 497000 172500 projected.tif  windsorpark-depth.tif
gdalwarp -ts 512 512 windsorpark-depth.tif  windsorpark-depth-small.tif

# Crop aerial raster
gdalwarp -te 496000 171500 497000 172500 ./SAT/102876-1_RGB.tif  windsorpark-raster.tif
gdalwarp -ts 512 512  windsorpark-raster.tif  windsorpark-raster-small.tif

# Copy raster to material directory
cp  windsorpark-raster-small.tif ../../../resources/media/materials/textures

# Draw a debug hillshade
gdaldem hillshade ./windsorpark-depth.tif ./windsorpark-hillshade.tif

# Convert to the wgs84 ellipsoid
# Use gdalinfo to get center OSGB coordinates
# then http://www.nearby.org.uk/coord.cgi?p=523763.000%2C++203027.000&f=full
# fo find the WGS84 equivalent to use as your center point reference!
