# Overview

> small introduction to most important files

## main

- specify name of las file, gml file, (osm) shp file and texture jpg here

## Window

> manage rendering, "camera", window and shaders  
> uses data stored in `DataStructure`

- move with W, A, S, D, shift, space
- look around with mouse
- F1 toggles rendering of normals and coordinate system
- F2 toggles rendering of splats vs points
- F3 toggles backface culling
- F4 toggles rendering of texture

## DataIO

> parse files and process LiDAR data (chapter 3)  
> gets called in `DataStructure`

reads and converts all the files

- las -> Point Cloud
- shp -> Polygons
- gml -> Buildings

function `preprocessWalls`

- convert shp Polygons to Buildings

function `filterAndColorPoints`  

- filters vegetation from point cloud by removing multi return points that don't belong to no osm/shp building
- marks points as wall points for next step
- filters outliers

function `insertWalls`

- mixes gml and (osm) shp walls
- fills holes in point cloud by inserting additional points on walls from buildings
- removes old wall points afterwards

function `readCache` and `writeCache`

two types of cache:
- one after reading data and processing it in DataIO (point_cache)
- one after splatting (splat_cache)

## DataStructure

> holds data  
> uses DataIO to get cached/fresh data  
> performs AdaSplats if there is no splat_cache

holds data

- point cloud
- texture coordinates
- tangents for splats (points without splat have tangent1 = (0,0,0))

function `adaSplats`





