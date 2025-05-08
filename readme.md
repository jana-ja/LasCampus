# Las Campus

Airborne Lidar scanners can be used to obtain large scale point clouds of the earths surface that are a great foundation for creating 3D models of the real world.
For my [Master Thesis](https://github.com/user-attachments/files/20094725/thesis.pdf) I used free available data from [Bezirksregierung Köln](https://www.bezreg-koeln.nrw.de/) to create a textured 3D Model of my university campus using a point-based rendering algorithm with this software.


<p align="center">
  <img src="https://github.com/user-attachments/assets/f03ce6ea-c901-4328-835d-78994ee0ec52" alt="Final Screenshot 2" width="25%"/>
  <img src="https://github.com/user-attachments/assets/3df6c6e8-dfea-46e5-b779-629ed32df645" alt="Final Screenshot 3" width="25%"/>
</p>




## Preprocessing

The Lidar point clouds contain objects that are either not relevant for most 3D Models and are hard to process and they typically have less data density on vertical surfaces, so they need to be preprocessed to get a better result. 

#### Remove vegetation and Outliers from PointCloud
<p align="center">
  <img src="https://github.com/user-attachments/assets/c629e766-c422-47e4-8150-308ed016f621" alt="clean 1" width="25%"/>
  <img src="https://github.com/user-attachments/assets/4fd960a6-f5fa-4f1f-b547-2449788b3797" alt="clean 2" width="25%"/>
</p>

#### If available use shp data to add points for walls
<p align="center">
  <img src="https://github.com/user-attachments/assets/f0c5cf61-75f9-4509-b468-e6d1daded6b7" alt="clean 1" width="25%"/>
</p>

## Modeling

I decided to try a surface splatting approach for this project.

#### Use splatting to create surface
<p align="center">
  <img src="https://github.com/user-attachments/assets/f26ecb10-ccbe-4900-8eb7-0a5b126f840b" alt="clean 1" width="25%"/>
  <img src="https://github.com/user-attachments/assets/b97d68cf-212a-41ec-90d2-2d94d5edd496" alt="clean 1" width="25%"/>
</p>

#### Add texture
<p align="center">
  <img src="https://github.com/user-attachments/assets/fc61e12c-6f45-43cd-a643-6c35f97b3213" alt="Final Screenshot 1" width="25%"/>
</p>

## Credit
Free <a href=https://www.bezreg-koeln.nrw.de/geobasis-nrw/produkte-und-dienste/hoehenmodelle/3d-messdaten>las</a>, <a href=https://www.bezreg-koeln.nrw.de/geobasis-nrw/produkte-und-dienste/3d-gebaeudemodelle>shp</a> and <a href=https://www.bezreg-koeln.nrw.de/geobasis-nrw/produkte-und-dienste/luftbild-und-satellitenbildinformationen/aktuelle-luftbild-und-0>texture</a> data from Bezirksregierung Köln NRW

Additional shp data from <a href=https://download.geofabrik.de/europe/germany/nordrhein-westfalen/arnsberg-regbez.html>OpenStreetMap</a>

Some of the Splatting Algorithms <a href=https://www.graphics.rwth-aachen.de/media/papers/phong_splatting1.pdf>PhongSplatting</a> and <a href=https://arxiv.org/abs/2203.09155>AdaSplats</a>



## Project Overview

> small introduction to most important files

#### main

- specify name of las file, gml file, (osm) shp file and texture jpg here

#### Window

> manage rendering, "camera", window and shaders  
> uses data stored in `DataStructure`

- move with W, A, S, D, shift, space
- look around with mouse
- F1 toggles rendering of normals and coordinate system
- F2 toggles rendering of splats vs points
- F3 toggles backface culling
- F4 toggles rendering of texture

#### DataIO

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

#### DataStructure

> holds data  
> uses DataIO to get cached/fresh data  
> performs AdaSplats if there is no splat_cache

holds data

- point cloud
- texture coordinates
- tangents for splats (points without splat have tangent1 = (0,0,0))

function `adaSplats`





