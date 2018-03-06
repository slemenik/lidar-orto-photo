# Merging Ortho-photo and LiDAR data into point cloud with color and normal information

### How it works

The program fuses the information from datasets of Slovenia ortho-photo (http://gis.arso.gov.si/arcgis/rest/services/DOF_2016/MapServer/export) and Lidar (http://gis.arso.gov.si/evode/profile.aspx?id=atlas_voda_Lidar@Arso) data and add color and normal information for individual point in LiDAR dataset into the original dataset (the LAS/LAZ format supports adding such information). Both datasets are available in different geodetic datum system. While points in LiDAR dataset are not evenly distributed, the orthophoto data is in image format so the information is available for individual pixel. Normal estimation is calculated based on http://imagine.enpc.fr/~marletr/publi/SGP-2012-Boulch-Marlet.pdf. LAS files don't support normal information, so the program saves the normal X,Y and Z coordinates in the X(t), Y(t) and Z(t) wavelength fields.

### How to use

By running the program it creates multiple LAZ files (each over 50 MB in size), complete with color and normal information. The program by default scans the complete Slovenia area, from the lowest left scan, to highest right one. 

You can change the scanned (calculated) area by changing parameter *SlovenianMapBounds*. The program works faster if you omit the normal calculations - set parameter *IncludeNormals* to false.

### Result
<img src="https://user-images.githubusercontent.com/32905529/37048579-9288ade0-216e-11e8-9289-9763b315686d.png" width="40%" height="40%"> and <img src="https://user-images.githubusercontent.com/32905529/37048583-93954504-216e-11e8-83b9-51988915d0b7.PNG" width="50%" height="70%"> 

are merged to

![img1](https://user-images.githubusercontent.com/32905529/37048578-918f7f72-216e-11e8-914e-ea46f2bdc1b1.PNG)
