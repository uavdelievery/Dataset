# Dataset
The UAV Delievery dataset created to advance the research in drone delivery, contains trajectory details of UAV in different speed, altitude, and wind conditions. The dataset is distributed under the GNU General Public License v3.
We created our dataset from the Truck based ["Online Food Delivery Platform"](https://www.sciencedirect.com/science/article/pii/S2352340919303609) using  Open Air Traffic Simulator (ATS) with a UAVTrajectory.py plugin. A pre-processing step is used to select the deliveries under distance 5km, due to battery constraints of UAVs. After the pre-processing, the dataset consists of a total number of 6911  deliveries that are simulated and collected in log files. The link for log files (dataset) is given below.  

### Link for the dataset:
[Dataset](https://drive.google.com/drive/folders/18qwp2zaRoBjtkId5sz83vArWgZxrZLCi?usp=sharing)

### Details of Dataset:
###### Folder naming:
The dataset is created with different hyperparameters like wind, UAV speed, altitude. For making differntiation among log files, we rename the folders as follows:
UAV_speed Wind_speed UAV_altitude

Each folder futher comprised of 5 log files, containing data of UAV in different wind conditions (i.e., no_wind, 0_wind, 90_wind, 180_wind, 270_wind).

###### Attributes
|Name|Description|Unit|
|----|-----|-------|
|simt|Simulation time|sec|
|id|Unique ID of UAV|string|
|type|Type of UAV|string|
|lat|Lattitude|deg|
|lon|Longitude|deg|
|alt|Altitude|m|
|distflown|Distane flown|m|
|temp|Air temperature|K|
|trk|Track angle|deg|
|hgd|Heading direction|deg|




***
### BlueSky Simulator [link](https://github.com/TUDelft-CNS-ATM/bluesky)
