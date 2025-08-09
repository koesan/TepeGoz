# Gozlem_Dronu

1. çok dornulu similsayonu başlat

roslaunch iq_sim multi_drone.launch


2. Dron bağlanıtlarını farklı bir termninalde başalt


sim_vehicle.py -v ArduCopter -f gazebo-drone1 --console -I0 --out=tcpin:0.0.0.0:8100 

sim_vehicle.py -v ArduCopter -f gazebo-drone2 --console -I1 --out=tcpin:0.0.0.0:8200 

sim_vehicle.py -v ArduCopter -f gazebo-drone2 --console -I2 --out=tcpin:0.0.0.0:8300 
