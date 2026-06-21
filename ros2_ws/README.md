# mobile_base
---
# Update 
- Run the workspace in Docker using
```
docker start ros2_dev
docker exec -it ros2_dev bash
```

# Updates 6/20/2026
- The new iteration of the robot is implemented, with new hardware and URDF. 
- Switch from deploying in Docker to deploying locally. Took over the ownership with:
```
cd ~
sudo chown -R $USER:$USER ros2_ws
```