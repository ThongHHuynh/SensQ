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

# Updates 6/27/2026
- Navigation is running using nav2
- Error: global costmap is off from local and lidar -> Solution: match the location start mapping to location start navigating