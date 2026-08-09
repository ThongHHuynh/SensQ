# SensQ operator app

The FastAPI backend and React UI are isolated in this folder. ROS 2 remains in
`../ros2_ws`.

```bash
cd /home/tom/SensQ
./app/dev.sh
```

The app supports map loading, AMCL initial-pose selection, Nav2 goals,
coverage-route preview/execution, AprilTag station creation, and docking.

Local runs use `../data/sensq.db` through SQLite. Set `DATABASE_URL` to use
PostgreSQL instead.

Set `SENSQ_USE_SIM_TIME=true` when the backend is connected to simulation.
