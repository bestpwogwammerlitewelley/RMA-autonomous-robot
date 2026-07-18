# Dev environment setup (Windows)

1. Install WSL: open Command Prompt, run `wsl --install`, create a Unix username/password when asked. **Reboot the PC after.**
2. Install Docker Desktop (docker.com, AMD64). Skip the sign-in. Wait for "Engine running" — first start is slow; if stuck >5 min, reboot again.
3. Test: `docker run hello-world` → should print a greeting.
4. Start the ROS 2 container: `docker run -it --name ros2 ros:humble` → you land at a `root@...` prompt (you are now inside Ubuntu).
5. Install demo nodes (base image doesn't include them):
   `apt update && apt install -y ros-humble-demo-nodes-cpp`
6. Run the talker: `ros2 run demo_nodes_cpp talker` → prints "Publishing: Hello World: N".
7. Open a SECOND terminal (PowerShell/CMD) and run:
   `docker exec -it ros2 bash`
   then INSIDE the container:
   `source /opt/ros/humble/setup.bash`
   `ros2 run demo_nodes_cpp listener`
   → prints "I heard: Hello World: N" matching the talker.

## Gotchas
- `PS C:\>` prompt = Windows: only `docker ...` commands work there.
- `root@...` prompt = inside the container: `ros2` and `source` commands go here.
- Every NEW shell inside the container needs `source /opt/ros/humble/setup.bash` first.
- Container already exists next time? Use `docker start -ai ros2` instead of `docker run`.
