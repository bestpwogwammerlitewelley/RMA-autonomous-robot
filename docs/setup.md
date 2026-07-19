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

## ESP32 setup (Arduino IDE, Windows)

1. Install Arduino IDE 2.x (arduino.cc)
2. USB driver: our boards use CP2102 → download "CP210x Universal Windows Driver"
   from Silabs, extract, right-click the `silabser` (Setup Information) file → Install.
   Then plug in ESP32 → a COM port appears in Device Manager → Ports.
3. IDE: File → Preferences → Additional boards URL:
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   Then Tools → Board → Boards Manager → install "esp32 by Espressif".
4. Tools → Board → ESP32 Dev Module; Tools → Port → your COM port.
5. Test upload: Blink example. Gotchas:
   - Add `#define LED_BUILTIN 2` at the top (not defined for this board)
   - If stuck at "Connecting...", hold the BOOT button on the board
   - Our clone boards have no visible program LED — use a Serial.println
     sketch + Serial Monitor at 115200 baud to verify instead.
   - IDE saying "offline" is harmless (cloud status only).
