# RoboCar Racing

## Outline
This repo includes the codebase for autonomous racing for an RC car following the F1Tenth competition specification. This porject is for EEE 587 - Optimal Control course at Arizona State University.

## Requirements
- Docker Desktop

## Building the Docker Container
To run the F1Tenth Gym environment for the first time, a stack of 2 Docker containers must be built from the Dockerfile provided in repository. The Docker containers are an environment that includes all the needed dependencies (ROS2, Python) without having to install them locally; they will only be available inside the containers. 
To build the containers, follow the steps below:

1. Clone the repository to a local directory **C:\\...\RoboCar-Racing**

2. Open **Docker Desktop**

3. Open a command prompt, and navigate to the cloned repository folder
```
cd C:\...\RoboCar-Racing
```

4. Run the following command in the command prompt to build the needed docker images and start the containers. This should take around 15 minutes.
```
docker-compose up --build
```
NOTE: The command might occasionally fail to connect a server that it is trying to reach. In that case, just run the command again. It will cache all the steps that it did before and continue from where it failed.

After the command runs successfully, your command prompt should look as follows:
![alt text](https://private-user-images.githubusercontent.com/98316521/417860818-762ef4fa-3cfa-4fe0-9653-018c7c3e7893.png?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NDA3MTE2NTQsIm5iZiI6MTc0MDcxMTM1NCwicGF0aCI6Ii85ODMxNjUyMS80MTc4NjA4MTgtNzYyZWY0ZmEtM2NmYS00ZmUwLTk2NTMtMDE4YzdjM2U3ODkzLnBuZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNTAyMjglMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjUwMjI4VDAyNTU1NFomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPWNiZjk0YjA4MDg5ZDZmOTc1ZDA4NWM0ZTNhYTVmMjYyM2ExZGIyNzYxZGE0NzMyNWZjNTc0Y2I4NjdkNjg4YzAmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.UfvMJMNzRS5nJU6Aez640nUGTOn_pCORQC1G5Ipd_n0)
Also, the 2 containers (sim-1 and novnc-1) in the stack (robocar-racing) should be up and running as can be seen in Docker Desktop.
![alt text](https://private-user-images.githubusercontent.com/98316521/417861676-5da4346f-cd59-44f9-90e1-1c31655f5b4c.png?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NDA3MTE2OTcsIm5iZiI6MTc0MDcxMTM5NywicGF0aCI6Ii85ODMxNjUyMS80MTc4NjE2NzYtNWRhNDM0NmYtY2Q1OS00NGY5LTkwZTEtMWMzMTY1NWY1YjRjLnBuZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNTAyMjglMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjUwMjI4VDAyNTYzN1omWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTA1OGIwNjY0ZDgwOWFmNmU3ZGYxNTAwODc2NDA3YWI1ZmMyNGFhOTA3OGI0YTE3YmYzY2Q0OGZiY2U0YjJjOTkmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.WcXASnasaZ1riff4S8fmh4OxtJ8kKMF2XkWFusvPlG4)

IMPORTANT: The 4 steps above are only needed once in order to build the containers. After the containers are successfully built, they can simply be started or stopped using the **Start** and **Stop** icons under the **Actions** column (check image above).

Alternatively, the containers can be started and stopped from the command prompt using:
```
docker start robocar-racing-sim-1
docker start robocar-racing-novnc-1
```
```
docker stop robocar-racing-sim-1
docker stop robocar-racing-novnc-1
```

## Running the F1Tenth Gym Environment
The environment runs inside the Docker containers created in the previous steps. 

To start the Gym environment inside the Docker containers, follow the steps below:

1. Start the 2 containers **sim-1** and **novnc-1** from Docker Desktop or from the command promt. (Check IMPORTANT in the previous section)

2. The next step is to open a bash session inside the **sim-1** container. To do that, open a new command prompt and run the following command:
```
docker exec -it robocar-racing-sim-1 /bin/bash
```
This is how the command prompt should look like after steps 1 & 2:
![alt text](https://private-user-images.githubusercontent.com/98316521/417861841-ad3fadfa-39d6-452a-8033-2c121773085f.png?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NDA3MTE4ODUsIm5iZiI6MTc0MDcxMTU4NSwicGF0aCI6Ii85ODMxNjUyMS80MTc4NjE4NDEtYWQzZmFkZmEtMzlkNi00NTJhLTgwMzMtMmMxMjE3NzMwODVmLnBuZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNTAyMjglMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjUwMjI4VDAyNTk0NVomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPWQ1MWNmNzFhNWMxYzY5OTM0YzFhNWQ4ZDJjZjhmODMyZGFkM2QyZGUwMDcxMTAxYjEzODlkZmU1NzdlNDcxZGImWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.sK7WBTAaf813ChvihAkyFIB4pEBYqFFBs_dRFPq48iE)

3. In the bash session, source the needed **ros** files, then build the project by running these commands in order:
```
cd .. 
source opt/ros/iron/setup.bash
cd sim_ws 
colcon build
source install/local_setup.bash
```
![alt text](https://private-user-images.githubusercontent.com/98316521/417861929-2c614b60-99de-4b14-9bb1-5660f47cd7a5.png?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NDA3MTE4ODUsIm5iZiI6MTc0MDcxMTU4NSwicGF0aCI6Ii85ODMxNjUyMS80MTc4NjE5MjktMmM2MTRiNjAtOTlkZS00YjE0LTliYjEtNTY2MGY0N2NkN2E1LnBuZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNTAyMjglMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjUwMjI4VDAyNTk0NVomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTMwMGYzY2I0ZTg4NDAxYWM5MTFkNDFiMGE2YzkyZjE3NWYwYTUxYzg0ZThkY2Y2OGViYTQ0NDRhOWMwYzJkYTYmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.YDq1QpxxYwSoNxWbWl2bPOePC9_f2yaQluvgGNIqtq8)

4. Start **tmux** in the bash session with:
```
tmux
```
**tmux** will allow you to open mutliple bash sessions inside the container without repeating steps 2-3. Use **ctrl+B - C** to open a new session in tmux. Use **ctrl+B - #** to navigate between the sessions with # being the number of the session. 

The image below shows how the terminal looks like after starting tmux and opening 2 sessions (session 0 and session 1), both running inside the container:

![alt text](https://private-user-images.githubusercontent.com/98316521/417862011-a806e456-bed0-45be-b4a8-ade19f620e94.png?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NDA3MTE4ODUsIm5iZiI6MTc0MDcxMTU4NSwicGF0aCI6Ii85ODMxNjUyMS80MTc4NjIwMTEtYTgwNmU0NTYtYmVkMC00NWJlLWI0YTgtYWRlMTlmNjIwZTk0LnBuZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNTAyMjglMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjUwMjI4VDAyNTk0NVomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPWI5ZTYxZjdhMzg2ZjFiNzliZjEyZjAwZmU5ZTIzM2ViNjM5MTU1YmNkOWJlNWM1ZTI1N2U1MTBiMDI1NzcwMzQmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.5IJVu28OycYmi7B0xgTYp8ZRUTGwpLvHJTSB7AN5B68)

5. In a web browser, open http://localhost:8080/vnc.html and click **Connect**.

For better visualization, change the **Scaling Mode** setting to **Local Scaling**.
![alt text](https://private-user-images.githubusercontent.com/98316521/417862765-35ae47a0-8c5f-4b14-b1c8-c3c4296988a8.png?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NDA3MTE5ODMsIm5iZiI6MTc0MDcxMTY4MywicGF0aCI6Ii85ODMxNjUyMS80MTc4NjI3NjUtMzVhZTQ3YTAtOGM1Zi00YjE0LWIxYzgtYzNjNDI5Njk4OGE4LnBuZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNTAyMjglMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjUwMjI4VDAzMDEyM1omWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTNiMjA0NWU0ZWQ5NjJlMjhiNDIyYTA4NzhmNDViYmEyZDA3NzhlMGYwZDhhOGMyY2M5ODYxOTU4Mzk3MzFmNzAmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.muKaTTi78HGRI_aJK-PJ_88rBTdLbZdkvNVTTpZIVUc)

6. In the first bash session, launch the gym environment by typing the command:
```
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
The webpage will update to the following:
![alt text](https://private-user-images.githubusercontent.com/98316521/417862031-483653ee-8b7b-4c3a-954d-211f437e9317.png?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NDA3MTE4ODUsIm5iZiI6MTc0MDcxMTU4NSwicGF0aCI6Ii85ODMxNjUyMS80MTc4NjIwMzEtNDgzNjUzZWUtOGI3Yi00YzNhLTk1NGQtMjExZjQzN2U5MzE3LnBuZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNTAyMjglMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjUwMjI4VDAyNTk0NVomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTUwZWRmOTIxZTNkMzJkNzEzYWRmZDc4ODE4M2M3MGY3Yjk2YmRkNzdlYTE4MWU3YTQ5OWM3YThlNmI0NTI2Y2YmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.XDN3OoKoA3j5ToaExC4K0q5ogTKmcALsG2nsDRRtWLw)

## Controlling the Car with Keyboard Keys
After starting the Gym environment, the car can be controlled with keyboard commands.
In another bash session, run the keyboard teleoperations node to be able to drive the car by pressing keyboard keys:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
![alt text](https://private-user-images.githubusercontent.com/98316521/417862046-43141718-ea81-45d1-b8ce-6f19bdc9e388.png?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NDA3MTE5ODMsIm5iZiI6MTc0MDcxMTY4MywicGF0aCI6Ii85ODMxNjUyMS80MTc4NjIwNDYtNDMxNDE3MTgtZWE4MS00NWQxLWI4Y2UtNmYxOWJkYzllMzg4LnBuZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNTAyMjglMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjUwMjI4VDAzMDEyM1omWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTk3M2MyZTNjYjc1YmRhYmFhOGQyYjA4YzYzY2Y5Yzg3N2Q2ZTI3YzY0ZGEyNmYxMGNjN2M4OTFmYjZkM2U3ZWUmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.H4xAPfTPlsoogjp5vjxrgPoZ0s40rEA9_gEeSmNVQtQ)

The car can be moved by clicking on keyboard keys while in this session as described in the image.