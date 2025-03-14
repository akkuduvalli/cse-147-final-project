# cse-147-final-project

## Usage 
Ensure that you have the WiringPi and GNU Libmicrohttpd libraries installed. 

### Starting the Web Server and Joystick + Autonomous interaction of Car

Run the following commands——

```
cd sensor_test
make clean
make
make run
```

### Starting the Web page 
In `website/index.html`, replace `"http://172.20.10.13:8080/status"` and `http://172.20.10.13:8080/direction?dir=${dir}&speed=${speed}` with the correct IP - address of your Raspberry Pi
```
cd website
python3 -m http.server 8000
```
Open forwarded port `8000`
