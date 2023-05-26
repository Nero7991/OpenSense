# OpenSense - Companion application for OpenSAS to achieve spectrum sensing and intelligent incumbent detection

clone the esc application: https://github.com/CCI-NextG-Testbed/OpenSense.git
In sas_esc folder, edit esc_node.cpp file:
  add // Paths to the certificates,keys and the url of the OpenSAS server (generated same as CBSD certificates using the script)
  **Example:** 
  ```
 std::string client_crt_path = "../certs/client_10.147.20.75-0.crt"; 
 std::string client_key_path = "../certs/client_10.147.20.75-0.key"; 
 std::string ca_crt_path = "../certs/ca.crt";
  ```
  Path to Open-SAS URL
  ```
  std::string opensas_url = "https://10.147.20.75:1443/sas-api/";
  ```
  Also, set the sensor node numbers. #define SENSOR_NODE 1 (different for each N310 as ESC with their co-ordinates)
  After the edits, in sas_esc folder, compile and build the ESC app.
  ```
mkdir build 
cd build
cmake ../
make esc_node
 ```
Note: before starting the ESC application, make sure OpenSAS is running.
Start the ESC app: 
```
./esc_node --freq 3650e6 --gain 75 --rate 122.88e6 --args "addr=192.168.119.2,master_clock_rate=122.88e6,clock_source=internal" --num-avgs 4
```
freq = center frequency
rate = sampling rate , should be grater than 100 MHz
num-avgs = nummber of averages on the FFT bins.

To log the output of ESC application, use:
```
./esc_node --freq 3650e6 --gain 75 --rate 122.88e6 --args "addr=192.168.119.2,master_clock_rate=122.88e6,clock_source=internal" --num-avgs 4 | tee log.txt
```
