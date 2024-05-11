# Purpose
This is a learning log of how to use [DHT22](https://osoyoo.com/ja/2018/03/15/arduino-lesson-dht22-humidity-and-temperature-sensor/) sensor with STM32F446RE. The DHT22 sensor is a digital sensor that can measure both temperature and humidity. It is a very popular sensor and is widely used in many applications. In this project, we will use the DHT22 sensor to measure the temperature and humidity of the environment and display the results on the serial monitor.

# Build/Run
Use STM32CubeIDE to build the project. 
The project is built for STM32F446RE microcontroller.
Then, upload the code to the microcontroller using a ST-Link programmer.

# Serial Monitor
On mac, you can check the port that the STM32F446RE is connected to with the following command:
```terminal
ls /dev/tty.*
```
Find the port that the STM32F446RE is connected to. 
Then, open the serial monitor with the following command:
```terminal
screen /dev/tty.usbmodem14103 {PORT_RATE}
```

