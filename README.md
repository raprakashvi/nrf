# nrf

### BLE_SPI
- #### UART + BLE + FDS 
  - can capture 2043 data points and write them to flash and transfer to BLE
  - uses FDS library , UART , and SPI together
  - need to include timer() events to control the flow
  - debugging to increase the flash storage size and use timers

- #### UART
  - can send sensor data through BLE using BLE UART example
  - slow, as the data is being captured and sent at the same time
  - no flash

- ### LIS3DH
  - contains .h and .c files for the sensor LIS3DH


### SPI_LIS3DH
- can print X Y Z values in Log at set sampling rate
- no way to know the current sampling rate 
- add file saving option to this


### BLE
- can send simple data to BLE 
- only allows to write one byte of data at a time
- difficult to merge them together later


