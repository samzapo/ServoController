### Simple Arduino Control Protocol (Dynamixel and simple sensors)
## Installing

    my-laptop$ git clone https://github.com/samzapo/ServoController.git
    my-laptop$ cd ServoController
    my-laptop$ mkdir build; cd build
    my-laptop$ cmake ..
    my-laptop$ make

    and then run the test program

## Usage:

  test the library with:

    my-laptop$ ./ServoController-Test <arduino serial port> 

# Example of the Protocol

# Read Request

 *4 byte header:*
    
    Read/Write/RW
    Position/Velocity/Load/Misc
    N = # of IDs
    L = # of Bytes per ID data

*`N` data blocks of size `(L + 1)` bytes*

    1:
      ID (1 byte)
      Data (L bytes)

....

    N:
      ID (1 byte)
      Data (L bytes)


  So lets say I send

    READ, POSITION, 5, 0, 1, 2, 3, 4, 5

  This means I want the Arduino to read back to me the position of 5 actuators, these actuators are: 1,2,3,4, and 5.  The value of L is 0 because the IDs are not accompanied by any additional data.

  I should get the response

    READ, POSITION, 5,2, ...
    1, POSITION_LOW_BYTE, POSITION_HIGH_BYTE  (size 3 bytes)
    2, POSITION_LOW_BYTE, POSITION_HIGH_BYTE
    3, POSITION_LOW_BYTE, POSITION_HIGH_BYTE
    4, POSITION_LOW_BYTE, POSITION_HIGH_BYTE
    5, POSITION_LOW_BYTE, POSITION_HIGH_BYTE

# Write

    WRITE, POSITION, 5,2, ...
    1, POSITION_LOW_BYTE, POSITION_HIGH_BYTE  (size 3 bytes)
    2, POSITION_LOW_BYTE, POSITION_HIGH_BYTE
    3, POSITION_LOW_BYTE, POSITION_HIGH_BYTE
    4, POSITION_LOW_BYTE, POSITION_HIGH_BYTE
    5, POSITION_LOW_BYTE, POSITION_HIGH_BYTE
