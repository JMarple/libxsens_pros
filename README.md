# XSENS PROS2 driver for the Vex Cortex

This library allows for any XSENS IMU to be used with the Vex Cortex when using PROS
Written by Justin Marple (justinjmarple@gmail.com), supported by Team Bots 'n' Stuff (BNS)

---
### Installation
First, ensure you have the latest version of liblogger installed: https://github.com/JMarple/liblogger_pros

Then install the same way:

Add a new depot

`"Name the depot": libxsens`

`"Depot location": jmarple/libxsens_pros`

Then download libxsens and add it to your project normally.

Alternatively to using the GUI, you can use the command line, described below:

```
pros conduct add-depot --name libxsens --registrar github-releases --location jmarple/libxsens_pros --no-configure
pros conduct download libxsens
pros conduct add-lib <project-path> libxsens
```

---

libxsens contains the following API calls:
 - void xsens_init(struct XsensVex* x, FILE* usart, unsigned int baud_rate)
 - void xsens_start_task(struct XsensVex* x)
 - void xsens_calibrate(struct XsensVex* x, int numSamples)
 - void xsens_reset_heading(struct XsensVex* x, double pitch, double roll, double yaw)
 - double xsens_get_pitch(struct XsensVex* x)
 - double xsens_get_roll(struct XsensVex* x)
 - double xsens_get_yaw(struct XsensVex* x)

A quick example:
```c
#include "xsens.h"

struct XsensVex xsens;

xsens_init(&xsens, uart1, 38400);
xsens_start_task(&xsens);
xsens_calibrate(&xsens);

while (true)
{
  printf("Yaw = %f\n", xsens_get_yaw(&xses));
  delay(50);
}
```

### Hardware

XSENS IMU's communicate over RS232, RS422 or RS485, depending on the version you ordered.  In order for an XSENS IMU to communicate with the cortex, a converter is required to convert those signals to UART.  RS232 to UART, RS422 to UART and RS485 to UART converters can be found cheaply online.  Ensure you wire correctly and check the electrical requirements for your specific IMU.  We are not responsible for any damage done to the IMU.  

It is also required to go into MT Manager, click the icon for output configuration, and choose Normal Mode.  Under Timestamp, click Packet Counter and Sample Time Fine. Under Inertial data, click DeltaQ, DeltaV, RateOfTurn, Acceleration, FreeAcceleration, FloatingPoint 32-bit, 25Hz.  Under magnetic field, click MagneticField, FloatingPoint 32-bit, 5Hz.  Under Status, click Status Word.  Then press OK.  You can adjust the baud rate and update rates to get the most data out of your IMU as possible.  

NOTE: If data is being sent out faster then the set baud rate, the IMU will hault and stop sending data.  This is a common reason for the IMU to not work.  
