#ifndef __XSENS_H__
#define __XSENS_H__

#define XSENS_DEBUGGING 0

#include "API.h"

#include "mt_message.h"

// A structure that holds data from the IMU
struct XsensVex
{
  // Accounts for a bias on the rateOfTurn output.
  double heading_bias;

  // Accumulated heading, yaw only
  double heading_yaw;

  // PROS File handler for the UART port
  FILE* usart;

  // Task handler for the thread reading and writing data
  TaskHandle handle;

  // Last recorded packet off the MTI device.
  // NOTE: Use the mutex in MTdata2 to read data safetly
  struct MTData2 lastPacket;
};

void xsens_init(struct XsensVex* x, FILE* usart, unsigned int baud_rate);
void xsens_task(void* param);
void xsens_start_task(struct XsensVex* x);

void xsens_calibrate(struct XsensVex* x, int numSamples);
void xsens_reset_heading(struct XsensVex* x, double value);

#endif
