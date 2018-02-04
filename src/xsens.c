#include "xsens.h"
#include "mt_message.h"

void xsens_init(struct XsensVex* x, FILE* usart, unsigned int baud_rate)
{
  x->usart = usart;
  x->lastPacket.mutex = mutexCreate();

  // Initialize USART port
  usartInit(usart, baud_rate, SERIAL_8N1);
}

void xsens_start_task(struct XsensVex* x)
{
  x->handle = taskCreate(xsens_task, TASK_DEFAULT_STACK_SIZE,
    (void*)x, TASK_PRIORITY_DEFAULT);
}

void xsens_task(void* param)
{
  struct XsensVex* x = (struct XsensVex*)param;

  // Defensive check
  if (x == 0) return;

  // Set the default heading to zero
  xsens_reset_heading(x, 0);

  // Tell IMU to go into measurement mode
  mtGoToMeasurement(x->usart);

  unsigned long lastTimeMTData2 = millis();

  // Main loop for gathering data
  while (true)
  {
    // Blocking function that waits for the next complete MT message
    struct MTMessage msg;
    int checksum_error = getMTMessage(&msg, x->usart);

    if (checksum_error)
    {
      printf("XSENS Checksum error\n");
      continue;
    }

    // If the message is an MTData2 message
    if (isMTData2(&msg))
    {
      // Save data to lastPacket
      parseMTData2(&msg, &x->lastPacket);

      unsigned long curTime = millis();
      double dT = (curTime - lastTimeMTData2)/1000.0;
      lastTimeMTData2 = curTime;

      // Determine the euler yaw angle from the current heading
      x->heading_yaw +=
        (x->lastPacket.XDI_RateOfTurn[2] - x->heading_bias) * 57.2958 * dT;
    }
  }
}

void xsens_calibrate(struct XsensVex* x, int numSamples)
{
  // Gives some time between when the xsens task is started
  // and when the first sample is recorded here.
  delay(200);

  double average_yaw = 0;
  uint32_t lastPacketNum;

  for (int i = 0; i < numSamples; i++)
  {
    mutexTake(x->lastPacket.mutex, -1);
    lastPacketNum = x->lastPacket.XDI_PacketCounter;
    average_yaw += x->lastPacket.XDI_RateOfTurn[2];
    mutexGive(x->lastPacket.mutex);

    // Wait for the next packet
    while (x->lastPacket.XDI_PacketCounter <= lastPacketNum)
      delay(10);
  }

  x->heading_bias = average_yaw / numSamples;
  xsens_reset_heading(x, 0);
}

void xsens_reset_heading(struct XsensVex* x, double value)
{
  x->heading_yaw = value;
}
