#include "xsens.h"
#include "mt_message.h"

void xsens_init(struct XsensVex* x, FILE* usart, unsigned int baud_rate)
{
  x->usart = usart;
  x->lastPacket.mutex = mutexCreate();

  // Initialize USART port
  usartInit(usart, baud_rate, SERIAL_8N1);

  logger_init(&x->log, LOGLEVEL_WARNING);
}

void xsens_task(void* param)
{
  struct XsensVex* x = (struct XsensVex*)param;

  // Defensive check
  if (x == 0)
  {
      logger_critical(&x->log, "Defensive Check Failed\n");
      return;
  }

  // Set the default heading to zero
  xsens_reset_heading(x, 0, 0, 0);

  // Tell IMU to go into measurement mode
  mtGoToMeasurement(x->usart);

  logger_info(&x->log, "XSENS Starting measurement loop\n");

  unsigned long lastTimeMTData2 = millis();

  // Main loop for gathering data
  while (true)
  {
    // Blocking function that waits for the next complete MT message
    struct MTMessage msg;
    int checksum_error = getMTMessage(&msg, x->usart);

    if (checksum_error)
    {
      logger_error(&x->log, "XSENS Checksum error\n");
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

      // Update the current heading by the rate of turn.
      // RateOfTurn will have a bias associated with it, so subtract the
      // heading bias calculated by xsens_calibrate.  Since RateOfTurn and
      // heading_bias is in rad/sec, a conversion is necessary to get it into
      // degrees.
      for (int i = 0; i < 3; i++)
        x->heading[i] +=
          (x->lastPacket.XDI_RateOfTurn[i] - x->heading_bias[i]) * 57.2958 * dT;
    }
  }
}

void xsens_calibrate(struct XsensVex* x, int numSamples)
{
  if (x == 0)
  {
    struct Logger* log = logger_get_global_log();
    logger_critical(log, "Defensive check failed\n");
    return;
  }

  // Gives some time between when the xsens task is started and when the first
  // sample is recorded here.  If not, the first packet might not have come
  // through yet and will be all zeros by default.
  delay(200);

  logger_info(&x->log, "XSENS calibrating\n");

  double average[3] = {0, 0, 0};
  uint32_t lastPacketNum;

  for (int i = 0; i < numSamples; i++)
  {
    mutexTake(x->lastPacket.mutex, -1);
    lastPacketNum = x->lastPacket.XDI_PacketCounter;
    for (int i = 0; i < 3; i++)
      average[i] += x->lastPacket.XDI_RateOfTurn[i];
    mutexGive(x->lastPacket.mutex);

    // Wait for the next packet
    while (x->lastPacket.XDI_PacketCounter <= lastPacketNum)
      delay(10);
  }

  for (int i = 0; i < 3; i++)
    x->heading_bias[i] = average[i] / numSamples;

  logger_info(&x->log, "Gyro Bias: %f %f %f\n",
    x->heading_bias[0],
    x->heading_bias[1],
    x->heading_bias[2]);

  xsens_reset_heading(x, 0, 0, 0);
}

void xsens_start_task(struct XsensVex* x)
{
  logger_info(&x->log, "XSENS Starting task\n");
  x->handle = taskCreate(xsens_task, TASK_DEFAULT_STACK_SIZE,
    (void*)x, TASK_PRIORITY_DEFAULT);
}

void xsens_reset_heading(struct XsensVex* x,
  double pitch, double roll, double yaw)
{
  mutexTake(x->lastPacket.mutex, -1);
  x->heading[0] = pitch;
  x->heading[1] = roll;
  x->heading[2] = yaw;
  mutexGive(x->lastPacket.mutex);
}

double xsens_get_pitch(struct XsensVex* x)
{
  return x->heading[0];
}

double xsens_get_roll(struct XsensVex* x)
{
  return x->heading[1];
}

double xsens_get_yaw(struct XsensVex* x)
{
  return x->heading[2];
}
