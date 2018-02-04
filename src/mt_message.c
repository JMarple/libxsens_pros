#include "mt_message.h"
#include <math.h>

int getMTMessage(struct MTMessage* msg, FILE* usart)
{
  // Defensive checks
  if (msg == 0) return 1;
  if (usart == 0) return 1;

  uint8_t checksum = 0;

  // Wait for preamble
  while (fgetc(usart) != 0xFA);

  // BID
  checksum += fgetc(usart);

  // MID
  msg->mid = fgetc(usart);
  checksum += msg->mid;

  // Data length
  msg->len = fgetc(usart);
  checksum += msg->len;

  // Data
  for (int i = 0; i < msg->len; i++)
  {
    msg->data[i] = fgetc(usart);
    checksum += msg->data[i];
  }

  // Checksum
  checksum += fgetc(usart);

  // Report checksum error
  if (checksum != 0) return 1;

  return 0;
}

int sendMTMessage(struct MTMessage* msg, FILE* usart)
{
  // Defensive checks
  if (msg == 0) return 1;
  if (usart == 0) return 1;

  uint8_t checksum = 0xFF;

  // Preamble: always 0xFA
  fputc(0xFA, usart);

  // BID: always 0xFF
  fputc(0xFF, usart);

  // MID: Message Identifier
  checksum += msg->mid;
  fputc(msg->mid, usart);

  // LEN: Data length
  checksum += msg->len;
  fputc(msg->len, usart);

  // Data
  for (int i = 0; i < msg->len; i++)
  {
    checksum += msg->data[i];
    fputc(msg->data[i], usart);
  }

  // Checksum
  fputc((0x00 - checksum), usart);

  return 0;
}

// Sends a simple request message.  No data is sent on these messages.
static void _simple_req_message(FILE* usart, unsigned int mid)
{
  // Defensive check
  if (usart == 0) return;

  struct MTMessage msg;
  msg.mid = mid;
  msg.len = 0x00;
  sendMTMessage(&msg, usart);
}

void mtGoToConfig(FILE* usart){ _simple_req_message(usart, 0x30); }
void mtGoToMeasurement(FILE* usart){ _simple_req_message(usart, 0x10); }
void mtReset(FILE* usart){  _simple_req_message(usart, 0x40); }
void mtReqDID(FILE* usart){ _simple_req_message(usart, 0x00); }
void mtReqProductCode(FILE* usart){ _simple_req_message(usart, 0x1C); }
void mtReqHardwareVersion(FILE* usart){ _simple_req_message(usart, 0x1E); }
void mtReqFWRev(FILE* usart){ _simple_req_message(usart, 0x12); }
void mtRunSelfTest(FILE* usart){ _simple_req_message(usart, 0x24); }
void mtReqBaudrate(FILE* usart){ _simple_req_message(usart, 0x18); }
void mtReqErrorMode(FILE* usart){ _simple_req_message(usart, 0xDA); }

void mtSetBaudrate(FILE* usart, enum MTBaudrate baudrate)
{
  // Defensive check
  if (usart == 0) return;

  struct MTMessage msg;
  msg.mid = 0x18;
  msg.len = 0x01;
  msg.data[0] = baudrate;
  sendMTMessage(&msg, usart);
}

void mtSetErrorMode(FILE* usart, unsigned int errormode)
{
  // Defensive check
  if (usart == 0) return;

  struct MTMessage msg;
  msg.mid = 0xDA;
  msg.len = 0x02;
  msg.data[0] = 0x00;
  msg.data[1] = errormode;
  sendMTMessage(&msg, usart);
}

int isMTData2(struct MTMessage* msg)
{
  return (msg->mid == 0x36);
}

static double _castIntToFloat(int a)
{
  int sign = (a & 0x80000000);
  int exponent = ((a & 0x7F800000) >> 23) - 127;
  double mantissa = 1 + ((a & 0x007FFFFF) / pow(2, 23));

  double value = pow(2, exponent) * mantissa;

  if (sign == 0x80000000)
  {
    value = -value;
  }

  return value;
}

static void _getDoubleFromBytes(double* output, uint8_t* data, int len)
{
  for (int j = 0; j < len; j+=4)
  {
    int a = data[j+3] | (data[j+2] << 8) | (data[j+1] << 16) | (data[j] << 24);
    float value = _castIntToFloat(a);
    output[j/4] = value;
  }
}

static void _getUInt32FromBytes(uint32_t* output, uint8_t* data, int len)
{
  for (int j = 0; j < len; j+= 4)
  {
    output[j/4] = data[j+3] | (data[j+2] << 8) | (data[j+1] << 16) | (data[j] << 24);
  }
}

void parseMTData2(struct MTMessage* msg, struct MTData2* data)
{
  mutexTake(data->mutex, -1);

  for (int i = 0; i < msg->len;)
  {
    // Get data code
    int code = (msg->data[i] << 8) | msg->data[i+1];
    i+=2;

    // Get length
    int len = msg->data[i];
    i+=1;

    // Parse payload
    if (code == XDI_DELTAQ)
      _getDoubleFromBytes(data->XDI_DeltaQ, &msg->data[i], len);
    else if (code == XDI_PACKETCOUNTER)
      data->XDI_PacketCounter = msg->data[i+1] | (msg->data[i] << 8);
    else if (code == XDI_SAMPLETIMEFINE)
      _getUInt32FromBytes(&data->XDI_SampleTimeFine, &msg->data[i], len);
    else if (code == XDI_RATEOFTURN)
      _getDoubleFromBytes(data->XDI_RateOfTurn, &msg->data[i], len);
    else if (code == XDI_MAGNETICFIELD)
      _getDoubleFromBytes(data->XDI_MagneticField, &msg->data[i], len);
    else if (code == XDI_DELTAV)
      _getDoubleFromBytes(data->XDI_DeltaV, &msg->data[i], len);
    else if (code == XDI_STATUSWORD)
      _getUInt32FromBytes(&data->XDI_StatusWord, &msg->data[i], len);
    else if (code == XDI_ACCELERATION)
      _getDoubleFromBytes(data->XDI_Acceleration, &msg->data[i], len);

    i+=len;
  }

  mutexGive(data->mutex);
}
