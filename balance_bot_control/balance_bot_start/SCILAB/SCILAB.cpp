/********************************SCILAB_LIBRARY*****************************/
#include "../UART/UART.h"	//provided this is the location where you saved your uart library file
#include <util/delay.h>
#include "scilab.h"
#include <Arduino.h>

static struct binarysource rxsource;
static struct binarysource txsource;
static unsigned char txcounter;
union DataTransformation data;
unsigned short crc;
unsigned char *ptr;

unsigned short base64comm_crc(unsigned short crc, unsigned char rx)
{
  const unsigned short base64comm_crc16_table[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
  0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
  0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
  0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
  0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
  0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
  0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
  0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
  0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
  0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
  0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
  0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
  0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
  0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
  0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
  0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
  0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
  0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
  0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
  0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
  0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
  0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

  return ((crc >> 8) ^ base64comm_crc16_table[rx ^ (crc & 0xff)]);
}

void binarysource_init(struct binarysource *source, unsigned char *buffer, unsigned short length)
{
  source->buffer=buffer;
  source->length=length;
  source->index=0;
  source->base64index=0;
  source->finished=0;
}

unsigned char binarysource_putnextbase64(struct binarysource *source, unsigned char c)
{
  unsigned char inputvalue=255;
  unsigned char res=BINARYSOURCE_OK;
  if (c=='=' || source->finished != 0)
  {
    res=BINARYSOURCE_EOF;
    source->finished=1;
  }
  else if (c>='a') {
    inputvalue=c-'a'+26;
  } else if (c>='A')
  {
    inputvalue=c-'A';
  }
  else if (c>='0')
  {
    inputvalue=c-'0'+52;
  }
  else if (c=='+')
  {
    inputvalue=62;
  }
  else if (c=='/')
  {
    inputvalue=63;
  }
  else
  {
    res=BINARYSOURCE_ERROR;
  }
  if (res==BINARYSOURCE_OK)
  {
    if (source->index<source->length)
    {
      switch(source->base64index)
      {
        case 0:
          source->buffer[source->index]=(inputvalue<<2)&0xFC;
          source->base64index=1;
          break;
        case 1:
          source->buffer[source->index]|=(inputvalue>>4)&0x03;
          source->index++;
          source->buffer[source->index]=(inputvalue<<4)&0xF0;
          res=BINARYSOURCE_OK;
          source->base64index=2;
          break;
        case 2:
          source->buffer[source->index]|=(inputvalue>>2)&0x0F;
          source->index++;
          source->buffer[source->index]=(inputvalue<<6)&0xC0;
          res=BINARYSOURCE_OK;
          source->base64index=3;
          break;
        case 3:
          source->buffer[source->index]|=(inputvalue)&0x3F;
          source->index++;
          res=BINARYSOURCE_OK;
          source->base64index=0;
          break;
        default:
          break;
      }
    }
    else
    {
      res=BINARYSOURCE_ERROR;
    }
  }
  return res;
}

unsigned char binarysource_getnextbase64(struct binarysource *source, unsigned char *c)
{
  unsigned char res=BINARYSOURCE_ERROR;
  unsigned char x;
  static const char base64[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  if (source->index<source->length && source->finished == 0)
  {
    switch(source->base64index)
    {
      case 0:
        x=source->buffer[source->index]>>2;
        *c=base64[x];
        res=BINARYSOURCE_OK;
        source->base64index=1;
        break;
      case 1:
        x=(source->buffer[source->index]<<4)&0x30;
        source->index++;
        x|=(source->buffer[source->index]>>4)&0x0F;
        *c=base64[x];
        res=BINARYSOURCE_OK;
        source->base64index=2;
        break;
      case 2:
        x=(source->buffer[source->index]<<2)&0x3C;
        source->index++;
        x|=(source->buffer[source->index]>>6)&0x03;
        *c=base64[x];
        res=BINARYSOURCE_OK;
        source->base64index=3;
        break;
      case 3:
        x=source->buffer[source->index]&0x3F;
        source->index++;
        *c=base64[x];
        res=BINARYSOURCE_OK;
        source->base64index=0;
        break;
      default:
        break;
    }
  }
  else
  {
    switch(source->base64index)
    {
      case 0:
        source->finished=1;
        res=BINARYSOURCE_EOF;
        break;
      case 1:
        res=BINARYSOURCE_ERROR; // this should never happen here
        break;
      case 2:
        *c='=';
        res=BINARYSOURCE_OK;
        source->base64index=3;
        break;
      case 3:
        *c='=';
        res=BINARYSOURCE_OK;
        source->base64index=0;
        break;
    }
  }
  return res;
}

unsigned short binarysource_getlength(struct binarysource *source)
{
  return source->index;
}

void send_data_frame(struct txFrame *txf)
{
	unsigned char tx;
	static char txbuffer[100];
	unsigned char txlength;
	char j;
	char i;
	  
	ptr=(unsigned char *)(txf);
	crc=0x43A6;
	j=sizeof(*txf)-2;
	for(i=0;i<j;i++)
	   {
		   crc=base64comm_crc(crc,ptr[i]);
	   }
	data.w.value=crc;
	txf->crc1=data.buffer.b0;
	txf->crc2=data.buffer.b1;

	binarysource_init(&txsource,(unsigned char *)txf,j+2);

	txlength=0;
	txbuffer[txlength]='<';
	txlength++;
	while(binarysource_getnextbase64(&txsource,&tx)==BINARYSOURCE_OK)
	   {
		   txbuffer[txlength]=tx;
		   txlength++;
	   }
	txbuffer[txlength]='>';
	txlength++;
	txcounter = 1;
	if (txcounter>0)
		{
			Serial1.write((const uint8_t*)txbuffer,txlength);
			txcounter--;
		}
}

void recieve_data_frame(struct rxFrame *rxf)
{
	int rxchar=0;
	unsigned char rx;
	char j;
	char i;
	unsigned char flag=0;
	   
	while ( (rxchar = Serial1.read()) >= 0 )
		{
			flag=1;
			rx=(char)rxchar;
			if (rx=='<')
			{
				binarysource_init(&rxsource,(unsigned char *)(rxf),sizeof(*rxf));
			}
			else if (rx=='>' && (sizeof(*rxf) == binarysource_getlength(&rxsource)))
			{
				ptr=(unsigned char *)(rxf);
				crc=0x43A6;
				j=sizeof(*rxf)-2;
				for(i=0;i<j;i++)
				{
					crc=base64comm_crc(crc,ptr[i]);
				}
			    data.buffer.b0=rxf->crc1;
				data.buffer.b1=rxf->crc2;
				if (data.w.value == crc)
				{
					txcounter=100;  // allow further packages to be sent
									// rx-Frame valid, transfer it to outputs
				}
			}
			else
			{
				binarysource_putnextbase64(&rxsource,rx);
			}
		}
			if (flag==0)
			{
			      // add additional pause if no new data was received to avoid to much tx packages
			      _delay_ms(1);
			}	
}

