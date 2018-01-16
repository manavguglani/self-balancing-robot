
#ifndef SCILAB_H_
#define SCILAB_H_


#define BINARYSOURCE_EOF      1
#define BINARYSOURCE_OK       0
#define BINARYSOURCE_ERROR    255

union DataTransformation
{
	struct
	{
		unsigned char b0;
		unsigned char b1;
		unsigned char b2;
		unsigned char b3;
	} buffer; // byte
	struct
	{
		unsigned char value;
		unsigned char dummy1;
		unsigned char dummy2;
		unsigned char dummy3;
	} b; // byte
	struct
	{
		unsigned short value;
		unsigned short dummy1;
	} w; // word
	struct
	{
		unsigned long value;
	} d; // double word
	struct
	{
		float value;
	} f;
};

struct rxFrame
{
	unsigned char rx_data;
	unsigned char crc1;
	unsigned char crc2;
	
};
/****************************
This is the structure which you need to modify
Add structures members as per your need and make corresponding changes in the scilab 
*****************************/
struct txFrame
{
	double tilt_angle;
	unsigned char crc1;
	unsigned char crc2;
};

struct binarysource {
	unsigned char *buffer;
	unsigned short length;
	unsigned short index;
	unsigned char finished;
	unsigned char base64index;
};

unsigned short base64comm_crc(unsigned short crc, unsigned char rx);
void binarysource_init(struct binarysource *source, unsigned char *buffer, unsigned short length);
unsigned char binarysource_putnextbase64(struct binarysource *source, unsigned char c);
unsigned char binarysource_getnextbase64(struct binarysource *source, unsigned char *c);
unsigned short binarysource_getlength(struct binarysource *source);
void send_data_frame(struct txFrame *txf);
void recieve_data_frame(struct rxFrame *rxf);

#endif 