#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#include "sys.h"

#define CAN_FILTER_ID       (0x00d5 << 13)
#define CAN_FILTER_MASK     (0x00ff << 13)

#define ONLYONCE       0x00
#define BEGIAN         0x01
#define TRANSING       0x02
#define END            0x03

#if 0
typedef union
{
	struct
	{
		uint32_t SourceID  : 8;
		uint32_t FUNC_ID   : 4;
		uint32_t ACK       : 1;
		uint32_t DestMACID : 8;
		uint32_t SrcMACID  : 8;
		uint32_t res       : 3;
	} __attribute__ ((packed)) CanID_Struct;
	uint32_t  CANx_ID;
} __attribute__ ((packed)) CAN_ID_UNION;


typedef union
{
	struct
	{
				uint8_t SegNum  : 6;
				uint8_t SegPolo : 2;
		uint8_t Data[7];
	} __attribute__ ((packed)) CanData_Struct;
	uint8_t CanData[8];
} __attribute__ ((packed)) CAN_DATA_UNION;

#else
typedef union
{
	struct
	{
		uint32_t SourceID  : 8;
		uint32_t FUNC_ID   : 4;
		uint32_t ACK       : 1;
		uint32_t DestMACID : 8;
		uint32_t SrcMACID  : 8;
		uint32_t res       : 3;
	} __attribute__ ((packed)) CanID_Struct;
	uint32_t  CANx_ID;
} __attribute__ ((packed)) CAN_ID_UNION;


typedef union
{
	struct
	{
		uint8_t Data[7];
		uint8_t SegPolo : 2;
		uint8_t SegNum  : 6;
	} __attribute__ ((packed)) CanData_Struct;
	uint8_t CanData[8];
} __attribute__ ((packed)) CAN_DATA_UNION;
#endif
struct can_timing_t {
	const char *baud;
	uint16_t brp; // brp[0:9]
	uint16_t ts; // res[15] lbkm[14] res[13:10] swj[9:8] res[7] ts2[6:4] ts1[3:0]
} __attribute__ ((packed));

#define CAN_MSG_SIZE  0x0F // DLC[0:3]
#define CAN_MSG_RTR   0x10 // RTR[4]
#define CAN_MSG_EID   0x20 // EID[5]
#define CAN_MSG_INV   0x40 // is message in-valid

struct can_message_t {
	uint32_t id;
	uint8_t data_len;
	uint8_t data[8];
} __attribute__ ((packed));

class loop_buffer {
public:
	#define RBUF_SIZE 10
	struct can_message_t rbuf[RBUF_SIZE];
  uint8_t  rbuf_head;
	uint8_t  rbuf_tail;
  inline void rbuf_enqueue(struct can_message_t *msg);
  inline struct can_message_t rbuf_dequeue(void);
  inline bool is_rbuf_has_data(void);
  inline void rbuf_clear(void);
  loop_buffer();
};

class can_interface {
private:
	struct can_timing_t *CAN_Timing;
  uint8_t 						changeBaudRate(const char* baud);
  NVIC_InitTypeDef 		CAN_Int;
public:
  loop_buffer 				rx_buf;
	can_interface(const uint8_t id, const char* baud);
  //~can_interface();
  void canHWReinit();
  void can_filter_apply();
  void can_filter_addmask(uint16_t cobid, uint16_t cobid_mask, uint8_t prio);
	uint8_t can_send(struct can_message_t *m);
  struct can_message_t can_read(void);
  void canAckBack(uint32_t CANx_ID, const uint8_t * const pdata, uint16_t len);
};

uint8_t can_receive();

extern can_interface can;
//extern can_interface can(0x66, "500K");
#endif // CAN_INTERFACE_H

