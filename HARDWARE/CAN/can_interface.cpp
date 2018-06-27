#include "can_interface.h"
#include "string.h"

can_interface can(0x66, "500K");
//"1M" "800K" "500K" "250K" "125K" "100K" "50K" "20K" "10K"
#define TIMINGS_NUM (sizeof(CAN_Timings)/sizeof(struct can_timing_t))
static const struct can_timing_t CAN_Timings[] = {
	{"1M",   0x03, 0x115 }, // 36MHz peripheral clock, sample @ 33%, SWJ = 2, TS1 = 1+1, TS2 = 5+1, BRP = 4
	{"500K", 0x07, 0x115 }, // 36MHz peripheral clock, sample @ 33%, SWJ = 2, TS1 = 1+1, TS2 = 5+1, BRP = 8
	{"250K", 0x0f, 0x115 }, // 36MHz peripheral clock, sample @ 33%, SWJ = 2, TS1 = 1+1, TS2 = 5+1, BRP = 16
};

#define FILTER_NUM	14
static CAN_FilterInitTypeDef CAN_Filters[FILTER_NUM] = {
	// Id_H,  Id_L    MskIdH  MskIdL  FIFO Filt# Mode                   Scale                  Active 
	{ 0x0000, 0x0000, 0x0000, 0x0000, 0,   0,    CAN_FilterMode_IdMask, CAN_FilterScale_32bit, ENABLE },
};

can_interface::can_interface(const uint8_t id, const char* baud) {
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	// Configure CAN pin: RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Configure CAN pin: TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//CAN_Timing = (struct can_timing_t *)&CAN_Timings[0];
	//changeBaudRate(baud);
	
	canHWReinit();
}

/*
can_interface::~can_interface() {
//	delete this->rx_buf;
}*/

uint8_t can_interface::changeBaudRate(const char* baud)
{
	uint8_t i;
	for (i = 0; i < TIMINGS_NUM; i++){
		if (strcmp(CAN_Timings[i].baud, baud) == 0) {
			CAN_Timing = (struct can_timing_t *)&CAN_Timings[i];
			return 0;
		}
	}
	return 1;
}

void can_interface::canHWReinit()
{
  CAN_InitTypeDef 				CAN_InitStructure;
	CAN_FilterInitTypeDef  	CAN_FilterInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
  CAN_StructInit(&CAN_InitStructure);
	
	CAN_InitStructure.CAN_Mode 			= CAN_Mode_Normal;
	CAN_InitStructure.CAN_Prescaler = 9;
	CAN_InitStructure.CAN_ABOM			= ENABLE;
	CAN_InitStructure.CAN_AWUM			= ENABLE;
	CAN_InitStructure.CAN_SJW				= CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1				= CAN_BS1_4tq;
	CAN_InitStructure.CAN_BS2				= CAN_BS2_3tq;
	CAN_DeInit(CAN1);
	CAN_Init(CAN1, &CAN_InitStructure);
	
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = ((CAN_FILTER_ID << 3) >> 16) & 0xffff;//0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = (uint16_t)(CAN_FILTER_ID << 3) | CAN_ID_EXT;//0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (CAN_FILTER_MASK << 3) >> 16;//0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = ((CAN_FILTER_MASK << 3) & 0xffff) | 0x06;//0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation 		= ENABLE;
	
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_Int.NVIC_IRQChannelCmd 								= ENABLE;
	CAN_Int.NVIC_IRQChannel 									= USB_LP_CAN1_RX0_IRQn;
	CAN_Int.NVIC_IRQChannelPreemptionPriority = 1;
	CAN_Int.NVIC_IRQChannelSubPriority 				= 1;
	NVIC_Init(&CAN_Int);
	
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

void can_interface::can_filter_apply()
{
	int i;
	// setup can filters
	for (i = 0; i < FILTER_NUM; i++) {
		if (CAN_Filters[i].CAN_FilterActivation == DISABLE) 
			break;
		CAN_FilterInit(&CAN_Filters[i]);
	}
}

void can_interface::can_filter_addmask(uint16_t cobid, uint16_t cobid_mask, uint8_t prio)
{
	uint8_t i = 0;

	for (i = 0; i < FILTER_NUM; i++) {
		if (CAN_Filters[i].CAN_FilterActivation == DISABLE)
			break;
	}

	// check limit
	if (i >= FILTER_NUM) 
		return;

	CAN_Filters[i].CAN_FilterActivation = ENABLE;
	CAN_Filters[i].CAN_FilterNumber = i;
	CAN_Filters[i].CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_Filters[i].CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_Filters[i].CAN_FilterFIFOAssignment = (prio > 0)? 1: 0;
	CAN_Filters[i].CAN_FilterIdHigh = cobid << 5;
	CAN_Filters[i].CAN_FilterIdLow = 0x0000;
	CAN_Filters[i].CAN_FilterMaskIdHigh = cobid_mask << 5;
	CAN_Filters[i].CAN_FilterMaskIdLow = 0x0004;
}
/* TODO: priority scheduling */
uint8_t can_interface::can_send(struct can_message_t *m)
{
	CanTxMsg transmit_msg;
	
	transmit_msg.ExtId = m->id;
	if ( m->data_len > 8)
	{
		return 0;
	}
	transmit_msg.DLC = m->data_len;
	transmit_msg.RTR = CAN_RTR_Data;
	transmit_msg.IDE = CAN_Id_Extended;
	memcpy(&transmit_msg.Data, m->data, m->data_len);
	
	return CAN_Transmit(CAN1, &transmit_msg);
}

struct can_message_t can_interface::can_read()
{
	return rx_buf.rbuf_dequeue();
}

void can_interface::canAckBack(uint32_t CANx_ID, const uint8_t * const pdata, uint16_t len)
{
  uint16_t t_len;
  struct can_message_t TxMessage;
  CAN_ID_UNION id;
  uint8_t src_mac_id_temp;
  CAN_DATA_UNION TxMsg;
  
  id.CANx_ID = CANx_ID;
  id.CanID_Struct.ACK = 1;
  src_mac_id_temp = id.CanID_Struct.DestMACID;
  id.CanID_Struct.DestMACID = id.CanID_Struct.SrcMACID;
  id.CanID_Struct.SrcMACID = src_mac_id_temp;
  
  t_len = len;
  if( t_len <=7 )
  {
      TxMsg.CanData_Struct.SegPolo = ONLYONCE;
      TxMsg.CanData_Struct.SegNum = 0;
      memcpy( TxMsg.CanData_Struct.Data, (const void *)pdata, t_len );
      memcpy( TxMessage.data, TxMsg.CanData, t_len + 1 );
      
      TxMessage.data_len = t_len + 1;
      can_send(&TxMessage);
  }
}
/* TX interrupt */
/* TODO: error propagation */
void USB_HP_CAN1_TX_IRQHandler(void) {
	if (CAN1->TSR & CAN_TSR_RQCP0) {
		CAN1->TSR |= CAN_TSR_RQCP0;
	}
	if (CAN1->TSR & CAN_TSR_RQCP1) {
		CAN1->TSR |= CAN_TSR_RQCP1;
	}
	if (CAN1->TSR & CAN_TSR_RQCP2) {
/*		CANController_Status |= (CAN1->TSR & CAN_TSR_ALST2)?CAN_STAT_ALST:0;
		CANController_Status |= (CAN1->TSR & CAN_TSR_TERR2)?CAN_STAT_TERR:0;
		CANController_Status |= (CAN1->TSR & CAN_TSR_TXOK2)?CAN_STAT_TXOK:0;*/
		CAN1->TSR |= CAN_TSR_RQCP2;
	}
}

loop_buffer::loop_buffer()
{
	rbuf_head = 0;
	rbuf_tail = 0;
}

inline void loop_buffer::rbuf_enqueue(struct can_message_t *msg)
{
  uint8_t next = (rbuf_head + 1) % RBUF_SIZE;
  if (next != rbuf_tail)
  {
    memcpy(&rbuf[rbuf_head], msg, sizeof(struct can_message_t));
    rbuf_head = next;
  }
}

inline struct can_message_t loop_buffer::rbuf_dequeue(void)
{
  struct can_message_t val;
  
	std::memset(&val, 0x0, sizeof(struct can_message_t));
  if (rbuf_head != rbuf_tail)
  {
    memcpy(&val, &rbuf[rbuf_tail], sizeof(struct can_message_t));
    rbuf_tail = (rbuf_tail + 1) % RBUF_SIZE;
  }
  return val;
}

inline bool loop_buffer::is_rbuf_has_data(void)
{
  return (rbuf_head != rbuf_tail);
}

inline void loop_buffer::rbuf_clear(void)
{
  rbuf_head = rbuf_tail = 0;
}

struct can_message_t rec_msg;

uint8_t can_receive()
{
	CanRxMsg receive_msg;
	struct can_message_t m;
	
	CAN_Receive(CAN1, CAN_FIFO0, &receive_msg);
	if (receive_msg.IDE == CAN_Id_Extended)
	{
		m.id = receive_msg.ExtId;
		m.data_len = receive_msg.DLC;
		memcpy(m.data, receive_msg.Data, receive_msg.DLC);
	}
	
	can.rx_buf.rbuf_enqueue(&m);
	
	return 0;
}

extern "C" void USB_LP_CAN1_RX0_IRQHandler(void)
{
	if ( SET == CAN_GetITStatus(CAN1, CAN_IT_FMP0) && SET == CAN_GetFlagStatus(CAN1, CAN_FLAG_FMP0) )
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_ClearFlag(CAN1, CAN_FLAG_FMP0);
		CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);
		can_receive();
		CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	}
}
//last line
