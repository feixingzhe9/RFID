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

bool can_interface::is_can_has_data(void)
{
	return rx_buf.is_rbuf_has_data();
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
    memset(rbuf, 0, sizeof(rbuf));
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


CAN_LONG_BUF_T can_long_frame_buf_ram;
CAN_LONG_BUF_T *can_long_frame_buf = &can_long_frame_buf_ram;

#define CAN_LONG_BUF_FULL   0xff
static uint8_t GetOneFreeBuf(void)
{
    for(uint8_t i = 0; i < CAN_LONG_BUF_NUM; i++)
    {
        if(can_long_frame_buf->can_rcv_buf[i].used_len == 0)
        {
            return i;
        }
    }
    return CAN_LONG_BUF_FULL;
}
static void FreeBuf(uint8_t index)
{
    can_long_frame_buf->can_rcv_buf[index].can_id = 0;
    can_long_frame_buf->can_rcv_buf[index].used_len = 0;
}
#define CAN_BUF_NO_THIS_ID      0xfe
static uint8_t GetTheBufById(uint32_t id)
{
    for(uint8_t i = 0; i < CAN_LONG_BUF_NUM; i++)
    {
        if(id == can_long_frame_buf->can_rcv_buf[i].can_id)
        {
            return i;
        }
    }
    return CAN_BUF_NO_THIS_ID;
}
void CanLongBufInit(void)
{ 
    can_long_frame_buf->GetOneFreeBuf = GetOneFreeBuf;
    can_long_frame_buf->GetTheBufById = GetTheBufById;
    can_long_frame_buf->FreeBuf = FreeBuf;
    
    //my_id = GetCanMacId();//test 
    
    //FifoInit(can_fifo, can_pkg, CAN_FIFO_SIZE);
}



char sw_version[] = "NoahC001M18A007";
#define CMD_NOT_FOUND   0
uint16_t cmd_procesing(CAN_ID_UNION *id, const uint8_t *data_in, const uint16_t data_in_len, uint8_t *data_out)
{

    id->CanID_Struct.ACK = 1;   
    id->CanID_Struct.DestMACID = id->CanID_Struct.SrcMACID;
    id->CanID_Struct.SrcMACID = RFID_CAN_MAC_SRC_ID;
    id->CanID_Struct.res = 0;

    static uint32_t can_test_cnt = 0;
     
    switch(id->CanID_Struct.FUNC_ID)
    {
        case CAN_FUN_ID_RESET:
            
            break;
        case CAN_FUN_ID_WRITE:
        case CAN_FUN_ID_READ:
            switch(id->CanID_Struct.SourceID)
            {
                case CAN_SOURCE_ID_READ_VERSION:

                    //if(data_in_len == 1)
                    {
                        //memcpy(&data_out[1],SW_VERSION[11],sizeof(SW_VERSION) - 11);
                        //data_out[0] = strlen(SW_VERSION) - 11;
                        memcpy(&data_out[1],&sw_version[11],sizeof(sw_version) - 11);
                        data_out[0] = strlen(sw_version) - 11;
                        return (data_out[0] + 1);
                    }
                    break;
   
                case CAN_SOURCE_ID_CAN_TEST:
                    can_test_cnt++;
                    memcpy(&data_out[0], (uint8_t *)&can_test_cnt, sizeof(can_test_cnt));
                    return sizeof(can_test_cnt);
                default :
                    break;
            }


        default: 
        break;
    }

    return CMD_NOT_FOUND;
}

uint8_t CanTxdataBuff[CAN_LONG_FRAME_LENTH_MAX] = {0};
void can_protocol(void)
{
    while(can.is_can_has_data() == true)
    {
        can_message_t can_data;
        memset(&can_data, 0 ,sizeof(can_data));
        can_data = can.can_read();
        
        
        CAN_ID_UNION id;
        CAN_DATA_UNION rx_buf;
        
        uint16_t tx_len = 0;
        uint8_t seg_polo = 0;  
        uint8_t rx_data_len = 0;
        
        //uint8_t buf_index = 0;
        //uint8_t seg_num = 0;
        //seg_num = rx_buf.CanData_Struct.SegNum;
        
        memset(&id, 0, sizeof(id));
        memset(&rx_buf, 0, sizeof(rx_buf));
        
        memcpy(rx_buf.CanData,  can_data.data, can_data.data_len);
        id.CANx_ID = can_data.id;
        seg_polo = rx_buf.CanData_Struct.SegPolo;
        
        rx_data_len = can_data.data_len;
        
        
        
        if(seg_polo == ONLYONCE)
        {
            //if( (id.CanID_Struct.SourceID < SOURCE_ID_PREPARE_UPDATE) && (id.CanID_Struct.SourceID > SOURCE_ID_CHECK_TRANSMIT) )
            if(RFID_CAN_MAC_SRC_ID == id.CanID_Struct.DestMACID)
            {
                memset(CanTxdataBuff, 0 ,sizeof(CanTxdataBuff));
                tx_len = cmd_procesing(&id, rx_buf.CanData_Struct.Data, rx_data_len - 1, CanTxdataBuff );
                //process the data here//
                
                if(tx_len > 0)
                {
                    //CanTX( MICO_CAN1, id.CANx_ID, CanTxdataBuff, tx_len );
                    can_message_t can_send_msg;
                    memset(&can_send_msg, 0, sizeof(can_send_msg));
                    can_send_msg.id = id.CANx_ID;
                    can_send_msg.data_len = tx_len;
                    memcpy(can_send_msg.data , CanTxdataBuff, can_send_msg.data_len);
                    can.can_send( &can_send_msg );
                }        
            }
        }
        else //long frame
        {
#if 0
            for(uint8_t i = 0; i < CAN_LONG_BUF_NUM; i++)
            {
                if(can_long_frame_buf->can_rcv_buf[i].used_len > 0)
                {
                    //if(os_get_time() - can_long_frame_buf->can_rcv_buf[i].start_time > CAN_LONG_FRAME_TIME_OUT)
                    {
                        //can_long_frame_buf->FreeBuf(i);
                    }
                }     
            }
            
            if(seg_polo == BEGIAN)
            {
                buf_index = can_long_frame_buf->GetTheBufById(id.CANx_ID);
                if(buf_index == CAN_BUF_NO_THIS_ID)
                {
                    buf_index = can_long_frame_buf->GetOneFreeBuf();
                }
                else
                {
                    //
                }
                
                if((buf_index == CAN_LONG_BUF_FULL) || (buf_index >= CAN_LONG_BUF_NUM))
                {
                    CanProtocolLog("LONG FRAME RCV BUF IS FULL! ! ! !\r\n");
                    
                    goto exit;
                }
                memcpy(&can_long_frame_buf->can_rcv_buf[buf_index].rcv_buf[0], rx_buf.CanData_Struct.Data, CAN_ONE_FRAME_DATA_LENTH);
                can_long_frame_buf->can_rcv_buf[buf_index].used_len = CAN_ONE_FRAME_DATA_LENTH;
                can_long_frame_buf->can_rcv_buf[buf_index].can_id = id.CANx_ID;
                can_long_frame_buf->can_rcv_buf[buf_index].start_time = os_get_time();
                CanProtocolLog("begin\r\n");
            }
            else if((seg_polo == TRANSING) || (seg_polo == END))
            {
                buf_index = can_long_frame_buf->GetTheBufById(id.CANx_ID);
                if((buf_index == CAN_BUF_NO_THIS_ID) || (buf_index >= CAN_LONG_BUF_NUM))
                {
                    CanProtocolLog("ERROR ! !\r\n long buff index is %d",buf_index);
                    goto exit;
                }
                can_long_frame_buf->can_rcv_buf[buf_index].start_time = os_get_time();
                if(seg_polo == TRANSING)
                {
                    memcpy(&can_long_frame_buf->can_rcv_buf[buf_index].rcv_buf[seg_num*CAN_ONE_FRAME_DATA_LENTH], rx_buf.CanData_Struct.Data, CAN_ONE_FRAME_DATA_LENTH);
                    can_long_frame_buf->can_rcv_buf[buf_index].used_len += CAN_ONE_FRAME_DATA_LENTH;
                    CanProtocolLog("transing\r\n");
                }
                if(seg_polo == END)
                {
                    memcpy(&can_long_frame_buf->can_rcv_buf[buf_index].rcv_buf[seg_num*CAN_ONE_FRAME_DATA_LENTH], rx_buf.CanData_Struct.Data, rx_data_len - 1);
                    can_long_frame_buf->can_rcv_buf[buf_index].used_len += rx_data_len - 1; 
                    
                    //process the data here//
                    /**********************/
                    //process the data here//
                    
                    CanTX( MICO_CAN1, id.CANx_ID, can_long_frame_buf->can_rcv_buf[buf_index].rcv_buf, can_long_frame_buf->can_rcv_buf[buf_index].used_len);  // test :send the data back;             
                    can_long_frame_buf->FreeBuf(buf_index);
                    CanProtocolLog("end\r\n");
                }       
            }
#endif
        }
    }
}
