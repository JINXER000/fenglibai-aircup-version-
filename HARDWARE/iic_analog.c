#include "iic_analog.h"
void IIC_GPIO_Config(GPIO_TypeDef* GPIOx_SDA ,uint16_t SDA_Pin,GPIO_TypeDef* GPIOx_SCL ,uint16_t SCL_Pin)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��

  //GPIOB8,B9��ʼ������
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	//3?��??��IIC��??�꨺?
	SET_SDA;
	SET_SCL;  
}
/*------------------------------------------
 o����y1|?��: ?��?aIIC����D��?������
 o����y2?��y: ?T
------------------------------------------*/
void IIC_Delay(void)
{
	uint8_t i = 0;
	while( i-- );
}

/*------------------------------------------
 o����y1|?��: IIC???������D��
 o����y2?��y: ?T
------------------------------------------*/
uint8_t IIC_Start_v2(void)
{
	SET_SDA;
	IIC_DELAY;

	SET_SCL;
	IIC_DELAY;

	if( IIC_SDA_STATE == RESET )
	{
		return IIC_BUS_BUSY;
	}

	RESET_SDA;
	IIC_DELAY;

	RESET_SCL;
	IIC_DELAY;

	if( IIC_SDA_STATE == SET )
	{
		return IIC_BUS_ERROR;
	}

	return IIC_BUS_READY;
}
/*------------------------------------------
 o����y1|?��: IIC����?1����D��
 o����y2?��y: ?T
------------------------------------------*/
void IIC_Stop_v2(void)
{
	RESET_SDA;
	IIC_DELAY;

	SET_SCL;
	IIC_DELAY;

	SET_SDA;
	IIC_DELAY;
}

/*------------------------------------------
 o����y1|?��: ����?��NACK
 o����y2?��y: ��?��?��y?Y����?��NACKD?o?
------------------------------------------*/
void IIC_SendNACK(void)
{
	RESET_SDA;
	IIC_DELAY;
	SET_SCL;
	IIC_DELAY;
	RESET_SCL; 
	IIC_DELAY; 
}
/*------------------------------------------
 o����y1|?��: ����?��ACKD?o?
 o����y2?��y: ?T
------------------------------------------*/
void IIC_SendACK(void)
{
	SET_SDA;
	IIC_DELAY;
	SET_SCL;
	IIC_DELAY;
	RESET_SCL; 
	IIC_DELAY;
}

/*------------------------------------------
 o����y1|?��: IIC����?����???��??��
 o����y2?��y: - Data 1��??����y?Y
------------------------------------------*/
uint8_t IIC_SendByte(uint8_t Data)
{
	 uint8_t i;
	 RESET_SCL;
	 for(i=0;i<8;i++)
	 {  
			//---------��y?Y?������----------
			if(Data & 0x80)
			{
				SET_SDA;
			}
			else
			{
				RESET_SDA;
			} 
			Data <<= 1;
			IIC_DELAY;
			//---��y?Y?���������3?��??��?������----
			//----2��������???��?��y??[?y??3?] 
			SET_SCL;
			IIC_DELAY;
			RESET_SCL;
			IIC_DELAY;//?������,����?1SCL?1??��?3���̨�����??��?SDA,�䨮??2������START/STOPD?o? 
	 }
	 //?����?�䨮?����?��|��e 
	 SET_SDA; 
	 IIC_DELAY;
	 SET_SCL;
	 IIC_DELAY;   
	 if(IIC_SDA_STATE)
	 {
			RESET_SCL;
			return IIC_NACK;
	 }
	 else
	 {
			RESET_SCL;
			return IIC_ACK;  
	 }    
}
/*------------------------------------------
 o����y1|?��: IIC?����?��???��??��
 o����y2?��y: ����??1��??����y?Y
------------------------------------------*/
uint8_t IIC_RecvByte(void)
{
	 uint8_t i,Dat = 0;
	 SET_SDA;
	 RESET_SCL; 
	 Dat = 0;
	 for(i=0; i<8; i++)
	 {
			SET_SCL;           //2����������?����?��y??[?y??3?],��?�䨮?����?��?o?��y?Y 
			IIC_DELAY; 
			Dat <<= 1;
			if(IIC_SDA_STATE)  //?����y??���䨬?
			{
				Dat|=0x01; 
			}   
			RESET_SCL;         //��?��?o??����??����?��y?Y  
			IIC_DELAY;         //�̨���y��y?Y��?��?o?         
	 }
	 return Dat;
}
/*------------------------------------------
 o����y1|?��: ?��IIC������?D�䨨?��y?Y
 o����y2?��y: - SlaveAddress �䨮?����??��
				   - REG_Address  ??��??�¦�??��
					 - REG_data     ��y?Y
------------------------------------------*/
void IIC_WriteData(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data)
{
	IIC_Start_v2();                  //?e��?D?o?
	IIC_SendByte(SlaveAddress);   //����?��������?��??��+D��D?o?
	IIC_SendByte(REG_Address);    //?��2???��??�¦�??��
	IIC_SendByte(REG_data);       //?��2???��??�¨�y?Y
	IIC_Stop_v2();                   //����?������?1D?o?
}

/*------------------------------------------
 o����y1|?��: �䨮IIC������??����?��y?Y
 o����y2?��y: - SlaveAddress �䨮?����??��
				   - REG_Address  ??��??�¦�??��
 ���� ?? ?��: ?����?��?1��??����y?Y
------------------------------------------*/
uint8_t IIC_ReadData(uint8_t SlaveAddress,uint8_t REG_Address)
{  
	uint8_t REG_data;
	IIC_Start_v2();                          //?e��?D?o?
	IIC_SendByte(SlaveAddress);           //����?��������?��??��+D��D?o?
	IIC_SendByte(REG_Address);            //����?����?���̣�?a��??����?//�䨮0?a��?	
	IIC_Start_v2();                          //?e��?D?o?
	IIC_SendByte(SlaveAddress+1);         //����?��������?��??��+?��D?o?
	REG_data = IIC_RecvByte();            //?��3???��??�¨�y?Y
	IIC_SendACK();   
	IIC_Stop_v2();                           //����?1D?o?
  return REG_data; 
}
/*------------------------------------------
 o����y1|?��: GPIO?????��
 o����y2?��y: - GPIO_TypeDef
 ���� ?? ?��: ???����??����y?Y
------------------------------------------*/

