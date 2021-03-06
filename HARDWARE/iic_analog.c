#include "iic_analog.h"
void IIC_GPIO_Config(GPIO_TypeDef* GPIOx_SDA ,uint16_t SDA_Pin,GPIO_TypeDef* GPIOx_SCL ,uint16_t SCL_Pin)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

  //GPIOB8,B9初始化设置
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	//3?ê??ˉIICμ??￡ê?
	SET_SDA;
	SET_SCL;  
}
/*------------------------------------------
 oˉêy1|?ü: ?￡?aIICê±Dò?óê±
 oˉêy2?êy: ?T
------------------------------------------*/
void IIC_Delay(void)
{
	uint8_t i = 0;
	while( i-- );
}

/*------------------------------------------
 oˉêy1|?ü: IIC???ˉê±Dò
 oˉêy2?êy: ?T
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
 oˉêy1|?ü: IICí￡?1ê±Dò
 oˉêy2?êy: ?T
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
 oˉêy1|?ü: ·￠?íNACK
 oˉêy2?êy: ê?μ?êy?Y·￠?íNACKD?o?
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
 oˉêy1|?ü: ·￠?íACKD?o?
 oˉêy2?êy: ?T
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
 oˉêy1|?ü: IIC·￠?íò???×??ú
 oˉêy2?êy: - Data 1×??úêy?Y
------------------------------------------*/
uint8_t IIC_SendByte(uint8_t Data)
{
	 uint8_t i;
	 RESET_SCL;
	 for(i=0;i<8;i++)
	 {  
			//---------êy?Y?¨á￠----------
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
			//---êy?Y?¨á￠±￡3?ò??¨?óê±----
			//----2úéúò???é?éy??[?y??3?] 
			SET_SCL;
			IIC_DELAY;
			RESET_SCL;
			IIC_DELAY;//?óê±,·à?1SCL?1??±?3éμíê±??±?SDA,′ó??2úéúSTART/STOPD?o? 
	 }
	 //?óê?′ó?úμ?ó|′e 
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
 oˉêy1|?ü: IIC?óê?ò???×??ú
 oˉêy2?êy: ·μ??1×??úêy?Y
------------------------------------------*/
uint8_t IIC_RecvByte(void)
{
	 uint8_t i,Dat = 0;
	 SET_SDA;
	 RESET_SCL; 
	 Dat = 0;
	 for(i=0; i<8; i++)
	 {
			SET_SCL;           //2úéúê±?óé?éy??[?y??3?],è?′ó?ú×?±?o?êy?Y 
			IIC_DELAY; 
			Dat <<= 1;
			if(IIC_SDA_STATE)  //?áòy??×′ì?
			{
				Dat|=0x01; 
			}   
			RESET_SCL;         //×?±?o??ù′??óê?êy?Y  
			IIC_DELAY;         //μè′yêy?Y×?±?o?         
	 }
	 return Dat;
}
/*------------------------------------------
 oˉêy1|?ü: ?òIICéè±?D′è?êy?Y
 oˉêy2?êy: - SlaveAddress ′ó?úμ??·
				   - REG_Address  ??′??÷μ??·
					 - REG_data     êy?Y
------------------------------------------*/
void IIC_WriteData(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data)
{
	IIC_Start_v2();                  //?eê?D?o?
	IIC_SendByte(SlaveAddress);   //·￠?íéè±?μ??·+D′D?o?
	IIC_SendByte(REG_Address);    //?ú2???′??÷μ??·
	IIC_SendByte(REG_data);       //?ú2???′??÷êy?Y
	IIC_Stop_v2();                   //·￠?íí￡?1D?o?
}

/*------------------------------------------
 oˉêy1|?ü: ′óIICéè±??áè?êy?Y
 oˉêy2?êy: - SlaveAddress ′ó?úμ??·
				   - REG_Address  ??′??÷μ??·
 ·μ ?? ?μ: ?áè?μ?1×??úêy?Y
------------------------------------------*/
uint8_t IIC_ReadData(uint8_t SlaveAddress,uint8_t REG_Address)
{  
	uint8_t REG_data;
	IIC_Start_v2();                          //?eê?D?o?
	IIC_SendByte(SlaveAddress);           //·￠?íéè±?μ??·+D′D?o?
	IIC_SendByte(REG_Address);            //·￠?í′?′￠μ￥?aμ??·￡?//′ó0?aê?	
	IIC_Start_v2();                          //?eê?D?o?
	IIC_SendByte(SlaveAddress+1);         //·￠?íéè±?μ??·+?áD?o?
	REG_data = IIC_RecvByte();            //?á3???′??÷êy?Y
	IIC_SendACK();   
	IIC_Stop_v2();                           //í￡?1D?o?
  return REG_data; 
}
/*------------------------------------------
 oˉêy1|?ü: GPIO?????÷
 oˉêy2?êy: - GPIO_TypeDef
 ·μ ?? ?μ: ???úμ??·êy?Y
------------------------------------------*/

