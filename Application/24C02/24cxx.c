#include "24cxx.h"
#include "delay.h"


/*
 * AT24CXX_Init
 * Initialize the I2C interface used by the AT24Cxx driver.
 * Call once during system startup before any EEPROM access.
 */
void AT24CXX_Init(void)
{
	IIC_Init(); // Initialize bit-banged I2C pins
}
/*
 * AT24CXX_ReadOneByte
 * Read one byte from the EEPROM at the specified address.
 * ReadAddr: absolute EEPROM address to read.
 * return  : the byte value read.
 * Notes   : Handles device address paging for >24C16 parts.
 */
u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{				  
	u8 temp=0;	
	WP(0);	
    IIC_Start();  
	if(EE_TYPE>AT24C16)
	{
		IIC_Send_Byte(0XA0);       // device address + write
		IIC_Wait_Ack();
		IIC_Send_Byte(ReadAddr>>8); // high address byte	    
	}else IIC_Send_Byte(0XA0+((ReadAddr/256)<<1)); // device address + page 	   
	IIC_Wait_Ack(); 
    IIC_Send_Byte(ReadAddr%256); // low address byte
	IIC_Wait_Ack();	    
	IIC_Start();  	 	   
	IIC_Send_Byte(0XA1);         // device address + read			   
	IIC_Wait_Ack();	 
    temp=IIC_Read_Byte(0);		   
    IIC_Stop(); // stop condition	  
	WP(1);  
	return temp;
}
/*
 * AT24CXX_WriteOneByte
 * Write one byte to the EEPROM at the specified address.
 * WriteAddr  : absolute EEPROM address to write.
 * DataToWrite: byte value to store.
 * Notes      : Includes write-cycle delay and handles >24C16 addressing.
 */
void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite)
{			
	WP(0);	
    IIC_Start();  
	if(EE_TYPE>AT24C16)
	{
		IIC_Send_Byte(0XA0);       // device address + write
		IIC_Wait_Ack();
		IIC_Send_Byte(WriteAddr>>8); // high address byte	  
	}else IIC_Send_Byte(0XA0+((WriteAddr/256)<<1)); // device address + page 	 
	IIC_Wait_Ack();	   
    IIC_Send_Byte(WriteAddr%256); // low address byte
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(DataToWrite);  // data byte							   
	IIC_Wait_Ack();  		    	   
    IIC_Stop(); // stop condition 
	delay_ms(10);	 
	WP(1);
}
/*
 * AT24CXX_WriteLenByte
 * Write a multi-byte value (16-bit or 32-bit) starting at WriteAddr.
 * WriteAddr  : start address.
 * DataToWrite: value to write.
 * Len        : number of bytes to write (2 or 4).
 */
void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u8 t;
	for(t=0;t<Len;t++)
	{
		AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

/*
 * AT24CXX_ReadLenByte
 * Read a multi-byte value (16-bit or 32-bit) starting at ReadAddr.
 * ReadAddr: start address.
 * return : the value read.
 * Len    : number of bytes to read (2 or 4).
 */
u32 AT24CXX_ReadLenByte(u16 ReadAddr,u8 Len)
{  	
	u8 t;
	u32 temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=AT24CXX_ReadOneByte(ReadAddr+Len-t-1); 	 				   
	}
	return temp;												    
}
/*
 * AT24CXX_Check
 * Verify the EEPROM is working by writing/reading a marker byte.
 * Uses the last address (16383) as a test location (adjust for other parts).
 * return 1: check failed
 * return 0: check passed
 */
u8 AT24CXX_Check(void)
{
	u8 temp;
	temp=AT24CXX_ReadOneByte(16383); // avoid writing on every boot			   
	if(temp==0X55)return 0;		   
	else // handle first-time initialization
	{
		AT24CXX_WriteOneByte(16383,0X55);
	    temp=AT24CXX_ReadOneByte(16383);	  
		if(temp==0X55)return 0;
	}
	return 1;											  
}

/*
 * AT24CXX_Read
 * Read a continuous block of bytes starting at ReadAddr.
 * ReadAddr : start address (for 24C02, 0..255).
 * pBuffer  : destination buffer.
 * NumToRead: number of bytes to read.
 */
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  
/*
 * AT24CXX_Write
 * Write a continuous block of bytes starting at WriteAddr.
 * WriteAddr : start address (for 24C02, 0..255).
 * pBuffer   : source buffer.
 * NumToWrite: number of bytes to write.
 */
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite)
{
	while(NumToWrite--)
	{
		AT24CXX_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}




