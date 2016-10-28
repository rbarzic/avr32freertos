//*----------------------------------------------------------------------------
//*         ATMEL Microcontroller Software Support  -  ROUSSET  -
//*----------------------------------------------------------------------------
//* The software is delivered "AS IS" without warranty or condition of any
//* kind, either express, implied or statutory. This includes without
//* limitation any warranty or condition with respect to merchantability or
//* fitness for any particular purpose, or against the infringements of
//* intellectual property rights of others.
//*----------------------------------------------------------------------------
//* File Name           : Emac.h
//* Object              : Emac header file
//* Creation            : Hi   11/18/2002
//*
//*----------------------------------------------------------------------------
#ifndef AT32C_EMAC_H
#define AT32C_EMAC_H

// #include "lwipopts.h"


/* Number of receive buffers */
#define NB_RX_BUFFERS			20

/* Size of each receive buffer - DO NOT CHANGE. */
#define ETH_RX_BUFFER_SIZE		128         

/* Number of Transmit buffers */
#define NB_TX_BUFFERS			( MEMP_NUM_PBUF / 2 )	

/* Size of each Transmit buffer. */
#define ETH_TX_BUFFER_SIZE		( PBUF_POOL_BUFSIZE  )   

/* Receive Transfer descriptor structure */
typedef struct  _AT32S_RxTdDescriptor {
	unsigned long addr;
	union
	{
		unsigned int status;
		struct {
			unsigned int Length:11;
			unsigned int Res0:1;
			unsigned int Rxbuf_off:2;
			unsigned int StartOfFrame:1;
			unsigned int EndOfFrame:1;
			unsigned int Cfi:1;
			unsigned int VlanPriority:3;
			unsigned int PriorityTag:1;
			unsigned int VlanTag:1;
			unsigned int TypeID:1;
			unsigned int Sa4Match:1;
			unsigned int Sa3Match:1;
			unsigned int Sa2Match:1;
			unsigned int Sa1Match:1;
			unsigned int Res1:1;
			unsigned int ExternalAdd:1;
			unsigned int UniCast:1;
			unsigned int MultiCast:1;
			unsigned int BroadCast:1;
		}S_Status;		
	}U_Status;
}AT32S_RxTdDescriptor, *AT32PS_RxTdDescriptor;


/* Transmit Transfer descriptor structure */
typedef struct _AT32S_TxTdDescriptor {
	unsigned int addr;
	union
	{
		unsigned int status;
		struct {
			unsigned int Length:11;
			unsigned int Res0:4;
			unsigned int LastBuff:1;
			unsigned int NoCrc:1;
			unsigned int Res1:10;
			unsigned int BufExhausted:1;
			unsigned int TransmitUnderrun:1;
			unsigned int TransmitError:1;
			unsigned int Wrap:1;
			unsigned int BuffUsed:1;
		}S_Status;		
	}U_Status;
}AT32S_TxTdDescriptor, *AT32PS_TxTdDescriptor;

#define AT32C_OWNERSHIP_BIT		0x00000001

/* Receive status defintion */
#define AT32C_BROADCAST_ADDR	((unsigned int) (1 << 31))	//* Broadcat address detected
#define AT32C_MULTICAST_HASH 	((unsigned int) (1 << 30))	//* MultiCast hash match
#define AT32C_UNICAST_HASH 	    ((unsigned int) (1 << 29))	//* UniCast hash match
#define AT32C_EXTERNAL_ADDR	    ((unsigned int) (1 << 28))	//* External Address match
#define AT32C_SA1_ADDR	    	((unsigned int) (1 << 26))	//* Specific address 1 match
#define AT32C_SA2_ADDR	    	((unsigned int) (1 << 25))	//* Specific address 2 match
#define AT32C_SA3_ADDR	    	((unsigned int) (1 << 24))	//* Specific address 3 match
#define AT32C_SA4_ADDR	    	((unsigned int) (1 << 23))	//* Specific address 4 match
#define AT32C_TYPE_ID	    	((unsigned int) (1 << 22))	//* Type ID match
#define AT32C_VLAN_TAG	    	((unsigned int) (1 << 21))	//* VLAN tag detected
#define AT32C_PRIORITY_TAG    	((unsigned int) (1 << 20))	//* PRIORITY tag detected
#define AT32C_VLAN_PRIORITY    	((unsigned int) (7 << 17))  //* PRIORITY Mask
#define AT32C_CFI_IND        	((unsigned int) (1 << 16))  //* CFI indicator
#define AT32C_EOF           	((unsigned int) (1 << 15))  //* EOF
#define AT32C_SOF           	((unsigned int) (1 << 14))  //* SOF
#define AT32C_RBF_OFFSET     	((unsigned int) (3 << 12))  //* Receive Buffer Offset Mask
#define AT32C_LENGTH_FRAME     	((unsigned int) 0x07FF)     //* Length of frame

/* Transmit Status definition */
#define AT32C_TRANSMIT_OK		((unsigned int) (1 << 31))	//*
#define AT32C_TRANSMIT_WRAP		((unsigned int) (1 << 30))	//* Wrap bit: mark the last descriptor
#define AT32C_TRANSMIT_ERR		((unsigned int) (1 << 29))	//* RLE:transmit error
#define AT32C_TRANSMIT_UND		((unsigned int) (1 << 28))	//* Transmit Underrun
#define AT32C_BUF_EX     		((unsigned int) (1 << 27))	//* Buffers exhausted in mid frame
#define AT32C_TRANSMIT_NO_CRC	((unsigned int) (1 << 16))	//* No CRC will be appended to the current frame
#define AT32C_LAST_BUFFER    	((unsigned int) (1 << 15))	//*

#define AT32C_EMAC_CLKEN 0x2

#endif //* AT32C_EMAC_H
