/*
 * (C) Copyright 2002
 * Gerald Van Baren, Custom IDEAS, vanbaren@cideas.com.
 *
 * Influenced by code from:
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <spi.h>

#include <malloc.h>

/*-----------------------------------------------------------------------
 * Definitions
 */

#ifdef DEBUG_SPI
#define PRINTD(fmt,args...)	printf (fmt ,##args)
#else
#define PRINTD(fmt,args...)
#endif
extern void set_soft_spi_ss(char set);
extern void set_soft_spi_clk(char set);
extern void set_soft_spi_sda(char set);
extern int  get_sda_io(void);

#define	SPI_DELAY	udelay(10)
#define	SPI_SDA(val)	set_soft_spi_sda(val)
#define	SPI_SCL(val)	set_soft_spi_clk(val)
#define	SPI_READ	get_sda_io()
struct soft_spi_slave {
	struct spi_slave slave;
	unsigned int mode;
};

static inline struct soft_spi_slave *to_soft_spi(struct spi_slave *slave)
{
	return container_of(slave, struct soft_spi_slave, slave);
}

/*=====================================================================*/
/*                         Public Functions                            */
/*=====================================================================*/

/*-----------------------------------------------------------------------
 * Initialization
 */
void  lcd_spi_init (void)
{
}

struct spi_slave *lcd_spi_setup_slave(unsigned int bus, unsigned int cs,
		unsigned int max_hz, unsigned int mode)
{
	struct soft_spi_slave *ss;

	if (!spi_cs_is_valid(bus, cs))
		return NULL;

	ss = spi_alloc_slave(struct soft_spi_slave, bus, cs);
	if (!ss)
		return NULL;

	ss->mode = mode;

	/* TODO: Use max_hz to limit the SCK rate */

	return &ss->slave;
}

void lcd_spi_free_slave(struct spi_slave *slave)
{
	struct soft_spi_slave *ss = to_soft_spi(slave);

	free(ss);
}

int lcd_spi_claim_bus(struct spi_slave *slave)
{
#ifdef CONFIG_SYS_IMMR
	volatile immap_t *immr = (immap_t *)CONFIG_SYS_IMMR;
#endif
	struct soft_spi_slave *ss = to_soft_spi(slave);

	/*
	 * Make sure the SPI clock is in idle state as defined for
	 * this slave.
	 */
	if (ss->mode & SPI_CPOL)
		SPI_SCL(1);
	else
		SPI_SCL(0);

	return 0;
}

void lcd_spi_release_bus(struct spi_slave *slave)
{
	/* Nothing to do */
}
/*


MX6_PAD_UART2_TX_DATA__GPIO1_IO20 ss
MX6_PAD_UART2_RX_DATA__GPIO1_IO21 clk
MX6_PAD_UART2_RTS_B__GPIO1_IO23  read
MX6_PAD_UART2_CTS_B__GPIO1_IO22 out
gpio_direction_output(IMX_GPIO_NR(5, 9) , 0);
*/





/*-----------------------------------------------------------------------
 * SPI transfer
 *
 * This writes "bitlen" bits out the SPI MOSI port and simultaneously clocks
 * "bitlen" bits in the SPI MISO port.  That's just the way SPI works.
 *
 * The source of the outgoing bits is the "dout" parameter and the
 * destination of the input bits is the "din" parameter.  Note that "dout"
 * and "din" can point to the same memory location, in which case the
 * input data overwrites the output data (since both are buffered by
 * temporary variables, this is OK).
 */
int  lcd_spi_xfer(struct spi_slave *slave, unsigned int bitlen,
		const void *dout, void *din, unsigned long flags)
{
#ifdef CONFIG_SYS_IMMR
	volatile immap_t *immr = (immap_t *)CONFIG_SYS_IMMR;
#endif
	struct soft_spi_slave *ss = to_soft_spi(slave);
	uchar		tmpdin  = 0;
	uchar		tmpdout = 0;
	const u8	*txd = dout;
	u8		*rxd = din;
	int		cpol = ss->mode & SPI_CPOL;
	int		cpha = ss->mode & SPI_CPHA;
	unsigned int	j;

	PRINTD("spi_xfer: slave %u:%u dout %08X din %08X bitlen %u\n",
		slave->bus, slave->cs, *(uint *)txd, *(uint *)rxd, bitlen);

	if (flags & SPI_XFER_BEGIN)
		spi_cs_activate(slave);

	for(j = 0; j < bitlen; j++) {
		/*
		 * Check if it is time to work on a new byte.
		 */
		if ((j % 8) == 0) {
			if (txd)
				tmpdout = *txd++;
			else
				tmpdout = 0;
			if(j != 0) {
				if (rxd)
					*rxd++ = tmpdin;
			}
			tmpdin  = 0;
		}

		if (!cpha)
			SPI_SCL(!cpol);
		SPI_SDA(tmpdout & 0x80);
		SPI_DELAY;
		if (cpha)
			SPI_SCL(!cpol);
		else
			SPI_SCL(cpol);
		tmpdin	<<= 1;
		tmpdin	|= SPI_READ;
		tmpdout	<<= 1;
		SPI_DELAY;
		if (cpha)
			SPI_SCL(cpol);
	}
	/*
	 * If the number of bits isn't a multiple of 8, shift the last
	 * bits over to left-justify them.  Then store the last byte
	 * read in.
	 */
	if (rxd) {
		if ((bitlen % 8) != 0)
			tmpdin <<= 8 - (bitlen % 8);
		*rxd++ = tmpdin;
	}

	if (flags & SPI_XFER_END)
		spi_cs_deactivate(slave);

	return(0);
}




static struct spi_slave *st_slave;
//默认
/*
SPI3_CS0 
board_spi_cs_gpio
*/
/*
UART2_TX_DATA        	SPI3_CS0   		MX6_PAD_UART2_TX_DATA__ECSPI3_SS0
UART2_RX_DATA        	SPI3_SCLK		MX6_PAD_UART2_RX_DATA__ECSPI3_SCLK
UART2_RTS             	SPI3_MOSI    	MX6_PAD_UART2_RTS_B__ECSPI3_MISO
UART2_CTS               SPI3_MISO 		MX6_PAD_UART2_CTS_B__ECSPI3_MOSI
*/      
#define ST7789_SPI_BUS  2   //第几个SPI  从1到4   需要修改相关函数 board_spi_cs_gpio
#define ST7789_SPI_CS  0   //第几个 CS 0
static int SPI_9608_WR_CMD(int data)
{
	unsigned char  buf[2]; /* write cmd + 7 registers */
	int ret;
	data = data;
	buf[0] =  (unsigned char)data|0xff;
	buf[1] = 0;
	if (!st_slave) {
		st_slave = lcd_spi_setup_slave(ST7789_SPI_BUS,
					ST7789_SPI_CS, 1000000,
					SPI_MODE_3);
		printf("spi3 base register %x\n",(int)st_slave);
		if (!st_slave)
			return -1;
	}
	ret = lcd_spi_xfer(st_slave, 64, buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
	lcd_spi_release_bus(st_slave);
	return ret;
} 
static int SPI_9608_WR_PAR(int data)
{
	unsigned char  buf[8]; /* write cmd + 7 registers */
	int ret;
	buf[0] = (unsigned char)data|0xff;
	buf[1] = 0;
	if (!st_slave) {
		st_slave = lcd_spi_setup_slave(ST7789_SPI_BUS,
					ST7789_SPI_CS, 1000000,
					SPI_MODE_3);
		if (!st_slave)
			return -1;
	}
	ret = lcd_spi_xfer(st_slave, 9, buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
	lcd_spi_release_bus(st_slave);
	return ret;
} 

void init_st7789_on_spi(void)
{
	// VCI=2.8V
	/////////////////// RGB18_ST7789+2.4CTC_2016-12-15 ///////////////////
   	/*****************************************	
	*********  MCU initialize code  *********	
	*****************************************/
	//ST7789+2.4CTC
	SPI_9608_WR_CMD(0x11); 
	mdelay(120);                
	//Delay 120ms                                      //Delay 120ms 
	//--------------------------------------Display Setting---------------------------------------// 
	SPI_9608_WR_CMD(0x36); 
	SPI_9608_WR_PAR(0x00); 

	SPI_9608_WR_CMD(0x3a); 
	SPI_9608_WR_PAR(0x06); 
	//--------------------------------ST7789S ----------------------------------// 
	//RGBCTRL (B1h): RGB Interface Control
	SPI_9608_WR_CMD(0xb1); 
	SPI_9608_WR_PAR(0x42); 
	SPI_9608_WR_PAR(0x18); 
	SPI_9608_WR_PAR(0x1a); 
	//--------------------------------ST7789S ----------------------------------//
	//RAMCTRL (B0h): RAM Control
	SPI_9608_WR_CMD(0xb0); 
	SPI_9608_WR_PAR(0x11); 
	SPI_9608_WR_PAR(0xf0); 
	//--------------------------------ST7789S Frame rate setting----------------------------------// 
	SPI_9608_WR_CMD(0xb2); 
	SPI_9608_WR_PAR(0x0c); 
	SPI_9608_WR_PAR(0x0c); 
	SPI_9608_WR_PAR(0x00); 
	SPI_9608_WR_PAR(0x33); 
	SPI_9608_WR_PAR(0x33); 
	
	SPI_9608_WR_CMD(0xb7); 
	SPI_9608_WR_PAR(0x35); 
	//---------------------------------ST7789S Power setting--------------------------------------// 
	SPI_9608_WR_CMD(0xbb); 
	SPI_9608_WR_PAR(0x35); 
	
	SPI_9608_WR_CMD(0xc0); 
	SPI_9608_WR_PAR(0x2c); 
	
	SPI_9608_WR_CMD(0xc2); 
	SPI_9608_WR_PAR(0x01);
	 
	SPI_9608_WR_CMD(0xc3); 
	SPI_9608_WR_PAR(0x11); 
	
	SPI_9608_WR_CMD(0xc4); 
	SPI_9608_WR_PAR(0x20); 
	
	SPI_9608_WR_CMD(0xc6); 
	SPI_9608_WR_PAR(0x0f); 
	
	SPI_9608_WR_CMD(0xd0); 
	SPI_9608_WR_PAR(0xa4); 
	SPI_9608_WR_PAR(0xa1); 
	//--------------------------------ST7789S gamma setting---------------------------------------// 
	SPI_9608_WR_CMD(0xe0); 
	SPI_9608_WR_PAR(0xd0); 
	SPI_9608_WR_PAR(0x00); 
	SPI_9608_WR_PAR(0x05); 
	SPI_9608_WR_PAR(0x0e); 
	SPI_9608_WR_PAR(0x15); 
	SPI_9608_WR_PAR(0x0d); 
	SPI_9608_WR_PAR(0x37); 
	SPI_9608_WR_PAR(0x43); 
	SPI_9608_WR_PAR(0x47); 
	SPI_9608_WR_PAR(0x09); 
	SPI_9608_WR_PAR(0x15); 
	SPI_9608_WR_PAR(0x12); 
	SPI_9608_WR_PAR(0x16); 
	SPI_9608_WR_PAR(0x19); 
	
	SPI_9608_WR_CMD(0xe1); 
	SPI_9608_WR_PAR(0xd0); 
	SPI_9608_WR_PAR(0x00); 
	SPI_9608_WR_PAR(0x05); 
	SPI_9608_WR_PAR(0x0d); 
	SPI_9608_WR_PAR(0x0c); 
	SPI_9608_WR_PAR(0x06); 
	SPI_9608_WR_PAR(0x2d); 
	SPI_9608_WR_PAR(0x44); 
	SPI_9608_WR_PAR(0x40); 
	SPI_9608_WR_PAR(0x0e); 
	SPI_9608_WR_PAR(0x1c); 
	SPI_9608_WR_PAR(0x18); 
	SPI_9608_WR_PAR(0x16); 
	SPI_9608_WR_PAR(0x19);

	SPI_9608_WR_CMD(0x29);
	mdelay(20);	
}
