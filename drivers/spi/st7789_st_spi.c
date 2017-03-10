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
#include <sandisk_log8.h> //sandisk_log
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
extern void set_reset_st7789(char set);
#define	SPI_DELAY	udelay(1)
#define	SPI_SDA(val)	set_soft_spi_sda(val)
#define	SPI_SCL(val)	set_soft_spi_clk(val)
#define	SPI_READ	get_sda_io()
#define SPI_RESET_PIN(val)  set_reset_st7789(val)
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
int spi_cs_is_valid(unsigned int bus, unsigned int cs)
{

		return 1;
}
/*
static inline struct mxc_spi_slave *to_mxc_spi_slave(struct spi_slave *slave)
{
	return container_of(slave, struct mxc_spi_slave, slave);
}
*/
extern int gpio_direction_output(unsigned gpio, int value);
void spi_cs_activate(struct spi_slave *slave)
{
	set_soft_spi_ss(0);
}

void spi_cs_deactivate(struct spi_slave *slave)
{
       set_soft_spi_ss(1);
}
/*
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

//	 TODO: Use max_hz to limit the SCK rate 

	return &ss->slave;
}*/

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
	int		cpol =  1;
	int		cpha =  SPI_CPHA;
	unsigned int	j;

	PRINTD("spi_xfer: slave %u:%u dout %08X din %08X bitlen %u\n",
		slave->bus, slave->cs, *(uint *)txd, *(uint *)rxd, bitlen);

//	if (flags & SPI_XFER_BEGIN)
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

//	if (flags & SPI_XFER_END)
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
static unsigned char  s_buf[128];
static int SPI_9608_WR_CMD(int data)
{

	int ret;
	unsigned char readbuf[128];
	data = data;
	s_buf[0] =  (unsigned char)((data>>1)&0x3f);
	s_buf[1] = (data&0x1)<<7;

	/*
	if (!st_slave) {
		st_slave = lcd_spi_setup_slave(ST7789_SPI_BUS,
					ST7789_SPI_CS, 1000000,
					SPI_MODE_3);
		printf("spi3 base register %x\n",(int)st_slave);
		if (!st_slave)
			return -1;
	}
	*/
	ret = lcd_spi_xfer(st_slave, 9, s_buf, readbuf, SPI_XFER_BEGIN | SPI_XFER_END);
	lcd_spi_release_bus(st_slave);
	
	return ret;
} 


static int SPI_9608_read(int data)
{

	int ret;
	unsigned char readbuf[128];


	s_buf[0] =  (unsigned char)((data>>1)&0x3f);
	s_buf[1] = (data&0x1)<<7;
	s_buf[2] = 0xff;
	s_buf[3] = 0xff;
	s_buf[4] = 0xff;
	s_buf[5] = 0xff;
	s_buf[6] = 0xff;
	s_buf[7] = 0xff;
	/*
	if (!st_slave) {
		st_slave = lcd_spi_setup_slave(ST7789_SPI_BUS,
					ST7789_SPI_CS, 1000000,
					SPI_MODE_3);
		printf("spi3 base register %x\n",(int)st_slave);
		if (!st_slave)
			return -1;
	}
	*/
	ret = lcd_spi_xfer(st_slave, 64, s_buf, readbuf, SPI_XFER_BEGIN | SPI_XFER_END);

	printf("readbuf[0] =%x\n",readbuf[0]);
	printf("readbuf[1] =%x\n",readbuf[1]);
	printf("readbuf[2] =%x\n",readbuf[2]);
	printf("readbuf[3] =%x\n",readbuf[3]);
	printf("readbuf[4] =%x\n",readbuf[4]);
	lcd_spi_release_bus(st_slave);
	ret = ((int)readbuf[0]<<24)|((int)readbuf[1]<<16)|((int)readbuf[2]<<8)|((int)readbuf[3]);
	ret <<= 1;
	printf("ret = 0x%x  dec = %d\n",ret,ret);
	return ret;
} 
static int SPI_9608_WR_PAR(int data)
{

	int ret;
	s_buf[0] = (unsigned char)((data>>1)|0x80);
	s_buf[1] = (data&0x1)<<7;

	/*
	if (!st_slave) {
		st_slave = lcd_spi_setup_slave(ST7789_SPI_BUS,
					ST7789_SPI_CS, 1000000,
					SPI_MODE_3);
		if (!st_slave)
			return -1;
	}*/
	ret = lcd_spi_xfer(st_slave, 9, s_buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
	lcd_spi_release_bus(st_slave);
	return ret;
} 
void test_spi_format(void)
{
	s_buf[2] = 0xff;
	s_buf[3] = 0xff;
	s_buf[4] = 0xff;
	s_buf[5] = 0xff;
	s_buf[6] = 0xff;
	s_buf[7] = 0xff;
	s_buf[8] = 0xff;
	s_buf[9] = 0xff;
#if 1
	while(1)
	{
	SPI_RESET_PIN(0);
	mdelay(100);
	SPI_RESET_PIN(1);
	mdelay(100);
	printf("reset cmd11\n");
	SPI_9608_WR_CMD(0x01);
	mdelay(10);
	SPI_9608_WR_CMD(0x11);
	mdelay(500);
	printf("cmd sleep out\n");
	 
	printf("SPI_9608_read ID =%x\n");
	SPI_9608_read(0x04);
	printf("++++++++++++++++++++++++++++\n");
	printf("read display display status\n");
	//page 167
	SPI_9608_WR_CMD(0X28);//DISON
	SPI_9608_read(0X09);

	SPI_9608_WR_CMD(0X29);
	SPI_9608_read(0X09);
	
	SPI_9608_WR_CMD(0X35);//TEON
	SPI_9608_read(0X09);

	SPI_9608_WR_CMD(0X34);
	SPI_9608_read(0X09);
	}
#endif
}
void test_display_off(void)
{
	int i = 0;
	SPI_RESET_PIN(0);
	mdelay(100);
	SPI_RESET_PIN(1);
	mdelay(100);
	printf("reset cmd11\n");
	SPI_9608_WR_CMD(0x01);
	mdelay(10);
	SPI_9608_WR_CMD(0x11);
	mdelay(500);
	printf("SPI_9608_read ID =%x\n");
	SPI_9608_read(0x04);
	
	
	printf("++++++++++++++++++++++++++++\n");
	printf("read display display status\n");
	//page 167
	while(1)
	{
	SPI_9608_WR_CMD(0X28);//DISON
	SPI_9608_read(0X09);
	mdelay(500);
	SPI_9608_WR_CMD(0X29);
	SPI_9608_read(0X09);
	mdelay(500);		
	i++;
	if(i>2)
		return;
	}

}
#define send_ctrl_cmd(val) SPI_9608_WR_CMD(val)
#define send_data_cmd(val) SPI_9608_WR_PAR(val)


static void draw_sandisk_log(void)
{
   unsigned short x0, y0, x1, y1, x, y;
   unsigned short h_X_start,l_X_start,h_X_end,l_X_end,h_Y_start,l_Y_start,h_Y_end,l_Y_end;
   unsigned int cnt = 0;
   unsigned char cl = 0;
   x0 = (unsigned short)0;
   y0 = (unsigned short)0;
   x1 = (unsigned short)239;
   y1 = (unsigned short)319;
   
   h_X_start=((x0&0xFF00)>>8);
   l_X_start=(x0&0x00FF);
   h_X_end=(((int)x1&0xFF00)>>8);
   l_X_end=(x1&0x00FF);
   
   h_Y_start=((y0&0xFF00)>>8);
   l_Y_start=(y0&0x00FF);
   h_Y_end=(((int)y1&0xFF00)>>8);
   l_Y_end=(y1&0x00FF);
   
   
   send_ctrl_cmd(0x2A);
   send_data_cmd(h_X_start); 
   send_data_cmd(l_X_start); 
   send_data_cmd(h_X_end); 
   send_data_cmd(l_X_end); 
   
   send_ctrl_cmd(0x2B);
   send_data_cmd(h_Y_start); 
   send_data_cmd(l_Y_start); 
   send_data_cmd(h_Y_end); 
   send_data_cmd(l_Y_end); 

   send_ctrl_cmd(0x29);
   send_ctrl_cmd(0x2C); 
   
 //  send_ctrl_cmd(0x37);
 //  send_data_cmd(0);
 //  send_data_cmd(cl++);

   
   
   
   printf("cl = %d\n",cl);
   printf("clear red colour\n");
	printf("h_X_start =%x\n l_X_start=%x \n h_X_end =%x \nl_X_end =%x\n",
	h_X_start, l_X_start, h_X_end, l_X_end);
	printf("h_Y_start =%x\n l_Y_start=%x \n h_Y_end =%x \n l_Y_end =%x\n",
	h_Y_start, l_Y_start, h_Y_end, l_Y_end);
	x = 0;
	y = 0;//sandisk_log
/*
   for (y = y0; y <= y1; ++ y) {
      for (x = x0; x <= x1; ++ x) {
		send_data_cmd(sandisk_log[cnt++]);
		send_data_cmd(sandisk_log[cnt++]);
		send_data_cmd(sandisk_log[cnt++]);
      }
   }
*/
	for (y = 0; y <= y1; ++ y) {
      for (x = x0; x <= x1; ++ x) {
		send_data_cmd(sandisk_log[cnt++]);
		send_data_cmd(sandisk_log[cnt++]);
		send_data_cmd(sandisk_log[cnt++]);
      }
   }
   printf("draw finished!   cnt = %d\n",cnt);
   
	
}

static void sw_clear_panel(unsigned int color)
{
   unsigned short x0, y0, x1, y1, x, y;
   unsigned short h_X_start,l_X_start,h_X_end,l_X_end,h_Y_start,l_Y_start,h_Y_end,l_Y_end;
   
   x0 = (unsigned short)0;
   y0 = (unsigned short)0;
   x1 = (unsigned short)239;
   y1 = (unsigned short)319;
   
   h_X_start=((x0&0xFF00)>>8);
   l_X_start=(x0&0x00FF);
   h_X_end=(((int)x1&0xFF00)>>8);
   l_X_end=(x1&0x00FF);
   
   h_Y_start=((y0&0xFF00)>>8);
   l_Y_start=(y0&0x00FF);
   h_Y_end=(((int)y1&0xFF00)>>8);
   l_Y_end=(y1&0x00FF);
   
   
   send_ctrl_cmd(0x2A);
   send_data_cmd(h_X_start); 
   send_data_cmd(l_X_start); 
   send_data_cmd(h_X_end); 
   send_data_cmd(l_X_end); 
   
   send_ctrl_cmd(0x2B);
   send_data_cmd(h_Y_start); 
   send_data_cmd(l_Y_start); 
   send_data_cmd(h_Y_end); 
   send_data_cmd(l_Y_end); 

   send_ctrl_cmd(0x29);
   send_ctrl_cmd(0x2C); 
   

   
   
   
   
   printf("clear red colour\n");
	printf("h_X_start =%x\n l_X_start=%x \n h_X_end =%x \nl_X_end =%x\n",
	h_X_start, l_X_start, h_X_end, l_X_end);
	printf("h_Y_start =%x\n l_Y_start=%x \n h_Y_end =%x \n l_Y_end =%x\n",
	h_Y_start, l_Y_start, h_Y_end, l_Y_end);
	x = 0;
	y = 0;
   for (y = y0; y <= y1; ++ y) {
      for (x = x0; x <= x1; ++ x) {
		send_data_cmd(0xfc);
		send_data_cmd(0);
		send_data_cmd(0);
      }
   }
   
	printf("clear green colour\n");
	printf("h_X_start =%x\n l_X_start=%x \n h_X_end =%x \nl_X_end =%x\n",
	h_X_start, l_X_start, h_X_end, l_X_end);
	printf("h_Y_start =%x\n l_Y_start=%x \n h_Y_end =%x \n l_Y_end =%x\n",
	h_Y_start, l_Y_start, h_Y_end, l_Y_end);
	printf("l_Y_start + 50\n");
	send_data_cmd(l_Y_start + 50); 	
	  x = 0;
	  y = 0;

   for (y = y0; y <= y1; ++ y) {
      for (x = x0; x <= x1; ++ x) {
		send_data_cmd(0);
		send_data_cmd(0xfc);
		send_data_cmd(0x0);
      }
   }
   
	x = 0;
	y = 0;
  printf("clear blue colour\n");
  printf("h_X_start =%x\n l_X_start=%x \n h_X_end =%x \nl_X_end =%x\n",
	h_X_start, l_X_start, h_X_end, l_X_end);
	printf("h_Y_start =%x\n l_Y_start=%x \n h_Y_end =%x \n l_Y_end =%x\n",
	h_Y_start, l_Y_start, h_Y_end, l_Y_end);
	printf("l_Y_start + 100\n");
	send_data_cmd(l_Y_start + 100); 
   for (y = y0; y <= y1; ++ y) {
      for (x = x0; x <= x1; ++ x) {
		send_data_cmd(0);
		send_data_cmd(0);
		send_data_cmd(0xfc);
      }
   }
}


static void init_lcm_registers(void)
	{
		SPI_RESET_PIN(0);
		mdelay(100);
		SPI_RESET_PIN(1);
		mdelay(100);
		send_ctrl_cmd(0x11);
		mdelay(120);  
				
		send_ctrl_cmd(0x36);
		send_data_cmd(0x00);//40
		
		send_ctrl_cmd(0x3a);
		send_data_cmd(0x06);//06
		
		send_ctrl_cmd(0xb2);
		send_data_cmd(0x28);
		send_data_cmd(0x28);
		send_data_cmd(0x05);
		send_data_cmd(0x33);
		send_data_cmd(0x33);
		
		
		
		send_ctrl_cmd(0xb7);
		send_data_cmd(0x35);
		
		send_ctrl_cmd(0xbb);
		send_data_cmd(0x3c);//23
		
//		send_ctrl_cmd(0xb1);
//		send_data_cmd(0x80);
//		send_data_cmd(0x10);
		
		
		send_ctrl_cmd(0xc0);
		send_data_cmd(0x2c);
		
		send_ctrl_cmd(0xc2);
		send_data_cmd(0x01);
		
		send_ctrl_cmd(0xc3);
		send_data_cmd(0x05);//14
		
		send_ctrl_cmd(0xc4);
		send_data_cmd(0x20);
		
		send_ctrl_cmd(0xc6);
		send_data_cmd(0x14); // 14
		
		send_ctrl_cmd(0xd0);
		send_data_cmd(0xa4);
		send_data_cmd(0xa1);
		
		send_ctrl_cmd(0xe0);
		send_data_cmd(0xd0);
		send_data_cmd(0x00);
		send_data_cmd(0x02);
		send_data_cmd(0x07);
		send_data_cmd(0x07);
		send_data_cmd(0x19);
		send_data_cmd(0x2e);
		send_data_cmd(0x54);
		send_data_cmd(0x41);
		send_data_cmd(0x2d);
		send_data_cmd(0x17);
		send_data_cmd(0x18);
		send_data_cmd(0x14);
		send_data_cmd(0x18);
		
		send_ctrl_cmd(0xe1);
		send_data_cmd(0xd0);
		send_data_cmd(0x00);
		send_data_cmd(0x02);
		send_data_cmd(0x07);
		send_data_cmd(0x04);
		send_data_cmd(0x24);
		send_data_cmd(0x2c);
		send_data_cmd(0x44);
		send_data_cmd(0x42);
		send_data_cmd(0x1c);
		send_data_cmd(0x1a);
		send_data_cmd(0x17);
		send_data_cmd(0x15);
        send_data_cmd(0x18);
		send_ctrl_cmd(0x35);
		send_data_cmd(0x00);//40
		
//		send_ctrl_cmd(0x44);
//		send_data_cmd(0x19);
		
		send_ctrl_cmd(0x2a);
		send_data_cmd(0x00);
		send_data_cmd(0x00);
		send_data_cmd(0x00);
		send_data_cmd(0xef);
		
		send_ctrl_cmd(0x2b);
		send_data_cmd(0x00);
		send_data_cmd(0x00);
		send_data_cmd(0x01);
		send_data_cmd(0x3f);
		
		send_ctrl_cmd(0x29);		

		sw_clear_panel(0x0);	

	}

void init_st7789_on_spi(void)
{
	// VCI=2.8V
	/////////////////// RGB18_ST7789+2.4CTC_2016-12-15 ///////////////////
   	/*****************************************	
	*********  MCU initialize code  *********	
	*****************************************/
	//ST7789+2.4CTC

	s_buf[2] = 0xff;
	s_buf[3] = 0xff;
	s_buf[4] = 0xff;
	s_buf[5] = 0xff;
	s_buf[6] = 0xff;
	s_buf[7] = 0xff;
	s_buf[8] = 0xff;
	s_buf[9] = 0xff;

	SPI_RESET_PIN(0);
	mdelay(100);
	SPI_RESET_PIN(1);
	mdelay(100);
	printf("reset cmd555555555555551\n");
#if 0
	SPI_9608_WR_CMD(0x01);
	test_display_off();
	mdelay(10);


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
	SPI_9608_WR_PAR(0x42);//42
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
	SPI_9608_WR_CMD(0x2c);
	init_lcm_registers();
	
	printf("draw sandisk log \n");
	#endif
	while(1)
	{
		printf("clefffar\n");
	//	sw_clear_panel(0xf0);
	send_ctrl_cmd(0x11);
	mdelay(120);
	send_ctrl_cmd(0x29);
	
	sw_clear_panel(0);
	draw_sandisk_log();
	}
	
}
