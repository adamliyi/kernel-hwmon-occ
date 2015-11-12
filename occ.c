/*
 * Open BMC OCC HWMON driver - read Power8 OCC (On Chip Controller) sensor data via i2c.
 *
 * Copyright (c) 2015 IBM (Alvin Wang, Li Yi) 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/delay.h>

//#define DEBUG    1

/* ------------------------------------------------------------*/
/* OCC sensor data format */
typedef struct {
	uint16_t sensor_id;
	uint16_t value;
} occ_sensor;

typedef struct {
	uint16_t sensor_id;
	uint32_t update_tag;
	uint32_t accumulator;
	uint16_t value;
} powr_sensor;

typedef struct {
	uint16_t curr_powercap;
	uint16_t curr_powerreading;
	uint16_t norm_powercap;
	uint16_t max_powercap;
	uint16_t min_powercap;
	uint16_t user_powerlimit;
} caps_sensor;

typedef struct {
	char sensor_type[5];
	uint8_t reserved0;
	uint8_t sensor_format;
	uint8_t sensor_length;
	uint8_t num_of_sensors;
	occ_sensor *sensor;
	powr_sensor *powr;
	caps_sensor *caps;
} sensor_data_block;

typedef struct {
	uint8_t status;
	uint8_t ext_status;
	uint8_t occs_present;
	uint8_t config;
	uint8_t occ_state;
	uint8_t reserved0;
	uint8_t reserved1;
	uint8_t error_log_id;
	uint32_t error_log_addr_start;
	uint16_t error_log_length;
	uint8_t reserved2;
	uint8_t reserved3;
	char occ_code_level[17];
	char sensor_eye_catcher[7];
	uint8_t num_of_sensor_blocks;
	uint8_t sensor_data_version;
	sensor_data_block* blocks;
} occ_poll_data;

typedef struct {
	uint8_t sequence_num;
	uint8_t cmd_type;
	uint8_t rtn_status;
	uint16_t data_length;
	occ_poll_data data;
	uint16_t chk_sum;
	uint16_t temp_block_id;	
	uint16_t freq_block_id;	
	uint16_t power_block_id;	
	uint16_t caps_block_id;	
} occ_response_t;

static occ_response_t occ_resp;

/* Each client has this additional data */
struct occ_drv_data {
	struct i2c_client	*client;
	struct device		*hwmon_dev;
	struct mutex		update_lock;
	char			valid;		/* !=0 if sensor data are valid */
	unsigned long		last_updated;	/* In jiffies */
	unsigned long		sample_time;	/* Mininum timer interval for sampling In jiffies */
	occ_response_t		*occ_resp;
};

/*-----------------------------------------------------------------------*/
/* i2c read and write occ sensors */

#define OCC_DATA_MAX 4096 /* 4KB at most */
#define I2C_STATUS_REG 0x000d0001
#define I2C_ERROR_REG  0x000d0002
#define I2C_READ_ERROR 1
#define I2C_WRITE_ERROR 2
#define I2C_DATABUFFER_SIZE_ERROR 3

/*
#define SCOM_OCC_SRAM_WOX  0x0006B013
#define SCOM_OCC_SRAM_WAND 0x0006B012
#define SCOM_OCC_SRAM_ADDR 0x0006B010
#define SCOM_OCC_SRAM_DATA 0x0006B015
*/

// To generate attn to OCC
#define ATTN_DATA                0x0006B035

// For BMC to read/write SRAM
#define OCB_ADDRESS              0x0006B070
#define OCB_DATA                 0x0006B075
#define OCB_STATUS_CONTROL_AND   0x0006B072
#define OCB_STATUS_CONTROL_OR    0x0006B073

#define OCC_COMMAND_ADDR 0xFFFF6000
#define OCC_RESPONSE_ADDR 0xFFFF7000

static int deinit_occ_resp_buf(occ_response_t *p)
{
	int b;

	if (p == NULL)
		return 0;

	if (p->data.blocks == NULL)
		return 0;

	for(b = 0; b < p->data.num_of_sensor_blocks; b++) {
		if (!p->data.blocks[b].sensor)	
			kfree(p->data.blocks[b].sensor);
		if (!p->data.blocks[b].powr)	
			kfree(p->data.blocks[b].powr);
		if (!p->data.blocks[b].caps)	
			kfree(p->data.blocks[b].caps);
	}

	kfree(p->data.blocks);
	
	memset(p, 0, sizeof(*p));

	return 0;
}

static ssize_t occ_i2c_read(struct i2c_client *client, char *buf, size_t count)
{
	int ret = 0;

	if (count > 8192)
		count = 8192;

	//printk("i2c_read: reading %zu bytes @0x%x.\n", count, client->addr);
	ret = i2c_master_recv(client, buf, count);
	return ret;
}

static ssize_t occ_i2c_write(struct i2c_client *client, const char *buf, size_t count)
{
	int ret = 0;

	if (count > 8192)
		count = 8192;

	//printk("i2c_write: writing %zu bytes @0x%x.\n", count, client->addr);
	ret = i2c_master_send(client, buf, count);
	return ret;
}

/* read two 4-byte value */
static int occ_getscom(struct i2c_client *client, uint32_t address, uint32_t *value0, uint32_t *value1)
{
	uint32_t ret = 0;
	char buf[8];
  	const char* address_buf = (const char*)&address;
	
	//P8 i2c slave requires address to be shifted by 1
	address = address << 1;
	
	ret = occ_i2c_write(client, address_buf, sizeof(address));
	/* FIXME: ast i2c driver does not read corret value */
	//if (ret != sizeof(address))
	//	return -I2C_WRITE_ERROR;
	
	ret = occ_i2c_read(client, buf, sizeof(buf));
	//if (ret != sizeof(buf))
	//	return -I2C_READ_ERROR;
	
	memcpy(value1, &buf[0], sizeof(*value1));
	memcpy(value0, &buf[4], sizeof(*value0));
	
	return 0;
}

/* read 8-byte value and put into data[offset] */
static int occ_getscomb(struct i2c_client *client, uint32_t address, char* data, int offset)
{
	uint32_t ret = 0;
	const char* address_buf = (const char*)&address;
	char buf[8];
	int b = 0;
  	
	//P8 i2c slave requires address to be shifted by 1
	address = address << 1;
	
	ret = occ_i2c_write(client, address_buf, sizeof(address));
	//if (ret != sizeof(address))
	//	return -I2C_WRITE_ERROR;
  
	ret = occ_i2c_read(client, buf, sizeof(buf));
	//if (ret != sizeof(buf))
	//	return -I2C_READ_ERROR;

  	for (b = 0; b < 8; b++) {
    		data[offset + b] = buf[7 - b];
  	}
  	
	return 0;
}

static int occ_putscom(struct i2c_client *client, uint32_t address, uint32_t data0, uint32_t data1)
{
	const char* address_buf = (const char*)&address;
	const char* d0 = (const char*)&data0;
	const char* d1 = (const char*)&data1;
	char buf[12];
	uint32_t ret = 0;

	//P8 i2c slave requires address to be shifted by 1
	address = address << 1;
	
	memcpy(&buf[0], address_buf, sizeof(address));
	memcpy(&buf[4], d1, sizeof(data1));
	memcpy(&buf[8],	d0, sizeof(data0));
       
	ret = occ_i2c_write(client, buf, sizeof(buf));
	//if (ret != sizeof(buf))
	//	return I2C_WRITE_ERROR;

	return 0;
}

static int occ_check_i2c_errors(struct i2c_client *client)
{
	uint32_t v0;
	uint32_t v1;
	
	occ_getscom(client, I2C_STATUS_REG, &v0, &v1);
	if (v0 != 0x80000000) {
    		printk("ERROR present in P8 I2C Slave.  Clearing...\n");
		occ_putscom(client, I2C_ERROR_REG, 0x00000000, 0x00000000);
		occ_putscom(client, I2C_STATUS_REG, 0x00000000, 0x00000000);
		return -1;
	}
	
	return 0;
}


static inline uint16_t get_occdata_length(char* d)
{
	uint16_t data_length = 0;
	
	data_length = d[3] << 8;
	data_length = data_length | d[4];
	return data_length;
}


static int parse_occ_response(char* d, occ_response_t* o)
{
	int b = 0;
	int s = 0;
	int ret = 0;
	int dnum = 45;
	
	o->sequence_num = d[0];
	o->cmd_type = d[1];
	o->rtn_status = d[2];
	o->data_length = d[3] << 8;
	o->data_length = o->data_length | d[4];
	o->data.status = d[5];
	o->data.ext_status = d[6];
	o->data.occs_present = d[7];
	o->data.config = d[8];
	o->data.occ_state = d[9];
	o->data.reserved0 = d[10];
	o->data.reserved1 = d[11];
	o->data.error_log_id = d[12];
	o->data.error_log_addr_start = d[13] << 24;
	o->data.error_log_addr_start = o->data.error_log_addr_start | d[14] << 16;
	o->data.error_log_addr_start = o->data.error_log_addr_start | d[15] << 8;
	o->data.error_log_addr_start = o->data.error_log_addr_start | d[16];
	o->data.error_log_length = d[17] << 8;
	o->data.error_log_length = o->data.error_log_length | d[18];
	o->data.reserved2 = d[19];
	o->data.reserved3 = d[20];
	strncpy(&o->data.occ_code_level[0], (const char*)&d[21], 16);
	strncpy(&o->data.sensor_eye_catcher[0], (const char*)&d[37], 6);
	o->data.sensor_eye_catcher[6]='\0';
	o->data.num_of_sensor_blocks=d[43];
	o->data.sensor_data_version = d[44];
	
	if (strcmp(o->data.sensor_eye_catcher, "SENSOR") != 0) {
		printk("ERROR: SENSOR not found at byte 37 (%s)\n",o->data.sensor_eye_catcher);
		return -1;
	}

	if (o->data.num_of_sensor_blocks == 0) {
		printk("ERROR: SENSOR block num is 0\n");
		return -1;
	}

	o->data.blocks = kzalloc(sizeof(sensor_data_block) * o->data.num_of_sensor_blocks, GFP_KERNEL);
	if (o->data.blocks == NULL)
		return -ENOMEM;
  	
	//printk("Reading %d sensor blocks\n", o->data.num_of_sensor_blocks);
	for(b = 0; b < o->data.num_of_sensor_blocks; b++) {
		/* 8-byte sensor block head */
		strncpy(&o->data.blocks[b].sensor_type[0], (const char*)&d[dnum], 4);
		o->data.blocks[b].reserved0 = d[dnum+4];
		o->data.blocks[b].sensor_format = d[dnum+5];
		o->data.blocks[b].sensor_length = d[dnum+6];
		o->data.blocks[b].num_of_sensors = d[dnum+7];
		dnum = dnum + 8;
		
		//printk("sensor block[%d]: type: %s, num_of_sensors: %d, sensor_length: %u\n",
			//b, o->data.blocks[b].sensor_type, o->data.blocks[b].num_of_sensors,
			//o->data.blocks[b].sensor_length);
	
		/* empty sensor block */	
		if (o->data.blocks[b].num_of_sensors <= 0)
			continue;
		if (o->data.blocks[b].sensor_length == 0)
			continue;
		
		if (strcmp(o->data.blocks[b].sensor_type, "FREQ") == 0) {	
			o->data.blocks[b].sensor = 
				kzalloc(sizeof(occ_sensor) * o->data.blocks[b].num_of_sensors, GFP_KERNEL);
			
			if (o->data.blocks[b].sensor == NULL) {
				ret = -ENOMEM;
				goto abort;
			}
			o->freq_block_id = b;
			for (s = 0; s < o->data.blocks[b].num_of_sensors; s++) {
				o->data.blocks[b].sensor[s].sensor_id = d[dnum] << 8;
        			o->data.blocks[b].sensor[s].sensor_id = 
					o->data.blocks[b].sensor[s].sensor_id | d[dnum+1];
				o->data.blocks[b].sensor[s].value = d[dnum+2] << 8;
				o->data.blocks[b].sensor[s].value = o->data.blocks[b].sensor[s].value | d[dnum+3];
				//printk("sensor[%d]-[%d]: id: %u, value: %u\n",
				//	b, s, o->data.blocks[b].sensor[s].sensor_id, o->data.blocks[b].sensor[s].value);
				dnum = dnum + o->data.blocks[b].sensor_length;
			}
		}
		else if (strcmp(o->data.blocks[b].sensor_type, "TEMP") == 0) {
				
			o->data.blocks[b].sensor = 
				kzalloc(sizeof(occ_sensor) * o->data.blocks[b].num_of_sensors, GFP_KERNEL);
			
			if (o->data.blocks[b].sensor == NULL) {
				ret = -ENOMEM;
				goto abort;
			}
			
			o->temp_block_id = b;
			for (s = 0; s < o->data.blocks[b].num_of_sensors; s++) {
				o->data.blocks[b].sensor[s].sensor_id = d[dnum] << 8;
        			o->data.blocks[b].sensor[s].sensor_id = 
					o->data.blocks[b].sensor[s].sensor_id | d[dnum+1];
				o->data.blocks[b].sensor[s].value = d[dnum+2] << 8;
				o->data.blocks[b].sensor[s].value = o->data.blocks[b].sensor[s].value | d[dnum+3];
				//printk("sensor[%d]-[%d]: id: %u, value: %u\n",
				//	b, s, o->data.blocks[b].sensor[s].sensor_id, o->data.blocks[b].sensor[s].value);
				dnum = dnum + o->data.blocks[b].sensor_length;
			}
		}
		else if (strcmp(o->data.blocks[b].sensor_type, "POWR") == 0) {
			
			o->data.blocks[b].powr =
				kzalloc(sizeof(powr_sensor) * o->data.blocks[b].num_of_sensors, GFP_KERNEL);
			
			if (o->data.blocks[b].powr == NULL) {
				ret = -ENOMEM;
				goto abort;
			}
			o->power_block_id = b;
			for (s = 0; s < o->data.blocks[b].num_of_sensors; s++) {
				o->data.blocks[b].powr[s].sensor_id = d[dnum] << 8;
				o->data.blocks[b].powr[s].sensor_id = o->data.blocks[b].powr[s].sensor_id | d[dnum+1];
				o->data.blocks[b].powr[s].update_tag = d[dnum+2] << 24;
				o->data.blocks[b].powr[s].update_tag = o->data.blocks[b].powr[s].update_tag | d[dnum+3] << 16;
				o->data.blocks[b].powr[s].update_tag = o->data.blocks[b].powr[s].update_tag | d[dnum+4] << 8;
				o->data.blocks[b].powr[s].update_tag = o->data.blocks[b].powr[s].update_tag | d[dnum+5];
				o->data.blocks[b].powr[s].accumulator = d[dnum+6] << 24;
				o->data.blocks[b].powr[s].accumulator = o->data.blocks[b].powr[s].accumulator | d[dnum+7] << 16;
				o->data.blocks[b].powr[s].accumulator = o->data.blocks[b].powr[s].accumulator | d[dnum+8] << 8;
				o->data.blocks[b].powr[s].accumulator = o->data.blocks[b].powr[s].accumulator | d[dnum+9];
				o->data.blocks[b].powr[s].value = d[dnum+10] << 8;
				o->data.blocks[b].powr[s].value = o->data.blocks[b].powr[s].value | d[dnum+11];
				
				//printk("sensor[%d]-[%d]: id: %u, value: %u\n",
				//	b, s, o->data.blocks[b].powr[s].sensor_id, o->data.blocks[b].powr[s].value);
				
				dnum = dnum + o->data.blocks[b].sensor_length;
			}
		}
		else if (strcmp(o->data.blocks[b].sensor_type, "CAPS") == 0) {
			
			o->data.blocks[b].caps =
				kzalloc(sizeof(caps_sensor) * o->data.blocks[b].num_of_sensors, GFP_KERNEL);

                        if (o->data.blocks[b].caps == NULL) {
                                ret = -ENOMEM;
                                goto abort;
                        }
                        o->caps_block_id = b;
			for (s = 0; s < o->data.blocks[b].num_of_sensors; s++) {
				o->data.blocks[b].caps[s].curr_powercap = d[dnum] << 8;
				o->data.blocks[b].caps[s].curr_powercap = o->data.blocks[b].caps[s].curr_powercap | d[dnum+1];
				o->data.blocks[b].caps[s].curr_powerreading = d[dnum+2] << 8;
				o->data.blocks[b].caps[s].curr_powerreading = o->data.blocks[b].caps[s].curr_powerreading | d[dnum+3];
				o->data.blocks[b].caps[s].norm_powercap = d[dnum+4] << 8;
				o->data.blocks[b].caps[s].norm_powercap = o->data.blocks[b].caps[s].norm_powercap | d[dnum+5];
				o->data.blocks[b].caps[s].max_powercap = d[dnum+6] << 8;
				o->data.blocks[b].caps[s].max_powercap = o->data.blocks[b].caps[s].max_powercap| d[dnum+7];
				o->data.blocks[b].caps[s].min_powercap = d[dnum+8] << 8;
				o->data.blocks[b].caps[s].min_powercap = o->data.blocks[b].caps[s].min_powercap| d[dnum+9];
				o->data.blocks[b].caps[s].user_powerlimit = d[dnum+10] << 8;
				o->data.blocks[b].caps[s].user_powerlimit = o->data.blocks[b].caps[s].user_powerlimit| d[dnum+11];

				dnum = dnum + o->data.blocks[b].sensor_length;
				//printk("CAPS sensor #%d:\n", s);
				//printk("curr_powercap is %x \n", o->data.blocks[b].caps[s].curr_powercap);
				//printk("curr_powerreading is %x \n", o->data.blocks[b].caps[s].curr_powerreading);
				//printk("norm_powercap is %x \n", o->data.blocks[b].caps[s].norm_powercap);
				//printk("max_powercap is %x \n", o->data.blocks[b].caps[s].max_powercap);
				//printk("min_powercap is %x \n", o->data.blocks[b].caps[s].min_powercap);
				//printk("user_powerlimit is %x \n", o->data.blocks[b].caps[s].user_powerlimit);
			}
		
		}
		else {
      			printk("ERROR: sensor type %s not supported\n", o->data.blocks[b].sensor_type);
			ret = -1;
			goto abort;
		}
	}

	return 0;
abort:
	deinit_occ_resp_buf(o);
	return ret;  
}

/* used for testing */
char fake_occ_rsp[OCC_DATA_MAX] = {
0x69, 0x00, 0x00, 0x00, 0xa4, 0xc3, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x6f, 0x70, 0x5f, 0x6f, 0x63, 0x63, 0x5f, 0x31, 0x35, 0x30, 0x37,
0x31, 0x36, 0x61, 0x00, 0x00, 0x53, 0x45, 0x4e, 0x53, 0x4f, 0x52, 0x04, 0x01, 0x54, 0x45, 0x4d,    
0x50, 0x00, 0x01, 0x04, 0x0a, 0x00 ,0x6a, 0x00, 0x00, 0x00, 0x6c, 0x00, 0x00, 0x00, 0x6d, 0x00,    
0x00,0x00,0x6e,0x00, 0x00,0x00,0x6f,0x00, 0x00,0x00,0x70,0x00, 0x00,0x00,0x71,0x00,
0x00,0x00,0x73,0x00, 0x00,0x00,0x74,0x00, 0x00,0x00,0x75,0x00, 0x00,0x46,0x52,0x45,    
0x51,0x00,0x01,0x04, 0x0a,0x00,0x76,0x00, 0x00,0x00,0x78,0x00, 0x00,0x00,0x79,0x00,    
0x00,0x00,0x7a,0x00, 0x00,0x00,0x7b,0x00, 0x00,0x00,0x7c,0x00, 0x00,0x00,0x7d,0x00,    
0x00,0x00,0x7f,0x00, 0x00,0x00,0x80,0x00, 0x00,0x00,0x81,0x00, 0x00,0x50,0x4f,0x57,    
0x52,0x00,0x01,0x0c, 0x00,0x43,0x41,0x50, 0x53,0x00,0x01,0x0c, 0x01,0x00,0x00,0x00,    
0x00,0x04,0xb0,0x09, 0x60,0x04,0x4c,0x00, 0x00,0x17,0xc5,}; 

//#define DUMP_RAW 1

static int occ_get_all(struct i2c_client *client, occ_response_t *occ_resp)
{
	char occ_data[OCC_DATA_MAX];
	uint16_t num_bytes = 0;
	int b = 0;
	int ret = 0;
/*
	//Procedure to access SRAM where OCC data is located	
	occ_putscom(client, SCOM_OCC_SRAM_WOX, 0x08000000, 0x00000000);
	occ_putscom(client, SCOM_OCC_SRAM_WAND, 0xFBFFFFFF, 0xFFFFFFFF);
	occ_putscom(client, SCOM_OCC_SRAM_ADDR, OCC_RESPONSE_ADDR, 0x00000000);
	occ_putscom(client, SCOM_OCC_SRAM_ADDR, OCC_RESPONSE_ADDR, 0x00000000);
	
	occ_getscomb(client, SCOM_OCC_SRAM_DATA, occ_data, 0);

*/

	// Init OCB
	occ_putscom(client, OCB_STATUS_CONTROL_OR,  0x08000000, 0x00000000);
	occ_putscom(client, OCB_STATUS_CONTROL_AND, 0xFBFFFFFF, 0xFFFFFFFF);

	// Send poll command to OCC
	occ_putscom(client, OCB_ADDRESS, OCC_COMMAND_ADDR, 0x00000000);
	occ_putscom(client, OCB_ADDRESS, OCC_COMMAND_ADDR, 0x00000000);
	occ_putscom(client, OCB_DATA, 0x00000001, 0x10001100);

	// Trigger ATTN
	occ_putscom(client, ATTN_DATA, 0x01010000, 0x00000000);

    	// TODO: check command status Refere to 
	// "1.6.2 OCC Command/Response Sequence" in OCC_OpenPwr_FW_Interfaces1.2.pdf
    	// Use sleep as workaround
	//msleep(2000);

	// Get response data
	occ_putscom(client, OCB_ADDRESS, OCC_RESPONSE_ADDR, 0x00000000);
	occ_getscomb(client, OCB_DATA, occ_data, 0);

	/* FIXME: use fake data to test driver without hw */
	//printk("i2c-occ: using FAKE occ data\n");
	//memcpy(&occ_data[0], &fake_occ_rsp[0], sizeof(occ_data));
	
	num_bytes = get_occdata_length(occ_data);
	
	//printk("OCC data length: %d\n", num_bytes);

#ifdef DUMP_RAW
	int i = 0;
	printk("\nRAW data\n==================\n");
	for (i = 0; i < 8; i++) {
		if(i == 4) printk("  ");
		printk("%02x", occ_data[i]);
	}
	printk("\n");
#endif
	
	if (num_bytes > OCC_DATA_MAX) {
      		printk("ERROR: OCC data length must be < 4KB\n");
		return -1;
	}
	
	if (num_bytes <= 0) {
      		printk("ERROR: OCC data length is zero\n");
		return -1;
	}
	
	for (b = 8; b < num_bytes + 8; b = b + 8) {
		//occ_getscomb(client, SCOM_OCC_SRAM_DATA, occ_data, b);
		occ_getscomb(client, OCB_DATA, occ_data, b);
#ifdef DUMP_RAW
	for (i = 0; i < 8; i++) {
        	if(i == 4) printk("  ");
		printk("%02x", occ_data[b+i]);
	}
	printk("\n");
#endif

	}
	
	/* FIXME: use fake data to test driver without hw */
	//memcpy(&occ_data[0], &fake_occ_rsp[0], sizeof(occ_data));
	
	ret = parse_occ_response(occ_data, occ_resp);

	return ret;	
}


static int occ_update_device(struct device *dev)
{
	struct occ_drv_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int ret = 0;

	mutex_lock(&data->update_lock);

	if (time_after(jiffies, data->last_updated + data->sample_time)
	    || !data->valid) {
		deinit_occ_resp_buf(data->occ_resp);
		
		ret = occ_get_all(client, data->occ_resp);

		data->last_updated = jiffies;
		data->valid = 1;
	}
	mutex_unlock(&data->update_lock);
	
	return ret;
}

/* ----------------------------------------------------------------------*/
/* sysfs attributes for hwmon */

static ssize_t show_occ_temp_input(struct device *dev, struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int n = attr->index; 	
	struct occ_drv_data *data = dev_get_drvdata(dev);
	int ret = 0;
	occ_sensor *sensor;
	int val = 0;
	
	ret = occ_update_device(dev);

	if (ret != 0)
	{
		/* FIXME: to test fake data */
		printk("ERROR: cannot get occ sensor data: %d\n", ret);
		return ret;
	}
	
	if (data->occ_resp->data.blocks == NULL ||
		data->occ_resp->data.blocks[data->occ_resp->temp_block_id].sensor == NULL)
		return -1;

	//printk("block_id: %d, sensor: %d\n", data->occ_resp->temp_block_id, n -1);	
	sensor = &data->occ_resp->data.blocks[data->occ_resp->temp_block_id].sensor[n - 1];
	val = sensor->value;
	//printk("temp%d sensor value: %d\n", n, val);

	//printk("------------- above are debug message, bellow is real output------------\n");	
	return sprintf(buf, "%d\n", val);
}

static ssize_t show_occ_temp_label(struct device *dev, struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int n = attr->index; 	
	struct occ_drv_data *data = dev_get_drvdata(dev);
	int ret = 0;
	occ_sensor *sensor;
	int val = 0;
	
	ret = occ_update_device(dev);

	if (ret != 0)
	{
		/* FIXME: to test fake data */
		printk("ERROR: cannot get occ sensor data: %d\n", ret);
		return ret;
	}
	
	if (data->occ_resp->data.blocks == NULL ||
		data->occ_resp->data.blocks[data->occ_resp->temp_block_id].sensor == NULL)
		return -1;

	//printk("temp_block_id: %d, sensor: %d\n", data->occ_resp->temp_block_id, n -1);	
	sensor = &data->occ_resp->data.blocks[data->occ_resp->temp_block_id].sensor[n - 1];
	val = sensor->sensor_id;
	//printk("temp%d sensor id: %d\n", n, val);

	//printk("------------- above are debug message, bellow is real output------------\n");	
	return sprintf(buf, "%d\n", val);
}

static ssize_t show_occ_power_label(struct device *dev, struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int n = attr->index; 	
	struct occ_drv_data *data = dev_get_drvdata(dev);
	int ret = 0;
	powr_sensor *sensor;
	int val = 0;
	
	ret = occ_update_device(dev);

	if (ret != 0)
	{
		/* FIXME: to test fake data */
		printk("ERROR: cannot get occ sensor data: %d\n", ret);
		return ret;
	}
	
	//printk("power_block_id: %d, sensor: %d\n", data->occ_resp->power_block_id, n -1);	
	
	if (data->occ_resp->data.blocks == NULL ||
		data->occ_resp->data.blocks[data->occ_resp->power_block_id].powr == NULL) 
		return -1;
	
	sensor = &data->occ_resp->data.blocks[data->occ_resp->power_block_id].powr[n - 1];
	val = sensor->sensor_id;
	//printk("power%d sensor id: %d\n", n, val);

	//printk("------------- above are debug message, bellow is real output------------\n");	
	return sprintf(buf, "%d\n", val);
}


static ssize_t show_occ_power_input(struct device *dev, struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int n = attr->index; 	
	struct occ_drv_data *data = dev_get_drvdata(dev);
	int ret = 0;
	powr_sensor *sensor;
	int val = 0;
	
	ret = occ_update_device(dev);

	if (ret != 0)
	{
		/* FIXME: to test fake data */
		printk("ERROR: cannot get occ sensor data: %d\n", ret);
		return ret;
	}
	
	//printk("power block_id: %d, sensor: %d\n", data->occ_resp->power_block_id, n -1);	
	
	if (data->occ_resp->data.blocks == NULL ||
		data->occ_resp->data.blocks[data->occ_resp->power_block_id].powr == NULL) 
		return -1;
	

	sensor = &data->occ_resp->data.blocks[data->occ_resp->power_block_id].powr[n - 1];
	val = sensor->value;
	//printk("power%d sensor value: %d\n", n, val);

	//printk("------------- above are debug message, bellow is real output------------\n");	
	return sprintf(buf, "%d\n", val);
}


static ssize_t show_occ_freq_label(struct device *dev, struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int n = attr->index; 	
	struct occ_drv_data *data = dev_get_drvdata(dev);
	int ret = 0;
	occ_sensor *sensor;
	int val = 0;
	
	ret = occ_update_device(dev);

	if (ret != 0)
	{
		/* FIXME: to test fake data */
		printk("ERROR: cannot get occ sensor data: %d\n", ret);
		return ret;
	}
	
	if (data->occ_resp->data.blocks == NULL ||
		data->occ_resp->data.blocks[data->occ_resp->freq_block_id].sensor == NULL) 
		return -1;

	//printk("freq_block_id: %d, sensor: %d\n", data->occ_resp->freq_block_id, n -1);	
	sensor = &data->occ_resp->data.blocks[data->occ_resp->freq_block_id].sensor[n - 1];
	val = sensor->sensor_id;
	//printk("freq%d sensor id: %d\n", n, val);

	//printk("------------- above are debug message, bellow is real output------------\n");	
	return sprintf(buf, "%d\n", val);
}


static ssize_t show_occ_freq_input(struct device *dev, struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int n = attr->index; 	
	struct occ_drv_data *data = dev_get_drvdata(dev);
	int ret = 0;
	occ_sensor *sensor;
	int val = 0;
	
	ret = occ_update_device(dev);

	if (ret != 0)
	{
		/* FIXME: to test fake data */
		printk("ERROR: cannot get occ sensor data: %d\n", ret);
		return ret;
	}
	
	if (data->occ_resp->data.blocks == NULL ||
		data->occ_resp->data.blocks[data->occ_resp->freq_block_id].sensor == NULL)
		return -1;

	//printk("block_id: %d, sensor: %d\n", data->occ_resp->freq_block_id, n -1);	
	sensor = &data->occ_resp->data.blocks[data->occ_resp->freq_block_id].sensor[n - 1];
	val = sensor->value;
	//printk("freq%d sensor value: %d\n", n, val);

	//printk("------------- above are debug message, bellow is real output------------\n");	
	return sprintf(buf, "%d\n", val);
}

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, show_occ_temp_input, NULL, 1);
static SENSOR_DEVICE_ATTR(temp2_input, S_IRUGO, show_occ_temp_input, NULL, 2);
static SENSOR_DEVICE_ATTR(temp3_input, S_IRUGO, show_occ_temp_input, NULL, 3);
static SENSOR_DEVICE_ATTR(temp4_input, S_IRUGO, show_occ_temp_input, NULL, 4);
static SENSOR_DEVICE_ATTR(temp5_input, S_IRUGO, show_occ_temp_input, NULL, 5);
static SENSOR_DEVICE_ATTR(temp6_input, S_IRUGO, show_occ_temp_input, NULL, 6);
static SENSOR_DEVICE_ATTR(temp7_input, S_IRUGO, show_occ_temp_input, NULL, 7);
static SENSOR_DEVICE_ATTR(temp8_input, S_IRUGO, show_occ_temp_input, NULL, 8);
static SENSOR_DEVICE_ATTR(temp9_input, S_IRUGO, show_occ_temp_input, NULL, 9);
static SENSOR_DEVICE_ATTR(temp1_label, S_IRUGO, show_occ_temp_label, NULL, 1);
static SENSOR_DEVICE_ATTR(temp2_label, S_IRUGO, show_occ_temp_label, NULL, 2);
static SENSOR_DEVICE_ATTR(temp3_label, S_IRUGO, show_occ_temp_label, NULL, 3);
static SENSOR_DEVICE_ATTR(temp4_label, S_IRUGO, show_occ_temp_label, NULL, 4);
static SENSOR_DEVICE_ATTR(temp5_label, S_IRUGO, show_occ_temp_label, NULL, 5);
static SENSOR_DEVICE_ATTR(temp6_label, S_IRUGO, show_occ_temp_label, NULL, 6);
static SENSOR_DEVICE_ATTR(temp7_label, S_IRUGO, show_occ_temp_label, NULL, 7);
static SENSOR_DEVICE_ATTR(temp8_label, S_IRUGO, show_occ_temp_label, NULL, 8);
static SENSOR_DEVICE_ATTR(temp9_label, S_IRUGO, show_occ_temp_label, NULL, 9);

static SENSOR_DEVICE_ATTR(power1_input, S_IRUGO, show_occ_power_input, NULL, 1);
static SENSOR_DEVICE_ATTR(power1_label, S_IRUGO, show_occ_power_label, NULL, 1);
static SENSOR_DEVICE_ATTR(power2_input, S_IRUGO, show_occ_power_input, NULL, 2);
static SENSOR_DEVICE_ATTR(power2_label, S_IRUGO, show_occ_power_label, NULL, 2);
static SENSOR_DEVICE_ATTR(power3_input, S_IRUGO, show_occ_power_input, NULL, 3);
static SENSOR_DEVICE_ATTR(power3_label, S_IRUGO, show_occ_power_label, NULL, 3);
static SENSOR_DEVICE_ATTR(power4_input, S_IRUGO, show_occ_power_input, NULL, 4);
static SENSOR_DEVICE_ATTR(power4_label, S_IRUGO, show_occ_power_label, NULL, 4);

static SENSOR_DEVICE_ATTR(freq1_input, S_IRUGO, show_occ_freq_input, NULL, 1);
static SENSOR_DEVICE_ATTR(freq1_label, S_IRUGO, show_occ_freq_label, NULL, 1);
static SENSOR_DEVICE_ATTR(freq2_input, S_IRUGO, show_occ_freq_input, NULL, 2);
static SENSOR_DEVICE_ATTR(freq2_label, S_IRUGO, show_occ_freq_label, NULL, 2);
static SENSOR_DEVICE_ATTR(freq3_input, S_IRUGO, show_occ_freq_input, NULL, 3);
static SENSOR_DEVICE_ATTR(freq3_label, S_IRUGO, show_occ_freq_label, NULL, 3);
static SENSOR_DEVICE_ATTR(freq4_input, S_IRUGO, show_occ_freq_input, NULL, 4);
static SENSOR_DEVICE_ATTR(freq4_label, S_IRUGO, show_occ_freq_label, NULL, 4);

static struct attribute *occ_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp2_input.dev_attr.attr,
	&sensor_dev_attr_temp3_input.dev_attr.attr,
	&sensor_dev_attr_temp4_input.dev_attr.attr,
	&sensor_dev_attr_temp5_input.dev_attr.attr,
	&sensor_dev_attr_temp6_input.dev_attr.attr,
	&sensor_dev_attr_temp7_input.dev_attr.attr,
	&sensor_dev_attr_temp8_input.dev_attr.attr,
	&sensor_dev_attr_temp9_input.dev_attr.attr,
	&sensor_dev_attr_temp1_label.dev_attr.attr,
	&sensor_dev_attr_temp2_label.dev_attr.attr,
	&sensor_dev_attr_temp3_label.dev_attr.attr,
	&sensor_dev_attr_temp4_label.dev_attr.attr,
	&sensor_dev_attr_temp5_label.dev_attr.attr,
	&sensor_dev_attr_temp6_label.dev_attr.attr,
	&sensor_dev_attr_temp7_label.dev_attr.attr,
	&sensor_dev_attr_temp8_label.dev_attr.attr,
	&sensor_dev_attr_temp9_label.dev_attr.attr,
	&sensor_dev_attr_power1_input.dev_attr.attr,
	&sensor_dev_attr_power2_input.dev_attr.attr,
	&sensor_dev_attr_power3_input.dev_attr.attr,
	&sensor_dev_attr_power4_input.dev_attr.attr,
	&sensor_dev_attr_power1_label.dev_attr.attr,
	&sensor_dev_attr_power2_label.dev_attr.attr,
	&sensor_dev_attr_power3_label.dev_attr.attr,
	&sensor_dev_attr_power4_label.dev_attr.attr,
	&sensor_dev_attr_freq1_input.dev_attr.attr,
	&sensor_dev_attr_freq2_input.dev_attr.attr,
	&sensor_dev_attr_freq3_input.dev_attr.attr,
	&sensor_dev_attr_freq4_input.dev_attr.attr,
	&sensor_dev_attr_freq1_label.dev_attr.attr,
	&sensor_dev_attr_freq2_label.dev_attr.attr,
	&sensor_dev_attr_freq3_label.dev_attr.attr,
	&sensor_dev_attr_freq4_label.dev_attr.attr,

	NULL
};
ATTRIBUTE_GROUPS(occ);

/*-----------------------------------------------------------------------*/
/* device probe and removal */

#define OCC_I2C_ADDR 0x50
#define OCC_I2C_NAME "occ-i2c"

enum occ_type {
	occ_id,
};

static int occ_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct occ_drv_data *data;
	unsigned long funcs;
	struct device_node *np = dev->of_node;
	u32 pval = 0;

	data = devm_kzalloc(dev, sizeof(struct occ_drv_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	data->occ_resp = &occ_resp;
	i2c_set_clientdata(client, data);
	mutex_init(&data->update_lock);
	data->sample_time = HZ;

	/* Yi: i2c-core should assign address to 
	 * client when detection - but it does not work  FIXME  */
	//client->addr = OCC_I2C_ADDR;

	/* Yi: read address from device table */
	if (of_property_read_u32(np, "reg", &pval)) {
		dev_err(&client->dev, "invalid reg\n");
	}
	client->addr = pval;
		
	/* configure the driver */
	dev_dbg(dev, "occ register hwmon @0x%x\n", client->addr);

	data->hwmon_dev = hwmon_device_register_with_groups(dev, "occ",
							    data, occ_groups);
	if (IS_ERR(data->hwmon_dev))
		return PTR_ERR(data->hwmon_dev);

	dev_dbg(dev, "%s: sensor '%s'\n",
		 dev_name(data->hwmon_dev), client->name);

	funcs = i2c_get_functionality(client->adapter);
	//dev_info(dev, "i2c adaptor supports function: 0x%lx\n", funcs); 

	/* Yi: seems always error? disable for now */
	//occ_check_i2c_errors(client);
	
	dev_info(dev, "occ i2c driver ready: i2c addr@0x%x\n", client->addr);

	return 0;
}

static int occ_remove(struct i2c_client *client)
{
	struct occ_drv_data *data = i2c_get_clientdata(client);

	/* free allocated sensor memory */	
	deinit_occ_resp_buf(data->occ_resp);

	hwmon_device_unregister(data->hwmon_dev);
	return 0;
}

/* used for old-style board info */
static const struct i2c_device_id occ_ids[] = {
	{ OCC_I2C_NAME, occ_id, },
	{ /* LIST END */ }
};
MODULE_DEVICE_TABLE(i2c, occ_ids);

static const struct of_device_id i2c_occ_of_match[] = {
	{.compatible = "ibm,occ-i2c"},
	{},
};

MODULE_DEVICE_TABLE(of, i2c_occ_of_match);

#ifdef CONFIG_PM
static int occ_suspend(struct device *dev)
{
	//struct i2c_client *client = to_i2c_client(dev);
	/* TODO */
	return 0;
}

static int occ_resume(struct device *dev)
{
	//struct i2c_client *client = to_i2c_client(dev);
	/* TODO */
	return 0;
}

static const struct dev_pm_ops occ_dev_pm_ops = {
	.suspend	= occ_suspend,
	.resume		= occ_resume,
};
#define OCC_DEV_PM_OPS (&occ_dev_pm_ops)
#else
#define OCC_DEV_PM_OPS NULL
#endif /* CONFIG_PM */

/* Yi: i2c-core uses i2c-detect() to detect device in bellow address list. 
   If exists, address will be assigned to client.
 * It is also possible to read address from device table. */
static const unsigned short normal_i2c[] = {0x50, 0x51, I2C_CLIENT_END };

/* Return 0 if detection is successful, -ENODEV otherwise */
static int occ_detect(struct i2c_client *new_client,
                       struct i2c_board_info *info)
{
	/* i2c-core need this function to create new device */
	strncpy(info->type, OCC_I2C_NAME, sizeof(OCC_I2C_NAME));
	return 0;
}

static struct i2c_driver occ_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= OCC_I2C_NAME,
		.pm	= OCC_DEV_PM_OPS,
		.of_match_table = i2c_occ_of_match,
	},
	.probe		= occ_probe,
	.remove		= occ_remove,
	.id_table	= occ_ids,
	.address_list	= normal_i2c,
	.detect		= occ_detect,
};

module_i2c_driver(occ_driver);

#if 0
/* Create new i2c device */
static struct i2c_board_info my_dev_info[] __initdata = {
	{
		I2C_BOARD_INFO(OCC_I2C_NAME, 0x50),
	},
};  

static struct i2c_client *my_client;

static int occ_init(void)
{
	static int sys_adap_bus_num = 3;
	struct i2c_adapter* adap = i2c_get_adapter(sys_adap_bus_num);
	
	if(adap==NULL) {
		printk("[OCC-DEBUG] i2c_get_adapter fail!\n");
		return -1;
	}

	my_client = i2c_new_device(adap, &my_dev_info[0]);
	if( my_client==NULL ){
		printk("[OCC-DEBUG] i2c_new_device fail!\n");
		return -1;
	}
	i2c_put_adapter(adap);
	return i2c_add_driver(&occ_driver); 
}

static void __exit occ_exit(void)  
{
	i2c_unregister_device(my_client);
	i2c_del_driver(&occ_driver);
}  

module_init(occ_init);
module_exit(occ_exit);

#endif

MODULE_AUTHOR("Li Yi <shliyi@cn.ibm.com>");
MODULE_DESCRIPTION("BMC OCC monitor driver");
MODULE_LICENSE("GPL");
