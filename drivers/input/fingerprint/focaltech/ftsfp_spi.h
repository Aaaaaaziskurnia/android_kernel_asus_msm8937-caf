#ifndef __FTSFP_SPI_H
#define __FTSFP_SPI_H

#include <linux/types.h>
#include <linux/notifier.h>
#include <linux/wakelock.h>

/**************************debug******************************/
#define FTSFP_DEBUG
/*#undef  FTSFP_DEBUG*/

#ifdef  FTSFP_DEBUG
#define ftsfp_dbg(fmt, args...) do { \
                        pr_warn("ftsfp:" fmt, ##args);\
                    } while (0)
#define FUNC_ENTRY()    pr_warn("ftsfp:%s, entry\n", __func__)
#define FUNC_EXIT()     pr_warn("ftsfp:%s, exit\n", __func__)
#else
#define ftsfp_dbg(fmt, args...)
#define FUNC_ENTRY()
#define FUNC_EXIT()
#endif

/**********************************************************/
enum FP_MODE{
	FTSFP_IMAGE_MODE = 0,
	FTSFP_KEY_MODE,
	FTSFP_SLEEP_MODE,
	FTSFP_FF_MODE,
	FTSFP_DEBUG_MODE = 0x56
};

struct ftsfp_key {
	unsigned int key;
	int value;
};

struct ftsfp_key_map_ft
{
    char *name;
    unsigned short val;
};

#define  FTSFP_IOC_MAGIC         'G'
#define  FTSFP_IOC_DISABLE_IRQ	_IO(FTSFP_IOC_MAGIC, 0)
#define  FTSFP_IOC_ENABLE_IRQ	_IO(FTSFP_IOC_MAGIC, 1)
#define  FTSFP_IOC_SETSPEED     _IOW(FTSFP_IOC_MAGIC, 2, unsigned int)
#define  FTSFP_IOC_RESET        _IO(FTSFP_IOC_MAGIC, 3)
#define  FTSFP_IOC_COOLBOOT     _IO(FTSFP_IOC_MAGIC, 4)
#define  FTSFP_IOC_SENDKEY      _IOW(FTSFP_IOC_MAGIC, 5, struct ftsfp_key)
#define  FTSFP_IOC_CLK_READY    _IO(FTSFP_IOC_MAGIC, 6)
#define  FTSFP_IOC_CLK_UNREADY  _IO(FTSFP_IOC_MAGIC, 7)
#define  FTSFP_IOC_PM_FBCABCK   _IO(FTSFP_IOC_MAGIC, 8)
#define  FTSFP_IOC_POWER_ON     _IO(FTSFP_IOC_MAGIC, 9)
#define  FTSFP_IOC_POWER_OFF    _IO(FTSFP_IOC_MAGIC, 10)
#define  FTSFP_IOC_GET_IO_RESOURCE  _IO(FTSFP_IOC_MAGIC, 11)
#define  FTSFP_IOC_GET_MCU_STATUS  _IOR(FTSFP_IOC_MAGIC, 17, u8)

#define  FTSFP_IOC_MAXNR        11

/*#define AP_CONTROL_CLK    1 */
#define USE_PLATFORM_BUS  1 
/*#define  USE_SPI_BUS	    1 */

#define FTSFP_FASYNC           0   /*If support fasync mechanism.*/
struct ftsfp_dev {
	dev_t devt;
	struct list_head device_entry;
#if defined(USE_SPI_BUS)
	struct spi_device *spi;
#elif defined(USE_PLATFORM_BUS)
	struct platform_device *spi;
#endif
	struct clk *core_clk;
	struct clk *iface_clk;

	struct input_dev *input;
	/* buffer is NULL unless this device is open (users > 0) */
	unsigned users;
	signed irq_gpio;
	signed reset_gpio;
	signed pwr_gpio;
	int irq;
	int irq_enabled;
	int clk_enabled;
#ifdef FTSFP_FASYNC
	struct fasync_struct *async;
#endif
	struct notifier_block notifier;
	char device_available;
	char fb_black;
	struct pinctrl *fingerprint_pinctrl;
	struct pinctrl_state *pinctrl_state_rst_out0;
	struct pinctrl_state *pinctrl_state_rst_out1;
	struct pinctrl_state *pinctrl_state_irq_active;
	struct wake_lock fts_wlock;
};

int ftsfp_parse_dts(struct ftsfp_dev* ftsfp_dev);
void ftsfp_cleanup(struct ftsfp_dev *ftsfp_dev);

int ftsfp_power_on(struct ftsfp_dev *ftsfp_dev);
int ftsfp_power_off(struct ftsfp_dev *ftsfp_dev);

int ftsfp_hw_reset(struct ftsfp_dev *ftsfp_dev, unsigned int delay_ms);
int ftsfp_irq_num(struct ftsfp_dev *ftsfp_dev);

#endif /*__FTSFP_SPI_H*/
