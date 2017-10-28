#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "ftsfp_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

/*GPIO pins reference.*/
int ftsfp_parse_dts(struct ftsfp_dev* ftsfp_dev)
{
    int rc = 0;
    
    /*get pwr resource*/
/*	ftsfp_dev->pwr_gpio = of_get_named_gpio(ftsfp_dev->spi->dev.of_node,"goodix,gpio_pwr",0);
    if(!gpio_is_valid(ftsfp_dev->pwr_gpio)) {
        ftsfp_dbg("PWR GPIO is invalid.\n");
        return -1;
    }
    rc = gpio_request(ftsfp_dev->pwr_gpio, "goodix_pwr");
    if(rc) {
        dev_err(&ftsfp_dev->spi->dev, "Failed to request PWR GPIO. rc = %d\n", rc);
        return -1;
    }
*/
    /*get reset resource*/
#if 0
    ftsfp_dev->reset_gpio = of_get_named_gpio(ftsfp_dev->spi->dev.of_node,"focal,gpio_reset",0);
    if(!gpio_is_valid(ftsfp_dev->reset_gpio)) {
        ftsfp_dbg("RESET GPIO is invalid.\n");
        return -1;
    }
    rc = gpio_request(ftsfp_dev->reset_gpio, "focal_reset");
    if(rc) {
        dev_err(&ftsfp_dev->spi->dev, "Failed to request RESET GPIO. rc = %d\n", rc);
        return -1;
    }
    gpio_direction_output(ftsfp_dev->reset_gpio, 1);
#endif
    /*get irq resourece*/
    ftsfp_dev->irq_gpio = of_get_named_gpio(ftsfp_dev->spi->dev.of_node,"focal,gpio_irq",0);
    ftsfp_dbg("irq_gpio:%d\n", ftsfp_dev->irq_gpio);
    if(!gpio_is_valid(ftsfp_dev->irq_gpio)) {
        ftsfp_dbg("IRQ GPIO is invalid.\n");
        return -1;
    }
	ftsfp_dbg("_________focal here!!!111\n");
	rc = devm_gpio_request(&ftsfp_dev->spi->dev, ftsfp_dev->irq_gpio, "focal,gpio_irq");
	if (rc) {
		ftsfp_dbg("failed to request gpio %d\n", ftsfp_dev->irq_gpio);
		return rc;
	}
	ftsfp_dbg("_________focal here!!!2222\n");
	/*
    rc = gpio_request(ftsfp_dev->irq_gpio, "focal_irq");
    if(rc) {
        dev_err(&ftsfp_dev->spi->dev, "Failed to request IRQ GPIO. rc = %d\n", rc);
        return -1;
    }
    gpio_direction_input(ftsfp_dev->irq_gpio);
*/
	ftsfp_dev->fingerprint_pinctrl = devm_pinctrl_get(&ftsfp_dev->spi->dev);
	ftsfp_dev->pinctrl_state_irq_active = pinctrl_lookup_state(ftsfp_dev->fingerprint_pinctrl, "focalfp_irq_active");
	if (IS_ERR(ftsfp_dev->pinctrl_state_irq_active)) {
		ftsfp_dbg("cannot find '%s'\n", "focalfp_irq_active");
		rc = -EINVAL;
	}
	pinctrl_select_state(ftsfp_dev->fingerprint_pinctrl, ftsfp_dev->pinctrl_state_irq_active);

    //power on
//	gpio_direction_output(ftsfp_dev->pwr_gpio, 1);

    return 0;
}

void ftsfp_cleanup(struct ftsfp_dev	* ftsfp_dev)
{
    ftsfp_dbg("[info] %s\n",__func__);
    if (gpio_is_valid(ftsfp_dev->irq_gpio)){
        gpio_free(ftsfp_dev->irq_gpio);
        ftsfp_dbg("remove irq_gpio success\n");
    }
    if (gpio_is_valid(ftsfp_dev->reset_gpio)){
        gpio_free(ftsfp_dev->reset_gpio);
        ftsfp_dbg("remove reset_gpio success\n");
    }
/*	if (gpio_is_valid(ftsfp_dev->pwr_gpio)){
        gpio_free(ftsfp_dev->pwr_gpio);
        ftsfp_dbg("remove pwr_gpio success\n");
    }
*/
}

/*power management*/
int ftsfp_power_on(struct ftsfp_dev* ftsfp_dev)
{
    int rc = 0;
/*	if (gpio_is_valid(ftsfp_dev->pwr_gpio)) {
        gpio_set_value(ftsfp_dev->pwr_gpio, 1);
    }
*/
    msleep(10);
    ftsfp_dbg("---- power on ok ----\n");

    return rc;
}

int ftsfp_power_off(struct ftsfp_dev* ftsfp_dev)
{
    int rc = 0;			
/*	if (gpio_is_valid(ftsfp_dev->pwr_gpio)) {
        gpio_set_value(ftsfp_dev->pwr_gpio, 1);
    }
*/
    ftsfp_dbg("---- power off ----\n");
    return rc;
}

/********************************************************************
 *CPU output low level in RST pin to reset GF. This is the MUST action for GF.
 *Take care of this function. IO Pin driver strength / glitch and so on.
 ********************************************************************/
int ftsfp_hw_reset(struct ftsfp_dev *ftsfp_dev, unsigned int delay_ms)
{	
    if(ftsfp_dev == NULL) {
        ftsfp_dbg("Input Device is NULL.\n");
        return -1;
    }
    gpio_direction_output(ftsfp_dev->reset_gpio, 1);
    gpio_set_value(ftsfp_dev->reset_gpio, 0);
    mdelay(3);
    gpio_set_value(ftsfp_dev->reset_gpio, 1);
    mdelay(delay_ms);
    return 0;
}

int ftsfp_irq_num(struct ftsfp_dev *ftsfp_dev)
{
    if(ftsfp_dev == NULL) {
        ftsfp_dbg("Input Device is NULL.\n");
        return -1;
    } else {
        return gpio_to_irq(ftsfp_dev->irq_gpio);
    }
}

