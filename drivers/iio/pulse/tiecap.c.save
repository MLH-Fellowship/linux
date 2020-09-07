// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * ECAP pulse capture driver
 *
 * Copyright (C) 2020 Linaro Limited
 * Author: Matt Porter <mporter@linaro.org>
 * Author: Darren Schachter <dts86@cornell.edu>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

/* ECAP regs and bits */
#define ECAP_CAP1			0x08
#define ECAP_CAP2			0x0c
#define ECAP_ECCTL1			0x28
#define ECAP_ECCTL1_RUN_FREE		BIT(15)
#define ECAP_ECCTL1_CAPLDEN		BIT(8)
#define ECAP_ECCTL1_CAP2POL		BIT(2)
#define ECAP_ECCTL1_CTRRST1		BIT(1)
#define ECAP_ECCTL1_CAP1POL		BIT(0)
#define ECAP_ECCTL1_PRESCALE_OFFSET	9
#define ECAP_ECCTL1_PRESCALE_MASK	(0x1F << ECAP_ECCTL1_PRESCALE_OFFSET)
#define ECAP_ECCTL2			0x2a
#define ECAP_ECCTL2_SYNCO_SEL_DIS	BIT(7)
#define ECAP_ECCTL2_TSCTR_FREERUN	BIT(4)
#define ECAP_ECCTL2_REARM		BIT(3)
#define ECAP_ECCTL2_STOP_WRAP_2		BIT(1)
#define ECAP_ECEINT			0x2c
#define ECAP_ECFLG			0x2e
#define ECAP_ECCLR			0x30
#define ECAP_ECINT_CTRCMP		BIT(7)
#define ECAP_ECINT_CTRPRD		BIT(6)
#define ECAP_ECINT_CTROVF		BIT(5)
#define ECAP_ECINT_CEVT4		BIT(4)
#define ECAP_ECINT_CEVT3		BIT(3)
#define ECAP_ECINT_CEVT2		BIT(2)
#define ECAP_ECINT_CEVT1		BIT(1)
#define ECAP_ECINT_ALL		(ECAP_ECINT_CTRCMP |	\
				ECAP_ECINT_CTRPRD |	\
				ECAP_ECINT_CTROVF |	\
				ECAP_ECINT_CEVT4 |	\
				ECAP_ECINT_CEVT3 |	\
				ECAP_ECINT_CEVT2 |	\
				ECAP_ECINT_CEVT1)

/* ECAP driver flags */
#define ECAP_PRESCALAR_OFFSET	3
#define ECAP_POL_CAP2_OFFSET	2
#define ECAP_POL_CAP1_OFFSET	1
#define ECAP_ENABLED			0
#define ECAP_PRESCALAR(flags)	(((uint8_t)(flags >> ECAP_PRESCALAR_OFFSET)) & 0x1F)


struct ecap_context {
	u32 cap1;
	u32 cap2;
	u16 ecctl1;
	u16 ecctl2;
	u16 eceint;
};

struct ecap_state {
	unsigned long   flags;	// keep track of state (enabled, polarity, etc.)
	struct mutex    lock;
	unsigned int    clk_rate;
	void __iomem    *regs;
	u32		*buf;
	struct ecap_context ctx;
};

#define dev_to_ecap_state(d) iio_priv(dev_to_iio_dev(d))

static const struct iio_chan_spec ecap_channels[] = {
	{
		.type			= IIO_PULSE,
		.channel		= 0,
		.info_mask_separate	= BIT(IIO_CHAN_INFO_SCALE),
		.scan_index		= 0,
		.scan_type = {
			.sign		= 'u',
			.realbits	= 32,
			.storagebits	= 32,
			.endianness	= IIO_LE,
		},
		.modified	= 0,
	},
	IIO_CHAN_SOFT_TIMESTAMP(1)
};

static ssize_t ecap_attr_pol_cap1_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct ecap_state *state = dev_to_ecap_state(dev);

	return sprintf(buf, "%d\n",
				test_bit(ECAP_POL_CAP1_OFFSET, &state->flags));
}

static ssize_t ecap_attr_pol_cap1_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t len)
{
	int ret;
	bool val;
	struct ecap_state *state = dev_to_ecap_state(dev);

	if (test_bit(ECAP_ENABLED, &state->flags))
		return -EINVAL;

	ret = strtobool(buf, &val);
	if (ret)
		return ret;

	if (val)
		set_bit(ECAP_POL_CAP1_OFFSET, &state->flags);
	else
		clear_bit(ECAP_POL_CAP1_OFFSET, &state->flags);

	return len;
}

static ssize_t ecap_attr_pol_cap2_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct ecap_state *state = dev_to_ecap_state(dev);

	return sprintf(buf, "%d\n",
				test_bit(ECAP_POL_CAP2_OFFSET, &state->flags));
}

static ssize_t ecap_attr_pol_cap2_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t len)
{
	int ret;
	bool val;
	struct ecap_state *state = dev_to_ecap_state(dev);

	if (test_bit(ECAP_ENABLED, &state->flags))
		return -EINVAL;

	ret = strtobool(buf, &val);
	if (ret)
		return ret;

	if (val)
		set_bit(ECAP_POL_CAP2_OFFSET, &state->flags);
	else
		clear_bit(ECAP_POL_CAP2_OFFSET, &state->flags);

	return len;
}

static ssize_t ecap_attr_prescalar_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	struct ecap_state *state = dev_to_ecap_state(dev);

	mutex_lock(&state->lock);
	ret = sprintf(buf, "%x\n", ECAP_PRESCALAR(state->flags));
	mutex_unlock(&state->lock);

	return ret;
}

static ssize_t ecap_attr_prescalar_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t len)
{
	int ret;
	long val;
	struct ecap_state *state = dev_to_ecap_state(dev);

	if (test_bit(ECAP_ENABLED, &state->flags))
		return -EINVAL;

	ret = kstrtol(buf, 16, &val);
	if (val > 0x05 && val != 0x1E && val != 0x1F)
		return -EINVAL;

	mutex_lock(&state->lock);
	state->flags &= ~(0x1F << ECAP_PRESCALAR_OFFSET); // clear bits
	state->flags |= (val << ECAP_PRESCALAR_OFFSET);
	mutex_unlock(&state->lock);

	return len;
}

static IIO_DEVICE_ATTR(pulse_cap1pol, 0644,
	ecap_attr_pol_cap1_show, ecap_attr_pol_cap1_store, 0);
static IIO_DEVICE_ATTR(pulse_cap2pol, 0644,
	ecap_attr_pol_cap2_show, ecap_attr_pol_cap2_store, 0);
static IIO_DEVICE_ATTR(pulse_prescalar, 0644,
	ecap_attr_prescalar_show, ecap_attr_prescalar_store, 0);

static struct attribute *ecap_attributes[] = {
	&iio_dev_attr_pulse_cap1pol.dev_attr.attr,
	&iio_dev_attr_pulse_cap2pol.dev_attr.attr,
	&iio_dev_attr_pulse_prescalar.dev_attr.attr,
	NULL
};

static struct attribute_group ecap_attribute_group = {
	.attrs = ecap_attributes,
};

static const struct iio_trigger_ops iio_interrupt_trigger_ops = {
	//.owner = THIS_MODULE,
};


static int ecap_read_raw(struct iio_dev *idev,
				struct iio_chan_spec const *ch, int *val,
				int *val2, long mask)
{
	struct ecap_state *state = iio_priv(idev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = NSEC_PER_SEC / state->clk_rate;
		return IIO_VAL_INT_PLUS_NANO;
	default:
		return -EINVAL;
	}
}

// (note: driver_module is no longer a field of iio_info)
static const struct iio_info ecap_info = {
	//.driver_module = THIS_MODULE,
	.attrs = &ecap_attribute_group,
	.read_raw = &ecap_read_raw
};

static irqreturn_t ecap_trigger_handler(int irq, void *private)
{
	struct iio_poll_func *pf = private;
	struct iio_dev *idev = pf->indio_dev;
	struct ecap_state *state = iio_priv(idev);

	/* Read pulse counter value */
	*state->buf = readl(state->regs + ECAP_CAP2);

	dev_dbg(&idev->dev, "TIECAP: Value: %d, Time: %lld\n", *state->buf, pf->timestamp);

	iio_push_to_buffers_with_timestamp(idev, state->buf, pf->timestamp);

	iio_trigger_notify_done(idev->trig);

	return IRQ_HANDLED;
}

static int ecap_buffer_predisable(struct iio_dev *idev)
{
	struct ecap_state *state = iio_priv(idev);
	int ret = 0;
	u16 ecctl2;

	dev_dbg(&idev->dev, "TIECAP: Buffer pre disable...\n");

	ret = iio_triggered_buffer_predisable(idev);

	/* Stop capture */
	clear_bit(ECAP_ENABLED, &state->flags);
	ecctl2 = readw(state->regs + ECAP_ECCTL2) & ~ECAP_ECCTL2_TSCTR_FREERUN;
	writew(ecctl2, state->regs + ECAP_ECCTL2);

	/* Disable and clear all interrupts */
	writew(0, state->regs + ECAP_ECEINT);
	writew(ECAP_ECINT_ALL, state->regs + ECAP_ECCLR);

	pm_runtime_put_sync(idev->dev.parent);

	return 0;
}

static int ecap_buffer_postenable(struct iio_dev *idev)
{
	struct ecap_state *state = iio_priv(idev);
	int ret = 0;
	u16 ecctl1, ecctl2;

	dev_dbg(&idev->dev, "TIECAP: Buffer post enable...\n");

	pm_runtime_get_sync(idev->dev.parent);

	ecctl1 = readw(state->regs + ECAP_ECCTL1);

	/* Configure pulse polarity */
	if (test_bit(ECAP_POL_CAP1_OFFSET, &state->flags)) {
		/* CAP1 falling */
		ecctl1 |= ECAP_ECCTL1_CAP1POL;
	} else {
		/* CAP1 rising */
		ecctl1 &= ~ECAP_ECCTL1_CAP1POL;
	}

	if (test_bit(ECAP_POL_CAP2_OFFSET, &state->flags)) {
		/* CAP2 falling */
		ecctl1 |= ECAP_ECCTL1_CAP2POL;
	} else {
		/* CAP2 rising */
		ecctl1 &= ~ECAP_ECCTL1_CAP2POL;
	}

	/* Configure pulse prescalar */
	ecctl1 &= ~ECAP_ECCTL1_PRESCALE_MASK;
	ecctl1 |= (ECAP_PRESCALAR(state->flags) << ECAP_ECCTL1_PRESCALE_OFFSET);

	writew(ecctl1, state->regs + ECAP_ECCTL1);


	/* Enable CAP2 interrupt */
	writew(ECAP_ECINT_CEVT2, state->regs + ECAP_ECEINT);

	/* Enable capture */
	ecctl2 = readw(state->regs + ECAP_ECCTL2);
	ecctl2 |= ECAP_ECCTL2_TSCTR_FREERUN | ECAP_ECCTL2_REARM;
	writew(ecctl2, state->regs + ECAP_ECCTL2);
	set_bit(ECAP_ENABLED, &state->flags);

	ret = iio_triggered_buffer_postenable(idev);

	return ret;
}

static const struct iio_buffer_setup_ops ecap_buffer_setup_ops = {
	.postenable = &ecap_buffer_postenable,
	.predisable = &ecap_buffer_predisable
};

static irqreturn_t ecap_interrupt_handler(int irq, void *private)
{
	struct iio_dev *idev = private;
	struct ecap_state *state = iio_priv(idev);
	u16 ints;

	dev_dbg(&idev->dev, "TIECAP: Interrupt handling...\n");

	iio_trigger_poll(idev->trig);

	/* Clear CAP2 interrupt */
	ints = readw(state->regs + ECAP_ECFLG);
	if (ints & ECAP_ECINT_CEVT2)
		writew(ECAP_ECINT_CEVT2, state->regs + ECAP_ECCLR);
	else
		dev_warn(&idev->dev, "unhandled interrupt flag: %04x\n", ints);

	return IRQ_HANDLED;
}

static void ecap_init_hw(struct iio_dev *idev)
{
	struct ecap_state *state = iio_priv(idev);

	// Update flags
	state->flags &= 0;

	// Initialize with CAP1 = rising, CAP2 = falling
	// (measure the on-time of the signal)
	set_bit(ECAP_POL_CAP2_OFFSET, &state->flags);

	// Configure ECAP module
	writew(ECAP_ECCTL1_RUN_FREE | ECAP_ECCTL1_CAPLDEN |
			ECAP_ECCTL1_CAP2POL | ECAP_ECCTL1_CTRRST1,
			state->regs + ECAP_ECCTL1);
	writew(ECAP_ECCTL2_SYNCO_SEL_DIS | ECAP_ECCTL2_STOP_WRAP_2,
			state->regs + ECAP_ECCTL2);
}

static int ecap_probe(struct platform_device *pdev)
{
	int irq, ret;
	struct iio_dev *idev;
	struct clk *clk;
	struct ecap_state *state;
	struct resource *r;
	struct iio_trigger *trig;

	dev_dbg(&pdev->dev, "TIECAP: Probing....\n");

	idev = devm_iio_device_alloc(&pdev->dev, sizeof(struct ecap_state));
	if (!idev)
		return -ENOMEM;
	state = iio_priv(idev);

	mutex_init(&state->lock);

	clk = devm_clk_get(&pdev->dev, "fck");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		return PTR_ERR(clk);
	}

	state->clk_rate = clk_get_rate(clk);
	if (!state->clk_rate) {
		dev_err(&pdev->dev, "failed to get clock rate\n");
		return -EINVAL;
	}

	platform_set_drvdata(pdev, idev);

	idev->dev.parent = &pdev->dev;
	idev->name = dev_name(&pdev->dev);
	idev->modes = INDIO_DIRECT_MODE;
	idev->info = &ecap_info;
	idev->channels = ecap_channels;
	/* One h/w capture and one s/w timestamp channel per instance */
	idev->num_channels = 2;

	trig = devm_iio_trigger_alloc(&pdev->dev, "%s-dev%d",
						idev->name, idev->id);

	if (!trig)
		return -ENOMEM;
	trig->dev.parent = idev->dev.parent;
	iio_trigger_set_drvdata(trig, idev);
	trig->ops = &iio_interrupt_trigger_ops;

	ret = iio_trigger_register(trig);
	if (ret) {
		dev_err(&pdev->dev, "failed to register trigger\n");
		return ret;
	}

	ret = iio_triggered_buffer_setup(idev, &iio_pollfunc_store_time,
						&ecap_trigger_handler,
						&ecap_buffer_setup_ops);
	if (ret)
		return ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq is specified\n");
		return irq;
	}
	ret = devm_request_irq(&pdev->dev, irq,
				&ecap_interrupt_handler,
				0, dev_name(&pdev->dev), idev);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to register device\n");
		goto uninit_buffer;
	}

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	state->regs = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(state->regs)) {
		dev_err(&pdev->dev, "unable to remap registers\n");
		ret = PTR_ERR(state->regs);
		goto uninit_buffer;
	}

	ret = iio_device_register(idev);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to register device\n");
		goto uninit_buffer;
	}

	state->buf = devm_kzalloc(&idev->dev, idev->scan_bytes, GFP_KERNEL);
	if (!state->buf) {
		ret = -ENOMEM;
		goto uninit_buffer;
	}

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	ecap_init_hw(idev);

	pm_runtime_put_sync(&pdev->dev);

	dev_dbg(&pdev->dev, "TIECAP: Probe complete.\n");

	return 0;

pwmss_clk_failure:
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	iio_device_unregister(idev);

uninit_buffer:
	iio_triggered_buffer_cleanup(idev);

	return ret;
}

static int ecap_remove(struct platform_device *pdev)
{
	struct iio_dev *idev = platform_get_drvdata(pdev);

	iio_device_unregister(idev);

	pm_runtime_disable(&pdev->dev);

	iio_triggered_buffer_cleanup(idev);

	dev_dbg(&pdev->dev, "TIECAP: Module removed.\n");
	return 0;
}

static int __maybe_unused ecap_suspend(struct device *dev)
{
	struct ecap_state *state = dev_to_ecap_state(dev);

	dev_dbg(dev, "TIECAP: Module suspended.\n");

	pm_runtime_get_sync(dev);
	state->ctx.cap1 = readl(state->regs + ECAP_CAP1);
	state->ctx.cap2 = readl(state->regs + ECAP_CAP2);
	state->ctx.eceint = readw(state->regs + ECAP_ECEINT);
	state->ctx.ecctl1 = readw(state->regs + ECAP_ECCTL1);
	state->ctx.ecctl2 = readw(state->regs + ECAP_ECCTL2);
	pm_runtime_put_sync(dev);

	/* If capture was active, disable eCAP */
	if (test_bit(ECAP_ENABLED, &state->flags))
		pm_runtime_put_sync(dev);

	return 0;
}

static int __maybe_unused ecap_resume(struct device *dev)
{
	struct ecap_state *state = dev_to_ecap_state(dev);

	dev_dbg(dev, "TIECAP: Module resumed.\n");

	/* If capture was active, enable ECAP */
	if (test_bit(ECAP_ENABLED, &state->flags))
		pm_runtime_get_sync(dev);

	pm_runtime_get_sync(dev);
	writel(state->ctx.cap1, state->regs + ECAP_CAP1);
	writel(state->ctx.cap2, state->regs + ECAP_CAP2);
	writew(state->ctx.eceint, state->regs + ECAP_ECEINT);
	writew(state->ctx.ecctl1, state->regs + ECAP_ECCTL1);
	writew(state->ctx.ecctl2, state->regs + ECAP_ECCTL2);
	pm_runtime_put_sync(dev);
	return 0;
}


static SIMPLE_DEV_PM_OPS(ecap_pm_ops, ecap_suspend, ecap_resume);

static const struct of_device_id ecap_of_ids[] = {
	{ .compatible   = "ti,am33xx-ecap" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ecap_of_ids);

/* Platform driver information */
static struct platform_driver ecap_driver = {
	.driver = {
		.name = "ecap_pulse", // the name "ecap" is used by the pwm_tiecap module
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ecap_of_ids),
		.pm = &ecap_pm_ops,
	},
	.probe = ecap_probe,
	.remove = ecap_remove,
};
module_platform_driver(ecap_driver);

/* Module information */
MODULE_DESCRIPTION("TI eCAP driver");
MODULE_AUTHOR("Matt Porter <porter@linaro.org>, Darren Schachter <dts86@cornell.edu>");
MODULE_LICENSE("GPL");
