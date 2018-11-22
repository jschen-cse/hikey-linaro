// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2018-19 BayLibre, SAS.

#include <linux/arm-smccc.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include <asm/system_misc.h>
#include <uapi/linux/psci.h>

struct restart_mode {
	const char *reason;
	unsigned long magic;
};

static struct restart_mode r_mode[] = {
	{ "cold_boot",		0x00 },
	{ "normal",		0x01 },
	{ "recovery",		0x02 },
	{ "factory_reset",	0x02 },
	{ "update",		0x03 },
	{ "fastboot",		0x04 },
	{ "suspend_off",	0x05 },
	{ "hibernate",		0x06 },
	{ "bootloader",		0x07 },
	{ "shutdown_reboot",	0x08 },
	{ "rpmbp",		0x09 },
	{ "crash_dump",		0x0b },
	{ "kernel_panic",	0x0c },
	{ "watchdog_reboot",	0x0d },
};

static int meson_restart_handler(struct notifier_block *this,
				 unsigned long mode, void *cmd)
{
	const char *normal = "normal";
	unsigned long magic = 0x01;
	const char *reason;
	int m;

	if (!cmd)
		reason = normal;
	else
		reason = cmd;

	for (m = 0; m < ARRAY_SIZE(r_mode); m++) {
		if (!strcmp(r_mode[m].reason, reason)) {
			magic = r_mode[m].magic;
			break;
		}
	}

	arm_smccc_smc(PSCI_0_2_FN_SYSTEM_RESET, magic, 0, 0, 0, 0, 0, 0, 0);

	while (1)
		cpu_do_idle();

	/* we shouldn't reach here. Send help */
	return 0;
}

static struct notifier_block meson_restart_nb = {
	.notifier_call = meson_restart_handler,
	.priority = 128,
};

static int meson_reboot_mode_probe(struct platform_device *pdev)
{
	int err;

	err = register_restart_handler(&meson_restart_nb);
	if (err) {
		dev_err(&pdev->dev, "cannot register restart handler (err=%d)\n", err);
		return err;
	}

	arm_pm_restart = NULL;

	return 0;
}

static const struct of_device_id meson_reboot_mode_id_table[] = {
	{ .compatible = "amlogic,meson-reboot-mode" },
	{ }
};
MODULE_DEVICE_TABLE(of, meson_reboot_mode_id_table);

static struct platform_driver meson_reboot_mode_driver = {
	.probe = meson_reboot_mode_probe,
	.driver = {
		.name = "meson-reboot-mode",
		.of_match_table = of_match_ptr(meson_reboot_mode_id_table),
	},
};
module_platform_driver(meson_reboot_mode_driver);

MODULE_DESCRIPTION("Amlogic S905*/GX*/AXG reboot mode driver");
MODULE_AUTHOR("Carlo Caione <ccaione@baylibre.com");
MODULE_LICENSE("GPL v2");
