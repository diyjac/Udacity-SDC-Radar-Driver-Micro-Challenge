#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x51d22b2b, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x651c85c2, __VMLINUX_SYMBOL_STR(os_if_spin_unlock) },
	{ 0x19a161d1, __VMLINUX_SYMBOL_STR(vCanCleanup) },
	{ 0x91831d70, __VMLINUX_SYMBOL_STR(seq_printf) },
	{ 0xc8eb0f7c, __VMLINUX_SYMBOL_STR(vCanTime) },
	{ 0x170220be, __VMLINUX_SYMBOL_STR(queue_empty) },
	{ 0xfb578fc5, __VMLINUX_SYMBOL_STR(memset) },
	{ 0xd5065308, __VMLINUX_SYMBOL_STR(queue_front) },
	{ 0x15679437, __VMLINUX_SYMBOL_STR(vCanDispatchEvent) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x9745ac16, __VMLINUX_SYMBOL_STR(set_capability_value) },
	{ 0x691087f9, __VMLINUX_SYMBOL_STR(queue_wakeup_on_space) },
	{ 0x6f580dd5, __VMLINUX_SYMBOL_STR(queue_pop) },
	{ 0xf4272a7d, __VMLINUX_SYMBOL_STR(vCanInitData) },
	{ 0x451c7bd4, __VMLINUX_SYMBOL_STR(os_if_kernel_malloc) },
	{ 0xdb7305a1, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
	{ 0x9338dd27, __VMLINUX_SYMBOL_STR(os_if_kernel_free) },
	{ 0xfa49442f, __VMLINUX_SYMBOL_STR(os_if_set_task_uninterruptible) },
	{ 0x8142aba8, __VMLINUX_SYMBOL_STR(vCanInit) },
	{ 0x389e4440, __VMLINUX_SYMBOL_STR(vCanFlushSendBuffer) },
	{ 0x6e94147d, __VMLINUX_SYMBOL_STR(queue_release) },
	{ 0x4a2261d9, __VMLINUX_SYMBOL_STR(os_if_wait_for_event_timeout_simple) },
	{ 0xdb5bb4d0, __VMLINUX_SYMBOL_STR(os_if_spin_lock) },
	{ 0xbac4b01c, __VMLINUX_SYMBOL_STR(os_if_wake_up_interruptible) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=kvcommon";


MODULE_INFO(srcversion, "B99B37FA82AE06832C20D1E");
