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

#ifdef RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xbd078ac0, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x8487a2b6, __VMLINUX_SYMBOL_STR(flush_work) },
	{ 0x5937ace7, __VMLINUX_SYMBOL_STR(videobuf_mmap_free) },
	{ 0x2d3385d3, __VMLINUX_SYMBOL_STR(system_wq) },
	{ 0x5e25804, __VMLINUX_SYMBOL_STR(__request_region) },
	{ 0xdbee26c0, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0x8953a9a, __VMLINUX_SYMBOL_STR(pci_bus_read_config_byte) },
	{ 0xf9a482f9, __VMLINUX_SYMBOL_STR(msleep) },
	{ 0x1fdc7df2, __VMLINUX_SYMBOL_STR(_mcount) },
	{ 0xc93a1b3, __VMLINUX_SYMBOL_STR(video_device_release_empty) },
	{ 0xd4d9ccec, __VMLINUX_SYMBOL_STR(param_ops_int) },
	{ 0xb8eec98b, __VMLINUX_SYMBOL_STR(del_timer) },
	{ 0xc364ae22, __VMLINUX_SYMBOL_STR(iomem_resource) },
	{ 0x58ad739d, __VMLINUX_SYMBOL_STR(videobuf_mmap_mapper) },
	{ 0x4bc03680, __VMLINUX_SYMBOL_STR(snd_pcm_period_elapsed) },
	{ 0xa54e9fa9, __VMLINUX_SYMBOL_STR(v4l2_device_unregister) },
	{ 0xfc897999, __VMLINUX_SYMBOL_STR(videobuf_queue_vmalloc_init) },
	{ 0x84bc974b, __VMLINUX_SYMBOL_STR(__arch_copy_from_user) },
	{ 0x5be45f22, __VMLINUX_SYMBOL_STR(remove_proc_entry) },
	{ 0x7f064722, __VMLINUX_SYMBOL_STR(videobuf_streamon) },
	{ 0xeae3dfd6, __VMLINUX_SYMBOL_STR(__const_udelay) },
	{ 0xab9f8c69, __VMLINUX_SYMBOL_STR(__video_register_device) },
	{ 0x8fdf772a, __VMLINUX_SYMBOL_STR(init_timer_key) },
	{ 0xd9e5570, __VMLINUX_SYMBOL_STR(videobuf_iolock) },
	{ 0xf630ab60, __VMLINUX_SYMBOL_STR(mutex_unlock) },
	{ 0x41f7c423, __VMLINUX_SYMBOL_STR(snd_pcm_stream_lock) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0x526c3a6c, __VMLINUX_SYMBOL_STR(jiffies) },
	{ 0x8e588539, __VMLINUX_SYMBOL_STR(v4l2_device_register) },
	{ 0xab40cca9, __VMLINUX_SYMBOL_STR(__init_waitqueue_head) },
	{ 0x351d69aa, __VMLINUX_SYMBOL_STR(pci_set_master) },
	{ 0xb1e749e, __VMLINUX_SYMBOL_STR(videobuf_read_one) },
	{ 0x97fdbab9, __VMLINUX_SYMBOL_STR(_raw_spin_unlock_irqrestore) },
	{ 0x6edb8a9d, __VMLINUX_SYMBOL_STR(__mutex_init) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0xb77b0159, __VMLINUX_SYMBOL_STR(v4l2_prio_init) },
	{ 0xaf83b041, __VMLINUX_SYMBOL_STR(videobuf_qbuf) },
	{ 0x36dc68b4, __VMLINUX_SYMBOL_STR(video_unregister_device) },
	{ 0x55c97f, __VMLINUX_SYMBOL_STR(snd_pcm_set_ops) },
	{ 0x6a0d745, __VMLINUX_SYMBOL_STR(videobuf_querybuf) },
	{ 0x5792f848, __VMLINUX_SYMBOL_STR(strlcpy) },
	{ 0x945a0ca0, __VMLINUX_SYMBOL_STR(pci_bus_write_config_dword) },
	{ 0xb5b47bc8, __VMLINUX_SYMBOL_STR(mutex_lock) },
	{ 0x67de853, __VMLINUX_SYMBOL_STR(snd_pcm_lib_free_pages) },
	{ 0xfa5bcf35, __VMLINUX_SYMBOL_STR(mod_timer) },
	{ 0xd6b8e852, __VMLINUX_SYMBOL_STR(request_threaded_irq) },
	{ 0xc4b40cb8, __VMLINUX_SYMBOL_STR(snd_pcm_lib_ioctl) },
	{ 0xf24b3dfe, __VMLINUX_SYMBOL_STR(__ioremap) },
	{ 0x50e85ca7, __VMLINUX_SYMBOL_STR(snd_pcm_lib_malloc_pages) },
	{ 0xba0ebf79, __VMLINUX_SYMBOL_STR(snd_card_new) },
	{ 0xf5ef842e, __VMLINUX_SYMBOL_STR(v4l_bound_align_image) },
	{ 0xf8613519, __VMLINUX_SYMBOL_STR(pci_bus_read_config_dword) },
	{ 0xb35dea8f, __VMLINUX_SYMBOL_STR(__arch_copy_to_user) },
	{ 0x505215f1, __VMLINUX_SYMBOL_STR(video_devdata) },
	{ 0x92460c0e, __VMLINUX_SYMBOL_STR(snd_pcm_stream_unlock) },
	{ 0xa202a8e5, __VMLINUX_SYMBOL_STR(kmalloc_order_trace) },
	{ 0x64eab62c, __VMLINUX_SYMBOL_STR(videobuf_vmalloc_free) },
	{ 0x8d15114a, __VMLINUX_SYMBOL_STR(__release_region) },
	{ 0xf1c3c77c, __VMLINUX_SYMBOL_STR(pci_unregister_driver) },
	{ 0xd2c819a9, __VMLINUX_SYMBOL_STR(videobuf_to_vmalloc) },
	{ 0x347b75e7, __VMLINUX_SYMBOL_STR(kmem_cache_alloc_trace) },
	{ 0x96220280, __VMLINUX_SYMBOL_STR(_raw_spin_lock_irqsave) },
	{ 0x6db9d60a, __VMLINUX_SYMBOL_STR(videobuf_dqbuf) },
	{ 0x65345022, __VMLINUX_SYMBOL_STR(__wake_up) },
	{ 0xdc14eda7, __VMLINUX_SYMBOL_STR(pci_pci_problems) },
	{ 0x4f68e5c9, __VMLINUX_SYMBOL_STR(do_gettimeofday) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x4829a47e, __VMLINUX_SYMBOL_STR(memcpy) },
	{ 0x6e465587, __VMLINUX_SYMBOL_STR(param_array_ops) },
	{ 0x545c2c73, __VMLINUX_SYMBOL_STR(videobuf_reqbufs) },
	{ 0x7f9da064, __VMLINUX_SYMBOL_STR(__pci_register_driver) },
	{ 0x45a55ec8, __VMLINUX_SYMBOL_STR(__iounmap) },
	{ 0x45643ab7, __VMLINUX_SYMBOL_STR(pci_get_device) },
	{ 0x86adeaa1, __VMLINUX_SYMBOL_STR(snd_pcm_lib_preallocate_pages_for_all) },
	{ 0xd7aab929, __VMLINUX_SYMBOL_STR(snd_card_free) },
	{ 0x6dd7d47, __VMLINUX_SYMBOL_STR(snd_card_register) },
	{ 0xf162b851, __VMLINUX_SYMBOL_STR(pci_dev_put) },
	{ 0x2e0d2f7f, __VMLINUX_SYMBOL_STR(queue_work_on) },
	{ 0x28318305, __VMLINUX_SYMBOL_STR(snprintf) },
	{ 0x615868c, __VMLINUX_SYMBOL_STR(snd_pcm_new) },
	{ 0xb7cbfa8, __VMLINUX_SYMBOL_STR(pci_enable_device) },
	{ 0xe1b77fe6, __VMLINUX_SYMBOL_STR(dma_release_from_coherent_attr) },
	{ 0x390f82db, __VMLINUX_SYMBOL_STR(dma_alloc_from_coherent_attr) },
	{ 0xd4742f3c, __VMLINUX_SYMBOL_STR(video_ioctl2) },
	{ 0xbdecb211, __VMLINUX_SYMBOL_STR(dma_ops) },
	{ 0xf20dabd8, __VMLINUX_SYMBOL_STR(free_irq) },
	{ 0xe914e41e, __VMLINUX_SYMBOL_STR(strcpy) },
	{ 0x1723d5e0, __VMLINUX_SYMBOL_STR(videobuf_streamoff) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=videobuf-core,videobuf-vmalloc";

MODULE_ALIAS("pci:v00001797d00006869sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v00006000d00000812sv*sd*bc*sc*i*");
