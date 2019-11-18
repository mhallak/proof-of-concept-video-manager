This is the kernel included driver introduced around 4.8.

Sensoray did a backport made for earlier kernels like 4.4 (Ubuntu 14.04 LTS).

This driver is not necessarily better or worse than the Techwell provided driver maintained by us.  It may have yet undiscovered bugs. It is provided as an option for newer kernels.

Note: VIDEOBUF2_DMA_CONTIG=m required in .config.

CONFIG_VIDEOBUF2_DMA_CONTIG=m

If you get an error for missing dma_contig_ops on 3.10 for example, add the following lines and rebuild the kernel modules(from your kernel source directory, not this directory).  CONFIG_VIDEOBUF2_DMA_CONTIG will automatically be added.  Adding the line CONFIG_VIDEOBUF2_DMA_CONTIG by itself may not work and may be erased from .config when you build the kernel.

CONFIG_VIDEO_DT3155=m
CONFIG_DT3155_CCIR=y
CONFIG_DT3155_STREAMING=y

