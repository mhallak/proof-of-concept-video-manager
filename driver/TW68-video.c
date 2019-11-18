/*
 * device driver for TW6868 based PCIe analog video capture cards
 * video4linux video interface
 *
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sort.h>

#include <media/v4l2-common.h>
/// #include <media/rds.h>

#include "TW68.h"
#include "TW68_defines.h"
#include "s812ioctl.h"

#define TW686X_VIDSTAT_VDLOSS BIT(7)
#define TW686X_VIDSTAT_HLOCK BIT(6)

#define VDREG8(a0) ((const u16[8]) { \
	a0 + 0x000, a0 + 0x010, a0 + 0x020, a0 + 0x030,	\
	a0 + 0x100, a0 + 0x110, a0 + 0x120, a0 + 0x130})
#define VIDSTAT			VDREG8(0x100)


/* ------------------------------------------------------------------ */

unsigned int video_debug;
static unsigned int gbuffers = 8;
static unsigned int noninterlaced; /* 0 */
static unsigned int gbufsize = 800*576*4;
static unsigned int gbufsize_max = 800*576*4;
static char secam[] = "--";
static unsigned int VideoFrames_limit = 64;    //16
static unsigned int sfield = 1;

static unsigned int vidstd_open = -1; // -1 == autodetect, 1 = NTSC, 2==PAL
// this setting is overriden if driver compiled with
// ntsc_op =y or pal_op =y

module_param(vidstd_open, int, 0644);
MODULE_PARM_DESC(vidstd_open, "-1 == autodetect 1==NTSC, 2==PAL. Othervalues defualt to autodetect Note: this setting is overriden if driver compiled with ntsc_op =y or pal_op =y");


module_param(sfield, int, 0644);
MODULE_PARM_DESC(sfield, "single field mode (half size image will not be scaled from the full image");

module_param(video_debug, int, 0644);
MODULE_PARM_DESC(video_debug, "enable debug messages [video]");
module_param(gbuffers, int, 0444);
MODULE_PARM_DESC(gbuffers, "number of capture buffers, range 2-32");
module_param(noninterlaced, int, 0644);
MODULE_PARM_DESC(noninterlaced, "capture non interlaced video");


#define dprintk(fmt, arg...)	if (video_debug&0x04)                   \
                printk(KERN_DEBUG "%s/video: " fmt, dev->name , ## arg)


/* const was added to some v4l2_ioctl_ops (see v4l2-ioctl.h) */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
#define V4L_CONST const
#else
#define V4L_CONST 
#endif


/* ------------------------------------------------------------------ */
/* data structs for video                                             */
/*
  static int video_out[][9] = {
  [CCIR656] = { 0x00, 0xb1, 0x00, 0xa1, 0x00, 0x04, 0x06, 0x00, 0x00 },
  };
*/

static struct TW68_format formats[] = {
        {
                .name     = "15 bpp RGB, le",
                .fourcc   = V4L2_PIX_FMT_RGB555,
                .depth    = 16,
                .pm       = 0x13 | 0x80,
        },{
                .name     = "16 bpp RGB, le",
                .fourcc   = V4L2_PIX_FMT_RGB565,
                .depth    = 16,
                .pm       = 0x10 | 0x80,
        },{
                .name     = "4:2:2 packed, YUYV",
                .fourcc   = V4L2_PIX_FMT_YUYV,
                .depth    = 16,
                .pm       = 0x00,
                .bswap    = 1,
                .yuv      = 1,
        },{
                .name     = "4:2:2 packed, UYVY",
                .fourcc   = V4L2_PIX_FMT_UYVY,
                .depth    = 16,
                .pm       = 0x00,
                .yuv      = 1,
        }
};

#define FORMATS ARRAY_SIZE(formats)

#define NORM_625_50                             \
	.h_start       = 0,                     \
                .h_stop        = 719,           \
                .video_v_start = 24,            \
                .video_v_stop  = 311,           \
                .vbi_v_start_0 = 7,             \
                .vbi_v_stop_0  = 22,            \
                .vbi_v_start_1 = 319,           \
                .src_timing    = 4

#define NORM_525_60                             \
	.h_start       = 0,                     \
                .h_stop        = 719,           \
                .video_v_start = 23,            \
                .video_v_stop  = 262,           \
                .vbi_v_start_0 = 10,            \
                .vbi_v_stop_0  = 21,            \
                .vbi_v_start_1 = 273,           \
                .src_timing    = 7

static struct TW68_tvnorm tvnorms[] = {
        {
                .name          = "PAL", /* autodetect */
                .id            = V4L2_STD_PAL,
                NORM_625_50,

                .sync_control  = 0x18,
                .luma_control  = 0x40,
                .chroma_ctrl1  = 0x81,
                .chroma_gain   = 0x2a,
                .chroma_ctrl2  = 0x06,
                .vgate_misc    = 0x1c,

        },{
                .name          = "PAL-BG",
                .id            = V4L2_STD_PAL_BG,
                NORM_625_50,

                .sync_control  = 0x18,
                .luma_control  = 0x40,
                .chroma_ctrl1  = 0x81,
                .chroma_gain   = 0x2a,
                .chroma_ctrl2  = 0x06,
                .vgate_misc    = 0x1c,

        },{
                .name          = "PAL-I",
                .id            = V4L2_STD_PAL_I,
                NORM_625_50,

                .sync_control  = 0x18,
                .luma_control  = 0x40,
                .chroma_ctrl1  = 0x81,
                .chroma_gain   = 0x2a,
                .chroma_ctrl2  = 0x06,
                .vgate_misc    = 0x1c,

        },{
                .name          = "PAL-DK",
                .id            = V4L2_STD_PAL_DK,
                NORM_625_50,

                .sync_control  = 0x18,
                .luma_control  = 0x40,
                .chroma_ctrl1  = 0x81,
                .chroma_gain   = 0x2a,
                .chroma_ctrl2  = 0x06,
                .vgate_misc    = 0x1c,

        },{
                .name          = "NTSC",
                .id            = V4L2_STD_NTSC,
                NORM_525_60,

                .sync_control  = 0x59,
                .luma_control  = 0x40,
                .chroma_ctrl1  = 0x89,
                .chroma_gain   = 0x2a,
                .chroma_ctrl2  = 0x0e,
                .vgate_misc    = 0x18,
        }
};
#define TVNORMS ARRAY_SIZE(tvnorms)

#define V4L2_CID_PRIVATE_INVERT      (V4L2_CID_PRIVATE_BASE + 0)
#define V4L2_CID_PRIVATE_Y_ODD       (V4L2_CID_PRIVATE_BASE + 1)
#define V4L2_CID_PRIVATE_Y_EVEN      (V4L2_CID_PRIVATE_BASE + 2)
#define V4L2_CID_PRIVATE_AUTOMUTE    (V4L2_CID_PRIVATE_BASE + 3)
#define V4L2_CID_PRIVATE_LASTP1      (V4L2_CID_PRIVATE_BASE + 4)

static const struct v4l2_queryctrl no_ctrl = {
        .name  = "42",
        .flags = V4L2_CTRL_FLAG_DISABLED,
};
static const struct v4l2_queryctrl video_ctrls[] = {
        /* --- video --- */
        {
                .id            = V4L2_CID_BRIGHTNESS,
                .name          = "Brightness",
                .minimum       = 0,
                .maximum       = 255,
                .step          = 1,
                .default_value = 125,
                .type          = V4L2_CTRL_TYPE_INTEGER,
        },{
                .id            = V4L2_CID_CONTRAST,
                .name          = "Contrast",
                .minimum       = 0,
                .maximum       = 200,  //127
                .step          = 1,
                .default_value = 96,
                .type          = V4L2_CTRL_TYPE_INTEGER,
        },{
                .id            = V4L2_CID_SATURATION,
                .name          = "Saturation",
                .minimum       = 0,
                .maximum       = 127,
                .step          = 1,
                .default_value = 64,
                .type          = V4L2_CTRL_TYPE_INTEGER,
        },{
                .id            = V4L2_CID_HUE,
                .name          = "Hue",
                .minimum       = -124,   //-128,
                .maximum       = 125,    // 127,
                .step          = 1,
                .default_value = 0,
                .type          = V4L2_CTRL_TYPE_INTEGER,
        },
        /* --- audio --- */
        {
                .id            = V4L2_CID_AUDIO_MUTE,
                .name          = "Mute",
                .minimum       = 0,
                .maximum       = 1,
                .type          = V4L2_CTRL_TYPE_BOOLEAN,
        },{
                .id            = V4L2_CID_AUDIO_VOLUME,
                .name          = "Volume",
                .minimum       = -15,
                .maximum       = 15,
                .step          = 1,
                .default_value = 0,
                .type          = V4L2_CTRL_TYPE_INTEGER,
        },
        /* --- private --- */
        {
                .id            = V4L2_CID_PRIVATE_Y_ODD,
                .name          = "y offset odd field",
                .minimum       = 0,
                .maximum       = 128,
                .step          = 1,
                .default_value = 0,
                .type          = V4L2_CTRL_TYPE_INTEGER,
        },{
                .id            = V4L2_CID_PRIVATE_Y_EVEN,
                .name          = "y offset even field",
                .minimum       = 0,
                .maximum       = 128,
                .step          = 1,
                .default_value = 0,
                .type          = V4L2_CTRL_TYPE_INTEGER,
        },{
                .id            = V4L2_CID_PRIVATE_AUTOMUTE,
                .name          = "automute",
                .minimum       = 0,
                .maximum       = 1,
                .default_value = 1,
                .type          = V4L2_CTRL_TYPE_BOOLEAN,
        },
        /* --- gpio --- */
        {
                .id            = S812_CID_GPIO,
                .name          = "gpio",
                .minimum       = 0,
                .maximum       = 0xFFFF,
                .default_value = 0xFF,
                .type          = V4L2_CTRL_TYPE_INTEGER,
        }
};
static const unsigned int CTRLS = ARRAY_SIZE(video_ctrls);

static const struct v4l2_queryctrl* ctrl_by_id(unsigned int id)
{
        unsigned int i;

        for (i = 0; i < CTRLS; i++)
                if (video_ctrls[i].id == id)
                        return video_ctrls+i;
        return NULL;
}

static struct TW68_format* format_by_fourcc(unsigned int fourcc)
{
        unsigned int i;
        for (i = 0; i < FORMATS; i++)
                if (formats[i].fourcc == fourcc )
                        return formats+i;
        return NULL;
}

/* ----------------------------------------------------------------------- */
/* resource management                                                     */
// Each TW68 device has 8 channels
static int res_get(struct TW68_vc *vc, struct TW68_fh *fh, unsigned int bit)
{
	if (fh->resources & bit)
		/* have it already allocated */
		return 1;

// 4.0.0 uses unlocked ioctl, which already holds this lock
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
	mutex_lock(&vc->lock);
#endif
	/* is it free? */
	if (vc->resources & bit) {
		/* no, someone else uses it */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
		mutex_unlock(&vc->lock);
#endif
		return 0;
	}
	/* it's free, grab it */
	fh->resources  |= bit;
	vc->resources |= bit;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
	mutex_unlock(&vc->lock);
#endif
	return 1;
}

static int res_locked(struct TW68_vc *vc, unsigned int bit)
{
        return vc->resources & bit;
}

static int res_check(struct TW68_fh *fh, unsigned int bit)
{
        return fh->resources & bit;
}

static void res_free(struct TW68_vc *vc, struct TW68_fh *fh, unsigned int bits)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
        mutex_lock(&vc->lock);
#endif
        fh->resources &= ~bits;
        vc->resources &= ~bits;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
        mutex_unlock(&vc->lock);
#endif
}

/* ------------------------------------------------------------------ */

static void set_tvnorm(struct TW68_dev *dev, struct TW68_tvnorm *norm)
{
        int	framesize;
        dprintk("set tv norm = %s\n",norm->name);
//        printk("------set tv norm = %s\n",norm->name);
        dev->tvnorm = norm;

        /* setup cropping */
        dev->crop_bounds.left    = norm->h_start;
        dev->crop_defrect.left   = norm->h_start;
        dev->crop_bounds.width   = norm->h_stop - norm->h_start +1;
        dev->crop_defrect.width  = norm->h_stop - norm->h_start +1;

        dev->crop_bounds.top     = (norm->vbi_v_stop_0+1)*2;
        dev->crop_defrect.top    = norm->video_v_start*2;
        dev->crop_bounds.height  = ((norm->id & V4L2_STD_525_60) ? 524 : 622) - dev->crop_bounds.top;
        dev->crop_defrect.height = (norm->video_v_stop - norm->video_v_start +1)*2;

        dev->crop_current = dev->crop_defrect;

        framesize = dev->crop_bounds.width * dev->crop_bounds.height * 16 >> 3;    // calculate byte size for 1 frame


//        printk("------dev->crop setting   set tv norm = %s,  width%d   height%d  size %d\n", norm->name, dev->crop_bounds.width, dev->crop_bounds.height,  framesize);

}


/* ------------------------------------------------------------------ */

struct cliplist {
        __u16 position;
        __u8  enable;
        __u8  disable;
};



/* ------------------------------------------------------------------ */

static int buffer_activate(struct TW68_dev *dev, ///unsigned int nId,
                           struct TW68_buf *buf,
                           struct TW68_buf *next)
{
        /// dprintk("buffer_activate buf=%p\n",buf);
        buf->vb.state = VIDEOBUF_ACTIVE;
        buf->top_seen = 0;

        return 0;  //-1;
}



static void free_buffer(struct videobuf_queue *q, struct TW68_buf *buf)
{
        struct TW68_fh *fh = q->priv_data;
        unsigned long flags;
        //printk(  "%s, state: %i\n", __func__, buf->vb.state);
        spin_lock_irqsave(&fh->vc->slock, flags);
        if (fh->vc->video_dmaq.curr) {
                if (buf == fh->vc->video_dmaq.curr) {
                        fh->vc->video_dmaq.curr = NULL;
                        fh->vc->is_streaming = 0;
                }
        }
        spin_unlock_irqrestore(&fh->vc->slock, flags);
        videobuf_vmalloc_free(&buf->vb);
        buf->vb.state = VIDEOBUF_NEEDS_INIT;
}


static int buffer_prepare(struct videobuf_queue *q,
                          struct videobuf_buffer *vb,
                          enum v4l2_field field)
{
        struct TW68_fh *fh = q->priv_data;
        struct TW68_dev *dev = fh->dev;
        struct TW68_buf *buf = container_of(vb,struct TW68_buf,vb);
        unsigned int size;
        unsigned int DMA_nCH = fh->DMA_nCH;
        unsigned int nId = 0;
        int err;
        unsigned long flags = 0;
        ///  printk("  >>>>buffer_prepare DMA_nCH%x  %p  vb->baddr:%p,  vb->bsize:%d   \n",  DMA_nCH, vb, vb->baddr, vb->bsize );
        /* sanity checks */
        if (NULL == fh->fmt)
                return -EINVAL;
        ////////////////////////////////////////////////////
        /*
          if (fh->width    < 48 ||
          fh->height   < 32 ||
          fh->width/4  > dev->crop_current.width  ||
          fh->height/4 > dev->crop_current.height ||
          fh->width    > dev->crop_bounds.width  ||
          fh->height   > dev->crop_bounds.height)
          return -EINVAL;
        */
        size = (fh->width * fh->height * fh->fmt->depth) >> 3;
        if (0 != buf->vb.baddr  &&  buf->vb.bsize < size)
                return -EINVAL;

        ////  cause PAL stop
///////////////////////////////////////////////////////////////////////////////
        if (VIDEOBUF_NEEDS_INIT == buf->vb.state) 
        {
                buf->vb.width = fh->width;
                buf->vb.height = fh->height;
                buf->vb.size = size;
                buf->vb.field = field;
                buf->fmt = fh->fmt;
                //buf->pt = &fh->pt_cap;
                nId = DMA_nCH;
                spin_lock_irqsave(&dev->vc[nId].slock, flags);
                dev->vc[nId].video_dmaq.curr = NULL;
                spin_unlock_irqrestore(&dev->vc[nId].slock, flags);
                /// printk("buffer_prepare INIT  vb->baddr:%x,  vb->bsize:%d   \n", vb->baddr, vb->bsize );
                err = videobuf_iolock(q,&buf->vb, NULL);
                if (err< 0)
                        goto oops;

                ///printk("  ffff>>>>buffer_prepare   dev->m_Page0 %p  dev->m_Page0.cpu %x %x %x 	ptr:0x%x  %x %x | %x %x \n", dev->m_Page0, ptr, *ptr, *(ptr+1), *(ptr+2), *(ptr+3) );
        }

        buf->vb.state = VIDEOBUF_PREPARED;
        buf->activate = buffer_activate;  //set activate fn ptr
        return 0;

oops:
        printk("buffer_prepare  OOPS \n");
        free_buffer(q, buf);

        return err;
}


int buffer_setup(struct videobuf_queue *q, unsigned int *count, unsigned int *size)
{
        unsigned int ChannelOffset, nId, pgn;
        u32	m_dwCHConfig, dwReg, dwRegH, dwRegW, nScaler, dwReg2;
        u32  m_StartIdx, m_EndIdx, m_nVideoFormat,			\
                m_bHorizontalDecimate, m_bVerticalDecimate, m_nDropChannelNum, \
                m_bDropMasterOrSlave, m_bDropField, m_bDropOddOrEven, m_nCurVideoChannelNum;
        struct TW68_dev *dev;
        struct TW68_fh *fh = q->priv_data;
        int single_field = 0;
	unsigned long flags = 0;

        dev = fh->dev;
	spin_lock_irqsave(&dev->slock, flags);
        ChannelOffset = (PAGE_SIZE <<1) /8 /8;
        // NTSC  FIELD entry number for 720*240*2
        /// ChannelOffset = 128;   ///85;
        fh = q->priv_data;
        nId = fh ->DMA_nCH;   // DMA channel

        
        dwReg2 =  reg_readl(DMA_CH0_CONFIG+ 2);
        dwReg =  reg_readl(DMA_CH0_CONFIG+ nId);

//        printk(" ******** buffer_setup ####  CH%d::   dwReg2: 0x%X   deReg 0x%X  \n", nId, dwReg2, dwReg );
        *size = fh->fmt->depth * fh->width * fh->height >> 3;    // calculate byte size for 1 frame
        if (nId < 4) {
                //dwReg =  reg_readl(DECODER0_SDT+ (nId*0x10));
                //printk(" ####%%%%%%%%%%%%%%%%%%%%%%% buffer_setup::  DECODER0_SDT %d,  0X%X \n", nId, dwReg);
                //reg_writel(DECODER0_SDT+ (nId*0x10), 7);		/// 0 NTSC
        } else {
                ///dwReg =  DeviceRead2864(dev,(exVD0_SDT + ((nId-4)*0x10)));
                //printk(" ####%%%%%%%%%%%%%%%%%%%%%%% buffer_setup::  DECODER0_SDT %d,  0X%X \n", nId, dwReg);
                ///DeviceWrite2864(dev,(exVD0_SDT + ((nId-4)*0x10)), 7);    /// 7 auto detect   slower

        }
////////////////////////////// decoder resize //////////////////////////////////////////
	single_field = (fh->vidq.field == V4L2_FIELD_TOP) ||
		(fh->vidq.field == V4L2_FIELD_BOTTOM);

	if (sfield) {
		if (single_field) {
			DecoderResize(dev, nId, fh->height, fh->width);
			BFDMA_setup(dev, nId, fh->height, *size / fh->height);
		} else {
			DecoderResize(dev, nId, fh->height/2, fh->width);
			BFDMA_setup(dev, nId, (fh->height /2), (*size /fh->height));   // BFbuf setup  DMA mode ...
		}
	} else {
		DecoderResize(dev, nId, fh->height/2, fh->width);
		BFDMA_setup(dev, nId, (fh->height /2), (*size /fh->height));   // BFbuf setup  DMA mode ...
	}

        /// Fixed_SG_Mapping(dev, nId, *size);  //   nDMA_channel



        dwReg2 = reg_readl(DMA_CH0_CONFIG+ 2);
        dwReg = reg_readl(DMA_CH0_CONFIG+ nId);

//        printk(" ********#### FM CH%d::   dwReg2: 0x%X   deReg 0x%X  H:%d W:%d\n", nId, dwReg2, dwReg, fh->height /2, (*size /fh->height) );


        if (0 == *count)
                *count = gbuffers;

        /// pgn = (*size + PAGE_SIZE -1) /PAGE_SIZE;

        pgn = TW68_buffer_pages(*size / 2) - 1;  // page number for 1 field
        //pgn = (pages+1) /2;
        /// *count = TW68_buffer_count(*size,*count);
	while (*size * *count > VideoFrames_limit * 1024 * 1024 *2 )
                (*count)--;
	m_nDropChannelNum = 0;
	m_bDropMasterOrSlave = 1;   // master
	m_bDropField = 0;      ////////////////////// 1
	m_bDropOddOrEven = 0;
	m_bHorizontalDecimate =0;
	m_bVerticalDecimate = 0;
	m_StartIdx = ChannelOffset * nId;
	m_EndIdx = m_StartIdx + pgn;    ///pgn;  85 :: 720 * 480
	m_nCurVideoChannelNum = 0;  // real-time video channel  starts 0
	m_nVideoFormat = dev->vc[nId].nVideoFormat;
//	printk(" #### buffer_setup:: W:%d H:%d  frame size: %d,  gbuffers: %d, ChannelOffset: %d  field pgn: %d  m_StartIdx %d  m_EndIdx %d \n", fh->width, fh->height, *size,  *count, ChannelOffset, pgn, m_StartIdx, m_EndIdx );
	m_dwCHConfig = ( m_StartIdx&0x3FF) |    // 10 bits
                ((m_EndIdx&0x3FF)<<10) |	 // 10 bits
                ((m_nVideoFormat&7)<<20) |
                ((m_bHorizontalDecimate&1)<<23) |
                ((m_bVerticalDecimate&1)<<24) |
                ((m_nDropChannelNum&3)<<25) |
                ((m_bDropMasterOrSlave&1)<<27) |    // 1 bit
                ((m_bDropField&1)<<28) |
                ((m_bDropOddOrEven&1)<<29) |
                ((m_nCurVideoChannelNum&3)<<30);
	
	reg_writel( DMA_CH0_CONFIG+ nId, m_dwCHConfig);
        dwReg = reg_readl(DMA_CH0_CONFIG+ nId);
//        printk(" ********#### buffer_setup CH%d::  m_StartIdx 0X%x  pgn%d  m_dwCHConfig: 0x%X   dwReg: 0x%X    \n", nId, m_StartIdx, pgn,  m_dwCHConfig, dwReg );
//////external video decoder settings//////////////////////////////////////////////////////////////////////////

        dwRegW = fh->width;

	if (sfield && single_field) {
		dwRegH = fh->height;
	} else {
		dwRegH = fh->height / 2;  // frame height
	}
        dwReg = dwRegW | (dwRegH<<16) | (1<<31);
        dwRegW = dwRegH = dwReg;
        //Video Size
        reg_writel(VIDEO_SIZE_REG, dwReg);	    //for Rev.A backward compatible
        ///  xxx dwReg = reg_readl(VIDEO_SIZE_REG);
//        printk(" #### buffer_setup:: VIDEO_SIZE_REG: 0x%X,   0x%X \n", VIDEO_SIZE_REG, dwReg);
        reg_writel(VIDEO_SIZE_REG0+nId, dwReg);  //for Rev.B or later only
        //Scaler
        dwRegW &= 0x7FF;
        dwRegW = (720*256) / dwRegW;
        dwRegH = (dwRegH>>16)&0x1FF;
        // 60HZ video
        dwRegH = (240*256) / dwRegH;
/// 0915  rev B  black ....
        nScaler = VSCALE1_LO ;  /// + (nId<<4); //VSCALE1_LO + 0|0x10|0x20|0x30
        dwReg = dwRegH & 0xFF;  //V
        ///if(nId >= 4) DeviceWrite2864(nAddr,tmp);
        /// reg_writel(nScaler,  dwReg);
        nScaler++;  //VH
        dwReg = (((dwRegH >> 8)& 0xF) << 4) | ( (dwRegW>>8) & 0xF );
        ///if(nId >= 4) DeviceWrite2864(nAddr,tmp);
        /// reg_writel(nScaler,  dwReg);
        nScaler++;  //H
        dwReg = dwRegW & 0xFF;
        ///if(nId >= 4) DeviceWrite2864(nAddr,tmp);
        /// reg_writel(nScaler,  dwReg);
        //setup for Black stripe remover
        dwRegW = fh->width; /// -12;  //EndPos
        dwRegH = 4;		   //StartPos
        dwReg = (dwRegW - dwRegH)*(1<<16)/ fh->width;
        dwReg = (dwRegH & 0x1F) |
                ((dwRegH & 0x3FF) << 5) |
                (dwReg <<15);
        reg_writel(DROP_FIELD_REG0+ nId, 0xBFFFFFFF);   //28 // B 30 FPS
///  reg_writel( DROP_FIELD_REG0+ nId,  0xBFFFCFFF);    // 28  // 26 FPS   last xx FC
///reg_writel( DROP_FIELD_REG0+ nId,  0x8FFCFCFF);    // 28  // 26 FPS   last xx FC
////   reg_writel( DROP_FIELD_REG0+ nId,  0xBF3F3F3F);   // 24 FPS
///  - -  reg_writel( DROP_FIELD_REG0+ nId,  0x8FCFCFCF);   // 24 FPS
        dwReg2 = reg_readl(DMA_CH0_CONFIG+ 2);
        dwReg = reg_readl(DMA_CH0_CONFIG+ nId);
//        printk(" ********#### buffer_setup CH%d::   dwReg2: 0x%X   deReg 0x%X  \n", nId, dwReg2, dwReg );

	spin_unlock_irqrestore(&dev->slock, flags);
        return 0;
}


static void buffer_queue(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
        struct TW68_fh *fh = q->priv_data;
        struct TW68_buf *buf = container_of(vb,struct TW68_buf,vb);
        int nId = fh->DMA_nCH;

        TW68_buffer_queue(fh->dev,&fh->dev->vc[nId].video_dmaq, buf);
}


static void buffer_release(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
        struct TW68_buf *buf = container_of(vb,struct TW68_buf,vb);
//        printk("$$$$$$   buffer_release  ioctl  called \n");
        free_buffer(q, buf);
}


static struct videobuf_queue_ops video_qops = {
        .buf_setup = buffer_setup,
        .buf_prepare = buffer_prepare,
        .buf_queue = buffer_queue,
        .buf_release = buffer_release,
};

#if 0
// future VB2
#if LINUX_VERSION_CODE > KERNEL_VERSION(3,11,0)
static struct vb2_ops vidb2_ops = {
        .buf_setup = buffer2_setup,
        .buf_prepare = buffer2_prepare,
        .buf_queue = buffer2_queue,
        .buf_release = buffer2_release,
};
#endif
#endif
/* ------------------------------------------------------------------ */


int TW68_g_ctrl_internal(struct TW68_dev *dev, struct TW68_fh *fh, struct v4l2_control *c)
{
        const struct v4l2_queryctrl* ctrl;
        int DMA_nCH = fh->DMA_nCH;
        int nId = DMA_nCH & 0xF;
        int regval =0;
        unsigned long flags = 0;
        struct TW68_vc *vc = &dev->vc[nId];
        ctrl = ctrl_by_id(c->id);
        if (NULL == ctrl)
                return -EINVAL;
	

        switch (c->id) {
        case V4L2_CID_BRIGHTNESS:
                if (!vc->video_param.valid_brightness) {
                        spin_lock_irqsave(&dev->slock, flags);
                        if (DMA_nCH<4) {
                                regval = reg_readl(CH1_BRIGHTNESS_REG + DMA_nCH *0x10 );
                                regval = (regval + 0x80) &0xFF;
                        } else {
                                regval = reg_readl(CH1_BRIGHTNESS_REG + 0x100 + (DMA_nCH -4) *0x10 );
                                regval = (regval + 0x80) &0xFF;
                        }
                        spin_unlock_irqrestore(&dev->slock, flags);
                        vc->video_param.valid_brightness = 1;
                        vc->video_param.ctl_brightness = regval;
                }
                c->value = vc->video_param.ctl_brightness;
                break;
        case V4L2_CID_HUE:  ///-128 +127
                if (!vc->video_param.valid_hue) {
                        spin_lock_irqsave(&dev->slock, flags);
                        if (DMA_nCH<4) {
                                regval = reg_readl(CH1_HUE_REG + DMA_nCH *0x10 );
                        } else {
                                regval = reg_readl(CH1_HUE_REG + 0x100 + (DMA_nCH -4) *0x10 );
                        }
                        spin_unlock_irqrestore(&dev->slock, flags);
                        vc->video_param.valid_hue = 1;
                        if (regval < 0x80)
                                dev->vc[nId].video_param.ctl_hue = regval;
                        else
                                dev->vc[nId].video_param.ctl_hue = (regval -0x100); 
                }
                c->value = dev->vc[nId].video_param.ctl_hue;
                break;
        case V4L2_CID_CONTRAST:
                if (!vc->video_param.valid_contrast) {
                        spin_lock_irqsave(&dev->slock, flags);
                        if (DMA_nCH<4) {
                                regval = reg_readl(CH1_CONTRAST_REG + DMA_nCH *0x10 );
                        } else {
                                regval = reg_readl(CH1_CONTRAST_REG + 0x100 + (DMA_nCH -4) *0x10 );
                        }
                        spin_unlock_irqrestore(&dev->slock, flags);
                        vc->video_param.ctl_contrast = regval;
                        vc->video_param.valid_contrast = 1;
                }
                c->value = vc->video_param.ctl_contrast;
                break;
        case V4L2_CID_SATURATION:
                if (!vc->video_param.valid_saturation) {
                        spin_lock_irqsave(&dev->slock, flags);
                        if (DMA_nCH<4) {
                                regval = reg_readl(CH1_SAT_U_REG + DMA_nCH *0x10 );
                        } else {
                                regval = reg_readl(CH1_SAT_U_REG + 0x100 + (DMA_nCH -4) *0x10 );
                        }
                        spin_unlock_irqrestore(&dev->slock, flags);
                        vc->video_param.ctl_saturation = regval / 2;
                        vc->video_param.valid_saturation = 1;
                }
                c->value = vc->video_param.ctl_saturation;
                break;
        case V4L2_CID_AUDIO_MUTE:
                c->value = dev->vc[nId].video_param.ctl_mute;                
                break;
        case V4L2_CID_AUDIO_VOLUME:
                c->value = dev->vc[nId].video_param.ctl_volume;
                break;
        case V4L2_CID_PRIVATE_Y_EVEN:
                c->value = dev->vc[nId].video_param.ctl_y_even;
                break;
        case V4L2_CID_PRIVATE_Y_ODD:
                c->value = dev->vc[nId].video_param.ctl_y_odd;
                break;
        case V4L2_CID_PRIVATE_AUTOMUTE:
                c->value = dev->vc[nId].video_param.ctl_automute;                                
                break;
        case S812_CID_GPIO:
                regval = reg_readl(GPIO_REG_BANK_2);
                c->value = regval;
                break;                
        default:
                return -EINVAL;
        }
        //printk("  nId%d  TW68_g_ctrl_internal  Get_control name=%s val=%d  regval 0x%X \n", nId, ctrl->name,c->value, regval);
        return 0;
}


///EXPORT_SYMBOL_GPL(TW68_g_ctrl_internal);

static int TW68_g_ctrl(struct file *file, void *priv, struct v4l2_control *c)
{
        struct TW68_fh *fh = priv;

        return TW68_g_ctrl_internal(fh->dev, fh, c);
}

int TW68_s_ctrl_internal(struct TW68_dev *dev,  struct TW68_fh *fh, struct v4l2_control *c)
{
        const struct v4l2_queryctrl* ctrl;
        int DMA_nCH, nId, err;
        int regval =0;
	unsigned long flags = 0;
        DMA_nCH = fh->DMA_nCH;
        nId = DMA_nCH & 0xF;


        ctrl = ctrl_by_id(c->id);
        if (NULL == ctrl)
                goto error;

        //printk("    TW68_s_ctrl_internal  set_control name=%s val=%d\n",ctrl->name,c->value);

        switch (ctrl->type) {
        case V4L2_CTRL_TYPE_BOOLEAN:
        case V4L2_CTRL_TYPE_MENU:
        case V4L2_CTRL_TYPE_INTEGER:
                if (c->value < ctrl->minimum)
                        c->value = ctrl->minimum;
                if (c->value > ctrl->maximum)
                        c->value = ctrl->maximum;
                break;
        default:
                /* nothing */;
        };

        spin_lock_irqsave(&dev->slock, flags);
        switch (c->id) {
        case V4L2_CID_BRIGHTNESS:
                dev->vc[nId].video_param.ctl_saturation = c->value;
                dev->vc[nId].video_param.valid_saturation = 1;
                regval = ((c->value - 0x80))  &0xFF;
                if (DMA_nCH<4)
                {
                        reg_writel(CH1_BRIGHTNESS_REG + DMA_nCH *0x10, regval );
                }
                else
                {
                        if (DMA_nCH<8)
                        {
                                ///regval = c->value  &0xFF;
                                ///regval = regval * 4 /5;
                                ///DeviceWrite2864(dev, (EX_BRIGHTNESS1 | 0x100) +(DMA_nCH-4) *0x10, regval);
                                reg_writel(CH1_BRIGHTNESS_REG + 0x100 +(DMA_nCH -4) *0x10, regval);

                        }
                        else
                                if (DMA_nCH == 0xF)
                                {
                                        reg_writel(CH1_BRIGHTNESS_REG, regval);
                                        reg_writel(CH2_BRIGHTNESS_REG, regval);
                                        reg_writel(CH3_BRIGHTNESS_REG, regval);
                                        reg_writel(CH4_BRIGHTNESS_REG, regval);
                                }
                }
                break;
        case V4L2_CID_CONTRAST:
                dev->vc[nId].video_param.ctl_contrast = c->value;
                dev->vc[nId].video_param.valid_contrast = 1;
                if (DMA_nCH<4) {
                        reg_writel(CH1_CONTRAST_REG + DMA_nCH *0x10, c->value);
                }
                else
                {
                        if (DMA_nCH<8) {
                                ///	DeviceWrite2864(dev, (EX_CONTRAST1 | 0x100) +(DMA_nCH-4) *0x10, c->value);
                                reg_writel(CH1_CONTRAST_REG + 0x100 +(DMA_nCH -4) *0x10, c->value);
                        } else
                                if (DMA_nCH == 0xF)
                                {
                                        reg_writel(CH1_CONTRAST_REG, c->value);
                                        reg_writel(CH2_CONTRAST_REG, c->value);
                                        reg_writel(CH3_CONTRAST_REG, c->value);
                                        reg_writel(CH4_CONTRAST_REG, c->value);
                                }
                }
                break;

        case V4L2_CID_HUE:
                dev->vc[nId].video_param.ctl_hue = c->value;
                dev->vc[nId].video_param.valid_hue = 1;
                regval = c->value; //  &0xFF;
                if (DMA_nCH<4)
                {
                        reg_writel(CH1_HUE_REG + DMA_nCH *0x10, regval);
                }
                else
                {
			
                        if (DMA_nCH<8)
                        {
                                ///DeviceWrite2864(dev, (EX_HUE1 | 0x100) +(DMA_nCH-4) *0x10, regval);
                                reg_writel(CH1_HUE_REG + 0x100 +(DMA_nCH -4) *0x10, regval);
                        }
                        else
                                if (DMA_nCH == 0xF)
                                {
                                        reg_writel(CH1_HUE_REG, regval);
                                        reg_writel(CH2_HUE_REG, regval);
                                        reg_writel(CH3_HUE_REG, regval);
                                        reg_writel(CH4_HUE_REG, regval);
                                }
                }
                break;

        case V4L2_CID_SATURATION:
                dev->vc[nId].video_param.ctl_saturation = c->value;
                dev->vc[nId].video_param.valid_saturation = 1;
                regval = c->value *2;
                if (DMA_nCH<4)
                {
                        reg_writel(CH1_SAT_U_REG + DMA_nCH *0x10, regval);
                        reg_writel(CH1_SAT_V_REG + DMA_nCH *0x10, regval);
                }
                else
                {
                        if (DMA_nCH<8)
                        {
                                ///DeviceWrite2864(dev, (EX_SAT_U1 | 0x100) +(DMA_nCH-4) *0x10, regval);
                                ///DeviceWrite2864(dev, (EX_SAT_V1 | 0x100) +(DMA_nCH-4) *0x10, regval);
                                reg_writel(CH1_SAT_U_REG + 0x100 + (DMA_nCH -4)*0x10, regval);
                                reg_writel(CH1_SAT_V_REG + 0x100 + (DMA_nCH -4)*0x10, regval);

                        }
                        else
                                if (DMA_nCH == 0xF)
                                { // write wrong addr  hang
                                        reg_writel(CH1_SAT_U_REG, regval);
                                        reg_writel(CH1_SAT_V_REG, regval);
                                        reg_writel(CH2_SAT_U_REG, regval);
                                        reg_writel(CH2_SAT_V_REG, regval);
                                        reg_writel(CH3_SAT_U_REG, regval);
                                        reg_writel(CH3_SAT_V_REG, regval);
                                        reg_writel(CH4_SAT_U_REG, regval);
                                        reg_writel(CH4_SAT_V_REG, regval);
                                }
                }
                break;
        case V4L2_CID_AUDIO_MUTE:
                dev->vc[nId].video_param.ctl_mute = c->value;                                
                //TW68__setmute(dev);
                break;
        case V4L2_CID_AUDIO_VOLUME:
                dev->vc[nId].video_param.ctl_volume = c->value;
                //TW68_tvaudio_setvolume(dev,dev->ctl_volume);
                break;
        case V4L2_CID_PRIVATE_Y_EVEN:
                dev->vc[nId].video_param.ctl_y_even = c->value;
                break;
        case V4L2_CID_PRIVATE_Y_ODD:
                dev->vc[nId].video_param.ctl_y_odd = c->value;
                break;
        case V4L2_CID_PRIVATE_AUTOMUTE:       
                dev->vc[nId].video_param.ctl_automute = c->value;
                break;        
        case S812_CID_GPIO:                
                regval = (c->value);
                reg_writel(GPIO_REG_BANK_2, regval );
                break;
        default:
                goto error;
        }
        err = 0;
        //printk("    TW68_s_ctrl_internal  set_control name=%s REAL val=%d   reg  0x%X\n",ctrl->name,c->value,  regval);

error:
	spin_unlock_irqrestore(&dev->slock, flags);
        return err;
}


///EXPORT_SYMBOL_GPL(TW68_s_ctrl_internal);

static int TW68_s_ctrl(struct file *file, void *f, struct v4l2_control *c)
{
        struct TW68_fh *fh = f;

        return TW68_s_ctrl_internal(fh->dev, fh, c);
}

/* ------------------------------------------------------------------ */

static struct videobuf_queue* TW68_queue(struct TW68_fh *fh)
{
        struct videobuf_queue* q = NULL;

        switch (fh->type) {
        case V4L2_BUF_TYPE_VIDEO_CAPTURE:
                q = &fh->vidq;
                //printk(KERN_INFO " videobuf_queue: V4L2_BUF_TYPE_VIDEO_CAPTURE %p\n", q);
                break;
        case V4L2_BUF_TYPE_VBI_CAPTURE:
                q = &fh->vbiq;
                printk(KERN_INFO " videobuf_queue: V4L2_BUF_TYPE_VBI_CAPTURE .\n");
                break;
        default:
                printk(KERN_INFO " videobuf_queue: not valid type,  break out  \n");
                BUG();
        }
        return q;
}
#if 0
////////////////////////////////////////////////////////////////////////////
static int TW68_resource(struct TW68_fh *fh)
{
        if (fh->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
                return RESOURCE_VIDEO;

        if (fh->type == V4L2_BUF_TYPE_VBI_CAPTURE)
                return RESOURCE_VBI;

        BUG();
        return 0;
}
#endif
////////////////////////////////////////////////////////////////////////////

static int video_open(struct file *file)
{
        int minor = video_devdata(file)->minor;
        struct TW68_dev *dev;
        struct TW68_fh *fh;
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        unsigned int request =0;
        unsigned int dmaCH;

        int k;  //, used;

        mutex_lock(&TW686v_devlist_lock);

        list_for_each_entry(dev, &TW686v_devlist, devlist) {
                for (k=0; k<8; k++)    // 8 - 9
                {
                        if (dev->vc[k].vdev.minor == minor) {
                                mutex_unlock(&TW686v_devlist_lock);
                                goto found;
                        }
                }
        }
	//if (dev->vbi_dev && (dev->vbi_dev->minor == minor)) {
	//	type = V4L2_BUF_TYPE_VBI_CAPTURE;
	//	goto found;
	//}
        printk(KERN_INFO "video_open  no real device found XXXX \n");
        mutex_unlock(&TW686v_devlist_lock);
        return -ENODEV;
found:
//    printk("opening %d\n", k);

        request = 1 << k;

        // check video decoder video standard and change default tvnormf
        dmaCH = k;

        //printk(KERN_INFO "video_open ID:%d  dmaCH %x   request %X \n",  k, dmaCH, request);

        /* allocate + initialize per filehandle data */
        fh = kzalloc(sizeof(*fh),GFP_KERNEL);
        if (NULL == fh)
                return -ENOMEM;

        if (dev->vc[k].viddetected == 0) {
#if defined (NTSC_STANDARD_NODETECT)
		printk("NTSC standard\n");
                dev->vc[k].tvnormf = &tvnorms[4];
                dev->vc[k].PAL50 = 0;
                fh->dW = NTSC_default_width;
                fh->dH = NTSC_default_height;
#elif defined (PAL_STANDARD_NODETECT)
		printk("PAL standard\n");
                dev->vc[k].tvnormf = &tvnorms[0];
                dev->vc[k].PAL50 = 1;
                fh->dW = PAL_default_width;
                fh->dH = PAL_default_height;
#else
		switch (vidstd_open) {
		case 1:
			printk("NTSC standard\n");
			dev->vc[k].tvnormf = &tvnorms[4];
			dev->vc[k].PAL50 = 0;
			fh->dW = NTSC_default_width;
			fh->dH = NTSC_default_height;
			break;
		case 2:
			printk("PAL standard\n");
			dev->vc[k].tvnormf = &tvnorms[0];
			dev->vc[k].PAL50 = 1;
			fh->dW = PAL_default_width;
			fh->dH = PAL_default_height;
			break;			
		default:
			printk("vidstd auto-detect\n");
			if (VideoDecoderDetect(dev, dmaCH) == 50) {
				dev->vc[k].tvnormf = &tvnorms[0];
				dev->vc[k].PAL50 = 1;
				fh->dW = PAL_default_width;
				fh->dH = PAL_default_height;
			} else {
				dev->vc[k].tvnormf = &tvnorms[4];
				dev->vc[k].PAL50 = 0;
				fh->dW = NTSC_default_width;
				fh->dH = NTSC_default_height;
			}
			break;
		}
#endif
                dev->vc[k].viddetected = 1;
                if (dev->vc[k].PAL50) {
                        reg_writel(MISC_CONTROL3, 0xc5);	
                        reg_writel(MISC_CONTROL3+ 0x100, 0xc5);	
                } else {
                        reg_writel(MISC_CONTROL3, 0x85);	
                        reg_writel(MISC_CONTROL3+ 0x100, 0x85);
                }
                if (k <= 3) {
                        reg_writel(DECODER0_SDT+ (k*0x10), dev->vc[k].PAL50 ? 1 : 0);
                } else {
                        reg_writel(DECODER0_SDT+0x100+ ((k-4)*0x10), dev->vc[k].PAL50 ? 1 : 0);
                }
		
        } else {
                if (dev->vc[k].PAL50) {
                        fh->dW = PAL_default_width;
                        fh->dH = PAL_default_height;
                } else {
                        fh->dW = NTSC_default_width;
                        fh->dH = NTSC_default_height;
                }

        }


        fh->vc = &dev->vc[k];
        /*printk( "\n&&&&&&&&&&&&&&&  0X%p  open /dev/video%d  minor%d type=%s k:%d   tvnorm::%s  %d\n", dev, dev->vc[k].vdev.num,  minor, //  0901 array
                v4l2_type_names[V4L2_BUF_TYPE_VIDEO_CAPTURE], k, dev->vc[k].tvnormf->name,
                dev->vc[k].PAL50);
        */

        file->private_data = fh;
        fh->dev = dev;
        mod_timer(&dev->delay_resync, jiffies+ msecs_to_jiffies(100));
        fh->DMA_nCH = k;//dev->vc[k].video_dmaq.DMA_nCH;  ///  k;    /// DMA index 
        fh->type = type;
        /// fh->fmt      = format_by_fourcc(V4L2_PIX_FMT_UYVY);   /// RGB24
        fh->fmt = format_by_fourcc(V4L2_PIX_FMT_YUYV);   /// YUY2 by default
        fh->width = fh->dW;  //704;  //720;
        fh->height = fh->dH;  //576;
        fh->vidq.field = 0;
        //printk( KERN_INFO "open minor=%d  fh->DMA_nCH= 0x%X type=%s\n",minor, fh->DMA_nCH, v4l2_type_names[type]);
        /// printk(KERN_INFO "fh initialized to PAL frame buffer  successful! \n");

        videobuf_queue_vmalloc_init(&fh->vidq, &video_qops,
                                    NULL, &fh->vc->slock,
                                    V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                    V4L2_FIELD_INTERLACED,
                                    sizeof(struct TW68_buf),
                                    fh
#ifdef HAVE_VIDEOBUF_VMALLOC_EXT_LOCK
                                    , NULL	/* ext_lock */
#endif
		);

        //printk(KERN_INFO "videobuf_queue_sg_init ()  successful! \n");
        /// video_mux(dev,dev->ctl_input);
        return 0;
} 



static ssize_t
video_read(struct file *file, char __user *data, size_t count, loff_t *ppos)
{
        struct TW68_fh *fh = file->private_data;

        switch (fh->type) {
        case V4L2_BUF_TYPE_VIDEO_CAPTURE:
                if (res_locked(fh->vc, RESOURCE_VIDEO))
                        return -EBUSY;

                return videobuf_read_one(TW68_queue(fh),
                                         data, count, ppos,
                                         file->f_flags & O_NONBLOCK);
#if 0
        case V4L2_BUF_TYPE_VBI_CAPTURE:
                if (!res_get(fh->dev,fh,RESOURCE_VBI))
                        return -EBUSY;
                return videobuf_read_stream(TW68_queue(fh),
                                            data, count, ppos, 1,
                                            file->f_flags & O_NONBLOCK);
#endif
                break;
        default:
                BUG();
                return 0;
        }
}


static unsigned int
video_poll(struct file *file, struct poll_table_struct *wait)
{ 
        struct TW68_fh *fh = file->private_data;
        struct videobuf_buffer *buf = NULL;
        unsigned int rc = 0;

	
        if (V4L2_BUF_TYPE_VIDEO_CAPTURE != fh->type)
                return POLLERR;

        if (V4L2_BUF_TYPE_VBI_CAPTURE == fh->type)
                return videobuf_poll_stream(file, &fh->vbiq, wait);

        if (res_check(fh, RESOURCE_VIDEO)) {
                mutex_lock(&fh->vidq.vb_lock);
                if (!list_empty(&fh->vidq.stream))
                        buf = list_entry(fh->vidq.stream.next, struct videobuf_buffer, stream);
        } else {
                mutex_lock(&fh->vidq.vb_lock);
                if (UNSET == fh->vidq.read_off) {
                        /* need to capture a new frame */
                        if (res_locked(fh->vc, RESOURCE_VIDEO)) {
                                goto err;
                        }
                        if (0 != fh->vidq.ops->buf_prepare(&fh->vidq,fh->vidq.read_buf,fh->vidq.field)) {
                                goto err;
                        }
                        fh->vidq.ops->buf_queue(&fh->vidq,fh->vidq.read_buf);
                        fh->vidq.read_off = 0;
                }
                buf = fh->vidq.read_buf;
        }

        if (!buf)
                goto err;

        poll_wait(file, &buf->done, wait);
        if (buf->state == VIDEOBUF_DONE ||
            buf->state == VIDEOBUF_ERROR)
                rc = POLLIN|POLLRDNORM;
        mutex_unlock(&fh->vidq.vb_lock);
        return rc;
err:
        mutex_unlock(&fh->vidq.vb_lock);
        return POLLERR;
}


static int video_release(struct file *file)
{
//        int minor = video_devdata(file)->minor;
        struct TW68_fh *fh = file->private_data;
        struct TW68_dev *dev = fh->dev;
        int DMA_nCH = fh->DMA_nCH;
        int nId = DMA_nCH;
        unsigned long flags = 0;
        //printk(KERN_INFO "video_release ()  CALLED  minor:%d  DMA_nCH: %X    video_fieldcount 0x%X ! \n",  minor, DMA_nCH, dev->vc[DMA_nCH].video_fieldcount);    /// 0-7
        /// printk(KERN_INFO " turn off overlay  CALLED   ! \n");
        //dev->video_opened &= ~( 1 << DMA_nCH );   /// set opened flag free
        mod_timer(&dev->delay_resync, jiffies+ msecs_to_jiffies(100));
        if (res_check(fh, RESOURCE_VIDEO)) {
                spin_lock_irqsave(&dev->vc[nId].slock, flags);
                dev->vc[nId].is_streaming = 0;
                spin_unlock_irqrestore(&dev->vc[nId].slock, flags);
                res_free(fh->vc, fh, RESOURCE_VIDEO);

                mutex_lock(&dev->start_lock);
                spin_lock_irqsave(&dev->slock, flags);
                stop_video_DMA(dev, DMA_nCH );				//  fh->DMA_nCH  = DMA ID
                dev->vc[nId].video_fieldcount = 0;
                spin_unlock_irqrestore(&dev->slock, flags);
                videobuf_streamoff(&fh->vidq);
                videobuf_mmap_free(&fh->vidq);
                del_timer(&dev->vc[nId].video_dmaq.timeout);

                // give device time to stop
                msleep(41);
                mutex_unlock(&dev->start_lock);
        }
	//	mutex_lock(&dev->start_lock);
	//msleep(2);
	//	mutex_unlock(&dev->start_lock);
        // printk(KERN_INFO " stop video capture  CALLED   ! \n");
        if (fh->vidq.read_buf) {
                buffer_release(&fh->vidq,fh->vidq.read_buf);
                kfree(fh->vidq.read_buf);
        }
        file->private_data = NULL;
        kfree(fh);
        //printk(KERN_INFO " video_release   kfree(fh);  CALLED   ! \n");
        return 0;
}

static int video_mmap(struct file *file, struct vm_area_struct * vma)
{
        struct TW68_fh *fh = file->private_data;
        int rc;
        struct videobuf_queue *q;
        q = TW68_queue(fh);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 11, 0)
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0)
        // workaround deadlock bug in 3.11 kernel
        mutex_lock(&q->vb_lock);
        q->ext_lock= (struct mutex *)1;
#endif
#endif
        //printk("video mmap\n");//printk("video_mmap :: vma= %p  %p    %d \n", vma->vm_start, vma->vm_end, (vma->vm_end - vma->vm_start) );
        rc = videobuf_mmap_mapper(q, vma);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 11, 0)
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0)
        // workaround deadlock bug in 3.11 kernel
        q->ext_lock= NULL;
        mutex_unlock(&q->vb_lock);
#endif
#endif
        return rc;
}

/* ------------------------------------------------------------------ */
#if 0
static int TW68_try_get_set_fmt_vbi_cap(struct file *file, void *priv,
                                        struct v4l2_format *f)
{
        struct TW68_fh *fh = priv;
        struct TW68_dev *dev = fh->dev;
        struct TW68_tvnorm *norm = dev->tvnorm;

        f->fmt.vbi.sampling_rate = 6750000 * 4;
        f->fmt.vbi.samples_per_line = 2048 /* VBI_LINE_LENGTH */;
        f->fmt.vbi.sample_format = V4L2_PIX_FMT_GREY;
        f->fmt.vbi.offset = 64 * 4;
        f->fmt.vbi.start[0] = norm->vbi_v_start_0;
        f->fmt.vbi.count[0] = norm->vbi_v_stop_0 - norm->vbi_v_start_0 +1;
        f->fmt.vbi.start[1] = norm->vbi_v_start_1;
        f->fmt.vbi.count[1] = f->fmt.vbi.count[0];
        f->fmt.vbi.flags = 0; /* VBI_UNSYNC VBI_INTERLACED */

        return 0;
}
#endif
static int TW68_g_fmt_vid_cap(struct file *file, void *priv,
                              struct v4l2_format *f)
{
        struct TW68_fh *fh = priv;

        // printk( " _g_fmt_vid_cap: TW68_fh %x || v4l2_format %x  fmt %x\n", priv, f, f->fmt);

        f->fmt.pix.width = fh->width;
        f->fmt.pix.height = fh->height;
        f->fmt.pix.field = fh->vidq.field;
        f->fmt.pix.pixelformat = fh->fmt->fourcc;
        f->fmt.pix.bytesperline =
                (f->fmt.pix.width * fh->fmt->depth) >> 3;
        f->fmt.pix.sizeimage =
                f->fmt.pix.height * f->fmt.pix.bytesperline;

        f->fmt.pix.colorspace = V4L2_COLORSPACE_SMPTE170M;
//        printk( " _g_fmt_vid_cap: width %d  height %d \n", f->fmt.pix.width, f->fmt.pix.height);
        f->fmt.pix.priv = 0;
        return 0;
}
#if 0
static int TW68_g_fmt_vid_overlay(struct file *file, void *priv,
                                  struct v4l2_format *f)
{
        struct TW68_fh *fh = priv;

        TW68_g_fmt_vid_cap(file, priv, f);
        return 0;


        /*
          if (TW68_no_overlay > 0) {
          printk(KERN_ERR "V4L2_BUF_TYPE_VIDEO_OVERLAY: no_overlay\n");
          return -EINVAL;
          }
          f->fmt.win = fh->win;
        */

        f->fmt.pix.width = fh->width;
        f->fmt.pix.height = fh->height;
        f->fmt.pix.field = fh->vidq.field;
        f->fmt.pix.pixelformat = fh->fmt->fourcc;
        f->fmt.pix.bytesperline =
                (f->fmt.pix.width * fh->fmt->depth) >> 3;
        f->fmt.pix.sizeimage =
                f->fmt.pix.height * f->fmt.pix.bytesperline;

        printk( " overlay _g_fmt_vid_cap: width %d  height %d \n", f->fmt.pix.width, f->fmt.pix.height);
        return 0;
}
#endif


static int TW68_try_fmt_vid_cap(struct file *file, void *priv,
                                struct v4l2_format *f)
{
        struct TW68_fh *fh = priv;
        struct TW68_dev *dev = fh->dev;
        struct TW68_format *fmt;
        enum v4l2_field field;
        unsigned int maxw, maxh;
        u32     k;
        u32 	nId = fh->DMA_nCH;

        fmt = format_by_fourcc(f->fmt.pix.pixelformat);
//        printk( "TW68 input  nId:%x   try_fmt:: %x  | width %d  height %d\n", 
        //              nId, f->fmt.pix.pixelformat, f->fmt.pix.width, f->fmt.pix.height  );


        if (NULL == fmt ) {
                //printk( "TW68 fmt:: no valid pixel format \n");
                return -EINVAL;
        }


        switch (fmt->fourcc) {
        case V4L2_PIX_FMT_YUYV:
        case V4L2_PIX_FMT_UYVY:
        case V4L2_PIX_FMT_RGB565:
                break;
        default:
                return -EINVAL;
        }

        k = nId;

        if (dev->vc[k].tvnormf->id & V4L2_STD_525_60) {
                maxw = 720;
                maxh = 480;
        } else {
                maxw = 720;
                maxh = 576;
        }
/*
        printk( "TW68 _try_fmt_vid_cap  tvnormf %d->name %s  id %X   maxh %d \n", nId,
                dev->vc[nId].tvnormf->name,
                (unsigned int)dev->vc[nId].tvnormf->id, maxh);
*/
        field = f->fmt.pix.field;

        if (V4L2_FIELD_ANY == field) {
                field = (f->fmt.pix.height > maxh/2)
                        ? V4L2_FIELD_INTERLACED
                        : V4L2_FIELD_BOTTOM;
                //printk("field now %d, h %d, max %d\n", field,f->fmt.pix.height, maxh/2);
		
        }
        switch (field) {
        case V4L2_FIELD_TOP:
        case V4L2_FIELD_BOTTOM:
        case V4L2_FIELD_ALTERNATE:
                maxh = maxh / 2;
		if (f->fmt.pix.height > maxh) {
			field = V4L2_FIELD_INTERLACED;
			//printk("forcing interlace %d\n", field);
		}
                break;
        case V4L2_FIELD_INTERLACED:
                break;
        default:
                return -EINVAL;
        }
        f->fmt.pix.field = field;

        ///printk( "TW68 field: %d _fmt: width %d  height %d \n", field, f->fmt.pix.width, f->fmt.pix.height);
//        printk( "TW68 _try_fmt_vid_cap fmt::pixelformat %x  field: %d _fmt: width %d  height %d \n", fmt->fourcc, field, f->fmt.pix.width, f->fmt.pix.height);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
        v4l_bound_align_image(&f->fmt.pix.width, 128, maxw, 2,    // 4 pixel  test 360  4,
                              &f->fmt.pix.height, 60, maxh, 0, 0);
#endif
        f->fmt.pix.bytesperline =
                (f->fmt.pix.width * fmt->depth) >> 3;
        f->fmt.pix.sizeimage =
                f->fmt.pix.height * f->fmt.pix.bytesperline;

        //printk( "TW68 _try_fmt_vid_cap: width %d  height %d      %d  %d \n", f->fmt.pix.width, f->fmt.pix.height, maxw, maxh);
        f->fmt.pix.priv = 0;
        f->fmt.pix.colorspace = V4L2_COLORSPACE_SMPTE170M;
        return 0;
}

static int TW68_s_fmt_vid_cap(struct file *file, void *priv,
                              struct v4l2_format *f)
{
        struct TW68_fh *fh = priv;
        struct TW68_dev *dev = fh->dev;
        int err;
        u32 nId = fh->DMA_nCH;
        struct TW68_format *fmt = format_by_fourcc(f->fmt.pix.pixelformat);
        //printk(KERN_ERR "~~~~~~~~~~~~~TW68_s_fmt_vid_cap: %x  W:%d H:%d  field:%X\n", f->fmt.pix.pixelformat, fh->width, fh->height, fh->vidq.field);

        if (res_locked(fh->vc, RESOURCE_VIDEO))
                return -EBUSY;

        err = TW68_try_fmt_vid_cap(file, priv, f);
        if (0 != err)
                return err;

        switch (fmt->fourcc) {
        case V4L2_PIX_FMT_YUYV:
                dev->vc[nId].nVideoFormat = VIDEO_FORMAT_YUYV;
                break;
        case V4L2_PIX_FMT_UYVY:
                dev->vc[nId].nVideoFormat = VIDEO_FORMAT_UYVY;
                break;
        case V4L2_PIX_FMT_RGB565:
                dev->vc[nId].nVideoFormat = VIDEO_FORMAT_RGB565;
                break;
        default:
                printk(KERN_INFO "s_fmt: invalid\n");
                return -EINVAL;
        }

        fh->fmt = format_by_fourcc(f->fmt.pix.pixelformat);
        fh->width = f->fmt.pix.width;
        fh->height = f->fmt.pix.height;
        fh->vidq.field = f->fmt.pix.field;

//        printk(KERN_ERR "~~~~~~~~~~~~~TW68_s_fmt_vid_cap: fh->fmt   W:%d H:%d  field:%X\n",  fh->width, fh->height, fh->vidq.field);

        return 0;
}

#if 0
static int TW68_s_fmt_vid_overlay(struct file *file, void *priv,
                                  struct v4l2_format *f)
{
        //printk( " TW68_s_fmt_vid_overlay   called #### \n" );
        TW68_try_fmt_vid_cap( file, priv, f);
        return 0;
}
#endif

int TW68_queryctrl(struct file *file, void *priv, struct v4l2_queryctrl *c)
{
        const struct v4l2_queryctrl *ctrl;

        if ((c->id <  V4L2_CID_BASE ||
             c->id >= V4L2_CID_LASTP1) &&
            (c->id <  V4L2_CID_PRIVATE_BASE ||
             c->id >= V4L2_CID_PRIVATE_LASTP1))
                return -EINVAL;
        ctrl = ctrl_by_id(c->id);
        *c = (NULL != ctrl) ? *ctrl : no_ctrl;
        return 0;
}
///EXPORT_SYMBOL_GPL(TW68_queryctrl);

static int TW68_enum_input(struct file *file, void *priv,
                           struct v4l2_input *i)
{
        struct TW68_fh *fh = priv;
        struct TW68_dev *dev = fh->dev;
        unsigned int n;
	unsigned int vidstat;
        int nId = fh->DMA_nCH; // channel
        n = i->index;
//        printk(KERN_INFO " TW68_enum_input   i=%p  n:%x CALLED   ! \n", i, n);
        if (n >= 4)
                return -EINVAL;
        if (NULL == card_in(dev, i->index).name)
                return -EINVAL;
        memset(i, 0, sizeof(*i));
        i->index = n;
        i->type = V4L2_INPUT_TYPE_CAMERA;
        strcpy(i->name, card_in(dev, n).name);
        if (n == dev->ctl_input) {
        }
        i->std = TW68_NORMS;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
	i->capabilities = V4L2_IN_CAP_STD;
#endif
        i->type = V4L2_INPUT_TYPE_CAMERA;
        sprintf(i->name, "Composite%d", n);

	i->status = 0;

	vidstat = reg_readl(VIDSTAT[nId]);
	if (vidstat & TW686X_VIDSTAT_VDLOSS)
		i->status |= V4L2_IN_ST_NO_SIGNAL;
	if (!(vidstat & TW686X_VIDSTAT_HLOCK))
		i->status |= V4L2_IN_ST_NO_H_LOCK;

        return 0;
}

static int TW68_g_input(struct file *file, void *priv, unsigned int *i)
{
        struct TW68_fh *fh = priv;
        struct TW68_dev *dev = fh->dev;

        *i = dev->ctl_input;
        return 0;
}

static int TW68_s_input(struct file *file, void *priv, unsigned int i)
{
        struct TW68_fh *fh = priv;
        struct TW68_dev *dev = fh->dev;
        // int err;

//        printk(KERN_INFO " TW68_s_input   i=%d  CALLED   ! \n", i);

        if (i < 0  ||  i >= TW68_INPUT_MAX)
                return -EINVAL;
        if (NULL == card_in(dev, i).name)
                return -EINVAL;

//        printk(KERN_INFO " TW68_s_input card in name: %s  i=%d  CALLED   ! \n", card_in(dev, i).name, i);
          return 0;    /////  ffmpeg input;
}

static int TW68_querycap(struct file *file, void  *priv,
                         struct v4l2_capability *cap)
{
        struct TW68_fh *fh = priv;
        struct TW68_dev *dev = fh->dev;
        strcpy(cap->driver, "TW6869");
        strlcpy(cap->card, TW68_boards[dev->board].name,
                sizeof(cap->card));
        sprintf(cap->bus_info, "PCI:%s", pci_name(dev->pci));
        //cap->version = TW68_VERSION_CODE;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 0)
        cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;
#else
        cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;
        cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
#endif
        return 0;
}

int TW68_s_std_internal(struct TW68_dev *dev, struct TW68_fh *fh, v4l2_std_id *id)
{
        unsigned int i, nId;
        v4l2_std_id fixup;

        nId = fh->DMA_nCH;
        for (i = 0; i < TVNORMS; i++)
                if (*id == tvnorms[i].id)
                        break;

        if (i == TVNORMS)
                for (i = 0; i < TVNORMS; i++)
                        if (*id & tvnorms[i].id)
                                break;
        if (i == TVNORMS)
                return -EINVAL;

        if ((*id & V4L2_STD_SECAM) && (secam[0] != '-')) {
                if (secam[0] == 'L' || secam[0] == 'l') {
                        if (secam[1] == 'C' || secam[1] == 'c')
                                fixup = V4L2_STD_SECAM_LC;

                        else
                                fixup = V4L2_STD_SECAM_L;
                } else {
                        if (secam[0] == 'D' || secam[0] == 'd')
                                fixup = V4L2_STD_SECAM_DK;
                        else
                                fixup = V4L2_STD_SECAM;
                }
                for (i = 0; i < TVNORMS; i++)
                        if (fixup == tvnorms[i].id)
                                break;
        }

        *id = tvnorms[i].id;
        if (*id != V4L2_STD_NTSC) {
                dev->vc[nId].PAL50 = 1;
                reg_writel(MISC_CONTROL3, 0xc5);	
                reg_writel(MISC_CONTROL3+ 0x100, 0xc5);
                if (nId <= 3) {
                        reg_writel(DECODER0_SDT+ (nId*0x10), 1);		/// 0 NTSC	
                } else {
                        reg_writel(DECODER0_SDT+0x100+ ((nId-4)*0x10), 1);		/// 0 NTSC	
                }
        } else {
                dev->vc[nId].PAL50 = 0;
                reg_writel(MISC_CONTROL3, 0x85);	
                reg_writel(MISC_CONTROL3+ 0x100, 0x85);
                if (nId <= 3) {
                        reg_writel(DECODER0_SDT+ (nId*0x10), 0);		/// 0 NTSC	
                } else {
                        reg_writel(DECODER0_SDT+0x100+ ((nId-4)*0x10), 0);		/// 0 NTSC	
                }
        }
//        printk(KERN_INFO " TW68__s_std_internal: *id = %x  i= %x \n", (int)*id, i);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
        mutex_lock(&fh->vc->lock);
#endif
        set_tvnorm(dev, &tvnorms[i]);
        dev->vc[nId].tvnormf = &tvnorms[i];
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
        mutex_unlock(&fh->vc->lock);
#endif
        return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
static int TW68_s_std(struct file *file, void *priv, v4l2_std_id id)
#else
static int TW68_s_std(struct file *file, void *priv, v4l2_std_id *id)
#endif
{
        struct TW68_fh *fh = priv;
/*	 	
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
        printk( "%s, _s_std v4l2_std_id =%d \n", __func__, (int)id);
#else
        printk( "%s, _s_std v4l2_std_id =%d \n", __func__, (int)*id);
#endif
*/
        if (res_locked(fh->vc, RESOURCE_VIDEO))
                return -EBUSY;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
        return TW68_s_std_internal(fh->dev, fh, &id);
#else
        return TW68_s_std_internal(fh->dev, fh, id);
#endif
}


static int TW68_g_std(struct file *file, void *priv, v4l2_std_id *id)
{
        struct TW68_fh *fh = priv;
        struct TW68_dev *dev = fh->dev;
        int nId = fh->DMA_nCH;
        *id = dev->tvnorm->id;
        *id = dev->vc[nId].tvnormf->id;
//        printk( "%s, _g_std v4l2_std_id =%d \n", __func__, (int)*id);
        return 0;
}

static int TW68_cropcap(struct file *file, void *priv,
                        struct v4l2_cropcap *cap)
{
        struct TW68_fh *fh = priv;
        struct TW68_dev *dev = fh->dev;

        /// 0824 TVtime  //printk(KERN_INFO " ============ TW68_cropcap  CALLED   ! \n");

        if (cap->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
                return -EINVAL;

        cap->bounds = dev->crop_bounds;
        cap->defrect = dev->crop_defrect;
        cap->pixelaspect.numerator = 1;
        cap->pixelaspect.denominator = 1;

        ///printk(" TW68_cropcap dev->tvnorm->id:0x%X\n\n", dev->tvnorm->id);
        if (dev->tvnorm->id & V4L2_STD_525_60) {
                cap->pixelaspect.numerator = 11;
                cap->pixelaspect.denominator = 10;
        }
        if (dev->tvnorm->id & V4L2_STD_625_50) {
                cap->pixelaspect.numerator = 54;
                cap->pixelaspect.denominator = 59;
        }

        ///printk(KERN_INFO " TW68__cropcap   successful   ! \n");
        return 0;
}

static int TW68_g_crop(struct file *file, void *f, struct v4l2_crop *crop)
{
        struct TW68_fh *fh = f;
        struct TW68_dev *dev = fh->dev;

        printk( "========TW68__g_crop \n");

        if (crop->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
                return -EINVAL;
        crop->c = dev->crop_current;
        return 0;
}

static int TW68_s_crop(struct file *file, void *f, V4L_CONST struct v4l2_crop *crop)
{
        struct TW68_fh *fh = f;
        struct TW68_dev *dev = fh->dev;
        struct v4l2_rect *b = &dev->crop_bounds;
        struct v4l2_rect c = crop->c;

        printk( "========TW68__s_crop \n");

        if (crop->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
                return -EINVAL;
        if (c.height < 0)
                return -EINVAL;
        if (c.width < 0)
                return -EINVAL;

        if (res_locked(fh->vc, RESOURCE_VIDEO))
                return -EBUSY;

        if (c.top < b->top)
                c.top = b->top;
        if (c.top > b->top + b->height)
                c.top = b->top + b->height;
        if (c.height > b->top - c.top + b->height)
                c.height = b->top - c.top + b->height;

        if (c.left < b->left)
                c.left = b->left;
        if (c.left > b->left + b->width)
                c.left = b->left + b->width;
        if (c.width > b->left - c.left + b->width)
                c.width = b->left - c.left + b->width;

        dev->crop_current = c;
        return 0;
}


static int TW68_g_audio(struct file *file, void *priv, struct v4l2_audio *a)
{
        strcpy(a->name, "audio");
        return 0;
}

static int TW68_s_audio(struct file *file, void *priv, V4L_CONST struct v4l2_audio *a)
{
        return 0;
}


static int TW68_enum_fmt_vid_cap(struct file *file, void  *priv,
                                 struct v4l2_fmtdesc *f)
{

        if (f->index >= FORMATS)
                return -EINVAL;

        strlcpy(f->description, formats[f->index].name,
                sizeof(f->description));

        f->pixelformat = formats[f->index].fourcc;
//        printk( "========TW68__enum_fmt_vid_cap  description %s \n", f->description);

        return 0;
}

#if 0
static int TW68_enum_fmt_vid_overlay(struct file *file, void  *priv,
                                     struct v4l2_fmtdesc *f)
{
        printk( "========TW68__enum_fmt_vid_overlay \n");

        if (TW68_no_overlay > 0) {
                printk(KERN_ERR "V4L2_BUF_TYPE_VIDEO_OVERLAY: no_overlay\n");
                return -EINVAL;
        }

        if ((f->index >= FORMATS) || formats[f->index].planar)
                return -EINVAL;

        strlcpy(f->description, formats[f->index].name,
                sizeof(f->description));

        f->pixelformat = formats[f->index].fourcc;

        return 0;
}
#endif


#ifdef CONFIG_VIDEO_V4L1_COMPAT
static int vidiocgmbuf(struct file *file, void *priv, struct video_mbuf *mbuf)
{
        struct TW68_fh *fh = file->private_data;
        return videobuf_cgmbuf(TW68_queue(fh), mbuf, 8);
}
#endif

static int TW68_reqbufs(struct file *file, void *priv,
                        struct v4l2_requestbuffers *p)
{
        struct TW68_fh *fh = priv;
        int rc;
        rc = videobuf_reqbufs(TW68_queue(fh), p);
        return rc;
}

static int TW68_querybuf(struct file *file, void *priv,
                         struct v4l2_buffer *b)
{
        struct TW68_fh *fh = priv;
        int rc;
        rc = videobuf_querybuf(TW68_queue(fh), b);
        return rc;
}

static int TW68_qbuf(struct file *file, void *priv, struct v4l2_buffer *b)
{
        struct TW68_fh *fh = priv;
	int rc;
        struct videobuf_queue* q = NULL;
        q = TW68_queue(fh);

	//	printk(KERN_INFO " ++TW68__qbuf videobuf_queue: 0X%p   v4l2_buffer: 0X%p.\n", q, b);

        rc = videobuf_qbuf(q, b);
	return rc;
}

static int TW68_dqbuf(struct file *file, void *priv, struct v4l2_buffer *b)
{
        struct TW68_fh *fh = priv;

        struct videobuf_queue* q = NULL;
        q = &fh->vidq;
	//printk(KERN_INFO " --TW68__dqbuf videobuf_queue: 0X%p  0X%p  v4l2_buffer: 0X%p.\n", q, TW68_queue(fh), b);
        return videobuf_dqbuf( q, b,		file->f_flags & O_NONBLOCK);
}

static int TW68_streamon(struct file *file, void *priv,
                         enum v4l2_buf_type type)
{
        struct TW68_fh *fh = priv;
        struct TW68_dev *dev = fh->dev;
        struct videobuf_queue *q;
        int streaming;
        u32  nId;
        unsigned long flags = 0;
        if (!res_get(fh->vc, fh, RESOURCE_VIDEO)) {
                printk("stream busy!\n");
                return -EBUSY;
        }
        mod_timer(&dev->delay_resync, jiffies+ msecs_to_jiffies(100));
        mutex_lock(&dev->start_lock);
        // fh->resources |= res;
        nId = fh->DMA_nCH;

        dev->vc[nId].Done = 0;
        q = TW68_queue(fh);

        //TW68_buffer_requeue(dev, &dev->video_dmaq[nId]);

        streaming = videobuf_streamon(q);
        //printk(KERN_INFO "%s: videobuf_streamon(TW68_queue(fh)) DMA %d  q->streaming:%X  streaming:%x.\n", dev->name, fh->DMA_nCH,  q->streaming, streaming);

        ///////////////////////////////////////////

        //dwReg2 = reg_readl(DMA_CH0_CONFIG+ 2);
        //dwReg = reg_readl(DMA_CH0_CONFIG+ nId);

        //printk(" ********####  streamon CH%d::   dwReg2: 0x%X   deReg 0x%X  \n", nId, dwReg2, dwReg );
        spin_lock_irqsave(&dev->vc[nId].slock, flags);
        dev->vc[nId].is_streaming = 1;
        TW68_set_dmabits(dev, fh->DMA_nCH);
        spin_unlock_irqrestore(&dev->vc[nId].slock, flags);
        mutex_unlock(&dev->start_lock);
        return streaming;
}


static int TW68_streamoff(struct file *file, void *priv,
                          enum v4l2_buf_type type)
{
        int err;
        struct TW68_fh *fh = priv;
        struct TW68_dev *dev = fh->dev;
        int DMA_nCH = fh->DMA_nCH;
        int nId = DMA_nCH;
        struct videobuf_queue *q;
        unsigned long flags = 0;
        q=TW68_queue(fh);
        if (!res_check(fh, RESOURCE_VIDEO)) {
                printk("don't hold resource\n");
                return 0;
        }
        mod_timer(&dev->delay_resync, jiffies+ msecs_to_jiffies(100));
        mutex_lock(&dev->start_lock);
        spin_lock_irqsave(&dev->slock, flags);
        nId = DMA_nCH;
        dev->vc[nId].video_fieldcount = 0;
        stop_video_DMA(dev, DMA_nCH );				//
        spin_unlock_irqrestore(&dev->slock, flags);
//        printk(KERN_INFO "%s:  DMA_nCH:%x   videobuf_streamoff delete video timeout    \n", dev->name, fh->DMA_nCH);


        del_timer(&dev->vc[nId].video_dmaq.timeout);


        spin_lock_irqsave(&dev->vc[nId].slock, flags);
        dev->vc[nId].is_streaming = 0;
        spin_unlock_irqrestore(&dev->vc[nId].slock, flags);
        err = videobuf_streamoff(q);
        res_free(fh->vc, fh, RESOURCE_VIDEO);
	//      printk(KERN_INFO "%s:%d videobuf_streamoff q->streaming:%x  return err:%x \n",  dev->name, fh->DMA_nCH, q->streaming, err );

        mutex_unlock(&dev->start_lock);
        return 0;
}

#if 0
static int TW68_g_parm(struct file *file, void *fh,
                       struct v4l2_streamparm *sp)
{

	sp->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	sp->parm.capture.capturemode = 0;//channel->cap_parm.capturemode;
	def_num = (channel->mode.format == FORMAT_NTSC) ? 1001 : 1000;
	def_dem = (channel->mode.format == FORMAT_NTSC) ? 30000 : 25000;
	sp->parm.capture.timeperframe.denominator = def_dem;
	switch (channel->mode.fdec) {
	default:
	case FDEC_1:
		sp->parm.capture.timeperframe.numerator = def_num;
		break;
	case FDEC_2:
		sp->parm.capture.timeperframe.numerator = def_num * 2;
		break;
	case FDEC_3:
		sp->parm.capture.timeperframe.numerator = def_num * 3;
		break;
	case FDEC_5:
		sp->parm.capture.timeperframe.numerator = def_num * 5;
		break;
	}
	dprintk(4, "%s capture mode, %d timeperframe %d/%d\n", __func__,
		sp->parm.capture.capturemode,
		sp->parm.capture.timeperframe.numerator,
		sp->parm.capture.timeperframe.denominator);

        return 0;
}
#endif

#define KERNEL_VERSION_NOINODE KERNEL_VERSION(2, 6, 29)

#if LINUX_VERSION_CODE >= KERNEL_VERSION_NOINODE
static long s812_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#else
static int s812_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
        struct TW68_fh *fh = file->private_data;
        struct TW68_dev *dev = fh->dev;
        int rc = 0;
        switch (cmd) {
        case S812_VIDIOC_READ_REG:
        {
                struct s812_reg reg;
                if (copy_from_user(&reg, (void*)arg, sizeof(reg)))
                        return -EFAULT;
                if (reg.type != 0)
                        return -EINVAL;
                reg.val = dev->last_dmaerr;
                if (copy_to_user((void*)arg, &reg, sizeof(reg)))
                        return -EFAULT;
                rc = 0;
        }
        break;
        case S812_VIDIOC_CLEAR_REG:
        {
                struct s812_reg reg;
                if (copy_from_user(&reg, (void*)arg, sizeof(reg)))
                        return -EFAULT;
                if (reg.type != 0)
                        return -EINVAL;
                dev->last_dmaerr = 0;
                reg.val = 0;
                if (copy_to_user((void*)arg, &reg, sizeof(reg)))
                        return -EFAULT;
                rc = 0;
        }
        break;
#if 0
        case S812_VIDIOC_WAIT_DMA_ERR:
        {
                struct s812_reg reg;
                reg.type = 0;
                reg.val = 0;
                rc = wait_event_interruptible(dev->dma_wq, (dev->last_dmaerr != 0));
                reg.val = dev->last_dmaerr;
                if (copy_to_user((void*)arg, &reg, sizeof(reg)))
                        return -EFAULT;
                return rc;
        }
#endif
        break;
        default:
#if LINUX_VERSION_CODE >= KERNEL_VERSION_NOINODE || V4L_DVB_TREE
                rc = video_ioctl2(file, cmd, arg);
#else
                rc = video_ioctl2(inode, file, cmd, arg);
#endif
                break;
        }
        return rc;
}

#define KERNEL_VERSION_NOINODE KERNEL_VERSION(2, 6, 29)

static const struct v4l2_file_operations video_fops =
{
        .owner = THIS_MODULE,
        .open = video_open,
        .release = video_release,
        .read = video_read,
        .poll = video_poll,
        .mmap = video_mmap,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 0, 0)
        .ioctl = s812_ioctl,
#else
        .unlocked_ioctl = s812_ioctl,
#endif

};


static const struct v4l2_ioctl_ops video_ioctl_ops = {
        .vidioc_querycap		= TW68_querycap,
        .vidioc_enum_fmt_vid_cap	= TW68_enum_fmt_vid_cap,
        .vidioc_g_fmt_vid_cap		= TW68_g_fmt_vid_cap,
        .vidioc_try_fmt_vid_cap		= TW68_try_fmt_vid_cap,
        .vidioc_s_fmt_vid_cap		= TW68_s_fmt_vid_cap,
        //.vidioc_enum_fmt_vid_overlay	= TW68_enum_fmt_vid_overlay,
        //.vidioc_g_fmt_vid_overlay	= TW68_g_fmt_vid_overlay,
        //.vidioc_try_fmt_vid_overlay	= TW68_try_fmt_vid_overlay,
        //.vidioc_s_fmt_vid_overlay	= TW68_s_fmt_vid_overlay,
        //.vidioc_g_fmt_vbi_cap		= TW68_try_get_set_fmt_vbi_cap,
        //.vidioc_try_fmt_vbi_cap		= TW68_try_get_set_fmt_vbi_cap,
        //.vidioc_s_fmt_vbi_cap		= TW68_try_get_set_fmt_vbi_cap,
        .vidioc_g_audio			= TW68_g_audio,
        .vidioc_s_audio			= TW68_s_audio,
        .vidioc_cropcap			= TW68_cropcap,
        .vidioc_reqbufs			= TW68_reqbufs,
        .vidioc_querybuf		= TW68_querybuf,
        .vidioc_qbuf			= TW68_qbuf,
        .vidioc_dqbuf			= TW68_dqbuf,
        .vidioc_s_std			= TW68_s_std,
        .vidioc_g_std			= TW68_g_std,
        .vidioc_enum_input		= TW68_enum_input,
        .vidioc_g_input			= TW68_g_input,
        .vidioc_s_input			= TW68_s_input,
        .vidioc_queryctrl		= TW68_queryctrl,
        .vidioc_g_ctrl			= TW68_g_ctrl,
        .vidioc_s_ctrl			= TW68_s_ctrl,
        .vidioc_streamon		= TW68_streamon,
        .vidioc_streamoff		= TW68_streamoff,
#ifdef CONFIG_VIDEO_V4L1_COMPAT
        .vidiocgmbuf			= vidiocgmbuf,
#endif
        .vidioc_g_crop			= TW68_g_crop,
        .vidioc_s_crop			= TW68_s_crop,
};


/* ----------------------------------------------------------- */
/* exported stuff                                              */

struct video_device TW68_video_template = {
        .name				= "TW686v-video",
        .fops				= &video_fops,
        .ioctl_ops 			= &video_ioctl_ops,
        .minor				= -1,
        .tvnorms			= TW68_NORMS,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
        .current_norm			= V4L2_STD_NTSC,
#endif
};



int TW68_video_init1(struct TW68_dev *dev)
{
        int k, m, n;
        __le32      *cpu;
        dma_addr_t   dma_addr;
        /* sanitycheck insmod options */
        if (gbuffers < 2 || gbuffers > VIDEO_MAX_FRAME)
                gbuffers = 2;
        if (gbufsize < 0 || gbufsize > gbufsize_max)
                gbufsize = gbufsize_max;
        gbufsize = (gbufsize + PAGE_SIZE - 1) & PAGE_MASK;

// pci_alloc_consistent   32 4 * 8  continuous field memory buffer

        for (n =0; n<8; n++)
                for (m =0; m<4; m++)
                {
                        cpu = pci_alloc_consistent(dev->pci, 800*300*2, &dma_addr);   // 8* 4096 contiguous  //*2
                        dev->BDbuf[n][m].cpu = cpu;
                        dev->BDbuf[n][m].dma_addr = dma_addr;
                        //printk("$$$$$$$$$$$$$$$$ TW68_  _video_init1 n:%dm:%d  cpu:%x    dma:%x   \n", n, m, cpu, dma_addr);
                        // assume aways successful   480k each field   total 32  <16MB
                }

        /* put some sensible defaults into the data structures ... */
        dev->ctl_volume = ctrl_by_id(V4L2_CID_AUDIO_VOLUME)->default_value;
        dev->ctl_mute = 1; // ctrl_by_id(V4L2_CID_AUDIO_MUTE)->default_value;
        dev->ctl_automute = ctrl_by_id(V4L2_CID_PRIVATE_AUTOMUTE)->default_value;

        ////////////////////////////////////////////////////////xxxxxxxxxxx
        init_timer(&dev->delay_resync);      //1021
        dev->delay_resync.function = resync;
        dev->delay_resync.data = (unsigned long)dev;   ///(unsigned long)(&dev);
        mod_timer(&dev->delay_resync, jiffies+ msecs_to_jiffies(300));


        ////////////////////////////////////////////////////////xxxxxxxxxxx

        for (k=0; k<8; k++) {
                INIT_LIST_HEAD(&dev->vc[k].video_dmaq.queued);
                INIT_LIST_HEAD(&dev->vc[k].video_dmaq.active);
                dev->vc[k].video_dmaq.dev = dev;
                dev->vc[k].video_dmaq.vc = &dev->vc[k];
                dev->vc[k].video_dmaq.id = k;
                init_timer(&dev->vc[k].video_dmaq.timeout);
                dev->vc[k].video_dmaq.timeout.function = TW68_buffer_timeout;
                dev->vc[k].video_dmaq.timeout.data = (unsigned long)(&dev->vc[k].video_dmaq);
                if (k<4) {
                        dev->vc[k].video_param.ctl_brightness = reg_readl(CH1_BRIGHTNESS_REG + k *0x10);
                        dev->vc[k].video_param.ctl_contrast = reg_readl(CH1_CONTRAST_REG + k *0x10);
                        dev->vc[k].video_param.ctl_hue = reg_readl(CH1_HUE_REG + k *0x10);
                        dev->vc[k].video_param.ctl_saturation = reg_readl(CH1_SAT_U_REG + k *0x10) /2;
                        dev->vc[k].video_param.ctl_mute = reg_readl(CH1_SAT_V_REG + k *0x10) /2;

                } else {
                        dev->vc[k].video_param.ctl_brightness     = reg_readl(CH1_BRIGHTNESS_REG + (k-4) *0x10 +0x100);
                        dev->vc[k].video_param.ctl_contrast   = reg_readl(CH1_CONTRAST_REG + (k-4) *0x10 +0x100);
                        dev->vc[k].video_param.ctl_hue        = reg_readl(CH1_HUE_REG + (k-4) *0x10 +0x100);
                        dev->vc[k].video_param.ctl_saturation = reg_readl(CH1_SAT_U_REG + (k-4) *0x10 +0x100) /2;
                        dev->vc[k].video_param.ctl_mute	   = reg_readl(CH1_SAT_V_REG + (k-4) *0x10 +0x100) /2;
                }
                dev->vc[k].video_param.valid_brightness = 1;
                dev->vc[k].video_param.valid_hue = 1;
                dev->vc[k].video_param.valid_saturation = 1;
                dev->vc[k].video_param.valid_contrast = 1;

                printk("TW68_  _video_init1[%d] def AMP: BR %d  CONT %d  HUE_ %d  SAT_U_%d SAT_V_%d   \n", k, 
                       dev->vc[k].video_param.ctl_brightness,
                       dev->vc[k].video_param.ctl_contrast, dev->vc[k].video_param.ctl_hue, dev->vc[k].video_param.ctl_saturation, dev->vc[k].video_param.ctl_mute);
        } // for k

        return 0;
}


int TW68_video_init2(struct TW68_dev *dev)
{
        /* init video hw */
        int k;
	int init = 0;

#if defined (NTSC_STANDARD_NODETECT)
        set_tvnorm(dev,&tvnorms[4]);
        for (k=0; k<8; k++)
                dev->vc[k].tvnormf = &tvnorms[4];
#elif defined (PAL_STANDARD_NODETECT)
        set_tvnorm(dev,&tvnorms[0]);
        for (k=0; k<8; k++)
                dev->vc[k].tvnormf = &tvnorms[0];
#else
        printk("TW68_  _video_init2    set_tvnorm  \n");
	if (vidstd_open == 1) // NTSC
		init = 4;
	
        set_tvnorm(dev,&tvnorms[init]);
        for (k=0; k<8; k++)
                dev->vc[k].tvnormf = &tvnorms[init];
#endif
        //video_mux(dev,0); ///0 3
        //TW68_tvaudio_setmute(dev);
        //TW68_tvaudio_setvolume(dev,dev->ctl_volume);
        return 0;
}

// called from interrupt handler

void TW68_irq_video_done(struct TW68_dev *dev, unsigned int nId, u32 dwRegPB)
{
        enum v4l2_field field;
        int Fn, PB;   /// d0, w0, 
        int single_field;
        unsigned long flags = 0;
        struct TW68_vc *vc = &dev->vc[nId];
        /// printk("TW68__irq_video_done .curr%p    nId= 0X%x    dwRegPB 0X%X \n", dev->video_dmaq[nId].curr, nId, dwRegPB);
        /// printk("TW68__irq_video_done   nId= 0X%x \n",  nId);
        if (!res_locked(&dev->vc[nId], RESOURCE_VIDEO))
                return;

        Fn = (dwRegPB >>24) & (1<< nId);
        PB = (dwRegPB ) & (1<< nId);

        spin_lock_irqsave(&vc->slock, flags);
        if (!vc->is_streaming) {
                spin_unlock_irqrestore(&vc->slock, flags);
                return;
        }

        if (vc->video_dmaq.curr) {
                // buffer is not valid, exit
                if ((vc->video_dmaq.curr->vb.state == VIDEOBUF_ERROR) ||
                    (vc->video_dmaq.curr->vb.state == VIDEOBUF_NEEDS_INIT)) {
                        //printk("buffer no longer valid %d\n",
                        //dev->vc[nId].video_dmaq.curr->vb.state);
                        spin_unlock_irqrestore(&vc->slock, flags);
                        return;
                }
                dev->vc[nId].video_fieldcount++;
                field = dev->vc[nId].video_dmaq.curr->vb.field;
                single_field = ((field == V4L2_FIELD_BOTTOM) || (field == V4L2_FIELD_TOP)) && sfield;
                /// printk("TW68__irq_video_done video_dmaq[nId].curr   nId  %x  dwRegPB 0x%X \n", nId, dwRegPB);
                dev->vc[nId].video_dmaq.curr->vb.field_count = dev->vc[nId].video_fieldcount;
                Fn = (dwRegPB >>24) & (1<< nId);
                PB = (dwRegPB ) & (1<< nId);
                /// printk("TW68__irq_video_done    d0:%d   w0:%d \n", d0, w0);
                //  weave frame output
                if (Fn == 0) {
                        // field 0 interrupt  program update  P field mapping
                        dev->vc[nId].Done = BF_Copy(dev, nId, Fn, PB, single_field);
                        dev->vc[nId].curPB = PB;
                        if (single_field && dev->vc[nId].Done) {
                                dev->vc[nId].Done = 0;
                                //printk("finished\n");

                                TW68_buffer_finish(dev, &dev->vc[nId].video_dmaq, VIDEOBUF_DONE);
                                TW68_buffer_next(dev,&(dev->vc[nId].video_dmaq));
                                goto done;
                        }
                        //Done =1;
                        //  /// printk(" irq video done nId%x  Fn:%d  field count = %d \n", nId, Fn, dev->video_fieldcount[nId]);
                        goto done;
                } else {
                        if (single_field) {
                                //printk("single skip %d\n", dev->vc[nId].Done);
                                goto done;
                        }
                        // copy bottom field, but only copy if first field was done.
                        // otherwise it will start on the next frame as desired
                        if (dev->vc[nId].Done) {
                                if (PB != dev->vc[nId].curPB) {
                                        dev->vc[nId].stat.pb_mismatch++;
                                        //printk("s812: DMA mismatch, skipping frame\n");
                                        dev->vc[nId].Done = 0;
                                        goto done;
                                }
                                BF_Copy(dev, nId, Fn, PB, 0);
                        }
                        ///   printk(" irq video done nId%x  Fn:%d BBB  field count = %d \n", nId, Fn, dev->video_fieldcount[nId]);
                        if (!dev->vc[nId].Done) {
                                // second field received without the first one.
                                goto done;
                        }
                        TW68_buffer_finish(dev, &dev->vc[nId].video_dmaq, VIDEOBUF_DONE);
                }
        } else {
                dev->vc[nId].Done = 0;
        }
        // B field interrupt  program update  P field mapping
        TW68_buffer_next(dev,&(dev->vc[nId].video_dmaq));
done:
        spin_unlock_irqrestore(&vc->slock, flags);
        return;
}



/* ----------------------------------------------------------- */
/*
 * Local variables:
 * c-basic-offset: 8
 * End:
 */
