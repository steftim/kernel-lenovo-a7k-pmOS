#include <mach/mt_typedefs.h>
#include <linux/types.h>
#include "primary_display.h"
#include "ddp_hal.h"
#include "disp_drv_log.h"
#include <linux/disp_assert_layer.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include "ddp_mmp.h"
#include "disp_session.h"
/* /common part */
//#define DAL_BPP	    (2)
#define DAL_BPP             (1)
#define DAL_WIDTH           (DISP_GetScreenWidth())
#define DAL_HEIGHT          (DISP_GetScreenHeight())

/* #ifdef CONFIG_MTK_FB_SUPPORT_ASSERTION_LAYER */

#include <linux/string.h>
#include <linux/semaphore.h>
#include <asm/cacheflush.h>
#include <linux/module.h>

#include "mtkfb_console.h"
#include "disp_drv_platform.h"
#include "disp_assert_layer_priv.h"
/* --------------------------------------------------------------------------- */
#define DAL_FORMAT          (DISP_FORMAT_RGB565)
#define DAL_BG_COLOR        (dal_bg_color)
#define DAL_FG_COLOR        (dal_fg_color)

#define RGB888_To_RGB565(x) ((((x) & 0xF80000) >> 8) |                      \
			     (((x) & 0x00FC00) >> 5) |                      \
			     (((x) & 0x0000F8) >> 3))

#define MAKE_TWO_RGB565_COLOR(high, low)  (((low) << 16) | (high))

#define DAL_LOCK()                                                          \
do {                                                                    \
	if (down_interruptible(&dal_sem)) {                                 \
		pr_debug("DISP/DAL" "Can't get semaphore in %s()\n",          \
		   __func__);                                           \
	    return DAL_STATUS_LOCK_FAIL;                                    \
	}                                                                   \
} while (0)

#define DAL_UNLOCK()  up(&dal_sem)

#define DAL_CHECK_MFC_RET(expr)								\
	do {										\
		MFC_STATUS ret = (expr);						\
		if (MFC_STATUS_OK != ret) {						\
			pr_debug("DISP/DAL " "Warning: call MFC_XXX function failed "	\
				"in %s(), line: %d, ret: %x\n",				\
				__func__, __LINE__, ret);				\
			return ret;							\
		}									\
	} while (0)



#define DAL_CHECK_DISP_RET(expr)                                            \
do {                                                                    \
	DISP_STATUS ret = (expr);                                           \
		if (DISP_STATUS_OK != ret) {                           \
			pr_debug("DISP/DAL" "Warning: call DISP_XXX function failed "          \
		   "in %s(), line: %d, ret: %x\n",                          \
		   __func__, __LINE__, ret);                            \
	    return ret;                                                     \
	}                                                                   \
} while (0)

#define DAL_LOG(fmt, arg...) pr_debug("DISP/DAL", fmt, ##arg)
/* --------------------------------------------------------------------------- */

static MFC_HANDLE mfc_handle;

static void *dal_fb_addr;
static unsigned long dal_fb_pa;
unsigned int isAEEEnabled = 0;

/* static BOOL  dal_shown   = FALSE; */
bool dal_shown = FALSE;
static BOOL dal_enable_when_resume = FALSE;
static BOOL dal_disable_when_resume = FALSE;
static unsigned int dal_fg_color = RGB888_To_RGB565(DAL_COLOR_WHITE);
static unsigned int dal_bg_color = RGB888_To_RGB565(DAL_COLOR_RED);

/* DECLARE_MUTEX(dal_sem); */
DEFINE_SEMAPHORE(dal_sem);

static char dal_print_buffer[2048];
/* --------------------------------------------------------------------------- */



UINT32 DAL_GetLayerSize(void)
{
	/* xuecheng, avoid lcdc read buffersize+1 issue */
	return DAL_WIDTH * DAL_HEIGHT * DAL_BPP + 4096;
}

static DAL_STATUS DAL_SetRedScreen(UINT32 *addr)
{
	UINT32 i;
	const UINT32 BG_COLOR = MAKE_TWO_RGB565_COLOR(DAL_BG_COLOR, DAL_BG_COLOR);

	for (i = 0; i < DAL_GetLayerSize() / sizeof(UINT32); ++i)
		*addr++ = BG_COLOR;
	return DAL_STATUS_OK;
}

DAL_STATUS DAL_SetScreenColor(DAL_COLOR color)
{
#if 1
	UINT32 i;
	UINT32 size;
	UINT32 BG_COLOR;
	MFC_CONTEXT *ctxt = NULL;
	UINT32 offset;
	unsigned int *addr;
	color = RGB888_To_RGB565(color);
	BG_COLOR = MAKE_TWO_RGB565_COLOR(color, color);

	ctxt = (MFC_CONTEXT *) mfc_handle;
	if (!ctxt)
		return DAL_STATUS_FATAL_ERROR;
	if (ctxt->screen_color == color)
		return DAL_STATUS_OK;
	offset = MFC_Get_Cursor_Offset(mfc_handle);
	addr = (unsigned int *)(ctxt->fb_addr + offset);

	size = DAL_GetLayerSize() - offset;
	for (i = 0; i < size / sizeof(UINT32); ++i)
		*addr++ = BG_COLOR;
	ctxt->screen_color = color;

	return DAL_STATUS_OK;
}
EXPORT_SYMBOL(DAL_SetScreenColor);

DAL_STATUS DAL_Init(unsigned long layerVA, unsigned long layerPA)
{
	pr_debug("%s, layerVA=0x%lx, layerPA=0x%lx\n", __func__, layerVA, layerPA);

	dal_fb_addr = (void *)layerVA;
	dal_fb_pa = layerPA;
	DAL_CHECK_MFC_RET(MFC_Open(&mfc_handle, dal_fb_addr, DAL_WIDTH,
		DAL_HEIGHT, DAL_BPP, DAL_FG_COLOR, DAL_BG_COLOR));

	/* DAL_Clean(); */
	DAL_SetScreenColor(DAL_COLOR_RED);

	return DAL_STATUS_OK;
}


DAL_STATUS DAL_SetColor(unsigned int fgColor, unsigned int bgColor)
{
	if (NULL == mfc_handle)
		return DAL_STATUS_NOT_READY;

	DAL_LOCK();
	dal_fg_color = RGB888_To_RGB565(fgColor);
	dal_bg_color = RGB888_To_RGB565(bgColor);
	DAL_CHECK_MFC_RET(MFC_SetColor(mfc_handle, dal_fg_color, dal_bg_color));
	DAL_UNLOCK();

	return DAL_STATUS_OK;
}
EXPORT_SYMBOL(DAL_SetColor);

DAL_STATUS DAL_Dynamic_Change_FB_Layer(unsigned int isAEEEnabled)
{
#if 0
	static int ui_layer_tdshp;

	pr_debug("[DDP] DAL_Dynamic_Change_FB_Layer, isAEEEnabled=%d\n", isAEEEnabled);

	if (DISP_DEFAULT_UI_LAYER_ID == DISP_CHANGED_UI_LAYER_ID) {
		pr_debug("[DDP] DAL_Dynamic_Change_FB_Layer, no dynamic switch\n");
		return DAL_STATUS_OK;
	}

	if (isAEEEnabled == 1) {
		/* change ui layer from DISP_DEFAULT_UI_LAYER_ID to DISP_CHANGED_UI_LAYER_ID */
		memcpy((void *)(&cached_layer_config[DISP_CHANGED_UI_LAYER_ID]),
		(void *)(&cached_layer_config[DISP_DEFAULT_UI_LAYER_ID]), sizeof(OVL_CONFIG_STRUCT));
		ui_layer_tdshp = cached_layer_config[DISP_DEFAULT_UI_LAYER_ID].isTdshp;
		cached_layer_config[DISP_DEFAULT_UI_LAYER_ID].isTdshp = 0;
		disp_path_change_tdshp_status(DISP_DEFAULT_UI_LAYER_ID, 0);
		/* change global variable value, else error-check will find layer 2,
		3 enable tdshp together */
		FB_LAYER = DISP_CHANGED_UI_LAYER_ID;
	} else {
		memcpy((void *)(&cached_layer_config[DISP_DEFAULT_UI_LAYER_ID]),
			(void *)(&cached_layer_config[DISP_CHANGED_UI_LAYER_ID]),
			sizeof(OVL_CONFIG_STRUCT));
		cached_layer_config[DISP_DEFAULT_UI_LAYER_ID].isTdshp = ui_layer_tdshp;
		FB_LAYER = DISP_DEFAULT_UI_LAYER_ID;
		memset((void *)(&cached_layer_config[DISP_CHANGED_UI_LAYER_ID]), 0,
			sizeof(OVL_CONFIG_STRUCT));
	}

	/* no matter memcpy or memset, layer ID should not be changed */
	cached_layer_config[DISP_DEFAULT_UI_LAYER_ID].layer = DISP_DEFAULT_UI_LAYER_ID;
	cached_layer_config[DISP_CHANGED_UI_LAYER_ID].layer = DISP_CHANGED_UI_LAYER_ID;
	cached_layer_config[DISP_DEFAULT_UI_LAYER_ID].isDirty = 1;
	cached_layer_config[DISP_CHANGED_UI_LAYER_ID].isDirty = 1;
#endif
	return DAL_STATUS_OK;
}

DAL_STATUS DAL_Clean(void)
{
	const UINT32 BG_COLOR = MAKE_TWO_RGB565_COLOR(DAL_BG_COLOR, DAL_BG_COLOR);
	DAL_STATUS ret = DAL_STATUS_OK;

	static int dal_clean_cnt;
	MFC_CONTEXT *ctxt = (MFC_CONTEXT *) mfc_handle;
	pr_debug("[MTKFB_DAL] DAL_Clean\n");
	if (NULL == mfc_handle)
		return DAL_STATUS_NOT_READY;

/* if (LCD_STATE_POWER_OFF == LCD_GetState()) */
/* return DAL_STATUS_LCD_IN_SUSPEND; */
	MMProfileLogEx(ddp_mmp_get_events()->dal_clean, MMProfileFlagStart, 0, 0);

	DAL_LOCK();

	DAL_CHECK_MFC_RET(MFC_ResetCursor(mfc_handle));

	ctxt->screen_color = 0;
	DAL_SetScreenColor(DAL_COLOR_RED);

/*
    if (LCD_STATE_POWER_OFF == LCD_GetState()) {
	    DISP_LOG_PRINT(ANDROID_LOG_INFO, "DAL", "dal_clean in power off\n");
	dal_disable_when_resume = TRUE;
	ret = DAL_STATUS_LCD_IN_SUSPEND;
	goto End;
    }
    */

	/* xuecheng, for debug */
#if 0
	if (is_early_suspended) {
		up(&sem_early_suspend);
		DISP_LOG_PRINT(ANDROID_LOG_INFO, "DAL", "dal_clean in power off\n");
		goto End;
	}
#endif

	/* TODO: if dal_shown=false, and 3D enabled, mtkfb may disable UI layer, please modify 3D driver */
	if (isAEEEnabled == 1) {
		disp_session_input_config session_input;
		disp_input_config *input;

		memset((void *)&session_input, 0, sizeof(session_input));

		session_input.setter = SESSION_USER_AEE;
		session_input.config_layer_num = 1;
		input = &session_input.config[0];

		input->src_phy_addr = (unsigned int)dal_fb_pa;
		input->layer_id = primary_display_get_option("ASSERT_LAYER");
		input->layer_enable = 0;
		input->src_offset_x = 0;
		input->src_offset_y = 0;
		input->src_width = DAL_WIDTH;
		input->src_height = DAL_HEIGHT;
		input->tgt_offset_x = 0;
		input->tgt_offset_y = 0;
		input->tgt_width = DAL_WIDTH;
		input->tgt_height = DAL_HEIGHT;
		input->alpha = 0x80;
		input->alpha_enable = 1;
		input->next_buff_idx = -1;
		input->src_pitch = DAL_WIDTH;
		input->src_fmt = DAL_FORMAT;
		input->next_buff_idx = -1;

		ret = primary_display_config_input_multiple(&session_input);

		/* DAL disable, switch UI layer to default layer 3 */
		pr_debug("[DDP]* isAEEEnabled from 1 to 0, %d\n", dal_clean_cnt++);
		isAEEEnabled = 0;
		DAL_Dynamic_Change_FB_Layer(isAEEEnabled);	/* restore UI layer to DEFAULT_UI_LAYER */
	}

	dal_shown = FALSE;
	dal_disable_when_resume = FALSE;

	primary_display_trigger(0, NULL, 0);


 End:
	DAL_UNLOCK();
	MMProfileLogEx(ddp_mmp_get_events()->dal_clean, MMProfileFlagEnd, 0, 0);
	return ret;
}
EXPORT_SYMBOL(DAL_Clean);

int is_DAL_Enabled(void)
{
	int ret = 0;
	DAL_LOCK();
	ret = isAEEEnabled;
	DAL_UNLOCK();

	return ret;
}

unsigned long get_Assert_Layer_PA(void)
{
	return dal_fb_pa;
}

DAL_STATUS DAL_Printf(const char *fmt, ...)
{
	va_list args;
	uint i;
	DAL_STATUS ret = DAL_STATUS_OK;
	disp_session_input_config session_input;
	disp_input_config *input;

	DISPFUNC();

	if (NULL == mfc_handle)
		return DAL_STATUS_NOT_READY;

	if (NULL == fmt)
		return DAL_STATUS_INVALID_ARGUMENT;

	MMProfileLogEx(ddp_mmp_get_events()->dal_printf, MMProfileFlagStart, 0, 0);
	DAL_LOCK();
	if (isAEEEnabled == 0) {
		pr_warn("[DDP] isAEEEnabled from 0 to 1, ASSERT_LAYER=%d, dal_fb_pa %lx\n",
			primary_display_get_option("ASSERT_LAYER"), dal_fb_pa);

		isAEEEnabled = 1;
		DAL_Dynamic_Change_FB_Layer(isAEEEnabled);
		/* default_ui_ layer coniig to changed_ui_layer */

		DAL_CHECK_MFC_RET(MFC_Open(&mfc_handle, dal_fb_addr,
			DAL_WIDTH, DAL_HEIGHT, DAL_BPP,
			DAL_FG_COLOR, DAL_BG_COLOR));

		/* DAL_Clean(); */
		memset((void *)&session_input, 0, sizeof(session_input));

		session_input.setter = SESSION_USER_AEE;
		session_input.config_layer_num = 1;
		input = &session_input.config[0];

		input->src_phy_addr = (unsigned int)dal_fb_pa;
		input->layer_id = primary_display_get_option("ASSERT_LAYER");
		input->layer_enable = 1;
		input->src_offset_x = 0;
		input->src_offset_y = 0;
		input->src_width = DAL_WIDTH;
		input->src_height = DAL_HEIGHT;
		input->tgt_offset_x = 0;
		input->tgt_offset_y = 0;
		input->tgt_width = DAL_WIDTH;
		input->tgt_height = DAL_HEIGHT;
		input->alpha = 0x80;
		input->alpha_enable = 1;
		input->next_buff_idx = -1;
		input->src_pitch = DAL_WIDTH;
		input->src_fmt = DAL_FORMAT;
		input->next_buff_idx = -1;

		ret = primary_display_config_input_multiple(&session_input);
	}

	va_start(args, fmt);
	i = vsprintf(dal_print_buffer, fmt, args);
	BUG_ON(i >= ARRAY_SIZE(dal_print_buffer));
	va_end(args);
	DAL_CHECK_MFC_RET(MFC_Print(mfc_handle, dal_print_buffer));

	flush_cache_all();

#if 0
	if (is_early_suspended) {
		up(&sem_early_suspend);
		DISP_LOG_PRINT(ANDROID_LOG_INFO, "DAL", "DAL_Printf in power off\n");
		goto End;
	}
#endif

	if (!dal_shown)
		dal_shown = TRUE;
	ret = primary_display_trigger(0, NULL, 0);

 End:

	DAL_UNLOCK();

	MMProfileLogEx(ddp_mmp_get_events()->dal_printf, MMProfileFlagEnd, 0, 0);

	return ret;
}
EXPORT_SYMBOL(DAL_Printf);

DAL_STATUS DAL_OnDispPowerOn(void)
{
#if 0
	DAL_LOCK();

	/* Re-enable assertion layer when display resumes */

	if (is_early_suspended) {
		if (dal_enable_when_resume) {
			dal_enable_when_resume = FALSE;
			if (!dal_shown) {
				mutex_lock(&OverlaySettingMutex);
				cached_layer_config[ASSERT_LAYER].src_x = 0;
				cached_layer_config[ASSERT_LAYER].src_y = 0;
				cached_layer_config[ASSERT_LAYER].src_w = DAL_WIDTH;
				cached_layer_config[ASSERT_LAYER].src_h = DAL_HEIGHT;
				cached_layer_config[ASSERT_LAYER].dst_x = 0;
				cached_layer_config[ASSERT_LAYER].dst_y = 0;
				cached_layer_config[ASSERT_LAYER].dst_w = DAL_WIDTH;
				cached_layer_config[ASSERT_LAYER].dst_h = DAL_HEIGHT;
				cached_layer_config[ASSERT_LAYER].layer_en = TRUE;
				cached_layer_config[ASSERT_LAYER].isDirty = true;
				atomic_set(&OverlaySettingDirtyFlag, 1);
				atomic_set(&OverlaySettingApplied, 0);
				mutex_unlock(&OverlaySettingMutex);
				dal_shown = TRUE;
			}
			goto End;
		} else if (dal_disable_when_resume) {
			dal_disable_when_resume = FALSE;
			mutex_lock(&OverlaySettingMutex);
			cached_layer_config[ASSERT_LAYER].layer_en = FALSE;
			cached_layer_config[ASSERT_LAYER].isDirty = true;
			atomic_set(&OverlaySettingDirtyFlag, 1);
			atomic_set(&OverlaySettingApplied, 0);
			mutex_unlock(&OverlaySettingMutex);
			dal_shown = FALSE;
			goto End;

		}
	}

 End:
	DAL_UNLOCK();
#endif
	return DAL_STATUS_OK;
}

/* ########################################################################## */
/* !CONFIG_MTK_FB_SUPPORT_ASSERTION_LAYER */
/* ########################################################################## */
#else
	unsigned int isAEEEnabled = 0;

	UINT32 DAL_GetLayerSize(void)
		{
		/* xuecheng, avoid lcdc read buffersize+1 issue */
		return DAL_WIDTH * DAL_HEIGHT * DAL_BPP + 4096;
	}

	DAL_STATUS DAL_Init(UINT32 layerVA, UINT32 layerPA)
		{
		NOT_REFERENCED(layerVA);
		NOT_REFERENCED(layerPA);

		return DAL_STATUS_OK;
	}
	DAL_STATUS DAL_SetColor(unsigned int fgColor, unsigned int bgColor)
		{
		NOT_REFERENCED(fgColor);
		NOT_REFERENCED(bgColor);

		return DAL_STATUS_OK;
	}
	DAL_STATUS DAL_Clean(void)
		{
		pr_debug("[MTKFB_DAL] DAL_Clean is not implemented\n");
		return DAL_STATUS_OK;
	}
	DAL_STATUS DAL_Printf(const char *fmt, ...)
		{
		NOT_REFERENCED(fmt);
		pr_debug("[MTKFB_DAL] DAL_Printf is not implemented\n");
		return DAL_STATUS_OK;
	}
	DAL_STATUS DAL_OnDispPowerOn(void)
		{
		return DAL_STATUS_OK;
	}
	DAL_STATUS DAL_SetScreenColor(DAL_COLOR color)
		{
		return DAL_STATUS_OK; }
#endif				/* CONFIG_MTK_FB_SUPPORT_ASSERTION_LAYER */

#ifdef DAL_LOWMEMORY_ASSERT
	/*EXPORT_SYMBOL(DAL_LowMemoryOn);*/
	/*EXPORT_SYMBOL(DAL_LowMemoryOff);*/
#endif
