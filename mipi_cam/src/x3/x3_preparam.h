#ifndef X3_PREPARAM_H
#define X3_PREPARAM_H

#include "x3_sdk_wrap.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
int f37_linear_vin_param_init(x3_vin_info_t* vin_info);
int f37_dol2_vin_param_init(x3_vin_info_t* vin_info);
int os8a10_linear_vin_param_init(x3_vin_info_t* vin_info);
int os8a10_dol2_vin_param_init(x3_vin_info_t* vin_info);
int os8a10_2k_linear_vin_param_init(x3_vin_info_t* vin_info);
int imx415_linear_vin_param_init(x3_vin_info_t* vin_info);
int ov8856_linear_vin_param_init(x3_vin_info_t* vin_info);
int vps_grp_param_init(x3_vps_info_t *vps_info, int width, int height);
int vps_chn_param_init(x3_vps_chn_attr_t *vps_chn_attr, int chn_id, int width, int height, int fps);
int venc_chn_param_init(x3_venc_chn_info_t *venc_info, int chn_id, int width, int height, int fps, int bitrate);
int vot_param_init(x3_vot_info_t *vot_info);
int vdec_chn_param_init(x3_vdec_chn_info_t *vdec_chn_info, int chn_id, int width, int height, char *stream_src);
int x3_rgn_timestamp_param_init(x3_rgn_info_t *rgn_info, int vps_grp_id, int vps_chn_id);
void* x3_rgn_set_timestamp_thread(void *ptr);

#ifdef __cplusplus
};
#endif /* __cplusplus */
#endif // X3_PREPARAM_H

