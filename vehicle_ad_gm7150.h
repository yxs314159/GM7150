#ifndef __VEHICLE_AD_GM7150_H__
#define __VEHICLE_AD_GM7150_H__

extern struct ad_dev *g_addev;

int gm7150_ad_init(struct ad_dev *ad);
int gm7150_ad_deinit(void);
int gm7150_ad_get_cfg(struct vehicle_cfg **cfg);
void gm7150_ad_check_cif_error(struct ad_dev *ad, int last_line);
int gm7150_check_id(struct ad_dev *ad);

#endif

