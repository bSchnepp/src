/*	$NetBSD$	*/

#ifndef VC4_DRM_WRITEBACK_H
#define VC4_DRM_WRITEBACK_H

int vc4_drm_writeback_connector_init(struct drm_device *, struct drm_writeback_connector *, const struct drm_connector_funcs *, const struct drm_encoder_helper_funcs *, const u32 *, int, u32);
int vc4_drm_writeback_connector_init_with_encoder(struct drm_device *, struct drm_writeback_connector *, struct drm_encoder *, const struct drm_connector_funcs *, const u32 *, int);
int vc4_drm_writeback_set_fb(struct drm_connector_state *, struct drm_framebuffer *);
int vc4_drm_writeback_prepare_job(struct drm_writeback_job *);
void vc4_drm_writeback_queue_job(struct drm_writeback_connector *, struct drm_connector_state *);
void vc4_drm_writeback_cleanup_job(struct drm_writeback_job *);
void vc4_drm_writeback_signal_completion(struct drm_writeback_connector *, int);
struct dma_fence *vc4_drm_writeback_get_out_fence(struct drm_writeback_connector *);

#endif