// SIGFM C API for libfprint drivers.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned char SigfmPix;
typedef struct SigfmImgInfo SigfmImgInfo;

SigfmImgInfo *sigfm_extract (const SigfmPix *pix, int width, int height);
void          sigfm_free_info (SigfmImgInfo *info);

int           sigfm_match_score (SigfmImgInfo *frame,
                                 SigfmImgInfo *enrolled);

unsigned char *sigfm_serialize_binary (SigfmImgInfo *info, int *outlen);
SigfmImgInfo  *sigfm_deserialize_binary (const unsigned char *bytes, int len);

int           sigfm_keypoints_count (SigfmImgInfo *info);
SigfmImgInfo *sigfm_copy_info (SigfmImgInfo *info);

#ifdef __cplusplus
}
#endif
