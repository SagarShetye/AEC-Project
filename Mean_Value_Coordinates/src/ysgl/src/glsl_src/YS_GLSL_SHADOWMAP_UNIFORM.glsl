uniform MIDP float useShadowMap[YSGLSL_SHADOWMAP_MAX_NUM_TEXTURE];
uniform MIDP float useShadowTest[YSGLSL_SHADOWMAP_MAX_NUM_TEXTURE];
uniform HIGHP mat4 shadowMapTransform[YSGLSL_SHADOWMAP_MAX_NUM_TEXTURE];
uniform sampler2D shadowMapTexture[YSGLSL_SHADOWMAP_MAX_NUM_TEXTURE];
uniform MIDP float lightDistScale[YSGLSL_SHADOWMAP_MAX_NUM_TEXTURE];
uniform MIDP float lightDistOffset[YSGLSL_SHADOWMAP_MAX_NUM_TEXTURE];
