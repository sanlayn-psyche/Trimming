#pragma once 

#include <cmath>
#include "TrimShared.h"
class CoverTexture
{
private:
	int m_H;
	int m_W;
    float* m_texture{nullptr};


public:
	static void* get_coverTexture(int h, int w);
    CoverTexture() = delete;
	CoverTexture(int w, int h);
	~CoverTexture();

	float get_coverage(float d, float cosalpha);

};
	
