#include "CoverTexture.h"
#include <stdexcept>

void* CoverTexture::get_coverTexture(int h, int w)
{
	float* ptr = (float*)malloc(h * w * sizeof(float));
	
	float gap_d = sqrt(2.0f) / float(h);
	float gap_a = 1.0f / float(w);

	float D = 0.0f;
	float cosAlpha = 0.0f;
	float sinAlpha = 0.0f;

	float edge_intersect[4] = { -1.f, -1.f, -1.f, -1.f };

	D = -sqrt(2.0) / 2.0;
	for (int j = 0; j < h; j++)
	{
		// a == 0 or 1

		if (D <= -0.5)
		{
			ptr[j] = 1.0;
			ptr[j + (w - 1)*(h)] = 1.0;
		}
		else if (D >= 0.5)
		{
			ptr[j] = 0.0;
			ptr[j + (w - 1) * (h)] = 0.0;
		}
		else
		{
			ptr[j] = (0.5 - D);
			ptr[j + (w - 1) * (h)] = (0.5 - D);
		}
		D += gap_d;
	}

	int inter1 = -1, inter2 = -1;

	for (int i = 1; i < w - 1; i++)
	{
		cosAlpha += gap_a;
		D = 0.0;
		for (int j = h / 2; j < h; j++)
		{
			sinAlpha = sqrt(1 - cosAlpha * cosAlpha);

			edge_intersect[0] = (0.5 * sinAlpha + D) / cosAlpha;
			edge_intersect[1] = (0.5 * cosAlpha - D) / sinAlpha;
			edge_intersect[2] = (-0.5 * sinAlpha + D) / cosAlpha;
			edge_intersect[3] = (-0.5 * cosAlpha - D) / sinAlpha;

			inter1 = -1;
			inter2 = -1;

			for (int k = 0; k < 4; k++)
			{
				if (edge_intersect[k] < -0.5 || edge_intersect[k] > 0.5)
				{
					edge_intersect[k] = -1;
				}
				else
				{
					if (inter1 == -1)
					{
						inter1 = k;
						inter2 = k;
					}
					else
					{
						inter2 = k;
					}
				}
			}
			edge_intersect[0] = 0.5 - edge_intersect[0];
			edge_intersect[1] = edge_intersect[1] + 0.5;
			edge_intersect[2] = 0.5 - edge_intersect[2];
			edge_intersect[3] = 0.5 + edge_intersect[3];

			if (inter2 == inter1)
			{
				ptr[j + i * (h)] = 0.0;
			}

			if (inter2 - inter1 == 1)
			{
				ptr[j + i * (h)] = float(0.5 * edge_intersect[inter1] * edge_intersect[inter2]);
				
			}
			if (inter2 - inter1 >= 2)
			{
				ptr[j + i * (h)] = float(0.5 * (edge_intersect[inter1] + edge_intersect[inter2]));

			}
			ptr[h - 1 - j + i * (h)] = 1.0 - ptr[j + i * (h)];

			if (ptr[j + i * (h)] <= FLOAT_ZERO_GEOMETRY_COMPUTE)
			{
				ptr[j + i * (h)] = 0.0;
			}
			if (ptr[h - 1 - j + i * (h)] <= FLOAT_ZERO_GEOMETRY_COMPUTE)
			{
				ptr[h - 1 - j + i * (h)] = 0.0;
			}
		
			D += gap_d;
		}
	}
    return ptr;
}

CoverTexture::CoverTexture(int w, int h)
{
	if (w <= 1 || h <= 1)
	{
		throw std::runtime_error("failed to create cover texture!\n");
	}
	m_W = w;
	m_H = h;
	m_texture = (float*)get_coverTexture(h, w);

}

CoverTexture::~CoverTexture()
{
	if (m_texture != nullptr)
	{
		free(m_texture);
	}
}

float CoverTexture::get_coverage(float d, float cosalpha)
{
	int i = round((d + sqrt(2.0) / 2.0) * float(m_H - 1));

	int j = round(cosalpha * float(m_W - 1));
	if (i >= m_H)
	{
		i = m_H - 1;
	}
	if (i < 0)
	{
		i = 0;
	}

	if (j >= m_W)
	{
		j = m_W - 1;
	}
	//return (*m_texture)(i, j);
	return m_texture[j*(m_H) + i];
	
}
