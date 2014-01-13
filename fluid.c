/*
 * FLUID: Fast lightweight universal image decoder
 * Copyleft 2013 Xiangyan Sun (wishstudio@gmail.com)
 *
 * This file is placed in the public domain.
 * For details, refer to LICENSE file.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "fluid.h"

#define INLINE __inline

/* General helpers */
#define LOBYTE(x) ((unsigned char) (x) & 0x0F)
#define HIBYTE(x) ((unsigned char) (x) >> 4)
#define LOINT(x) ((unsigned int) (x) & 0xFFFF)
#define HIINT(x) ((unsigned int) (x) >> 16)
#define BITMASK(len) ((1 << (len)) - 1)
#define EXTRACT_UINT8(data, x) \
	{ x = (uint8_t)*(data)++; }
#define GET_UINT16_BIG(data) (((data)[0] << 8) | (data)[1])
#define EXTRACT_UINT16_BIG(data, x) \
	{ x = GET_UINT16_BIG(data); (data) += 2; }
#define EXTRACT_UINT16_LITTLE(data, x) \
	{ x = *(uint16_t *)(data); (data) += 2; }
#define EXTRACT_UINT24_BIG(data, x) \
	{ x = ((data)[0] << 16) | ((data)[1] << 8) | (data)[2]; (data) += 3; }
#define EXTRACT_UINT32_BIG(data, x) \
	{ x = ((data)[0] << 24) | ((data)[1] << 16) | ((data)[2] << 8) | (data)[3]; (data) += 4; }
#define EXTRACT_UINT32_LITTLE(data, x) \
	{ x = *(uint32_t *)(data); (data) += 4; }

static int extract_bits_big(const unsigned char **data, int *bit, int *size, int bits)
{
	int x;
	if (*bit == 8)
	{
		*bit = 0;
		(*data)++, (*size)--;
	}
	if (*bit + bits <= 8)
	{
		x = ((*data)[0] >> (8 - (*bit + bits))) & BITMASK(bits);
		*bit += bits;
	}
	else if (*bit + bits <= 16)
	{
		x = ((((*data)[0] << 8) | (*data)[1]) >> (16 - (*bit + bits))) & BITMASK(bits);
		*bit += bits - 8;
		(*data)++, (*size)--;
	}
	else if (*bit + bits <= 24)
	{
		x = ((((*data)[0] << 16) | (*data)[1] << 8 | (*data)[2]) >> (24 - (*bit + bits))) & BITMASK(bits);
		*bit += bits - 16;
		(*data) += 2, (*size) -= 2;
	}
	return x;
}

static int extract_bits_little(const unsigned char **data, int *bit, int *size, int bits)
{
	int x;
	if (*bit == 8)
	{
		*bit = 0;
		(*data)++, (*size)--;
	}
	if (*bit + bits <= 8)
	{
		x = ((*data)[0] >> *bit) & BITMASK(bits);
		*bit += bits;
	}
	else if (*bit + bits <= 16)
	{
		x = ((((*data)[1] << 8) | (*data)[0]) >> *bit) & BITMASK(bits);
		*bit += bits - 8;
		(*data)++, (*size)--;
	}
	else if (*bit + bits <= 24)
	{
		x = ((((*data)[2] << 16) | (*data)[1] << 8 | (*data)[0]) >> *bit) & BITMASK(bits);
		*bit += bits - 16;
		(*data) += 2, (*size) -= 2;
	}
	return x;
}

/* Rescale sample from depth-bit to 8-bit */
static INLINE unsigned int sample_rescale(unsigned int depth, unsigned int sample)
{
	if (depth >= 8) /* Down sample */
		return sample >> (depth - 8);
	else /* Up sample */
	{
		if (sample & 1)
			return (sample << (8 - depth)) | BITMASK(8 - depth);
		else
			return sample << (8 - depth);
	}
}

static INLINE int color_clamp(int c)
{
	if ((unsigned int) c > 255)
	{
		if (c < 0)
			return 0;
		return 255;
	}
	return c;
}

/* Zlib deflate decoder */
#define DEFLATE_ALPHABET_SIZE			288
#define DEFLATE_HUFFMAN_MAX_CODELEN		15
#define DEFLATE_HUFFMAN_TREE_SIZE		(1 << DEFLATE_HUFFMAN_MAX_CODELEN)
typedef struct
{
	int codelen[DEFLATE_ALPHABET_SIZE], codelen_count[DEFLATE_HUFFMAN_MAX_CODELEN + 1], next_code[DEFLATE_HUFFMAN_MAX_CODELEN + 1];
	int hm_lit[DEFLATE_HUFFMAN_TREE_SIZE], hm_dist[DEFLATE_HUFFMAN_TREE_SIZE];
	/* Packed huffman data: 
	 * High 16 bits: Huffman code length
	 * Low 16 bits: Original alphabet
	 */
} DEFLATE_status;

#define HC_VAL(len, alphabet) (((len) << 16) | (alphabet))
#define HC_ISVALID(x) ((x) != -1)
#define HC_LEN(x) HIINT(x)
#define HC_ALPHABET(x) LOINT(x)

static const int HCLEN_ORDER[19] = { 16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15 };
static const int LEN_BASE[29] = { 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258 };
static const int LEN_BITS[29] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0 };
static const int DIST_BASE[30] = { 1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577 };
static const int DIST_BITS[30] = { 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13 };

static int zlib_huffman_code(DEFLATE_status *status, int n, int *dest)
{
	int i, code;
	memset(status->codelen_count, 0, sizeof(status->codelen_count));
	for (i = 0; i < (1 << DEFLATE_HUFFMAN_MAX_CODELEN); i++)
		dest[i] = -1;

	for (i = 0; i < n; i++)
		if (status->codelen[i] > 0)
			status->codelen_count[status->codelen[i]]++;
	status->next_code[0] = 0;
	for (i = 1; i <= DEFLATE_HUFFMAN_MAX_CODELEN; i++)
		status->next_code[i] = (status->next_code[i - 1] + status->codelen_count[i - 1]) << 1;
	for (i = 0; i < n; i++)
		if (status->codelen[i] > 0)
		{
			code = status->next_code[status->codelen[i]]++;
			if (code >= (1 << status->codelen[i])) /* Too much codepoints for a given length */
				return 0;
			dest[code] = HC_VAL(status->codelen[i], i);
		}
	return 1;
}

/* Byte order:
 * Data elements are packed into bytes in order of
 *   increasing bit number within the byte, i.e., starting
 *   with the least-significant bit of the byte.
 * Data elements other than Huffman codes are packed
 *   starting with the least-significant bit of the data
 *   element.
 * Huffman codes are packed starting with the most-
 *   significant bit of the code.
 */
static int zlib_extract_huffman_code(const unsigned char **data, int *bit, int *size, int *huffman, int *code)
{
	int i, b, c;
	for (c = 0, i = 0;;)
	{
		b = extract_bits_little(data, bit, size, 1);
		c = (c << 1) | b;
		if (++i > DEFLATE_HUFFMAN_MAX_CODELEN)
			return 0;
		if (HC_ISVALID(huffman[c]) && HC_LEN(huffman[c]) == i)
		{
			*code = HC_ALPHABET(huffman[c]);
			return 1;
		}
	}
}

static int zlib_read_huffman_codelen(const unsigned char **data, int *bit, int *size, int count, int *hm, int *codelen)
{
	int i, j, lit;
	for (i = 0; i < count;)
	{
		if (*size <= 4)
			return 0;
		if (!zlib_extract_huffman_code(data, bit, size, hm, &lit) || lit > 18)
			return 0;
		if (lit < 16) /* Literal */
			codelen[i++] = lit;
		else if (lit == 16) /* Repeat last */
		{
			if (i == 0)
				return 0;
			j = j = 3 + extract_bits_little(data, bit, size, 2);
			if (i + j > count)
				return 0;
			for (; j > 0; i++, j--)
				codelen[i] = codelen[i - 1];
		}
		else if (lit == 17) /* Repeat zero */
		{
			j = 3 + extract_bits_little(data, bit, size, 3);
			if (i + j > count)
				return 0;
			for (; j > 0; j--)
				codelen[i++] = 0;
		}
		else /* Repeat zero */
		{
			j = 11 + extract_bits_little(data, bit, size, 7);
			if (i + j > count)
				return 0;
			for (; j > 0; j--)
				codelen[i++] = 0;
		}
	}
	return 1;
}

static int zlib_deflate_decode(const unsigned char *data, int size, unsigned char *raw, int rawsize)
{
	int cmf, flg;
	unsigned char *current; /* Output pointer */
	unsigned char *cp; /* Copy pointer */
	unsigned int bit; /* Current bit of *data (0 - 8) */

	int bfinal, btype;
	int hlit, hdist, hclen;
	int i;
	int lit, dist, len, nlen;

	DEFLATE_status status;

	if (size < 6)
		return 0;
	/* Zlib header */
	size -= 2;
	EXTRACT_UINT8(data, cmf);
	EXTRACT_UINT8(data, flg);
	if (LOBYTE(cmf) != 8) /* Deflate */
		return 0;
	/* TODO: Check FLG */
	current = raw;
	bit = 0;
	bfinal = 0;
	while (current < raw + rawsize)
	{
		if (bfinal == 1) /* From last block */
			return 0;
		if (size <= 4)
			return 0;
		bfinal = extract_bits_little(&data, &bit, &size, 1);
		btype = extract_bits_little(&data, &bit, &size, 2);
		if (btype == 3)
			return 0;

		if (btype == 0) /* Non-compressed */
		{
			if (bit > 0)
				data++, bit = 0;
			if (size <= 4)
				return 0;
			EXTRACT_UINT16_LITTLE(data, len);
			EXTRACT_UINT16_LITTLE(data, nlen);
			size -= 4;
			if (size < 4 + len)
				return 0;
			size -= len;
			if ((len | nlen) != 0xFFFF)
				return 0;
			for (i = 0; i < len; i++)
				*current++ = *data++;
		}
		else /* Compressed */
		{
			/* Construct huffman code */
			if (btype == 1) /* Fixed huffman code */
			{
				for (i = 0; i < 144; i++)
					status.codelen[i] = 8;
				for (i = 144; i < 256; i++)
					status.codelen[i] = 9;
				for (i = 256; i < 280; i++)
					status.codelen[i] = 7;
				for (i = 280; i < 288; i++)
					status.codelen[i] = 8;
				zlib_huffman_code(&status, 288, status.hm_lit);
				for (i = 0; i < 32; i++)
					status.codelen[i] = 5;
				zlib_huffman_code(&status, 32, status.hm_dist);
			}
			else /* Dynamic huffman code */
			{
				hlit = 257 + extract_bits_little(&data, &bit, &size, 5);
				hdist = 1 + extract_bits_little(&data, &bit, &size, 5);
				hclen = 4 + extract_bits_little(&data, &bit, &size, 4);
				/* Generate length descriptor huffman code */
				for (i = 0; i < 19; i++)
					status.codelen[i] = 0;
				for (i = 0; i < hclen; i++)
				{
					if (size <= 4)
						return 0;
					status.codelen[HCLEN_ORDER[i]] = extract_bits_little(&data, &bit, &size, 3);
				}
				if (!zlib_huffman_code(&status, 19, status.hm_dist))
					return 0;
				/* Generate literal/length huffman code */
				if (!zlib_read_huffman_codelen(&data, &bit, &size, hlit, status.hm_dist, status.codelen))
					return 0;
				if (!zlib_huffman_code(&status, hlit, status.hm_lit))
					return 0;
				/* Generate distance huffman code */
				if (!zlib_read_huffman_codelen(&data, &bit, &size, hdist, status.hm_dist, status.codelen))
					return 0;
				if (!zlib_huffman_code(&status, hdist, status.hm_dist))
					return 0;
			}
			/* Actual decompressing */
			for (;;)
			{
				/* Extract literal/length */
				if (size <= 4)
					return 0;
				if (!zlib_extract_huffman_code(&data, &bit, &size, status.hm_lit, &lit) || lit > 285)
					return 0;
				if (lit == 256) /* End of block */
					break;
				if (lit < 256) /* Literal data */
				{
					if (current + 1 > raw + rawsize)
						return 0;
					*current++ = lit;
				}
				else /* Distance/length pair */
				{
					len = LEN_BASE[lit - 257] + extract_bits_little(&data, &bit, &size, LEN_BITS[lit - 257]);
					/* Extract distance */
					if (size <= 4)
						return 0;
					if (!zlib_extract_huffman_code(&data, &bit, &size, status.hm_dist, &lit))
						return 0;
					if (lit > 29) /* Invalid code point */
						return 0;
					dist = DIST_BASE[lit] + extract_bits_little(&data, &bit, &size, DIST_BITS[lit]);
					/* Copy data */
					cp = current - dist;
					if (cp < raw || current + len > raw + rawsize)
						return 0;
					for (i = 0; i < len; i++)
						*current++ = *cp++;
				}
			}
		}
	}
	return 1;
}

/* PNG decoder */
typedef struct
{
	int width, height;
	int depth, color_type;
	int compression_method, filter_method, interlace_method;
	int sample_per_pixel;
	int zlen, rawlen, imagelen;
	unsigned char *zraw, *raw, *defiltered, *interlaced, *image;
	/* Palette */
	int palette_count;
	const unsigned char *palette;
	/* Interlacing */
	int adam7_pass_width[8], adam7_pass_height[8];
	/* Transparency */
	int transparency_count;
	const unsigned char *transparency;
} PNG_status;

static int png_extract_chunk(const unsigned char **data, int *size, const unsigned char **chunk_type, const unsigned char **chunk_data, int *chunk_len)
{
	if (*size < 12)
		return 0;
	EXTRACT_UINT32_BIG(*data, *chunk_len);
	if (*size < 12 + *chunk_len)
		return 0;
	*size -= 12 + *chunk_len;
	*chunk_type = *data;
	*data += 4;
	*chunk_data = *data;
	*data += *chunk_len;
	*data += 4; /* CRC */
	return 1;
}

static int png_extract_data_size(PNG_status *status, const unsigned char *data, int size, const unsigned char *ctype, const unsigned char *cdata, int clen)
{
	status->zlen = 0;
	do
	{
		status->zlen += clen;
		if (!png_extract_chunk(&data, &size, &ctype, &cdata, &clen))
			return 0;
	} while (ctype[0] == 'I' && ctype[1] == 'D' && ctype[2] == 'A' && ctype[3] == 'T');
	return 1;
}

static INLINE int png_get_scanline_len(int width, int depth, int sample_per_pixel)
{
	int sample_per_byte;
	if (width == 0)
		return 0;
	if (depth < 8)
	{
		sample_per_byte = 8 / depth;
		return 1 + (width * sample_per_pixel + sample_per_byte - 1) / sample_per_byte;
	}
	else
		return 1 + width * sample_per_pixel * depth / 8;
}

static INLINE int png_paeth_predictor(int a, int b, int c)
{
	int p, pa, pb, pc;
	p = a + b - c;
	pa = abs(p - a);
	pb = abs(p - b);
	pc = abs(p - c);
	if (pa <= pb && pa <= pc)
		return a;
	else if (pb <= pc)
		return b;
	else
		return c;
}

static void png_defilter(const unsigned char *data, unsigned char *image, int width, int height, int depth, int sample_per_pixel)
{
	unsigned char type;
	int i, j;
	int ap, bp; /* Byte offset of a, b */
	unsigned int a, b, c, k;
	int scanline_len;

	scanline_len = png_get_scanline_len(width, depth, sample_per_pixel);

	if (depth < 8)
		ap = -1;
	else
		ap = -(depth / 8 * sample_per_pixel);
	bp = -scanline_len;

	for (i = 0; i < height; i++)
	{
		EXTRACT_UINT8(data, type);
		*image++ = type;
		if (type == 0) /* None */
		{
			for (j = 1; j < scanline_len; j++)
			{
				EXTRACT_UINT8(data, k);
				*image++ = k;
			}
		}
		else if (type == 1) /* Sub */
		{
			for (j = 1; j < scanline_len; j++)
			{
				a = (j + ap > 0) ? image[ap] : 0;
				EXTRACT_UINT8(data, k);
				*image++ = k + a;
			}
		}
		else if (type == 2) /* Up */
		{
			for (j = 1; j < scanline_len; j++)
			{
				b = (i > 0) ? image[bp] : 0;
				EXTRACT_UINT8(data, k);
				*image++ = k + b;
			}
		}
		else if (type == 3) /* Average */
		{
			for (j = 1; j < scanline_len; j++)
			{
				a = (j + ap > 0) ? image[ap] : 0;
				b = (i > 0) ? image[bp] : 0;
				EXTRACT_UINT8(data, k);
				*image++ = k + (a + b) / 2;
			}
		}
		else if (type == 4) /* Paeth */
		{
			for (j = 1; j < scanline_len; j++)
			{
				a = (j + ap > 0) ? image[ap] : 0;
				b = (i > 0) ? image[bp] : 0;
				c = (i > 0 && j + ap > 0) ? image[ap + bp] : 0;
				EXTRACT_UINT8(data, k);
				*image++ = k + png_paeth_predictor(a, b, c);
			}
		}
	}
}

/*
 * Adam7 passes:
 *
 * 1 6 4 6 2 6 4 6
 * 7 7 7 7 7 7 7 7
 * 5 6 5 6 5 6 5 6
 * 7 7 7 7 7 7 7 7
 * 3 6 4 6 3 6 4 6
 * 7 7 7 7 7 7 7 7
 * 5 6 5 6 5 6 5 6
 * 7 7 7 7 7 7 7 7
 */
static const int adam7_horizontal_start[8] = { 0, 1, 5, 1, 3, 1, 2, 1 };
static const int adam7_vertical_start[8]   = { 0, 1, 1, 5, 1, 3, 1, 2 };
static const int adam7_horizontal_delta[8] = { 0, 8, 8, 4, 4, 2, 2, 1 };
static const int adam7_vertical_delta[8]   = { 0, 8, 8, 8, 4, 4, 2, 2 };
static void png_extract_adam7_extent(PNG_status *status)
{
	int i;
	for (i = 1; i <= 7; i++)
	{
		status->adam7_pass_width[i] = (status->width + adam7_horizontal_delta[i] - adam7_horizontal_start[i]) / adam7_horizontal_delta[i];
		status->adam7_pass_height[i] = (status->height + adam7_vertical_delta[i] - adam7_vertical_start[i]) / adam7_vertical_delta[i];
	}
}

static void png_deinterlace_adam7(PNG_status *status, const unsigned char *data, unsigned char *image)
{
	int pass, i, j;
	for (pass = 1; pass <= 7; pass++)
		for (i = adam7_vertical_start[pass] - 1; i < status->height; i += adam7_vertical_delta[pass])
			for (j = adam7_horizontal_start[pass] - 1; j < status->width; j += adam7_horizontal_delta[pass])
			{
				*(uint32_t *) (image + (status->width * i + j) * 4) = *(uint32_t *) data;
				data += 4;
			}
}

static int png_extract_pixels(PNG_status *status, const unsigned char *data, unsigned char *dest, int width, int height, int size)
{
	unsigned char *image;
	unsigned int bit;
	int i, j, index;
	unsigned int sampler, sampleg, sampleb;
	int tr, tg, tb;
	
	image = dest;
	bit = 0;
	if (status->color_type == 0) /* Grayscale */
	{
		if (status->transparency)
			tg = GET_UINT16_BIG(status->transparency);
		for (i = 0; i < height; i++)
		{
			data++; /* Filter type byte */
			for (j = 0; j < width; j++)
			{
				sampleg = extract_bits_big(&data, &bit, &size, status->depth);
				image[0] = image[1] = image[2] = sample_rescale(status->depth, sampleg);
				image[3] = (status->transparency && sampleg == tg) ? 0 : 0xFF;
				image += 4;
			}
			if (bit > 0) /* Skip remaining bits */
				data++, bit = 0;
		}
	}
	else if (status->color_type == 2) /* Truecolor */
	{
		if (status->transparency)
		{
			tr = GET_UINT16_BIG(status->transparency);
			tg = GET_UINT16_BIG(status->transparency + 2);
			tb = GET_UINT16_BIG(status->transparency + 4);
		}
		for (i = 0; i < height; i++)
		{
			data++; /* Filter type byte */
			for (j = 0; j < width; j++)
			{
				sampler = extract_bits_big(&data, &bit, &size, status->depth);
				sampleg = extract_bits_big(&data, &bit, &size, status->depth);
				sampleb = extract_bits_big(&data, &bit, &size, status->depth);
				image[0] = sample_rescale(status->depth, sampler);
				image[1] = sample_rescale(status->depth, sampleg);
				image[2] = sample_rescale(status->depth, sampleb);
				image[3] = (status->transparency && sampler == tr && sampleg == tg && sampleb == tb) ? 0 : 0xFF;
				image += 4;
			}
			if (bit > 0) /* Skip remaining bits */
				data++, bit = 0;
		}
	}
	else if (status->color_type == 3) /* Indexed */
	{
		for (i = 0; i < height; i++)
		{
			data++; /* Filter type byte */
			for (j = 0; j < width; j++)
			{
				index = extract_bits_big(&data, &bit, &size, status->depth);
				if (index >= status->palette_count)
					return 0;
				image[0] = status->palette[index * 3 + 0];
				image[1] = status->palette[index * 3 + 1];
				image[2] = status->palette[index * 3 + 2];
				image[3] = (status->transparency && index < status->transparency_count) ? status->transparency[index] : 0xFF;
				image += 4;
			}
			if (bit > 0) /* Skip remaining bits */
				data++, bit = 0;
		}
	}
	else if (status->color_type == 4) /* Gray with alpha */
	{
		for (i = 0; i < height; i++)
		{
			data++; /* Filter type byte */
			for (j = 0; j < width; j++)
			{
				image[0] = image[1] = image[2] = sample_rescale(status->depth, extract_bits_big(&data, &bit, &size, status->depth));
				image[3] = sample_rescale(status->depth, extract_bits_big(&data, &bit, &size, status->depth));
				image += 4;
			}
			if (bit > 0) /* Skip remaining bits */
				data++, bit = 0;
		}
	}
	else if (status->color_type == 6) /* Truecolor with alpha */
	{
		for (i = 0; i < height; i++)
		{
			data++; /* Filter type byte */
			for (j = 0; j < width; j++)
			{
				image[0] = sample_rescale(status->depth, extract_bits_big(&data, &bit, &size, status->depth));
				image[1] = sample_rescale(status->depth, extract_bits_big(&data, &bit, &size, status->depth));
				image[2] = sample_rescale(status->depth, extract_bits_big(&data, &bit, &size, status->depth));
				image[3] = sample_rescale(status->depth, extract_bits_big(&data, &bit, &size, status->depth));
				image += 4;
			}
			if (bit > 0) /* Skip remaining bits */
				data++, bit = 0;
		}
	}
	return 1;
}

static char *png_decode(const unsigned char *data, int size, int *width, int *height)
{
	PNG_status status;
	const unsigned char *ctype, *cdata;
	unsigned char *zraw;
	int clen, isize, i, j, k;

	status.zraw = NULL;
	status.raw = NULL;
	status.defiltered = NULL;
	status.interlaced = NULL;
	status.image = NULL;
	status.palette = NULL;
	status.transparency = NULL;
	
	/* Dealing with IHDR chunk */
	if (png_extract_chunk(&data, &size, &ctype, &cdata, &clen) &&
		clen == 13 &&
		ctype[0] == 'I' && ctype[1] == 'H' && ctype[2] == 'D' && ctype[3] == 'R')
	{
		EXTRACT_UINT32_BIG(cdata, status.width);
		EXTRACT_UINT32_BIG(cdata, status.height);
		EXTRACT_UINT8(cdata, status.depth);
		EXTRACT_UINT8(cdata, status.color_type);
		EXTRACT_UINT8(cdata, status.compression_method);
		EXTRACT_UINT8(cdata, status.filter_method);
		EXTRACT_UINT8(cdata, status.interlace_method);

		if (status.width < 0 || status.height < 0)
			return 0;
		*width = status.width;
		*height = status.height;

		/* Initialization and basic checking */
		if (status.color_type == 0) /* Grayscale */
		{
			status.sample_per_pixel = 1;
			if (status.depth != 1 && status.depth != 2 && status.depth != 4 && status.depth != 8 && status.depth != 16)
				goto FINISH;
		}
		else if (status.color_type == 2) /* Truecolor */
		{
			status.sample_per_pixel = 3;
			if (status.depth != 8 && status.depth != 16)
				goto FINISH;
		}
		else if (status.color_type == 3) /* Indexed */
		{
			status.sample_per_pixel = 1;
			if (status.depth != 1 && status.depth != 2 && status.depth != 4 && status.depth != 8)
				goto FINISH;
		}
		else if (status.color_type == 4) /* Gray with alpha */
		{
			status.sample_per_pixel = 2;
			if (status.depth != 8 && status.depth != 16)
				goto FINISH;
		}
		else if (status.color_type == 6) /* Truecolor with alpha */
		{
			status.sample_per_pixel = 4;
			if (status.depth != 8 && status.depth != 16)
				goto FINISH;
		}
		else
			goto FINISH;
		if (status.compression_method != 0)
			goto FINISH;
		if (status.filter_method != 0)
			goto FINISH;
		if (status.interlace_method == 0)
			status.rawlen = png_get_scanline_len(status.width, status.depth, status.sample_per_pixel) * status.height;
		else if (status.interlace_method == 1)
		{
			png_extract_adam7_extent(&status);
			status.rawlen = 0;
			for (i = 1; i <= 7; i++)
				status.rawlen += png_get_scanline_len(status.adam7_pass_width[i], status.depth, status.sample_per_pixel) * status.adam7_pass_height[i];
		}
		else
			goto FINISH;

		/* Dealing with remaining chunks */
		while (png_extract_chunk(&data, &size, &ctype, &cdata, &clen))
		{
			if (ctype[0] == 'I' && ctype[1] == 'D' && ctype[2] == 'A' && ctype[3] == 'T')
			{
				/* Non-contiguous IDAT chunks */
				if (status.zraw)
					goto FINISH;

				/* Extract total data size */
				if (!png_extract_data_size(&status, data, size, ctype, cdata, clen))
					goto FINISH;

				/* Create a whole buffer for zlib data */
				status.zraw = malloc(status.zlen);
				if (!status.zraw)
					goto FINISH;

				/* Copy zlib data */
				zraw = status.zraw;
				do
				{
					memcpy(zraw, cdata, clen);
					zraw += clen;
					/* Since data size is correctly extracted, no need to check again */
					png_extract_chunk(&data, &size, &ctype, &cdata, &clen);
				} while (zraw < status.zraw + status.zlen);
			}
			if (ctype[0] == 'I' && ctype[1] == 'E' && ctype[2] == 'N' && ctype[3] == 'D')
				break;
			else if (ctype[0] == 'P' && ctype[1] == 'L' && ctype[2] == 'T' && ctype[3] == 'E')
			{
				/* Palette */
				if (clen % 3 || clen / 3 > (1 << status.depth))
					goto FINISH;
				status.palette_count = clen / 3;
				status.palette = cdata;
			}
			else if (ctype[0] == 't' && ctype[1] == 'R' && ctype[2] == 'N' && ctype[3] == 'S')
			{
				/* Transparency */
				if (status.color_type == 0 && clen != 2)
					goto FINISH;
				else if (status.color_type == 2 && clen != 6)
					goto FINISH;
				else if (status.color_type == 3 && (!status.palette || clen > status.palette_count))
					goto FINISH;
				status.transparency_count = clen;
				status.transparency = cdata;
			}
		}

		if (status.color_type == 3 && !status.palette) /* No palette for indexed color type */
			goto FINISH;

		if (status.zraw == NULL)
			goto FINISH;

		/* Zlib decompress */
		status.raw = malloc(status.rawlen);
		if (!status.raw)
			goto FINISH;
		if (!zlib_deflate_decode(status.zraw, status.zlen, status.raw, status.rawlen))
			goto FINISH;

		status.defiltered = malloc(status.rawlen);
		if (!status.defiltered)
			goto FINISH;

		if (status.interlace_method == 0)
			png_defilter(status.raw, status.defiltered, status.width, status.height, status.depth, status.sample_per_pixel);
		else
		{
			j = 0;
			for (i = 1; i <= 7; i++)
			{
				png_defilter(status.raw + j, status.defiltered + j, status.adam7_pass_width[i], status.adam7_pass_height[i], status.depth, status.sample_per_pixel);
				j += png_get_scanline_len(status.adam7_pass_width[i], status.depth, status.sample_per_pixel) * status.adam7_pass_height[i];
			}
		}

		status.imagelen = status.width * status.height * 4;
		status.interlaced = malloc(status.imagelen);
		if (!status.interlaced)
			goto FINISH;
		if (status.interlace_method == 0)
		{
			if (!png_extract_pixels(&status, status.defiltered, status.interlaced, status.width, status.height, status.rawlen))
				goto FINISH;
			status.image = status.interlaced;
			status.interlaced = NULL;
		}
		else
		{
			j = 0;
			k = 0;
			for (i = 1; i <= 7; i++)
			{
				isize = png_get_scanline_len(status.adam7_pass_width[i], status.depth, status.sample_per_pixel) * status.adam7_pass_height[i];
				if (!png_extract_pixels(&status, status.defiltered + j, status.interlaced + k, status.adam7_pass_width[i], status.adam7_pass_height[i], isize))
					goto FINISH;
				j += isize;
				k += status.adam7_pass_width[i] * status.adam7_pass_height[i] * 4;
			}
			status.image = malloc(status.imagelen);
			if (!status.image)
				goto FINISH;
			png_deinterlace_adam7(&status, status.interlaced, status.image);
		}
	}
FINISH:
	if (status.defiltered)
		free(status.defiltered);
	if (status.zraw)
		free(status.zraw);
	if (status.raw)
		free(status.raw);
	if (status.interlaced)
		free(status.interlaced);
	return status.image;
}

/* JPEG Decoder */
#define JPEG_SOF0		0xC0
#define JPEG_SOF1		0xC1
#define JPEG_SOF2		0xC2
#define JPEG_SOF3		0xC3
#define JPEG_DHT		0xC4
#define JPEG_SOF5		0xC5
#define JPEG_SOF6		0xC6
#define JPEG_SOF7		0xC7
#define JPEG_JPG		0xC8
#define JPEG_SOF9		0xC9
#define JPEG_SOF10		0xCA
#define JPEG_SOF11		0xCB
#define JPEG_DAC		0xCC
#define JPEG_SOF13		0xCD
#define JPEG_SOF14		0xCE
#define JPEG_SOF15		0xCF
#define JPEG_RST0		0xD0
#define JPEG_RST1		0xD1
#define JPEG_RST2		0xD2
#define JPEG_RST3		0xD3
#define JPEG_RST4		0xD4
#define JPEG_RST5		0xD5
#define JPEG_RST6		0xD6
#define JPEG_RST7		0xD7
#define JPEG_SOI		0xD8
#define JPEG_EOI		0xD9
#define JPEG_SOS		0xDA
#define JPEG_DQT		0xDB
#define JPEG_DNL		0xDC
#define JPEG_DRI		0xDD
#define JPEG_DHP		0xDE
#define JPEG_EXP		0xDF
#define JPEG_COMPONENTS_COUNT		256
#define JPEG_SCAN_COMPONENTS_COUNT	5
#define JPEG_HUFFMAN_LENGTH_MAX		255
#define JPEG_HUFFMAN_LENGTH_COUNT	17
typedef struct
{
	int H, V, Tq;
	int hs, vs;
	int linebytes, lines;
	int valid;
	unsigned char *raw;
} JPEG_component;

typedef struct
{
	int Pq; /* Precision */
	int Qk[64];
	int valid;
} JPEG_quantization_table;

typedef struct
{
	int Tc; /* Table class: 0 = DC, 1 = AC */
	int L[JPEG_HUFFMAN_LENGTH_COUNT];
	int hmin[JPEG_HUFFMAN_LENGTH_COUNT], hmax[JPEG_HUFFMAN_LENGTH_COUNT];
	int V[JPEG_HUFFMAN_LENGTH_COUNT][JPEG_HUFFMAN_LENGTH_MAX];
	int valid;
} JPEG_huffman_table;

typedef struct
{
	/* Frame header */
	int P;
	int Y, X;
	int Nf;
	int hmax, vmax;
	int hcnt, vcnt;
	/* Scan header */
	int Ns;
	int Cs[JPEG_SCAN_COMPONENTS_COUNT], Td[JPEG_SCAN_COMPONENTS_COUNT], Ta[JPEG_SCAN_COMPONENTS_COUNT];
	int Ss, Se, Ah, Al;
	int pred[JPEG_SCAN_COMPONENTS_COUNT];
	/* Restart interval */
	int Ri;
	/* Tables */
	JPEG_component comp[JPEG_COMPONENTS_COUNT];
	JPEG_quantization_table qtable[4];
	JPEG_huffman_table hdc[4], hac[4];
	/* Final image */
	unsigned char *image;
} JPEG_status;

static int jpeg_extract_bits(const unsigned char **data, int *bit, int *size, int bits, int *raw)
{
	*raw = 0;
	for (;;)
	{
		*raw = (*raw << (8 - *bit)) | ((*data)[0] & BITMASK(8 - *bit));
		if (bits <= 8 - *bit)
		{
			*raw >>= 8 - *bit - bits;
			*bit += bits;
			return 1;
		}
		else
		{
			bits -= 8 - *bit;
			*bit = 0;
			if ((*data)[0] == 0xFF)
			{
				if (*size < 2)
					return 0;
				(*size) -= 2;
				if ((*data)[1] != 0x00)
					return 0;
				(*data) += 2;
			}
			else
			{
				if (*size < 1)
					return 0;
				(*size)--;
				(*data)++;
			}
		}
	}
}

static int jpeg_extract_segment(const unsigned char **data, int *size, unsigned char *seg_type, const unsigned char **seg_data, int *seg_len)
{
	if (*size < 2)
		return 0;
	EXTRACT_UINT8(*data, *seg_type);
	(*size)--;
	if (*seg_type != 0xFF)
		return 0;
	while (*size >= 1)
	{
		EXTRACT_UINT8(*data, *seg_type);
		(*size)--;
		if (*seg_type != 0xFF)
			break;
	}
	if ((*seg_type >= JPEG_RST0 && *seg_type <= JPEG_RST7) || *seg_type == JPEG_SOI || *seg_type == JPEG_EOI) /* Standard-alone marker */
		return 1;
	if (*size < 2)
		return 0;
	EXTRACT_UINT16_BIG(*data, *seg_len);
	*size -= 2;
	*seg_len -= 2;
	if (*size < *seg_len)
		return 0;
	*size -= *seg_len;
	*seg_data = *data;
	*data += *seg_len;
	return 1;
}

static int jpeg_process_restart_interval(JPEG_status *status, unsigned char stype, const unsigned char *sdata, int slen)
{
	if (slen != 2)
		return 0;
	EXTRACT_UINT16_BIG(sdata, status->Ri);
	return 1;
}

static int jpeg_process_quantization_table(JPEG_status *status, unsigned char stype, const unsigned char *sdata, int slen)
{
	int i, j, Pq, Tq;
	if (slen == 0)
		return 0;
	while (slen > 0)
	{
		if (slen < 1)
			return 0;
		slen--;
		EXTRACT_UINT8(sdata, j);
		Pq = HIBYTE(j);
		if (Pq == 0)
			Pq = 8;
		else if (Pq == 1)
			Pq = 16;
		else
			return 0;
		Tq = LOBYTE(j);
		status->qtable[Tq].valid = 1;
		status->qtable[Tq].Pq = Pq;
		if (slen < Pq / 8 * 64)
			return 0;
		slen -= Pq / 8 * 64;
		if (Pq == 8)
		{
			for (i = 0; i < 64; i++)
				EXTRACT_UINT8(sdata, status->qtable[Tq].Qk[i]);
		}
		else
		{
			for (i = 0; i < 64; i++)
				EXTRACT_UINT16_BIG(sdata, status->qtable[Tq].Qk[i]);
		}
	}
	return 1;
}

static int jpeg_process_huffman_table(JPEG_status *status, unsigned char stype, const unsigned char *sdata, int slen)
{
	int i, j, mt, Tc, Th;
	JPEG_huffman_table *huffman;
	if (slen == 0)
		return 0;
	while (slen > 0)
	{
		if (slen < 17)
			return 0;
		slen -= 17;
		EXTRACT_UINT8(sdata, j);
		Tc = HIBYTE(j);
		Th = LOBYTE(j);
		if (Tc > 1 || Th > 3)
			return 0;
		if (Tc == 0)
			huffman = &status->hdc[Th];
		else
			huffman = &status->hac[Th];
		huffman->valid = 1;
		mt = 0;
		for (i = 1; i <= 16; i++)
		{
			EXTRACT_UINT8(sdata, huffman->L[i]);
			mt += huffman->L[i];
		}
		if (slen < mt)
			return 0;
		slen -= mt;
		for (i = 1; i <= 16; i++)
		for (j = 0; j < huffman->L[i]; j++)
			EXTRACT_UINT8(sdata, huffman->V[i][j]);
		/* Initialize hmin and hmax */
		j = 0;
		for (i = 1; i <= 16; i++)
		{
			if (huffman->L[i] == 0)
				huffman->hmin[i] = huffman->hmax[i] = -1;
			else
			{
				huffman->hmin[i] = j;
				huffman->hmax[i] = j + huffman->L[i] - 1;
				if (huffman->hmax[i] >= (1 << i))
					return 0;
				j += huffman->L[i];
			}
			j <<= 1;
		}
	}
	return 1;
}

static int jpeg_extract_huffman_code(JPEG_huffman_table *huffman, const unsigned char **data, int *bit, int *size, unsigned char *code)
{
	int i, j, k;

	j = 0;
	for (i = 1; i <= 16; i++)
	{
		if (!jpeg_extract_bits(data, bit, size, 1, &k))
			return 0;
		j = (j << 1) | k;
		if (j >= huffman->hmin[i] && j <= huffman->hmax[i])
		{
			*code = huffman->V[i][j - huffman->hmin[i]];
			return 1;
		}
	}
	return 0;
}

static int jpeg_process_segment(JPEG_status *status, unsigned char stype, const unsigned char *sdata, int slen)
{
	if (stype == JPEG_DRI)
		return jpeg_process_restart_interval(status, stype, sdata, slen);
	else if (stype == JPEG_DQT)
		return jpeg_process_quantization_table(status, stype, sdata, slen);
	else if (stype == JPEG_DHT)
		return jpeg_process_huffman_table(status, stype, sdata, slen);
	else /* Unrecognized segment marker, just return without processing */
		return 1;
}

static int jpeg_process_sof(JPEG_status *status, unsigned char stype, const unsigned char *sdata, int slen)
{
	int i, j, k;
	if (slen < 6)
		return 0;
	slen -= 6;
	EXTRACT_UINT8(sdata, status->P);
	EXTRACT_UINT16_BIG(sdata, status->Y);
	EXTRACT_UINT16_BIG(sdata, status->X);
	EXTRACT_UINT8(sdata, status->Nf);
	if (status->P != 8)
		return 0;
	if (status->Y == 0 || status->X == 0)
		return 0;
	if (status->Nf != 1 && status->Nf != 3) /* We only accept grayscale or YCbCr */
		return 0;
	if (slen != status->Nf * 3)
		return 0;
	for (i = 1; i <= status->Nf; i++)
	{
		EXTRACT_UINT8(sdata, k);
		if (k != i) /* No non-standard mapping please */
			return 0;
		status->comp[i].valid = 1;
		EXTRACT_UINT8(sdata, j);
		status->comp[i].H = HIBYTE(j);
		status->comp[i].V = LOBYTE(j);
		if (status->comp[i].H == 0 || status->comp[i].H > 4 || status->comp[i].V == 0 || status->comp[i].V > 4)
			return 0;
		EXTRACT_UINT8(sdata, status->comp[i].Tq);
		status->hmax = max(status->hmax, status->comp[i].H);
		status->vmax = max(status->vmax, status->comp[i].V);
	}
	for (i = 1; i <= status->Nf; i++)
	{
		if (status->hmax % status->comp[i].H != 0 || status->vmax % status->comp[i].V != 0) /* 4:3 and 3:2 is Unsupported */
			return 0;
		status->comp[i].hs = status->hmax / status->comp[i].H;
		status->comp[i].vs = status->vmax / status->comp[i].V;
	}
	status->vcnt = (status->Y + status->vmax * 8 - 1) / (status->vmax * 8);
	status->hcnt = (status->X + status->hmax * 8 - 1) / (status->hmax * 8);
	/* Allocate memory for components */
	for (i = 1; i <= status->Nf; i++)
	{
		status->comp[i].linebytes = status->comp[i].H * status->hcnt * 8;
		status->comp[i].lines = status->comp[i].V * status->vcnt * 8;
		status->comp[i].raw = malloc(status->comp[i].linebytes * status->comp[i].lines);
		if (!status->comp[i].raw)
			return 0;
	}
	return 1;
}

/* The EXTEND procedure in JPEG specification */
static INLINE int jpeg_extend(int raw, int t)
{
	int Vt;
	
	Vt = 1 << (t - 1);
	if (raw < Vt)
		return raw + ((-1) << t) + 1;
	else
		return raw;
}

/* TODO: Optimization */
static const double pi = 3.1415926535897932384626433832795028841971693993751;
static void jpeg_idct(int *src)
{
	int S[64];
	int x, y, u, v;
	double ans, cu, cv;

	memcpy(S, src, 64 * sizeof(int));
	for (y = 0; y < 8; y++)
		for (x = 0; x < 8; x++)
		{
			ans = 0;
			for (v = 0; v < 8; v++)
				for (u = 0; u < 8; u++)
				{
					cv = (v == 0) ? 1.0 / sqrt(2) : 1;
					cu = (u == 0) ? 1.0 / sqrt(2) : 1;
					ans += cv * cu * S[v * 8 + u] * cos((2 * x + 1) * u * pi / 16) * cos((2 * y + 1) * v * pi / 16);
				}
			src[y * 8 + x] = (int)(ans / 4);
		}
}

static const int jpeg_zigzag[8][8] = {
	{  0,  1,  5,  6, 14, 15, 27, 28 },
	{  2,  4,  7, 13, 16, 26, 29, 42 },
	{  3,  8, 12, 17, 25, 30, 41, 43 },
	{  9, 11, 18, 24, 31, 40, 44, 53 },
	{ 10, 19, 23, 32, 39, 45, 52, 54 },
	{ 20, 22, 33, 38, 46, 51, 55, 60 },
	{ 21, 34, 37, 47, 50, 56, 59, 61 },
	{ 35, 36, 48, 49, 57, 58, 62, 63 },
};
static int jpeg_process_scan_header(JPEG_status *status, unsigned char stype, const unsigned char *sdata, int slen)
{
	int i, j;

	if (slen < 4)
		return 0;
	slen -= 4;
	EXTRACT_UINT8(sdata, status->Ns);
	if (slen != status->Ns * 2)
		return 0;
	for (i = 1; i <= status->Ns; i++)
	{
		EXTRACT_UINT8(sdata, status->Cs[i]);
		if (!status->comp[status->Cs[i]].valid)
			return 0;
		EXTRACT_UINT8(sdata, j);
		status->Td[i] = HIBYTE(j);
		status->Ta[i] = LOBYTE(j);
		if (!status->hdc[status->Td[i]].valid || !status->hac[status->Ta[i]].valid)
			return 0;
	}
	EXTRACT_UINT8(sdata, status->Ss);
	EXTRACT_UINT8(sdata, status->Se);
	EXTRACT_UINT8(sdata, j);
	status->Ah = HIBYTE(j);
	status->Al = LOBYTE(j);
	/* These shall all be zero for sequential DCT process */
	if (status->Ss != 0 || status->Se != 63 || status->Ah != 0 || status->Al != 0)
		return 0;
	return 1;
}

static int jpeg_extract_scan(JPEG_status *status, const unsigned char **data, int *size)
{
	int i, j, k, c, g, mx, my, x, y, X, Y;
	int mcucnt, mcutotal;
	int bit;
	unsigned char t, rs, r, s;
	int raw[64], co[64], tmp;
	/* Extents of MCU */
	mcutotal = status->hcnt * status->vcnt;
	mcucnt = 0;
	bit = 0;

	for (k = 1; k <= status->Ns; k++)
		status->pred[k] = 0;
	for (i = 0; i < status->vcnt; i++)
		for (j = 0; j < status->hcnt; j++)
		{
			/* Decode MCU */
			for (k = 1; k <= status->Ns; k++)
			{
				c = status->Cs[k];
				for (my = 0; my < status->comp[c].V; my++)
					for (mx = 0; mx < status->comp[c].H; mx++)
					{
						/* Decode 8x8 block */
						/* Read data in */
						if (!jpeg_extract_huffman_code(&status->hdc[status->Td[k]], data, &bit, size, &t))
							return 0;
						if (t > 16)
							return 0;
						if (!jpeg_extract_bits(data, &bit, size, t, &tmp))
							return 0;
						status->pred[k] += t ? jpeg_extend(tmp, t) : 0;
						raw[0] = status->pred[k];
						for (g = 1; g <= 63; g++)
							raw[g] = 0;
						for (g = 1;;)
						{
							if (!jpeg_extract_huffman_code(&status->hac[status->Ta[k]], data, &bit, size, &rs))
								return 0;
							r = HIBYTE(rs);
							s = LOBYTE(rs);
							if (s == 0)
							{
								if (r != 15)
									break;
								g += 16;
								if (g > 63)
									return 0;
							}
							else
							{
								g += r;
								if (g > 63)
									return 0;
								if (!jpeg_extract_bits(data, &bit, size, s, &tmp))
									return 0;
								raw[g] = s ? jpeg_extend(tmp, s) : 0;
								if (g == 63)
									break;
								g++;
							}
						}
						/* Dequantization and de-zigzag */
						for (y = 0; y < 8; y++)
							for (x = 0; x < 8; x++)
							{
								g = jpeg_zigzag[y][x];
								co[y * 8 + x] = raw[g] * status->qtable[status->comp[c].Tq].Qk[g];
							}
						/* IDCT */
						jpeg_idct(co);
						/* Write data */
						for (y = 0; y < 8; y++)
							for (x = 0; x < 8; x++)
							{
								Y = (i * status->comp[c].V + my) * 8 + y;
								X = (j * status->comp[c].H + mx) * 8 + x;
								status->comp[c].raw[status->comp[c].linebytes * Y + X] = color_clamp(co[y * 8 + x] + 128);
							}
					}
			}
			mcucnt++;
			if (status->Ri != 0 && (mcucnt % status->Ri == 0) && mcucnt < mcutotal) /* Should occur a RST marker */
			{
				for (k = 1; k <= status->Ns; k++)
					status->pred[k] = 0;
				/* Skip remaining bits in current byte */
				if (bit > 0)
					(*data)++, bit = 0, (*size)--;
				if (*size < 2)
					return 0;
				if (**data != 0xFF)
					return 0;
				(*data)++, (*size)--;
				if (**data < JPEG_RST0 || **data > JPEG_RST7)
					return 0;
				(*data)++, (*size)--;
			}
		}

	return 1;
}

static char *jpeg_decode(const unsigned char *data, int size, int *width, int *height)
{
	unsigned char stype;
	const unsigned char *sdata;
	int slen;
	JPEG_status status;
	int i, j, k;
	int Y, Cb, Cr;

	memset(&status, 0, sizeof(JPEG_status));

	if (!jpeg_extract_segment(&data, &size, &stype, &sdata, &slen) || stype != JPEG_SOI)
		goto FINISH;
	for (;;)
	{
		if (!jpeg_extract_segment(&data, &size, &stype, &sdata, &slen))
			goto FINISH;
		if (stype == JPEG_SOF0)
			break;
		if (!jpeg_process_segment(&status, stype, sdata, slen))
			goto FINISH;
	}
	if (!jpeg_process_sof(&status, stype, sdata, slen))
		goto FINISH;

	for (;;)
	{
		if (!jpeg_extract_segment(&data, &size, &stype, &sdata, &slen))
			goto FINISH;
		if (stype == JPEG_SOS)
			break;
		if (!jpeg_process_segment(&status, stype, sdata, slen))
			goto FINISH;
	}
	if (!jpeg_process_scan_header(&status, stype, sdata, slen))
		goto FINISH;
	if (!jpeg_extract_scan(&status, &data, &size))
		goto FINISH;

	status.image = malloc(status.Y * status.X * 4);
	if (!status.image)
		goto FINISH;
	*width = status.X;
	*height = status.Y;

	if (status.Nf == 1)
	{
		for (i = 0; i < status.Y; i++)
			for (j = 0; j < status.X; j++)
			{
				k = (i * status.X + j) * 4;
				status.image[k] = status.image[k + 1] = status.image[k + 2] = status.comp[1].raw[status.comp[1].linebytes * i + j];
				status.image[k + 3] = 0xFF;
			}
	}
	else /* Nf == 3 */
	{
		/* Resample and YCbCr -> RGB */
		for (i = 0; i < status.Y; i++)
			for (j = 0; j < status.X; j++)
			{
				k = (i * status.X + j) * 4;
				Y = status.comp[1].raw[(i / status.comp[1].vs) * status.comp[1].linebytes + j / status.comp[1].hs];
				Cb = status.comp[2].raw[(i / status.comp[2].vs) * status.comp[2].linebytes + j / status.comp[2].hs];
				Cr = status.comp[3].raw[(i / status.comp[3].vs) * status.comp[3].linebytes + j / status.comp[3].hs];

				status.image[k + 0] = color_clamp((int)(Y + 1.402 * (Cr - 128)));
				status.image[k + 1] = color_clamp((int)(Y - 0.34414 * (Cb - 128) - 0.71414 * (Cr - 128)));
				status.image[k + 2] = color_clamp((int)(Y + 1.772 * (Cb - 128)));
				status.image[k + 3] = 255;
			}
	}
	
FINISH:
	for (i = 0; i < JPEG_COMPONENTS_COUNT; i++)
		if (status.comp[i].raw)
			free(status.comp[i].raw);
	return status.image;
}

/* PSD Decoder */
#define PSD_BITMAP		0
#define PSD_GRAYSCALE	1
#define PSD_INDEXED		2
#define PSD_RGB			3
#define PSD_CMYK		4
#define PSD_MULTI		7
#define PSD_DUOTONE		8
#define PSD_LAB			9
typedef struct
{
	int channels;
	int width, height;
	int depth;
	int color_mode;
	int compression_method;
	unsigned char *image;
} PSD_status;

static char *psd_decode(const unsigned char *data, int size, int *width, int *height)
{
	int i, j, k, c, bit, expected_size;
	PSD_status status;

	memset(&status, 0, sizeof(PSD_status));

	if (size < 16)
		return 0;
	
	/* File header */
	EXTRACT_UINT16_BIG(data, k); /* Version */
	if (k != 1)
		return 0;
	data += 6; /* Reserved */
	EXTRACT_UINT16_BIG(data, status.channels);
	EXTRACT_UINT32_BIG(data, status.height);
	EXTRACT_UINT32_BIG(data, status.width);
	EXTRACT_UINT16_BIG(data, status.depth);
	EXTRACT_UINT16_BIG(data, status.color_mode);
	size -= 16;

	*width = status.width;
	*height = status.height;
	
	if (status.channels != 3)
		return 0;
	if (status.width == 0 || status.height == 0)
		return 0;
	if (status.depth != 1 && status.depth != 8 && status.depth != 16 && status.depth != 32)
		return 0;
	if (status.color_mode != PSD_RGB)
		return 0;

	/* Color mode data */
	if (size < 4)
		return 0;
	EXTRACT_UINT32_BIG(data, k);
	if (size < 4 + k)
		return 0;
	data += k; /* Just skip */
	size -= k;

	/* Image resources */
	if (size < 4)
		return 0;
	EXTRACT_UINT32_BIG(data, k);
	if (size < 4 + k)
		return 0;
	data += k; /* just skip */
	size -= k;

	/* Layer and mask information */
	if (size < 4)
		return 0;
	EXTRACT_UINT32_BIG(data, k);
	if (size < 4 + k)
		return 0;
	data += k; /* just skip */
	size -= k;

	/* Image data */
	if (size < 2)
		return 0;
	EXTRACT_UINT16_BIG(data, status.compression_method);
	size -= 2;

	status.image = malloc(status.width * status.height * 4);
	if (!status.image)
		goto FINISH;
	if (status.compression_method == 0)
	{
		expected_size = status.channels * status.width * status.height;
		if (status.depth == 1)
			expected_size = (expected_size + 7) / 8;
		else
			expected_size = expected_size * (status.depth / 8);
		if (size < expected_size)
		{
			free(status.image);
			goto FINISH;
		}
		bit = 0;
		for (c = 0; c < status.channels; c++)
			for (i = 0; i < status.height; i++)
				for (j = 0; j < status.width; j++)
				{
					k = (i * status.width + j) * 4;
					status.image[k + c] = sample_rescale(status.depth, extract_bits_big(&data, &bit, &size, status.depth));
				}
		for (i = 0; i < status.height; i++)
			for (j = 0; j < status.width; j++)
			{
				k = (i * status.width + j) * 4;
				status.image[k + 3] = 255;
			}
	}
	else if (status.compression_method == 1)
	{
	}
	else /* Unsupported compression method */
		return 0;

FINISH:
	return status.image;
}

char *fluid_decode(const char *_data, int size, int *width, int *height)
{
	const unsigned char *data = _data;
	/* Identify image format and call corresponding image decoder */
	/* Check PNG */
	if (size >= 8)
	{
		if (data[0] == 137 && data[1] == 80 && data[2] == 78 && data[3] == 71 &&
			data[4] == 13 && data[5] == 10 && data[6] == 26 && data[7] == 10)
			return png_decode(data + 8, size - 8, width, height);
	}
	/* Check JPEG */
	if (size >= 1)
	{
		if (data[0] == 0xFF)
			return jpeg_decode(data, size, width, height);
	}
	/* Check PSD */
	if (size >= 4)
	{
		if (data[0] == '8' && data[1] == 'B' && data[2] == 'P' && data[3] == 'S')
			return psd_decode(data + 4, size - 4, width, height);
	}
	return NULL;
}
