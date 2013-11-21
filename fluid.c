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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "fluid.h"

#define INLINE __inline

/* General helpers */
#define LOBYTE(x) ((x) & 0x0F)
#define HIBYTE(x) ((unsigned char) (x) >> 4)
#define LOINT(x) ((x) & 0xFFFF)
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

/* Rescale sample from depth-bit to 8-bit */
static INLINE unsigned int png_rescale_sample(unsigned int depth, unsigned int sample)
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
				image[0] = image[1] = image[2] = png_rescale_sample(status->depth, sampleg);
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
				image[0] = png_rescale_sample(status->depth, sampler);
				image[1] = png_rescale_sample(status->depth, sampleg);
				image[2] = png_rescale_sample(status->depth, sampleb);
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
				image[0] = image[1] = image[2] = png_rescale_sample(status->depth, extract_bits_big(&data, &bit, &size, status->depth));
				image[3] = png_rescale_sample(status->depth, extract_bits_big(&data, &bit, &size, status->depth));
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
				image[0] = png_rescale_sample(status->depth, extract_bits_big(&data, &bit, &size, status->depth));
				image[1] = png_rescale_sample(status->depth, extract_bits_big(&data, &bit, &size, status->depth));
				image[2] = png_rescale_sample(status->depth, extract_bits_big(&data, &bit, &size, status->depth));
				image[3] = png_rescale_sample(status->depth, extract_bits_big(&data, &bit, &size, status->depth));
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

char *fluid_decode(const char *_data, int size, int *width, int *height)
{
	const unsigned char *data = _data;
	/* Check image format and call corresponding image decoder */
	/* Check PNG */
	if (size >= 8)
	{
		if (data[0] == 137 && data[1] == 80 && data[2] == 78 && data[3] == 71 &&
			data[4] == 13 && data[5] == 10 && data[6] == 26 && data[7] == 10)
			return png_decode(data + 8, size - 8, width, height);
	}
	return NULL;
}
