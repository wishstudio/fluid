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

/* General helpers */
#define LOBYTE(x) ((x) & 0x0F)
#define HIBYTE(x) ((unsigned char) (x) >> 4)
#define LOINT(x) ((x) & 0xFFFF)
#define HIINT(x) ((unsigned int) (x) >> 16)
#define BITMASK(len) ((1 << (len)) - 1)
#define EXTRACT_UINT8(data, x) \
	{ x = (uint8_t)*(data)++; }
#define EXTRACT_UINT16_BIG(data, x) \
	{ x = ((data)[0] << 8) | (data)[1]; (data) += 2; }
#define EXTRACT_UINT24_BIG(data, x) \
	{ x = ((data)[0] << 16) | ((data)[1] << 8) | (data)[2]; (data) += 3; }
#define EXTRACT_UINT32_BIG(data, x) \
	{ x = ((data)[0] << 24) | ((data)[1] << 16) | ((data)[2] << 8) | (data)[3]; (data) += 4; }
#define EXTRACT_UINT32_LITTLE(data, x) \
	{ x = *(uint32_t *)(data); (data) += 4; }

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
static int zlib_extract_bits(const unsigned char **data, int *bit, int *size, int bits)
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

static int zlib_extract_huffman_code(const unsigned char **data, int *bit, int *size, int *huffman, int *code)
{
	int i, b, c;
	for (c = 0, i = 0;;)
	{
		b = zlib_extract_bits(data, bit, size, 1);
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
			j = j = 3 + zlib_extract_bits(data, bit, size, 2);
			if (i + j > count)
				return 0;
			for (; j > 0; i++, j--)
				codelen[i] = codelen[i - 1];
		}
		else if (lit == 17) /* Repeat zero */
		{
			j = 3 + zlib_extract_bits(data, bit, size, 3);
			if (i + j > count)
				return 0;
			for (; j > 0; j--)
				codelen[i++] = 0;
		}
		else /* Repeat zero */
		{
			j = 11 + zlib_extract_bits(data, bit, size, 7);
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
	int lit, dist, len;

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
		bfinal = zlib_extract_bits(&data, &bit, &size, 1);
		btype = zlib_extract_bits(&data, &bit, &size, 2);
		if (btype == 3)
			return 0;

		if (btype == 0) /* Non-compressed */
		{
			/* TODO */
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
				hlit = 257 + zlib_extract_bits(&data, &bit, &size, 5);
				hdist = 1 + zlib_extract_bits(&data, &bit, &size, 5);
				hclen = 4 + zlib_extract_bits(&data, &bit, &size, 4);
				/* Generate length descriptor huffman code */
				for (i = 0; i < 19; i++)
					status.codelen[i] = 0;
				for (i = 0; i < hclen; i++)
				{
					if (size <= 4)
						return 0;
					status.codelen[HCLEN_ORDER[i]] = zlib_extract_bits(&data, &bit, &size, 3);
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
					if (current >= raw + rawsize)
						return 0;
					*current++ = lit;
				}
				else /* Distance/length pair */
				{
					len = LEN_BASE[lit - 257] + zlib_extract_bits(&data, &bit, &size, LEN_BITS[lit - 257]);
					/* Extract distance */
					if (size <= 4)
						return 0;
					if (!zlib_extract_huffman_code(&data, &bit, &size, status.hm_dist, &lit))
						return 0;
					if (lit > 29) /* Invalid code point */
						return 0;
					dist = DIST_BASE[lit] + zlib_extract_bits(&data, &bit, &size, DIST_BITS[lit]);
					/* Copy data */
					cp = current - dist;
					if (cp < raw || current + len >= raw + rawsize)
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
	int zlen, rawlen, imagelen;
	unsigned char *zraw, *raw, *image;
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

static char *png_decode(const unsigned char *data, int size, int *width, int *height)
{
	PNG_status status;
	const unsigned char *ctype, *cdata;
	unsigned char *zraw;
	int clen;

	status.zraw = NULL;
	status.raw = NULL;
	status.image = NULL;
	
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

		if (status.compression_method != 0)
			goto FINISH;
		if (status.color_type != 0 && status.color_type != 2 && status.color_type != 3 && status.color_type != 4 && status.color_type != 6)
			goto FINISH;
		if (status.depth == 1)
			status.rawlen = (status.width + 7) / 8 * status.height;
		else if (status.depth == 2)
			status.rawlen = (status.width + 3) / 4 * status.height;
		else if (status.depth == 4)
			status.rawlen = (status.width + 1) / 2 * status.height;
		else if (status.depth == 8)
			status.rawlen = status.width * status.height;
		else if (status.depth == 16)
			status.rawlen = status.width * status.height * 2;
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
		}

		/* Zlib decompress */
		status.raw = malloc(status.rawlen);
		if (!status.raw)
			goto FINISH;
		if (!zlib_deflate_decode(status.zraw, status.zlen, status.raw, status.rawlen))
			goto FINISH;

		status.imagelen = status.width * status.height * 4;
		status.image = malloc(status.imagelen);
		if (status.image)
		{
		}
	}
FINISH:
	if (status.zraw)
		free(status.zraw);
	if (status.raw)
		free(status.raw);
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