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

#pragma once
#ifndef _FLUID_H
#define _FLUID_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * fluid_decode: Decode an image
 * @data: [in] The image data
 * @size: [in] Size of the data in bytes
 * @width: [out] Width of the image in pixels
 * @height: [out] Height of the image in pixels
 * Return: Raw RGBA data, or NULL if failed
 */
char *fluid_decode(const char *data, int size, int *width, int *height);

#ifdef __cplusplus
}
#endif

#endif
