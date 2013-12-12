Overview
=====

FLUID (Fast Lightweight Universal Image Decoder) is a very lightweight and easy to integrate universal image decoder.

Supported image formats:

* PNG (All visible chunks except gamma, support interlaced)
* JPEG (JFIF/Baseline, no progressive JPEG)
* PSD (Raw RGB only)

Currently fluid is perfect for game developments. Support for other popular formats are planned and will be done when I get time (and request).

Usage
=====
    /*
     * fluid_decode: Decode an image
     * @data: [in] The image data
     * @size: [in] Size of the data in bytes
     * @width: [out] Width of the image in pixels
     * @height: [out] Height of the image in pixels
     * Return: Raw RGBA data, or NULL if failed
     */
    char *fluid_decode(const char *data, int size, int *width, int *height);

Install
=====
Integrating fluid to your project is very simple. You just grab fluid.c and fluid.h to anywhere in your project, add it to the build system, and you're done.

License
=====
I decided to place fluid in the [public domain](http://unlicense.org/). So you can do whatever you want with it, without concerning about licensing.

Contribution
=====
Due to the fact that maintaining a public domain code base is difficult (basically each contributor should write an agreement to say he'd like his contribution to be placed in public domain, but [sometimes this is not enough](http://www.sqlite.org/copyright.html)). I won't accept code contributions to this project at this time. But bug reports and reasonsable feature requests are highly appreciated, as usual.
