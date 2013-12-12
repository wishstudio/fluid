Overview
=====

FLUID (Fast Lightweight Universal Image Decoder) is a very lightweight and easy to integrate (single file) universal image decoder.

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

Viewer
=====
The project contains a basic image viewer based on fluid. It is used to test the functionality during development and placed here for your interest. To build and run it, use the supplied Visual Studio project.

License
=====
I decided to place fluid in the [public domain](http://unlicense.org/). So you can do whatever you want with it, without concerning about licensing.

Contribution
=====
As I want to keep fluid a public domain project, every contributor must agree this. As the method used in [SQLite copyright notice](http://www.sqlite.org/copyright.html), any new contributions must be accompanied with the following statements:

_The author or authors of this code dedicate any and all copyright interest in this code to the public domain. We make this dedication for the benefit of the public at large and to the detriment of our heirs and successors. We intend this dedication to be an overt act of relinquishment in perpetuity of all present and future rights to this code under copyright law._

If you don't do this your contribution won't be merged. Plus if you make changes as an employee, due to the complexity mentioned in the above document I won't accept any contributions.

However, bug reports and reasonsable feature requests are highly appreciated, as usual.
