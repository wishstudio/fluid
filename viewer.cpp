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

#include <Windows.h>
#include <ShObjIdl.h>

#include "fluid.h"

/* Windows manifest */
#pragma comment(linker,"\"/manifestdependency:type='win32' \
 name='Microsoft.Windows.Common-Controls' version='6.0.0.0' \
 processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")

/* Control definitions */
#define IDC_BROWSE_BUTTON	101

HBITMAP bitmap;
int bitmapWidth, bitmapHeight;

static char *readFile(LPWSTR fileName, int *fileSize)
{
	HANDLE hFile = CreateFileW(fileName, GENERIC_READ, FILE_SHARE_READ, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (hFile == INVALID_HANDLE_VALUE)
		return nullptr;

	LARGE_INTEGER size;
	if (GetFileSizeEx(hFile, &size) == 0)
	{
		CloseHandle(hFile);
		return nullptr;
	}

	(*fileSize) = (int) size.QuadPart;
	char *content = (char *) malloc(*fileSize);
	if (content == nullptr)
	{
		CloseHandle(hFile);
		return nullptr;
	}
	DWORD bytesRead;
	if (!ReadFile(hFile, content, (*fileSize), &bytesRead, nullptr))
	{
		free(content);
		content = nullptr;
	}
	CloseHandle(hFile);
	return content;
}

static void loadImage(HWND hWnd, LPWSTR fileName)
{
	char *data;
	int size;
	data = readFile(fileName, &size);
	if (data == nullptr)
	{
		MessageBoxW(hWnd, L"Cannot read file content!", L"Critical", MB_ICONERROR | MB_OK);
		return;
	}

	int width, height;
	char *decoded = fluid_decode(data, size, &width, &height);
	free(data);
	if (decoded == nullptr)
	{
		MessageBoxW(hWnd, L"Decode image file failed.", L"Critical", MB_ICONERROR | MB_OK);
		return;
	}

	/* ARGB -> BGRA, Premultiply alpha */
	unsigned char *r = (unsigned char *) decoded;
	for (int i = 0; i < height; i++)
		for (int j = 0; j < width; j++)
		{
			unsigned char t = r[0];
			r[0] = r[2];
			r[2] = t;

			/* Premultiply (for display) */
			r[0] = r[0] * r[3] / 255 + (255 - r[3]);
			r[1] = r[1] * r[3] / 255 + (255 - r[3]);
			r[2] = r[2] * r[3] / 255 + (255 - r[3]);

			r += 4;
		}

	if (bitmap)
		DeleteObject(bitmap);
	bitmapWidth = width;
	bitmapHeight = height;

	BITMAPINFO bmi;
	bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	bmi.bmiHeader.biWidth = width;
	bmi.bmiHeader.biHeight = -height;
	bmi.bmiHeader.biPlanes = 1;
	bmi.bmiHeader.biBitCount = 32;
	bmi.bmiHeader.biCompression = BI_RGB;
	bmi.bmiHeader.biSizeImage = width * height * 4;
	HDC hdc = GetDC(hWnd);
	bitmap = CreateDIBitmap(hdc, &bmi.bmiHeader, CBM_INIT, decoded, &bmi, DIB_RGB_COLORS);
	ReleaseDC(hWnd, hdc);
	SendMessageW(hWnd, WM_PAINT, 0, 0);
	free(decoded);
}

static void openFile(HWND hWnd)
{
	HRESULT hr;
	IFileDialog *pFileDialog = nullptr;
	IShellItem *pShellItem = nullptr;
	PWSTR pFilePath = nullptr;
	DWORD dwFlags;

	/* Create FileOpenDialog */
	hr = CoCreateInstance(CLSID_FileOpenDialog, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&pFileDialog));
	if (FAILED(hr))
		goto FINISH;

	/* Get options */
	hr = pFileDialog->GetOptions(&dwFlags);
	if (FAILED(hr))
		goto FINISH;

	/* Get shell items only for file system items */
	hr = pFileDialog->SetOptions(dwFlags | FOS_FORCEFILESYSTEM);
	if (FAILED(hr))
		goto FINISH;

	/* Set file types */
	COMDLG_FILTERSPEC fileTypes[] =
	{
		{ L"PNG images", L"*.png" }
	};
	hr = pFileDialog->SetFileTypes(ARRAYSIZE(fileTypes), fileTypes);
	if (FAILED(hr))
		goto FINISH;

	/* Show dialog */
	hr = pFileDialog->Show(hWnd);
	if (FAILED(hr))
		goto FINISH;

	hr = pFileDialog->GetResult(&pShellItem);
	if (FAILED(hr))
		goto FINISH;

	hr = pShellItem->GetDisplayName(SIGDN_FILESYSPATH, &pFilePath);
	if (FAILED(hr))
		goto FINISH;

	loadImage(hWnd, pFilePath);

FINISH:
	if (pFilePath)
		CoTaskMemFree(pFilePath);
	if (pShellItem)
		pShellItem->Release();
	if (pFileDialog)
		pFileDialog->Release();
}

static LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message)
	{
	case WM_DESTROY:
		PostQuitMessage(0);
		return 1;

	case WM_COMMAND:
		switch (LOWORD(wParam))
		{
		case IDC_BROWSE_BUTTON:
			openFile(hWnd);
			break;
		}
		break;

	case WM_PAINT:
		HDC dc = GetDC(hWnd);
		HDC bitmapDC = CreateCompatibleDC(dc);
		SelectObject(bitmapDC, bitmap);
		StretchBlt(dc, 10, 100, bitmapWidth * 10, bitmapHeight * 10, bitmapDC, 0, 0, bitmapWidth, bitmapHeight, SRCCOPY);
		DeleteDC(bitmapDC);
		ReleaseDC(hWnd, dc);
		break;
	}
	return DefWindowProcW(hWnd, message, wParam, lParam);
}

int CALLBACK WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	CoInitialize(nullptr);

	WNDCLASSEXW wcex;
	wcex.cbSize = sizeof(WNDCLASSEX);
	wcex.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
	wcex.lpfnWndProc = WndProc;
	wcex.cbClsExtra = 0;
	wcex.cbWndExtra = 0;
	wcex.hInstance = hInstance;
	wcex.hIcon = 0;
	wcex.hCursor = LoadCursorW(nullptr, IDC_ARROW);
	wcex.hbrBackground = (HBRUSH)GetStockObject(WHITE_BRUSH);
	wcex.lpszMenuName = nullptr;
	wcex.lpszClassName = L"fluid_viewer";
	wcex.hIconSm = 0;
	RegisterClassExW(&wcex);

	/* Create the window */
	HWND windowHandle = CreateWindowExW(0, L"fluid_viewer", L"",
		WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
		CW_USEDEFAULT, CW_USEDEFAULT, 0, 0, nullptr, nullptr, hInstance, nullptr);

	if (windowHandle == nullptr)
		return 0;

	/* Show the window */
	ShowWindow(windowHandle, SW_SHOW);
	UpdateWindow(windowHandle);

	/* Set window size */
	SetWindowLongW(windowHandle, GWL_STYLE, GetWindowLong(windowHandle, GWL_STYLE) & ~(WS_POPUP | WS_EX_TOPMOST));

	RECT r;
	GetWindowRect(windowHandle, &r);
	/* Convert client size to window size */
	RECT rect;
	rect.top = 0;
	rect.left = 0;
	rect.right = 1024;
	rect.bottom = 768;
	AdjustWindowRect(&rect, WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX, FALSE);

	MoveWindow(windowHandle, r.left, r.top, rect.right - rect.left, rect.bottom - rect.top, FALSE);

	/* Add controls */
	CreateWindowExW(0, L"BUTTON", L"Browse file...",
		WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON,
		20, 20, 100, 26,
		windowHandle, (HMENU) IDC_BROWSE_BUTTON, hInstance, nullptr);

	/* Message loop */
	MSG msg;
	ZeroMemory(&msg, sizeof msg);
	while (GetMessageW(&msg, nullptr, 0, 0))
	{
		TranslateMessage(&msg);
		DispatchMessageW(&msg);
		if (msg.message == WM_QUIT)
			break;
	}
	if (bitmap)
		DeleteObject(bitmap);

	CoUninitialize();
	return 0;
}
