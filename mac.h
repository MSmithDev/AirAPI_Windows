// TODO: post-draft pr, remove bg / investigation comments and links

#include <cstdint>
#include <pthread.h>

// Windows types and typealiases: https://learn.microsoft.com/en-us/windows/win32/winprog/windows-data-types?redirectedfrom=MSDN
typedef void *LPVOID; // pointer to any type, used in std windows lib functions signatures: 
typedef pthread_t *HANDLE; // void pointer alias in windows: https://stackoverflow.com/a/34870510/11274568
typedef HANDLE HINSTANCE;
typedef HINSTANCE HMODULE;
typedef int BOOL;
typedef BOOL TRUE;
typedef BOOL FALSE;

// explicit typedefs for macOS: https://cplusplus.com/forum/general/7306/
// DEBUG: trying alternate defs: https://puredata.info/dev/gemwiki/FreeFrame
typedef unsigned int DWORD; // relating to windows specific type

// typedef uint32_t DWORD;
typedef unsigned char BYTE;

// typedef int8_t  BYTE;
// unsure if I need these typealiases
// TODO: review later
typedef uint8_t  CHAR;
typedef uint16_t WORD;
typedef int16_t SHORT;
typedef int32_t LONG;

// WINAPI / CALLPACK is a standard function call alias in windows.h lib
#define WINAPI __stdcall;
#define APIENTRY WINAPI;
#define CALLBACK __stdcall;