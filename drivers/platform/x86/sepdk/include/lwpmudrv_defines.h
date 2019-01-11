/* ****************************************************************************
 *  Copyright(C) 2009-2018 Intel Corporation.  All Rights Reserved.
 *
 *  This file is part of SEP Development Kit
 *
 *  SEP Development Kit is free software; you can redistribute it
 *  and/or modify it under the terms of the GNU General Public License
 *  version 2 as published by the Free Software Foundation.
 *
 *  SEP Development Kit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  As a special exception, you may use this file as part of a free software
 *  library without restriction.  Specifically, if other files instantiate
 *  templates or use macros or inline functions from this file, or you
 *  compile this file and link it with other files to produce an executable
 *  this file does not by itself cause the resulting executable to be
 *  covered by the GNU General Public License.  This exception does not
 *  however invalidate any other reasons why the executable file might be
 *  covered by the GNU General Public License.
 * ****************************************************************************
 */

#ifndef _LWPMUDRV_DEFINES_H_
#define _LWPMUDRV_DEFINES_H_

#if defined(__cplusplus)
extern "C" {
#endif
//
// Start off with none of the OS'es are defined
//
#undef DRV_OS_WINDOWS
#undef DRV_OS_LINUX
#undef DRV_OS_SOLARIS
#undef DRV_OS_MAC
#undef DRV_OS_ANDROID
#undef DRV_OS_UNIX

//
// Make sure none of the architectures is defined here
//
#undef DRV_IA32
#undef DRV_EM64T

//
// Make sure one (and only one) of the OS'es gets defined here
//
// Unfortunately entirex defines _WIN32 so we need to check for linux
// first.  The definition of these flags is one and only one
// _OS_xxx is allowed to be defined.
//
#if defined(__ANDROID__)
#define DRV_OS_ANDROID
#define DRV_OS_UNIX
#elif defined(__linux__)
#define DRV_OS_LINUX
#define DRV_OS_UNIX
#elif defined(sun)
#define DRV_OS_SOLARIS
#define DRV_OS_UNIX
#elif defined(_WIN32)
#define DRV_OS_WINDOWS
#elif defined(__APPLE__)
#define DRV_OS_MAC
#define DRV_OS_UNIX
#elif defined(__FreeBSD__)
#define DRV_OS_FREEBSD
#define DRV_OS_UNIX
#else
#error "Compiling for an unknown OS"
#endif

//
// Make sure one (and only one) architecture is defined here
// as well as one (and only one) pointer__ size
//
#if defined(_M_IX86) || defined(__i386__)
#define DRV_IA32
#elif defined(_M_AMD64) || defined(__x86_64__)
#define DRV_EM64T
#else
#error "Unknown architecture for compilation"
#endif

//
// Add a well defined definition of compiling for release (free) vs.
// debug (checked). Once again, don't assume these are the only two values,
// always have an else clause in case we want to expand this.
//
#if defined(DRV_OS_UNIX)
#define WINAPI
#endif

/*
 *  Add OS neutral defines for file processing.  This is needed in both
 *  the user code and the kernel code for cleanliness
 */
#undef DRV_FILE_DESC
#undef DRV_INVALID_FILE_DESC_VALUE
#define DRV_ASSERT assert

#if defined(DRV_OS_WINDOWS)

#define DRV_FILE_DESC HANDLE
#define DRV_INVALID_FILE_DESC_VALUE INVALID_HANDLE_VALUE

#elif defined(DRV_OS_LINUX) || defined(DRV_OS_SOLARIS) ||                      \
	defined(DRV_OS_ANDROID)

#define DRV_IOCTL_FILE_DESC SIOP
#define DRV_FILE_DESC SIOP
#define DRV_INVALID_FILE_DESC_VALUE -1

#elif defined(DRV_OS_FREEBSD)

#define DRV_IOCTL_FILE_DESC S64
#define DRV_FILE_DESC S64
#define DRV_INVALID_FILE_DESC_VALUE -1

#elif defined(DRV_OS_MAC)
#if defined __LP64__
#define DRV_IOCTL_FILE_DESC S64
#define DRV_FILE_DESC S64
#define DRV_INVALID_FILE_DESC_VALUE (S64)(-1)
#else
#define DRV_IOCTL_FILE_DESC S32
#define DRV_FILE_DESC S32
#define DRV_INVALID_FILE_DESC_VALUE (S32)(-1)
#endif

#else

#error "Compiling for an unknown OS"

#endif

#define OUT
#define IN
#define INOUT

//
// VERIFY_SIZEOF let's you insert a compile-time check that the size of a data
// type (e.g. a struct) is what you think it should be.  Usually it is
// important to know what the actual size of your struct is, and to make sure
// it is the same across all platforms.  So this will prevent the code from
// compiling if something happens that you didn't expect, whether it's because
// you counted wring, or more often because the compiler inserted padding that
// you don't want.
//
// NOTE: 'elem' and 'size' must both be identifier safe, e.g. matching the
// regular expression /^[0-9a-zA-Z_]$/.
//
// Example:
//   typedef struct { void *ptr; int data; } mytype;
//   VERIFY_SIZEOF(mytype, 8);
//                         ^-- this is correct on 32-bit platforms, but fails
//                             on 64-bit platforms, indicating a possible
//                             portability issue.
//
#define VERIFY_SIZEOF(type, size)                                            \
	{                                                                    \
		enum {                                                       \
		sizeof_##type##_eq_##size = 1 / (int)(sizeof(type) == size)  \
		}                                                            \
	}

#if defined(DRV_OS_WINDOWS)
#define DRV_DLLIMPORT __declspec(dllimport)
#define DRV_DLLEXPORT __declspec(dllexport)
#endif
#if defined(DRV_OS_UNIX)
#define DRV_DLLIMPORT
#define DRV_DLLEXPORT
#endif

#if defined(DRV_OS_WINDOWS)
#define FSI64RAW "I64"
#define DRV_PATH_SEPARATOR "\\"
#define L_DRV_PATH_SEPARATOR L"\\"
#endif

#if defined(DRV_OS_UNIX)
#define FSI64RAW "ll"
#define DRV_PATH_SEPARATOR "/"
#define L_DRV_PATH_SEPARATOR L"/"
#endif

#define FSS64 "%" FSI64RAW "d"
#define FSU64 "%" FSI64RAW "u"
#define FSX64 "%" FSI64RAW "x"

#if defined(DRV_OS_WINDOWS)
#define DRV_RTLD_NOW 0
#endif
#if defined(DRV_OS_UNIX)
#if defined(DRV_OS_FREEBSD)
#define DRV_RTLD_NOW 0
#else
#define DRV_RTLD_NOW RTLD_NOW
#endif
#endif

#define DRV_STRLEN (U32)(strlen)
#define DRV_WCSLEN (U32)(wcslen)
#define DRV_STRCSPN strcspn
#define DRV_STRCHR strchr
#define DRV_STRRCHR strrchr
#define DRV_WCSRCHR wcsrchr

#if defined(DRV_OS_WINDOWS)
#define DRV_STCHARLEN DRV_WCSLEN
#else
#define DRV_STCHARLEN DRV_STRLEN
#endif

#if defined(DRV_OS_WINDOWS)
#define DRV_STRCPY strcpy_s
#define DRV_STRNCPY strncpy_s
#define DRV_STRICMP _stricmp
#define DRV_STRNCMP strncmp
#define DRV_STRNICMP _strnicmp
#define DRV_STRDUP _strdup
#define DRV_WCSDUP _wcsdup
#define DRV_STRCMP strcmp
#define DRV_WCSCMP wcscmp
#define DRV_SNPRINTF _snprintf_s
#define DRV_SNWPRINTF _snwprintf_s
#define DRV_VSNPRINTF _vsnprintf_s
#define DRV_SSCANF sscanf_s
#define DRV_STRCAT strcat_s
#define DRV_STRNCAT strncat_s
#define DRV_MEMCPY memcpy_s
#define DRV_WMEMCPY wmemcpy_s
#define DRV_STRTOK strtok_s
#define DRV_STRTOUL strtoul
#define DRV_STRTOULL _strtoui64
#define DRV_STRTOQ _strtoui64
#define DRV_FOPEN(fp, name, mode) fopen_s(&(fp), (name), (mode))
#define DRV_WFOPEN(fp, name, mode) _wfopen_s(&(fp), (name), (mode))
#define DRV_FCLOSE(fp)          \
	{                           \
		if ((fp) != NULL) {     \
			fclose((fp));       \
		}                       \
	}
#define DRV_WCSCPY wcscpy_s
#define DRV_WCSNCPY wcsncpy_s
#define DRV_WCSCAT wcscat_s
#define DRV_WCSNCAT wcsncat_s
#define DRV_WCSTOK wcstok_s
#define DRV_WCSSTR wcsstr
#define DRV_STRERROR strerror_s
#define DRV_SPRINTF sprintf_s
#define DRV_VSPRINTF vsprintf_s
#define DRV_VSWPRINTF vswprintf_s
#define DRV_GETENV_S getenv_s
#define DRV_WGETENV_S wgetenv_s
#define DRV_PUTENV(name) _putenv(name)
#define DRV_USTRCMP(X, Y) DRV_WCSCMP(X, Y)
#define DRV_USTRDUP(X) DRV_WCSDUP(X)
#define DRV_ACCESS(X) _access_s(X, 4)
#define DRV_STRSTR strstr

#define DRV_STCHAR_COPY DRV_WCSNCPY

#define DRV_GETENV(buf, buf_size, name) _dupenv_s(&(buf), &(buf_size), (name))
#define DRV_WGETENV(buf, buf_size, name) _wdupenv_s(&(buf), &(buf_size), (name))
#define DRV_SCLOSE(fp) _close(fp)
#define DRV_WRITE(fp, buf, buf_size) _write(fp, buf, buf_size);
#define DRV_SOPEN_S(fp, name, oflag, shflag, pmode)                          \
	_sopen_s((fp), (name), (oflag), (shflag), (pmode))
#endif

#if defined(DRV_OS_UNIX)
/*
   Note: Many of the following macros have a "size" as the second argument.
   Generally speaking, this is for compatibility with the _s versions
   available on Windows. On Linux/Solaris/Mac, it is ignored.
   On Windows, it is the size of the destination buffer and is used wrt
   memory checking features available in the C runtime in debug mode.
   Do not confuse it with the number of bytes to be copied, or such.

   On Windows, this size should correspond to the number of allocated characters
   (char or wchar_t) pointed to by the first argument. See MSDN or more details.
*/
#define DRV_STRICMP strcasecmp
#define DRV_STRDUP strdup
#define DRV_STRNDUP strndup
#define DRV_STRCMP strcmp
#define DRV_STRNCMP strncmp
#define DRV_STRSTR strstr
#define DRV_SNPRINTF(buf, buf_size, length, args...)                          \
	snprintf((buf), (length), ##args)
#define DRV_SNWPRINTF(buf, buf_size, length, args...)                         \
	snwprintf((buf), (length), ##args)
#define DRV_VSNPRINTF(buf, buf_size, length, args...)                         \
	vsnprintf((buf), (length), ##args)
#define DRV_SSCANF sscanf
#define DRV_STRCPY(dst, dst_size, src) strcpy((dst), (src))
#define DRV_STRNCPY(dst, dst_size, src, n) strncpy((dst), (src), (n))
#define DRV_STRCAT(dst, dst_size, src) strcat((dst), (src))
#define DRV_STRNCAT(dst, dst_size, src, n) strncat((dst), (src), (n))
#define DRV_MEMCPY(dst, dst_size, src, n) memcpy((dst), (src), (n))
#define DRV_STRTOK(tok, delim, context) strtok((tok), (delim))
#define DRV_STRTOUL strtoul
#define DRV_STRTOULL strtoull
#define DRV_STRTOL strtol
#define DRV_FOPEN(fp, name, mode) { (fp) = fopen((name), (mode)); }
#define DRV_FCLOSE(fp)                     \
	{                                  \
		if ((fp) != NULL) {        \
			fclose((fp));      \
		}                          \
	}

#define DRV_WCSCPY(dst, dst_size, src) wcscpy((dst), (const wchar_t *)(src))
#define DRV_WCSNCPY(dst, dst_size, src, count)                                 \
	wcsncpy((dst), (const wchar_t *)(src), (count))
#define DRV_WCSCAT(dst, dst_size, src) wcscat((dst), (const wchar_t *)(src))
#define DRV_WCSTOK(tok, delim, context)                                        \
	wcstok((tok), (const wchar_t *)(delim), (context))
#define DRV_STRERROR strerror
#define DRV_SPRINTF(dst, dst_size, args...) sprintf((dst), ##args)
#define DRV_VSPRINTF(dst, dst_size, length, args...)                           \
	vsprintf((dst), (length), ##args)
#define DRV_VSWPRINTF(dst, dst_size, length, args...)                          \
	vswprintf((dst), (length), ##args)
#define DRV_GETENV_S(dst, dst_size) getenv(dst)
#define DRV_WGETENV_S(dst, dst_size) wgetenv(dst)
#define DRV_PUTENV(name) putenv(name)
#define DRV_GETENV(buf, buf_size, name) ((buf) = getenv((name)))
#define DRV_USTRCMP(X, Y) DRV_STRCMP(X, Y)
#define DRV_USTRDUP(X) DRV_STRDUP(X)
#define DRV_ACCESS(X) access(X, X_OK)

#define DRV_STCHAR_COPY DRV_STRNCPY
#endif

#if defined(DRV_OS_WINDOWS)
#define DRV_STRTOK_R(tok, delim, context) strtok_s((tok), (delim), (context))
#else
#define DRV_STRTOK_R(tok, delim, context) strtok_r((tok), (delim), (context))
#endif

#if defined(DRV_OS_LINUX) || defined(DRV_OS_MAC) || defined(DRV_OS_FREEBSD)
#define DRV_STRTOQ strtoq
#endif

#if defined(DRV_OS_ANDROID)
#define DRV_STRTOQ strtol
#endif

#if defined(DRV_OS_SOLARIS)
#define DRV_STRTOQ strtoll
#endif

#if defined(DRV_OS_LINUX) || defined(DRV_OS_FREEBSD) || defined(DRV_OS_MAC)
#define DRV_WCSDUP wcsdup
#endif

#if defined(DRV_OS_SOLARIS)
#define DRV_WCSDUP solaris_wcsdup
#endif

#if defined(DRV_OS_ANDROID)
#define DRV_WCSDUP android_wcsdup
#endif

/*
 * Windows uses wchar_t and linux uses char for strings.
 * Need an extra level of abstraction to standardize it.
 */
#if defined(DRV_OS_WINDOWS)
#define DRV_STDUP DRV_WCSDUP
#define DRV_FORMAT_STRING(x) L##x
#define DRV_PRINT_STRING(stream, format, ...)                                  \
	fwprintf((stream), (format), __VA_ARGS__)
#else
#define DRV_STDUP DRV_STRDUP
#define DRV_FORMAT_STRING(x) x
#define DRV_PRINT_STRING(stream, format, ...)                                  \
	fprintf((stream), (format), __VA_ARGS__)
#endif

/*
 * OS return types
 */
#if defined(DRV_OS_UNIX)
#define OS_STATUS int
#define OS_SUCCESS 0
#if defined(BUILD_DRV_ESX)
#define OS_ILLEGAL_IOCTL -1
#define OS_NO_MEM -2
#define OS_FAULT -3
#define OS_INVALID -4
#define OS_NO_SYSCALL -5
#define OS_RESTART_SYSCALL -6
#define OS_IN_PROGRESS -7
#else
#define OS_ILLEGAL_IOCTL -ENOTTY
#define OS_NO_MEM -ENOMEM
#define OS_FAULT -EFAULT
#define OS_INVALID -EINVAL
#define OS_NO_SYSCALL -ENOSYS
#define OS_RESTART_SYSCALL -ERESTARTSYS
#define OS_IN_PROGRESS -EALREADY
#endif
#endif
#if defined(DRV_OS_WINDOWS)
#define OS_STATUS NTSTATUS
#define OS_SUCCESS STATUS_SUCCESS
#define OS_ILLEGAL_IOCTL STATUS_UNSUCCESSFUL
#define OS_NO_MEM STATUS_UNSUCCESSFUL
#define OS_FAULT STATUS_UNSUCCESSFUL
#define OS_INVALID STATUS_UNSUCCESSFUL
#define OS_NO_SYSCALL STATUS_UNSUCCESSFUL
#define OS_RESTART_SYSCALL STATUS_UNSUCCESSFUL
#define OS_IN_PROGRESS STATUS_UNSUCCESSFUL
#endif

/****************************************************************************
 **  Driver State defintions
 ***************************************************************************/
#define DRV_STATE_UNINITIALIZED 0
#define DRV_STATE_RESERVED 1
#define DRV_STATE_IDLE 2
#define DRV_STATE_PAUSED 3
#define DRV_STATE_STOPPED 4
#define DRV_STATE_RUNNING 5
#define DRV_STATE_PAUSING 6
#define DRV_STATE_PREPARE_STOP 7
#define DRV_STATE_TERMINATING 8

#define MATCHING_STATE_BIT(state) ((U32)1 << state)
#define STATE_BIT_UNINITIALIZED MATCHING_STATE_BIT(DRV_STATE_UNINITIALIZED)
#define STATE_BIT_RESERVED MATCHING_STATE_BIT(DRV_STATE_RESERVED)
#define STATE_BIT_IDLE MATCHING_STATE_BIT(DRV_STATE_IDLE)
#define STATE_BIT_PAUSED MATCHING_STATE_BIT(DRV_STATE_PAUSED)
#define STATE_BIT_STOPPED MATCHING_STATE_BIT(DRV_STATE_STOPPED)
#define STATE_BIT_RUNNING MATCHING_STATE_BIT(DRV_STATE_RUNNING)
#define STATE_BIT_PAUSING MATCHING_STATE_BIT(DRV_STATE_PAUSING)
#define STATE_BIT_PREPARE_STOP MATCHING_STATE_BIT(DRV_STATE_PREPARE_STOP)
#define STATE_BIT_TERMINATING MATCHING_STATE_BIT(DRV_STATE_TERMINATING)
#define STATE_BIT_ANY ((U32)-1)

#define IS_COLLECTING_STATE(state)                                             \
	(!!(MATCHING_STATE_BIT(state) &                                        \
		(STATE_BIT_RUNNING | STATE_BIT_PAUSING | STATE_BIT_PAUSED)))

/*
 *  Stop codes
 */
#define DRV_STOP_BASE 0
#define DRV_STOP_NORMAL 1
#define DRV_STOP_ASYNC 2
#define DRV_STOP_CANCEL 3
#define SEP_FREE(loc)                   \
	{                               \
		if ((loc)) {            \
			free(loc);      \
			loc = NULL;     \
		}                       \
	}

#define MAX_EVENTS 256 // Limiting maximum multiplexing events to 256.
#if defined(DRV_OS_UNIX)
#define UNREFERENCED_PARAMETER(p) ((p) = (p))
#endif

/*
 * Global marker names
 */
#define START_MARKER_NAME "SEP_START_MARKER"
#define PAUSE_MARKER_NAME "SEP_PAUSE_MARKER"
#define RESUME_MARKER_NAME "SEP_RESUME_MARKER"

#define DRV_SOC_STRING_LEN (100 + MAX_MARKER_LENGTH)

/*
 * Temp path
 */
#define SEP_TMPDIR "SEP_TMP_DIR"
#if defined(DRV_OS_WINDOWS)
#define OS_TMPDIR "TEMP"
#define GET_DEFAULT_TMPDIR(dir, size)                                  \
	{                                                              \
		GetTempPath((U32)size, dir);                           \
	}
#else
#define OS_TMPDIR "TMPDIR"
/*
 * Unix has default tmp dir
 */
#if defined(DRV_OS_ANDROID)
#define TEMP_PATH "/data"
#else
#define TEMP_PATH "/tmp"
#endif
#define GET_DEFAULT_TMPDIR(dir, size)                                       \
	{                                                                   \
		DRV_STRCPY((STCHAR *)dir, (U32)size, (STCHAR *)TEMP_PATH);  \
	}
#endif

#define OS_ID_UNKNOWN -1
#define OS_ID_NATIVE 0
#define OS_ID_VMM 0
#define OS_ID_MODEM 1
#define OS_ID_ANDROID 2
#define OS_ID_SECVM 3
#define OS_ID_ACORN 0xFFFF

#define PERF_HW_VER4 (5)
#if defined(__cplusplus)
}
#endif

#endif
