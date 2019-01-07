/**
 * This header implements macros for creating storage objects 
 * with 1 byte (8bit) packing across all supported platforms.
 *
 * Example use...
 *
 *     PACK_PREFIX
 *     typedef struct _nameOfType_t
 *     {
 *         uint8_t aByte;
 *         uint16_t aShort;
 *         uint32_t aWord;
 *     } PACK_ATTRIB nameOfType_t;
 *     PACK_SUFFIX
 *
 */
#pragma once

#ifdef __GNUC__
	/* All GCC versions supports packed attribute */
	#define PACK_PREFIX /* empty */
	#define PACK_SUFFIX /* empty */
	#define PACK_ATTRIB __attribute__ ((packed))
#else
	/* MSVC uses pragma's to being and end regions of code */
	#define PACK_PREFIX	__pragma(pack(push, 1))
	#define PACK_SUFFIX	__pragma(pack(pop))
	#define PACK_ATTRIB /* empty */
#endif

/**
 * CTRE_ASSERT(cond)
 * This can be moved into a seperate header later (Platform-assert).
 */
#if defined(__GNUC__)
	#define CTRE_ASSERT(cond)	do{}while(0)
#elif defined(WIN32) || defined(_WIN32) || defined(_WIN64)
	/* Windows OS */
	#if defined (_DEBUG)
		/* Windows OS - Debug build */
		#include <assert.h>
		#define CTRE_ASSERT(cond)	assert(cond)
	#else
		/* Windows OS - Release build */
		#define CTRE_ASSERT(cond)	do{}while(0)
	#endif
#else
	#define CTRE_ASSERT(cond)	do{}while(0)
#endif

/**
* CTRE_Application_CrashHandler(cond)
* This can be moved into a seperate header later.
*/
#if defined(WIN32) || defined(_WIN32) || defined(_WIN64)
	#define CTRE_IMPLEMENT_SHUTDOWN_HANDLER(shutdown_handler)													\
											static void shutdown_handler();										\
											BOOL WINAPI CTRE_Global_ConsoleHandlerRoutine(DWORD dwCtrlType) {	\
												if (dwCtrlType == CTRL_CLOSE_EVENT) { shutdown_handler(); }		\
												return FALSE;													\
											}																	\
											static void shutdown_handler()


	#define CTRE_REGISTER_SHUTDOWN_HANDLER(shutdown_handler) \
		do{ (void)SetConsoleCtrlHandler(CTRE_Global_ConsoleHandlerRoutine, TRUE); } while(0)

#elif defined(__GNUC__)

	#include <sys/signal.h>		

	#define CTRE_IMPLEMENT_SHUTDOWN_HANDLER(shutdown_handler)	\
		static void shutdown_handler(int signo)	

	#define CTRE_REGISTER_SHUTDOWN_HANDLER(shutdown_handler) \
										{	\
											struct sigaction sigact;	\
											sigact.sa_handler = shutdown_handler;	\
											sigemptyset(&sigact.sa_mask);	\
											sigact.sa_flags = 0;	\
											sigaction(SIGINT, &sigact, NULL);	\
											sigaction(SIGTERM, &sigact, NULL);	\
										}
#else

	#define CTRE_IMPLEMENT_SHUTDOWN_HANDLER(shutdown_handler) static void shutdown_handler(int signo)	
	#define CTRE_REGISTER_SHUTDOWN_HANDLER(shutdown_handler)

#endif