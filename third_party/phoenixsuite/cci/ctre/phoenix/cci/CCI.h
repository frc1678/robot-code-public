#ifndef CCIEXPORT

	#if defined(WIN32) || defined(_WIN32) || defined(_WIN64)
			#define CCIEXPORT	__declspec(dllexport)
    #elif (defined(__GNUC__) && ((__GNUC__ > 4) || (__GNUC__ == 4) && (__GNUC_MINOR__ > 2))) || __has_attribute(visibility)
    	    #define CCIEXPORT     __attribute__((visibility("default")))
    #else
        #define CCIEXPORT
    #endif
#endif