/**
 * @brief OS abstracted Library Loader class for DLLs/shared-objects.
 * @author Ozrien
 *
 * This is useful for basic plugin systems.

 * Example Use...
 * <pre>
 * {@code
 *     ctre::phoenix::runtime::LibLoader ldr;
 *     typedef int (*example_func_t)(...params...);
 *     try {
 *         ldr.Open("example.dll");
 *         int retval = LIBLOADER_LOOKUP(ldr, example_func_t)(...params...);
 *     } catch (const LibLoaderException & e) {}
 * }
 * </pre>
 */
#pragma once

#include "ctre/phoenix/ErrorCode.h"
#include <stdexcept>
#include <string>
#include <map>

 /**
  * Use preprocessor to simplifiy getting func ptr by name, then calling.
  * Example use: LIBLOADER_LOOKUP(loader, some_func_typedef)(... some func params ...)
  */
#define LIBLOADER_LOOKUP(libloader, funcT) (libloader).LookupFunc<funcT>(#funcT)

namespace ctre {
	namespace phoenix {
		namespace runtime {
			/**
			* LibLoaderException
			*/
			class LibLoaderException : public std::runtime_error {
			public:

				LibLoaderException(phoenix::ErrorCode errorCode, const std::string& BaseMessage, const std::string & libPath)
					: std::runtime_error((BaseMessage + " " + libPath).c_str())
				{
					_errorCode = errorCode;
				}
				LibLoaderException(phoenix::ErrorCode errorCode, const std::string& BaseMessage, const std::string & libPath, const std::string & funcName)
					: std::runtime_error((BaseMessage + " " + funcName + " in " + libPath).c_str())
				{
					_errorCode = errorCode;
				}

				phoenix::ErrorCode GetPhoenixErrorCode() const
				{
					return _errorCode;
				}
			private:
				phoenix::ErrorCode _errorCode;
			};
		} // runtime
	} // phoenix
} // ctre

#if defined(WIN32) || defined(_WIN32) || defined(_WIN64)
#include <Windows.h>
namespace ctre {
	namespace phoenix {
		namespace runtime {
			/**
			 * LibLoader
			 */
			class LibLoader {

			public:
				/** default c'tor */
				LibLoader() { /* empty */ }

				/* no copy c'tr */
				LibLoader(const LibLoader&) = delete;

				/** d'tor */
				~LibLoader() { Close(); }

				/**
				 * @param libPath filePath to dll to load, passed into LoadLibrary.
				 * Routine will throw LibLoaderException on failure.
				 */
				void Open(const std::string & libPath) {
					/* if library already loaded, close it and start all over */
					Close();
					/* keep a copy for posterity */
					_libPath = libPath;
					/* attempt to load the module (dll) */
					_handle = LoadLibraryA(_libPath.c_str());
					/* throw if it didn't load */
					if (_handle == NULL) { throw LibLoaderException(ErrorCode::LibraryCouldNotBeLoaded, "Could not load", _libPath); }
				}
                /**
                * @return true if DLL/SO is loaded.
                */
                bool IsOpen() const{
                        return (_handle != NULL);
                }

				/**
				* Close DLL and clear func ptrs.
				*/
				void Close() {
					/* dump the table, this is not necessary but good to dump bad func ptr*/
					_mp.clear();
					/* free the module */
					if (_handle != NULL) {
						FreeLibrary(_handle);
						_handle = NULL;
					}
				}
				/**
				 * Lookup a function by name inside DLL.  If func has been found already, performance should
				 * be fast since it is a map lookup.
				 * @param funcName string copy of function name.
				 * @return pointer to found function.
				 * Routine will throw LibLoaderException on failure.
				 */
				template<typename T> T LookupFunc(const std::string & funcName)
				{
					/* make sure Open was called successfully */
					if (_handle == NULL) { throw LibLoaderException(ErrorCode::LibraryCouldNotBeLoaded, "Could not load", _libPath); }

					/* first check map, we might have it already */
					auto iter = _mp.find(funcName);
					if (iter != _mp.end()) { return (T)iter->second; }

					/* pull func ptr out of lib */
					T funcPtr = (T)GetProcAddress(_handle, funcName.c_str());

					/* did we get it? */
					if (funcPtr == NULL) {
						/* couldn't find it dll, throw to caller */
						throw LibLoaderException(ErrorCode::MissingRoutineInLibrary, "Could not find routine", _libPath, funcName);
					}
					/* insert into coll */
					_mp[funcName] = (void*)funcPtr;

					/* pass new ptr to caller */
					return funcPtr;
				}

			private:
				std::string _libPath; //!< Copy of lib path, mostly for error reporting
				std::map<std::string, void *> _mp; //!< Coll for func ptrs we've found this far
				HINSTANCE _handle = NULL; //!< handle to module (dll)
			};
		} // runtime
	} // phoenix
} // ctre
#else
#include <dlfcn.h>
#include <iostream>

namespace ctre {
	namespace phoenix {
		namespace runtime {
			/**
			 * LibLoader
			 */
			class LibLoader {

			public:
				/** default c'tor */
				LibLoader() { /* empty */ }

				/* no copy c'tr */
				LibLoader(const LibLoader&) = delete;

				/** d'tor */
				~LibLoader() { Close(); }

				/**
				 * @param libPath filePath to dll to load, passed into LoadLibrary.
				 * Routine will throw LibLoaderException on failure.
				 */
				void Open(const std::string & libPath) {
                    /* if library already loaded, close it and start all over */
                    Close();
                    /* keep a copy for posterity */
					_libPath = libPath;
					/* attempt to load the module (dll) */

                    _handle = dlopen(_libPath.c_str(), RTLD_NOW);
					
					/* throw if it didn't load */
					if (!_handle) { 
                        std::string baseErr = dlerror();
                        baseErr +=  " | With path: ";

                        std::cout << baseErr << _libPath << std::endl; 
                        throw; //LibLoaderException(ErrorCode::LibraryCouldNotBeLoaded, baseErr, _libPath); 
                    }
				}
                /**
                * @return true if DLL/SO is loaded.
                */
                bool IsOpen() const{
                        return (_handle != NULL);
                }
				
                /**
				* Close DLL and clear func ptrs.
				*/
				void Close(bool performDlclose) {
					/* dump the table, this is not necessary but good to dump bad func ptr*/
					_mp.clear();
					/* free the module (fails in linux for unknown reasons). */
                    if (_handle && performDlclose) {
                        dlclose(_handle);
                    }
				}
    
                void Close() {
                    Close(false);
                }

				/**
				 * Lookup a function by name inside DLL.  If func has been found already, performance should
				 * be fast since it is a map lookup.
				 * @param funcName string copy of function name.
				 * @return pointer to found function.
				 * Routine will throw LibLoaderException on failure.
				 */
				template<typename T> T LookupFunc(const std::string & funcName)
				{
					/* make sure Open was called successfully */
					if (!_handle) { 
                        std::string baseErr = dlerror();
                        baseErr +=  " | With path: ";

                        std::cout << baseErr << _libPath << std::endl; 
                        throw; //LibLoaderException(ErrorCode::LibraryCouldNotBeLoaded,baseErr, _libPath); 
                    }

					/* first check map, we might have it already */
					auto iter = _mp.find(funcName);
					if (iter != _mp.end()) { return (T)iter->second; }

					/* pull func ptr out of lib */
					T funcPtr = (T)dlsym(_handle, funcName.c_str());

					/* did we get it? */
					if (funcPtr == NULL) {
						/* couldn't find it dll, throw to caller */
                        std::string baseErr = dlerror();
                        baseErr +=  " | For: ";	                    					

                        std::cout << baseErr << funcName << " in " << _libPath << std::endl; 
    
                        throw;// LibLoaderException(ErrorCode::MissingRoutineInLibrary,"Could not find routine", _libPath, funcName);
					}
					/* insert into coll */
					_mp[funcName] = (void*)funcPtr;

					/* pass new ptr to caller */
					return funcPtr;
				}

			private:
				std::string _libPath; //!< Copy of lib path, mostly for error reporting
				std::map<std::string, void *> _mp; //!< Coll for func ptrs we've found this far
				void *_handle = NULL; //!< handle to module (dll)
			};
		} // runtime
	} // phoenix
} // ctre


#endif
