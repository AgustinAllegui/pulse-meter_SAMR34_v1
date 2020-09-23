/**
 * @file logMacros.h
 * @author Agustin Allegui (a.allegui@gmail.com.com)
 * @brief Macros para mensajes de log.
 * @version 0.1
 * @date 2020-09-17
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef LOG_MACROS_H
#define LOG_MACROS_H

// definiciones de niveles de log

#define NONE_LEVEL 0
#define FATAL_LEVEL 1
#define ERROR_LEVEL 2
#define WARNING_LEVEL 3
#define INFO_LEVEL 4
#define DEBUG_LEVEL 5
#define TRACE_LEVEL 6

/**
 * @brief Nivel de mesajes de log en la consola.
 * 
 * @note
 * NONE_LEVEL
 * FATAL_LEVEL
 * ERROR_LEVEL
 * WARNING_LEVEL
 * INFO_LEVEL
 * DEBUG_LEVEL
 * TRACE_LEVEL
 */
#define LOG_LEVEL TRACE_LEVEL

//------------------------------------------------
// Definiciones de los macros de log

#if LOG_LEVEL >= FATAL_LEVEL
#define logFatal(format, ...) printf("[FATAL] " format "\r\n", ##__VA_ARGS__)
#else
#define logFatal(...)
#endif

#if LOG_LEVEL >= ERROR_LEVEL
#define logError(format, ...) printf("[ERROR] " format "\r\n", ##__VA_ARGS__)
#else
#define logError(...)
#endif

#if LOG_LEVEL >= WARNING_LEVEL
#define logWarning(format, ...) printf("[WARNING] " format "\r\n", ##__VA_ARGS__)
#else
#define logWarning(...)
#endif

#if LOG_LEVEL >= INFO_LEVEL
#define logInfo(format, ...) printf("[INFO] " format "\r\n", ##__VA_ARGS__)
#else
#define logInfo(...)
#endif

#if LOG_LEVEL >= DEBUG_LEVEL
#define logDebug(format, ...) printf("[DEBUG] " format "\r\n", ##__VA_ARGS__)
#else
#define logDebug(...)
#endif

#if LOG_LEVEL >= TRACE_LEVEL
#define logTrace(format, ...) printf("[TRACE] " format "\r\n", ##__VA_ARGS__)
#else
#define logTrace(...)
#endif

#endif
