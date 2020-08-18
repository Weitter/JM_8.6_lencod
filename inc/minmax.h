
/*!
 ************************************************************************
 *  \file
 *     minmax.h
 *  \brief
 *     defines min and max macros for non WIN32 environments
 ************************************************************************
 */
#ifndef _MINMAX_
#define _MINMAX_

#ifndef WIN32
#define max(a, b) (((a) > (b)) ? (a) : (b))
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#endif

