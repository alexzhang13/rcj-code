/*
#sourcefile  navigate_defs.h
#category    API
#description navigation library for maze robot
#title       navigation algorithms API
#parentlink  mindex.html
*
* Copyright Alex Zhang
* All Rights Reserved
*
*              @(#) $Id$
*/

#ifndef NAVIGATE_DEFS_H
#define NAVIGATE_DEFS_H

#ifdef WIN32
    #ifdef NAVIGATE_EXPORTS
      #define NAVIGATE_EXPORT __declspec(dllexport)
    #else
      #define NAVIGATE_EXPORT __declspec(dllimport)
    #endif
#else
  #define NAVIGATE_EXPORT
#endif

#endif /* NAVIGATE_DEFS_H */
/* =================================================================== */
/* end navigate_defs.h */

