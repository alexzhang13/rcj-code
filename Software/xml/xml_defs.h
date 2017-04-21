/*
#sourcefile  xml_defs.h
#category    API
#description xml library for maze robot
#title       xml read/write IO API
#parentlink  mindex.html
*
* Copyright Alex Zhang
* All Rights Reserved
*
*              @(#) $Id$
*/

#ifndef XML_DEFS_H
#define XML_DEFS_H

#ifdef WIN32
    #ifdef XML_EXPORTS
      #define XML_EXPORT __declspec(dllexport)
    #else
      #define XML_EXPORT __declspec(dllimport)
    #endif
#else
  #define XML_EXPORT
#endif

#endif /* XML_DEFS_H */
/* =================================================================== */
/* end xml_defs.h */

