
#ifndef HRVO_EXPORT_H
#define HRVO_EXPORT_H

#ifdef HRVO_STATIC_DEFINE
#  define HRVO_EXPORT
#  define HRVO_NO_EXPORT
#else
#  ifndef HRVO_EXPORT
#    ifdef HRVO_EXPORTS
        /* We are building this library */
#      define HRVO_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define HRVO_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef HRVO_NO_EXPORT
#    define HRVO_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef HRVO_DEPRECATED
#  define HRVO_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef HRVO_DEPRECATED_EXPORT
#  define HRVO_DEPRECATED_EXPORT HRVO_EXPORT HRVO_DEPRECATED
#endif

#ifndef HRVO_DEPRECATED_NO_EXPORT
#  define HRVO_DEPRECATED_NO_EXPORT HRVO_NO_EXPORT HRVO_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef HRVO_NO_DEPRECATED
#    define HRVO_NO_DEPRECATED
#  endif
#endif

#endif /* HRVO_EXPORT_H */
