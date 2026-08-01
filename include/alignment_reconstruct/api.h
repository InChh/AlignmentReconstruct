#pragma once

#if defined(_WIN32)
#if defined(ALIGNMENT_RECONSTRUCT_EXPORTS)
#define ALIGNMENT_RECONSTRUCT_API __declspec(dllexport)
#else
#define ALIGNMENT_RECONSTRUCT_API __declspec(dllimport)
#endif
#else
#define ALIGNMENT_RECONSTRUCT_API
#endif
