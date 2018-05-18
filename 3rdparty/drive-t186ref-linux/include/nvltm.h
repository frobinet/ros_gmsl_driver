/**
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file nvltm.h
 * \brief The Local Tone Map API
 *
 * This file contains the \ref ltm_api "Local Tone Map API".
 */

#ifndef _NVLTM_H
#define _NVLTM_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \defgroup ltm_api Local Tone Map API
 *
 * The Local Tone Map API encompasses functions required to perform
 * client side Tone Map creation from NvMedia IPP generated Tone Map
 * information.
 *
 * @{
 */

/** \brief Major Version number */
#define NVLTM_VERSION_MAJOR         1
/** \brief Minor Version number */
#define NVLTM_VERSION_MINOR         0

/** \brief Default lookup table width */
#define NVLTM_DEFAULT_LOOKUP_WIDTH  32
/** \brief Default lookup table height */
#define NVLTM_DEFAULT_LOOKUP_HEIGHT 32
/** \brief Default lookup table depth */
#define NVLTM_DEFAULT_LOOKUP_DEPTH  64
/** \brief Maximum lookup table width */
#define NVLTM_MAX_LOOKUP_WIDTH  256
/** \brief Maximum lookup table height */
#define NVLTM_MAX_LOOKUP_HEIGHT 256
/** \brief Maximum lookup table depth */
#define NVLTM_MAX_LOOKUP_DEPTH 512

/**
 * \hideinitializer
 * \brief The set of all possible error codes.
 */
typedef enum {
    /** \hideinitializer The operation completed successfully; no error. */
    NVLTM_STATUS_OK = 0,
    /** Bad parameter was passed. */
    NVLTM_STATUS_BAD_PARAMETER,
    /** Out of memory. */
    NVLTM_STATUS_OUT_OF_MEMORY,
    /** Error */
    NVLTM_STATUS_ERROR
} NvLTMStatus;

/**
 * \brief NvLTM Version descriptor.
 */
typedef struct {
    /*! Major version */
    unsigned char major;
    /*! Minor version */
    unsigned char minor;
} NvLTMVersion;

/**
 * \brief Version information for the NvLTM library.
 */
typedef struct {
    /*! Library version information */
    NvLTMVersion libVersion;
} NvLTMVersionInfo;

/**
 * \brief  An opaque handle representing an NvLTM object.
 */
typedef void * NvLTMHandle;

/**
 * \brief Configuration for NvLTM object creation.
 */
typedef struct {
    /*! Maximum width of the lookup table */
    unsigned int lookupWidth;
    /*! Maximum height of the lookup table */
    unsigned int lookupHeight;
    /*! Maximum depth of the lookup table */
    unsigned int lookupDepth;
} NvLTMConfiguration;

/**
 * \brief Create an NvLTM object.
 * \param[in] ltmConfig Configuration structure.
 * \param[out] nvltm The created NvLTM handle.
 * \return \ref NvLTMStatus. \ref NVLTM_STATUS_OK or \ref NVLTM_STATUS_ERROR
 */
NvLTMStatus
NvLTMCreate(
    NvLTMConfiguration *ltmConfig,
    NvLTMHandle *nvltm
);

/**
 * \brief Destroy an NvLTM object.
 * \param[in] nvltm The NvLTM handle to be destroyed.
 * \return void
 */
void
NvLTMDestroy(
    NvLTMHandle nvltm
);


/**
 * \brief Get the lookup table size. This function helps to allocate
 * memory for the lookup table.
 * \param[in] nvltm The NvLTM handle.
 * \param[out] maxLookupTableSize Pointer to the maximum size of the lookup
 * table filled by this function.
 * \return \ref NvLTMStatus. \ref NVLTM_STATUS_OK or \ref NVLTM_STATUS_ERROR
 */
NvLTMStatus
NvLTMGetLookupTableSize(
    NvLTMHandle nvltm,
    unsigned int *maxLookupTableSize
);

/**
 * \brief Parameters of NvLTM lookup table.
 */
typedef struct {
    /*! width of the lookup table */
    unsigned int width;
    /*! Height of the lookup table */
    unsigned int height;
    /*! Depth of the lookup table */
    unsigned int depth;
} NvLTMTableDimension;

/**
 * \brief Get the lookup table parameters from metadata.
 * \param[in] nvltm The NvLTM handle.
 * \param[in] ltmData Pointer to the LTM data providied by the IPP.
 * \param[out] dimension Pointer to an \ref NvLTMTableDimension structure.
 * \return \ref NvLTMStatus. \ref NVLTM_STATUS_OK or \ref NVLTM_STATUS_BAD_PARAMETER
 */
NvLTMStatus
NvLTMGetLookupTableDimension(
    NvLTMHandle nvltm,
    void *ltmData,
    NvLTMTableDimension *dimension
);

/**
 * \brief Process the NvMedia IPP generated Local Tone Map information
 * into a table that is used for GPU tone mapping
 * \param[in] ltmData Pointer to the LTM data providied by the IPP.
 * \param[out] lookupTable Pointer to a client allocated memory where the lookup table
 * is going to be generated.
 * \return \ref NvLTMStatus. \ref NVLTM_STATUS_OK or \ref NVLTM_STATUS_ERROR
 */
NvLTMStatus
NvLTMProcessToneMap(
    NvLTMHandle nvltm,
    void *ltmData,
    void *lookupTable);

/**
 * \brief Returns the version information for the NvLTM library.
 * \param[in] versionInfo Pointer to a \ref NvLTMVersionInfo structure
 *                        to be filled by the function.
 * \return \ref NvLTMStatus The completion status of the operation.
 * Possible values are:
 * \n \ref NVLTM_STATUS_OK
 * \n \ref NVLTM_STATUS_BAD_PARAMETER if the pointer is invalid.
 */
NvLTMStatus
NvLTMGetVersionInfo(
    NvLTMVersionInfo *versionInfo
);

/*@}*/

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* _NVLTM_H */
