/***************************************************************************//**
* @file   iio_ad7606.h
* @brief  Header file for AD7606 IIO interface
********************************************************************************
* Copyright (c) 2024 Analog Devices, Inc.
* Copyright (c) 2024 BayLibre, SAS.
*
* SPDX-License-Identifier: BSD-3-Clause
*******************************************************************************/
#ifndef _AD7606_IIO_H_
#define _AD7606_IIO_H_

#include <stdint.h>

#include "iio.h"
#include "iio_types.h"
#include "ad7606.h"

struct ad7606_iio_dev {
	struct ad7606_dev *ad7606_dev;
	struct iio_device *iio_dev;
	int sign_bit;
};

/* Init the IIO interface */
int ad7606_iio_init(struct ad7606_iio_dev **dev,
		    struct ad7606_init_param *init_param);
int ad7606_iio_remove(struct ad7606_iio_dev *dev);

#endif /* __AD7606_IIO_H__ */
