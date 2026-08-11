/***************************************************************************//**
* @file   iio_ad7616.h
* @brief  Header file for AD7616 IIO interface
********************************************************************************
* Copyright (c) 2024 Analog Devices, Inc.
* Copyright (c) 2024 BayLibre, SAS.
*
* SPDX-License-Identifier: BSD-3-Clause
*******************************************************************************/
#ifndef _AD7616_IIO_H_
#define _AD7616_IIO_H_

#include <stdint.h>

#include "iio.h"
#include "iio_types.h"
#include "ad7616.h"

struct ad7616_iio_dev {
	struct ad7616_dev *ad7616_dev;
	struct iio_device *iio_dev;
};

/* Init the IIO interface */
int ad7616_iio_init(struct ad7616_iio_dev **dev,
		    struct ad7616_init_param *init_param);
int ad7616_iio_remove(struct ad7616_iio_dev *dev);

void iio_ad7616_event_handler(struct iio_desc *desc);

#endif /* __AD7616_IIO_H__ */
