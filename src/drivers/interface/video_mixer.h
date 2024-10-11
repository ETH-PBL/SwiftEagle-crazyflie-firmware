/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 */

 
#ifndef SRC_VIDEOMIXER_H_
#define SRC_VIDEOMIXER_H_


int mixer_init(void);
int mixer_stop_dvs_layer(void);
int mixer_start_dvs_layer(void);

#endif /* SRC_VIDEOMIXER_H_ */
