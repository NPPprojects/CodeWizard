/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#ifndef GATT_SVR_H
#define GATT_SVR_H

/* Includes */
/* NimBLE GATT APIs */
#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"

/* NimBLE GAP APIs */
#include "host/ble_gap.h"


/* Maximum length for received string */
#define MAX_RECEIVED_STRING_LEN 128

/* Global variable to store the received string */
extern char received_string[MAX_RECEIVED_STRING_LEN];
extern SemaphoreHandle_t received_string_semaphore;





/* Public function declarations */
void send_heart_rate_indication(void);
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
void gatt_svr_subscribe_cb(struct ble_gap_event *event);
int gatt_svc_init(void);

#endif // GATT_SVR_H
