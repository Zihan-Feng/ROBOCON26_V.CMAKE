/**
 * @file UsbPortC.h
 * @author Keten (2863861004@qq.com)
 * @brief 由于USB_DEVICE 是纯c库，因此需要包装c接口供其调用
 * @version 0.1
 * @date 2026-04-19
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void UsbPort_OnRxFromIsr(const uint8_t *data, uint32_t len);
void UsbPort_OnTxCpltFromIsr(void);
void UsbPort_PumpTx(void);
uint8_t UsbPort_WriteAsync(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif
