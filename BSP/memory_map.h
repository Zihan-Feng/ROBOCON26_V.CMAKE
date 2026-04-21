/**
 * @file memory_map.h
 * @author Keten (2863861004@qq.com)
 * @brief 定义链接脚本段位置，需要查看链接脚本
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

// dtcmram为dma不可达区域，因此需要额外定义dma缓冲区
#if defined(__GNUC__)
#define DMA_BUFFER_ATTR __attribute__((section(".dma_buffer"), aligned(32)))
#else
#define DMA_BUFFER_ATTR
#endif