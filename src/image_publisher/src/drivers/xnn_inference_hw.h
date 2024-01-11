// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
// AXI_CPU
// 0x0000 : Control signals
//          bit 0  - ap_start (Read/Write/COH)
//          bit 1  - ap_done (Read/COR)
//          bit 2  - ap_idle (Read)
//          bit 3  - ap_ready (Read)
//          bit 7  - auto_restart (Read/Write)
//          others - reserved
// 0x0004 : Global Interrupt Enable Register
//          bit 0  - Global Interrupt Enable (Read/Write)
//          others - reserved
// 0x0008 : IP Interrupt Enable Register (Read/Write)
//          bit 0  - enable ap_done interrupt (Read/Write)
//          bit 1  - enable ap_ready interrupt (Read/Write)
//          others - reserved
// 0x000c : IP Interrupt Status Register (Read/TOW)
//          bit 0  - ap_done (COR/TOW)
//          bit 1  - ap_ready (COR/TOW)
//          others - reserved
// 0x0010 : Data signal of Pred_out
//          bit 31~0 - Pred_out[31:0] (Read)
// 0x0014 : Control signal of Pred_out
//          bit 0  - Pred_out_ap_vld (Read/COR)
//          others - reserved
// 0x1000 ~
// 0x1fff : Memory 'Data_In' (768 * 32b)
//          Word n : bit [31:0] - Data_In[n]
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XNN_INFERENCE_AXI_CPU_ADDR_AP_CTRL       0x0000
#define XNN_INFERENCE_AXI_CPU_ADDR_GIE           0x0004
#define XNN_INFERENCE_AXI_CPU_ADDR_IER           0x0008
#define XNN_INFERENCE_AXI_CPU_ADDR_ISR           0x000c
#define XNN_INFERENCE_AXI_CPU_ADDR_PRED_OUT_DATA 0x0010
#define XNN_INFERENCE_AXI_CPU_BITS_PRED_OUT_DATA 32
#define XNN_INFERENCE_AXI_CPU_ADDR_PRED_OUT_CTRL 0x0014
#define XNN_INFERENCE_AXI_CPU_ADDR_DATA_IN_BASE  0x1000
#define XNN_INFERENCE_AXI_CPU_ADDR_DATA_IN_HIGH  0x1fff
#define XNN_INFERENCE_AXI_CPU_WIDTH_DATA_IN      32
#define XNN_INFERENCE_AXI_CPU_DEPTH_DATA_IN      768

