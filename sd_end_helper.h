// sd_end_helper.h - tenere nella stessa cartella dello sketch.
// STM32SD (bsp_sd.c) inizializza l'hardware solo se lo stato HAL e' RESET: dopo un tentativo fallito,
// o dopo aver tolto l'alimentazione alla SD, serve un DeInit, altrimenti begin() non riparte.
// Usa SD.end() se la tua versione di STM32SD ce l'ha, altrimenti BSP_SD_DeInit().
#pragma once
#include <STM32SD.h>

extern "C" uint8_t BSP_SD_DeInit(void);

template <class T>
inline auto sdEndImpl(T &card, int) -> decltype(card.end(), void()) { card.end(); }

template <class T>
inline void sdEndImpl(T &, long) { BSP_SD_DeInit(); }

inline void sdEnd() { sdEndImpl(SD, 0); }
