#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include "Configuration.h"

#if defined(SD_SETTINGS)
#include "cardreader.h"
extern CardReader *p_card;
#endif

void Config_ResetDefault();

#ifndef DISABLE_M503
void Config_PrintSettings();
#else
FORCE_INLINE void Config_PrintSettings() {}
#endif

#if defined(EEPROM_SETTINGS)||defined(SD_SETTINGS)
void Config_StoreSettings();
void Config_RetrieveSettings();
#else
FORCE_INLINE void Config_StoreSettings() {}
FORCE_INLINE void Config_RetrieveSettings() { Config_ResetDefault(); Config_PrintSettings(); }
#endif

#endif//CONFIG_STORE_H
