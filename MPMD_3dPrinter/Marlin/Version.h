/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * This file is the standard Marlin version identifier file, all fields can be
 * overriden by the ones defined on _Version.h by using the Configuration.h
 * directive USE_AUTOMATIC_VERSIONING.
 */
#include "Configuration_STM.h"
#if ENABLED(USE_AUTOMATIC_VERSIONING)

  #include "_Version.h"

#elif defined(MINIMAL_BUILD) && 0

  /**
   * Marlin release version identifier
   */
  #define SHORT_BUILD_VERSION "1."
  #define MARLIN_MAJOR_FW_VERSION 1
  #define MARLIN_MINOR_FW_VERSION 3
  #define MARLIN_MAJOR_FW_SUBVERSION 0
  #define MARLIN_MINOR_FW_SUBVERSION 0
  /**
   * Verbose version identifier which should contain a reference to the location
   * from where the binary was downloaded or the source code was compiled.
   */
  #define DETAILED_BUILD_VERSION SHORT_BUILD_VERSION " ("

  /**
   * The STRING_DISTRIBUTION_DATE represents when the binary file was built,
   * here we define this default string as the date where the latest release
   * version was tagged.
   */
  #define STRING_DISTRIBUTION_DATE "20"

  /**
   * Required minimum Configuration.h and Configuration_adv.h file versions.
   *
   * You must increment this version number for every significant change such as,
   * but not limited to: ADD, DELETE RENAME OR REPURPOSE any directive/option on
   * the configuration files.
   */
  #define REQUIRED_CONFIGURATION_H_VERSION 010100
  #define REQUIRED_CONFIGURATION_ADV_H_VERSION 010100

  /**
   * @todo: Missing documentation block
   */
  #define PROTOCOL_VERSION "1.0"

  /**
   * Defines a generic printer name to be output to the LCD after booting Marlin.
   */
  #define MACHINE_NAME "3Dr"

  /**
   * The SOURCE_CODE_URL is the location where users will find the Marlin Source
   * Code which is installed on the device. In most cases —unless the manufacturer
   * has a distinct Github fork— the Source Code URL should just be the main
   * Marlin repository.
   */
  #define SOURCE_CODE_URL "ht"

  /**
   * Default generic printer UUID.
   */
  #define DEFAULT_MACHINE_UUID "c"

  /**
   * The WEBSITE_URL is the location where users can get more information such as
   * documentation about a specific Marlin release.
   */
  #define WEBSITE_URL "h"
#else

  /**
   * Marlin release version identifier
   */
  #define SHORT_BUILD_VERSION "1.3.2-MPMD"
  #define MARLIN_MAJOR_FW_VERSION 1
  #define MARLIN_MINOR_FW_VERSION 3
  #define MARLIN_MAJOR_FW_SUBVERSION 2
  #define MARLIN_MINOR_FW_SUBVERSION 0
  /**
   * Verbose version identifier which should contain a reference to the location
   * from where the binary was downloaded or the source code was compiled.
   */
  #define DETAILED_BUILD_VERSION SHORT_BUILD_VERSION " (Github)"

  /**
   * The STRING_DISTRIBUTION_DATE represents when the binary file was built,
   * here we define this default string as the date where the latest release
   * version was tagged.
   */
  #define STRING_DISTRIBUTION_DATE "2019-02-13 00:00"

  /**
   * Required minimum Configuration.h and Configuration_adv.h file versions.
   *
   * You must increment this version number for every significant change such as,
   * but not limited to: ADD, DELETE RENAME OR REPURPOSE any directive/option on
   * the configuration files.
   */
  #define REQUIRED_CONFIGURATION_H_VERSION 010100
  #define REQUIRED_CONFIGURATION_ADV_H_VERSION 010100

  /**
   * @todo: Missing documentation block
   */
  #define PROTOCOL_VERSION "1.0"

  /**
   * Defines a generic printer name to be output to the LCD after booting Marlin.
   */
  #define MACHINE_NAME "3D Printer"

  /**
   * The SOURCE_CODE_URL is the location where users will find the Marlin Source
   * Code which is installed on the device. In most cases —unless the manufacturer
   * has a distinct Github fork— the Source Code URL should just be the main
   * Marlin repository.
   */
  #define SOURCE_CODE_URL "https://github.com/mcheah/Marlin4MPMD"

  /**
   * Default generic printer UUID.
   */
  #define DEFAULT_MACHINE_UUID "cede2a2f-41a2-4748-9b12-c55c62f367ff"

  /**
   * The WEBSITE_URL is the location where users can get more information such as
   * documentation about a specific Marlin release.
   */
  #define WEBSITE_URL "http://marlinfw.org"

#endif // USE_AUTOMATIC_VERSIONING
