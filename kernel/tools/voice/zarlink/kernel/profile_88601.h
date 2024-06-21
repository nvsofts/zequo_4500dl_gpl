/*
 * Multiline4ch_8ch_PW2_4.h --
 *
 * This header file exports the Profile data types
 *
 * Project Info --
 *   Type:   Design for ZLR88601 4 to 8 Line, Multiline, Lite Coefficients
 *   Date:   Thursday, August 08, 2013 10:08:23
 *   Device: ZL880 ZL88601
 *
 *   This file was generated with Profile Wizard Version: P2.4.0
 *
 * Project Comments --
 *  ----------------------------------------------------------------------------------------------------------------------------------
 *  Profile Wizard Coefficient Revision 2.9 Release Notes:
 *  ----------------------------------------------------------------------------------------------------------------------------------
 *  I. General:
 *  1. This is the second release of the ZL88601 worldwide coefficients.
 *  2. This release includes support for the following 44 countries:
 *  Argentina (AR), Austria (AT), Australia (AU), Belgium (BE), Brazil (BR), Bulgaria (BG), Canada (CA), Switzerland (CH),
 *  Chile (CL), China (CN), Czech Republic (CZ), Germany (DE), Denmark (DK), Ecuador (EC), Spain (ES), Finland (FI),
 *  France (FR), UK (GB), Greece (GR), Hong Kong SAR China (HK), Hungary (HU), Indonesia (ID), Ireland (IE), Israel (IL),
 *  India (IN), Iceland (IS), Italy (IT), Japan (JP), S. Korea (KR), Mexico (MX), Malaysia (MY), Netherlands (NL),
 *  Norway (NO), New Zealand (NZ), Poland (PL), Portugal (PT), Russian Federation (RU), Sweden (SE), Singapore (SG),
 *  Thailand (TH), Turkey (TK), Taiwan (TW), USA (US), and South Africa (ZA).
 *  2. The coefficients in this and all releases are provided for use only with the Microsemi VoicePath API-II (VP-API-II). Please refer
 *  to the terms and conditions for licensing the software regarding terms and conditions of usage. These profiles are provided for
 *  reference only with no guarantee whatsoever by Microsemi Corporation.
 *  3. This release is for the ZLR88621L SM2 Line Module based on the ZL88601 and for the ZLR88621H Line Module set for a VBATH = -81V.
 *
 *  II. Device Profile:
 *  1. The default settings for the Device Profile are:
 *         PCLK = 8192 kHz
 *         PCM Transmit Edge = Positive
 *         Transmit Time Slot = 0
 *         Receive Time Slot = 0
 *         Interrupt Mode = Open Drain
 *         Switching Regulator Y Control Mode = Single
 *         Switching Regulator Y Voltage = 27V
 *         Switching Regulator Z Control Mode = Single
 *         Switching Regulator Z Voltage = 81V
 *         IO21 Pin Mode = Digital
 *         IO22 Pin Mode = Analog Voltage Sense
 *
 *  2. The settings may be changed by the user as necessary.  Please refer to the ZL880 and VP-API-II documentation for information
 *  about the supported settings.
 *
 *  II. DC Profiles:
 *  1. The DC_FXS_ZL880_ABS100V_DEF Profile is the default unless a country specific profile is selected. Example DC profile settings are
 *  provided for China, ETSI and the USA.
 *
 *  III. AC Profiles:
 *  1. FXS Coefficients assume -6dBr RX (Output from chipset) and 0dB TX relative gain levels.
 *  2. Supported countries not individually listed should use the default 600R profile AC_FXS_RF14_600R_DEF.
 *  4. AC FXS Coefficients assume the use of two 7 ohm series resistors or PTCs. Customers using other PTC resistance values (such as
 *  25ohms or 50 ohms) should not use these AC coefficients and can request alternate ones from Microsemi.
 *  5. This release includes both Narrowband and Wideband coefficients. Note that the ZL880 Series devices support per channel Narrowband or
 *  Wideband audio selection.
 *
 *  IV. Ring Profiles:
 *  1. RING_ZL880_ABS100V_DEF is the default ringing profile and should be used for all countries which do not have a listed ringing profile.
 *  The default ringing profile is set for a sine wave ringing with an amplitude of 50Vrms (70.7Vpk)with no DC bias and a frequency of 25 Hz
 *  generated by fixed (non-tracking) supply.
 *  2. Most ringing profiles on the list are sinusoidal with an amplitude of 50Vrms with no DC bias generated by fixed (non-tracking) supply.
 *  3. The ringing definitions may be changed based on the requirements of the target market as long as the total amplitude (AC + DC
 *  components) does not exceed the limits set forth in the ZL88601/602 data sheet.
 *
 *  V. Tone Profiles:
 *  1. These profiles are available only in the full version of the VP-API-II.
 *
 *  VI. Cadence Profiles:
 *  1. These profiles are available only in the full version of the VP-API-II.
 *
 *  VII. Caller ID Profiles:
 *  1. These profiles are available only in the full version of the VP-API-II.
 *
 *  VIII. Metering Profiles:
 *  1. These profiles are available only in the full version of the VP-API-II.
 *
 *  (c) Copyright 2012 Microsemi Corporation. All rights reserved.
 *
 *  -----------------------------------------------------------------------------------------------------------------------------------------------------
 */

#ifndef MULTILINE4CH_8CH_PW2_4_H
#define MULTILINE4CH_8CH_PW2_4_H

#ifdef VP_API_TYPES_H
#include "vp_api_types.h"
#else
typedef unsigned char VpProfileDataType;
#endif


/************** Device_Parameters **************/
/* Device Configuration Data - ZL88601 100V ABS (-90V/-30V) Master and Slave */
extern const VpProfileDataType ZLR88621L_ABS90V_OC_DEVICE[];

/************** DC_Parameters **************/
extern const VpProfileDataType DC_FXS_ZL880_ABS100V_DEF[];/* DC FXS Defaults - Use for all countries unless country file exists - 23mA Current Feed */

/************** AC_Coefficients **************/
extern const VpProfileDataType AC_FXS_RF14_600R_DEF[];/* AC FXS RF14 600R Normal Coefficients (Default)  */

/************** Ring_Parameters **************/
extern const VpProfileDataType RING_ZL880_ABS100V_DEF[];/* Default Ringing 25Hz 50Vrms Fixed, AC Trip - Use for all countries unless country profile exists */

/************** Call_Progress_Tones **************/

/************** Cadence_Definitions **************/

/************** Caller_ID **************/

/************** Metering_Profile **************/

#endif /* MULTILINE4CH_8CH_PW2_4_H */
