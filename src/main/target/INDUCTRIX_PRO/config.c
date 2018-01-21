/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#ifdef TARGET_CONFIG

#include "common/axis.h"
#include "common/utils.h"

#include "common/maths.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/controlrate_profile.h"

#include "fc/rc_modes.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
//#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"

#include "hardware_revision.h"

void targetConfiguration(void)
{
    // alternative defaults settings for Beebrain target
    motorConfigMutable()->dev.motorPwmRate = 4000;
    failsafeConfigMutable()->failsafe_delay = 2;
    failsafeConfigMutable()->failsafe_off_delay = 0;

    motorConfigMutable()->minthrottle = 1049;

    gyroConfigMutable()->gyro_lpf = GYRO_LPF_188HZ;
    gyroConfigMutable()->gyro_soft_lpf_hz = 90;
    gyroConfigMutable()->gyro_soft_notch_hz_1 = 0;
    gyroConfigMutable()->gyro_soft_notch_hz_2 = 0;

    rcControlsConfigMutable()->yaw_deadband = 30;
    rcControlsConfigMutable()->deadband = 20;

    batteryConfigMutable()->vbatmaxcellvoltage = (uint8_t) 42;
    batteryConfigMutable()->vbatmincellvoltage = (uint8_t) 33;
    batteryConfigMutable()->vbatwarningcellvoltage = (uint8_t) 36;

    /*for (int channel = 0; channel < NON_AUX_CHANNEL_COUNT; channel++) {
        rxChannelRangeConfigsMutable(channel)->min = 1180;
        rxChannelRangeConfigsMutable(channel)->max = 1860;
    }*/

    for (uint8_t pidProfileIndex = 0; pidProfileIndex < MAX_PROFILE_COUNT; pidProfileIndex++) {
        pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);

        pidProfile->pid[PID_ROLL].P = 70;
        pidProfile->pid[PID_ROLL].I = 70;
        pidProfile->pid[PID_ROLL].D = 24;
        pidProfile->pid[PID_PITCH].P = 75;
        pidProfile->pid[PID_PITCH].I = 80;
        pidProfile->pid[PID_PITCH].D = 25;
        pidProfile->pid[PID_YAW].P = 190;
        pidProfile->pid[PID_YAW].I = 120;
        pidProfile->pid[PID_YAW].D = 20;
        pidProfile->pid[PID_LEVEL].P = 100;
        pidProfile->pid[PID_LEVEL].I = 50;
        pidProfile->pid[PID_LEVEL].D = 100;

        pidProfile->dtermSetpointWeight = 254;
        pidProfile->setpointRelaxRatio = 100;
    }

    for (uint8_t rateProfileIndex = 0; rateProfileIndex < CONTROL_RATE_PROFILE_COUNT; rateProfileIndex++) {
        controlRateConfig_t *controlRateConfig = controlRateProfilesMutable(rateProfileIndex);

        controlRateConfig->rcRate8 = 100;
        controlRateConfig->rcYawRate8 = 100;
        controlRateConfig->rcExpo8 = 0;
        controlRateConfig->rates[FD_ROLL] = 85;
        controlRateConfig->rates[FD_PITCH] = 85;
        controlRateConfig->rates[FD_YAW] = 85;
        controlRateConfig->thrMid8 = 55;
        controlRateConfig->thrExpo8 = 70;
    }

    modeActivationCondition_t *modeActivationConditionConfig = modeActivationConditionsMutable(0);
    modeActivationConditionConfig->modeId = BOXARM;
    modeActivationConditionConfig->auxChannelIndex = 1;
    modeActivationConditionConfig->range.startStep = CHANNEL_VALUE_TO_STEP(1700);
    modeActivationConditionConfig->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    modeActivationConditionConfig = modeActivationConditionsMutable(1);
    modeActivationConditionConfig->modeId = BOXANGLE;
    modeActivationConditionConfig->auxChannelIndex = 0;
    modeActivationConditionConfig->range.startStep = CHANNEL_VALUE_TO_STEP(1700);
    modeActivationConditionConfig->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    modeActivationConditionConfig = modeActivationConditionsMutable(2);
    modeActivationConditionConfig->modeId = BOXAIRMODE;
    modeActivationConditionConfig->auxChannelIndex = 0;
    modeActivationConditionConfig->range.startStep = CHANNEL_VALUE_TO_STEP(1300);
    modeActivationConditionConfig->range.endStep = CHANNEL_VALUE_TO_STEP(1700);

    rxFailsafeChannelConfig_t *channelFailsafeConfig = rxFailsafeChannelConfigsMutable(5);
    channelFailsafeConfig->mode = RX_FAILSAFE_MODE_SET;
    channelFailsafeConfig->step = CHANNEL_VALUE_TO_RXFAIL_STEP(1000);

    parseRcChannels("TAER1234", rxConfigMutable());
}

void targetValidateConfiguration(void)
{
    if (hardwareRevision < NAZE32_REV5 && accelerometerConfig()->acc_hardware == ACC_ADXL345) {
        accelerometerConfigMutable()->acc_hardware = ACC_NONE;
    }
}
#endif
