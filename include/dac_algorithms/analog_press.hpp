#ifndef DAC_ALGORITHMS__ANALOG_PRESS_HPP
#define DAC_ALGORITHMS__ANALOG_PRESS_HPP

#include "communication_protocols/joybus/gcReport.hpp"

#include "gpio_to_button_sets/F1.hpp"

namespace DACAlgorithms {
namespace AnalogPress {


GCReport getGCReport(GpioToButtonSets::F1::ButtonSet buttonSet);

}
}

#endif