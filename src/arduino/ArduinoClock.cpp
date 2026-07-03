#ifdef ARDUINO

#include "ArduinoClock.h"

namespace roboarm {

ArduinoClock& sharedArduinoClock() {
    static ArduinoClock instance;
    return instance;
}

}  // namespace roboarm

#endif  // ARDUINO
