import QtQuick 2.4
import Ros 1.0

EmergencyWindowForm {

    rotation: 180

    RosStringPublisher {
        id: reset
        topic: "reset"
    }

    resetButton.onClicked: {
        globalStates.state = "start"
        reset.text = "RESET"
    }
}
