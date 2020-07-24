import QtQuick 2.0
import QtQuick.Controls 2.2
import Ros 1.0

Rectangle {

    id: stopButton

    RosStringPublisher{
        id: stop
        topic: "emergency_stop"
    }

    width: 200
    height: 200

    color: "transparent"

    property string image_basic

    Image {
        id: image_basic
        fillMode: Image.PreserveAspectFit
        anchors.fill: parent
        source: parent.image_basic
        opacity: 1.0
    }


    MouseArea {
        anchors.fill: parent
        onClicked: {
            stop.text = "STOP"
            globalStates.state = "emergencystop"
        }

    }

}
