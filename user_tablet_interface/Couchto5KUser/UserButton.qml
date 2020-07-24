import QtQuick 2.0
import QtQuick.Controls 2.2
import Ros 1.0

Rectangle {

    id: userButton

    RosStringPublisher {
        id: userStart
        topic: "user_start"
    }

    width: 200
    height: 200

    color: "transparent"

    property string image_basic
    property string user_id

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
            userStart.text = user_id
            globalStates.state = "basicscreen"
        }

    }

}
