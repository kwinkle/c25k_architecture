import QtQuick 2.0
import QtQuick.Controls 2.2
import Ros 1.0

Rectangle {

    id: actionButton

    RosStringPublisher{
        id: sessionmood
        topic: "session_mood"
    }

    width: 200
    height: 200

    color: "transparent"

    property string mood
    property string image_basic
    property string image_select

    Image {
        id: image_basic
        fillMode: Image.PreserveAspectFit
        anchors.fill: parent
        source: parent.image_basic
        opacity: 1.0
    }

    Image {
        id: image_select
        fillMode: Image.PreserveAspectFit
        anchors.fill: parent
        source: parent.image_select
        opacity: 0
    }

    NumberAnimation {
        id: actionSelection
        target: image_select
        properties: "opacity"
        from: 1
        to: 0
        duration: 1500
        onStopped: {
            globalStates.state = "basicscreen"
        }
    }

    MouseArea {
        anchors.fill: parent
        onClicked: {
            image_select.opacity = 1
            actionSelection.start()
            sessionmood.text = mood
        }

    }

}
