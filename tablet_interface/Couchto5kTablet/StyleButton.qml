import QtQuick 2.0
import QtQuick.Controls 2.2
import Ros 1.0

Rectangle {

    RosValidatedC25KPublisher{
        trigger_state_id: suggestedAction.trigger_state_id
        trigger_timestamp: suggestedAction.trigger_timestamp
        action_id: suggestedAction.action_id
        id: actionPublisher
        topic: "validated_actions"
    }

    width: 200
    height: 200

    color: "transparent"

    id: styleButton

    property string action
    property string style
    property string image
    property string image_basic
    property string image_highlight
    property string image_select

    property int countdown

    property alias actionHighlight: actionHighlight
    property alias actionUnHighlight: actionUnHighlight
    property alias actionUnSelection: actionUnSelection
    property alias timeBar: timeBar
    property alias timeBarAnimation: timeBarAnimation

    Image {
        id: image_basic
        fillMode: Image.PreserveAspectFit
        anchors.fill: parent
        source: parent.image_basic
        opacity: 1.0
    }

    Image {
        id: image_highlight
        fillMode: Image.PreserveAspectFit
        anchors.fill: parent
        source: parent.image_highlight
        opacity: 0
    }

    Image {
        id: image_select
        fillMode: Image.PreserveAspectFit
        anchors.fill: parent
        source: parent.image_select
        opacity: 0
    }

    Rectangle{
        width: 50
        height: 50
        color: "transparent"

        anchors.left: styleButton.left
        anchors.leftMargin: 75
        anchors.top: styleButton.bottom
        anchors.topMargin: 2


    }

    NumberAnimation {
        id: actionHighlight
        target: image_highlight
        properties: "opacity"
        from: 0
        to: 1
        duration: 10
        //loops: Animation.Infinite
    }

    NumberAnimation {
        id: actionUnHighlight
        target: image_highlight
        properties: "opacity"
        from: 1
        to: 0
        duration: 10
        //loops: Animation.Infinite
    }

    NumberAnimation {
        id: actionUnSelection
        target: image_select
        properties: "opacity"
        from: 1
        to: 0
        duration: 1000
        onStopped: {
        }
    }


    Timebar {
        id: timeBar
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
        visible: false

        NumberAnimation {
            id: timeBarAnimation
            target: timeBar
            property: "ratio"
            from: 1
            to: 0
            duration: 10000
        }


    }

    MouseArea {
        anchors.fill: parent
        onClicked: {
            if (suggestedAction.style === style) { //clicking the suggested style
                timeBar.enabled = false //turn off animation stuff
                timeBar.visible = false
                image_highlight.opacity = 0
                image_select.opacity = 1 //highlight that this is the selected style
                programmeActionTimer.validation = "VALIDATED"
                currentState.state = style

            } else {
                currentState = style //clicking a different style
                console.log(style)
                image_select.opacity = 1 //highlight that this is the selected style
                nextActionStyle.text = style //update gui to show selected style name
                //animation of previously suggested image will be dealt with via the currentState change...
                actionPublisher.action = suggestedAction.action //publish a training message for telling learner the suggested style was refused
                actionPublisher.style = suggestedAction.style
                actionPublisher.duration = suggestedAction.duration
                actionPublisher.validation = "REFUSED"
                actionPublisher.publish()
                console.log("publishing refused style validated action")
            }

        }

    }

}
