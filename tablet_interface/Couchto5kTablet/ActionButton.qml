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

    width: 220
    height: 220

    color: "transparent"

    id: actionButton

    property string action
    property string style
    property string image
    property string image_basic
    property string image_highlight
    property string image_select
    property string page

    property int countdown

    property alias actionHighlight: actionHighlight
    property alias timeBar: timeBar
    property alias timeBarAnimation: timeBarAnimation
    property alias image_cancel: image_cancel

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

//        anchors.left: actionButton.left
//        anchors.leftMargin: 75
        anchors.horizontalCenter: image_highlight.horizontalCenter
        anchors.top: actionButton.bottom
        anchors.topMargin: 2

        Image {
            id: image_cancel
            fillMode: Image.PreserveAspectFit
            anchors.fill: parent
            source: "res/cancel.png"
            visible: false

            MouseArea{
                anchors.fill: parent
                onClicked: {
                    console.log("cancel this action")
                    image_highlight.opacity = 0 //turn off action highlight
                    timeBarAnimation.stop()
                    timeBar.enabled = false
                    timeBar.visible = false
                    timeBar.ratio = 0
                    image_cancel.visible = false
                    console.log("publishing refused action - for learning only")
                    actionPublisher.action = action
                    actionPublisher.style = style
                    actionPublisher.duration = 0
                    actionPublisher.validation = "REFUSED"
                    actionPublisher.publish()
                    globalstates.state = page
                }
            }
        }


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
        id: actionSelection
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
            onStopped: {
                if (timeBar.ratio == 0) {
                image_highlight.opacity = 0;
                timeBar.enabled = false;
                timeBar.visible = false;
                image_cancel.visible = false
                actionPublisher.action = action;
                actionPublisher.style = style;
                actionPublisher.duration = 0;
                actionPublisher.validation = "PASSIVE_ACCEPTED"
                console.log("publishing passive accepted engagement action")
                actionPublisher.publish()
                globalstates.state = page
                }
            }
        }


    }

    MouseArea {
        anchors.fill: parent
        onClicked: {
            actionSelection.start()
            actionPublisher.action = action
            actionPublisher.style = style
            actionPublisher.duration = 0

            if (image_highlight.opacity == 0) {
                actionPublisher.validation = "UNPROMPTED"
                console.log("publishing unprompted action")
                actionPublisher.publish()
            } else {
                actionPublisher.validation = "VALIDATED" //chosen based on suggestion
                console.log("publishing validated suggested action")
                actionPublisher.publish()
                timeBarAnimation.stop()
                timeBar.enabled = false
                timeBar.visible = false
                timeBar.ratio = 0
                image_cancel.visible = false
                globalstates.state = page
            }

            image_highlight.opacity = 0
            image_select.opacity = 1
        }

    }

}
