import QtQuick 2.9
import QtQuick.Controls 2.2

Page {
    id: robotbehaviour
    width: 1366
    height: 768

    property alias funanimation: funanimation
    //    property alias smalltalk: smalltalk
    property alias getcloser: getcloser
    property alias backoff: backoff

    //    property alias moreeyes: moreeyes
    //    property alias lesseyes: lesseyes
    header: Label {
        id: title
        text: qsTr("Robot Behaviour")
        font.pixelSize: 30
        padding: 10

        StopButton {
            id: stop
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: title.top
            anchors.topMargin: 5
            image_basic: "res/stop.png"
        }
    }

    //    Label {
    //        text: qsTr("Robot Behaviour")
    //        anchors.centerIn: parent
    //        font.pointSize: 20
    //    }
    ActionButton {
        action: "ANIMATION"
        style: "POSITIVE"
        id: funanimation
        countdown: 2
        image_basic: "res/funanimation.png"
        image_highlight: "res/funanimationHighlight.png"
        image_select: "res/funanimationSelect.png"
        page: "robotbehaviour"
        x: 65
        y: 25
    }

    //    ActionButton {
    //        action: "SMALLTALK"
    //        style: "NONE" //TO-DO: STYLE for this type of action?
    //        id: smalltalk
    //        countdown: 2
    //        image_basic: "res/smalltalk.png"
    //        image_highlight: "res/smalltalkHighlight.png"
    //        image_select: "res/smalltalkSelect.png"
    //        page: "robotbehaviour"
    //        x: 85
    //        y: 400
    //    }
    ActionButton {
        action: "GET_CLOSER"
        style: "CHALLENGING"
        id: getcloser
        countdown: 2
        image_basic: "res/getcloser.png"
        image_highlight: "res/getcloserHighlight.png"
        image_select: "res/getcloserSelect.png"
        page: "robotbehaviour"
        x: 765
        y: 25
    }

    ActionButton {
        action: "BACK_OFF"
        style: "NEUTRAL"
        id: backoff
        countdown: 2
        image_basic: "res/backoff.png"
        image_highlight: "res/backoffHighlight.png"
        image_select: "res/backoffSelect.png"
        page: "robotbehaviour"
        x: 415
        y: 25
    }

    //    ActionButton {
    //        action: "MOREEYES"
    //        style: "SYMPATHETIC"
    //        id: moreeyes
    //        countdown: 2
    //        image_basic: "res/moreeyes.png"
    //        image_highlight: "res/moreeyesHighlight.png"
    //        image_select: "res/moreeyesSelect.png"
    //        x: 435
    //        y: 400
    //    }

    //    ActionButton {
    //        action: "LESSEYES"
    //        style: "NONE"
    //        id: lesseyes
    //        countdown: 2
    //        image_basic: "res/lesseyes.png"
    //        image_highlight: "res/lesseyesHighlight.png"
    //        image_select: "res/lesseyesSelect.png"
    //        x: 785
    //        y: 400
    //    }
    Text {
        id: stateTextLabel
        x: 1050
        y: 0
        width: 600
        height: 50
        text: qsTr("User should be on a:")
        font.pointSize: 15
    }

    Rectangle {
        width: 600
        height: 50
        anchors.left: stateTextLabel.left
        anchors.leftMargin: 100
        anchors.top: stateTextLabel.bottom
        Text {
            text: programmeState.text
            font.pointSize: 15
        }
    }

    Text {
        id: actionTimeTextLabel
        x: 1050
        y: 80
        width: 600
        height: 50
        text: qsTr("Time left on walk/run:")
        font.pointSize: 15
    }

    Rectangle {
        width: 600
        height: 50
        anchors.left: actionTimeTextLabel.left
        anchors.leftMargin: 100
        anchors.top: actionTimeTextLabel.bottom
        Text {
            text: stateDuration.text
            font.pointSize: 15
        }
    }

    Text {
        id: speedTextLabel
        x: 1050
        y: 180
        width: 600
        height: 50
        text: qsTr("Current speed:")
        font.pointSize: 15
    }

    Rectangle {
        width: 600
        height: 50
        anchors.left: speedTextLabel.left
        anchors.leftMargin: 100
        anchors.top: speedTextLabel.bottom
        Text {
            text: speed.text
            font.pointSize: 15
        }
    }

    Text {
        id: hrTextLabel
        x: 1050
        y: 280
        width: 600
        height: 50
        text: qsTr("Current heart rate:")
        font.pointSize: 15
    }

    Rectangle {
        width: 600
        height: 50
        anchors.left: hrTextLabel.left
        anchors.leftMargin: 100
        anchors.top: hrTextLabel.bottom
        Text {
            text: hr.text
            font.pointSize: 15
        }
    }

    Text {
        id: sessionTimeLabel
        x: 1050
        y: 380
        width: 600
        height: 50
        text: qsTr("Time left on session:")
        font.pointSize: 15
    }

    Rectangle {
        width: 600
        height: 50
        anchors.left: sessionTimeLabel.left
        anchors.leftMargin: 100
        anchors.top: sessionTimeLabel.bottom
        Text {
            text: sessionDuration.text
            font.pointSize: 15
        }
    }

    Text {
        id: latestMoodLabel
        x: 1050
        y: 480
        width: 600
        height: 50
        text: qsTr("Latest Mood/PRE: ")
        font.pointSize: 15
    }

    Rectangle {
        width: 600
        height: 50
        anchors.left: latestMoodLabel.left
        anchors.leftMargin: 100
        anchors.top: latestMoodLabel.bottom
        Text {
            text: mood.text
            font.pointSize: 15
        }
    }
}
