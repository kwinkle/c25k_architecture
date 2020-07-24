import QtQuick 2.9
import QtQuick.Controls 2.2

Page {
    id: sympathetic
    width: 1366
    height: 768

    property alias time: time
    property alias checkpre: checkpre
    property alias praise: praise
    property alias sympathise: sympathise
    //property alias moreeyes: moreeyes
    property alias challenge: challenge
    property alias speeddown: speeddown

    header: Label {
        id: title
        text: qsTr("Sympathetic")
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
    //        text: qsTr("Sympathetic")
    //        anchors.centerIn: parent
    //        font.pointSize: 20
    //    }
    ActionButton {
        action: "TIME"
        style: "SYMPATHETIC"
        id: time
        countdown: 2
        image_basic: "res/time.png"
        image_highlight: "res/timeHighlight.png"
        image_select: "res/timeSelect.png"
        page: "sympathetic"
        x: 65
        y: 25
    }

    ActionButton {
        action: "CHECKPRE"
        style: "SYMPATHETIC"
        id: checkpre
        countdown: 2
        image_basic: "res/checkpre.png"
        image_highlight: "res/checkpreHighlight.png"
        image_select: "res/checkpreSelect.png"
        page: "sympathetic"
        x: 765
        y: 290
    }

    ActionButton {
        action: "PRAISE"
        style: "SYMPATHETIC"
        id: praise
        countdown: 2
        image_basic: "res/praise.png"
        image_highlight: "res/praiseHighlight.png"
        image_select: "res/praiseSelect.png"
        page: "sympathetic"
        x: 765
        y: 25
    }

    ActionButton {
        action: "SYMPATHISE"
        style: "SYMPATHETIC"
        id: sympathise
        countdown: 2
        image_basic: "res/sympathise.png"
        image_highlight: "res/sympathiseHighlight.png"
        image_select: "res/sympathiseSelect.png"
        page: "sympathetic"
        x: 65
        y: 290
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
    ActionButton {
        action: "SPEEDDOWN"
        style: "SYMPATHETIC"
        id: speeddown
        countdown: 2
        image_basic: "res/speeddown.png"
        image_highlight: "res/speeddownHighlight.png"
        image_select: "res/speeddownSelect.png"
        page: "sympathetic"
        x: 415
        y: 290
    }

    ActionButton {
        action: "CHALLENGE"
        style: "SYMPATHETIC"
        id: challenge
        countdown: 2
        image_basic: "res/challenge.png"
        image_highlight: "res/challengeHighlight.png"
        image_select: "res/challengeSelect.png"
        page: "sympathetic"
        x: 415
        y: 25
    }

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
