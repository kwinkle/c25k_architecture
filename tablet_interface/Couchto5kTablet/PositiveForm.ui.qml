import QtQuick 2.9
import QtQuick.Controls 2.2

Page {
    id: positive
    width: 1366
    height: 768

    property alias maintain: maintain
    property alias praise: praise
    property alias time: time
    property alias humour: humour

    //property alias userhistory: userhistory
    header: Label {
        id: title
        text: qsTr("Positive")
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
    //        text: qsTr("Positive")
    //        font.pointSize: 20
    //        anchors.centerIn: parent
    //    }
    ActionButton {
        action: "TIME"
        style: "POSITIVE"
        id: time
        countdown: 2
        image_basic: "res/time.png"
        image_highlight: "res/timeHighlight.png"
        image_select: "res/timeSelect.png"
        page: "positive"
        x: 65
        y: 25
    }

    ActionButton {
        action: "HUMOUR"
        style: "POSITIVE"
        id: humour
        countdown: 2
        image_basic: "res/humour.png"
        image_highlight: "res/humourHighlight.png"
        image_select: "res/humourSelect.png"
        page: "positive"
        x: 65
        y: 290
    }

    //    ActionButton {
    //        action: "USERHISTORY"
    //        style: "POSITIVE"
    //        id: userhistory
    //        countdown: 2
    //        image_basic: "res/history.png"
    //        image_highlight: "res/historyHighlight.png"
    //        image_select: "res/historySelect.png"
    //        x: 785
    //        y: 50
    //    }
    ActionButton {
        action: "MAINTAIN"
        style: "POSITIVE"
        id: maintain
        countdown: 2
        image_basic: "res/maintain.png"
        image_highlight: "res/maintainHighlight.png"
        image_select: "res/maintainSelect.png"
        page: "positive"
        x: 415
        y: 25
    }

    ActionButton {
        action: "PRAISE"
        style: "POSITIVE"
        id: praise
        countdown: 2
        image_basic: "res/praise.png"
        image_highlight: "res/praiseHighlight.png"
        image_select: "res/praiseSelect.png"
        page: "positive"
        x: 765
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
