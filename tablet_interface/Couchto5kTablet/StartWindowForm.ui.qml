import QtQuick 2.4
import QtQuick.Controls 1.0

Item {
    width: 1366
    height: 768

    Text {
        id: text1
        x: 390
        y: 355
        width: 586
        height: 59
        text: qsTr("Waiting for user to start & complete warm up")
        horizontalAlignment: Text.AlignHCenter
        font.pixelSize: 25
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

//    Text {
//        id: speedTextLabel
//        x: 1050
//        y: 180
//        width: 600
//        height: 50
//        text: qsTr("Current speed:")
//        font.pointSize: 15
//    }

//    Rectangle {
//        width: 600
//        height: 50
//        anchors.left: speedTextLabel.left
//        anchors.leftMargin: 100
//        anchors.top: speedTextLabel.bottom
//        Text {
//            text: speed.text
//            font.pointSize: 15
//        }
//    }

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

}
