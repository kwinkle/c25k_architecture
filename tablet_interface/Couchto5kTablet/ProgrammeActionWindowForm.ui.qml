import QtQuick 2.9
import QtQuick.Controls 2.2

Page {
    width: 1366
    height: 768

    property alias nextAction: nextAction
    //property alias nextActionStyle: nextActionStyle
    property alias neutral: neutral
    property alias positive: positive
    property alias challenging: challenging
    property alias sympathetic: sympathetic


    //    Text {
    //        x: 246
    //        y: 23
    //        width: 589
    //        height: 52
    //        text: qsTr("Next programme action is:")
    //        font.pointSize: 19
    //    }
    header: Label {
        id: title
        text: qsTr("Mandatory Programme Action")
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

    Rectangle {
        id: time
        x: 100
        y: 50
        width: 100
        height: 76
        Text {
            width: 100
            height: 76
            text: programmeActionTime.text
            font.pointSize: 20
        }
    }

    Text {
        x: 250
        y: 50
        width: 600
        height: 50
        text: qsTr("Next programme action is:")
        font.pointSize: 20
    }

    Rectangle {
        id: nextAction
        x: 280
        y: 100
        width: 600
        height: 50
        Text {
            text: suggestedAction.action
            font.pointSize: 20
        }
    }

//    Text {
//        x: 250
//        y: 150
//        width: 600
//        height: 50
//        text: qsTr("With style:")
//        font.pointSize: 20
//    }

//    Rectangle {
//        x: 280
//        y: 200
//        width: 600
//        height: 50
//        Text {
//            id: nextActionStyle
//            text: suggestedAction.style
//            font.pointSize: 20
//        }
//    }

    StyleButton {
        style: "NEUTRAL"
        id: neutral
        countdown: 2 //TO-DO: on ALL style buttons - set countdown to match timing requirements for issuing instruction
        image_basic: "res/neutral.png"
        image_highlight: "res/neutralHighlight.png"
        image_select: "res/neutralSelect.png"
        visible: true
        x: 60
        y: 350
    }

    StyleButton {
        style: "POSITIVE"
        id: positive
        countdown: 2
        image_basic: "res/positive.png"
        image_highlight: "res/positiveHighlight.png"
        image_select: "res/positiveSelect.png"
        visible: true
        x: 310
        y: 350
    }

    StyleButton {
        style: "CHALLENGING"
        id: challenging
        countdown: 2
        image_basic: "res/challenging.png"
        image_highlight: "res/challengingHighlight.png"
        image_select: "res/challengingSelect.png"
        visible: true
        x: 560
        y: 350
    }

    StyleButton {
        style: "SYMPATHETIC"
        id: sympathetic
        countdown: 2
        image_basic: "res/sympathetic.png"
        image_highlight: "res/sympatheticHighlight.png"
        image_select: "res/sympatheticSelect.png"
        visible: true
        x: 810
        y: 350
    }

//    Text {
//        id: stateTextLabel
//        x: 1050
//        y: 0
//        width: 600
//        height: 50
//        text: qsTr("User should be on a:")
//        font.pointSize: 15
//    }

//    Rectangle {
//        width: 600
//        height: 50
//        anchors.left: stateTextLabel.left
//        anchors.leftMargin: 100
//        anchors.top: stateTextLabel.bottom
//        Text {
//            text: programmeState.text
//            font.pointSize: 15
//        }
//    }

//    Text {
//        id: actionTimeTextLabel
//        x: 1050
//        y: 80
//        width: 600
//        height: 50
//        text: qsTr("Time left on walk/run:")
//        font.pointSize: 15
//    }

//    Rectangle {
//        width: 600
//        height: 50
//        anchors.left: actionTimeTextLabel.left
//        anchors.leftMargin: 100
//        anchors.top: actionTimeTextLabel.bottom
//        Text {
//            text: stateDuration.text
//            font.pointSize: 15
//        }
//    }

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

//    Text {
//        id: hrTextLabel
//        x: 1050
//        y: 280
//        width: 600
//        height: 50
//        text: qsTr("Current heart rate:")
//        font.pointSize: 15
//    }

//    Rectangle {
//        width: 600
//        height: 50
//        anchors.left: hrTextLabel.left
//        anchors.leftMargin: 100
//        anchors.top: hrTextLabel.bottom
//        Text {
//            text: hr.text
//            font.pointSize: 15
//        }
//    }

//    Text {
//        id: sessionTimeLabel
//        x: 1050
//        y: 380
//        width: 600
//        height: 50
//        text: qsTr("Time left on session:")
//        font.pointSize: 15
//    }

//    Rectangle {
//        width: 600
//        height: 50
//        anchors.left: sessionTimeLabel.left
//        anchors.leftMargin: 100
//        anchors.top: sessionTimeLabel.bottom
//        Text {
//            text: sessionDuration.text
//            font.pointSize: 15
//        }
//    }

//    Text {
//        id: latestMoodLabel
//        x: 1050
//        y: 480
//        width: 600
//        height: 50
//        text: qsTr("Latest Mood/PRE: ")
//        font.pointSize: 15
//    }

//    Rectangle {
//        width: 600
//        height: 50
//        anchors.left: latestMoodLabel.left
//        anchors.leftMargin: 100
//        anchors.top: latestMoodLabel.bottom
//        Text {
//            text: mood.text
//            font.pointSize: 15
//        }
//    }
}
