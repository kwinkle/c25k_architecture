import QtQuick 2.3
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtGraphicalEffects 1.0

ProgressBar{
    id: timeBar

    anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
        width: parent.width*1.3
        height: parent.width*1.3
        property string blinkColor: "green"
        property alias animation: blinkAnimation
        property double ratio: 1
        //property int timer_time: 2000
        value: 1.0 - ratio
        //value: 2000 - timer_time
        x:0
        y:0
        z:-1

        style: ProgressBarStyle {
              panel : Rectangle {
                color: "transparent"
                implicitWidth: width
                implicitHeight: width
                Rectangle{
                    id: innerRing
                    z: 1
                    anchors.fill: parent
                    radius: Math.max(width, height) / 2
                    color: "transparent"
                    border.color: "#80808080"
                    border.width: 8

                    ConicalGradient{
                        source: innerRing
                        anchors.fill: parent
                        gradient: Gradient{
                            GradientStop { position: 0.00; color: "#FFDC143C" }
                            GradientStop { position: control.value; color: "#FFDC143C" }
                            GradientStop { position: control.value + 0.009; color: "#FF00FF00" }
                            GradientStop { position: 1.00; color: "#FF00FF00" }
                        }
                    }
                }
            }/*
            background: Rectangle {
                radius: 2
                color: "Crimson"
                border.color: "black"
                border.width: 1
                implicitWidth: 200
                implicitHeight: 24
            }
            progress: Rectangle {
                color: "lime"
                border.color: "black"
                implicitWidth: 200
                implicitHeight: 24
            }*/
        }
        Rectangle{
            id: blinkFrame
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
            width:parent.width
            height:parent.width
            radius:parent.width / 2
            color: blinkColor
            opacity: 0
        }
        SequentialAnimation{
            id:blinkAnimation
            NumberAnimation {target: blinkFrame; property: "opacity"; from: 0; to: .5; duration: 200}
            NumberAnimation {target: blinkFrame; property: "opacity"; from: .5; to: 0; duration: 200}
            NumberAnimation {target: blinkFrame; property: "opacity"; from: 0; to: .5; duration: 200}
            NumberAnimation {target: blinkFrame; property: "opacity"; from: .5; to: 0; duration: 200}
        }
//        onRatioChanged: {
//            if(ratio === 0) value = 0.991
//            else value = 1.0 - ratio
//        }

//        onTimer_timeChanged: {
//            if(timer_time === 0) value = 1999
//            else value = 2000 - timer_time
//        }

}
