import QtQuick 2.4
import QtQuick.Controls 2.2

Item {
    Rectangle{
        width: 1366
        height: 768

        color: "black"

        Text {
            id: text1
            x: 42
            y: 42
            width: 1276
            height: 294
            text: subtitlesSubscriber.text
            wrapMode: Text.WrapAtWordBoundaryOrAnywhere
            horizontalAlignment: Text.AlignHCenter
            font.pixelSize: 40
            color: "white"
        }

        //    StopButton {
        //        id: stop
        //        anchors.horizontalCenter: parent
        //        image_basic: "res/stop.png"
        //        y: 500
        //        x: 583
        //    }
    }
}
