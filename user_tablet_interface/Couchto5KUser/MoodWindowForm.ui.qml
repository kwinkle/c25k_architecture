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

        ActionButton {
            id: negative
            mood: "NEGATIVE"
            image_basic: "res/notgreat.png"
            image_select: "res/notgreat_select.png"
            anchors.left: neutral.right
            anchors.leftMargin: 200
            y: 300
        }

        ActionButton {
            id: neutral
            mood: "NEUTRAL"
            anchors.horizontalCenter: parent
            image_basic: "res/ok.png"
            image_select: "res/ok_select.png"
            y: 300
            x: 583
        }

        ActionButton {
            id: positive
            mood: "POSITIVE"
            image_basic: "res/great.png"
            image_select: "res/great_select.png"
            anchors.right: neutral.left
            anchors.rightMargin: 200
            y: 300
        }

    }
}


/*##^## Designer {
    D{i:0;autoSize:true;height:480;width:640}
}
 ##^##*/
