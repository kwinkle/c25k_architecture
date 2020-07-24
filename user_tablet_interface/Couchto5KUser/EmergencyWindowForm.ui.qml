import QtQuick 2.4
import QtQuick.Controls 2.2

Item {
    width: 1366
    height: 768

    property alias resetButton: resetButton

    Text {
        id: text1
        x: 589
        y: 373
        text: qsTr("Programme Stopped")
        font.pixelSize: 20
    }

    Button {
        id: resetButton
        x: 633
        y: 444
        text: qsTr("RESET")
    }
}
