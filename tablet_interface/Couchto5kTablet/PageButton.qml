import QtQuick 2.0

Rectangle {

    id: pageButton

    width: 341.5
    height: 75

    border.color: "black"
    border.width: 2

    property string page

    MouseArea {
        anchors.fill: parent
        onClicked: {
            globalstates.state = page
        }

    }

    Text {
        text: page
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
        font.pixelSize: 30
    }

}
