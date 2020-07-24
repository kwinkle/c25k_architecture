import QtQuick 2.4
import QtQuick.Controls 2.2

Item {

    width: 1366
    height: 768


    //property alias startButton: startButton
    Text {
        id: text1
        x: 567
        y: 50
        text: qsTr("Please Select User")
        horizontalAlignment: Text.AlignHCenter
        font.pixelSize: 25
    }

    UserButton {
        user_id: "DP"
        image_basic: "res/DP.png"
        x: 80
        y: 150
    }

    UserButton {
        user_id: "FB"
        image_basic: "res/FB.png"
        x: 330
        y: 150
    }

    UserButton {
        user_id: "GB"
        image_basic: "res/GB.png"
        x: 580
        y: 150
    }

    UserButton {
        user_id: "JF"
        image_basic: "res/JF.png"
        x: 830
        y: 150
    }

    UserButton {
        user_id: "JW"
        image_basic: "res/JW.png"
        x: 1080
        y: 150
    }

    UserButton {
        user_id: "LB"
        image_basic: "res/LB.png"
        x: 80
        y: 450
    }

    UserButton {
        user_id: "DB"
        image_basic: "res/DB.png"
        x: 330
        y: 450
    }

    UserButton {
        user_id: "MB"
        image_basic: "res/MB.png"
        x: 580
        y: 450
    }

    UserButton {
        user_id: "MR"
        image_basic: "res/MR.png"
        x: 830
        y: 450
    }

    UserButton {
        user_id: "PT"
        image_basic: "res/PT.png"
        x: 1080
        y: 450
    }

    //    Button {
    //        id: startButton
    //        x: 633
    //        y: 237
    //        text: qsTr("Start")
    //    }
}
