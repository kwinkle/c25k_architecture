import QtQuick 2.9
import QtQuick.Window 2.2
import QtQuick.Controls 2.2
import Ros 1.0

ApplicationWindow {
    id: mainwindow
    visible: true
    width: 1366
    height: 768
    title: qsTr("Couch to 5km User Interface")

    RosStringSubscriber{
        id: checkMoodSubscriber
        topic: "check_mood"
        onTextChanged: {
            globalStates.state = "checkmood"
        }
    }

    RosStringSubscriber{
        id: subtitlesSubscriber
        topic: "subtitles"
    }

    RosStringSubscriber{
        id: sessionEndSubscriber
        topic: "session_complete"
        onTextChanged: {
            globalStates.state="start"
            //BasicScreen.finished_signal = sessionEndSubscriber
        }
    }


    StateGroup {
        id: globalStates

        states: [

            State {
                name: "start"
                PropertyChanges{
                    target: startWindow
                    visible: true
                }
                PropertyChanges{
                    target: moodWindow
                    visible: false
                }
                PropertyChanges {
                    target: basicWindow
                    visible: false
                }
                PropertyChanges {
                    target: emergencyWindow
                    visible: false
                }
            },

            State {
                name: "checkmood"
                PropertyChanges{
                    target: startWindow
                    visible: false
                }
                PropertyChanges{
                    target: moodWindow
                    visible: true
                }
                PropertyChanges {
                    target: basicWindow
                    visible: false
                }
                PropertyChanges {
                    target: emergencyWindow
                    visible: false
                }
            },

            State{
                name: "basicscreen"
                PropertyChanges {
                    target: startWindow
                    visible: false
                }
                PropertyChanges {
                    target: moodWindow
                    visible: false
                }
                PropertyChanges {
                    target: basicWindow
                    visible: true
                }
                PropertyChanges {
                    target: emergencyWindow
                    visible: false
                }
            },

            State{
                name: "emergencystop"
                PropertyChanges {
                    target: startWindow
                    visible: false
                }
                PropertyChanges {
                    target: moodWindow
                    visible: false
                }
                PropertyChanges {
                    target: basicWindow
                    visible: false
                }
                PropertyChanges {
                    target: emergencyWindow
                    visible: true
                }
            },

            State {
                name: "end"
            }

        ]
    }

    Item {
        id: startWindow
        visible: true

        StartWindow{}
    }

    Item {
        id: moodWindow
        visible: false

        MoodWindow{}
    }

    Item {
        id: basicWindow
        visible: false
        BasicScreen{}
    }

    Item {
        id: emergencyWindow
        visible: false
        EmergencyWindow{}
    }

}
