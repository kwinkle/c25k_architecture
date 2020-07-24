import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import QtQuick.Window 2.3

import Ros 1.0


ApplicationWindow {
    id: mainwindow
    visible: true
    width: 1366
    height: 768
    title: qsTr("Couch to 5km Instructor Interface")


    StateGroup {
        id: globalstates

        states: [
            State {
                name: "start"
                PropertyChanges {
                    target:startwindow
                    visible:true
                }
                PropertyChanges {
                    target: robotPageButton
                    visible:false
                }
                PropertyChanges {
                    target: challengingPageButton
                    visible:false
                }
                PropertyChanges {
                    target: positivePageButton
                    visible:false
                }
                PropertyChanges {
                    target: sympatheticPageButton
                    visible:false
                }
            },

//            State {
//                name: "instructorengage"
//                PropertyChanges {
//                    target: startwindow
//                    visible: false
//                }
//                PropertyChanges {
//                    target: engagewindow
//                    visible: true
//                }
//                PropertyChanges {
//                    target: tabBar
//                    visible: true
//                }
//            },

            State {
                name: "programmeaction"
//                PropertyChanges {
//                    target: engagewindow
//                    visible: false
//                }
//                PropertyChanges{
//                    target: tabBar
//                    visible: false
//                }
                PropertyChanges {
                    target: startwindow
                    visible: false

                }
                PropertyChanges{
                    target: robotbehaviourwindow
                    visible: false
                }
                PropertyChanges{
                    target: challengingwindow
                    visible: false
                }
                PropertyChanges {
                    target: positivewindow
                    visible: false
                }
                PropertyChanges{
                    target: sympatheticwindow
                    visible: false
                }
                PropertyChanges {
                    target: startwindow
                    visible: false
                }
                PropertyChanges {
                    target: programmewindow
                    visible: true
                }
                PropertyChanges {
                    target: robotPageButton
                    visible:false
                }
                PropertyChanges {
                    target: challengingPageButton
                    visible:false
                }
                PropertyChanges {
                    target: positivePageButton
                    visible:false
                }
                PropertyChanges {
                    target: sympatheticPageButton
                    visible:false
                }
            },

            State {
                name: "robotbehaviour"
                PropertyChanges {
                    target: startwindow
                    visible: false
                }
                PropertyChanges{
                    target: robotbehaviourwindow
                    visible: true
                }
                PropertyChanges{
                    target: challengingwindow
                    visible: false
                }
                PropertyChanges {
                    target: positivewindow
                    visible: false
                }
                PropertyChanges{
                    target: sympatheticwindow
                    visible: false
                }
                PropertyChanges {
                    target: robotPageButton
                    visible:true
                }
                PropertyChanges {
                    target: challengingPageButton
                    visible:true
                }
                PropertyChanges {
                    target: positivePageButton
                    visible:true
                }
                PropertyChanges {
                    target: sympatheticPageButton
                    visible:true
                }
//                PropertyChanges {
//                    target: engagewindow
//                    visible: true
//                }
//                PropertyChanges {
//                    target: swipeView
//                    currentIndex: 0
//                }
            },

            State {
                name: "positive"
                PropertyChanges {
                    target: startwindow
                    visible: false
                }
                PropertyChanges{
                    target: robotbehaviourwindow
                    visible: false
                }
                PropertyChanges{
                    target: challengingwindow
                    visible: false
                }
                PropertyChanges {
                    target: positivewindow
                    visible: true
                }
                PropertyChanges{
                    target: sympatheticwindow
                    visible: false
                }
                PropertyChanges {
                    target: robotPageButton
                    visible:true
                }
                PropertyChanges {
                    target: challengingPageButton
                    visible:true
                }
                PropertyChanges {
                    target: positivePageButton
                    visible:true
                }
                PropertyChanges {
                    target: sympatheticPageButton
                    visible:true
                }
//                PropertyChanges {
//                    target: engagewindow
//                    visible: true
//                }
//                PropertyChanges {
//                    target: swipeView
//                    currentIndex: 1
//                }
            },

            State {
                name: "challenging"
                PropertyChanges {
                    target: startwindow
                    visible: false
                }
//                PropertyChanges {
//                    target: engagewindow
//                    visible: true
//                }
//                PropertyChanges {
//                    target: swipeView
//                    currentIndex: 2
//                }
                PropertyChanges{
                    target: robotbehaviourwindow
                    visible: false
                }
                PropertyChanges{
                    target: challengingwindow
                    visible: true
                }
                PropertyChanges {
                    target: positivewindow
                    visible: false
                }
                PropertyChanges{
                    target: sympatheticwindow
                    visible: false
                }
                PropertyChanges {
                    target: robotPageButton
                    visible:true
                }
                PropertyChanges {
                    target: challengingPageButton
                    visible:true
                }
                PropertyChanges {
                    target: positivePageButton
                    visible:true
                }
                PropertyChanges {
                    target: sympatheticPageButton
                    visible:true
                }

            },

            State {
                name: "sympathetic"
                PropertyChanges {
                    target: startwindow
                    visible: false
                }
                PropertyChanges{
                    target: robotbehaviourwindow
                    visible: false
                }
                PropertyChanges{
                    target: challengingwindow
                    visible: false
                }
                PropertyChanges {
                    target: positivewindow
                    visible: false
                }
                PropertyChanges{
                    target: sympatheticwindow
                    visible: true
                }
                PropertyChanges {
                    target: robotPageButton
                    visible:true
                }
                PropertyChanges {
                    target: challengingPageButton
                    visible:true
                }
                PropertyChanges {
                    target: positivePageButton
                    visible:true
                }
                PropertyChanges {
                    target: sympatheticPageButton
                    visible:true
                }
//                PropertyChanges {
//                    target: engagewindow
//                    visible: true
//                }
//                PropertyChanges {
//                    target: swipeView
//                    currentIndex: 3
//                }
            },


            State {
                name: "end"
                PropertyChanges {
                    target: robotPageButton
                    visible:false
                }
                PropertyChanges {
                    target: challengingPageButton
                    visible:false
                }
                PropertyChanges {
                    target: positivePageButton
                    visible:false
                }
                PropertyChanges {
                    target: sympatheticPageButton
                    visible:false
                }
            }


        ]
    }

    Item {
        id: startwindow
        visible: true

        StartWindow {
            id: startTab

        }
    }

    Item {
        id: programmewindow
        visible: false

        ProgrammeActionWindow {
            id: programmeTab

        }
    }

    Item {
        id: robotbehaviourwindow
        visible: false

        RobotBehaviour{
            id: robotTab
        }
    }

    Item {
        id: challengingwindow
        visible: false

        Challenging{
            id: challengeTab
        }
    }

    Item {
        id: positivewindow
        visible: false

        Positive{
            id: positiveTab
        }
    }

    Item {
        id: sympatheticwindow
        visible: false

        Sympathetic{
            id: sympatheticTab
        }
    }

    PageButton{
        id: robotPageButton
        page: "robotbehaviour"
        x:0
        y:618
        visible: false
    }

    PageButton{
        id: challengingPageButton
        page: "challenging"
        x:341.5
        y:618
        visible: false
    }

    PageButton{
        id: positivePageButton
        page: "positive"
        x:683
        y:618
        visible: false
    }

    PageButton{
        id: sympatheticPageButton
        page: "sympathetic"
        x:1024.5
        y:618
        visible: false
    }

//    Item {
//        id: engagewindow
//        visible: false

//        SwipeView { //TO-FIX: swipe doesn't work
//            id: swipeView
//            anchors.fill: parent
//            currentIndex: tabBar.currentIndex
//            RobotBehaviour {
//                id:robotbehaviourTab
//            }

//            Positive {
//                id: positiveTab
//            }

//            Challenging {
//                id: challengingTab
//            }

//            Sympathetic {
//                id: sympatheticTab
//            }
//        }

//    }

//    footer: TabBar {
//        visible: false
//        id: tabBar
//        currentIndex: swipeView.currentIndex

//        TabButton {
//            text: qsTr("Robot Behaviour")
//        }
//        TabButton {
//            text: qsTr("Positive")
//        }
//        TabButton {
//            text: qsTr("Challenging")
//        }
//        TabButton {
//            text: qsTr("Sympathetic")
//        }

//    }

    RosC25KActionSubscriber{
        id:suggestedActionSubscriber
        topic: "unvalidated_actions"

        onSuggestedAction: {
            console.log("heard a suggested action")
            if (suggestedActionSubscriber.action === "WALK" || suggestedActionSubscriber.action === "RUN") {
                console.log("heard a programme action")
                globalstates.state = "programmeaction"
                programmeTab.suggestedAction = suggestedActionSubscriber;
            } else if (suggestedActionSubscriber.action === "ANIMATION" || suggestedActionSubscriber.action === "BACK_OFF" || suggestedActionSubscriber.action === "GET_CLOSER") {
                globalstates.state = "robotbehaviour"
                robotTab.suggestedAction = suggestedActionSubscriber;
            } else if (suggestedActionSubscriber.action === "CHECKPRE"){
                globalstates.state = "sympathetic"
                sympatheticTab.suggestedAction = suggestedActionSubscriber;
            }
            else {
                if (suggestedActionSubscriber.style === "CHALLENGING") {
                    console.log("heard a challenging action")
                    globalstates.state = "challenging"
                    challengeTab.suggestedAction = suggestedActionSubscriber;
                } else if (suggestedActionSubscriber.style === "SYMPATHETIC") {
                    globalstates.state = "sympathetic"
                    sympatheticTab.suggestedAction = suggestedActionSubscriber;
                } else if (suggestedActionSubscriber.style === "POSITIVE") {
                    globalstates.state = "positive"
                    positiveTab.suggestedAction = suggestedActionSubscriber;
                }
            }
        }

    }

    RosStringSubscriber{
        id:warmupComplete
        topic: "warmup_complete"

        onTextChanged: {
            globalstates.state = "robotbehaviour"
        }
    }

//    RosStringSubscriber{
//        id:userStart
//        topic: "user_start"

//        onTextChanged: {
//            globalstates.state = "robotbehaviour"
//        }
//    }

    RosStringSubscriber{
        id: programmeStateSubscriber
        topic: "programme_state"

        onTextChanged: {
            robotTab.programmeState = programmeStateSubscriber
            challengeTab.programmeState = programmeStateSubscriber
            sympatheticTab.programmeState = programmeStateSubscriber
            positiveTab.programmeState = programmeStateSubscriber
            startTab.programmeState = programmeStateSubscriber
        }
    }

    RosStringSubscriber{
        id: speedSubscriber
        topic: "speed_str"

        onTextChanged: {
            if (globalstates.state == "robotbehaviour") {
                robotTab.speed = speedSubscriber
            } else if (globalstates.state == "challenging") {
                challengeTab.speed = speedSubscriber
            } else if (globalstates.state == "sympathetic") {
                sympatheticTab.speed = speedSubscriber
            } else if (globalstates.state == "positive") {
                positiveTab.speed = speedSubscriber
            } else if (globalstates.state == "programeaction") {
                programmeTab.speed = speedSubscriber
            } else if (globalstates.state == "start") {
                startTab.speed = speedSubscriber
            }

        }
    }

    RosStringSubscriber{
        id: hrSubscriber
        topic: "hr_str"

        onTextChanged: {
            if (globalstates.state == "robotbehaviour") {
                robotTab.hr = hrSubscriber
            } else if (globalstates.state == "challenging") {
                challengeTab.hr = hrSubscriber
            } else if (globalstates.state == "sympathetic") {
                sympatheticTab.hr = hrSubscriber
            } else if (globalstates.state == "positive") {
                positiveTab.hr = hrSubscriber
            } else if (globalstates.state == "programeaction") {
                programmeTab.hr = hrSubscriber
            } else if (globalstates.state == "start") {
                startTab.hr = hrSubscriber
            }

        }
    }

    RosStringSubscriber{
        id: stateDurationSubscriber
        topic: "stateDuration_str"

        onTextChanged: {
            if (globalstates.state == "robotbehaviour") {
                robotTab.stateDuration = stateDurationSubscriber
            } else if (globalstates.state == "challenging") {
                challengeTab.stateDuration = stateDurationSubscriber
            } else if (globalstates.state == "sympathetic") {
                sympatheticTab.stateDuration = stateDurationSubscriber
            } else if (globalstates.state == "positive") {
                positiveTab.stateDuration = stateDurationSubscriber
            } else if (globalstates.state == "programeaction") {
                programmeTab.stateDuration = stateDurationSubscriber
            }

        }
    }

    RosStringSubscriber{
        id: sessionDurationSubscriber
        topic: "sessionDuration_str"

        onTextChanged: {
            if (globalstates.state == "robotbehaviour") {
                robotTab.sessionDuration = sessionDurationSubscriber
            } else if (globalstates.state == "challenging") {
                challengeTab.sessionDuration = sessionDurationSubscriber
            } else if (globalstates.state == "sympathetic") {
                sympatheticTab.sessionDuration = sessionDurationSubscriber
            } else if (globalstates.state == "positive") {
                positiveTab.sessionDuration = sessionDurationSubscriber
            } else if (globalstates.state == "programeaction") {
                programmeTab.sessionDuration = sessionDurationSubscriber
            }

        }
    }

    RosStringSubscriber{
        id:moodSubscriber
        topic: "session_mood"

        onTextChanged: {
            robotTab.mood = moodSubscriber
            challengeTab.mood = moodSubscriber
            sympatheticTab.mood = moodSubscriber
            positiveTab.mood = moodSubscriber
            startTab.mood = moodSubscriber
        }
    }

    RosStringSubscriber{
        id: sessionEndSubscriber
        topic: "session_complete"
        onTextChanged: {
            globalstates.state="start"
            //BasicScreen.finished_signal = sessionEndSubscriber
        }
    }


}
