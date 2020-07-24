import QtQuick 2.4
import Ros 1.0

ChallengingForm {

//    RosC25KActionSubscriber{
//        id:suggestedAction
//        topic: "unvalidated_actions"

        property var suggestedAction
        property var programmeState
        property var stateDuration
        property var speed
        property var hr
        property var sessionDuration
        property var mood

        onSuggestedActionChanged: {
            if (suggestedAction.style === "CHALLENGING") {
                if (suggestedAction.action === "TIME") {
                    time.actionHighlight.start()
                    time.timeBar.enabled = true
                    time.timeBar.visible = true
                    time.image_cancel.visible = true
                    time.timeBarAnimation.start()
                    time.image_cancel.visible = true
//                } else if (suggestedAction.action === "HISTORY") {
//                    userhistory.actionHighlight.start()
//                    userhistory.timeBar.enabled = true
//                    userhistory.timeBar.visible = true
//                    userhistory.image_cancel.visible = true
//                    userhistory.timeBarAnimation.start()
//                    userhistory.image_cancel.visible = true
                } else if (suggestedAction.action === "GET_CLOSER") {
                    getcloser.actionHighlight.start()
                    getcloser.timeBar.enabled = true
                    getcloser.timeBar.visible = true
                    getcloser.image_cancel.visible = true
                    getcloser.timeBarAnimation.start()
                    getcloser.image_cancel.visible = true
                } else if (suggestedAction.action === "MOTIVATOR") {
                    usermotivator.actionHighlight.start()
                    usermotivator.timeBar.enabled = true
                    usermotivator.timeBar.visible = true
                    usermotivator.image_cancel.visible = true
                    usermotivator.timeBarAnimation.start()
                    usermotivator.image_cancel.visible = true
//                } else if (suggestedAction.action === "MAINTAIN") {
//                    maintain.actionHighlight.start()
//                    maintain.timeBar.enabled = true
//                    maintain.timeBar.visible = true
//                    maintain.image_cancel.visible = true
//                    maintain.timeBarAnimation.start()
//                    maintain.image_cancel.visible = true
                } else if (suggestedAction.action === "CHALLENGE") {
                    challenge.actionHighlight.start()
                    challenge.timeBar.enabled = true
                    challenge.timeBar.visible = true
                    challenge.image_cancel.visible = true
                    challenge.timeBarAnimation.start()
                    challenge.image_cancel.visible = true
                } else if (suggestedAction.action === "SPEEDUP") { //this button needs to be added (make icon)
                    speedup.actionHighlight.start()
                    speedup.timeBar.enabled = true
                    speedup.timeBar.visible = true
                    speedup.image_cancel.visible = true
                    speedup.timeBarAnimation.start()
                    speedup.image_cancel.visible = true
                }

            }

        }

    }

