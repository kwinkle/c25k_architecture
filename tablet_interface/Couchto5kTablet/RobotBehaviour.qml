import QtQuick 2.0
import Ros 1.0

RobotBehaviourForm {

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
            if (suggestedAction.action === "ANIMATION") {
                funanimation.actionHighlight.start()
                funanimation.timeBar.enabled = true
                funanimation.timeBar.visible = true
                funanimation.image_cancel.visible = true
                funanimation.timeBarAnimation.start()
                funanimation.image_cancel.visible = true
            } else if (suggestedAction.action === "BACK_OFF") {
                backoff.actionHighlight.start()
                backoff.timeBar.enabled = true
                backoff.timeBar.visible = true
                backoff.image_cancel.visible = true
                backoff.timeBarAnimation.start()
                backoff.image_cancel.visible = true
            } else if (suggestedAction.action === "GET_CLOSER") {
                getcloser.actionHighlight.start()
                getcloser.timeBar.enabled = true
                getcloser.timeBar.visible = true
                getcloser.image_cancel.visible = true
                getcloser.timeBarAnimation.start()
                getcloser.image_cancel.visible = true
            } else if (suggestedAction.action === "SMALLTALK") {
                smalltalk.actionHighlight.start()
                smalltalk.timeBar.enabled = true
                smalltalk.timeBar.visible = true
                smalltalk.image_cancel.visible = true
                smalltalk.timeBarAnimation.start()
                smalltalk.image_cancel.visible = true
            } /*else if (suggestedAction === "LESSEYES") {
                lesseyes.actionHighlight.start()
                lesseyes.timeBar.enabled = true
                lesseyes.timeBar.visible = true
                lesseyes.image_cancel.visible = true
                lesseyes.timeBarAnimation.start()
                lesseyes.image_cancel.visible = true
            }*/

        }

    }
