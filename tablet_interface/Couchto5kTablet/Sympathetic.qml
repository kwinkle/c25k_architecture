import QtQuick 2.0
import Ros 1.0

SympatheticForm {

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
            if (suggestedAction.style === "SYMPATHETIC") {
                if (suggestedAction.action === "CHALLENGE") {
                    challenge.actionHighlight.start()
                    challenge.timeBar.enabled = true
                    challenge.timeBar.visible = true
                    challenge.image_cancel.visible = true
                    challenge.timeBarAnimation.start()
                    challenge.image_cancel.visible = true
                } else if (suggestedAction.action === "TIME") {
                    time.actionHighlight.start()
                    time.timeBar.enabled = true
                    time.timeBar.visible = true
                    time.image_cancel.visible = true
                    time.timeBarAnimation.start()
                    time.image_cancel.visible = true
                } else if (suggestedAction.action === "CHECKPRE") {
                    checkpre.actionHighlight.start()
                    checkpre.timeBar.enabled = true
                    checkpre.timeBar.visible = true
                    checkpre.image_cancel.visible = true
                    checkpre.timeBarAnimation.start()
                    checkpre.image_cancel.visible = true
                } else if (suggestedAction.action === "PRAISE") {
                    praise.actionHighlight.start()
                    praise.timeBar.enabled = true
                    praise.timeBar.visible = true
                    praise.image_cancel.visible = true
                    praise.timeBarAnimation.start()
                    praise.image_cancel.visible = true
                } /*else if (suggestedAction.action === "MOREEYES") {
                    moreeyes.actionHighlight.start()
                    moreeyes.timeBar.enabled = true
                    moreeyes.timeBar.visible = true
                    moreeyes.image_cancel.visible = true
                    moreeyes.timeBarAnimation.start()
                    moreeyes.image_cancel.visible = true
                }*/ else if (suggestedAction.action === "SYMPATHISE") {
                    sympathise.actionHighlight.start()
                    sympathise.timeBar.enabled = true
                    sympathise.timeBar.visible = true
                    sympathise.image_cancel.visible = true
                    sympathise.timeBarAnimation.start()
                    sympathise.image_cancel.visible = true
                }  else if (suggestedAction.action === "SPEEDDOWN") {
                    speeddown.actionHighlight.start()
                    speeddown.timeBar.enabled = true
                    speeddown.timeBar.visible = true
                    speeddown.image_cancel.visible = true
                    speeddown.timeBarAnimation.start()
                    speeddown.image_cancel.visible = true
            }

        }

    }

}
