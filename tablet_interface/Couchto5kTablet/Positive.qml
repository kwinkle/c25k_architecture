import QtQuick 2.0
import Ros 1.0

PositiveForm {

    property var suggestedAction
    property var programmeState
    property var stateDuration
    property var speed
    property var hr
    property var sessionDuration
    property var mood

    onSuggestedActionChanged: {
            if (suggestedAction.style === "POSITIVE") {
                if (suggestedAction.action === "MAINTAIN"){
                    maintain.actionHighlight.start()
                    maintain.timeBar.enabled = true
                    maintain.timeBar.visible = true
                    maintain.image_cancel.visible = true
                    maintain.timeBarAnimation.start()
                    maintain.image_cancel.visible = true
                } else if (suggestedAction.action === "PRAISE") {
                    praise.actionHighlight.start()
                    praise.timeBar.enabled = true
                    praise.timeBar.visible = true
                    praise.image_cancel.visible = true
                    praise.timeBarAnimation.start()
                    praise.image_cancel.visible = true
                } else if (suggestedAction.action === "TIME") {
                    time.actionHighlight.start()
                    time.timeBar.enabled = true
                    time.timeBar.visible = true
                    time.image_cancel.visible = true
                    time.timeBarAnimation.start()
                    time.image_cancel.visible = true
                } else if (suggestedAction.action === "HUMOUR") {
                    humour.actionHighlight.start()
                    humour.timeBar.enabled = true
                    humour.timeBar.visible = true
                    humour.image_cancel.visible = true
                    humour.timeBarAnimation.start()
                    humour.image_cancel.visible = true
                } /*else if (suggestedAction.action === "HISTORY") {
                    userhistory.actionHighlight.start()
                    userhistory.timeBar.enabled = true
                    userhistory.timeBar.visible = true
                    userhistory.image_cancel.visible = true
                    userhistory.timeBarAnimation.start()
                    userhistory.image_cancel.visible = true
                }*/

            }

        }

    }
